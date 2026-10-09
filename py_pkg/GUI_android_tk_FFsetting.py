#!/usr/bin/env python3
"""
FF設定GUI（ADRC＋5次関数FF）: 試行結果の確認と、次に実機へ送るFFパラメータの設定・送信

GUI_android_tk.py（目標値を送るGUI）と同時に使う。このGUIは目標値を送らない。

・FFパラメータ: /board{1..5}_FFparam_float/sub（Float32MultiArray, 36要素 = 6スロット × [a, b, c, d, e, T]）
    f(t) = a t⁵ + b t⁴ + c t³ + d t² + e t（0 ≤ t < T、それ以外は 0）。e は f(T)=0 から決まる（test_code7.py と同じ）。
    FFが掛かるのは24自由度（board4・5 はスロット0〜2 = DOF 18〜20, 22〜24）。DOF 21・25 とスロット3〜5は [0,0,0,0,0,T]。
    実機は目標値メッセージを受信するたびに、FFを t=0 から掛け直す（同じ値の再送でも）。
・FF波形は2つの極値 (t1,y1),(t2,y2) と T から一意に決める。(0,T) 内の極値がちょうど2個にならない設定は送信しない。
・実機のFF係数を読み返すトピックはないので、「GUIが送信した値（受信確認なし）」として表示し、適用済みとは表示しない。
  起動時は FF=0 を前提値（未送信・未確認）とする。FFを送るのは「FFパラメータ送信」「FFを全て0にして送信」を押して
  確認したときだけで、自動では送らない。ボードの配信が途切れて再開したら、実機が起動したとみなして FF=0 として扱う。
・試行: 「● 記録待機」→ 目標値トピックの値が1自由度でも変わった時点を 0 秒として、記録時間のあいだ全自由度の
  実測POT・目標値・z3・PWM を記録する。0〜EVAL_TIME 秒は評価区間で、この間は他の目標値を送らない。
・評価: 目標値が変わった自由度ごとに、減衰係数 ζ=1 の3次遅れ系 1/((T1·s + 1)(s² + 2ζωn·s + ωn²)) を
        実測POTに同定し（test_code7.py と同じ）
        J_i = Σ_k (POT_meas − POT_model)² / ΔPOT_i²   （評価区間を CTRL_DT 刻みに補間した EVAL_M 点）
  を求める。試行の評価値 J は J_i の平均で、J が最小の試行が best。評価できた自由度がない試行と、
  評価区間中に別の目標値を受信した試行は best の候補にしない。
・now（今回）/ last（直前）/ best の試行データは TRIAL_DIR に保存し、再起動後も引き継ぐ。
・各試行の行の「保存」: <保存先>/<now|last|best>_YYYYMMDD_HHMM/（日時は試行の記録開始時刻）
      <フォルダ名>.csv          … 時系列（GUI_android_tk.py の記録CSVと同じ列 ＋ PWMの組に FF{d}・ADRC{d}、
                                  最後に受信した目標値メッセージ Time_cmd, cmd{d}）
      <フォルダ名>_results.csv  … 試行結果（test_code7.py の結果CSVと同じ列 Init_n, Target_n, T_n, t1_n, y1_n, t2_n, y2_n。
                                  n は test_code7.py のFF番号 1〜24）
      DOF/DOF<d>/<now|last|best>_<time-pot|time-FF|time-PWM|time-z3>_YYYYMMDD_HHMM.png
・設定（T・y上下限・プリセット・保存先・記録時間）は SETTINGS_FILE に保存する。FFの極値は起動時に必ず 0（FFなし）から始める。
"""
import csv
import json
import math
import multiprocessing
import os
import queue
import sys
import threading
import time
import traceback
from collections import deque
from datetime import datetime

import numpy as np
import scipy.linalg
import scipy.optimize

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt16MultiArray

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

# 同じフォルダの GUI_android_tk.py（ros2 run ではパッケージ内、直接起動ではスクリプトと同じフォルダ）
if __package__:
    from . import GUI_android_tk as gat
else:
    import GUI_android_tk as gat


# ------------------------------
# 自由度・トピック（GUI_android_tk.py と共通）
# ------------------------------
N_DOF = 26
BOARD_LAYOUT = gat.BOARD_LAYOUT         # (board名, 先頭DOF, DOF数)
TOPIC_TARGET = gat.TOPIC_TARGET         # 目標値（Float32MultiArray, 26要素）。受信だけする
TOPIC_POT = gat.TOPIC_POT               # 実測（UInt16MultiArray）: [0..5] = 実測POT, [6..11] = ボードが使っている目標値
TOPIC_PWM = gat.TOPIC_PWM               # PWM（FF込みの総入力。Float32MultiArray, 6要素）
TOPIC_Z3 = gat.TOPIC_Z3                 # ESO外乱推定値 z3（Float32MultiArray, 6要素）
TOPIC_FF = '/board{}_FFparam_float/sub' # FF係数（Float32MultiArray, 36要素 = 6スロット × [a,b,c,d,e,T]）。{} はボード番号 1〜5
SENSOR_QOS_DEPTH = gat.SENSOR_QOS_DEPTH

FF_SLOTS = 6                            # 1ボードのFFスロット数
# ボードごとの、FFが掛かる自由度の数（スロット0から順に先頭DOFから対応する。test_code7.py の build_dof_map() と同じ）
FF_BOARD_DOFS = {'board1': 6, 'board2': 6, 'board3': 6, 'board4': 3, 'board5': 3}


def build_ff_dof_map():
    """FF番号 n（0〜23）→ DOF番号（26要素の添字）。board1-3 → 0..17 / board4 → 18,19,20 / board5 → 22,23,24"""
    dof_map = []
    for board, offset, _count in BOARD_LAYOUT:
        dof_map.extend(range(offset, offset + FF_BOARD_DOFS[board]))
    return dof_map


FF_DOF_MAP = build_ff_dof_map()                             # FF番号 → DOF
FF_INDEX = {d: n for n, d in enumerate(FF_DOF_MAP)}         # DOF → FF番号（FFのないDOFは含まない）
N_FF = len(FF_DOF_MAP)                                      # 24
DOF_BOARD = {}                                              # DOF → (ボード番号 1〜5, board名, ボード内index, ボードのDOF数)
for _k, (_board, _offset, _count) in enumerate(BOARD_LAYOUT, start=1):
    for _local in range(_count):
        DOF_BOARD[_offset + _local] = (_k, _board, _local, _count)

# ------------------------------
# 評価（test_code7.py と同じ時間軸）
# ------------------------------
CTRL_DT = 0.011             # 実機 loop() の実周期 [s]。評価の時間軸の刻み
EVAL_TIME = 5.0             # 評価区間 [s]（目標値の変化から）。この間は他の目標値を送らない
EVAL_M = int(np.floor(EVAL_TIME / CTRL_DT + 1e-9)) + 1      # 評価点数 #{k : k·CTRL_DT <= EVAL_TIME} = 455
MAX_SAMPLE_GAP = 0.05       # 受信間隔がこれを超えた区間は欠測とみなす [s]
GAP_WARN_RATIO = 0.05       # 評価区間の欠測がこの割合を超えた自由度は警告する

# ------------------------------
# FF波形の設定
# ------------------------------
PWM_LIMIT = 255.0           # 実機PWMの絶対上限
FF_T_DEFAULT = 1.5          # FF終了時刻 T の初期値 [s]
FF_T_RANGE = (0.1, 10.0)    # T の範囲 [s]
Y_LIMIT_DEFAULT = (-50.0, 50.0)     # 極値 y の下限・上限の初期値 [PWM]
DEFAULT_FF_PEAK = 10.0      # 前回の極値が使えないときに提案する極値の大きさ [PWM]
T_STEP = 0.01               # ▲▼ボタンで動かす量（時刻）[s]
Y_STEP = 0.5                # ▲▼ボタンで動かす量（y）[PWM]
EXTREMA_IMAG_TOL = 1e-6     # f'(t)=0 の根を実根とみなす虚部の許容値（τ = t/T の単位）
EXTREMA_LEAD_TOL = 1e-9     # f'(τ) の最高次の係数がこれ未満（最大係数との比）なら次数を下げる
EXTREMA_SIMPLE_TOL = 1e-7   # |f''(τi)| / max(|y1|,|y2|) がこれ以下の極値は重根とみなす
RANGE_SCAN_N = 200          # 可動範囲を調べるときの1座標の走査点数
RANGE_BISECT_ITERS = 22     # 可動範囲の境界を詰める二分法の回数

# ------------------------------
# 記録
# ------------------------------
RECORD_DURATION_DEFAULT = 10.0
RECORD_DURATION_MIN = EVAL_TIME     # 評価区間より短い記録は評価できないので許さない
RECORD_DURATION_MAX = 600.0
BOARD_RESTART_GAP = 3.0     # ボードの配信がこの時間途切れてから再開したら、実機が起動したとみなす [s]

# ------------------------------
# ファイル
# ------------------------------
SETTINGS_FILE = os.path.expanduser('~/.armrobot_gui_ff_setting.json')
TRIAL_DIR = os.path.expanduser('~/.armrobot_gui_ff_trials')
ROW_KEYS = ('now', 'last', 'best')
GRAPH_NAMES = ('time-pot', 'time-FF', 'time-PWM', 'time-z3')
NUM_PRESETS = 4             # プリセット1〜3 = now/last/best の試行で使ったFF、4 = ユーザー設定


# ==============================================================================
# FF波形（5次関数）
# ==============================================================================
def ff_poly(coeffs, t):
    """f(t) = a t⁵ + b t⁴ + c t³ + d t² + e t（区間の切り出しはしない）"""
    a, b, c, d, e = [float(v) for v in coeffs[:5]]
    t = np.asarray(t, dtype=float)
    return ((((a * t + b) * t + c) * t + d) * t + e) * t


def coeff_e(a, b, c, d, T):
    """f(T)=0 から決まる1次の係数 e（test_code7.py の coeff_e と同じ）"""
    return -(a * T ** 4 + b * T ** 3 + c * T ** 2 + d * T)


def _value_rows(tau):
    """g(τ) = g5 τ⁵ + g4 τ⁴ + g3 τ³ + g2 τ² + g1 τ の係数に掛かる行ベクトル"""
    return np.stack([tau ** 5, tau ** 4, tau ** 3, tau ** 2, tau], axis=-1)


def _slope_rows(tau):
    """g'(τ) の係数に掛かる行ベクトル"""
    return np.stack([5 * tau ** 4, 4 * tau ** 3, 3 * tau ** 2, 2 * tau, np.ones_like(tau)], axis=-1)


def quintic_from_extrema(T, t1, y1, t2, y2):
    """極値 (t1,y1),(t2,y2) と T から f の係数 [a,b,c,d,e] を求める（配列で渡せばまとめて解く）

    f(T)=0, f'(t1)=f'(t2)=0, f(t1)=y1, f(t2)=y2 の5式（f(0)=0 は定数項がないので常に成立）で一意に決まる
    （test_code7.py の quintic_from_extrema と同じ）。桁をそろえるため τ = t/T の空間で解いて戻す。
    戻り値: (係数 (n,5), τ空間の係数 (n,5), 解けたか (n,))。0 < t1 < t2 < T でない行は NaN
    """
    T, t1, y1, t2, y2 = np.broadcast_arrays(*[np.atleast_1d(np.asarray(v, dtype=float)) for v in (T, t1, y1, t2, y2)])
    with np.errstate(divide='ignore', invalid='ignore'):
        tau1, tau2 = t1 / T, t2 / T
    ok = (np.isfinite(T) & (T > 0) & np.isfinite(tau1) & np.isfinite(tau2) & np.isfinite(y1) & np.isfinite(y2)
          & (tau1 > 0) & (tau2 > tau1) & (tau2 < 1))
    ta = np.where(ok, tau1, 1.0 / 3.0)
    tb = np.where(ok, tau2, 2.0 / 3.0)
    A = np.stack([_value_rows(np.ones_like(ta)), _slope_rows(ta), _slope_rows(tb), _value_rows(ta), _value_rows(tb)],
                 axis=-2)
    zero = np.zeros_like(ta)
    rhs = np.stack([zero, zero, zero, np.where(ok, y1, 0.0), np.where(ok, y2, 0.0)], axis=-1)
    G = np.linalg.solve(A, rhs[..., None])[..., 0]
    G[~ok] = np.nan
    with np.errstate(invalid='ignore', over='ignore'):
        coeffs = G / (T[:, None] ** np.array([5, 4, 3, 2, 1]))
    return coeffs, G, ok


def slope_roots(G):
    """τ空間の係数 G (n,5) について、g'(τ)（4次以下）の根を (n,4) の複素数で返す（足りない分は NaN）

    最高次の係数が（最大係数との比で）ほぼ 0 の行は次数を下げて解く。極値が対称な形（例: t(t-T/2)(t-T)）では
    t⁵・t⁴ の係数が丸め誤差程度の値になり、そのまま同伴行列を作ると巨大な偽の根が出るため。
    """
    D = np.asarray(G, dtype=float) * np.array([5.0, 4.0, 3.0, 2.0, 1.0])
    n = len(D)
    roots = np.full((n, 4), np.nan + 0j)
    scale = np.abs(D).max(axis=1) if n else np.zeros(0)
    good = np.isfinite(D).all(axis=1) & (scale > 0)
    Dn = np.zeros_like(D)
    Dn[good] = D[good] / scale[good, None]
    big = np.abs(Dn) > EXTREMA_LEAD_TOL
    lead = np.where(big.any(axis=1), big.argmax(axis=1), 5)
    for m in (4, 3, 2, 1):
        rows = np.flatnonzero(good & (lead == 4 - m))
        if rows.size == 0:
            continue
        c = Dn[rows, 4 - m:]
        monic = c[:, 1:] / c[:, :1]
        C = np.zeros((rows.size, m, m))
        C[:, 0, :] = -monic
        for i in range(1, m):
            C[:, i, i - 1] = 1.0
        roots[rows, :m] = np.linalg.eigvals(C)
    return roots


def interior_roots_count(G):
    """g'(τ)=0 の実根のうち 0<τ<1 にあるものの個数（重根は重複して数える）"""
    r = slope_roots(G)
    real = (np.abs(r.imag) <= EXTREMA_IMAG_TOL) & (r.real > 1e-9) & (r.real < 1 - 1e-9)
    return real.sum(axis=1)


def _simple_at(G, tau, amp):
    """τ で |g''(τ)| が十分大きい（重根でない）か"""
    g5, g4, g3, g2 = G[:, 0], G[:, 1], G[:, 2], G[:, 3]
    ddg = 20 * g5 * tau ** 3 + 12 * g4 * tau ** 2 + 6 * g3 * tau + 2 * g2
    return np.abs(ddg) > EXTREMA_SIMPLE_TOL * amp


def valid_batch(T, t1, y1, t2, y2, ylo, yhi):
    """極値条件をすべて満たすか（配列でまとめて判定）

    条件: 0 < t1 < t2 < T、y1·y2 < 0、下限 ≤ y1,y2 ≤ 上限、
          (0,T) 内の f'(t)=0 の実根がちょうど2個（= t1, t2）で、どちらも単純根
    """
    coeffs, G, ok = quintic_from_extrema(T, t1, y1, t2, y2)
    T, t1, y1, t2, y2, ylo, yhi = np.broadcast_arrays(
        *[np.atleast_1d(np.asarray(v, dtype=float)) for v in (T, t1, y1, t2, y2, ylo, yhi)])
    ok = ok & (y1 * y2 < 0) & (y1 >= ylo) & (y1 <= yhi) & (y2 >= ylo) & (y2 <= yhi)
    if ok.any():
        idx = np.flatnonzero(ok)
        Gs = G[idx]
        amp = np.maximum(np.abs(y1[idx]), np.abs(y2[idx]))
        good = interior_roots_count(Gs) == 2
        good &= _simple_at(Gs, t1[idx] / T[idx], amp) & _simple_at(Gs, t2[idx] / T[idx], amp)
        ok[idx] = good
    return ok


class FFCheck:
    """1自由度のFF設定の判定結果"""

    def __init__(self):
        self.ok = False
        self.coeffs = None          # [a,b,c,d,e]（計算できなければ None）
        self.errors = []            # 満たしていない条件（表示用の文）
        self.extrema_t = []         # (0,T) 内にある f'(t)=0 の実根 [s]（条件を満たさないときの表示用）


def check_ff(T, t1, y1, t2, y2, ylo, yhi):
    """FF設定 (T, t1, y1, t2, y2) と y の上下限を判定し、満たしていない条件を文で返す"""
    res = FFCheck()
    vals = (T, t1, y1, t2, y2, ylo, yhi)
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals):
        res.errors.append("数値でない値があります")
        return res
    if not (FF_T_RANGE[0] <= T <= FF_T_RANGE[1]):
        res.errors.append(f"FF終了時刻 T は {FF_T_RANGE[0]:g}〜{FF_T_RANGE[1]:g} s にしてください")
        return res
    if not (ylo < 0 < yhi):
        res.errors.append("y の上下限は 下限 < 0 < 上限 にしてください（極値は山と谷で符号が逆になるため）")
    if not (0 < t1 < t2 < T):
        res.errors.append("0 < t1 < t2 < T にしてください（t1 が第1極値、t2 が第2極値の時刻）")
    if not (y1 * y2 < 0):
        res.errors.append("y1 と y2 は符号を逆にしてください（山→谷 または 谷→山。0 は不可）")
    for name, y in (("y1", y1), ("y2", y2)):
        if y > yhi:
            res.errors.append(f"{name} = {y:g} が上限 {yhi:g} を超えています")
        elif y < ylo:
            res.errors.append(f"{name} = {y:g} が下限 {ylo:g} を下回っています")

    coeffs, G, solved = quintic_from_extrema(T, t1, y1, t2, y2)
    if solved[0] and np.isfinite(coeffs[0]).all():
        res.coeffs = coeffs[0]
        r = slope_roots(G)[0]
        real = np.sort(r.real[(np.abs(r.imag) <= EXTREMA_IMAG_TOL) & (r.real > 1e-9) & (r.real < 1 - 1e-9)])
        res.extrema_t = [float(v) * T for v in real]
        if not res.errors:
            amp = max(abs(y1), abs(y2))
            n_ext = len(real)
            simple = bool(_simple_at(G, np.array([t1 / T]), amp)[0] and _simple_at(G, np.array([t2 / T]), amp)[0])
            if n_ext != 2 or not simple:
                listing = ", ".join(f"{v:.3f}" for v in res.extrema_t)
                res.errors.append(
                    f"(0,T) 内の極値が{'重根を含めて ' if not simple else ''}{n_ext} 個になります（{listing} s）。"
                    "ちょうど2個（t1, t2）である必要があります")
    res.ok = not res.errors
    return res


def feasible_intervals(T, vals, ylo, yhi, coord, n=RANGE_SCAN_N):
    """1つの座標だけを動かしたときに条件を満たす範囲 [(下端, 上端), ...]（他の3座標は固定）

    vals = {'t1','y1','t2','y2'}。座標の取り得る範囲（t は (0,T)、y は [下限, 上限]）を n 点で調べ、
    条件を満たす区間の端を二分法で詰める。区間は複数に分かれることがある。
    """
    if coord in ('t1', 't2'):
        lo, hi = 0.0, float(T)
        grid = np.linspace(lo, hi, n + 2)[1:-1]
    else:
        lo, hi = float(ylo), float(yhi)
        if not hi > lo:
            return []
        grid = np.linspace(lo, hi, n)

    def valid(x):
        p = dict(vals)
        p[coord] = x
        return valid_batch(T, p['t1'], p['y1'], p['t2'], p['y2'], ylo, yhi)

    mask = valid(grid)
    if not mask.any():
        return []
    edges = np.flatnonzero(np.diff(mask.astype(np.int8)))
    starts = [0] if mask[0] else []
    ends = []
    for i in edges:
        if mask[i + 1]:
            starts.append(i + 1)
        else:
            ends.append(i)
    if mask[-1]:
        ends.append(len(grid) - 1)

    # 区間の端を二分法で詰める（全区間の端をまとめて、条件を満たす側 a・満たさない側 b を更新する）
    keys, inner, outer = [], [], []
    for s in starts:
        if s > 0:
            keys.append(('L', s)); inner.append(grid[s]); outer.append(grid[s - 1])
    for e in ends:
        if e < len(grid) - 1:
            keys.append(('R', e)); inner.append(grid[e]); outer.append(grid[e + 1])
    refined = {}
    if keys:
        a, b = np.array(inner), np.array(outer)
        for _ in range(RANGE_BISECT_ITERS):
            mid = 0.5 * (a + b)
            v = valid(mid)
            a = np.where(v, mid, a)
            b = np.where(v, b, mid)
        refined = dict(zip(keys, a))

    edge_open = coord in ('t1', 't2')       # t は開区間 (0,T) なので端点そのものは含まない
    out = []
    for s, e in zip(starts, ends):
        left = refined[('L', s)] if s > 0 else (grid[s] if edge_open else lo)
        right = refined[('R', e)] if e < len(grid) - 1 else (grid[e] if edge_open else hi)
        out.append((float(left), float(right)))
    return out


def _scale_into_limits(y1, y2, ylo, yhi):
    """(y1, y2) を同じ倍率で縮めて上下限に収める（f は (y1,y2) の1次同次なので極値の時刻・個数は変わらない）"""
    s = np.ones_like(y1)
    for y in (y1, y2):
        with np.errstate(divide='ignore', invalid='ignore'):
            s = np.where(y > yhi, np.minimum(s, yhi / y), s)
            s = np.where(y < ylo, np.minimum(s, ylo / y), s)
    return y1 * s, y2 * s


def nearest_feasible(T, t1s, y1s, t2s, y2s, ylo, yhi):
    """目標の極値 (t1s,y1s),(t2s,y2s) に最も近い、条件を満たす極値を探す

    y1s と y2s は符号が逆（0 でない）であること。山・谷の順序（y1 の符号）は保つ。
    距離 D = ((t1−t1s)² + (t2−t2s)²)/T² + ((y1−y1s)² + (y2−y2s)²)/A²（A = max(|y1s|,|y2s|)）
      1. そのままで条件を満たせばそれを返す
      2. 時刻はそのままで、高さの比 r = −y2/y1 だけを変えて探す
      3. それでも無ければ時刻も動かして探す（粗い格子 → 最良点のまわりを細かく）
    戻り値: (t1, y1, t2, y2) または None
    """
    A = max(abs(y1s), abs(y2s))
    if not (ylo < 0 < yhi) or A <= 0 or y1s * y2s >= 0:
        return None
    if valid_batch(T, t1s, y1s, t2s, y2s, ylo, yhi)[0]:
        return (float(t1s), float(y1s), float(t2s), float(y2s))

    tmin = 0.01 * T
    t1c = float(np.clip(t1s, tmin, T - 2 * tmin))
    t2c = float(np.clip(t2s, t1c + tmin, T - tmin))

    def heights(r):
        """比 r（> 0）で (y1s, y2s) に最も近い (y1, y2=−r·y1)"""
        y1 = (y1s - r * y2s) / (1 + r * r)
        return _scale_into_limits(y1, -r * y1, ylo, yhi)

    def best_of(t1, t2, r):
        y1, y2 = heights(r)
        ok = valid_batch(T, t1, y1, t2, y2, ylo, yhi)
        if not ok.any():
            return None
        D = ((t1 - t1s) ** 2 + (t2 - t2s) ** 2) / T ** 2 + ((y1 - y1s) ** 2 + (y2 - y2s) ** 2) / A ** 2
        D = np.where(ok, D, np.inf)
        i = int(np.argmin(D))
        return float(D[i]), float(t1[i]), float(y1[i]), float(t2[i]), float(y2[i]), float(r[i])

    # 2. 時刻を保ったまま比だけ変える
    r = np.logspace(-2.5, 2.5, 1001)
    best = best_of(np.full_like(r, t1c), np.full_like(r, t2c), r)
    if best is not None:
        return best[1:5]

    # 3. 時刻も動かす（粗い格子）
    tg = np.linspace(0.02, 0.98, 37) * T
    rg = np.logspace(-2.5, 2.5, 61)
    T1g, T2g, Rg = np.meshgrid(tg, tg, rg, indexing='ij')
    sel = T2g > T1g + 0.02 * T
    best = best_of(T1g[sel], T2g[sel], Rg[sel])
    if best is None:
        return None
    # 最良点のまわりを細かく
    _, b1, _, b2, _, br = best
    dt = tg[1] - tg[0]
    f1 = np.linspace(b1 - dt, b1 + dt, 11)
    f2 = np.linspace(b2 - dt, b2 + dt, 11)
    fr = br * np.logspace(-0.1, 0.1, 21)
    F1, F2, FR = np.meshgrid(f1, f2, fr, indexing='ij')
    sel = (F1 > 0) & (F2 < T) & (F2 > F1)
    fine = best_of(F1[sel], F2[sel], FR[sel])
    if fine is not None and fine[0] < best[0]:
        best = fine
    return best[1:5]


def extrema_for_csv(setting, T):
    """結果CSV用の極値 (t1, y1, t2, y2)。FFなしは test_code7.py の calc_extrema_from_ff と同じ (0.33T, 0, 0.66T, 0)"""
    if setting is None or not setting.get('enabled'):
        return (T * 0.33, 0.0, T * 0.66, 0.0)
    return (setting['t1'], setting['y1'], setting['t2'], setting['y2'])


def ff_series(ff6, restarts, t):
    """時刻 t（試行開始からの秒）のFF入力。restarts（目標値メッセージを受信した時刻）のたびに t=0 から掛け直す

    ff6 = [a, b, c, d, e, T]。最後に受信してから T 秒以上たっていれば 0。
    """
    t = np.asarray(t, dtype=float)
    out = np.zeros_like(t)
    coeffs = np.asarray(ff6[:5], dtype=float)
    T = float(ff6[5])
    restarts = np.asarray(restarts, dtype=float)
    if restarts.size == 0 or not np.any(coeffs != 0):
        return out
    idx = np.searchsorted(restarts, t, side='right') - 1
    has = idx >= 0
    tau = np.where(has, t - restarts[np.clip(idx, 0, None)], -1.0)
    on = has & (tau >= 0) & (tau < T)
    out[on] = ff_poly(coeffs, tau[on])
    return out


def format_formula(coeffs, T, lang='ja'):
    """f(t) の数式（表示用）。lang='en' はPNG用（matplotlib に日本語フォントがない環境でも文字化けしない）"""
    if coeffs is None:
        return "f(t) = （計算できません）" if lang == 'ja' else "f(t) = (not computable)"
    if not np.any(np.asarray(coeffs) != 0):
        return "f(t) = 0（FFなし）" if lang == 'ja' else "f(t) = 0 (no FF)"
    text = "f(t) ="
    for i, (v, power) in enumerate(zip(coeffs, ("t^5", "t^4", "t^3", "t^2", "t"))):
        sign = "−" if v < 0 else ("" if i == 0 else "+")
        text += f" {sign}{'' if i == 0 else ' '}{abs(v):.5g} {power}" if sign else f" {abs(v):.5g} {power}"
    if lang != 'ja':
        return text.replace("−", "-") + f"    (0 <= t < T={T:g} s, otherwise 0)"
    return text + f"    (0 ≤ t < T={T:g} s、それ以外は 0)"


# ==============================================================================
# 3次遅れ系（減衰係数 ζ=1）の同定と評価（test_code7.py の fit_target_model と同じモデル）
#   G(s) = 1/((T1·s + 1)(s² + 2ζωn·s + ωn²))、ζ = TARGET_ZETA = 1 に固定して T1, ωn を同定する
# ==============================================================================
TARGET_ZETA = 1.0           # 減衰係数 ζ（固定）
MODEL_TEXT = "1/((T1·s+1)(s²+2ζωn·s+ωn²)), ζ=1"


def system_coeffs(T1, zeta, wn):
    """(T1·s + 1)(s² + 2ζωn·s + ωn²) を T1 で割って s³ + a2 s² + a1 s + a0 に展開した係数（test_code7.py と同じ）

        (T1 s + 1)(s² + 2ζωn s + ωn²) = T1 s³ + (2ζωn T1 + 1) s² + (ωn² T1 + 2ζωn) s + ωn²
    """
    a2 = (2 * zeta * wn * T1 + 1) / T1
    a1 = (wn ** 2 * T1 + 2 * zeta * wn) / T1
    a0 = wn ** 2 / T1
    return a2, a1, a0


def target_coeffs(T1, wn):
    """目標モデル（ζ = TARGET_ZETA = 1）の係数（test_code7.py の _target_coeffs と同じ）"""
    return system_coeffs(T1, TARGET_ZETA, wn)


def free_response(T1, wn, dt, M):
    """初期状態 [1, 0, 0] からの自由応答（位置）を時刻 k·dt（k=0..M-1）で返す（ZOH離散化＝厳密なサンプリング）

    x(k) = A_d^k x(0) を、A_d^n を倍々に作りながらまとめて求める（M 点を Python のループで回さない）。
    """
    a2, a1, a0 = target_coeffs(T1, wn)
    Ac = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [-a0, -a1, -a2]])
    P = scipy.linalg.expm(Ac * dt)
    X = np.empty((M, 3))
    X[0] = (1.0, 0.0, 0.0)
    n = 1
    while n < M:
        m = min(n, M - n)
        X[n:n + m] = X[:m] @ P.T
        P = P @ P
        n += m
    return X[:, 0]


def fit_target_model(y_data, y0, dt=CTRL_DT):
    """y = P − Pf の実測 y_data（y(0)=y0）に、目標モデル 1/((T1·s+1)(s²+2ζωn·s+ωn²))（ζ=1）の自由応答を最小二乗で当てる

    test_code7.py の fit_target_model と同じモデル・同じ残差。T1, wn は対数で探し、局所解を避けるため
    いくつかの初期値から始めて残差が最小のものを使う。戻り値: (T1, wn)
    """
    M = len(y_data)
    y_data = np.asarray(y_data, dtype=float)

    def residuals(q):
        return y0 * free_response(math.exp(q[0]), math.exp(q[1]), dt, M) - y_data

    best = None
    for T1_0, wn_0 in ((0.1, 10.0), (0.02, 5.0), (0.3, 20.0), (0.05, 40.0)):
        try:
            res = scipy.optimize.least_squares(
                residuals, x0=[math.log(T1_0), math.log(wn_0)],
                bounds=([math.log(1e-4), math.log(1e-2)], [math.log(100.0), math.log(1e3)]), method='trf')
        except (ValueError, np.linalg.LinAlgError):
            continue
        if np.isfinite(res.cost) and (best is None or res.cost < best.cost):
            best = res
    if best is None:
        raise RuntimeError("3次遅れ系を同定できませんでした")
    return math.exp(best.x[0]), math.exp(best.x[1])


def rise_times(T1, wn):
    """目標モデルのステップ応答（自由応答の 1−y/y0）が 10%・90% に達する時刻 (T10, T90) [s]"""
    span = 30.0 * (T1 + 2.0 / wn)       # 十分整定する長さ
    for _ in range(4):
        N = 20001
        dt = span / (N - 1)
        frac = 1.0 - free_response(T1, wn, dt, N)
        if frac[-1] >= 0.9:
            break
        span *= 4
    t = np.arange(len(frac)) * dt
    out = []
    for level in (0.1, 0.9):
        k = int(np.argmax(frac >= level))
        if frac[k] < level:
            out.append(float('nan'))
        elif k == 0:
            out.append(0.0)
        else:
            out.append(float(t[k - 1] + (level - frac[k - 1]) / (frac[k] - frac[k - 1]) * dt))
    return out[0], out[1]


def resample_by_time(stamps, values, grid):
    """受信時刻で grid 上へ線形補間する（test_code7.py の resample_by_time と同じ）"""
    ts = np.asarray(stamps, dtype=float)
    vs = np.asarray(values, dtype=float)
    ok = np.isfinite(ts) & np.isfinite(vs)
    ts, vs = ts[ok], vs[ok]
    if ts.size == 0:
        return None
    order = np.argsort(ts, kind='stable')
    ts, vs = ts[order], vs[order]
    uniq_ts, uniq_idx = np.unique(ts, return_index=True)
    return np.interp(grid, uniq_ts, vs[uniq_idx])


def gap_mask(stamps, grid, max_gap=MAX_SAMPLE_GAP):
    """grid の各点が「近くに実測サンプルが無い（補間で作った）」か（test_code7.py の gap_mask と同じ）"""
    ts = np.unique(np.asarray(stamps, dtype=float))
    t = np.asarray(grid, dtype=float)
    if ts.size == 0:
        return np.ones(t.size, dtype=bool)
    pos = np.searchsorted(ts, t)
    d_prev = np.where(pos > 0, t - ts[np.clip(pos - 1, 0, ts.size - 1)], np.inf)
    d_next = np.where(pos < ts.size, ts[np.clip(pos, 0, ts.size - 1)] - t, np.inf)
    return np.minimum(d_prev, d_next) > max_gap


def eval_grid():
    return np.arange(EVAL_M) * CTRL_DT


# ==============================================================================
# 試行データ
# ==============================================================================
def zero_ff_table():
    """FFなし（全係数0）の24自由度分の [a,b,c,d,e,T]"""
    table = np.zeros((N_FF, 6))
    table[:, 5] = FF_T_DEFAULT
    return table


class Trial:
    """1回の試行（記録・FF・評価結果）

    meta（JSONにできる値）:
      id, start (ISO形式), stamp (YYYYMMDD_HHMM), duration [s]
      init_target / target … 26自由度の目標値（変化前 / 変化後。分からなければ None）
      ff_table   … この試行でGUIが送信済みだったFF [24][a,b,c,d,e,T]
      ff_setting … そのFFの元にした設定 [24]{enabled,T,t1,y1,t2,y2}（FFなしは None）
      ff_status  … ボードごとのFFの状態（'startup' 起動時の前提値 / 'sent' 送信済み / 'nosub' 受信側なしで送信 /
                   'restart' 実機起動の検出で0とみなした）
      warnings, eval（DOFごとの評価）, J, n_eval, candidate, candidate_note, next_init
    series: board名 → {'pot_t','pot_v','pwm_t','pwm_v','z3_t','z3_v'}（時刻は試行開始からの秒）
    cmd_t, cmd_v … 試行中に受信した目標値メッセージ（FFを掛け直した時刻）
    """

    def __init__(self, meta, series, cmd_t, cmd_v):
        self.meta = meta
        self.series = series
        self.cmd_t = np.asarray(cmd_t, dtype=float)
        self.cmd_v = np.asarray(cmd_v, dtype=float).reshape(len(self.cmd_t), N_DOF) if len(cmd_t) else np.zeros((0, N_DOF))

    @property
    def id(self):
        return self.meta['id']

    @property
    def J(self):
        return self.meta.get('J')

    def ff6(self, dof):
        """この試行で DOF に掛かっていたFF [a,b,c,d,e,T]（FFのないDOFは全0）"""
        n = FF_INDEX.get(dof)
        if n is None:
            return [0.0, 0.0, 0.0, 0.0, 0.0, FF_T_DEFAULT]
        return list(self.meta['ff_table'][n])

    def ff_setting(self, dof):
        n = FF_INDEX.get(dof)
        return None if n is None else self.meta['ff_setting'][n]

    def dof_series(self, dof, kind):
        """DOF の (時刻, 値)。kind = 'pot' / 'desired' / 'pwm' / 'z3'"""
        _, board, local, count = DOF_BOARD[dof]
        s = self.series.get(board)
        if s is None:
            return np.zeros(0), np.zeros(0)
        if kind in ('pot', 'desired'):
            t, v = s['pot_t'], s['pot_v']
            col = local if kind == 'pot' else count + local
        else:
            t, v = s[f'{kind}_t'], s[f'{kind}_v']
            col = local
        if len(t) == 0:
            return np.zeros(0), np.zeros(0)
        return t, v[:, col]

    def ff_at(self, dof, t):
        return ff_series(self.ff6(dof), self.cmd_t, t)

    def model_pot(self, dof, t):
        """同定した3次遅れ系の POT（評価できた自由度だけ。それ以外は None）"""
        ev = self.meta['eval'][dof]
        if ev.get('status') != 'ok':
            return None
        t = np.asarray(t, dtype=float)
        M = int(np.floor(t.max() / CTRL_DT + 1e-9)) + 1 if t.size else 1
        resp = free_response(ev['T1'], ev['wn'], CTRL_DT, max(M, 2))
        tk_ = np.arange(len(resp)) * CTRL_DT
        y0 = self.meta['init_target'][dof] - self.meta['target'][dof]
        return self.meta['target'][dof] + y0 * np.interp(t, tk_, resp)

    # ---- 保存・読み込み（npz。meta は JSON 文字列で入れる） ----
    def save(self, path):
        arrays = {'meta_json': np.array(json.dumps(self.meta)), 'cmd_t': self.cmd_t, 'cmd_v': self.cmd_v}
        for board, s in self.series.items():
            for key, arr in s.items():
                arrays[f'{board}__{key}'] = arr
        tmp = path + '.tmp.npz'
        np.savez_compressed(tmp, **arrays)
        os.replace(tmp, path)

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as z:
            meta = json.loads(str(z['meta_json']))
            series = {}
            for name in z.files:
                if '__' in name:
                    board, key = name.split('__', 1)
                    series.setdefault(board, {})[key] = z[name]
            return cls(meta, series, z['cmd_t'], z['cmd_v'])

    def payload(self):
        """別プロセス（保存処理）へ渡す形"""
        return {'meta': self.meta, 'series': self.series, 'cmd_t': self.cmd_t, 'cmd_v': self.cmd_v}

    @classmethod
    def from_payload(cls, p):
        return cls(p['meta'], p['series'], p['cmd_t'], p['cmd_v'])


def evaluate_trial(trial):
    """試行の評価（GUIとは別スレッドで呼ぶ）。meta に eval・J・n_eval・candidate・warnings を書き込む

    目標値が変わった自由度（ΔPOT ≠ 0）ごとに、評価区間 0〜EVAL_TIME の実測POTを CTRL_DT 刻みに補間し、
    3次遅れ系 1/((T1·s+1)(s²+2ζωn·s+ωn²))（ζ=1）を同定して J_i = Σ(POT_meas − POT_model)² / ΔPOT² を求める。
    J = 平均(J_i)。
    """
    meta = trial.meta
    grid = eval_grid()
    warnings = meta.setdefault('warnings', [])
    ev_all = []
    for dof in range(N_DOF):
        ev = {'status': 'unchanged', 'dpot': None, 'J': None, 'T1': None, 'wn': None,
              'T10': None, 'T90': None, 'gap': None, 'note': ''}
        ev_all.append(ev)
        pi, pf = meta['init_target'][dof], meta['target'][dof]
        if pi is None or pf is None:
            ev['status'] = 'noinit'
            ev['note'] = "変化前の目標値が分からないため評価できません"
            continue
        dpot = pf - pi
        ev['dpot'] = dpot
        if dpot == 0:
            ev['note'] = "目標値が変化していないため評価の対象外"
            continue
        t, v = trial.dof_series(dof, 'pot')
        ok = np.isfinite(v)
        if ok.sum() < 2 or t[ok].max() < EVAL_TIME - 2 * CTRL_DT:
            ev['status'] = 'nodata'
            ev['note'] = "評価区間の実測POTがそろっていないため評価できません"
            warnings.append(f"DOF {dof}: 評価区間の実測POTがそろっていません（評価の対象外）")
            continue
        pot = resample_by_time(t[ok], v[ok], grid)
        gap = float(gap_mask(t[ok], grid).mean())
        ev['gap'] = gap
        if gap > GAP_WARN_RATIO:
            warnings.append(f"DOF {dof}: 評価区間の {gap * 100:.0f}% が欠測（補間）です")
        try:
            T1, wn = fit_target_model(pot - pf, pi - pf)
        except Exception as e:
            ev['status'] = 'error'
            ev['note'] = f"同定できませんでした: {e}"
            warnings.append(f"DOF {dof}: 3次遅れ系を同定できませんでした")
            continue
        model = pf + (pi - pf) * free_response(T1, wn, CTRL_DT, EVAL_M)
        ev.update(status='ok', T1=T1, wn=wn, J=float(np.sum((pot - model) ** 2) / dpot ** 2))
        ev['T10'], ev['T90'] = rise_times(T1, wn)

    meta['eval'] = ev_all
    Js = [ev['J'] for ev in ev_all if ev['status'] == 'ok']
    meta['n_eval'] = len(Js)
    meta['J'] = float(np.mean(Js)) if Js else None

    notes = []
    if not Js:
        notes.append("評価できた自由度がありません")
    early = [float(t) for t in trial.cmd_t if 0 < t <= EVAL_TIME]
    if early:
        notes.append(f"評価区間中に別の目標値を受信しました（{', '.join(f'{t:.2f}' for t in early)} s）")
    if meta['duration'] < EVAL_TIME:
        notes.append("記録時間が評価区間より短い")
    meta['candidate'] = not notes
    meta['candidate_note'] = "、".join(notes)
    return trial


def next_initial(trial, dof, T, ylo, yhi):
    """次の試行の極値の初期値（試行 trial の結果から）

    時刻: 同定した3次遅れ系の 10%・90% 到達時刻 (T10, T90)
    y   : その試行で使ったFFの極値の y
    条件を満たさなければ最も近い実現可能な点へ動かし、その理由を返す。
    戻り値: {'values': (t1,y1,t2,y2) または None, 'target': 目標の値, 'reasons': [...], 'exact': bool}
    """
    ev = trial.meta['eval'][dof]
    setting = trial.ff_setting(dof)
    reasons = []
    if ev.get('status') == 'ok' and ev.get('T10') is not None and np.isfinite(ev['T10']):
        t1s, t2s = ev['T10'], ev['T90']
    elif setting is not None and setting.get('enabled'):
        t1s, t2s = setting['t1'], setting['t2']
        reasons.append(f"前回の試行で同定できなかったため（{ev.get('note') or ev.get('status')}）、"
                       "時刻は前回のFFの極値の時刻を使います")
    else:
        t1s, t2s = T * (3 - math.sqrt(3)) / 6, T * (3 + math.sqrt(3)) / 6
        reasons.append(f"前回の試行で同定できなかったため（{ev.get('note') or ev.get('status')}）、"
                       "時刻は t(t−T/2)(t−T) 形の極値の時刻を使います")
    if setting is not None and setting.get('enabled'):
        y1s, y2s = setting['y1'], setting['y2']
    else:
        dpot = ev.get('dpot') or 0.0
        sign = -1.0 if dpot < 0 else 1.0
        y1s, y2s = sign * DEFAULT_FF_PEAK, -sign * DEFAULT_FF_PEAK
        reasons.append(f"前回の試行はFFなし（極値の y = 0）で、y = 0 では極値が2つになりません。"
                       f"y は ±{DEFAULT_FF_PEAK:g}（{'山→谷' if sign > 0 else '谷→山'}）を提案します")
    target = (float(t1s), float(y1s), float(t2s), float(y2s))
    for name, tv in (("t1(T10)", t1s), ("t2(T90)", t2s)):
        if tv >= T:
            reasons.append(f"{name} = {tv:.3f} s が FF終了時刻 T = {T:g} s 以上です")
    values = nearest_feasible(T, t1s, y1s, t2s, y2s, ylo, yhi)
    exact = values is not None and np.allclose(values, target, rtol=0, atol=1e-12)
    if values is None:
        reasons.append("条件を満たす初期値が見つかりませんでした（y の上下限を確認してください）")
    elif not exact:
        moved = [f"{n} {a:.4g}→{b:.4g}" for n, a, b in zip(("t1", "y1", "t2", "y2"), target, values)
                 if abs(a - b) > 1e-9]
        reasons.append("そのままでは (0,T) 内の極値が2つにならない（または上下限を超える）ため、"
                       "最も近い実現可能な値へ動かしました: " + ", ".join(moved))
    return {'values': values, 'target': target, 'reasons': reasons, 'exact': exact}


class TrialStore:
    """now・last・best の試行を保持し、TRIAL_DIR に保存する（再起動後も引き継ぐ）

    index.json: {"now": id, "last": id, "best": id}。試行ごとに trial_<id>.npz。
    同じ試行が複数の行を占めることがある（例: now が best でもある）。
    """

    def __init__(self, folder=None):
        self.folder = folder or TRIAL_DIR
        self.rows = {k: None for k in ROW_KEYS}
        self.load_error = None
        try:
            os.makedirs(self.folder, exist_ok=True)
            path = os.path.join(self.folder, 'index.json')
            if os.path.exists(path):
                with open(path) as f:
                    index = json.load(f)
                cache = {}
                for key in ROW_KEYS:
                    tid = index.get(key)
                    if not tid:
                        continue
                    if tid not in cache:
                        cache[tid] = Trial.load(self.trial_path(tid))
                    self.rows[key] = cache[tid]
        except Exception as e:     # 読めない試行データは引き継がない（GUIは起動する）
            self.rows = {k: None for k in ROW_KEYS}
            self.load_error = f"{self.folder} の試行データを読み込めませんでした: {e}"

    def trial_path(self, tid):
        return os.path.join(self.folder, f'trial_{tid}.npz')

    def add(self, trial):
        """新しい試行を now にする（now → last）。評価値が best より小さければ best にもする。戻り値: best を更新したか"""
        prev_now = self.rows['now']
        self.rows['last'] = prev_now
        self.rows['now'] = trial
        best = self.rows['best']
        updated = bool(trial.meta.get('candidate')) and (best is None or best.J is None or trial.J < best.J)
        if updated:
            self.rows['best'] = trial
        self.write_index()
        return updated

    def write_index(self):
        os.makedirs(self.folder, exist_ok=True)
        keep = set()
        for t in self.rows.values():
            if t is not None:
                keep.add(t.id)
                if not os.path.exists(self.trial_path(t.id)):
                    t.save(self.trial_path(t.id))
        tmp = os.path.join(self.folder, 'index.json.tmp')
        with open(tmp, 'w') as f:
            json.dump({k: (t.id if t is not None else None) for k, t in self.rows.items()}, f, indent=2)
        os.replace(tmp, os.path.join(self.folder, 'index.json'))
        for name in os.listdir(self.folder):
            if name.startswith('trial_') and name.endswith('.npz') and name[6:-4] not in keep:
                try:
                    os.remove(os.path.join(self.folder, name))
                except OSError:
                    pass


# ==============================================================================
# 設定ファイル
# ==============================================================================
def default_settings():
    return {
        'start_dof': FF_DOF_MAP[0],
        'record_dir': None,
        'record_duration': RECORD_DURATION_DEFAULT,
        'dof': {str(d): {'T': FF_T_DEFAULT, 'ylo': Y_LIMIT_DEFAULT[0], 'yhi': Y_LIMIT_DEFAULT[1]} for d in FF_DOF_MAP},
        'presets': [[None] * N_FF for _ in range(NUM_PRESETS)],
    }


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _is_preset(p):
    return (isinstance(p, dict) and isinstance(p.get('enabled'), bool) and _num(p.get('T'))
            and all(_num(p.get(k)) for k in ('t1', 'y1', 't2', 'y2')))


def load_settings(path=None):
    """設定ファイルを読む。戻り値 (settings, 警告文 or None)。読めない項目は既定値"""
    path = path or SETTINGS_FILE
    settings = default_settings()
    if not os.path.exists(path):
        return settings, None
    try:
        with open(path) as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise ValueError("JSONの最上位がオブジェクトではありません")
    except (OSError, ValueError) as e:
        return settings, f"{path} を読めないため既定値を使います。\n{e}"

    bad = []
    d = data.get('start_dof')
    if isinstance(d, int) and not isinstance(d, bool) and 0 <= d < N_DOF:
        settings['start_dof'] = d
    elif d is not None:
        bad.append('start_dof')
    rd = data.get('record_dir')
    if isinstance(rd, str) and rd:
        settings['record_dir'] = rd
    elif rd is not None:
        bad.append('record_dir')
    dur = data.get('record_duration')
    if _num(dur) and RECORD_DURATION_MIN <= dur <= RECORD_DURATION_MAX:
        settings['record_duration'] = float(dur)
    elif dur is not None:
        bad.append('record_duration')
    dofs = data.get('dof')
    if isinstance(dofs, dict):
        for key, v in dofs.items():
            if key not in settings['dof'] or not isinstance(v, dict):
                continue
            T, ylo, yhi = v.get('T'), v.get('ylo'), v.get('yhi')
            if _num(T) and FF_T_RANGE[0] <= T <= FF_T_RANGE[1]:
                settings['dof'][key]['T'] = float(T)
            if _num(ylo) and _num(yhi) and -PWM_LIMIT <= ylo < 0 < yhi <= PWM_LIMIT:
                settings['dof'][key]['ylo'], settings['dof'][key]['yhi'] = float(ylo), float(yhi)
    elif dofs is not None:
        bad.append('dof')
    presets = data.get('presets')
    if isinstance(presets, list):
        for k in range(min(NUM_PRESETS, len(presets))):
            row = presets[k]
            if not isinstance(row, list):
                continue
            for n in range(min(N_FF, len(row))):
                if _is_preset(row[n]):
                    settings['presets'][k][n] = {key: row[n][key] for key in ('enabled', 'T', 't1', 'y1', 't2', 'y2')}
                    if 'src' in row[n] and isinstance(row[n]['src'], str):
                        settings['presets'][k][n]['src'] = row[n]['src']
    elif presets is not None:
        bad.append('presets')
    if bad:
        return settings, f"{path} の次の項目が不正なため既定値にしました:\n  " + ", ".join(bad)
    return settings, None


def save_settings(settings, path=None):
    path = path or SETTINGS_FILE
    data = dict(settings)
    data['version'] = 1
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    os.replace(tmp, path)


# ==============================================================================
# 保存（CSV・PNG）。別プロセスで動かす
# ==============================================================================
def make_save_folder(base_dir, label, stamp):
    """<保存先>/<label>_YYYYMMDD_HHMM（同じ名前があれば _2, _3 … を付ける）"""
    name = f"{label}_{stamp}"
    path = os.path.join(base_dir, name)
    n = 2
    while os.path.exists(path):
        path = os.path.join(base_dir, f"{name}_{n}")
        n += 1
    os.makedirs(path)
    return path


def results_header():
    """test_code7.py の結果CSV（save_optimal_results_to_csv）と同じ列名"""
    header = []
    for n in range(1, N_FF + 1):
        header += [f'Init_{n}', f'Target_{n}', f'T_{n}', f't1_{n}', f'y1_{n}', f't2_{n}', f'y2_{n}']
    return header


def results_row(trial):
    """結果CSVの1行（FF番号順に 初期目標値・与えた目標値・T・極値）"""
    row = []
    for n, dof in enumerate(FF_DOF_MAP):
        T = float(trial.meta['ff_table'][n][5])
        t1, y1, t2, y2 = extrema_for_csv(trial.meta['ff_setting'][n], T)
        init, target = trial.meta['init_target'][dof], trial.meta['target'][dof]
        row += [float('nan') if init is None else float(init), float('nan') if target is None else float(target),
                T, float(t1), float(y1), float(t2), float(y2)]
    return row


def write_results_csv(trial, path):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(results_header())
        w.writerow(['' if (isinstance(v, float) and math.isnan(v)) else v for v in results_row(trial)])


def timeseries_blocks(trial):
    """時系列CSVの列のまとまり [(列名, 行のリスト)]（GUI_android_tk.py の記録CSVと同じ並び ＋ FF・ADRC・目標値メッセージ）"""
    fi, ff = gat._fmt_int, gat._fmt_float
    blocks = []
    for k, (board, offset, count) in enumerate(BOARD_LAYOUT, start=1):
        s = trial.series.get(board, {})
        dofs = range(offset, offset + count)
        # 実測POT・ボードが使っている目標値
        t, v = s.get('pot_t', np.zeros(0)), s.get('pot_v', np.zeros((0, 2 * count)))
        rows = [[f"{t[i]:.6f}"] + [fi(x) for x in v[i]] for i in range(len(t))]
        blocks.append((gat.record_header(k, 'pot', offset, count), rows))
        # PWM ＋ そのPWMの時刻でのFF入力・ADRC入力（= PWM − FF）
        t, v = s.get('pwm_t', np.zeros(0)), s.get('pwm_v', np.zeros((0, count)))
        ffv = np.column_stack([trial.ff_at(d, t) for d in dofs]) if len(t) else np.zeros((0, count))
        adrc = v - ffv
        header = gat.record_header(k, 'pwm', offset, count) + [f"FF{d}" for d in dofs] + [f"ADRC{d}" for d in dofs]
        rows = [[f"{t[i]:.6f}"] + [ff(x) for x in v[i]] + [ff(x) for x in ffv[i]] + [ff(x) for x in adrc[i]]
                for i in range(len(t))]
        blocks.append((header, rows))
        # z3
        t, v = s.get('z3_t', np.zeros(0)), s.get('z3_v', np.zeros((0, count)))
        rows = [[f"{t[i]:.6f}"] + [ff(x) for x in v[i]] for i in range(len(t))]
        blocks.append((gat.record_header(k, 'z3', offset, count), rows))
    # 試行中に受信した目標値メッセージ（FFを t=0 から掛け直した時刻）
    rows = [[f"{trial.cmd_t[i]:.6f}"] + [ff(x) for x in trial.cmd_v[i]] for i in range(len(trial.cmd_t))]
    blocks.append((["Time_cmd"] + [f"cmd{d}" for d in range(N_DOF)], rows))
    return blocks


def write_timeseries_csv(trial, path):
    blocks = timeseries_blocks(trial)
    max_rows = max((len(rows) for _, rows in blocks), default=0)
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow([name for header, _ in blocks for name in header])
        for i in range(max_rows):
            row = []
            for header, rows in blocks:
                row.extend(rows[i] if i < len(rows) else [""] * len(header))
            w.writerow(row)


GRAPH_TEXT = {   # (GUI, PNG)。PNG は GUI_android_tk.py のグラフと同じく英語にする（matplotlib に日本語フォントがない環境のため）
    'pot': ("実測POT", "POT (measured)"),
    'model': ("3次遅れ系(ζ=1)", "3rd-order model (zeta=1)"),
    'desired': ("目標値", "POT desired"),
    'window': ("評価区間", "eval window"),
    'ff': ("FF入力", "FF input"),
    'ext': ("極値", "extremum "),
    'noslot': ("FFなし（実機にFFスロットがない）", "no FF (no FF slot)"),
    'noff': ("FFなし", "no FF"),
    'pwm': ("PWM（実測）", "PWM (measured)"),
    'adrc': ("ADRC入力 = PWM − FF", "ADRC input = PWM - FF"),
    'unchanged': (None, "not evaluated (target unchanged)"),
    'noinit': (None, "not evaluated (initial target unknown)"),
    'nodata': (None, "not evaluated (POT data missing)"),
    'error': (None, "identification failed"),
}


def graph_content(trial, dof, name, lang='ja'):
    """1枚のグラフの内容（GUIのグラフとPNGで共通）

    戻り値 dict: series [(x, y, 色, 線種, 凡例)], markers [(x, y, 文字, 色)], vlines [(x, 色, 文字)],
                 texts [文字], ylabel, note（右上の補足）
    """
    L = (lambda key: GRAPH_TEXT[key][0]) if lang == 'ja' else (lambda key: GRAPH_TEXT[key][1])
    ev = trial.meta['eval'][dof]
    out = {'series': [], 'markers': [], 'vlines': [], 'texts': [], 'ylabel': '', 'note': ''}
    dur = float(trial.meta['duration'])
    if name == 'time-pot':
        t, v = trial.dof_series(dof, 'pot')
        out['series'].append((t, v, '#d00000', '', L('pot')))
        if ev.get('status') == 'ok':
            tm = np.arange(int(np.floor(dur / CTRL_DT + 1e-9)) + 1) * CTRL_DT
            out['series'].append((tm, trial.model_pot(dof, tm), '#009000', '', L('model')))
            out['note'] = f"J_i={ev['J']:.4g}"
            out['texts'].append(f"{L('model')}: {MODEL_TEXT}   T1={ev['T1']:.4g} s, ωn={ev['wn']:.4g} rad/s"
                                if lang == 'ja' else
                                f"model: {MODEL_TEXT}   T1={ev['T1']:.4g} s, wn={ev['wn']:.4g} rad/s")
        else:
            out['note'] = (ev.get('note') or '') if lang == 'ja' else GRAPH_TEXT.get(ev.get('status'), (None, ''))[1]
        t, v = trial.dof_series(dof, 'desired')
        out['series'].append((t, v, '#0000d0', 'dash', L('desired')))
        out['vlines'].append((EVAL_TIME, '#808080', L('window')))
        out['ylabel'] = 'POT'
    elif name == 'time-FF':
        tf = np.linspace(0.0, dur, int(dur / 0.002) + 1)
        out['series'].append((tf, trial.ff_at(dof, tf), '#c06000', '', L('ff')))
        setting = trial.ff_setting(dof)
        ff6 = trial.ff6(dof)
        if setting is not None and setting.get('enabled'):
            for i, (tx, ty) in enumerate(((setting['t1'], setting['y1']), (setting['t2'], setting['y2'])), start=1):
                out['markers'].append((tx, ty, f"{L('ext')}{i} ({tx:.3f} s, {ty:.4g})", '#c06000'))
            out['texts'].append(format_formula(ff6[:5], ff6[5], lang))
        elif dof not in FF_INDEX:
            out['note'] = L('noslot')
        else:
            out['note'] = L('noff')
        for t0 in trial.cmd_t[1:]:
            out['vlines'].append((float(t0), '#c0c0c0', ''))
        out['ylabel'] = 'FF [PWM]'
    elif name == 'time-PWM':
        t, v = trial.dof_series(dof, 'pwm')
        out['series'].append((t, v, '#8000a0', '', L('pwm')))
        out['series'].append((t, v - trial.ff_at(dof, t), '#e08000', '', L('adrc')))
        out['ylabel'] = 'PWM'
    else:
        t, v = trial.dof_series(dof, 'z3')
        out['series'].append((t, v, '#007070', '', "z3"))
        out['ylabel'] = 'z3'
    return out


def write_graph_png(trial, dof, name, path, label):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg

    c = graph_content(trial, dof, name, lang='en')
    fig = Figure(figsize=(8, 4.5), dpi=100)
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(1, 1, 1)
    plotted = False
    for x, y, color, dash, lab in c['series']:
        x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
        ok = np.isfinite(x) & np.isfinite(y)
        if ok.any():
            ax.plot(x[ok], y[ok], color=color, linestyle='--' if dash else '-', linewidth=1.2, label=lab)
            plotted = True
    for x, y, text, color in c['markers']:
        ax.plot([x], [y], 'o', color=color)
        ax.annotate(text, (x, y), textcoords='offset points', xytext=(6, 6), fontsize=8)
    for x, color, text in c['vlines']:
        ax.axvline(x, color=color, linestyle=':', linewidth=1.0)
    if plotted:
        ax.legend(loc='best', fontsize=8)
        ax.set_xlim(left=0, right=float(trial.meta['duration']))
    else:
        ax.text(0.5, 0.5, "no data", ha='center', va='center', transform=ax.transAxes, color='gray')
    title = f"{label}  DOF {dof}  {name}"
    if c['note']:
        title += f"   ({c['note']})"
    for text in c['texts']:     # FFの数式はタイトルの下の行に書く（グラフ内だと極値の注記と重なるため）
        title += "\n" + text
    ax.set_title(title, fontsize=8 if c['texts'] else 9)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(c['ylabel'])
    ax.grid(True, alpha=0.4)
    fig.tight_layout()
    fig.savefig(path)


def save_trial_outputs(payload, folder, label, progress):
    """保存ボタンの処理本体（GUIとは別プロセスで動かす）"""
    try:
        trial = Trial.from_payload(payload)
        stamp = trial.meta['stamp']
        base = os.path.basename(folder)
        write_timeseries_csv(trial, os.path.join(folder, base + '.csv'))
        write_results_csv(trial, os.path.join(folder, base + '_results.csv'))
        progress.put(('csv', folder))
        total = N_DOF * len(GRAPH_NAMES)
        made = 0
        for dof in range(N_DOF):
            out_dir = os.path.join(folder, 'DOF', f'DOF{dof}')
            os.makedirs(out_dir, exist_ok=True)
            for name in GRAPH_NAMES:
                write_graph_png(trial, dof, name, os.path.join(out_dir, f"{label}_{name}_{stamp}.png"), label)
                made += 1
                progress.put(('graph', made, total))
        progress.put(('done', folder))
    except Exception:
        progress.put(('error', traceback.format_exc()))


# ==============================================================================
# 記録（1回の試行）
# ==============================================================================
class Recorder:
    """待機 → 記録中 → 終了（または取消）。受信コールバック（ROSスレッド）がデータを渡す"""
    ARMED, RECORDING, DONE, CANCELLED = range(4)

    def __init__(self, duration):
        self.duration = float(duration)     # 記録時間 [s]。記録中もGUIから変更される
        self.state = self.ARMED
        self.lock = threading.Lock()
        self.t0 = None                      # 記録開始の時刻（time.monotonic()）
        self.t_end = None                   # 記録を終えた時刻（試行開始からの秒）
        self.start = None                   # 記録開始の日時
        self.init_target = None
        self.target = None
        self.ff = None                      # 記録開始の時点でGUIが送信済みだったFF
        self.data = {board: {'pot': [], 'pwm': [], 'z3': []} for board, _, _ in BOARD_LAYOUT}
        self.cmd = []                       # 試行中に受信した目標値メッセージ (時刻, 値)
        self.rows = 0

    def trigger(self, t, init_target, target, ff):
        with self.lock:
            if self.state != self.ARMED:
                return
            self.t0 = t
            self.start = datetime.now()
            self.init_target = init_target
            self.target = target
            self.ff = ff
            self.cmd.append((0.0, target))
            self.state = self.RECORDING

    def elapsed(self):
        return 0.0 if self.t0 is None else time.monotonic() - self.t0

    def add(self, kind, board, t, values):
        with self.lock:
            if self.state != self.RECORDING:
                return
            tr = t - self.t0
            if tr > self.duration:
                self._finish_locked(self.duration)
                return
            if tr >= 0:
                self.data[board][kind].append((tr, values))
                self.rows += 1

    def add_cmd(self, t, values):
        with self.lock:
            if self.state == self.RECORDING and 0 <= t - self.t0 <= self.duration:
                self.cmd.append((t - self.t0, values))

    def _finish_locked(self, t_end):
        if self.state == self.RECORDING:
            self.t_end = float(min(t_end, self.duration))
            self.state = self.DONE

    def finish(self):
        """記録を終える（待機中なら取り消す）"""
        with self.lock:
            if self.state == self.ARMED:
                self.state = self.CANCELLED
            else:
                self._finish_locked(self.elapsed())

    def cancel(self):
        """記録したデータを捨てる"""
        with self.lock:
            if self.state in (self.ARMED, self.RECORDING):
                self.state = self.CANCELLED

    def to_trial(self):
        series = {}
        for board, _offset, count in BOARD_LAYOUT:
            d = self.data[board]
            out = {}
            for kind, width in (('pot', 2 * count), ('pwm', count), ('z3', count)):
                lst = d[kind]
                out[f'{kind}_t'] = np.array([x[0] for x in lst], dtype=float)
                out[f'{kind}_v'] = (np.array([x[1] for x in lst], dtype=float).reshape(len(lst), width)
                                    if lst else np.zeros((0, width)))
            series[board] = out
        pad = lambda v: [float(v[i]) if i < len(v) else float('nan') for i in range(N_DOF)]
        meta = {
            'id': self.start.strftime('%Y%m%d_%H%M%S'),
            'start': self.start.isoformat(timespec='seconds'),
            'stamp': self.start.strftime('%Y%m%d_%H%M'),
            'duration': float(self.t_end if self.t_end is not None else self.duration),
            'init_target': list(self.init_target),
            'target': list(self.target),
            'ff_table': [list(map(float, row)) for row in self.ff['table']],
            'ff_setting': self.ff['setting'],
            'ff_status': self.ff['status'],
            'warnings': [],
        }
        return Trial(meta, series, [c[0] for c in self.cmd], [pad(c[1]) for c in self.cmd])


# ==============================================================================
# ROS2 ノード
# ==============================================================================
# GUIが最後に送信したFFの状態（ボードごと）。どれも実機の値を確かめたものではない
SENT_STATUS_TEXT = {
    'startup':   "起動時の前提値 FF=0（まだ送信していません・未確認）",
    'sent':      "GUIが送信（受信確認なし）",
    'nosub':     "送信時に受信側が見つかりませんでした（届いていない可能性）",
    'restart':   "実機の起動を検出したため FF=0 とみなしています（未確認）",
    'foreign':   "他のプログラムがFFを送信しました（このGUIの値と違う可能性）",
}


SENT_STATUS_SHORT = {'startup': "未送信(0前提)", 'sent': "送信済", 'nosub': "受信側なし",
                     'restart': "実機起動→0", 'foreign': "他プログラム"}


class FFSettingNode(Node):
    def __init__(self):
        super().__init__('ff_setting_gui_node')
        self.lock = threading.Lock()
        self.events = queue.SimpleQueue()       # GUIへの通知 ('board_first'|'board_restart'|'foreign_ff', board番号, ...)

        self.board_desired = [None] * N_DOF     # ボードが使っている目標値（実測トピックの [6..11]）
        self.last_cmd_values = None             # 目標値トピックで最後に受信した値
        self.last_rx = {}                       # board名 → 実測トピックを最後に受信した時刻
        self.session = None                     # Recorder（GUIスレッドが設定する）

        # GUIが最後に送信したFF（実機の値ではない）
        self.sent_table = zero_ff_table()
        self.sent_setting = [None] * N_FF
        self.sent_status = {k: ['startup', None] for k in range(1, len(BOARD_LAYOUT) + 1)}
        self.own_ff = deque(maxlen=64)          # 自分の送信 (時刻, ボード番号, float32に丸めた値)

        self.ff_pubs = {}
        for k, (board, offset, count) in enumerate(BOARD_LAYOUT, start=1):
            self.ff_pubs[k] = self.create_publisher(Float32MultiArray, TOPIC_FF.format(k), 10)
            self.create_subscription(
                UInt16MultiArray, TOPIC_POT.format(board),
                lambda msg, b=board, o=offset, c=count, k=k: self.pot_cb(msg, k, b, o, c), SENSOR_QOS_DEPTH)
            for kind, topic in (('pwm', TOPIC_PWM), ('z3', TOPIC_Z3)):
                self.create_subscription(
                    Float32MultiArray, topic.format(board),
                    lambda msg, kd=kind, b=board, c=count: self.float_cb(msg, kd, b, c), SENSOR_QOS_DEPTH)
            # 他のプログラムがFFを送信したことに気付くためだけに受信する（値は使わない）
            self.create_subscription(Float32MultiArray, TOPIC_FF.format(k),
                                     lambda msg, k=k: self.ff_cb(msg, k), 10)
        self.create_subscription(Float32MultiArray, TOPIC_TARGET, self.target_cb, 10)

    # ---- 受信 ----
    def pot_cb(self, msg, k, board, offset, count):
        t = time.monotonic()
        data = msg.data
        row = [float('nan')] * (2 * count)
        for i in range(count):
            if i < len(data):
                row[i] = float(data[i])
            if 6 + i < len(data):
                v = float(data[6 + i])
                self.board_desired[offset + i] = v
                row[count + i] = v
        prev = self.last_rx.get(board)
        self.last_rx[board] = t
        if prev is None:
            self.events.put(('board_first', k))
        elif t - prev > BOARD_RESTART_GAP:
            # 配信が途切れて再開した → 実機が起動したとみなす（実機のFFは起動時 0）
            self.reset_board_ff(k, 'restart')
            self.events.put(('board_restart', k, t - prev))
        s = self.session
        if s is not None:
            s.add('pot', board, t, row)

    def float_cb(self, msg, kind, board, count):
        t = time.monotonic()
        s = self.session
        if s is None or s.state != Recorder.RECORDING:
            return
        data = msg.data
        s.add(kind, board, t, [float(data[i]) if i < len(data) else float('nan') for i in range(count)])

    def target_cb(self, msg):
        t = time.monotonic()
        values = tuple(msg.data)
        with self.lock:
            s = self.session
            if s is not None:
                if s.state == Recorder.ARMED and self.target_changed(values):
                    prev = self.last_cmd_values
                    init = []
                    for i in range(N_DOF):
                        if prev is not None and i < len(prev):
                            init.append(float(prev[i]))
                        else:
                            init.append(self.board_desired[i])
                    target = [float(values[i]) if i < len(values) else None for i in range(N_DOF)]
                    s.trigger(t, init, target, self._sent_snapshot_locked())
                elif s.state == Recorder.RECORDING:
                    s.add_cmd(t, values)
            self.last_cmd_values = values

    def target_changed(self, values):
        """受信した目標値が、直前の目標値と1つの自由度でも違うか（GUI_android_tk.py と同じ判定）"""
        prev = self.last_cmd_values
        for i in range(min(N_DOF, len(values))):
            if prev is not None:
                if i >= len(prev) or values[i] != prev[i]:
                    return True
            else:
                bd = self.board_desired[i]
                if bd is None or abs(values[i] - bd) >= 0.5:
                    return True
        return False

    def ff_cb(self, msg, k):
        vals = gat.to_float32_tuple(msg.data)
        now = time.monotonic()
        with self.lock:
            own = any(b == k and v == vals and now - t0 < 10.0 for t0, b, v in self.own_ff)
            if not own:
                self.sent_status[k] = ['foreign', datetime.now().strftime('%H:%M:%S')]
        if not own:
            self.events.put(('foreign_ff', k))

    # ---- FFの送信 ----
    def board_data(self, k, table):
        """ボード k へ送る36要素（6スロット × [a,b,c,d,e,T]）"""
        board, offset, _count = BOARD_LAYOUT[k - 1]
        data = []
        for s in range(FF_SLOTS):
            if s < FF_BOARD_DOFS[board]:
                data.extend(float(x) for x in table[FF_INDEX[offset + s]])
            else:
                data.extend([0.0, 0.0, 0.0, 0.0, 0.0, FF_T_DEFAULT])
        return data

    def publish_ff(self, table, settings, statuses):
        """statuses = {ボード番号: 状態}。そのボードの6スロットをまとめて送る"""
        stamp = datetime.now().strftime('%H:%M:%S')
        with self.lock:
            for k, status in statuses.items():
                msg = Float32MultiArray()
                msg.data = self.board_data(k, table)
                self.own_ff.append((time.monotonic(), k, gat.to_float32_tuple(msg.data)))
                self.ff_pubs[k].publish(msg)
                board, offset, _ = BOARD_LAYOUT[k - 1]
                for s in range(FF_BOARD_DOFS[board]):
                    n = FF_INDEX[offset + s]
                    self.sent_table[n] = table[n]
                    self.sent_setting[n] = None if settings[n] is None else dict(settings[n])
                self.sent_status[k] = [status, stamp]

    def reset_board_ff(self, k, status):
        board, offset, _ = BOARD_LAYOUT[k - 1]
        with self.lock:
            for s in range(FF_BOARD_DOFS[board]):
                n = FF_INDEX[offset + s]
                self.sent_table[n][:5] = 0.0
                self.sent_setting[n] = None
            self.sent_status[k] = [status, datetime.now().strftime('%H:%M:%S')]

    def _sent_snapshot_locked(self):
        return {'table': self.sent_table.copy().tolist(),
                'setting': [None if s is None else dict(s) for s in self.sent_setting],
                'status': {str(k): list(v) for k, v in self.sent_status.items()}}

    def sent_snapshot(self):
        with self.lock:
            return self._sent_snapshot_locked()

    def ff_subscriber_present(self, k):
        """ボード k のFFトピックに、このノード以外の受信側（実機）がいるか"""
        try:
            infos = self.get_subscriptions_info_by_topic(TOPIC_FF.format(k))
        except Exception:
            return False
        me = (self.get_name(), self.get_namespace())
        return any((i.node_name, i.node_namespace) != me for i in infos)


# ==============================================================================
# グラフ（tk.Canvas）
# ==============================================================================
def nice_ticks(lo, hi, max_n):
    span = hi - lo
    if not span > 0:
        return [lo]
    raw = span / max(1, max_n)
    mag = 10 ** math.floor(math.log10(raw))
    step = next(m * mag for m in (1, 2, 5, 10) if m * mag >= raw)
    start = math.ceil(lo / step - 1e-9) * step
    ticks = []
    v = start
    while v <= hi + 1e-9 * step:
        ticks.append(round(v / step) * step)
        v += step
    return ticks


def decimate(x, y, width):
    """1ピクセルに何点もあるときは、ピクセルごとの最小・最大だけ残す（振動の振幅を潰さずに点を減らす）"""
    n = len(x)
    if n <= 2 * width or n < 3:
        return x, y
    span = max(x[-1] - x[0], 1e-12)
    b = np.clip(((x - x[0]) / span * width).astype(int), 0, width - 1)
    starts = np.flatnonzero(np.r_[True, b[1:] != b[:-1]])
    mn = np.minimum.reduceat(y, starts)
    mx = np.maximum.reduceat(y, starts)
    return np.repeat(x[starts], 2), np.column_stack([mn, mx]).ravel()


class PlotCanvas:
    """試行結果の1枚のグラフ。内容（graph_content の戻り値）と横軸の表示範囲が変わったときだけ描き直す"""
    MARGINS = (50, 8, 18, 22)   # 左, 右, 上, 下 [px]

    def __init__(self, parent, width=300, height=180):
        self.canvas = tk.Canvas(parent, width=width, height=height, bg='white',
                                highlightthickness=1, highlightbackground='#c8c8c8')
        self.title = ''
        self.content = None
        self.empty_text = "（試行データなし）"
        self.xr = (0.0, 1.0)

    def set(self, title, content, xr, empty_text=None):
        self.title, self.content, self.xr = title, content, xr
        if empty_text is not None:
            self.empty_text = empty_text
        self.redraw()

    def resize(self, w, h):
        if (int(self.canvas.cget('width')), int(self.canvas.cget('height'))) != (w, h):
            self.canvas.config(width=w, height=h)
            self.redraw()

    def redraw(self):
        c = self.canvas
        c.delete('all')
        w, h = int(c.cget('width')), int(c.cget('height'))
        left, right, top, bottom = self.MARGINS
        texts = self.content['texts'] if self.content is not None else []
        bottom += 22 * len(texts)       # 数式などはx軸の目盛りの下に書く
        pw, ph = max(10, w - left - right), max(10, h - top - bottom)
        font = (UI_FONT, 8)
        c.create_text(4, 2, anchor='nw', text=self.title, font=(UI_FONT, 8, 'bold'))
        if self.content is None:
            c.create_text(w / 2, h / 2, text=self.empty_text, fill='#909090', font=(UI_FONT, 9))
            return
        cont = self.content
        if cont.get('note'):
            c.create_text(w - 4, 2, anchor='ne', text=cont['note'], font=font, fill='#404040')
        x0, x1 = self.xr
        if not x1 > x0:
            x1 = x0 + 1e-3

        # 表示範囲のデータから縦軸を決める
        vis = []
        for x, y, *_ in cont['series']:
            x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
            sel = (x >= x0) & (x <= x1) & np.isfinite(y)
            if sel.any():
                vis.append((y[sel].min(), y[sel].max()))
        for x, y, *_ in cont['markers']:
            if x0 <= x <= x1:
                vis.append((y, y))
        if vis:
            lo, hi = min(v[0] for v in vis), max(v[1] for v in vis)
        else:
            lo, hi = -1.0, 1.0
        if hi - lo < 1e-9:
            lo, hi = lo - 1.0, hi + 1.0
        pad = 0.06 * (hi - lo)
        lo, hi = lo - pad, hi + pad

        def X(v):
            return left + pw * (np.asarray(v, dtype=float) - x0) / (x1 - x0)

        def Y(v):
            return top + ph * (1 - (np.asarray(v, dtype=float) - lo) / (hi - lo))

        # 目盛り
        for v in nice_ticks(lo, hi, max(2, ph // 30)):
            yy = float(Y(v))
            c.create_line(left, yy, left + pw, yy, fill='#e0e0e0')
            c.create_text(left - 4, yy, anchor='e', text=f"{v:.6g}", font=font)
        for v in nice_ticks(x0, x1, max(2, pw // 55)):
            xx = float(X(v))
            c.create_line(xx, top, xx, top + ph, fill='#e0e0e0')
            c.create_text(xx, top + ph + 3, anchor='n', text=f"{v:.6g}", font=font)
        c.create_rectangle(left, top, left + pw, top + ph, outline='#808080')
        if lo < 0 < hi:
            c.create_line(left, float(Y(0)), left + pw, float(Y(0)), fill='#b0b0b0')

        for x, color, text in cont['vlines']:
            if x0 <= x <= x1:
                xx = float(X(x))
                c.create_line(xx, top, xx, top + ph, fill=color, dash=(2, 3))
                if text:
                    c.create_text(xx + 2, top + 2, anchor='nw', text=text, font=font, fill=color)

        # 線（表示範囲だけ切り出して間引く）
        legend = []
        for x, y, color, dash, label in cont['series']:
            x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
            sel = (x >= x0) & (x <= x1) & np.isfinite(y)
            legend.append((color, dash, label))
            if sel.sum() < 2:
                continue
            xs, ys = decimate(x[sel], y[sel], pw)
            pts = np.column_stack((X(xs), Y(ys))).ravel().tolist()
            c.create_line(*pts, fill=color, width=1.5, dash=(4, 2) if dash else None)

        for x, y, text, color in cont['markers']:
            if x0 <= x <= x1:
                xx, yy = float(X(x)), float(Y(y))
                c.create_oval(xx - 3, yy - 3, xx + 3, yy + 3, fill=color, outline='black')
                c.create_text(xx + 5, yy - 2, anchor='sw', text=text, font=font)

        # 凡例（右上。線と重ならないよう白地を敷く）
        ly = top + 4
        for color, dash, label in legend:
            tid = c.create_text(left + pw - 4, ly, anchor='ne', text=label, font=font)
            bx = c.bbox(tid)
            bg = c.create_rectangle(bx[0] - 25, bx[1], bx[2] + 1, bx[3], fill='white', outline='')
            c.tag_lower(bg, tid)
            c.create_line(bx[0] - 22, ly + 6, bx[0] - 4, ly + 6, fill=color, width=2, dash=(4, 2) if dash else None)
            ly += 13
        for i, text in enumerate(cont['texts']):
            c.create_text(4, top + ph + 16 + 22 * i, anchor='nw', text=text, font=(UI_FONT, 7), fill='#303030',
                          width=w - 8)


# ==============================================================================
# 1自由度のFF設定（GUIスレッドだけが触る）
# ==============================================================================
class DofModel:
    def __init__(self, dof, T, ylo, yhi):
        self.dof = dof
        self.enabled = False            # 起動時は必ずFFなし（極値・係数すべて0）
        self.T = float(T)
        self.t1 = self.y1 = self.t2 = self.y2 = 0.0
        self.ylo, self.yhi = float(ylo), float(yhi)
        self.listeners = []
        self._check = None

    def values(self):
        return {'t1': self.t1, 'y1': self.y1, 't2': self.t2, 'y2': self.y2}

    def setting(self):
        return {'enabled': self.enabled, 'T': self.T, 't1': self.t1, 'y1': self.y1, 't2': self.t2, 'y2': self.y2}

    def update(self, **kw):
        for key, v in kw.items():
            setattr(self, key, bool(v) if key == 'enabled' else float(v))
        if not self.enabled:
            self.t1 = self.y1 = self.t2 = self.y2 = 0.0
        self._check = None
        for fn in list(self.listeners):
            fn(self)

    def check(self):
        if self._check is None:
            if self.enabled:
                self._check = check_ff(self.T, self.t1, self.y1, self.t2, self.y2, self.ylo, self.yhi)
            else:
                c = FFCheck()
                c.ok = FF_T_RANGE[0] <= self.T <= FF_T_RANGE[1]
                c.coeffs = np.zeros(5)
                if not c.ok:
                    c.errors.append(f"FF終了時刻 T は {FF_T_RANGE[0]:g}〜{FF_T_RANGE[1]:g} s にしてください")
                self._check = c
        return self._check

    def coeffs6(self):
        """送信する [a,b,c,d,e,T]。条件を満たさなければ None"""
        c = self.check()
        return [float(v) for v in c.coeffs] + [self.T] if c.ok else None


COORD_LABELS = {'t1': "t1 [s]  第1極値の時刻", 'y1': "y1 [PWM] 第1極値", 't2': "t2 [s]  第2極値の時刻",
                'y2': "y2 [PWM] 第2極値"}


class CoordControl:
    """極値の1座標: スライダー・▲▼ボタン・手入力と、条件を満たしたまま動かせる範囲の表示"""

    def __init__(self, block, parent, coord, row):
        self.block = block
        self.coord = coord
        self.is_t = coord in ('t1', 't2')
        self.res = 0.001 if self.is_t else 0.1
        self.step = T_STEP if self.is_t else Y_STEP
        tk.Label(parent, text=COORD_LABELS[coord], font=(UI_FONT, 9)).grid(row=row, column=0, sticky='w', padx=(4, 2))
        self.down = tk.Button(parent, text="▼", width=2, repeatdelay=400, repeatinterval=60,
                              command=lambda: self.nudge(-1))
        self.down.grid(row=row, column=1)
        self.scale = tk.Scale(parent, orient=tk.HORIZONTAL, resolution=self.res, showvalue=False, length=260,
                              command=self.on_scale)
        self.scale.grid(row=row, column=2, sticky='ew')
        # tk.Scale の command は範囲の変更（T や上下限を変えたとき）で値が丸められたときにも呼ばれる。
        # それで極値が勝手に書き換わらないよう、ユーザーがスライダーを操作している間だけ値を採用する
        self.pressed = False
        self.last_user = 0.0
        self.scale.bind("<ButtonPress-1>", lambda e: self.user_touch(True))
        self.scale.bind("<ButtonRelease-1>", lambda e: self.user_touch(False))
        self.scale.bind("<KeyPress>", lambda e: self.user_touch(None))
        self.up = tk.Button(parent, text="▲", width=2, repeatdelay=400, repeatinterval=60,
                            command=lambda: self.nudge(+1))
        self.up.grid(row=row, column=3)
        self.entry = tk.Entry(parent, width=9, justify='right')
        self.entry.grid(row=row, column=4, padx=2)
        self.entry.bind("<Return>", self.on_entry)
        self.entry.bind("<FocusOut>", self.on_entry)
        self.range_label = tk.Label(parent, text="", font=(UI_FONT, 8), anchor='w', justify='left', width=40,
                                    wraplength=280)
        self.range_label.grid(row=row, column=5, sticky='w', padx=4)
        self.bar = tk.Canvas(parent, height=7, highlightthickness=0)
        self.bar.grid(row=row + 1, column=2, sticky='ew')
        self.written = None
        self.intervals = []
        self.bar.bind("<Configure>", lambda e: self.draw_bar())

    def model(self):
        return self.block.model

    def set_limits(self):
        m = self.model()
        lo, hi = (0.0, m.T) if self.is_t else (m.ylo, m.yhi)
        if (float(self.scale.cget('from')), float(self.scale.cget('to'))) != (lo, hi):
            self.scale.config(from_=lo, to=hi)

    def show_value(self):
        m = self.model()
        v = getattr(m, self.coord)
        state = tk.NORMAL if m.enabled else tk.DISABLED
        for w in (self.down, self.up, self.scale, self.entry):
            w.config(state=state)
        if m.enabled and abs(float(self.scale.get()) - v) > self.res / 2:
            self.scale.set(v)
        text = f"{v:.4f}" if self.is_t else f"{v:.3f}"
        if self.block.app.focused() is not self.entry or not m.enabled:
            self.entry.config(state=tk.NORMAL)
            self.entry.delete(0, tk.END)
            self.entry.insert(0, text if m.enabled else "0")
            self.entry.config(bg='white', state=state)
            self.written = text

    def user_touch(self, pressed):
        if pressed is not None:
            self.pressed = pressed
        self.last_user = time.monotonic()

    def on_scale(self, value):
        m = self.model()
        if not m.enabled or not (self.pressed or time.monotonic() - self.last_user < 0.3):
            return
        v = float(value)
        if abs(v - getattr(m, self.coord)) <= self.res / 2:
            return      # プログラムから set() したときの呼び出し
        m.update(**{self.coord: v})

    def nudge(self, sign):
        m = self.model()
        if m.enabled:
            m.update(**{self.coord: round(getattr(m, self.coord) + sign * self.step, 6)})

    def on_entry(self, _event=None):
        m = self.model()
        if not m.enabled:
            return
        text = self.entry.get().strip()
        if text == self.written:
            return
        try:
            v = float(text)
            if not math.isfinite(v):
                raise ValueError
        except ValueError:
            self.entry.config(bg='#ffc0c0')
            return
        self.entry.config(bg='white')
        m.update(**{self.coord: v})

    def show_range(self, intervals, valid_now):
        """条件を満たしたまま動かせる範囲（valid_now）、またはエラーを解消するための移動方向と範囲を表示する"""
        self.intervals = intervals
        m = self.model()
        unit = "s" if self.is_t else ""
        fmt = (lambda x: f"{x:.3f}") if self.is_t else (lambda x: f"{x:.2f}")
        v = getattr(m, self.coord)
        if not m.enabled:
            text, color = "", 'black'
        elif valid_now:
            here = [iv for iv in intervals if iv[0] - 1e-9 <= v <= iv[1] + 1e-9]
            iv = here[0] if here else None
            text = f"可動範囲 {fmt(iv[0])}〜{fmt(iv[1])}{unit}" if iv else "可動範囲: 現在値のみ"
            others = [x for x in intervals if x is not iv]
            if others:
                text += "（ほか " + ", ".join(f"{fmt(a)}〜{fmt(b)}" for a, b in others) + "）"
            color = '#006000'
        elif not intervals:
            text, color = "この座標だけでは解消できません（他の座標も動かしてください）", '#a00000'
        else:
            dist = [(min(abs(v - a), abs(v - b)), a, b) for a, b in intervals]
            _, a, b = min(dist)
            if v < a:
                arrow = "→ 右へ" if self.is_t else "▲ 上げる"
                text = f"{arrow} +{fmt(a - v)}{unit} 以上（{fmt(a)}〜{fmt(b)}{unit} で解消）"
            else:
                arrow = "← 左へ" if self.is_t else "▼ 下げる"
                text = f"{arrow} −{fmt(v - b)}{unit} 以上（{fmt(a)}〜{fmt(b)}{unit} で解消）"
            color = '#a05000'
        self.range_label.config(text=text, fg=color)
        self.draw_bar()

    def draw_bar(self):
        c = self.bar
        c.delete('all')
        m = self.model()
        if not m.enabled:
            return
        w = c.winfo_width()
        if w < 10:
            return

        def px(v):
            try:
                return float(self.scale.coords(v)[0])
            except tk.TclError:
                return 0.0
        lo, hi = float(self.scale.cget('from')), float(self.scale.cget('to'))
        c.create_rectangle(px(lo), 1, px(hi), 6, fill='#f0c0c0', outline='')
        for a, b in self.intervals:
            c.create_rectangle(px(max(a, lo)), 1, max(px(min(b, hi)), px(max(a, lo)) + 2), 6,
                               fill='#40b040', outline='')
        x = px(min(max(getattr(m, self.coord), lo), hi))
        c.create_line(x, 0, x, 7, fill='black', width=2)


# ==============================================================================
# 1自由度ぶんの表示（FF設定 ＋ now/last/best の4グラフ）
# ==============================================================================
CONSTRAINT_HELP = (
    "条件: 0 < t1 < t2 < T、y1·y2 < 0（山→谷 か 谷→山）、下限 ≤ y1, y2 ≤ 上限、"
    "(0,T) 内の f′(t)=0 の実根がちょうど2個（t1, t2）で重根でない。\n"
    "判定: f(T)=0, f′(t1)=f′(t2)=0, f(t1)=y1, f(t2)=y2 から係数を一意に求め、f′(t)（4次式）の根を同伴行列の固有値で"
    "求めて (0,T) 内の実根（虚部 ≤ 1e-6）を数える。条件を満たさない設定は送信しない。")


class DofBlock:
    PRESET_NAMES = ("now", "last", "best", "ユーザー")

    def __init__(self, app, parent, dof):
        self.app = app
        self.dof = dof
        self.model = app.models.get(dof)
        k, board, local, _ = DOF_BOARD[dof]
        if self.model is not None:
            title = f"DOF {dof}   （{board} スロット{local}、test_code7.py のFF番号 {FF_INDEX[dof] + 1}）"
        else:
            title = f"DOF {dof}   （{board} の index{local}。FFスロットなし）"
        self.frame = tk.LabelFrame(parent, text=title, font=(UI_FONT, 10, 'bold'), padx=4, pady=2)
        self.refresh_job = None
        self.range_jobs = {}
        self.width = 1200
        if self.model is not None:
            self.build_ff_panel()
            self.model.listeners.append(self.on_model_change)
        else:
            tk.Label(self.frame, text="この自由度にはFFがありません（board4・5 のスロット3以降はFFなし）。"
                                      "試行結果のグラフだけ表示します。", fg='#606060').pack(anchor='w')
        self.build_results()

    def destroy(self):
        if self.model is not None and self.on_model_change in self.model.listeners:
            self.model.listeners.remove(self.on_model_change)
        for job in [self.refresh_job] + list(self.range_jobs.values()):
            if job is not None:
                self.app.root.after_cancel(job)
        self.frame.destroy()

    # ---- FF設定 ----
    def build_ff_panel(self):
        top = tk.Frame(self.frame)
        top.pack(fill=tk.X, pady=(0, 2))
        self.enabled_var = tk.BooleanVar(value=False)
        tk.Checkbutton(top, text="FFを与える", variable=self.enabled_var, font=(UI_FONT, 10, 'bold'),
                       command=self.on_enable).pack(side=tk.LEFT)
        tk.Label(top, text="   FF終了時刻 T [s]").pack(side=tk.LEFT)
        tk.Button(top, text="▼", width=2, repeatdelay=400, repeatinterval=60,
                  command=lambda: self.nudge_T(-1)).pack(side=tk.LEFT)
        self.T_entry = tk.Entry(top, width=6, justify='right')
        self.T_entry.pack(side=tk.LEFT)
        tk.Button(top, text="▲", width=2, repeatdelay=400, repeatinterval=60,
                  command=lambda: self.nudge_T(+1)).pack(side=tk.LEFT)
        tk.Label(top, text="   y 下限").pack(side=tk.LEFT)
        self.ylo_entry = tk.Entry(top, width=6, justify='right')
        self.ylo_entry.pack(side=tk.LEFT)
        tk.Label(top, text="上限").pack(side=tk.LEFT)
        self.yhi_entry = tk.Entry(top, width=6, justify='right')
        self.yhi_entry.pack(side=tk.LEFT)
        for e in (self.T_entry, self.ylo_entry, self.yhi_entry):
            e.bind("<Return>", self.on_param_entry)
            e.bind("<FocusOut>", self.on_param_entry)
        tk.Label(top, text="   プリセット:").pack(side=tk.LEFT)
        for i, name in enumerate(self.PRESET_NAMES):
            tk.Button(top, text=f"P{i + 1} {name}", command=lambda i=i: self.app.apply_preset(self.dof, i)
                      ).pack(side=tk.LEFT, padx=1)
        tk.Button(top, text="次試行の初期値を適用", command=lambda: self.app.apply_next_init([self.dof])
                  ).pack(side=tk.LEFT, padx=(8, 0))

        mid = tk.Frame(self.frame)
        mid.pack(fill=tk.X)
        left = tk.Frame(mid)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.wave = tk.Canvas(left, height=190, bg='white', highlightthickness=1, highlightbackground='#c8c8c8')
        self.wave.pack(fill=tk.X, expand=True)
        self.wave.bind("<Configure>", lambda e: self.draw_wave())
        self.formula = tk.Label(left, text="", anchor='w', justify='left', font=(UI_FONT, 9))
        self.formula.pack(fill=tk.X)
        self.status = tk.Label(left, text="", anchor='w', justify='left', font=(UI_FONT, 9))
        self.status.pack(fill=tk.X)
        right = tk.Frame(mid)
        right.pack(side=tk.LEFT, fill=tk.Y, padx=(6, 0))
        right.columnconfigure(2, weight=1)
        self.coords = {}
        for i, coord in enumerate(('t1', 'y1', 't2', 'y2')):
            self.coords[coord] = CoordControl(self, right, coord, row=2 * i)
        self.help = tk.Label(self.frame, text=CONSTRAINT_HELP, anchor='w', justify='left', font=(UI_FONT, 8),
                             fg='#606060')
        self.help.pack(fill=tk.X)
        self.proposal = tk.Label(self.frame, text="", anchor='w', justify='left', font=(UI_FONT, 9), fg='#304080')
        self.proposal.pack(fill=tk.X)
        self.sent_label = tk.Label(self.frame, text="", anchor='w', justify='left', font=(UI_FONT, 9))
        self.sent_label.pack(fill=tk.X)
        self.refresh_now()

    def on_enable(self):
        if self.enabled_var.get():
            self.app.enable_ff(self.dof)
        else:
            self.model.update(enabled=False)

    def nudge_T(self, sign):
        m = self.model
        self.app.set_dof_params(self.dof, T=min(max(round(m.T + sign * T_STEP, 6), FF_T_RANGE[0]), FF_T_RANGE[1]))

    def on_param_entry(self, _event=None):
        m = self.model
        try:
            T = float(self.T_entry.get())
            ylo = float(self.ylo_entry.get())
            yhi = float(self.yhi_entry.get())
        except ValueError:
            self.status.config(text="T・y の上下限には数値を入力してください", fg='#a00000')
            return
        bad = []
        if not FF_T_RANGE[0] <= T <= FF_T_RANGE[1]:
            bad.append(f"T は {FF_T_RANGE[0]:g}〜{FF_T_RANGE[1]:g} s")
        if not (-PWM_LIMIT <= ylo < 0 < yhi <= PWM_LIMIT):
            bad.append(f"y の上下限は −{PWM_LIMIT:g} ≤ 下限 < 0 < 上限 ≤ {PWM_LIMIT:g}")
        for e, ok in ((self.T_entry, FF_T_RANGE[0] <= T <= FF_T_RANGE[1]),
                      (self.ylo_entry, -PWM_LIMIT <= ylo < 0), (self.yhi_entry, 0 < yhi <= PWM_LIMIT)):
            e.config(bg='white' if ok else '#ffc0c0')
        if bad:
            self.status.config(text="入力範囲: " + "、".join(bad), fg='#a00000')
            return
        if (T, ylo, yhi) != (m.T, m.ylo, m.yhi):
            self.app.set_dof_params(self.dof, T=T, ylo=ylo, yhi=yhi)

    def on_model_change(self, _model):
        if self.refresh_job is None:
            self.refresh_job = self.app.root.after(30, self.refresh_now)

    def refresh_now(self):
        self.refresh_job = None
        m = self.model
        self.enabled_var.set(m.enabled)
        focus = self.app.focused()
        for e, v in ((self.T_entry, m.T), (self.ylo_entry, m.ylo), (self.yhi_entry, m.yhi)):
            if focus is not e:
                e.delete(0, tk.END)
                e.insert(0, f"{v:g}")
                e.config(bg='white')
        chk = m.check()
        for c in self.coords.values():
            c.set_limits()
            c.show_value()
        if m.enabled:
            self.formula.config(text=format_formula(chk.coeffs, m.T))
            if chk.ok:
                self.status.config(text="○ 条件を満たしています（送信できます）", fg='#006000')
            else:
                self.status.config(text="× エラー: " + " / ".join(chk.errors), fg='#c00000')
            vals = m.values()
            for coord, ctl in self.coords.items():
                ctl.show_range(feasible_intervals(m.T, vals, m.ylo, m.yhi, coord), chk.ok)
        else:
            self.formula.config(text=format_formula(np.zeros(5), m.T))
            self.status.config(text="FFなし（極値・係数はすべて 0）。送信すると [0,0,0,0,0,T] を送ります",
                               fg='#404040' if chk.ok else '#c00000')
            for ctl in self.coords.values():
                ctl.show_range([], True)
        self.refresh_sent()
        self.draw_wave()

    def refresh_sent(self):
        if self.model is None:
            return
        snap = self.app.sent_snap
        n = FF_INDEX[self.dof]
        k = DOF_BOARD[self.dof][0]
        status, stamp = snap['status'][str(k)]
        s = snap['setting'][n]
        if s is None:
            text = "FFなし（極値 0・全係数 0）"
        else:
            text = f"t1={s['t1']:.4f} s, y1={s['y1']:.3f}, t2={s['t2']:.4f} s, y2={s['y2']:.3f}, T={s['T']:g} s"
        when = f" {stamp}" if stamp else ""
        unsent = self.app.is_unsent(self.dof)
        self.sent_label.config(
            text=f"GUIが最後に送信した値: {text}   [{SENT_STATUS_TEXT[status]}{when}]"
                 + ("   ※上の設定はまだ送信していません（次の試行には送信済みの値が使われます）" if unsent else "") + "\n"
                 "実機に適用中の値: 取得できません（実機からFF係数を読み返すトピックがないため、上の値が適用済みとは限りません）",
            fg='#a05000' if status in ('nosub', 'foreign') or unsent else '#303030')
        self.draw_wave()

    def show_proposal(self, text):
        if self.model is not None:
            self.proposal.config(text=text)

    def draw_wave(self):
        if self.model is None:
            return
        c = self.wave
        c.delete('all')
        w, h = c.winfo_width(), int(c.cget('height'))
        if w < 50:
            return
        m = self.model
        chk = m.check()
        left, right, top, bottom = 52, 10, 8, 20
        pw, ph = w - left - right, h - top - bottom
        T = m.T
        x1 = T * 1.08
        tt = np.linspace(0, x1, max(200, pw))
        curves = []
        if m.enabled and chk.coeffs is not None:
            f = np.where(tt < T, ff_poly(chk.coeffs, tt), 0.0)
            curves.append((f, '#1050d0' if chk.ok else '#d02020', None, "設定中のFF"))
        snap = self.app.sent_snap
        n = FF_INDEX[self.dof]
        sent = snap['table'][n]
        if np.any(np.asarray(sent[:5]) != 0):
            fs = np.where(tt < sent[5], ff_poly(sent[:5], tt), 0.0)
            curves.append((fs, '#909090', (4, 3), "GUIが最後に送信したFF"))
        lo, hi = m.ylo, m.yhi
        for f, *_ in curves:
            fin = f[np.isfinite(f)]
            if fin.size:
                lo = max(min(lo, fin.min()), 3 * m.ylo)
                hi = min(max(hi, fin.max()), 3 * m.yhi)
        pad = 0.08 * (hi - lo)
        lo, hi = lo - pad, hi + pad

        def X(v):
            return left + pw * np.asarray(v, dtype=float) / x1

        def Y(v):
            return top + ph * (1 - (np.clip(np.asarray(v, dtype=float), lo, hi) - lo) / (hi - lo))

        font = (UI_FONT, 8)
        for v in nice_ticks(lo, hi, max(2, ph // 30)):
            c.create_line(left, float(Y(v)), left + pw, float(Y(v)), fill='#e8e8e8')
            c.create_text(left - 4, float(Y(v)), anchor='e', text=f"{v:.6g}", font=font)
        for v in nice_ticks(0, x1, max(2, pw // 60)):
            c.create_line(float(X(v)), top, float(X(v)), top + ph, fill='#e8e8e8')
            c.create_text(float(X(v)), top + ph + 3, anchor='n', text=f"{v:.6g}", font=font)
        c.create_rectangle(left, top, left + pw, top + ph, outline='#808080')
        c.create_line(left, float(Y(0)), left + pw, float(Y(0)), fill='#a0a0a0')
        c.create_line(float(X(T)), top, float(X(T)), top + ph, fill='#808080', dash=(2, 3))
        c.create_text(float(X(T)) + 2, top + ph - 2, anchor='sw', text=f"T={T:g}s", font=font, fill='#606060')
        # y の上下限（水平の点線）
        for v, name in ((m.yhi, "上限"), (m.ylo, "下限")):
            c.create_line(left, float(Y(v)), left + pw, float(Y(v)), fill='#d07000', dash=(3, 3))
            c.create_text(left + 4, float(Y(v)) + (-2 if v > 0 else 2), anchor='sw' if v > 0 else 'nw',
                          text=f"{name} {v:g}", font=font, fill='#d07000')
        ly = top + 4
        for f, color, dash, label in curves:
            pts = np.column_stack((X(tt), Y(f))).ravel().tolist()
            c.create_line(*pts, fill=color, width=2, dash=dash)
            c.create_text(left + pw - 4, ly, anchor='ne', text=label, font=font, fill=color)
            ly += 13
        if m.enabled:
            for i, (tx, ty) in enumerate(((m.t1, m.y1), (m.t2, m.y2)), start=1):
                xx, yy = float(X(tx)), float(Y(ty))
                c.create_oval(xx - 4, yy - 4, xx + 4, yy + 4, fill='#ffd040', outline='black')
                c.create_text(xx + 6, yy, anchor='w', text=f"極値{i} ({tx:.3f}, {ty:.2f})", font=font)
            # 余分な極値（条件を満たさないとき）
            for tx in chk.extrema_t:
                if min(abs(tx - m.t1), abs(tx - m.t2)) > 1e-4 * T and chk.coeffs is not None:
                    xx, yy = float(X(tx)), float(Y(ff_poly(chk.coeffs, tx)))
                    c.create_line(xx - 5, yy - 5, xx + 5, yy + 5, fill='#d02020', width=2)
                    c.create_line(xx - 5, yy + 5, xx + 5, yy - 5, fill='#d02020', width=2)

    # ---- 試行結果 ----
    def build_results(self):
        res = tk.Frame(self.frame)
        res.pack(fill=tk.X, pady=(4, 0))
        self.res = res
        self.plots = {}
        self.range_scales = {}
        tk.Label(res, text="表示期間 [s]", font=(UI_FONT, 8)).grid(row=0, column=0, sticky='e')
        for j, name in enumerate(GRAPH_NAMES, start=1):
            box = tk.Frame(res)
            box.grid(row=0, column=j, sticky='ew', padx=1)
            tk.Label(box, text=name, font=(UI_FONT, 9, 'bold')).grid(row=0, column=0, columnspan=4)
            tk.Label(box, text="開始", font=(UI_FONT, 8)).grid(row=1, column=0)
            tk.Label(box, text="終了", font=(UI_FONT, 8)).grid(row=1, column=2)
            s0 = tk.Scale(box, orient=tk.HORIZONTAL, resolution=0.1, from_=0, to=10, length=100, width=10,
                          font=(UI_FONT, 7), command=lambda v, n=name: self.on_range(n, 0))
            s1 = tk.Scale(box, orient=tk.HORIZONTAL, resolution=0.1, from_=0, to=10, length=100, width=10,
                          font=(UI_FONT, 7), command=lambda v, n=name: self.on_range(n, 1))
            s0.grid(row=1, column=1, sticky='ew')
            s1.grid(row=1, column=3, sticky='ew')
            box.columnconfigure(1, weight=1)
            box.columnconfigure(3, weight=1)
            s1.set(10)
            self.range_scales[name] = (s0, s1)
        self.max_t = 10.0
        self.row_labels = {}
        for i, key in enumerate(ROW_KEYS, start=1):
            lab = tk.Label(res, text=key, font=(UI_FONT, 8), justify='left', anchor='nw', width=16, wraplength=120)
            lab.grid(row=i, column=0, sticky='nw')
            self.row_labels[key] = lab
            for j, name in enumerate(GRAPH_NAMES, start=1):
                p = PlotCanvas(res)
                p.canvas.grid(row=i, column=j, padx=1, pady=1)
                self.plots[(key, name)] = p
            tk.Button(res, text=f"保存\n({key})", command=lambda k=key: self.app.save_row(k)
                      ).grid(row=i, column=5, padx=(4, 0), sticky='ns')

    def on_range(self, name, which):
        s0, s1 = self.range_scales[name]
        a, b = float(s0.get()), float(s1.get())
        if b <= a:      # 開始 < 終了 を保つ
            if which == 0:
                s1.set(min(self.max_t, a + 0.1))
            else:
                s0.set(max(0.0, b - 0.1))
            return
        job = self.range_jobs.get(name)
        if job is not None:
            self.app.root.after_cancel(job)
        self.range_jobs[name] = self.app.root.after(40, lambda: self.redraw_column(name))

    def xrange(self, name):
        s0, s1 = self.range_scales[name]
        a, b = float(s0.get()), float(s1.get())
        return (a, b) if b > a else (0.0, self.max_t)

    def redraw_column(self, name):
        self.range_jobs[name] = None
        for key in ROW_KEYS:
            p = self.plots[(key, name)]
            p.xr = self.xrange(name)
            p.redraw()

    def refresh_results(self):
        rows = self.app.store.rows
        durs = [t.meta['duration'] for t in rows.values() if t is not None]
        new_max = max(durs) if durs else 10.0
        if abs(new_max - self.max_t) > 1e-9:
            for s0, s1 in self.range_scales.values():
                at_end = float(s1.get()) >= self.max_t - 1e-6
                s0.config(to=new_max)
                s1.config(to=new_max)
                if at_end:
                    s1.set(new_max)
            self.max_t = new_max
        for key in ROW_KEYS:
            trial = rows[key]
            self.row_labels[key].config(text=self.app.row_text(key, trial, self.dof))
            for name in GRAPH_NAMES:
                p = self.plots[(key, name)]
                if trial is None:
                    p.set(f"{key}", None, self.xrange(name))
                else:
                    p.set(f"{key}  DOF {self.dof}", self.app.graph(trial, self.dof, name), self.xrange(name))

    def layout(self, width):
        """表示幅に合わせて波形・グラフの大きさを決める"""
        self.width = width
        cell = max(200, int((width - 140 - 80) / 4))
        hgt = int(min(210, max(150, cell * 0.55)))
        for p in self.plots.values():
            p.resize(cell, hgt)
        for s0, s1 in self.range_scales.values():
            s0.config(length=max(60, cell // 2 - 40))
            s1.config(length=max(60, cell // 2 - 40))
        if self.model is not None:
            wrap = max(400, width - 20)
            for lab in (self.help, self.proposal, self.sent_label):
                lab.config(wraplength=wrap)
            self.status.config(wraplength=max(300, width - 700))
            self.formula.config(wraplength=max(300, width - 700))


# ==============================================================================
# GUI本体
# ==============================================================================
UI_FONT = "TkDefaultFont"   # setup_japanese_font の後で日本語フォント名になる

# ウィンドウ・ダイアログのタイトル（タイトルバー）は英字にする。
#   WSLg ではタイトルバーをWindows側（Weston）が描き、日本語のグリフを持たないため文字化けする
#   （GUI_android_tk.py などのタイトルも英字）。ダイアログの本文はTkが日本語フォントで描くので日本語のまま。
WINDOW_TITLE = "FF setting GUI (ADRC + quintic FF)"
TITLE_SEND = "Send FF parameters"
TITLE_SEND_ZERO = "Send zero FF"
TITLE_RECORD = "Record"
TITLE_SAVE = "Save"
TITLE_PRESET = "Preset"
TITLE_BULK = "Bulk setting"
TITLE_FF = "FF setting"
TITLE_SETTINGS = "Settings / trial data"


class FFSettingApp:
    STATE_STYLE = {
        'idle':        ("停止中", "#909090"),
        'armed':       ("待機中", "#e08000"),
        'eval_window': ("● 記録中（評価区間）", "#d00000"),
        'recording':   ("● 記録中", "#1060c0"),
        'evaluating':  ("評価中", "#7030a0"),
        'done':        ("記録終了", "#208020"),
        'error':       ("エラー", "#700000"),
    }

    def __init__(self, node, settings, settings_warning=None, store=None):
        global UI_FONT
        self.node = node
        self.settings = settings
        self.store = store if store is not None else TrialStore()
        self.models = {d: DofModel(d, **settings['dof'][str(d)]) for d in FF_DOF_MAP}
        self.blocks = {}
        self.scroll_canvases = set()
        self.eval_queue = queue.SimpleQueue()
        self.eval_thread = None
        self.saves = []
        self.graph_cache = {}
        self.prefs_job = None
        self.layout_job = None
        self.sent_snap = node.sent_snapshot()
        self.next_init_cache = {}
        self.warn_lines = deque(maxlen=4)

        self.root = tk.Tk()
        gat.setup_japanese_font(self.root)
        UI_FONT = gat.UI_FONT
        self.root.title(WINDOW_TITLE)
        self.monitors = gat.list_monitors()
        self.root.geometry(gat.window_geometry_for(gat.initial_monitor(self.root, self.monitors)))

        self.build_top_bar()
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        self.tab_main = tk.Frame(self.notebook)
        self.tab_bulk = tk.Frame(self.notebook)
        self.tab_preset = tk.Frame(self.notebook)
        self.notebook.add(self.tab_main, text="FF設定・試行結果")
        self.notebook.add(self.tab_bulk, text="全自由度の一括設定")
        self.notebook.add(self.tab_preset, text="プリセット")
        self.build_main_tab()
        self.build_bulk_tab()
        self.build_preset_tab()

        for d in FF_DOF_MAP:
            self.models[d].listeners.append(self.on_any_model_change)
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            self.root.bind_all(seq, self.on_mousewheel)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.set_visible_dofs([settings['start_dof']])
        self.update_presets_from_store(save=False)
        self.refresh_sent_views()
        self.show_record_state('idle', "「● 記録待機」を押してから、GUI_android_tk.py で目標値を送ってください")
        self.poll()
        warns = [w for w in (settings_warning, self.store.load_error) if w]
        if warns:
            self.root.after(300, lambda: messagebox.showwarning(TITLE_SETTINGS, "\n\n".join(warns)))

    # =========================================================
    # 上部（記録・FF送信）: どのタブでも見える
    # =========================================================
    def build_top_bar(self):
        bar = tk.Frame(self.root, bd=1, relief=tk.GROOVE)
        bar.pack(fill=tk.X, padx=4, pady=(4, 2))
        r1 = tk.Frame(bar)
        r1.pack(fill=tk.X, padx=4, pady=2)
        self.record_button = tk.Button(r1, text="● 記録待機", width=16, command=self.on_record_button,
                                       font=(UI_FONT, 10, 'bold'))
        self.record_button.pack(side=tk.LEFT)
        tk.Label(r1, text="  記録時間 [s]").pack(side=tk.LEFT)
        self.duration_var = tk.StringVar()
        self.duration_spin = tk.Spinbox(r1, from_=RECORD_DURATION_MIN, to=RECORD_DURATION_MAX, increment=1, width=6,
                                        textvariable=self.duration_var)
        self.duration_spin.pack(side=tk.LEFT)
        self.duration_var.set(gat.format_value(self.settings['record_duration']))
        self.duration_var.trace_add("write", lambda *a: self.on_duration_change())
        tk.Label(r1, text=f"（評価区間は最初の {EVAL_TIME:g} s）   保存先").pack(side=tk.LEFT)
        self.record_dir_var = tk.StringVar(value=self.settings['record_dir'] or os.getcwd())
        tk.Entry(r1, textvariable=self.record_dir_var, width=40).pack(side=tk.LEFT)
        tk.Button(r1, text="参照", command=self.choose_record_dir).pack(side=tk.LEFT, padx=2)

        r2 = tk.Frame(bar)
        r2.pack(fill=tk.X, padx=4, pady=2)
        self.record_state = tk.Label(r2, text="", width=18, fg='white', font=(UI_FONT, 12, 'bold'))
        self.record_state.pack(side=tk.LEFT)
        self.progress = ttk.Progressbar(r2, length=220, maximum=1000)
        self.progress.pack(side=tk.LEFT, padx=6)
        self.record_detail = tk.Label(r2, text="", font=(UI_FONT, 10, 'bold'), anchor='w', justify='left')
        self.record_detail.pack(side=tk.LEFT, fill=tk.X, expand=True)

        r3 = tk.Frame(bar)
        r3.pack(fill=tk.X, padx=4, pady=2)
        self.send_button = tk.Button(r3, text="FFパラメータ送信（全自由度）", command=self.send_ff,
                                     bg='#d8f0d8', font=(UI_FONT, 10, 'bold'))
        self.send_button.pack(side=tk.LEFT)
        self.zero_button = tk.Button(r3, text="FFを全て0にして送信", command=self.send_zero)
        self.zero_button.pack(side=tk.LEFT, padx=6)
        self.sent_summary = tk.Label(r3, text="", font=(UI_FONT, 9), anchor='w', justify='left')
        self.sent_summary.pack(side=tk.LEFT, fill=tk.X, expand=True)
        msgs = tk.Frame(bar)
        msgs.pack(fill=tk.X, padx=4)
        msgs.columnconfigure(0, weight=1)
        self.save_status = tk.Label(msgs, text="", font=(UI_FONT, 9), anchor='w', fg='#204080')
        self.warn_label = tk.Label(msgs, text="", font=(UI_FONT, 9), anchor='w', justify='left', fg='#b04000')
        self.save_status.grid(row=0, column=0, sticky='ew')
        self.warn_label.grid(row=1, column=0, sticky='ew')
        self.save_status.grid_remove()
        self.warn_label.grid_remove()

    def show_record_state(self, state, detail):
        text, color = self.STATE_STYLE[state]
        self.record_state.config(text=text, bg=color)
        self.record_detail.config(text=detail, fg=color if state in ('eval_window', 'recording') else 'black')
        s = self.node.session
        if state == 'armed':
            self.record_button.config(text="待機を取消", state=tk.NORMAL)
        elif state == 'eval_window':
            self.record_button.config(text="記録を中止（破棄）", state=tk.NORMAL)
        elif state == 'recording':
            self.record_button.config(text="■ ここで記録終了", state=tk.NORMAL)
        elif state == 'evaluating':
            self.record_button.config(text="● 記録待機", state=tk.DISABLED)
        else:
            self.record_button.config(text="● 記録待機", state=tk.NORMAL)
        recording = s is not None and s.state == Recorder.RECORDING
        for b in (self.send_button, self.zero_button):
            b.config(state=tk.DISABLED if recording else tk.NORMAL)

    def read_duration(self):
        try:
            v = float(self.duration_var.get())
        except ValueError:
            return None
        return v if RECORD_DURATION_MIN <= v <= RECORD_DURATION_MAX else None

    def on_duration_change(self):
        v = self.read_duration()
        self.duration_spin.config(bg='white' if v is not None else '#ffc0c0')
        if v is None:
            return
        s = self.node.session
        if s is not None:
            s.duration = v
        if v != self.settings['record_duration']:
            self.settings['record_duration'] = v
            self.save_prefs_later()

    def choose_record_dir(self):
        path = filedialog.askdirectory(initialdir=self.record_dir_var.get() or os.getcwd(), title="Select save folder")
        if path:
            self.record_dir_var.set(path)
            self.settings['record_dir'] = path
            self.save_prefs_later()

    def base_dir(self):
        base = os.path.abspath(os.path.expanduser(self.record_dir_var.get().strip() or os.getcwd()))
        os.makedirs(base, exist_ok=True)
        if not os.access(base, os.W_OK):
            raise OSError("書き込み権限がありません")
        if base != self.settings['record_dir']:
            self.settings['record_dir'] = base
            self.save_prefs_later()
        return base

    def on_record_button(self):
        s = self.node.session
        if s is None:
            duration = self.read_duration()
            if duration is None:
                messagebox.showerror(TITLE_RECORD, f"記録時間は {RECORD_DURATION_MIN:g}〜{RECORD_DURATION_MAX:g} s で入力してください"
                                               f"（評価区間 {EVAL_TIME:g} s より短くはできません）")
                return
            self.node.session = Recorder(duration)
            unsent = self.unsent_dofs()
            note = (f"   ※DOF {', '.join(map(str, unsent))} のFF設定は未送信です（この試行には送信済みの値が使われます）"
                    if unsent else "")
            self.show_record_state('armed', "目標値を送ってください。目標値が1自由度でも変わった時点を 0 秒として記録します" + note)
        elif s.state == Recorder.ARMED:
            s.finish()
        elif s.state == Recorder.RECORDING:
            if s.elapsed() < EVAL_TIME:
                if messagebox.askokcancel(TITLE_RECORD, f"評価区間（{EVAL_TIME:g} s）が終わっていないため評価できません。"
                                                     "この記録を破棄しますか？"):
                    s.cancel()
            else:
                s.finish()

    # =========================================================
    # 定期処理
    # =========================================================
    def poll(self):
        try:
            self.poll_events()
            self.poll_record()
            self.poll_eval()
            self.poll_saves()
        except Exception:
            traceback.print_exc()
        self.root.after(100, self.poll)

    def focused(self):
        try:
            return self.root.focus_get()
        except (KeyError, tk.TclError):
            return None

    def add_warning(self, text):
        self.warn_lines.append(f"{datetime.now().strftime('%H:%M:%S')}  {text}")
        self.warn_label.config(text="\n".join(self.warn_lines))
        self.warn_label.grid()

    def poll_events(self):
        changed = False
        while True:
            try:
                ev = self.node.events.get_nowait()
            except queue.Empty:
                break
            kind, k = ev[0], ev[1]
            if kind == 'board_restart':
                self.add_warning(f"board{k} の配信が {ev[2]:.1f} s 途切れて再開しました。実機が起動したとみなし、"
                                 f"board{k} のFFを 0 として扱います（未確認）。必要ならFFを送信し直してください")
                changed = True
            elif kind == 'foreign_ff':
                self.add_warning(f"他のプログラムが board{k} のFFを送信しました。実機のFFはこのGUIの送信値と違う可能性があります"
                                 "（その値は使いません）")
                changed = True
        snap = self.node.sent_snapshot()
        if changed or snap != self.sent_snap:
            self.sent_snap = snap
            self.refresh_sent_views()

    def poll_record(self):
        s = self.node.session
        if s is None:
            if self.eval_thread is None:
                self.progress['value'] = 0
            return
        if s.state == Recorder.RECORDING:
            el = s.elapsed()
            if el >= s.duration:
                s.finish()
            self.progress['value'] = int(1000 * min(1.0, el / max(s.duration, 1e-6)))
            if el < EVAL_TIME:
                self.show_record_state('eval_window',
                                       f"評価区間（0〜{EVAL_TIME:g} s）: 他の目標値を送らないでください  残り {EVAL_TIME - el:.1f} s"
                                       f"     記録 {el:.1f} / {s.duration:g} s（受信 {s.rows} 件）")
            else:
                self.show_record_state('recording',
                                       f"評価区間は終わりました。目標値（Reset pose など）を送っても構いません"
                                       f"     記録 {min(el, s.duration):.1f} / {s.duration:g} s（受信 {s.rows} 件）")
        elif s.state == Recorder.CANCELLED:
            self.node.session = None
            self.show_record_state('idle', "記録を取り消しました")
        elif s.state == Recorder.DONE:
            self.node.session = None
            if s.t_end is not None and s.t_end < EVAL_TIME - 1e-6:
                self.show_record_state('error', "評価区間が終わる前に記録が終わったため、評価できません（破棄しました）")
                return
            trial = s.to_trial()
            used = {t.id for t in self.store.rows.values() if t is not None}
            base, n = trial.meta['id'], 2
            while trial.meta['id'] in used:
                trial.meta['id'] = f"{base}_{n}"
                n += 1
            st = trial.meta['ff_status']
            for k, (status, _) in st.items():
                if status in ('nosub', 'foreign'):
                    trial.meta['warnings'].append(f"board{k}: {SENT_STATUS_TEXT[status]}")
            params = {d: (m.T, m.ylo, m.yhi) for d, m in self.models.items()}
            self.eval_thread = threading.Thread(target=self.eval_worker, args=(trial, params), daemon=True)
            self.eval_thread.start()
            self.show_record_state('evaluating', "3次遅れ系の同定と評価をしています…")

    def eval_worker(self, trial, params):
        try:
            evaluate_trial(trial)
            nxt = {}
            for d in FF_DOF_MAP:
                T, ylo, yhi = params[d]
                r = next_initial(trial, d, T, ylo, yhi)
                nxt[str(d)] = {'values': None if r['values'] is None else list(r['values']),
                               'target': list(r['target']), 'reasons': r['reasons'], 'exact': bool(r['exact']),
                               'params': [T, ylo, yhi]}
            trial.meta['next_init'] = nxt
            trial.save(self.store.trial_path(trial.id))
            self.eval_queue.put(('ok', trial))
        except Exception:
            self.eval_queue.put(('error', traceback.format_exc()))

    def poll_eval(self):
        try:
            kind, obj = self.eval_queue.get_nowait()
        except queue.Empty:
            return
        self.eval_thread = None
        if kind == 'error':
            print(obj, file=sys.stderr)
            self.show_record_state('error', "評価中にエラーが起きました（詳細は端末に表示）")
            return
        trial = obj
        try:
            best_updated = self.store.add(trial)
        except OSError as e:
            best_updated = False
            self.add_warning(f"試行データを保存できませんでした: {e}")
        self.graph_cache = {k: v for k, v in self.graph_cache.items()
                            if k[0] in {t.id for t in self.store.rows.values() if t is not None}}
        self.next_init_cache = {}
        self.update_presets_from_store()
        applied = self.apply_next_init([d for d in FF_DOF_MAP if self.models[d].enabled], from_trial=True)
        for block in self.blocks.values():
            block.refresh_results()
        self.refresh_proposals()
        J = trial.J
        text = (f"記録終了（{trial.meta['start']}）: J = {J:.4g}（評価 {trial.meta['n_eval']} 自由度）"
                if J is not None else f"記録終了（{trial.meta['start']}）: 評価できた自由度がありません")
        if best_updated:
            text += " → best を更新"
        elif not trial.meta['candidate']:
            text += f"  ※best の候補外: {trial.meta['candidate_note']}"
        if applied:
            text += f"   次試行の初期値を {len(applied)} 自由度に設定しました"
        self.show_record_state('done', text)
        for w in trial.meta['warnings']:
            self.add_warning(w)

    # =========================================================
    # FFの送信
    # =========================================================
    def send_ff(self):
        bad = [d for d in FF_DOF_MAP if self.models[d].coeffs6() is None]
        if bad:
            messagebox.showerror(TITLE_SEND,
                                 "次の自由度のFF設定が条件を満たしていないため、送信しません（全自由度とも送信していません）:\n"
                                 + "\n".join(f"  DOF {d}: {' / '.join(self.models[d].check().errors)}" for d in bad))
            return
        table = np.array([self.models[d].coeffs6() for d in FF_DOF_MAP])
        settings = [self.models[d].setting() if self.models[d].enabled else None for d in FF_DOF_MAP]
        self.publish_all(table, settings)

    def send_zero(self):
        """全自由度を「FFなし」にして送る。確認でOKを押すまで、設定も変えず何も送らない"""
        if not messagebox.askokcancel(TITLE_SEND_ZERO,
                                      "全自由度のFF設定を「FFなし」（極値・係数すべて0）にして送信します。よろしいですか？\n"
                                      "（今の設定はプリセットに登録していなければ失われます）"):
            return
        table = zero_ff_table()
        for n, d in enumerate(FF_DOF_MAP):
            table[n, 5] = self.models[d].T
        if not self.publish_all(table, [None] * N_FF, TITLE_SEND_ZERO):
            return
        for d in FF_DOF_MAP:
            self.models[d].update(enabled=False)

    def publish_all(self, table, settings, title=None):
        """全ボードへ送る。戻り値: 送ったか（記録中・確認でキャンセルなら False で、何も送らない）"""
        title = title or TITLE_SEND
        s = self.node.session
        if s is not None and s.state == Recorder.RECORDING:
            messagebox.showerror(title, "記録中はFFを送信できません（試行のFFが分からなくなるため）")
            return False
        boards = range(1, len(BOARD_LAYOUT) + 1)
        missing = [k for k in boards if not self.node.ff_subscriber_present(k)]
        if missing and not messagebox.askokcancel(
                title, "次のボードのFFの受信側が見つかりません。送信しても届かない可能性があります:\n  "
                + ", ".join(f"board{k}" for k in missing) + "\n送信しますか？"):
            return False
        self.node.publish_ff(table, settings, {k: ('nosub' if k in missing else 'sent') for k in boards})
        self.sent_snap = self.node.sent_snapshot()
        self.refresh_sent_views()
        return True

    def refresh_sent_views(self):
        snap = self.sent_snap
        parts = []
        for k in range(1, len(BOARD_LAYOUT) + 1):
            status, stamp = snap['status'][str(k)]
            short = SENT_STATUS_SHORT[status]
            parts.append(f"board{k}: {short}{' ' + stamp if stamp else ''}")
        self.sent_summary.config(text="GUIの送信状態（受信確認なし）  " + "  |  ".join(parts))
        for block in self.blocks.values():
            block.refresh_sent()
        self.refresh_bulk_sent()

    # =========================================================
    # FF設定の操作（DOFブロック・一括設定・プリセットから共通）
    # =========================================================
    def set_dof_params(self, dof, **kw):
        m = self.models[dof]
        m.update(**kw)
        self.settings['dof'][str(dof)] = {'T': m.T, 'ylo': m.ylo, 'yhi': m.yhi}
        self.next_init_cache.pop(dof, None)
        self.save_prefs_later()

    def default_shape(self, dof):
        """前回の試行が使えないときの初期値: t(t−T/2)(t−T) 形（極値 ±DEFAULT_FF_PEAK）を上下限に収めたもの"""
        m = self.models[dof]
        t1, t2 = m.T * (3 - math.sqrt(3)) / 6, m.T * (3 + math.sqrt(3)) / 6
        return nearest_feasible(m.T, t1, DEFAULT_FF_PEAK, t2, -DEFAULT_FF_PEAK, m.ylo, m.yhi)

    def next_init_for(self, dof):
        """now の試行から求めた次試行の初期値（今の T・上下限で計算）"""
        trial = self.store.rows['now']
        if trial is None:
            return None
        if dof not in self.next_init_cache:
            m = self.models[dof]
            stored = trial.meta.get('next_init', {}).get(str(dof))
            # 評価したときと T・上下限が同じなら、そのときの結果を使う
            if stored is not None and tuple(stored.get('params', ())) == (m.T, m.ylo, m.yhi):
                self.next_init_cache[dof] = stored
            else:
                r = next_initial(trial, dof, m.T, m.ylo, m.yhi)
                self.next_init_cache[dof] = {'values': r['values'], 'target': r['target'], 'reasons': r['reasons'],
                                             'exact': r['exact']}
        return self.next_init_cache[dof]

    def enable_ff(self, dof):
        """FFを有効にする。極値は次試行の初期値（now の試行がなければ既定の形）から始める"""
        m = self.models[dof]
        init = self.next_init_for(dof)
        vals = init['values'] if init is not None else None
        if vals is None:
            vals = self.default_shape(dof)
        if vals is None:
            messagebox.showerror(TITLE_FF, f"DOF {dof}: y の上下限の中で条件を満たす初期値が見つかりません")
            m.update(enabled=False)
            return
        m.update(enabled=True, t1=vals[0], y1=vals[1], t2=vals[2], y2=vals[3])

    def apply_next_init(self, dofs, from_trial=False):
        """次試行の初期値（T10・T90 と前回の y）を設定する。戻り値: 設定した自由度"""
        applied = []
        missing = []
        for d in dofs:
            init = self.next_init_for(d)
            if init is None or init['values'] is None:
                missing.append(d)
                continue
            v = init['values']
            self.models[d].update(enabled=True, t1=v[0], y1=v[1], t2=v[2], y2=v[3])
            applied.append(d)
        if missing and not from_trial:
            messagebox.showinfo(TITLE_FF,
                                "次の自由度は初期値を求められません（now の試行がない、または条件を満たす値がない）:\n  "
                                + ", ".join(f"DOF {d}" for d in missing))
        self.refresh_proposals()
        return applied

    def refresh_proposals(self):
        for dof, block in self.blocks.items():
            if dof in self.models:
                block.show_proposal(self.proposal_text(dof))

    def proposal_text(self, dof):
        init = self.next_init_for(dof)
        if init is None:
            return "次試行の初期値: now の試行がまだありません"
        tgt = init['target']
        text = (f"次試行の初期値（now の試行から）: 目標 t1=T10={tgt[0]:.3f} s, y1={tgt[1]:.3f}, "
                f"t2=T90={tgt[2]:.3f} s, y2={tgt[3]:.3f}")
        if init['values'] is not None and not init['exact']:
            v = init['values']
            text += f"  → 実現可能な値 t1={v[0]:.3f}, y1={v[1]:.3f}, t2={v[2]:.3f}, y2={v[3]:.3f}"
        if init['reasons']:
            text += "\n  " + "\n  ".join(init['reasons'])
        return text

    def apply_preset(self, dof, k):
        n = FF_INDEX[dof]
        p = self.settings['presets'][k][n]
        if p is None:
            messagebox.showinfo(TITLE_PRESET, f"DOF {dof} のプリセット{k + 1}は未登録です")
            return
        if not FF_T_RANGE[0] <= p['T'] <= FF_T_RANGE[1]:
            messagebox.showerror(TITLE_PRESET, f"プリセット{k + 1}の T = {p['T']} は範囲外です")
            return
        m = self.models[dof]
        if p['enabled']:
            m.update(T=p['T'], enabled=True, t1=p['t1'], y1=p['y1'], t2=p['t2'], y2=p['y2'])
        else:
            m.update(T=p['T'], enabled=False)
        self.set_dof_params(dof)

    def update_presets_from_store(self, save=True):
        """プリセット1〜3 に now・last・best の試行で使ったFFの極値を登録する"""
        for k, key in enumerate(ROW_KEYS):
            trial = self.store.rows[key]
            if trial is None:
                continue
            for n in range(N_FF):
                s = trial.meta['ff_setting'][n]
                T = float(trial.meta['ff_table'][n][5])
                if s is None:
                    p = {'enabled': False, 'T': T, 't1': 0.0, 'y1': 0.0, 't2': 0.0, 'y2': 0.0}
                else:
                    p = {key2: s[key2] for key2 in ('enabled', 'T', 't1', 'y1', 't2', 'y2')}
                p['src'] = f"{key} の試行 {trial.id}"
                self.settings['presets'][k][n] = p
        if save:
            self.save_prefs_later()
        self.refresh_preset_tab()

    def on_any_model_change(self, model):
        self.schedule_bulk_row(model.dof)

    def is_unsent(self, dof):
        """FF設定が、GUIが最後に送信した値と違うか"""
        m = self.models[dof]
        n = FF_INDEX[dof]
        sent = self.sent_snap['setting'][n]
        if not m.enabled:
            return sent is not None or abs(self.sent_snap['table'][n][5] - m.T) > 1e-12
        return sent is None or any(abs(sent[k] - getattr(m, k)) > 1e-12 for k in ('T', 't1', 'y1', 't2', 'y2'))

    def unsent_dofs(self):
        return [d for d in FF_DOF_MAP if self.is_unsent(d)]

    # =========================================================
    # 試行結果の表示・保存
    # =========================================================
    def graph(self, trial, dof, name):
        key = (trial.id, dof, name)
        if key not in self.graph_cache:
            self.graph_cache[key] = graph_content(trial, dof, name)
        return self.graph_cache[key]

    def row_text(self, key, trial, dof):
        if trial is None:
            return f"{key}\n（なし）"
        m = trial.meta
        lines = [f"{key}  {m['start'].replace('T', ' ')[5:]}"]
        lines.append(f"J = {m['J']:.4g}（{m['n_eval']} DOF の平均）" if m['J'] is not None else "J = なし")
        ev = m['eval'][dof]
        if ev.get('status') == 'ok':
            lines.append(f"このDOF: J_i = {ev['J']:.4g}\nΔPOT = {ev['dpot']:g}")
        else:
            lines.append(f"このDOF: {ev.get('note') or ev.get('status')}")
        if not m.get('candidate'):
            lines.append(f"best候補外: {m.get('candidate_note')}")
        return "\n".join(lines)

    def save_row(self, key):
        trial = self.store.rows[key]
        if trial is None:
            messagebox.showinfo(TITLE_SAVE, f"{key} の試行はまだありません")
            return
        try:
            folder = make_save_folder(self.base_dir(), key, trial.meta['stamp'])
        except OSError as e:
            messagebox.showerror(TITLE_SAVE, f"保存先にフォルダを作れません:\n{e}")
            return
        ctx = multiprocessing.get_context('spawn')
        q = ctx.Queue()
        proc = ctx.Process(target=save_trial_outputs, args=(trial.payload(), folder, key, q), name="ff_trial_save")
        proc.start()
        self.saves.append({'proc': proc, 'queue': q, 'folder': folder, 'key': key, 'dead': 0, 'text': "開始"})
        self.update_save_status()

    def poll_saves(self):
        if not self.saves:
            return
        for job in list(self.saves):
            while True:
                try:
                    msg = job['queue'].get_nowait()
                except queue.Empty:
                    break
                if msg[0] == 'csv':
                    job['text'] = "CSVを書きました。グラフを作成中"
                elif msg[0] == 'graph':
                    job['text'] = f"グラフ {msg[1]}/{msg[2]}"
                elif msg[0] == 'done':
                    job['text'] = "完了"
                    job['finished'] = True
                elif msg[0] == 'error':
                    print(msg[1], file=sys.stderr)
                    job['text'] = "エラー（詳細は端末に表示）"
                    job['finished'] = True
            if job.get('finished'):
                job['proc'].join(timeout=1)
                self.saves.remove(job)
                self.set_save_status(f"保存 {job['key']}: {job['text']}  → {job['folder']}")
                continue
            if not job['proc'].is_alive():
                job['dead'] += 1
                if job['dead'] >= 5:
                    self.saves.remove(job)
                    self.set_save_status(f"保存 {job['key']}: 異常終了（exitcode={job['proc'].exitcode}）"
                                         f"  → {job['folder']}")
                    continue
        if self.saves:
            self.update_save_status()

    def set_save_status(self, text):
        self.save_status.config(text=text)
        self.save_status.grid()

    def update_save_status(self):
        self.save_status.grid()
        self.save_status.config(text="保存中: " + "   ".join(f"{j['key']} {j['text']} → {j['folder']}" for j in self.saves))

    # =========================================================
    # メインタブ（表示する自由度・自由度ごとのブロック）
    # =========================================================
    def build_main_tab(self):
        bar = tk.Frame(self.tab_main)
        bar.pack(fill=tk.X, padx=6, pady=(4, 0))
        tk.Label(bar, text="表示する自由度", font=(UI_FONT, 9, 'bold')).grid(row=0, column=0, padx=(0, 6), sticky='w')
        boxes = tk.Frame(bar)
        boxes.grid(row=0, column=1, sticky='w')
        self.dof_vars = []
        for d in range(N_DOF):
            v = tk.BooleanVar(value=False)
            tk.Checkbutton(boxes, text=str(d) + ("" if d in FF_INDEX else "*"), variable=v,
                           command=self.on_dof_toggle).grid(row=d // 13, column=d % 13, sticky='w')
            self.dof_vars.append(v)
        sel = tk.Frame(bar)
        sel.grid(row=0, column=2, padx=8, sticky='w')
        tk.Button(sel, text="全解除", command=lambda: self.set_visible_dofs([])).grid(row=0, column=0, sticky='ew')
        tk.Button(sel, text="FFを与えているDOF",
                  command=lambda: self.set_visible_dofs([d for d in FF_DOF_MAP if self.models[d].enabled])
                  ).grid(row=1, column=0, sticky='ew')
        tk.Label(bar, text="* = FFスロットなし", font=(UI_FONT, 8), fg='#606060').grid(row=0, column=3, sticky='w')
        self.main_canvas, self.main_inner = self.make_scrollable(self.tab_main, stretch=True)
        self.main_canvas.bind("<Configure>", lambda e: self.schedule_layout(), add="+")
        self.main_empty = tk.Label(self.main_inner, text="表示する自由度を上で選択してください", fg='#808080')

    def on_dof_toggle(self):
        self.set_visible_dofs([d for d in range(N_DOF) if self.dof_vars[d].get()])

    def set_visible_dofs(self, dofs):
        dofs = sorted(set(dofs))
        for d in range(N_DOF):
            self.dof_vars[d].set(d in dofs)
        for d in list(self.blocks):
            if d not in dofs:
                self.blocks.pop(d).destroy()
        for d in dofs:
            if d not in self.blocks:
                block = DofBlock(self, self.main_inner, d)
                self.blocks[d] = block
                block.refresh_results()
                if d in self.models:
                    block.show_proposal(self.proposal_text(d))
        for i, d in enumerate(dofs):
            self.blocks[d].frame.grid(row=i, column=0, sticky='ew', padx=2, pady=4)
        if dofs:
            self.main_empty.grid_remove()
            if dofs[0] != self.settings['start_dof']:
                self.settings['start_dof'] = dofs[0]
                self.save_prefs_later()
        else:
            self.main_empty.grid(row=0, column=0, padx=20, pady=20)
        self.schedule_layout()

    def schedule_layout(self):
        if self.layout_job is not None:
            self.root.after_cancel(self.layout_job)
        self.layout_job = self.root.after(60, self.layout_blocks)

    def layout_blocks(self):
        self.layout_job = None
        width = self.main_canvas.winfo_width()
        if width < 50:
            return
        for block in self.blocks.values():
            block.layout(max(1100, width - 16))

    def make_scrollable(self, parent, stretch=False):
        """縦・横にスクロールできる領域を作り (Canvas, 中身の Frame) を返す

        stretch: 中身の幅を表示幅に合わせて広げる（メインタブ）。False なら中身は必要な幅のまま左に寄せる。
        """
        outer = tk.Frame(parent)
        outer.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        outer.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)
        canvas = tk.Canvas(outer, highlightthickness=0)
        vbar = tk.Scrollbar(outer, orient=tk.VERTICAL, command=canvas.yview)
        hbar = tk.Scrollbar(outer, orient=tk.HORIZONTAL, command=canvas.xview)
        inner = tk.Frame(canvas)
        if stretch:
            inner.columnconfigure(0, weight=1)
        window = canvas.create_window((0, 0), window=inner, anchor='nw')
        canvas.configure(yscrollcommand=vbar.set, xscrollcommand=hbar.set)
        canvas.grid(row=0, column=0, sticky='nsew')
        vbar.grid(row=0, column=1, sticky='ns')

        def update(_event=None):
            view = canvas.winfo_width()
            canvas.itemconfigure(window, width=max(view, inner.winfo_reqwidth()) if stretch else inner.winfo_reqwidth())
            if inner.winfo_reqwidth() > view > 1:
                hbar.grid(row=1, column=0, sticky='ew')
            else:
                hbar.grid_remove()
                canvas.xview_moveto(0)
            canvas.configure(scrollregion=canvas.bbox('all'))

        inner.bind("<Configure>", update)
        canvas.bind("<Configure>", update)
        self.scroll_canvases.add(canvas)
        return canvas, inner

    def on_mousewheel(self, event):
        try:
            w = self.root.winfo_containing(event.x_root, event.y_root)
        except (KeyError, tk.TclError):
            return
        while w is not None:
            if w in self.scroll_canvases:
                delta = -1 if (event.num == 5 or event.delta < 0) else 1
                if event.state & 0x0001:
                    w.xview_scroll(-delta, 'units')
                else:
                    w.yview_scroll(-delta, 'units')
                return
            w = w.master

    # =========================================================
    # 一括設定タブ
    # =========================================================
    BULK_COLS = ('t1', 'y1', 't2', 'y2')

    def build_bulk_tab(self):
        ctrl = tk.Frame(self.tab_bulk)
        ctrl.pack(fill=tk.X, padx=6, pady=4)
        r1 = tk.Frame(ctrl)
        r1.pack(fill=tk.X)
        tk.Button(r1, text="全選択", command=lambda: [v.set(True) for v in self.bulk_sel.values()]).pack(side=tk.LEFT)
        tk.Button(r1, text="全解除", command=lambda: [v.set(False) for v in self.bulk_sel.values()]).pack(side=tk.LEFT)
        tk.Label(r1, text="   選択した自由度に:").pack(side=tk.LEFT)
        tk.Button(r1, text="FFを与える", command=lambda: self.bulk(lambda d: self.enable_ff(d))).pack(side=tk.LEFT, padx=1)
        tk.Button(r1, text="FFなしにする", command=lambda: self.bulk(lambda d: self.models[d].update(enabled=False))
                  ).pack(side=tk.LEFT, padx=1)
        tk.Button(r1, text="次試行の初期値を適用", command=lambda: self.apply_next_init(self.bulk_selected())
                  ).pack(side=tk.LEFT, padx=1)
        for k, name in enumerate(DofBlock.PRESET_NAMES):
            tk.Button(r1, text=f"P{k + 1} {name}を適用",
                      command=lambda k=k: self.bulk(lambda d: self.apply_preset(d, k))).pack(side=tk.LEFT, padx=1)
        tk.Button(r1, text="表示する自由度にする", command=lambda: (self.set_visible_dofs(self.bulk_selected()),
                                                                 self.notebook.select(self.tab_main))
                  ).pack(side=tk.LEFT, padx=(8, 0))
        r2 = tk.Frame(ctrl)
        r2.pack(fill=tk.X, pady=(4, 0))
        tk.Label(r2, text="選択した自由度の  T [s]").pack(side=tk.LEFT)
        self.bulk_T = tk.Entry(r2, width=6)
        self.bulk_T.pack(side=tk.LEFT)
        tk.Label(r2, text=" y 下限").pack(side=tk.LEFT)
        self.bulk_ylo = tk.Entry(r2, width=6)
        self.bulk_ylo.pack(side=tk.LEFT)
        tk.Label(r2, text=" 上限").pack(side=tk.LEFT)
        self.bulk_yhi = tk.Entry(r2, width=6)
        self.bulk_yhi.pack(side=tk.LEFT)
        tk.Label(r2, text=" t1").pack(side=tk.LEFT)
        self.bulk_vals = {}
        for i, c in enumerate(self.BULK_COLS):
            if i:
                tk.Label(r2, text=f" {c}").pack(side=tk.LEFT)
            e = tk.Entry(r2, width=7)
            e.pack(side=tk.LEFT)
            self.bulk_vals[c] = e
        tk.Button(r2, text="入力した項目を適用（空欄は変えない）", command=self.bulk_apply_values).pack(side=tk.LEFT, padx=6)
        tk.Label(ctrl, text="表の値は Enter で反映します。送信は上の「FFパラメータ送信」で全自由度まとめて行います。",
                 font=(UI_FONT, 8), fg='#606060').pack(anchor='w')

        _, inner = self.make_scrollable(self.tab_bulk)
        heads = ("選択", "DOF", "FF", "T [s]", "t1 [s]", "y1", "t2 [s]", "y2", "y下限", "y上限", "判定", "GUIが最後に送信した値（受信確認なし）")
        for j, h in enumerate(heads):
            tk.Label(inner, text=h, font=(UI_FONT, 9, 'bold')).grid(row=0, column=j, padx=2, sticky='w')
        self.bulk_sel = {}
        self.bulk_rows = {}
        self.bulk_jobs = {}
        for i, d in enumerate(FF_DOF_MAP, start=1):
            sv = tk.BooleanVar(value=False)
            tk.Checkbutton(inner, variable=sv).grid(row=i, column=0)
            self.bulk_sel[d] = sv
            tk.Label(inner, text=f"{d}（FF{FF_INDEX[d] + 1}）").grid(row=i, column=1, sticky='w')
            ev = tk.BooleanVar(value=False)
            tk.Checkbutton(inner, variable=ev, command=lambda d=d, ev=ev: (
                self.enable_ff(d) if ev.get() else self.models[d].update(enabled=False))).grid(row=i, column=2)
            entries = {}
            for j, key in enumerate(('T', 't1', 'y1', 't2', 'y2', 'ylo', 'yhi'), start=3):
                e = tk.Entry(inner, width=8, justify='right')
                e.grid(row=i, column=j, padx=1)
                e.bind("<Return>", lambda _e, d=d, key=key: self.on_bulk_entry(d, key))
                entries[key] = e
            judge = tk.Label(inner, text="", anchor='w', width=24)
            judge.grid(row=i, column=10, sticky='w')
            sent = tk.Label(inner, text="", anchor='w', font=(UI_FONT, 8))
            sent.grid(row=i, column=11, sticky='w')
            self.bulk_rows[d] = {'enabled': ev, 'entries': entries, 'judge': judge, 'sent': sent}
            self.refresh_bulk_row(d)
        self.refresh_bulk_sent()

    def bulk_selected(self):
        return [d for d in FF_DOF_MAP if self.bulk_sel[d].get()]

    def bulk(self, fn):
        sel = self.bulk_selected()
        if not sel:
            messagebox.showinfo(TITLE_BULK, "自由度を選択してください")
            return
        for d in sel:
            fn(d)

    def bulk_apply_values(self):
        sel = self.bulk_selected()
        if not sel:
            messagebox.showinfo(TITLE_BULK, "自由度を選択してください")
            return
        vals = {}
        for key, e in (('T', self.bulk_T), ('ylo', self.bulk_ylo), ('yhi', self.bulk_yhi), *self.bulk_vals.items()):
            text = e.get().strip()
            if text:
                try:
                    vals[key] = float(text)
                except ValueError:
                    messagebox.showerror(TITLE_BULK, f"{key} に数値を入力してください")
                    return
        if 'T' in vals and not FF_T_RANGE[0] <= vals['T'] <= FF_T_RANGE[1]:
            messagebox.showerror(TITLE_BULK, f"T は {FF_T_RANGE[0]:g}〜{FF_T_RANGE[1]:g} s にしてください")
            return
        for d in sel:
            m = self.models[d]
            ylo, yhi = vals.get('ylo', m.ylo), vals.get('yhi', m.yhi)
            if not (-PWM_LIMIT <= ylo < 0 < yhi <= PWM_LIMIT):
                messagebox.showerror(TITLE_BULK, f"y の上下限は −{PWM_LIMIT:g} ≤ 下限 < 0 < 上限 ≤ {PWM_LIMIT:g} にしてください")
                return
        for d in sel:
            p = {k: vals[k] for k in ('T', 'ylo', 'yhi') if k in vals}
            ext = {k: vals[k] for k in self.BULK_COLS if k in vals}
            if ext:
                if not self.models[d].enabled:
                    self.enable_ff(d)
                p.update(ext)
            self.set_dof_params(d, **p)

    def on_bulk_entry(self, d, key):
        e = self.bulk_rows[d]['entries'][key]
        try:
            v = float(e.get())
        except ValueError:
            e.config(bg='#ffc0c0')
            return
        e.config(bg='white')
        m = self.models[d]
        if key in self.BULK_COLS:
            if not m.enabled:
                self.enable_ff(d)
            m.update(**{key: v})
        elif key == 'T':
            if not FF_T_RANGE[0] <= v <= FF_T_RANGE[1]:
                e.config(bg='#ffc0c0')
                return
            self.set_dof_params(d, T=v)
        else:
            ylo, yhi = (v, m.yhi) if key == 'ylo' else (m.ylo, v)
            if not (-PWM_LIMIT <= ylo < 0 < yhi <= PWM_LIMIT):
                e.config(bg='#ffc0c0')
                return
            self.set_dof_params(d, ylo=ylo, yhi=yhi)

    def schedule_bulk_row(self, d):
        if self.bulk_jobs.get(d) is None:
            self.bulk_jobs[d] = self.root.after(50, lambda: self.refresh_bulk_row(d))

    def refresh_bulk_row(self, d):
        self.bulk_jobs[d] = None
        m = self.models[d]
        row = self.bulk_rows[d]
        row['enabled'].set(m.enabled)
        focus = self.focused()
        for key, e in row['entries'].items():
            if focus is e:
                continue
            v = getattr(m, key)
            e.config(state=tk.NORMAL, bg='white')
            e.delete(0, tk.END)
            e.insert(0, f"{v:.6g}")
        chk = m.check()
        if not m.enabled:
            row['judge'].config(text="FFなし", fg='#404040')
        elif chk.ok:
            row['judge'].config(text="○ 条件を満たす", fg='#006000')
        else:
            row['judge'].config(text="× " + chk.errors[0][:40], fg='#c00000')

    def refresh_bulk_sent(self):
        if not hasattr(self, 'bulk_rows'):
            return
        snap = self.sent_snap
        for d in FF_DOF_MAP:
            n = FF_INDEX[d]
            s = snap['setting'][n]
            status, stamp = snap['status'][str(DOF_BOARD[d][0])]
            text = ("FFなし" if s is None else f"t1={s['t1']:.3f} y1={s['y1']:.2f} t2={s['t2']:.3f} y2={s['y2']:.2f} T={s['T']:g}")
            self.bulk_rows[d]['sent'].config(text=f"{text}  [{SENT_STATUS_SHORT[status]}{' ' + stamp if stamp else ''}]")

    # =========================================================
    # プリセットタブ
    # =========================================================
    def build_preset_tab(self):
        top = tk.Frame(self.tab_preset)
        top.pack(fill=tk.X, padx=6, pady=6)
        tk.Label(top, text="自由度").pack(side=tk.LEFT)
        self.preset_dof = ttk.Combobox(top, state='readonly', width=24,
                                       values=[f"DOF {d}（FF番号 {FF_INDEX[d] + 1}）" for d in FF_DOF_MAP])
        self.preset_dof.current(0)
        self.preset_dof.bind("<<ComboboxSelected>>", lambda e: self.refresh_preset_tab())
        self.preset_dof.pack(side=tk.LEFT, padx=4)
        tk.Label(top, text="プリセット1〜3は now・last・best の試行で使ったFFが自動で登録されます（編集不可）。"
                           "プリセット4は自由に設定できます。全自由度ぶん保存し、再起動後も引き継ぎます。",
                 font=(UI_FONT, 9), fg='#505050').pack(side=tk.LEFT, padx=8)
        grid = tk.Frame(self.tab_preset)
        grid.pack(fill=tk.X, padx=6)
        fields = ('enabled', 'T', 't1', 'y1', 't2', 'y2')
        labels = ("FFを与える", "T [s]", "t1 [s]", "y1", "t2 [s]", "y2", "判定", "登録元")
        for i, lab in enumerate(labels, start=1):
            tk.Label(grid, text=lab, font=(UI_FONT, 9, 'bold')).grid(row=i, column=0, sticky='e', padx=4)
        self.preset_cells = []
        for k, name in enumerate(DofBlock.PRESET_NAMES):
            col = k + 1
            tk.Label(grid, text=f"プリセット{k + 1}（{name}）", font=(UI_FONT, 10, 'bold')).grid(row=0, column=col, padx=8)
            cells = {}
            if k < 3:
                for i, f in enumerate(fields, start=1):
                    cells[f] = tk.Label(grid, text="", width=14, anchor='e', relief=tk.SUNKEN, bg='#f4f4f4')
                    cells[f].grid(row=i, column=col, padx=8, pady=1)
            else:
                self.p4_enabled = tk.BooleanVar(value=False)
                cells['enabled'] = tk.Checkbutton(grid, variable=self.p4_enabled)
                cells['enabled'].grid(row=1, column=col)
                for i, f in enumerate(fields[1:], start=2):
                    cells[f] = tk.Entry(grid, width=14, justify='right')
                    cells[f].grid(row=i, column=col, padx=8, pady=1)
            cells['judge'] = tk.Label(grid, text="", width=22, anchor='w', wraplength=180, justify='left')
            cells['judge'].grid(row=7, column=col, padx=8)
            cells['src'] = tk.Label(grid, text="", width=22, anchor='w', wraplength=180, justify='left', font=(UI_FONT, 8))
            cells['src'].grid(row=8, column=col, padx=8)
            cells['wave'] = tk.Canvas(grid, width=200, height=90, bg='white', highlightthickness=1,
                                      highlightbackground='#c8c8c8')
            cells['wave'].grid(row=9, column=col, padx=8, pady=4)
            tk.Button(grid, text="FF設定に適用", command=lambda k=k: self.apply_preset(self.preset_tab_dof(), k)
                      ).grid(row=10, column=col, pady=2)
            self.preset_cells.append(cells)
        btns = tk.Frame(grid)
        btns.grid(row=11, column=4, pady=4)
        tk.Button(btns, text="現在のFF設定をプリセット4に入れる", command=self.p4_from_current).pack(fill=tk.X)
        tk.Button(btns, text="プリセット4を保存", command=self.p4_save, bg='#d8f0d8').pack(fill=tk.X, pady=2)
        tk.Button(btns, text="編集を破棄", command=self.refresh_preset_tab).pack(fill=tk.X)
        self.preset_status = tk.Label(self.tab_preset, text="", font=(UI_FONT, 9))
        self.preset_status.pack(anchor='w', padx=6)
        self.refresh_preset_tab()

    def preset_tab_dof(self):
        return FF_DOF_MAP[max(0, self.preset_dof.current())]

    def refresh_preset_tab(self):
        if not hasattr(self, 'preset_cells'):
            return
        d = self.preset_tab_dof()
        n = FF_INDEX[d]
        m = self.models[d]
        for k, cells in enumerate(self.preset_cells):
            p = self.settings['presets'][k][n]
            if k < 3:
                for f in ('enabled', 'T', 't1', 'y1', 't2', 'y2'):
                    if p is None:
                        text = "未登録" if f == 'enabled' else ""
                    elif f == 'enabled':
                        text = "する" if p['enabled'] else "しない（FFなし）"
                    else:
                        text = f"{p[f]:.4g}" if p['enabled'] or f == 'T' else "0"
                    cells[f].config(text=text)
            else:
                self.p4_enabled.set(bool(p and p['enabled']))
                for f in ('T', 't1', 'y1', 't2', 'y2'):
                    cells[f].delete(0, tk.END)
                    if p is not None:
                        cells[f].insert(0, f"{p[f]:.6g}")
                    elif f == 'T':
                        cells[f].insert(0, f"{m.T:g}")
            self.draw_preset(cells, p, m)
        self.preset_status.config(text="")

    def draw_preset(self, cells, p, m):
        c = cells['wave']
        c.delete('all')
        if p is None:
            cells['judge'].config(text="未登録", fg='#606060')
            cells['src'].config(text="")
            return
        cells['src'].config(text=p.get('src', 'ユーザー設定'))
        if not p['enabled']:
            cells['judge'].config(text="FFなし", fg='#404040')
            return
        chk = check_ff(p['T'], p['t1'], p['y1'], p['t2'], p['y2'], m.ylo, m.yhi)
        cells['judge'].config(text="○ 条件を満たす" if chk.ok else "× " + " / ".join(chk.errors),
                              fg='#006000' if chk.ok else '#c00000')
        if chk.coeffs is None:
            return
        w, h = 200, 90
        tt = np.linspace(0, p['T'], 120)
        f = ff_poly(chk.coeffs, tt)
        lo, hi = min(f.min(), m.ylo), max(f.max(), m.yhi)
        X = lambda v: 4 + (w - 8) * np.asarray(v) / p['T']
        Y = lambda v: 4 + (h - 8) * (1 - (np.asarray(v) - lo) / (hi - lo))
        c.create_line(4, float(Y(0)), w - 4, float(Y(0)), fill='#b0b0b0')
        for v in (m.ylo, m.yhi):
            c.create_line(4, float(Y(v)), w - 4, float(Y(v)), fill='#d07000', dash=(3, 3))
        c.create_line(*np.column_stack((X(tt), Y(f))).ravel().tolist(), fill='#1050d0' if chk.ok else '#d02020', width=2)

    def p4_read(self):
        cells = self.preset_cells[3]
        try:
            vals = {f: float(cells[f].get()) for f in ('T', 't1', 'y1', 't2', 'y2')}
        except ValueError:
            return None
        vals['enabled'] = bool(self.p4_enabled.get())
        return vals

    def p4_from_current(self):
        m = self.models[self.preset_tab_dof()]
        cells = self.preset_cells[3]
        self.p4_enabled.set(m.enabled)
        for f in ('T', 't1', 'y1', 't2', 'y2'):
            cells[f].delete(0, tk.END)
            cells[f].insert(0, f"{getattr(m, f):.6g}")
        self.preset_status.config(text="現在のFF設定を入れました（まだ保存していません）", fg='#a05000')

    def p4_save(self):
        d = self.preset_tab_dof()
        m = self.models[d]
        p = self.p4_read()
        if p is None:
            messagebox.showerror(TITLE_PRESET, "数値を入力してください")
            return
        if not FF_T_RANGE[0] <= p['T'] <= FF_T_RANGE[1]:
            messagebox.showerror(TITLE_PRESET, f"T は {FF_T_RANGE[0]:g}〜{FF_T_RANGE[1]:g} s にしてください")
            return
        if p['enabled']:
            chk = check_ff(p['T'], p['t1'], p['y1'], p['t2'], p['y2'], m.ylo, m.yhi)
            if not chk.ok:
                messagebox.showerror(TITLE_PRESET, "条件を満たしていないため保存しません:\n" + "\n".join(chk.errors))
                return
        else:
            p.update(t1=0.0, y1=0.0, t2=0.0, y2=0.0)
        p['src'] = f"ユーザー設定 {datetime.now().strftime('%Y-%m-%d %H:%M')}"
        self.settings['presets'][3][FF_INDEX[d]] = p
        self.save_prefs_later(0)
        self.refresh_preset_tab()
        self.preset_status.config(text=f"DOF {d} のプリセット4を保存しました", fg='#006000')

    # =========================================================
    # 設定の保存・終了
    # =========================================================
    def save_prefs_later(self, delay_ms=1000):
        if self.prefs_job is not None:
            self.root.after_cancel(self.prefs_job)
        self.prefs_job = self.root.after(delay_ms, self.save_prefs)

    def save_prefs(self):
        self.prefs_job = None
        try:
            save_settings(self.settings)
        except OSError as e:
            print(f"{SETTINGS_FILE} に保存できません: {e}", file=sys.stderr)

    def on_close(self):
        s = self.node.session
        if s is not None and s.state == Recorder.RECORDING:
            if not messagebox.askokcancel(TITLE_RECORD, "記録中です。この記録を破棄して終了しますか？"):
                return
            s.cancel()
        if self.eval_thread is not None:
            # 評価中の試行は、評価が終わるのを待って now・last・best に反映してから終える
            self.eval_thread.join(timeout=60)
            self.poll_eval()
        if self.prefs_job is not None:
            self.root.after_cancel(self.prefs_job)
        self.save_prefs()
        self.root.destroy()

    def run(self):
        self.root.mainloop()
        for job in self.saves:
            print(f"保存の完了を待っています: {job['folder']}", file=sys.stderr)
            job['proc'].join()


def main():
    rclpy.init()
    settings, warning = load_settings()
    node = FFSettingNode()
    spin_thread = threading.Thread(target=gat.spin_ros, args=(node,), daemon=True)
    spin_thread.start()
    app = FFSettingApp(node, settings, warning)
    app.run()
    # 受信を止めてから終了する（受信コールバックの途中でPythonが終了すると異常終了することがあるため）
    rclpy.try_shutdown()
    spin_thread.join(timeout=2)
    node.destroy_node()


if __name__ == '__main__':
    main()
