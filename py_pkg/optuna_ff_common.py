#!/usr/bin/env python3
"""Optuna による5次関数FFの探索（optimize_optuna_ff_ROS2.py）の計算・通信・保存

計画書 第4版（2026-10-23 承認）の実装。EL2（optimize_EL2_common.py ほか）は import せず、必要な関数を移植した。
移植した部分には「# 移植元: <ファイル>:<行> @f8fba60」を付け、変えた箇所には「# 変更:」で理由を書いた。
EL2 との出力の一致は test/test_optuna_ff_port.py で確かめる。

■ 時間軸（100 Hz 配信。制御周期は使わない）
  実機は rclc タイマで 10 ms ごとに tk・PWM・z3・z1・derivatives・z2 を続けて配信する（.ino:47, 794-833）。
  受信した /board{k}_tk/pub に受信順で配信番号 j を振り（局所の受信間隔の中央値の 1.5 倍を超える間隔で、後の受信が
  追いつかなかった分だけ欠損として番号を飛ばす。assign_ticks）、
  目標値のエコーが新しい値になった配信を j0 として t_j = (j − j0)·0.010 [s] とする。
  配信値は直近に終わった制御周回の値（0〜11 ms 古い）。PWM・z1・z2・z3 は POT よりさらに1周回古い。
■ FF（案4。全候補で「端点0・(0,T) の極値ちょうど2個・|FF| ≤ A_max」を保証する写像）
  u = (u1, u2, u3, A) → 極値の位置 s1, s2（可行域の表）・高さの比 r = −y2/y1（閉形式の区間）・符号付き振幅 A
  → 極値 (s1·T, y1), (s2·T, y2) → f(t) = a t⁵ + b t⁴ + c t³ + d t² + e t（f(0) = f(T) = 0）
  ファーム（.ino:1207-1246）は t = (millis() − t_start)/1000 [s] で、t ≥ T では 0。
■ 目標軌道（study の前に1回だけ作って固定する。数列も全試行で同一）
  ζ=1 の3次遅れ (T1 s+1)(s+ωn)² の自由応答 y0·φ(t) を、「T までに |y| ≤ ε|ΔP| に入り以後も維持」
  （φ は単調なので φ(T) ≤ ε|ΔP|/|y0| と同値）の拘束付きで基準試行（FF=0）に当てはめる。
■ 失敗
  評価に失敗した試行は TrialState.FAIL（値なし）。原因・段階・ファイル名は tell の前に user_attrs に書く。
"""
import csv
import hashlib
import itertools
import json
import math
import os
import threading
import time
import warnings
from collections import deque
from dataclasses import dataclass, field

import numpy as np
import scipy.optimize
from scipy.linalg import expm
from scipy.signal import cont2discrete


# ==============================================================================
# 移植：自由度・トピック・定数・目標軌道・5次関数（optimize_EL2_common.py）
# ==============================================================================
# 移植元: optimize_EL2_common.py:56-82 @f8fba60
N_DOF = 26
BOARD_LAYOUT = [            # (board名, 先頭DOF, DOF数)。ボード番号は並び順（board1 = 1）
    ('board1', 0, 6),
    ('board2', 6, 6),
    ('board3', 12, 6),
    ('board4', 18, 4),
    ('board5', 22, 4),
]
FF_BOARD_DOFS = {'board1': 6, 'board2': 6, 'board3': 6, 'board4': 3, 'board5': 3}


def build_ff_dof_map():
    """FF番号 n（0〜23）→ DOF番号。board1-3 → 0..17 / board4 → 18,19,20 / board5 → 22,23,24"""
    out = []
    for board, offset, _count in BOARD_LAYOUT:
        out.extend(range(offset, offset + FF_BOARD_DOFS[board]))
    return out


FF_DOF_MAP = build_ff_dof_map()                         # FF番号 → DOF
FF_INDEX = {d: n for n, d in enumerate(FF_DOF_MAP)}     # DOF → FF番号（FFのないDOF 21・25 は含まない）
N_FF = len(FF_DOF_MAP)                                  # 24
NO_FF_DOFS = (21, 25)                                   # FFの枠がなく、現在は制御もしていないDOF（目標値は初期値に固定する）
DOF_BOARD = {}                                          # DOF → (ボード番号 1〜5, board名, ボード内の番号)
for _k, (_b, _o, _c) in enumerate(BOARD_LAYOUT, start=1):
    for _j in range(_c):
        DOF_BOARD[_o + _j] = (_k, _b, _j)


# 移植元: optimize_EL2_common.py:84-94 @f8fba60
TOPIC_TARGET = '/board_android_float/sub'       # Pub: 目標値（Float32MultiArray, 26要素）
TOPIC_FF = '/board{}_FFparam_float/sub'         # Pub: FF（Float32MultiArray, 36要素 = 6スロット × [a,b,c,d,e,T]）
TOPIC_POT = '/board{}_tk/pub'                   # Sub: UInt16MultiArray, [0..5] = POT（周回 k）, [6..11] = 目標値（周回 k）
TOPIC_PWM = '/board{}_tk_PWM_float/pub'         # Sub: u_ADRC + FF（周回 k−1）
TOPIC_Z3 = '/board{}_tk_z3_float/pub'           # Sub: z3（周回 k−1 の ESO 更新後）
TOPIC_Z1 = '/board{}_tk_z1_float/pub'           # Sub: z1（周回 k−1 の ESO 更新後）
TOPIC_Z2 = '/board{}_tk_z2_float/pub'           # Sub: z2（周回 k−1 の ESO 更新後）
TOPIC_DER = '/board{}_tk_derivatives_float/pub'  # Sub: derivatives（周回 k）
TOPIC_PARAM = '/board{}_tk_ADRCparam_float/pub'  # Sub: ADRCパラメータの現在値（50要素, [2+4i+p]）
SENSOR_QOS_DEPTH = 100
KINDS = ('pot', 'pwm', 'z3', 'z1', 'z2', 'der')     # 受信する時系列の種類（記録・復元の単位）


# 移植元: optimize_EL2_common.py:104-106 @f8fba60
PUBLISH_DT = 0.010          # ROS2 の配信周期 [s]（周回番号の復元で欠落を数えるときの目安にだけ使う）
PWM_LIMIT = 255.0           # 弁へ出る入力の上限（128 ± u/2 を [0,255] でクリップ → 実効 ±255）
EVAL_FACTOR = 1.5           # 測定・評価区間 = EVAL_FACTOR × T


# 移植元: optimize_EL2_common.py:112-115 @f8fba60（目標軌道の上下限 tgt_* だけを使う）
DEFAULT_ID_BOUNDS = {
    'sys_T1': (2e-3, 20.0), 'sys_zeta': (-0.4, 5.0), 'sys_wn': (0.5, 50.0), 'sys_b0': (-1e5, 1e5),
    'tgt_T1': (1e-3, 10.0), 'tgt_wn': (0.1, 100.0),
}


# 移植元: optimize_EL2_common.py:119-119 @f8fba60
STEP_WARN = 10.0            # |ΔP| がこれ未満なら同定が不安定になりうると警告する [count]


# 移植元: optimize_EL2_common.py:125-127 @f8fba60（L に PUBLISH_DT を渡して評価区間の配信数 N_e を求める）
def eval_steps(T, L):
    """評価区間の最終ステップ N = max{k : k·L ≤ 1.5T}。測定・評価は k = 0..N の N+1 点"""
    return int(np.floor(EVAL_FACTOR * float(T) / float(L) + 1e-9))


# 移植元: optimize_EL2_common.py:138-153 @f8fba60
@dataclass
class AdrcParams:
    kp: float
    kd: float
    ic: float
    lam: float

    def valid(self):
        return all(np.isfinite([self.kp, self.kd, self.ic, self.lam])) and self.ic > 0.0 and self.lam > 0.0

    def betas(self):
        """実機 computeBeta() と同じ三重極配置 β1 = 3λ, β2 = 3λ², β3 = λ³"""
        return 3.0 * self.lam, 3.0 * self.lam ** 2, self.lam ** 3

    def as_list(self):
        return [self.kp, self.kd, self.ic, self.lam]


# 移植元: optimize_EL2_common.py:181-194 @f8fba60（目標軌道の離散化に使う）
def system_coeffs(T1, zeta, wn):
    """(T1 s + 1)(s² + 2ζωn s + ωn²) を s³ + a2 s² + a1 s + a0 に展開して返す"""
    a2 = (2.0 * zeta * wn * T1 + 1.0) / T1
    a1 = (wn ** 2 * T1 + 2.0 * zeta * wn) / T1
    a0 = wn ** 2 / T1
    return a2, a1, a0


def discretize(a2, a1, a0, b0, L):
    """可制御正準形 ẋ = A x + B u（x = [位置, 速度, 加速度]）を周回間隔 L で ZOH 離散化する（PWM は1周保持されるため）"""
    A = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [-a0, -a1, -a2]])
    B = np.array([[0.0], [0.0], [float(b0)]])
    Ad, Bd, _, _, _ = cont2discrete((A, B, np.zeros((1, 3)), np.zeros((1, 1))), float(L), method='zoh')
    return Ad, Bd.ravel()


# 移植元: optimize_EL2_common.py:295-337 @f8fba60
def target_coeffs(T1, wn):
    """(T1 s + 1)(s + ωn)² の係数（system_coeffs の ζ=1）"""
    return system_coeffs(T1, 1.0, wn)


def target_states(T1, wn, y0, L, n):
    """y 系（目標値が原点）での自由応答の状態列 (n+1, 3) = [位置, 速度, 加速度]。初期状態 [y0, 0, 0]"""
    Ad, _ = discretize(*target_coeffs(T1, wn), 0.0, L)
    X = np.zeros((n + 1, 3))
    X[0] = [y0, 0.0, 0.0]
    for k in range(n):
        X[k + 1] = Ad @ X[k]
    return X


def fit_target(y_meas, valid, y0, L, bounds, prev=None):
    """実測 y = P − Pf（k = 0..N）に ζ=1 の3次遅れ系の自由応答を最小二乗で当てはめる。戻り値 ((T1, wn), rms)"""
    y_meas = np.asarray(y_meas, dtype=float)
    n = len(y_meas) - 1
    m = np.asarray(valid, dtype=bool) & np.isfinite(y_meas)
    lo = [np.log(bounds['tgt_T1'][0]), np.log(bounds['tgt_wn'][0])]
    hi = [np.log(bounds['tgt_T1'][1]), np.log(bounds['tgt_wn'][1])]

    def res(q):
        with np.errstate(all='ignore'):
            X = target_states(np.exp(q[0]), np.exp(q[1]), y0, L, n)
            r = (X[:, 0] - y_meas)[m]
        return np.nan_to_num(r, nan=1e6, posinf=1e6, neginf=-1e6)

    starts = [[np.log(a), np.log(b)] for a in (0.03, 0.15, 0.5) for b in (3.0, 8.0, 20.0)]
    if prev is not None and np.all(np.isfinite(prev)) and min(prev) > 0:
        starts.insert(0, [np.log(prev[0]), np.log(prev[1])])
    best = None
    for q0 in starts:
        try:
            r = scipy.optimize.least_squares(res, np.clip(q0, lo, hi), bounds=(lo, hi), method='trf', x_scale='jac')
        except Exception:
            continue
        if best is None or r.cost < best.cost:
            best = r
    T1, wn = float(np.exp(best.x[0])), float(np.exp(best.x[1]))
    rms = float(np.sqrt(np.mean(res(best.x) ** 2))) if m.any() else float('nan')
    return (T1, wn), rms


# 移植元: optimize_EL2_common.py:350-363 @f8fba60
def normalized_error(y_meas, y_tgt, dP, valid):
    """J_i = (1/N_valid) Σ_k (POT − Target)² / ΔP²（T・周回間隔・欠測点数に依存しない無次元量）"""
    if dP == 0:
        return float('nan')
    m = np.asarray(valid, dtype=bool) & np.isfinite(y_meas) & np.isfinite(y_tgt)
    if not m.any():
        return float('nan')
    d = (np.asarray(y_meas)[m] - np.asarray(y_tgt)[m]) / float(dP)
    return float(np.mean(d * d))


def rms_percent(J_i):
    """J_i を ΔP に対する RMS 比 [%] にした表示用の値"""
    return float(np.sqrt(J_i) * 100.0) if np.isfinite(J_i) and J_i >= 0 else float('nan')


# 移植元: optimize_EL2_common.py:372-528 @f8fba60
def _rf(t):
    return np.array([t ** 5, t ** 4, t ** 3, t ** 2, t])


def _rfp(t):
    return np.array([5 * t ** 4, 4 * t ** 3, 3 * t ** 2, 2 * t, 1.0])


def _rfpp(t):
    return np.array([20 * t ** 3, 12 * t ** 2, 6 * t, 2.0, 0.0])


def quintic_matrix(T, t1, t2):
    return np.vstack([_rf(T), _rfp(t1), _rfp(t2), _rf(t1), _rf(t2)])


def quintic_from_extrema(T, t1, t2, y1, y2):
    """極値 (t1,y1),(t2,y2) から [a, b, c, d, e] を求める"""
    return np.linalg.solve(quintic_matrix(T, t1, t2), np.array([0.0, 0.0, 0.0, float(y1), float(y2)]))


def coeff_e(a, b, c, d, T):
    return -(a * T ** 4 + b * T ** 3 + c * T ** 2 + d * T)


def ff_eval(coeffs, T, t):
    """FF(t)。0 ≤ t < T で5次関数、それ以外は 0（ファームの AddInput と同じ）"""
    t = np.asarray(t, dtype=float)
    v = np.polyval(np.append(np.asarray(coeffs, dtype=float), 0.0), t)
    return np.where((t >= 0.0) & (t < T), v, 0.0)


def ff_samples(coeffs, T, L, n):
    """周回 k = 0..n−1 の FF（t = k·L）"""
    return ff_eval(coeffs, T, np.arange(n) * L)


def interior_critical_points(coeffs, T, imag_tol=1e-7):
    """f'(t) = 0 の実根のうち 0 < t < T にあるもの（昇順）"""
    a, b, c, d, e = [float(v) for v in coeffs]
    if not np.all(np.isfinite([a, b, c, d, e])):
        return None
    scale = max(abs(a) * T ** 4, abs(b) * T ** 3, abs(c) * T ** 2, abs(d) * T, abs(e), 1e-300)
    roots = np.roots(np.array([5 * a * T ** 4, 4 * b * T ** 3, 3 * c * T ** 2, 2 * d * T, e]) / scale)  # τ = t/T で解く
    out = sorted(float(r.real) * T for r in roots if abs(r.imag) <= imag_tol and 0.0 < r.real < 1.0)
    return out


def extrema_ok(coeffs, T, simple_tol=1e-6):
    """(0,T) 内の極値がちょうど2個で、どちらも単純根（重根でない）なら (t1, t2) を、そうでなければ None を返す"""
    cps = interior_critical_points(coeffs, T)
    if cps is None or len(cps) != 2:
        return None
    a, b, c, d, _ = [float(v) for v in coeffs]
    amp = max(abs(ff_eval(coeffs, T, cps[0])), abs(ff_eval(coeffs, T, cps[1])), 1e-12)
    for t in cps:
        fpp = 20 * a * t ** 3 + 12 * b * t ** 2 + 6 * c * t + 2 * d
        if abs(fpp) * T ** 2 <= simple_tol * amp:
            return None
    return cps[0], cps[1]


def extrema_of(coeffs, T):
    """CSV・表示用の極値 (t1, y1, t2, y2)。f≡0 は既存の calc_extrema_from_ff と同じ (0.33T, 0, 0.66T, 0)"""
    if not np.any(np.abs(np.asarray(coeffs, dtype=float)) > 0.0):
        return (T * 0.33, 0.0, T * 0.66, 0.0)
    cps = interior_critical_points(coeffs, T) or []
    if len(cps) >= 2:
        t1, t2 = cps[:2]
    elif len(cps) == 1:
        t1, t2 = cps[0], T / 2
    else:
        t1, t2 = T * 0.33, T * 0.66
    return (t1, float(ff_eval(coeffs, T, t1)), t2, float(ff_eval(coeffs, T, t2)))


def count_interior_batch(P, T):
    """係数 [a..e] を行に並べた配列 (R,5) について、(0,T) 内の f' の実根の個数を数える（τ = t/T で計算）"""
    P = np.asarray(P, dtype=float)
    D = P * np.array([5.0 * T ** 4, 4.0 * T ** 3, 3.0 * T ** 2, 2.0 * T, 1.0])     # f'(τT) の τ 多項式の係数
    out = np.zeros(len(D), dtype=int)
    lead = D[:, 0]
    big = np.abs(D).max(axis=1)
    ok = np.isfinite(D).all(axis=1) & (np.abs(lead) > 1e-10 * big)
    if ok.any():
        Dn = D[ok] / lead[ok, None]
        C = np.zeros((int(ok.sum()), 4, 4))
        C[:, 0, :] = -Dn[:, 1:]
        C[:, 1, 0] = C[:, 2, 1] = C[:, 3, 2] = 1.0
        ev = np.linalg.eigvals(C)
        out[ok] = ((np.abs(ev.imag) <= 1e-7) & (ev.real > 0.0) & (ev.real < 1.0)).sum(axis=1)
    for i in np.flatnonzero(~ok):
        cps = interior_critical_points(P[i], T)
        out[i] = len(cps) if cps is not None else 0
    return out


def feasible(T, p, t_margin=0.01, sep_min=0.02, y_min=1e-9):
    """極値座標 p = (t1, t2, y1, y2) が「(0,T) 内の極値ちょうど2個（t1, t2 に一致）」を満たすか"""
    t1, t2, y1, y2 = [float(v) for v in p]
    if not (t_margin * T < t1 < t2 < (1.0 - t_margin) * T) or (t2 - t1) < sep_min * T:
        return False
    if not (np.isfinite(y1) and np.isfinite(y2)) or y1 * y2 >= 0.0 or min(abs(y1), abs(y2)) < y_min:
        return False
    try:
        co = quintic_from_extrema(T, t1, t2, y1, y2)
    except np.linalg.LinAlgError:
        return False
    ext = extrema_ok(co, T)
    return ext is not None and abs(ext[0] - t1) < 1e-5 * T and abs(ext[1] - t2) < 1e-5 * T


def ratio_intervals(T, t1, t2, r_grid=None, bisect=22, r_hint=None):
    """(t1,t2) を固定したとき、極値がちょうど2個になる高さの比 r = −y2/y1 の区間のリスト [(lo, hi), ...]

    f は (y1, y2) について1次同次なので、実行可能かどうかは比 r だけで決まる（y1 の符号にもよらない）。
    区間が格子の間隔より狭いと見落とすので、今の比 r_hint があれば格子に加える。
    """
    if r_grid is None:
        r_grid = np.logspace(-2.5, 2.5, 121)
    if r_hint is not None and np.isfinite(r_hint) and r_hint > 0:
        r_grid = np.unique(np.concatenate([r_grid, [r_hint * (1 - 1e-6), r_hint, r_hint * (1 + 1e-6)]]))
    try:
        g1 = quintic_from_extrema(T, t1, t2, 1.0, 0.0)
        g2 = quintic_from_extrema(T, t1, t2, 0.0, 1.0)
    except np.linalg.LinAlgError:
        return []
    ok = count_interior_batch(g1[None, :] - r_grid[:, None] * g2[None, :], T) == 2

    def is_ok(r):
        return count_interior_batch((g1 - r * g2)[None, :], T)[0] == 2

    out = []
    i = 0
    while i < len(r_grid):
        if not ok[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(r_grid) and ok[j + 1]:
            j += 1
        lo, hi = np.log(r_grid[i]), np.log(r_grid[j])
        if i > 0:                                       # 下端を二分法で詰める
            a, b = np.log(r_grid[i - 1]), lo
            for _ in range(bisect):
                m = 0.5 * (a + b)
                a, b = (a, m) if is_ok(np.exp(m)) else (m, b)
            lo = b
        if j + 1 < len(r_grid):                         # 上端を二分法で詰める
            a, b = hi, np.log(r_grid[j + 1])
            for _ in range(bisect):
                m = 0.5 * (a + b)
                a, b = (m, b) if is_ok(np.exp(m)) else (a, m)
            hi = a
        out.append((float(np.exp(lo)), float(np.exp(hi))))
        i = j + 1
    return out


# 移植元: optimize_EL2_common.py:568-586 @f8fba60
def retract(T, p, rel=1e-6):
    """t1, t2, y1 を保ったまま、高さの比 r を (t1,t2) の実行可能区間へ戻す（射影）。戻せなければ None"""
    t1, t2, y1, y2 = [float(v) for v in p]
    if feasible(T, p):
        return np.asarray(p, dtype=float)
    if not (0.0 < t1 < t2 < T) or y1 == 0.0:
        return None
    r = -y2 / y1 if y2 * y1 < 0 else 1e-12
    ivs = ratio_intervals(T, t1, t2, r_hint=r)
    if not ivs:
        return None
    best = None
    for lo, hi in ivs:
        rc = min(max(r, lo * (1 + rel)), hi * (1 - rel))
        dist = abs(np.log(rc) - np.log(r))
        if best is None or dist < best[0]:
            best = (dist, rc)
    q = np.array([t1, t2, y1, -best[1] * y1])
    return q if feasible(T, q) else None

# ==============================================================================
# 移植：GUI_android_tk.py の定数（関節の可動範囲の既定値・初期姿勢・プリセットの読み込み）
# ==============================================================================
# 移植元: GUI_android_tk.py:132-147 @f8fba60
POT_RANGE = [
    (185, 700), (135, 550), (130, 680), (10, 734), (66, 259), (192, 389),
    (70, 600), (60, 465), (115, 619), (22, 794), (239, 430), (205, 395),
    (30, 660), (30, 690), (110, 830), (3, 630), (3, 700),(9, 660),
    (275, 360), (115, 785), (192, 440), (284, 557),
    (323, 580), (188, 630), (375, 500), (300, 490),
]

# Initial desired values（設定ファイルが無いときの初期姿勢）
INITIAL_DESIRED = [
    500, 200, 500, 300, 170, 300,
    160, 410, 200, 500, 350, 220,
    300, 200, 400, 350, 420, 400,
    325, 370, 280, 420,
    360, 390, 420, 390
]


# 移植元: GUI_android_tk.py:196-197 @f8fba60
SETTINGS_FILE = os.path.expanduser('~/.armrobot_gui_android_tk.json')
NUM_PRESETS = 5


# 移植元: GUI_android_tk.py:254-258 @f8fba60
def _is_pose(values):
    """26自由度ぶんの有限な数値のリストか"""
    return (isinstance(values, list) and len(values) == 26
            and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                    for v in values))


def load_gui_poses(path=None):
    """GUI_android_tk.py の設定ファイルから (初期姿勢26個, プリセット NUM_PRESETS 個（未登録は None）) を読む。

    # 移植元: GUI_android_tk.py:261-290 @f8fba60（load_settings の initial_pose・presets の部分だけ。他の項目は使わない）
    """
    path = path or SETTINGS_FILE
    initial = [float(v) for v in INITIAL_DESIRED]
    presets = [None] * NUM_PRESETS
    if not os.path.exists(path):
        return initial, presets
    try:
        with open(path) as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise ValueError
    except (OSError, ValueError):
        return initial, presets
    init = data.get('initial_pose')
    if _is_pose(init):
        initial = [float(v) for v in init]
    raw = data.get('presets')
    raw = raw if isinstance(raw, list) else []
    for k in range(min(NUM_PRESETS, len(raw))):
        if _is_pose(raw[k]):
            presets[k] = [float(v) for v in raw[k]]
    return initial, presets


# ==============================================================================
# 新規の定数
# ==============================================================================
TICK_DT = PUBLISH_DT                # 時間軸の刻み [s]（配信 100 Hz。制御周期は使わない）
HISTORY_S = 1.2                     # 記録に含める、目標値を切り替える前の受信 [s]
REST_WINDOW_S = 0.5                 # 静止位置 P0 を求める、切り替え直前の区間 [s]
MARGIN_TICKS = 15                   # 評価区間のあとに余分に受信する配信数
STUDY_NAME = 'optuna_ff'
MAP_VERSION = 'case4-v2-exact'      # 写像の版（条件 hash に入る）
R_MIN, R_MAX = 1e-3, 1e3            # 高さの比 r = −y2/y1 の探索範囲
A_MIN = 1.0                         # |A| の下限 [PWM]（A = 0 は極値が0個になるので、|A| < 1 は ±1 に寄せる）
XI_CAP = math.log(10.0)             # 目標軌道の完了を T/10 より早くはしない（ξ ≤ ln 10）
WN_MAX = 100.0                      # 目標軌道の ωn の上限 [rad/s]（DEFAULT_ID_BOUNDS の tgt_wn と同じ）
FAIL_CLASSES = ('presend', 'tabu', 'unsafe', 'comm', 'sensor', 'eval', 'stopped', 'orphan', 'error')
RETRY_CLASSES = ('comm', 'eval', 'stopped', 'orphan')      # 同じ u を1回だけ再投入する失敗
NO_ROBOT_CLASSES = ('presend', 'tabu')                      # 実機を動かさない失敗


EVAL_MIN_S = 5.0            # 評価区間の最短 [s]（2026-10-23 ユーザー指示: 1.5T が5秒に満たなければ5秒評価する）


def eval_time(T):
    """測定・評価区間の長さ [s] = max(1.5T, 5 s)。1.5T が5秒未満なら5秒、5秒を超えるなら 1.5T"""
    return max(EVAL_FACTOR * float(T), EVAL_MIN_S)


def n_eval(T):
    """評価区間の最終の配信番号 N_e = ⌊max(1.5T, 5 s)/0.010⌋（j − j0 = 0..N_e の N_e+1 点）"""
    return int(np.floor(eval_time(T) / TICK_DT + 1e-9))


# ==============================================================================
# FF の写像（案4）
# ==============================================================================
def _basis_s(s1, s2):
    """s 領域（T = 1）で、極値 (s1, 1),(s2, 0) と (s1, 0),(s2, 1) の5次関数の係数 g1, g2（[..., 5] = [a..e]）。

    quintic_matrix(1, s1, s2) と同じ行（f(1)=0, f'(s1)=0, f'(s2)=0, f(s1)=y1, f(s2)=y2）を配列のまま解く。
    """
    s1b, s2b = np.broadcast_arrays(np.asarray(s1, dtype=float), np.asarray(s2, dtype=float))
    M = np.zeros(s1b.shape + (5, 5))
    M[..., 0, :] = 1.0
    for k, p in enumerate((5, 4, 3, 2, 1)):
        M[..., 1, k] = p * s1b ** (p - 1)
        M[..., 2, k] = p * s2b ** (p - 1)
        M[..., 3, k] = s1b ** p
        M[..., 4, k] = s2b ** p
    rhs = np.zeros(s1b.shape + (5, 2))
    rhs[..., 3, 0] = 1.0
    rhs[..., 4, 1] = 1.0
    sol = np.linalg.solve(M, rhs)
    return sol[..., 0], sol[..., 1]


def _q_bernstein(c5, s1, s2):
    """FF'(s) = (s − s1)(s − s2)·Q(s) としたときの2次式 Q のベルンシュタイン係数 (α, β, γ) = (Q(0), Q(0)+Q'(0)/2, Q(1))。

    FF' = 5a s⁴ + 4b s³ + 3c s² + 2d s + e と (s² − σ s + π)(q2 s² + q1 s + q0) の係数を比べて
    q2 = 5a, q1 = 4b + σ q2, q0 = 3c + σ q1 − π q2（σ = s1 + s2, π = s1·s2）。
    """
    a, b, c = c5[..., 0], c5[..., 1], c5[..., 2]
    sig, pi_ = s1 + s2, s1 * s2
    q2 = 5.0 * a
    q1 = 4.0 * b + sig * q2
    q0 = 3.0 * c + sig * q1 - pi_ * q2
    return q0, q0 + 0.5 * q1, q0 + q1 + q2


def bernstein_positive(alpha, beta, gamma):
    """2次式のベルンシュタイン係数から「[0,1] で Q > 0」を判定する（α>0, γ>0, β>−√(αγ)）"""
    alpha, beta, gamma = (np.asarray(v, dtype=float) for v in (alpha, beta, gamma))
    with np.errstate(invalid='ignore'):
        return (alpha > 0) & (gamma > 0) & ((beta >= 0) | (beta * beta < alpha * gamma))


def r_intervals(s1, s2, r_min=R_MIN, r_max=R_MAX):
    """(s1, s2) ごとに、(0,1) の極値がちょうど2個（どちらも単純根）になる比 r = −y2/y1 の範囲を [r_min, r_max] で求める。

    y1 = 1, y2 = −r とすると FF = g1 − r·g2 で、Q̃ = sgn(y1)·Q = Q のベルンシュタイン係数 α, β, γ は r の1次式になる。
    条件 α>0, γ>0, β>−√(αγ)（Q̃ > 0 on [0,1] と同値）の境目は α・β・γ の零点と β² − αγ（2次式）の零点だけなので、
    それらで [r_min, r_max] を区切り、各小区間の中点で条件を調べる（閉形式。格子や二分法は使わない）。
    戻り値 (lo, hi, contiguous)。条件を満たす r がなければ lo = hi = nan。contiguous は満たす小区間が連続しているか。
    """
    s1b, s2b = np.broadcast_arrays(np.asarray(s1, dtype=float), np.asarray(s2, dtype=float))
    g1, g2 = _basis_s(s1b, s2b)
    A1, B1, C1 = _q_bernstein(g1, s1b, s2b)
    A2, B2, C2 = _q_bernstein(g2, s1b, s2b)
    pa, qa, pb, qb, pc, qc = A1, -A2, B1, -B2, C1, -C2          # α(r) = pa + qa·r など

    def lin_root(p, q):
        with np.errstate(divide='ignore', invalid='ignore'):
            return np.where(q != 0.0, -p / q, np.nan)

    Aq = qb * qb - qa * qc                                       # β² − αγ = Aq r² + Bq r + Cq
    Bq = 2.0 * pb * qb - (pa * qc + qa * pc)
    Cq = pb * pb - pa * pc
    with np.errstate(divide='ignore', invalid='ignore'):
        disc = Bq * Bq - 4.0 * Aq * Cq
        sq = np.sqrt(np.where(disc >= 0.0, disc, np.nan))
        h1 = np.where(Aq != 0.0, (-Bq - sq) / (2.0 * Aq), np.where(Bq != 0.0, -Cq / Bq, np.nan))
        h2 = np.where(Aq != 0.0, (-Bq + sq) / (2.0 * Aq), np.nan)
    shape = s1b.shape
    cand = np.stack([np.full(shape, r_min), np.full(shape, r_max), lin_root(pa, qa), lin_root(pb, qb),
                     lin_root(pc, qc), h1, h2], axis=-1)
    cand = np.where(np.isfinite(cand) & (cand > r_min) & (cand < r_max), cand, r_max)
    cand[..., 0] = r_min
    cand = np.sort(cand, axis=-1)
    left, right = cand[..., :-1], cand[..., 1:]
    valid = right > left * (1.0 + 1e-12)
    mid = np.sqrt(left * right)
    al = pa[..., None] + qa[..., None] * mid
    be = pb[..., None] + qb[..., None] * mid
    ga = pc[..., None] + qc[..., None] * mid
    ok = valid & bernstein_positive(al, be, ga)
    n = ok.shape[-1]
    has = ok.any(axis=-1)
    first = np.argmax(ok, axis=-1)
    last = n - 1 - np.argmax(ok[..., ::-1], axis=-1)
    idx = np.arange(n)
    inside = (idx >= first[..., None]) & (idx <= last[..., None])
    contiguous = ~(inside & valid & ~ok).any(axis=-1) | ~has
    lo = np.take_along_axis(left, first[..., None], axis=-1)[..., 0]
    hi = np.take_along_axis(right, last[..., None], axis=-1)[..., 0]
    return np.where(has, lo, np.nan), np.where(has, hi, np.nan), contiguous


def r_interval_margin(s1, s2, margin=1.01):
    """余裕（比で margin）を取った r の区間 (lo·margin, hi/margin)。区間が空、または1つにまとまっていなければ (nan, nan)"""
    lo, hi, cont = r_intervals(s1, s2)
    lo2, hi2 = lo * margin, hi / margin
    ok = np.isfinite(lo) & cont & (hi2 > lo2 * (1.0 + 1e-12))
    return np.where(ok, lo2, np.nan), np.where(ok, hi2, np.nan)


class MapError(Exception):
    pass


class FFMapTable:
    """(s1, s2) の可行域。s1 ごとに「1% の余裕を取った r の区間が空でない s2」の範囲を扱う（計画書 §2.3、2026-10-23 案4）。

    表の作り方: s1 を n_s1 点、s2 を [s1+sep, 1−m] の n_s2 点に取り、可行な s2 が1区間であることを検査する（そうでなければ例外）。
    境界は二分法で詰めて保存する。表は「s1 の範囲 [s1min, s1max]」と「境界を求める二分法の初期値」にだけ使う。
    境界 lo(s1), hi(s1) は補間せず、その s1 で毎回二分法により正確に求めてから、内側へ shrink（1e−4）だけ寄せる
    （補間すると s1 ≈ 0.1555 の境界の折れ目で 4e−4 ほど外へ出るため。2026-10-23 にユーザーが案4を選んだ）。
    """

    def __init__(self, m=0.02, sep=0.02, r_margin=1.01, n_s1=2001, n_s2=4001, shrink=1e-4, log=None, build=True):
        self.params = {'m': float(m), 'sep': float(sep), 'r_margin': float(r_margin), 'n_s1': int(n_s1),
                       'n_s2': int(n_s2), 'shrink': float(shrink), 'r_min': R_MIN, 'r_max': R_MAX,
                       'version': MAP_VERSION}
        self.s1 = self.lo_raw = self.hi_raw = None
        self._cache = {}
        if build:
            self._build(log)

    def key(self):
        return dict(self.params)

    def _feasible(self, s1, s2):
        lo, _hi = r_interval_margin(s1, s2, self.params['r_margin'])
        return np.isfinite(lo)

    def _feasible1(self, s1, s2):
        return bool(self._feasible(np.array([s1]), np.array([s2]))[0])

    def _bisect(self, s1, bad, good, tol=1e-7):
        """bad（不可）と good（可）の間の境界を、可の側で返す"""
        while abs(good - bad) > tol:
            mid = 0.5 * (bad + good)
            if self._feasible1(s1, mid):
                good = mid
            else:
                bad = mid
        return good

    def _build(self, log=None):
        p = self.params
        m, sep, n1, n2, shrink = p['m'], p['sep'], p['n_s1'], p['n_s2'], p['shrink']
        t0 = time.monotonic()
        s1_all = np.linspace(m, 1.0 - m - sep, n1)
        lo = np.full(n1, np.nan)
        hi = np.full(n1, np.nan)
        for i, s1 in enumerate(s1_all):
            s2 = np.linspace(s1 + sep, 1.0 - m, n2)
            rlo, _rhi, cont = r_intervals(np.full(n2, s1), s2)
            if np.isfinite(rlo).any() and not cont[np.isfinite(rlo)].all():
                raise ValueError(f"s1 = {s1:.6f}: r の可行集合が1区間ではない s2 があります")
            feas = self._feasible(np.full(n2, s1), s2)
            if not feas.any():
                continue
            idx = np.flatnonzero(feas)
            if idx[-1] - idx[0] + 1 != len(idx):
                raise ValueError(f"s1 = {s1:.6f}: 可行な s2 の集合が1区間ではありません")
            a = s2[idx[0]] if idx[0] == 0 else self._bisect(s1, s2[idx[0] - 1], s2[idx[0]])
            b = s2[idx[-1]] if idx[-1] == n2 - 1 else self._bisect(s1, s2[idx[-1] + 1], s2[idx[-1]])
            if b - a > 2.0 * shrink:
                lo[i], hi[i] = a, b
        rows = np.flatnonzero(np.isfinite(lo))
        if len(rows) == 0:
            raise ValueError("可行な (s1, s2) がありません")
        if rows[-1] - rows[0] + 1 != len(rows):
            raise ValueError("可行な s1 の集合が1区間ではありません")
        self.s1, self.lo_raw, self.hi_raw = s1_all[rows], lo[rows], hi[rows]
        if log:
            log(f"FF の可行域の表を作りました: s1 ∈ [{self.s1[0]:.4f}, {self.s1[-1]:.4f}]（{len(self.s1)} 点, "
                f"{time.monotonic() - t0:.1f} 秒）")

    @property
    def s1min(self):
        return float(self.s1[0])

    @property
    def s1max(self):
        return float(self.s1[-1])

    def bounds(self, s1):
        """s1 での s2 の範囲 (lo, hi)。その s1 で二分法により正確に求めた境界から、内側へ shrink だけ寄せる"""
        s1 = float(s1)
        if s1 in self._cache:
            return self._cache[s1]
        p = self.params
        a_dom, b_dom = s1 + p['sep'], 1.0 - p['m']
        s2 = np.linspace(a_dom, b_dom, 401)                     # その s1 で可行な s2 が1区間であることを確かめる
        feas = self._feasible(np.full(len(s2), s1), s2)
        idx = np.flatnonzero(feas)
        if len(idx) == 0:
            raise MapError(f"s1 = {s1:.6f} で可行な s2 がありません")
        if idx[-1] - idx[0] + 1 != len(idx):
            raise MapError(f"s1 = {s1:.6f} で可行な s2 の集合が1区間ではありません")
        hint = 0.5 * (s2[idx[0]] + s2[idx[-1]])
        if not self._feasible1(s1, hint):
            hint = s2[idx[len(idx) // 2]]
        lo = a_dom if idx[0] == 0 and self._feasible1(s1, a_dom) else self._bisect(s1, s2[max(idx[0] - 1, 0)], hint)
        hi = b_dom if idx[-1] == len(s2) - 1 and self._feasible1(s1, b_dom) else \
            self._bisect(s1, s2[min(idx[-1] + 1, len(s2) - 1)], hint)
        lo, hi = lo + p['shrink'], hi - p['shrink']
        if not hi > lo:
            raise MapError(f"s1 = {s1:.6f} で余裕を取ると s2 の範囲が空になります")
        if len(self._cache) > 20000:
            self._cache.clear()
        self._cache[s1] = (lo, hi)
        return lo, hi

    def verify(self, n_s1=4001, n_u2=201):
        """[s1min, s1max] × u2 の格子で、写像した (s1, s2) がすべて可行であることを確かめる（単体テスト用）。戻り値 不可の点の数"""
        bad = 0
        for s1 in np.linspace(self.s1min, self.s1max, n_s1):
            lo, hi = self.bounds(s1)
            s2 = lo + np.linspace(0.0, 1.0, n_u2) * (hi - lo)
            bad += int((~self._feasible(np.full(n_u2, s1), s2)).sum())
        return bad

    def save(self, path):
        tmp = path + '.tmp.npz'
        np.savez(tmp, s1=self.s1, lo_raw=self.lo_raw, hi_raw=self.hi_raw, params=json.dumps(self.params))
        os.replace(tmp, path)

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as z:
            t = cls(build=False)
            t.params = json.loads(str(z['params']))
            t.s1, t.lo_raw, t.hi_raw = z['s1'].copy(), z['lo_raw'].copy(), z['hi_raw'].copy()
        return t

    @classmethod
    def cached(cls, m=0.02, sep=0.02, log=None, cache_dir=None):
        """~/.cache/optuna_ff/ に同じ条件の表があれば読み、なければ作って置く"""
        cache_dir = cache_dir or os.path.expanduser('~/.cache/optuna_ff')
        probe = cls(m=m, sep=sep, build=False)
        name = 'ffmap_' + hashlib.sha256(json.dumps(probe.params, sort_keys=True).encode()).hexdigest()[:16] + '.npz'
        path = os.path.join(cache_dir, name)
        if os.path.exists(path):
            try:
                t = cls.load(path)
                if t.params == probe.params:
                    return t
            except (OSError, ValueError, KeyError):
                pass
        if log:
            log("FF の可行域の表を作っています（初回だけ。十数秒かかります）…")
        t = cls(m=m, sep=sep, log=log)
        try:
            os.makedirs(cache_dir, exist_ok=True)
            t.save(path)
        except OSError:
            pass
        return t


def ffmap_table_cached(m, sep):
    """FFMapTable.cached の関数版（別プロセスで呼べるように）"""
    return FFMapTable.cached(m=m, sep=sep)


class FFMap:
    """u = (u1, u2, u3, A) と FF の極値・係数の対応（案4）"""

    def __init__(self, table, A_max):
        self.table = table
        self.A_max = float(A_max)

    def design(self, u1, u2, u3, A, T):
        """u → 極値と係数。戻り値の辞書: s1, s2, r, r_lo, r_hi, y1, y2, A_eff, t1, t2, coeffs（t [s] の係数 [a..e]）"""
        tb = self.table
        s1 = tb.s1min + float(u1) * (tb.s1max - tb.s1min)
        lo, hi = tb.bounds(s1)
        s2 = lo + float(u2) * (hi - lo)
        rl, rh = r_interval_margin(np.array([s1]), np.array([s2]), tb.params['r_margin'])
        rl, rh = float(rl[0]), float(rh[0])
        if not (np.isfinite(rl) and rh > rl):
            raise MapError(f"(s1, s2) = ({s1:.5f}, {s2:.5f}) で r の区間が空です")
        r = math.exp(math.log(rl) + float(u3) * (math.log(rh) - math.log(rl)))
        A = float(A)
        A_eff = math.copysign(max(abs(A), A_MIN), A if A != 0.0 else 1.0)
        y1 = A_eff / max(1.0, r)
        y2 = -r * y1
        cs = quintic_from_extrema(1.0, s1, s2, y1, y2)
        coeffs = cs / np.array([T ** 5, T ** 4, T ** 3, T ** 2, T])
        return {'s1': s1, 's2': s2, 'r': r, 'r_lo': rl, 'r_hi': rh, 'y1': y1, 'y2': y2, 'A_eff': A_eff,
                't1': s1 * T, 't2': s2 * T, 'coeffs': coeffs}

    def inverse(self, t1, y1, t2, y2, T, tol=1e-9):
        """極値 → u。表の範囲の外なら (None, 理由)"""
        tb = self.table
        if not (np.isfinite([t1, y1, t2, y2]).all() and y1 != 0.0 and y1 * y2 < 0.0):
            return None, '極値が異符号の2個ではない'
        s1, s2 = t1 / T, t2 / T
        u1 = (s1 - tb.s1min) / (tb.s1max - tb.s1min)
        if not (-tol <= u1 <= 1.0 + tol):
            return None, f's1 = {s1:.4f} が表の範囲 [{tb.s1min:.4f}, {tb.s1max:.4f}] の外'
        u1 = min(max(u1, 0.0), 1.0)
        lo, hi = tb.bounds(tb.s1min + u1 * (tb.s1max - tb.s1min))
        u2 = (s2 - lo) / (hi - lo)
        if not (-tol <= u2 <= 1.0 + tol):
            return None, f's2 = {s2:.4f} が s1 での範囲 [{lo:.4f}, {hi:.4f}] の外'
        u2 = min(max(u2, 0.0), 1.0)
        rl, rh = r_interval_margin(np.array([s1]), np.array([s2]), tb.params['r_margin'])
        rl, rh = float(rl[0]), float(rh[0])
        r = -y2 / y1
        if not (np.isfinite(rl) and rl * (1 - tol) <= r <= rh * (1 + tol)):
            return None, f'r = {r:.4g} が余裕を取った区間 [{rl:.4g}, {rh:.4g}] の外'
        u3 = min(max((math.log(r) - math.log(rl)) / (math.log(rh) - math.log(rl)), 0.0), 1.0)
        A = math.copysign(max(abs(y1), abs(y2)), y1)
        if abs(A) > self.A_max * (1 + tol):
            return None, f'振幅 {abs(A):.1f} が A_max = {self.A_max:g} を超える'
        return {'u1': u1, 'u2': u2, 'u3': u3, 'A': A}, ''

    def normalized(self, u):
        """タブーの距離に使う正規化（u1〜u3 はそのまま、A は (A + A_max)/(2 A_max)）"""
        return np.array([u['u1'], u['u2'], u['u3'], (u['A'] + self.A_max) / (2.0 * self.A_max)])


# ==============================================================================
# ファームの FF 計算の再現と、送信前の検証（計画書 §3.2）
# ==============================================================================
def firmware_ff(coeffs, T, t_ms):
    """ファームの AddInput（.ino:1207-1246）の出力 FF を float32 で再現する。

    t_ms は目標値が変わってからの経過 [ms]（整数）。timeFF = (float(millis) − t_start)/1000.0 を float に代入し、
    [0, T] でクリップ、timeFF == T なら 0。式は a*t*t*t*t*t + b*t*t*t*t + c*t*t*t + d*t*t + e*t を左から float32 で計算する
    （コンパイラが積和を FMA にまとめると最後の桁が違いうるが、0.5 PWM の判定には影響しない）。
    """
    c = np.asarray(coeffs, dtype=np.float32)
    T32 = np.float32(T)
    t = (np.asarray(t_ms, dtype=np.float64) / 1000.0).astype(np.float32)
    t = np.maximum(np.float32(0.0), np.minimum(T32, t))
    v = _poly_f32(c, t)
    return np.where(t == T32, np.float32(0.0), v)


def _poly_f32(c, t):
    a, b, cc, d, e = (np.float32(x) for x in c)
    t = np.asarray(t, dtype=np.float32)
    return a * t * t * t * t * t + b * t * t * t * t + cc * t * t * t + d * t * t + e * t


def presend_check(coeffs, T, A_max, tol_amp=0.5, tol_end=0.5):
    """float32 に丸めた係数で、送る前に検証する。

    検査: (1) extrema_ok（(0,T) の極値ちょうど2個・単純根）, (2) 1 ms 格子でファームを再現した max|FF| ≤ A_max + tol_amp,
          (3) |f(T)| ≤ tol_end（float32 への丸めで f(T) = 0 が崩れていないこと）。f(0) = 0 は定数項がないので自明（検査しない）。
    記録だけ: T で 0 に切り替わる直前 10 ms の最大 |FF|（段差）。
    """
    c32 = np.asarray(coeffs, dtype=np.float32)
    T32 = np.float32(T)
    T64 = float(T32)
    reasons = []
    ext = extrema_ok(c32.astype(float), T64)
    if ext is None:
        reasons.append('(0,T) の極値がちょうど2個（単純根）ではない')
    t_ms = np.arange(0, int(math.ceil(1000.0 * T64)) + 1)
    v = firmware_ff(c32, T32, t_ms)
    max_abs = float(np.max(np.abs(v))) if np.all(np.isfinite(v)) else float('inf')
    if not max_abs <= A_max + tol_amp:
        reasons.append(f'max|FF| = {max_abs:.2f} > A_max + {tol_amp:g}')
    end = float(_poly_f32(c32, T32))
    if not abs(end) <= tol_end:
        reasons.append(f'|f(T)| = {abs(end):.3f} > {tol_end:g}')
    tail = (t_ms >= 1000.0 * T64 - 10.0) & (t_ms < 1000.0 * T64)
    step = float(np.max(np.abs(v[tail]))) if tail.any() else 0.0
    return {'ok': not reasons, 'reasons': reasons, 'max_abs': max_abs, 'end_value': end, 'step_at_T': step,
            'extrema': ext, 'coeffs_f32': c32, 'T_f32': T32}


# ==============================================================================
# 目標軌道（拘束付きフィット。計画書 §4）
# ==============================================================================
def _companion(T1, wn):
    a2, a1, a0 = system_coeffs(T1, 1.0, wn)
    return np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [-a0, -a1, -a2]])


def phi_at(T1, wn, t):
    """φ(t)：(T1 s+1)(s+ωn)² の自由応答（初期状態 [1, 0, 0]）の位置。expm で計算する（κ=1 付近の桁落ちを避ける）"""
    return float((expm(_companion(T1, wn) * float(t)) @ np.array([1.0, 0.0, 0.0]))[0])


def completion_theta(kappa, eps_eff):
    """無次元の完了時刻 θ(κ)：φ(θ; T1=κ, ωn=1) = ε_eff の唯一の根（φ は単調減少）。ε_eff ≥ 1 なら 0"""
    if eps_eff >= 1.0:
        return 0.0
    if eps_eff <= 0.0:
        raise ValueError("ε_eff は正の値にしてください")
    A = _companion(float(kappa), 1.0)

    def f(lt):
        return float((expm(A * math.exp(lt)) @ np.array([1.0, 0.0, 0.0]))[0]) - eps_eff

    lo, hi = 1e-6, 10.0
    while f(math.log(hi)) >= 0.0:
        hi *= 2.0
        if hi > 1e7:
            raise ValueError(f"κ = {kappa:g}: 完了時刻が 1e7 を超えます")
    return math.exp(scipy.optimize.brentq(f, math.log(lo), math.log(hi), xtol=1e-13, rtol=1e-12))


def completion_time(T1, wn, eps_eff):
    return completion_theta(T1 * wn, eps_eff) / wn


def measured_completion(y, valid, band, dt=TICK_DT):
    """実測の完了時刻：それ以後の有効な点がすべて |y| ≤ band に入る最初の時刻（評価区間の最後まで入らなければ nan）"""
    y = np.asarray(y, dtype=float)
    m = np.asarray(valid, dtype=bool) & np.isfinite(y)
    idx = np.flatnonzero(m)
    if len(idx) == 0:
        return float('nan')
    out = idx[np.abs(y[idx]) > band]
    if len(out) == 0:
        return 0.0
    if out[-1] == idx[-1]:
        return float('nan')
    nxt = idx[idx > out[-1]][0]
    return float(nxt * dt)


@dataclass
class TargetSpec:
    """1自由度の目標軌道（固定）。seq() が全試行で同じ数列（POT の値）を返す"""
    dof: int
    Pi: float
    Pf: float
    T: float
    eps: float
    N_e: int
    dt: float
    y0_ref: float
    P0_ref: float
    eps_eff: float
    T1: float
    wn: float
    kappa: float
    xi: float
    theta: float
    active: bool
    method: str
    cost: float
    free_T1: float
    free_wn: float
    free_t_eps: float
    t_eps_meas: list
    n_runs: int
    warnings: list = field(default_factory=list)
    J_noise: float = float('nan')
    J_quant: float = float('nan')
    J_ref: list = field(default_factory=list)

    @property
    def dP(self):
        return self.Pf - self.Pi

    def seq(self):
        """目標軌道の POT の値（j − j0 = 0..N_e）"""
        return target_states(self.T1, self.wn, self.y0_ref, self.dt, self.N_e)[:, 0] + self.Pf

    def to_dict(self):
        d = dict(self.__dict__)
        return {k: (float(v) if isinstance(v, (np.floating,)) else v) for k, v in d.items()}

    @classmethod
    def from_dict(cls, d):
        return cls(**d)


def _fit_free_multi(ys, ms, y0, dt, bounds):
    """複数の基準試行への自由フィット（fit_target と同じ開始点・上下限・手法で、残差を縦に並べる）"""
    n = len(ys[0]) - 1
    lo = [np.log(bounds['tgt_T1'][0]), np.log(bounds['tgt_wn'][0])]
    hi = [np.log(bounds['tgt_T1'][1]), np.log(bounds['tgt_wn'][1])]

    def res(q):
        with np.errstate(all='ignore'):
            X = target_states(np.exp(q[0]), np.exp(q[1]), y0, dt, n)
            r = np.concatenate([(X[:, 0] - y)[m] for y, m in zip(ys, ms)])
        return np.nan_to_num(r, nan=1e6, posinf=1e6, neginf=-1e6)

    starts = [[np.log(a), np.log(b)] for a in (0.03, 0.15, 0.5) for b in (3.0, 8.0, 20.0)]
    best = None
    for q0 in starts:
        try:
            r = scipy.optimize.least_squares(res, np.clip(q0, lo, hi), bounds=(lo, hi), method='trf', x_scale='jac')
        except Exception:
            continue
        if best is None or r.cost < best.cost:
            best = r
    return (float(np.exp(best.x[0])), float(np.exp(best.x[1])))


def fit_target_constrained(dof, ys, valids, P0s, Pi, Pf, T, eps, bounds=None, dt=TICK_DT, log=None):
    """基準試行（y = P − Pf, j − j0 = 0..N_e）に、完了拘束 φ(T) ≤ ε_eff 付きで ζ=1 の3次遅れ系を当てはめる（計画書 §4）"""
    bounds = bounds or DEFAULT_ID_BOUNDS
    ys = [np.asarray(y, dtype=float) for y in ys]
    ms = [np.asarray(v, dtype=bool) & np.isfinite(y) for y, v in zip(ys, valids)]
    N = len(ys[0]) - 1
    dP = float(Pf - Pi)
    P0_ref = float(np.mean(P0s))
    y0 = P0_ref - float(Pf)
    if dP == 0.0 or y0 == 0.0:
        raise ValueError(f"DOF {dof}: ΔP または y0 が 0 なので目標軌道を作れません")
    eps_eff = float(eps) * abs(dP) / abs(y0)
    n_res = int(sum(m.sum() for m in ms))
    warnings = []

    def resid(T1, wn):
        with np.errstate(all='ignore'):
            phi = target_states(T1, wn, 1.0, dt, N)[:, 0]
            r = np.concatenate([(y0 * phi - y)[m] for y, m in zip(ys, ms)]) / abs(dP)
        return np.nan_to_num(r, nan=1e3, posinf=1e3, neginf=-1e3)

    def cost(T1, wn):
        r = resid(T1, wn)
        return float(np.mean(r * r)) if len(r) else float('nan')

    def in_bounds(T1, wn):
        return (bounds['tgt_T1'][0] <= T1 <= bounds['tgt_T1'][1]) and (bounds['tgt_wn'][0] <= wn <= WN_MAX)

    # 1. 自由フィット（基準試行が1回なら移植した fit_target そのもの。前回値は使わない）
    if len(ys) == 1:
        (T1f, wnf), _rms = fit_target(ys[0], ms[0], y0, dt, bounds, prev=None)
    else:
        T1f, wnf = _fit_free_multi(ys, ms, y0, dt, bounds)
    t_meas = [measured_completion(y, m, eps * abs(dP), dt) for y, m in zip(ys, ms)]
    common = dict(dof=dof, Pi=float(Pi), Pf=float(Pf), T=float(T), eps=float(eps), N_e=N, dt=dt, y0_ref=y0,
                  P0_ref=P0_ref, eps_eff=eps_eff, free_T1=T1f, free_wn=wnf, t_eps_meas=t_meas, n_runs=len(ys))
    if eps_eff >= 1.0:
        warnings.append(f"始点がすでに許容帯の中（ε_eff = {eps_eff:.3g} ≥ 1）なので拘束なし（自由フィット）にしました")
        return TargetSpec(T1=T1f, wn=wnf, kappa=T1f * wnf, xi=float('nan'), theta=0.0, active=False, method='free',
                          cost=cost(T1f, wnf), free_t_eps=0.0, warnings=warnings, **common)
    t_free = completion_time(T1f, wnf, eps_eff)
    if t_free <= T:
        warnings.append(f"基準の応答（自由フィット）がすでに T 以内に完了しています（{t_free:.3f} s ≤ T = {T:g} s）。"
                        f"拘束は効かず、FF で改善できる余地は小さいと考えられます")
        th = completion_theta(T1f * wnf, eps_eff)
        return TargetSpec(T1=T1f, wn=wnf, kappa=T1f * wnf, xi=float(np.log(wnf * T / th)) if th > 0 else float('nan'),
                          theta=th, active=False, method='free', cost=cost(T1f, wnf), free_t_eps=t_free,
                          warnings=warnings, **common)

    cands = []
    # 3. 境界 ξ = 0（ωn = θ(κ)/T）上の1次元探索：log κ の121点格子 → 有界 Brent 法
    def bnd_cost(lk):
        kappa = math.exp(lk)
        th = completion_theta(kappa, eps_eff)
        wn = th / T
        T1 = kappa / wn
        return cost(T1, wn) if in_bounds(T1, wn) else float('inf')

    grid = np.linspace(math.log(1e-3), math.log(1e3), 121)
    gc = np.array([bnd_cost(lk) for lk in grid])
    if np.isfinite(gc).any():
        i = int(np.nanargmin(gc))
        a, b = grid[max(i - 1, 0)], grid[min(i + 1, len(grid) - 1)]
        r = scipy.optimize.minimize_scalar(bnd_cost, bounds=(a, b), method='bounded', options={'xatol': 1e-7})
        lk = float(r.x) if np.isfinite(r.fun) and r.fun <= gc[i] else float(grid[i])
        kappa = math.exp(lk)
        th = completion_theta(kappa, eps_eff)
        cands.append((bnd_cost(lk), kappa, 0.0, th, 'boundary'))

    # 4. (log κ, σ) の箱付き least_squares（ξ = σ·ξ_max(κ)、ξ_max = min(ln 10, ln(ωn_max T/θ))）を多点から
    def unpack(q):
        kappa = math.exp(q[0])
        th = completion_theta(kappa, eps_eff)
        xmax = min(XI_CAP, math.log(WN_MAX * T / th))
        if xmax < 0:
            return None
        xi = float(q[1]) * xmax
        wn = th / T * math.exp(xi)
        T1 = kappa / wn
        return (kappa, xi, th, T1, wn) if in_bounds(T1, wn) else None

    def resid2(q):
        u = unpack(q)
        return resid(u[3], u[4]) if u is not None else np.full(n_res, 1e3)

    lo_b, hi_b = [math.log(1e-3), 0.0], [math.log(1e3), 1.0]
    for k0 in (0.03, 0.1, 0.3, 1.0, 3.0, 10.0, 30.0):
        for s0 in (0.0, 0.3, 0.7):
            try:
                r = scipy.optimize.least_squares(resid2, [math.log(k0), s0], bounds=(lo_b, hi_b), method='trf',
                                                 x_scale='jac')
            except Exception:
                continue
            u = unpack(r.x)
            if u is not None:
                cands.append((cost(u[3], u[4]), u[0], u[1], u[2], 'interior'))
    cands = [c for c in cands if np.isfinite(c[0])]
    if not cands:
        raise ValueError(f"DOF {dof}: 拘束を満たす目標軌道が見つかりません（T・ε・上下限を見直してください）")
    c_best = min(cands, key=lambda c: c[0])
    _cst, kappa, xi, th, how = c_best
    wn = th / T * math.exp(xi)
    T1 = kappa / wn
    if abs(kappa - 1.0) < 0.02:
        warnings.append("拘束付きフィットは κ ≈ 1（3重極）になりました。基準が T より遅いとき、T に完了する族の中で"
                        "3重極がすべての時刻で最も緩やかなので、実測の形によらずこの解になります（仕様どおり）")
    return TargetSpec(T1=T1, wn=wn, kappa=kappa, xi=xi, theta=th, active=True, method=how, cost=cost(T1, wn),
                      free_t_eps=t_free, warnings=warnings, **common)


def noise_floor(spec_list, ys_by_dof, valids_by_dof):
    """J_noise = ½·mean_{組}(mean(((y_r1 − y_r2)/ΔP)²))（自由度で平均）と、量子化の下限 mean_i 1/(12ΔP_i²)"""
    vals = []
    for sp in spec_list:
        ys, vs = ys_by_dof[sp.dof], valids_by_dof[sp.dof]
        pairs = []
        for i in range(len(ys)):
            for k in range(i + 1, len(ys)):
                m = vs[i] & vs[k] & np.isfinite(ys[i]) & np.isfinite(ys[k])
                if m.any():
                    pairs.append(0.5 * float(np.mean(((ys[i][m] - ys[k][m]) / sp.dP) ** 2)))
        if pairs:
            vals.append(float(np.mean(pairs)))
    J_noise = float(np.mean(vals)) if vals else float('nan')
    J_quant = float(np.mean([1.0 / (12.0 * sp.dP ** 2) for sp in spec_list])) if spec_list else float('nan')
    return J_noise, J_quant


# ==============================================================================
# 100 Hz の時間軸（計画書 §5.1）
# ==============================================================================
TICK_GAP_FACTOR = 1.5   # 受信間隔が局所の中央値のこの倍を超えたところだけを欠損の候補にする（修正案1の 1.8 から変更。assign_ticks）
TICK_WIN = 5            # 候補の前後で「遅れのない到着」を見る受信数。これより近い候補は1つのクラスタにまとめて判定する
TICK_HALF = 50          # 局所の受信間隔の中央値を取る範囲（前後それぞれの間隔数）
TICK_COMPRESSED = 0.7   # 局所の中央値のこの倍より短い間隔は「遅れを取り戻している途中」（実機では通常 0.25% 未満）
TICK_EARLY = 0.3        # 欠損の位置を決めるとき、この配信数より早い到着を強く罰する（遅れは正なので早着はほぼ起きない）
TICK_ENUM_MAX = 20000   # 欠損の置き方の組み合わせがこれより多いクラスタは、丸めで位置を決める


def local_intervals(d, half=TICK_HALF):
    """各受信間隔 d_n の前後 half 個の中央値（0.5〜1.5 倍の外を除いてもう一度中央値）。受信間隔の変化に追従する"""
    d = np.asarray(d, dtype=float)
    if len(d) == 0:
        return np.empty(0)
    pad = np.concatenate([np.full(half, np.nan), d, np.full(half, np.nan)])
    W = np.lib.stride_tricks.sliding_window_view(pad, 2 * half + 1)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        med = np.nanmedian(W, axis=1)
        W2 = np.where((W > 0.5 * med[:, None]) & (W < 1.5 * med[:, None]), W, np.nan)
        med2 = np.nanmedian(W2, axis=1)
    return np.where(np.isfinite(med2), med2, med)


def _place_drops(est, m):
    """クラスタ（受信 s..e、L = e − s）の中で m 個の欠損を置き、受信 s+1..e−1 の s からの番号の差を返す。

    est[k] = (t_{s+k} − 遅れのない s の到着)/u は「番号の差 + 遅れ」。遅れは正なので、早い到着（est が番号の差より
    TICK_EARLY 以上小さい）を強く罰し、遅れは小さく罰する費用が最小になる置き方を全通り調べて選ぶ。
    """
    L = len(est) - 1
    base = np.arange(1, L)
    if L <= 1 or m == 0:
        return base

    def cost(jk):
        lam = est[1:L] - jk
        return float(np.sum(np.where(lam >= -TICK_EARLY, np.abs(lam), 10.0 * -lam)))

    if math.comb(L + m - 1, m) > TICK_ENUM_MAX:
        out = np.empty(L - 1, dtype=int)
        p = 0
        for k in range(1, L):
            p = max(p + 1, min(int(round(est[k] - TICK_EARLY)), m + k))
            out[k - 1] = p
        return out
    best, best_cost = base, None
    for pos in itertools.combinations_with_replacement(range(L), m):    # 間隔 p（受信 s+p → s+p+1）に欠損を置く
        jk = base + np.searchsorted(np.asarray(pos), base, side='left')
        cst = cost(jk)
        if best_cost is None or cst < best_cost:
            best, best_cost = jk, cst
    return best


def assign_ticks(t, u_hint=TICK_DT):
    """受信時刻 t（PC の時計、昇順）に配信番号 j を振る（受信順に数える方式。2026-10-27 の修正案1）。

    1. 受信順に番号を振る。受信間隔 d_n が局所の中央値 U_n（前後 TICK_HALF 間隔。受信間隔の変化に追従する）の
       TICK_GAP_FACTOR 倍を超えるところだけを欠損の候補にする。
    2. 一時的な遅れでも間隔は大きくなるが、遅れた後はまとまって届いて追いつく。そこで候補の前後 TICK_WIN 個の受信から
       「遅れのない到着」（遅れは正なので、u で外挿した到着の最小値）T_s・T_e を求め、追いつかなかった分
       m = round((T_e − T_s)/u − (e − s)) を欠損数とする（0 以上、間隔から見た数 Σ(round(d/u) − 1) 以下）。
       後ろの窓は、追いつきの途中の受信（詰まった間隔 < TICK_COMPRESSED·U が TICK_WIN 個以内に続く間）を越えた先まで伸ばす
       （PC が長く止まった後のまとまった到着や、数受信かけてゆっくり追いつく場合も、時間どおりに戻った受信で判定する。
       受信間隔が変わっただけなら詰まった間隔は出ないので伸ばさない）。窓の中に続く候補は1つのクラスタにまとめて判定する。
    3. クラスタの中のどの間隔で欠けたかは _place_drops で決める。遅れてまとまって届いた直後の欠損は、受信時刻だけでは
       位置が決まらず、クラスタの中で ±1〜2 配信ずれることがある。ずれはクラスタの外へ持ち越さない。
    受信間隔の変化（PC 側の受信の遅れの変化）は欠損と数えない。以前の位相法は区間全体で1つの周期を仮定したため、
    受信間隔が途中で変わると存在しない欠損を数えた（2026-10-27 の実機検証）。位相法の数は count_drops_phase で検算として残す。
    限界：PC 側の受信が数十受信にわたって少しずつ遅れ続け、窓の外でまとめて取り戻す場合は、遅れを欠損と数えることがある
    （同じプロセスで動かした偽ボードで見た。実機の記録 455 本にはない）。評価区間に入れば欠損として測り直しになる。
    閾値を修正案1の 1.8 倍から 1.5 倍にした理由：遅れて届いた受信の直後に欠損があると、間隔は 2 倍より短くなる。
    実機の受信間隔は局所中央値の 1.1〜1.3 倍が 1.8% あり、1.8 倍では欠損の約 1% を見落とす。候補を増やしても欠損数は 2. の
    追いつきで決まるので、実機の記録 455 本で欠損ありと数えるのは 1.5 倍でも 0 本のまま。
    戻り値 (j（最初を 0）, u（局所中央値の中央値。PC の時計で測った 10 ms）, 欠損数)。
    """
    t = np.asarray(t, dtype=float)
    n = len(t)
    if n < 3:
        return np.arange(n), u_hint, 0
    d = np.diff(t)
    U = local_intervals(d)
    U = np.where(np.isfinite(U) & (U > 0), U, u_hint)
    clusters = []                                      # [候補の間隔の番号の並び, まとまった到着の最後の受信]
    for g in np.flatnonzero(d > TICK_GAP_FACTOR * U):
        g = int(g)
        if clusters and g <= clusters[-1][1] + TICK_WIN - 1:
            clusters[-1][0].append(g)
        else:
            clusters.append([[g], 0])
        k = g + 1                                      # 追いつきの途中の受信を飛ばす：次の TICK_WIN 個の間隔に詰まった間隔
        while True:                                    # （局所中央値の TICK_COMPRESSED 倍未満）がある間は先へ進む
            w = np.arange(k, min(n - 1, k + TICK_WIN))
            q = w[d[w] < TICK_COMPRESSED * U[w]]
            if len(q) == 0:
                break
            k = int(q[-1]) + 1
        clusters[-1][1] = k
    j = np.arange(n)
    add = 0
    done = 0
    for cl, be in clusters:
        s, e = cl[0], cl[-1] + 1                       # 候補の間隔は受信 s → s+1, …, e−1 → e
        u = float(U[s])
        j[done:s + 1] = np.arange(done, s + 1) + add
        kb = np.arange(max(0, s - TICK_WIN + 1), s + 1)
        ka = np.arange(e, min(n, be + TICK_WIN))       # まとまった到着の後の TICK_WIN 個まで（長く止まった後も追いつきを見る）
        Ts = float(np.min(t[kb] + (s - kb) * u))
        Te = float(np.min(t[ka] - (ka - e) * u))
        m_hi = int(sum(max(0, int(round(d[g] / u)) - 1) for g in cl))
        m = min(max(int(round((Te - Ts) / u - (e - s))), 0), m_hi)
        j[s + 1:e] = s + add + _place_drops((t[s:e + 1] - Ts) / u, m)
        add += m
        done = e
    j[done:] = np.arange(done, n) + add
    return j, float(np.median(U)), int(add)


def count_drops_phase(t, u_hint=TICK_DT):
    """検算用：位相法（2026-10-23〜27 に使っていた方式）で数えた欠損数と周期 u。

    周期 u を、位相のそろい具合 R(u) = |mean exp(2πi (t − t_0)/u)| が最大になる値として求め（受信間隔の中央値の ±3% を
    1201 点で探し、有界 Brent 法で詰める）、j_n = round((t_n − t_0 − a0)/u) を後ろから単調にする。
    区間全体で1つの周期を仮定するので、受信間隔が途中で変わると存在しない欠損を数える。戻り値 (欠損数, u)。
    """
    t = np.asarray(t, dtype=float)
    n = len(t)
    if n < 3:
        return 0, u_hint
    x = t - t[0]
    d = np.diff(x)
    good = d[(d > 0.3 * u_hint) & (d < 1.8 * u_hint)]
    u0 = float(np.median(good)) if len(good) >= 3 else u_hint

    def R(u):
        return float(np.abs(np.mean(np.exp(2j * np.pi * x / u))))

    us = u0 * np.linspace(0.97, 1.03, 1201)
    Rs = np.array([R(u) for u in us])
    i = int(np.argmax(Rs))
    step = us[1] - us[0]
    r = scipy.optimize.minimize_scalar(lambda u: -R(u), bounds=(us[i] - step, us[i] + step), method='bounded',
                                       options={'xatol': 1e-12})
    u = float(r.x) if -r.fun >= Rs[i] else float(us[i])
    a0 = float(np.angle(np.mean(np.exp(2j * np.pi * x / u)))) / (2.0 * np.pi) * u
    j = np.round((x - a0) / u).astype(int)
    for k in range(n - 2, -1, -1):
        j[k] = min(j[k], j[k + 1] - 1)
    return int(j[-1] - j[0] - (n - 1)), u


def align_ticks(t_kind, t_ref, j_ref, u_hint=TICK_DT):
    """他の種類（PWM など）のメッセージにも配信番号を振り、tk の番号に合わせる。

    種類ごとに assign_ticks で番号を振ってから、各メッセージに受信時刻が最も近い tk の番号との差の最頻値をずらし量にする
    （同じ配信のメッセージは続けて届くので、個々の遅れに左右されない）。戻り値 j（tk の番号系）
    """
    t_kind = np.asarray(t_kind, dtype=float)
    if len(t_kind) == 0 or len(t_ref) == 0:
        return np.zeros(len(t_kind), dtype=int)
    jk, _u, _nd = assign_ticks(t_kind, u_hint)
    pos = np.clip(np.searchsorted(t_ref, t_kind), 1, len(t_ref) - 1)
    near = np.where(np.abs(t_ref[pos - 1] - t_kind) <= np.abs(t_ref[pos] - t_kind), pos - 1, pos)
    off = j_ref[near] - jk
    vals, counts = np.unique(off, return_counts=True)
    return jk + int(vals[np.argmax(counts)])


class TrialIssue(Exception):
    """試行の測定・解析で起きた問題。kind: 'comm'（測り直す）/ 'eval'（解析できない）"""

    def __init__(self, kind, reason):
        super().__init__(reason)
        self.kind = kind
        self.reason = reason


@dataclass
class DofSeries:
    """1自由度の、配信の時間軸にそろえた測定（j − j0 = -H..N_e+余裕）"""
    dof: int
    board: str
    local: int
    j0: int
    jrel: np.ndarray
    pot: np.ndarray
    echo: np.ndarray
    pwm: np.ndarray
    z1: np.ndarray
    z2: np.ndarray
    z3: np.ndarray
    der: np.ndarray
    Pi: float
    Pf: float
    P0: float
    H: int
    N_e: int
    echo_delay: float

    @property
    def t(self):
        return self.jrel * TICK_DT

    @property
    def dP(self):
        return self.Pf - self.Pi

    def win(self):
        return slice(self.H, self.H + self.N_e + 1)

    def missing(self):
        """評価区間 j − j0 = 0..N_e の中で受信できなかった配信の数"""
        return int(np.sum(~np.isfinite(self.pot[self.win()])))


def extract_series(raw, initial, target, dofs, t_send, T, history_s=HISTORY_S):
    """受信データ {board: {kind: (t, V)}} を、配信の時間軸で自由度ごとに切り出す（計画書 §5.1）。

    j0 はその自由度の目標値のエコー（[6+i]）が round(新しい目標値) になった最初の配信。同じボードの自由度は同じ j0。
    目標値を送ってから 1 s 以内（PC の時計）にエコーが変わらなければ TrialIssue('comm')。
    戻り値 ({dof: DofSeries}, {board: 情報})
    """
    N_e = n_eval(T)
    H = int(round(history_s / TICK_DT))
    out, info = {}, {}
    by_board = {}
    for d in dofs:
        by_board.setdefault(DOF_BOARD[d][1], []).append(d)
    for board, bdofs in by_board.items():
        count = next(c for b, _o, c in BOARD_LAYOUT if b == board)
        kinds = raw.get(board, {})
        if 'pot' not in kinds or len(kinds['pot'][0]) < 3:
            raise TrialIssue('comm', f"{board}: POT（/tk）を受信できていません")
        t_tk, V = kinds['pot']
        j_tk, u, n_drop = assign_ticks(t_tk)
        j0s = []
        for d in bdofs:
            local = DOF_BOARD[d][2]
            new = int(round(float(target[d])))
            if new == int(round(float(initial[d]))):
                continue
            hit = np.flatnonzero((V[:, count + local] == new) & (t_tk >= t_send - 0.05))
            if len(hit) == 0:
                raise TrialIssue('comm', f"DOF {d}: 目標値のエコーが {new} になりませんでした")
            delay = float(t_tk[hit[0]] - t_send)
            if delay > 1.0:
                raise TrialIssue('comm', f"DOF {d}: 目標値のエコーが変わるまで {delay:.2f} s かかりました（> 1 s）")
            j0s.append((int(j_tk[hit[0]]), delay))
        if not j0s:
            raise TrialIssue('eval', f"{board}: 目標値が変わる自由度がないので t=0 を決められません")
        if max(j for j, _ in j0s) - min(j for j, _ in j0s) > 1:
            raise TrialIssue('eval', f"{board}: 自由度ごとのエコーの変化がそろいません {j0s}")
        j0, delay = min(j0s)
        aligned = {'pot': (j_tk, V)}
        for kind in ('pwm', 'z1', 'z2', 'z3', 'der'):
            if kind in kinds and len(kinds[kind][0]):
                aligned[kind] = (align_ticks(kinds[kind][0], t_tk, j_tk), kinds[kind][1])
        jrel = np.arange(-H, N_e + MARGIN_TICKS + 1)
        grid = {}
        for kind, (jj, VV) in aligned.items():
            ncol = VV.shape[1]
            A = np.full((len(jrel), ncol), np.nan)
            pos = jj - j0 + H
            ok = (pos >= 0) & (pos < len(jrel))
            first = np.ones(len(pos), dtype=bool)
            seen = set()
            for i in np.flatnonzero(ok):
                if pos[i] in seen:
                    first[i] = False
                else:
                    seen.add(pos[i])
            sel = ok & first
            A[pos[sel]] = VV[sel]
            grid[kind] = A
        empty = np.full((len(jrel), count), np.nan)
        for d in bdofs:
            local = DOF_BOARD[d][2]
            pot = grid['pot'][:, local]
            pre = pot[max(0, H - int(round(REST_WINDOW_S / TICK_DT))):H]
            pre = pre[np.isfinite(pre)]
            P0 = float(np.mean(pre)) if len(pre) else float('nan')
            out[d] = DofSeries(dof=d, board=board, local=local, j0=j0, jrel=jrel, pot=pot,
                               echo=grid['pot'][:, count + local],
                               pwm=grid.get('pwm', empty)[:, local], z1=grid.get('z1', empty)[:, local],
                               z2=grid.get('z2', empty)[:, local], z3=grid.get('z3', empty)[:, local],
                               der=grid.get('der', empty)[:, local], Pi=float(initial[d]), Pf=float(target[d]),
                               P0=P0, H=H, N_e=N_e, echo_delay=delay)
        dt_tk = np.diff(t_tk)
        n_gap = int(np.sum(dt_tk > TICK_GAP_FACTOR * local_intervals(dt_tk)))     # 欠損の候補にした大きな受信間隔
        n_drop_phase, _u = count_drops_phase(t_tk)                                 # 検算（受信間隔が変わると多めに数える）
        info[board] = {'j0': j0, 'u_pc': u, 'n_drop': n_drop, 'n_msgs': len(t_tk), 'echo_delay': delay, 'n_gap': n_gap,
                       'n_drop_phase': n_drop_phase}
    return out, info


def evaluate(series, targets):
    """J_i = mean((POT − Target)/ΔP)²（評価区間・受信できた配信）、J = 自由度の平均（計画書 §5.2）"""
    J_i = {}
    for d, s in series.items():
        w = s.win()
        pot = s.pot[w]
        J_i[d] = normalized_error(pot, targets[d].seq(), s.dP, np.isfinite(pot))
    vals = [v for v in J_i.values() if np.isfinite(v)]
    J = float(np.mean(vals)) if len(vals) == len(J_i) and vals else float('nan')
    return J, J_i


# ==============================================================================
# 移植：実機との通信（RobotLink）と1試行の部品（optimize_EL2_common.py）
# ==============================================================================
# 移植元: optimize_EL2_common.py:1645-1645 @f8fba60
CSV_KIND = {'pot': 'pot', 'pwm': 'pwm', 'z3': 'z3', 'z1': 'z1', 'z2': 'z2', 'der': 'derivatives'}


# 移植元: optimize_EL2_common.py:1697-1884 @f8fba60
RING_SEC = 3.0              # 切替前の履歴として残す受信の長さ [s]（PCの時計）


class RobotLink:
    """目標値・FF の送信と、5ボードの POT・PWM・z1・z2・z3・derivatives・ADRCパラメータの受信。

    受信はボードごとに1つの MutuallyExclusiveCallbackGroup で順に処理する（同じ配信の中の到着順を保つため）。
    受信時刻（time.monotonic）は欠落の判定と、同じ配信の組み合わせにだけ使う。
    """

    def __init__(self, node_name='el2_optimizer'):
        import rclpy
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from std_msgs.msg import Float32MultiArray, UInt16MultiArray
        self._rclpy = rclpy
        self._F32 = Float32MultiArray
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(node_name)
        self.lock = threading.Lock()
        self.ring = {b: {k: deque() for k in KINDS} for b, _, _ in BOARD_LAYOUT}
        self.capture = None
        self.status = {b: None for b, _, _ in BOARD_LAYOUT}
        self.last_pot = {b: None for b, _, _ in BOARD_LAYOUT}
        self.last_return = None                                         # 最後に初期姿勢を送った時刻（monotonic）
        self.pub_target = self.node.create_publisher(Float32MultiArray, TOPIC_TARGET, 10)
        self.pub_ff = {k: self.node.create_publisher(Float32MultiArray, TOPIC_FF.format(k), 10) for k in range(1, 6)}
        for k, (board, _off, _cnt) in enumerate(BOARD_LAYOUT, start=1):
            grp = MutuallyExclusiveCallbackGroup()
            self.node.create_subscription(UInt16MultiArray, TOPIC_POT.format(k),
                                          lambda m, b=board: self._cb(b, 'pot', m), SENSOR_QOS_DEPTH, callback_group=grp)
            for kind, topic in (('pwm', TOPIC_PWM), ('z3', TOPIC_Z3), ('z1', TOPIC_Z1), ('z2', TOPIC_Z2), ('der', TOPIC_DER)):
                self.node.create_subscription(Float32MultiArray, topic.format(k),
                                              lambda m, b=board, kd=kind: self._cb(b, kd, m), SENSOR_QOS_DEPTH,
                                              callback_group=grp)
            self.node.create_subscription(Float32MultiArray, TOPIC_PARAM.format(k),
                                          lambda m, b=board: self._cb_status(b, m), 10,
                                          callback_group=MutuallyExclusiveCallbackGroup())
        self.executor = MultiThreadedExecutor(num_threads=8)
        self.executor.add_node(self.node)
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        while not self._stop.is_set() and self._rclpy.ok():
            try:
                self.executor.spin_once(timeout_sec=0.05)
            except Exception:
                if self._stop.is_set():
                    break

    def _cb(self, board, kind, msg):
        t = time.monotonic()
        v = tuple(float(x) for x in msg.data)
        with self.lock:
            ring = self.ring[board][kind]
            ring.append((t, v))
            while ring and ring[0][0] < t - RING_SEC:
                ring.popleft()
            if self.capture is not None:
                self.capture[board][kind].append((t, v))
            if kind == 'pot':
                self.last_pot[board] = (t, v)

    def _cb_status(self, board, msg):
        data = [float(x) for x in msg.data]
        if len(data) != 50:
            return
        with self.lock:
            self.status[board] = (time.monotonic(), data)

    # ---------------- 参照 ----------------
    def adrc_params(self, max_age=2.0):
        """{DOF: AdrcParams or None}（状態トピックが max_age 秒以上届いていないボードは None）"""
        now = time.monotonic()
        out = {}
        with self.lock:
            st = dict(self.status)
        for board, offset, count in BOARD_LAYOUT:
            s = st[board]
            for j in range(count):
                if s is None or now - s[0] > max_age:
                    out[offset + j] = None
                    continue
                vals = s[1][2 + 4 * j: 6 + 4 * j]
                out[offset + j] = AdrcParams(*vals) if all(math.isfinite(v) for v in vals) else None
        return out

    def board_targets(self, max_age=1.0):
        """ボードが今使っている目標値（/board{k}_tk/pub の [6..11]）を26個のリストで返す（届いていなければ None）"""
        now = time.monotonic()
        out = [None] * N_DOF
        with self.lock:
            lp = dict(self.last_pot)
        for board, offset, count in BOARD_LAYOUT:
            p = lp[board]
            if p is None or now - p[0] > max_age:
                continue
            for j in range(count):
                out[offset + j] = p[1][6 + j]
        return out

    def receiving(self, max_age=1.0):
        now = time.monotonic()
        with self.lock:
            return {b: (p is not None and now - p[0] <= max_age) for b, p in self.last_pot.items()}

    # ---------------- 送信 ----------------
    def send_targets(self, pose):
        msg = self._F32()
        msg.data = [float(v) for v in pose]
        self.pub_target.publish(msg)

    def send_ff(self, ff_rows, T):
        """ff_rows: FF番号 0〜23 の [a,b,c,d,e]（24行）。各ボード 36要素 = 6スロット × [a,b,c,d,e,T]"""
        rows = np.asarray(ff_rows, dtype=float).reshape(N_FF, 5)
        for k, (board, _offset, _count) in enumerate(BOARD_LAYOUT, start=1):
            data = []
            n_ff = FF_BOARD_DOFS[board]
            first = sum(FF_BOARD_DOFS[b] for b, _, _ in BOARD_LAYOUT[:k - 1])
            for slot in range(6):
                if slot < n_ff:
                    data.extend([float(v) for v in rows[first + slot]] + [float(T)])
                else:
                    data.extend([0.0, 0.0, 0.0, 0.0, 0.0, float(T)])
            msg = self._F32()
            msg.data = data
            self.pub_ff[k].publish(msg)

    # ---------------- 記録 ----------------
    def start_capture(self, history_s=1.2):
        """切替前 history_s 秒ぶんの受信を含めて記録を始める"""
        now = time.monotonic()
        with self.lock:
            self.capture = {b: {k: [x for x in self.ring[b][k] if x[0] >= now - history_s] for k in KINDS}
                            for b, _, _ in BOARD_LAYOUT}

    def capture_counts(self, since):
        """時刻 since（monotonic）より後に受信した PWM の値の変化の数（ボードごと。周回数の目安）"""
        with self.lock:
            cap = self.capture
            if cap is None:
                return {}
            out = {}
            for b, _, count in BOARD_LAYOUT:
                msgs = [m for m in cap[b]['pwm'] if m[0] > since]
                out[b] = sum(1 for i in range(1, len(msgs)) if msgs[i][1][:count] != msgs[i - 1][1][:count])
            return out

    def stop_capture(self):
        """記録を終えて {board: {kind: (t, V)}} を返す（pot の V は POT, 目標値 の 2*count 列）"""
        with self.lock:
            cap, self.capture = self.capture, None
        out = {}
        for board, _offset, count in BOARD_LAYOUT:
            raw = {}
            for kind in KINDS:
                msgs = (cap or {}).get(board, {}).get(kind, [])
                if not msgs:
                    continue
                t = np.array([m[0] for m in msgs])
                if kind == 'pot':
                    V = np.array([list(m[1][:count]) + list(m[1][6:6 + count]) for m in msgs if len(m[1]) >= 6 + count])
                    t = np.array([m[0] for m in msgs if len(m[1]) >= 6 + count])
                else:
                    V = np.array([m[1][:count] for m in msgs if len(m[1]) >= count])
                    t = np.array([m[0] for m in msgs if len(m[1]) >= count])
                if len(t):
                    order = np.argsort(t, kind='stable')
                    raw[kind] = (t[order], V[order].astype(float))
            out[board] = raw
        return out

    def close(self):
        if self._stop.is_set():
            return
        self._stop.set()
        self.thread.join(timeout=2.0)
        try:
            self.executor.shutdown()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass


# 移植元: optimize_EL2_common.py:1890-1915 @f8fba60
@dataclass
class TrialRecord:
    raw: dict                       # {board: {kind: (t, V)}}
    initial: list                   # 26個
    target: list                    # 26個
    ff: np.ndarray                  # (24, 5) 送ったFF
    T: float
    L: float
    params: dict                    # {DOF: AdrcParams or None}（試行の直前に実機から読んだ値）
    params_after: dict
    t_target: float                 # 目標値を送った時刻（monotonic。CSV の時刻の原点）
    started_at: str
    aborted: bool = False
    note: str = ''


class StopRequested(Exception):
    pass


def _sleep_checked(sec, stop_event):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        if stop_event is not None and stop_event.is_set():
            raise StopRequested()
        time.sleep(min(0.05, max(0.0, end - time.monotonic())))


# 移植元: optimize_EL2_common.py:1928-1957 @f8fba60
def wait_rest(link, initial, dofs, stop_event, log, window=0.5, band=3.0, offset_tol=8.0, max_extra=30.0):
    """最適化対象の自由度が初期値の近くで静止するまで待つ（直近 window 秒の POT の幅 ≤ band、平均と初期値の差 ≤ offset_tol）。

    前の試行の戻りや振動が残ったまま次の試行を始めると、保持PWM・初期状態が「静止」の前提から外れ、同定も評価も崩れる。
    max_extra 秒待っても満たさなければ警告して進む。
    """
    t_end = time.monotonic() + max_extra
    warned = False
    while True:
        now = time.monotonic()
        bad = []
        with link.lock:
            rings = {b: list(link.ring[b]['pot']) for b, _, _ in BOARD_LAYOUT}
        for d in dofs:
            _k, board, local = DOF_BOARD[d]
            vals = [m[1][local] for m in rings[board] if m[0] >= now - window]
            if len(vals) < 10:
                bad.append((d, 'no data'))
                continue
            if max(vals) - min(vals) > band or abs(np.mean(vals) - float(initial[d])) > offset_tol:
                bad.append((d, f"{min(vals):.0f}〜{max(vals):.0f}"))
        if not bad:
            return True
        if not warned:
            log(f"  初期姿勢で静止していない自由度があるので待ちます: {bad}")
            warned = True
        if now > t_end:
            log(f"  警告: {max_extra:g} 秒待っても静止しませんでした（{bad}）。このまま試行します")
            return False
        _sleep_checked(0.2, stop_event)


# ==============================================================================
# OptunaLink：移植した RobotLink に計画書 §6・§7 の機能を足す（RobotLink 本体は変えない）
# ==============================================================================
OWN_KEEP_S = 10.0           # 自分の送信として覚えておく時間 [s]（他プログラムの送信の判定に使う）


class OptunaLink(RobotLink):
    """変更点（RobotLink に足したもの）:
    ・rclpy を SIGINT ハンドラなしで初期化する（Ctrl+C で ROS が先に止まると FF=0 を送れないため。§7.6）
    ・目標値・FF を float32 に丸めて送り、送った値を記録する（§3.1）。FF の2段停止（§7.2）
    ・目標値・FF トピックを自分でも購読し、自分が送っていない値を「他プログラムの送信」として記録する（§7.7）
    ・自分以外の publisher 数、受信停止の検出、記録中の配信数の計数
    """

    def __init__(self, node_name='optuna_ff_optimizer'):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import Float32MultiArray
        if not rclpy.ok():
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        super().__init__(node_name)
        self.own = deque(maxlen=1024)
        self.foreign = deque(maxlen=256)
        self.last_ff = {k: None for k in range(1, len(BOARD_LAYOUT) + 1)}
        self.sent_any = False
        self.node.create_subscription(Float32MultiArray, TOPIC_TARGET, lambda m: self._watch(TOPIC_TARGET, m), 10)
        for k in range(1, len(BOARD_LAYOUT) + 1):
            topic = TOPIC_FF.format(k)
            self.node.create_subscription(Float32MultiArray, topic, lambda m, tp=topic: self._watch(tp, m), 10)

    # ---------------- 他プログラムの検出 ----------------
    def _watch(self, topic, msg):
        now = time.monotonic()
        vals = tuple(float(x) for x in np.asarray(msg.data, dtype=np.float32))
        with self.lock:
            own = any(tp == topic and v == vals and now - t0 < OWN_KEEP_S for t0, tp, v in self.own)
            if not own:
                self.foreign.append((now, topic))

    def foreign_since(self, t):
        with self.lock:
            return [x for x in self.foreign if x[0] >= t]

    def other_publishers(self):
        """自分以外の publisher の数 {topic: n}（送っていない GUI も数に入るので、これだけでは中断しない）"""
        out = {TOPIC_TARGET: self.node.count_publishers(TOPIC_TARGET) - 1}
        for k in range(1, len(BOARD_LAYOUT) + 1):
            out[TOPIC_FF.format(k)] = self.node.count_publishers(TOPIC_FF.format(k)) - 1
        return out

    # ---------------- 送信（float32 に丸めて記録） ----------------
    def _publish(self, pub, topic, data):
        vals = [float(v) for v in np.asarray(data, dtype=np.float32)]
        with self.lock:
            self.own.append((time.monotonic(), topic, tuple(vals)))
        msg = self._F32()
        msg.data = vals
        pub.publish(msg)
        self.sent_any = True
        return vals

    def send_targets(self, pose):
        """変更: float32 に丸めて送り、送った値を記録する"""
        return self._publish(self.pub_target, TOPIC_TARGET, pose)

    @staticmethod
    def ff_tables(ff_rows, T):
        """移植した send_ff と同じ並び（各ボード 36要素 = 6スロット × [a,b,c,d,e,T]、FF のないスロットは [0,0,0,0,0,T]）"""
        rows = np.asarray(ff_rows, dtype=float).reshape(N_FF, 5)
        out = {}
        for k, (board, _offset, _count) in enumerate(BOARD_LAYOUT, start=1):
            data = []
            n_ff = FF_BOARD_DOFS[board]
            first = sum(FF_BOARD_DOFS[b] for b, _, _ in BOARD_LAYOUT[:k - 1])
            for slot in range(6):
                if slot < n_ff:
                    data.extend([float(v) for v in rows[first + slot]] + [float(T)])
                else:
                    data.extend([0.0, 0.0, 0.0, 0.0, 0.0, float(T)])
            out[k] = [float(v) for v in np.asarray(data, dtype=np.float32)]
        return out

    def send_ff_tables(self, tables):
        for k, data in tables.items():
            self.last_ff[k] = self._publish(self.pub_ff[k], TOPIC_FF.format(k), data)

    def send_ff(self, ff_rows, T):
        """変更: float32 に丸めて送り、送った値（ボードごとの36要素）を返す"""
        tables = self.ff_tables(ff_rows, T)
        self.send_ff_tables(tables)
        return tables

    def stop_ff(self, log=None):
        """FF を2段で止める（§7.2）。① 係数はそのままで T=0（T は1語なので、受信と読み出しが重なっても「旧 T」か「T=0」の
        どちらかにしかならない）→ ② 0.05 s 後に係数0・T=0（T=0 なので係数が混ざっても出力は 0）"""
        first = {}
        for k in self.last_ff:
            d = list(self.last_ff[k] or [0.0] * 36)
            for s in range(6):
                d[6 * s + 5] = 0.0
            first[k] = d
        self.send_ff_tables(first)
        time.sleep(0.05)
        self.send_ff_tables({k: [0.0] * 36 for k in self.last_ff})
        if log:
            log("  FF を止めました（T=0 → 係数0・T=0）")

    # ---------------- 受信の状態 ----------------
    def pot_age(self, board):
        with self.lock:
            p = self.last_pot[board]
        return float('inf') if p is None else time.monotonic() - p[0]

    def stalled(self, boards, max_age=0.5):
        return [b for b in boards if self.pot_age(b) > max_age]

    def continuous(self, board, need=1.0, max_gap=0.1):
        """直近 need 秒、POT の受信が max_gap 以上途切れていないか"""
        now = time.monotonic()
        with self.lock:
            ts = [m[0] for m in self.ring[board]['pot'] if m[0] >= now - need - max_gap]
        if len(ts) < 2 or now - ts[-1] > max_gap or ts[0] > now - need + max_gap:
            return False
        return float(np.max(np.diff(ts))) < max_gap

    def wait_recovery(self, boards, stop_event, log, need=1.0, give_up=60.0):
        """受信が need 秒続けて途切れなくなるまで待つ（§7.3b）。give_up 秒で False"""
        if all(self.continuous(b, need) for b in boards):
            return True
        log(f"  受信が止まっています（{self.stalled(boards)}）。回復を待ちます（最大 {give_up:g} 秒）")
        t_end = time.monotonic() + give_up
        while time.monotonic() < t_end:
            _sleep_checked(0.1, stop_event)
            if all(self.continuous(b, need) for b in boards):
                log("  受信が回復しました")
                return True
        return False

    def wait_echo(self, dofs, values, timeout=0.5):
        """目標値のエコー（/tk の [6+i]）が round(values) になるまで待つ"""
        t_end = time.monotonic() + timeout
        want = {d: int(round(float(values[d]))) for d in dofs}
        while True:
            cur = self.board_targets(max_age=0.5)
            if all(cur[d] is not None and int(round(cur[d])) == want[d] for d in dofs):
                return True
            if time.monotonic() >= t_end:
                return False
            time.sleep(0.01)

    def capture_counts_since(self, since):
        """記録中の、時刻 since より後の /tk の受信数（ボードごと。配信数の目安）"""
        with self.lock:
            cap = self.capture
            if cap is None:
                return {}
            return {b: sum(1 for m in cap[b]['pot'] if m[0] > since) for b, _, _ in BOARD_LAYOUT}

    def capture_slice(self, board, kind, start):
        with self.lock:
            cap = self.capture
            if cap is None:
                return []
            return list(cap[board][kind][start:])

    def recent_pot(self, board, window):
        now = time.monotonic()
        with self.lock:
            return [m[1] for m in self.ring[board]['pot'] if m[0] >= now - window]


# ==============================================================================
# 1試行の実行（計画書 §7）
# ==============================================================================
@dataclass
class SafetyConfig:
    sensor_lo: float = 2.0          # センサ値の有効範囲（10 bit ADC）。外は故障とみなす
    sensor_hi: float = 1021.0
    joint: list = None              # 関節の可動範囲 26個の (lo, hi)。既定は POT_RANGE
    dev_const: float = 50.0         # 偏差の上限 |P − Pf| > 1.5|ΔP| + dev_const
    dev_factor: float = 1.5
    rest_tol: float = 3.0           # 静止判定の許容ずれ [count]（基準は P0_ref）
    rest_band: float = 3.0          # 静止判定の幅 [count] / 0.5 s
    still_wait: float = 5.0         # 安全な戻しで静止を待つ上限 [s]
    missing_tol: int = 0            # 評価区間で欠損してよい配信の数
    remeasure_max: int = 3          # 1試行あたりの測り直しの上限
    quiet_s: float = 5.0            # 他プログラムの送信が止まってから待つ時間 [s]

    def joint_range(self, d):
        return tuple(self.joint[d]) if self.joint else tuple(POT_RANGE[d])


class UnsafeAbort(Exception):
    """FF が原因の中断（関節の可動範囲外・偏差の上限超え）"""


class SensorFault(Exception):
    """センサ値が有効範囲外（故障とみなす）"""


class CommFail(Exception):
    """測り直しの上限を超えた・受信が回復しない（study を止める）"""


def wait_still(link, dofs, max_wait, band=3.0, window=0.5):
    """対象の自由度の POT が window 秒で band 以内に収まるまで待つ（上限 max_wait。安全のための待ちなので停止要求では抜けない）"""
    t_end = time.monotonic() + max_wait
    while time.monotonic() < t_end:
        ok = True
        for d in dofs:
            _k, board, local = DOF_BOARD[d]
            vals = [v[local] for v in link.recent_pot(board, window)]
            if len(vals) < 10 or max(vals) - min(vals) > band:
                ok = False
                break
        if ok:
            return True
        time.sleep(0.05)
    return False


def safe_return(link, initial, dofs, safety, log):
    """安全な戻し（§7.2）: FF を2段で止める → 静止を待つ（上限あり）→ 初期姿勢を送り、エコーを確かめる。
    逆向きのステップをすぐには出さない（静止を待ってから送る）。停止要求があっても最後まで行う。"""
    link.stop_ff(log)
    if dofs and not wait_still(link, dofs, safety.still_wait, safety.rest_band):
        log(f"  警告: {safety.still_wait:g} 秒待っても静止しませんでした。初期姿勢を送ります")
    ctrl = [d for d in range(N_DOF) if d not in NO_FF_DOFS]
    for attempt in range(3):
        link.send_targets(initial)
        link.last_return = time.monotonic()
        if link.wait_echo(ctrl, initial, timeout=0.5):
            log("  初期姿勢へ戻しました（エコーを確認）")
            return True
        log(f"  警告: 初期姿勢のエコーを確認できません（{attempt + 1}回目）")
    return False


@dataclass
class Measurement:
    rec: TrialRecord
    series: dict
    info: dict
    t_send: float
    ff_tables: dict
    rest_ok: bool
    attempts: list


def _monitor(link, dofs, initial, target, safety, starts):
    """測定中の監視（§7.4）。新しく届いた POT を調べ、範囲外なら例外"""
    by_board = {}
    for d in dofs:
        by_board.setdefault(DOF_BOARD[d][1], []).append(d)
    for board, bdofs in by_board.items():
        new = link.capture_slice(board, 'pot', starts.get(board, 0))
        starts[board] = starts.get(board, 0) + len(new)
        for _t, v in new:
            for d in bdofs:
                local = DOF_BOARD[d][2]
                P = float(v[local])
                if not (safety.sensor_lo <= P <= safety.sensor_hi):
                    raise SensorFault(f"DOF {d}: センサ値 {P:g} が有効範囲 [{safety.sensor_lo:g}, {safety.sensor_hi:g}] の外")
                lo, hi = safety.joint_range(d)
                if not (lo <= P <= hi):
                    raise UnsafeAbort(f"DOF {d}: POT {P:g} が関節の可動範囲 [{lo:g}, {hi:g}] の外")
                lim = safety.dev_factor * abs(float(target[d]) - float(initial[d])) + safety.dev_const
                if abs(P - float(target[d])) > lim:
                    raise UnsafeAbort(f"DOF {d}: |P − Pf| = {abs(P - float(target[d])):.0f} > 偏差の上限 {lim:.0f}")


def measure_once(link, initial, target, dofs, ff_rows, T, wait_s, safety, stop_event, log, rest_ref=None):
    """1回の測定（§7.1）。成功すれば Measurement。測り直すべき問題は TrialIssue('comm') を投げる（e.raw に生データ）"""
    boards = sorted({DOF_BOARD[d][1] for d in dofs})
    N_e = n_eval(T)
    need = N_e + 1 + MARGIN_TICKS
    started = time.strftime('%Y-%m-%d %H:%M:%S')
    cur = link.board_targets()
    if link.last_return is None or any(v is None or abs(v - float(i)) >= 0.5 for v, i in zip(cur, initial)):
        safe_return(link, initial, dofs, safety, log)
    if link.stalled(boards) and not link.wait_recovery(boards, stop_event, log):
        raise CommFail(f"受信が回復しません: {link.stalled(boards)}")
    wait = max(float(wait_s), float(T) + 0.5)
    remain = wait - (time.monotonic() - link.last_return)
    if remain > 0:
        log(f"  初期姿勢で待機 {remain:.1f} 秒")
        _sleep_checked(remain, stop_event)
    ref = list(initial) if rest_ref is None else [rest_ref.get(d, initial[d]) for d in range(N_DOF)]
    rest_ok = wait_rest(link, ref, dofs, stop_event, log, band=safety.rest_band,
                        offset_tol=safety.rest_tol if rest_ref is not None else 8.0)
    params = link.adrc_params()
    missing = [d for d in dofs if params.get(d) is None or not params[d].valid()]
    if missing:
        raise TrialIssue('comm', f"ADRCパラメータを実機から受信できない自由度があります: DOF {missing}")
    if link.foreign_since(time.monotonic() - safety.quiet_s):
        log(f"  他のプログラムが目標値・FF を送っています。{safety.quiet_s:g} 秒止まるまで待ちます")
        t_end = time.monotonic() + 60.0
        while link.foreign_since(time.monotonic() - safety.quiet_s):
            if time.monotonic() > t_end:
                raise TrialIssue('comm', "他のプログラムの送信が止まりません")
            _sleep_checked(0.2, stop_event)
    raw = None
    t_send = time.monotonic()
    try:
        tables = link.send_ff(ff_rows, T)
        time.sleep(0.15)
        link.start_capture(HISTORY_S)
        time.sleep(0.05)
        t_send = time.monotonic()
        link.send_targets(target)
        moved = [d for d in dofs if int(round(float(target[d]))) != int(round(float(initial[d])))]
        if not link.wait_echo(moved, target, timeout=0.5):
            raise TrialIssue('comm', "目標値のエコーが 0.5 秒以内に変わりません")
        log(f"  目標値を送信しました。評価区間 {eval_time(T):.2f} 秒（max(1.5T, 5 秒)、{N_e + 1} 配信）測定します")
        timeout = need * TICK_DT * 1.3 + 3.0
        starts = {}
        while True:
            _sleep_checked(0.02, stop_event)
            _monitor(link, dofs, initial, target, safety, starts)
            st = link.stalled(boards)
            if st:
                raise TrialIssue('comm', f"測定中に受信が止まりました: {st}")
            if link.foreign_since(t_send + 0.01):
                raise TrialIssue('comm', "測定中に他のプログラムが目標値・FF を送りました")
            cnt = link.capture_counts_since(t_send)
            if all(cnt.get(b, 0) >= need for b in boards):
                break
            if time.monotonic() - t_send > timeout:
                log(f"  警告: 配信数がそろわないまま測定を終えます（{cnt}）")
                break
        raw = link.stop_capture()
    except BaseException as exc:
        if link.capture is not None:
            raw = link.stop_capture()
        try:
            safe_return(link, initial, dofs, safety, log)
        finally:
            exc.raw = raw
            exc.t_send = t_send
            exc.params = params                       # 中断した回の npz にも、試行開始時・終了時の ADRC パラメータを残す
            exc.params_after = link.adrc_params()
        raise
    safe_return(link, initial, dofs, safety, log)
    rec = TrialRecord(raw=raw, initial=list(initial), target=list(target), ff=np.asarray(ff_rows, dtype=float),
                      T=float(T), L=TICK_DT, params=params, params_after=link.adrc_params(), t_target=t_send,
                      started_at=started)
    try:
        series, info = extract_series(raw, initial, target, dofs, t_send, T)
    except TrialIssue as exc:
        exc.raw, exc.t_send, exc.rec = raw, t_send, rec
        exc.params, exc.params_after = rec.params, rec.params_after
        raise
    bad = {d: s.missing() for d, s in series.items() if s.missing() > safety.missing_tol}
    if bad:
        exc = TrialIssue('comm', f"評価区間で配信が欠けています（許容 {safety.missing_tol}）: {bad}")
        exc.raw, exc.t_send, exc.rec = raw, t_send, rec
        exc.params, exc.params_after = rec.params, rec.params_after
        raise exc
    return Measurement(rec=rec, series=series, info=info, t_send=t_send, ff_tables=tables, rest_ok=rest_ok, attempts=[])


def measure_with_retries(link, initial, target, dofs, ff_rows, T, wait_s, safety, stop_event, log, rest_ref=None,
                         on_attempt=None):
    """measure_once を、測り直すべき問題（TrialIssue('comm')）のときだけ上限まで繰り返す（§7.3b）。

    on_attempt(a, raw, t_send, reason, params, params_after) は失敗した回の生データを保存するために呼ぶ
    （params・params_after は試行開始時・終了時の ADRC パラメータ。読む前に失敗したときは None）。上限を超えたら CommFail。
    """
    attempts = []
    for a in range(safety.remeasure_max + 1):
        try:
            m = measure_once(link, initial, target, dofs, ff_rows, T, wait_s, safety, stop_event, log, rest_ref)
            m.attempts = attempts
            return m
        except TrialIssue as exc:
            if exc.kind != 'comm':
                raise
            attempts.append(exc.reason)
            if on_attempt is not None:
                on_attempt(a, getattr(exc, 'raw', None), getattr(exc, 't_send', None), exc.reason,
                           getattr(exc, 'params', None), getattr(exc, 'params_after', None))
            log(f"  測り直します（{a + 1}/{safety.remeasure_max}）: {exc.reason}")
            if a >= safety.remeasure_max:
                err = CommFail(f"測り直しが上限 {safety.remeasure_max} 回を超えました: {attempts}")
                err.attempts = attempts
                raise err
            if link.stalled(sorted({DOF_BOARD[d][1] for d in dofs})):
                if not link.wait_recovery(sorted({DOF_BOARD[d][1] for d in dofs}), stop_event, log):
                    err = CommFail("受信が回復しません")
                    err.attempts = attempts
                    raise err
    raise CommFail("測り直しの上限を超えました")


# ==============================================================================
# 移植：保存（optimize_EL2_common.py）
# ==============================================================================
# 移植元: optimize_EL2_common.py:2155-2155 @f8fba60
CSV_ENCODING = 'utf-8-sig'      # CSV は BOM 付き UTF-8（Excel で開いても日本語が文字化けしないように）


# 移植元: optimize_EL2_common.py:2181-2243 @f8fba60
def results_header():
    """既存 optimize_EL_ff_ROS2.py の結果CSV（save_optimal_results_to_csv）と同じ列名（n は FF番号 1〜24）"""
    h = []
    for n in range(1, N_FF + 1):
        h += [f'Init_{n}', f'Target_{n}', f'T_{n}', f't1_{n}', f'y1_{n}', f't2_{n}', f'y2_{n}']
    return h


def write_results_csv(path, initial, target, T, ff_rows, append=False):
    """最良の試行で与えたFFの極値（既存と同じ形式）。極値のない自由度は (0.33T, 0, 0.66T, 0)"""
    row = []
    for n, d in enumerate(FF_DOF_MAP):
        t1, y1, t2, y2 = extrema_of(ff_rows[n], T)
        row += [float(initial[d]), float(target[d]), float(T), float(t1), float(y1), float(t2), float(y2)]
    exists = os.path.exists(path) and os.path.getsize(path) > 0
    enc = 'utf-8' if (append and exists) else CSV_ENCODING              # 追記のときはファイルの途中に BOM を入れない
    with open(path, 'a' if append else 'w', newline='', encoding=enc) as f:
        w = csv.writer(f)
        if not (append and exists):
            w.writerow(results_header())
        w.writerow(row)


def write_kv_csv(path, rows):
    """条件の表。rows = [(区分, 項目, 値, 説明), ...]"""
    with open(path, 'w', newline='', encoding=CSV_ENCODING) as f:
        w = csv.writer(f)
        w.writerow(['section', 'key', 'value', 'note'])
        for r in rows:
            w.writerow(['' if v is None else v for v in r])


def write_dict_rows(path, rows):
    """辞書のリストを CSV にする（列はすべての行の項目の和集合、最初に出た順）"""
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, 'w', newline='', encoding=CSV_ENCODING) as f:
        w = csv.writer(f)
        w.writerow(keys)
        for r in rows:
            w.writerow([_cell(r.get(k)) for k in keys])


def _cell(v):
    if v is None:
        return ''
    if isinstance(v, (float, np.floating)):
        return '' if not np.isfinite(v) else '%.9g' % v
    return v


def _record_header(k, kind, offset, count):
    """GUI_android_tk.record_header と同じ列名（z1・z2・derivatives も同じ規則）"""
    dofs = range(offset, offset + count)
    if kind == 'pot':
        return [f"Time{k}"] + [f"POT{d}" for d in dofs] + [f"POTdesired{d}" for d in dofs]
    if kind == 'pwm':
        return [f"Time{k}_PWM"] + [f"PWM{d}" for d in dofs]
    name = CSV_KIND[kind]
    return [f"Time{k}_{name}"] + [f"{name}_{d}" for d in dofs]


# 移植元: optimize_EL2_common.py:2375-2404 @f8fba60
def save_trial_npz(path, rec):
    """試行の受信データとメタ情報を npz に保存する（load_trial_npz で読み戻して解析し直せる）"""
    data = {'initial': np.array(rec.initial, dtype=float), 'target': np.array(rec.target, dtype=float), 'ff': rec.ff,
            'T': rec.T, 'L': rec.L, 'started_at': rec.started_at, 'aborted': rec.aborted, 't_target': rec.t_target}
    for d, p in rec.params.items():
        if p is not None:
            data[f'param_{d}'] = np.array(p.as_list())
    for board, kinds in rec.raw.items():
        for kind, (t, V) in kinds.items():
            data[f'{board}_{kind}_t'] = t
            data[f'{board}_{kind}_v'] = V
    with open(path, 'wb') as f:
        np.savez(f, **data)


def load_trial_npz(path, L=None):
    """save_trial_npz で保存した試行を TrialRecord に戻す（L を与えるとその周回間隔で解析する）"""
    with np.load(path, allow_pickle=False) as z:
        d = {k: z[k] for k in z.files}
    raw = {}
    for board, _o, _c in BOARD_LAYOUT:
        kinds = {}
        for kind in KINDS:
            if f'{board}_{kind}_t' in d:
                kinds[kind] = (d[f'{board}_{kind}_t'], d[f'{board}_{kind}_v'])
        raw[board] = kinds
    params = {dof: (AdrcParams(*d[f'param_{dof}']) if f'param_{dof}' in d else None) for dof in range(N_DOF)}
    return TrialRecord(raw=raw, initial=list(d['initial']), target=list(d['target']), ff=d['ff'], T=float(d['T']),
                       L=float(d['L'] if L is None else L), params=params, params_after=params,
                       t_target=float(d['t_target']), started_at=str(d['started_at']), aborted=bool(d['aborted']))


# 移植元: optimize_EL2_common.py:2161-2170 @f8fba60
def make_run_folder(base, prefix='optimize'):
    """<base>/<prefix>_YYYYMMDD_HHMM を作る（同じ名前があれば _2, _3 … を付ける）"""
    name = time.strftime(f'{prefix}_%Y%m%d_%H%M')
    path = os.path.join(base, name)
    n = 2
    while os.path.exists(path):
        path = os.path.join(base, f"{name}_{n}")
        n += 1
    os.makedirs(os.path.join(path, TRIALS_DIR))           # 変更: ビューア用の loops/ の代わりに trials/ を作る
    return path


TRIALS_DIR = 'trials'


# 移植元: optimize_EL2_common.py:2246-2294 @f8fba60
def write_trial_timeseries(path, rec, ff_time_by_board):
    """試行の受信データそのもの（GUI_android_tk.py の記録CSVと同じ列 ＋ PWM の組に FF{d}・ADRC{d}、
    その後ろに z1・z2・derivatives の組、最後に送った目標値 Time_cmd, cmd{d}）。

    時刻は目標値を送った瞬間からの秒数（PCの時計。WSL2 では数%遅れることがある）。
    変更: FF{d} は周回番号ではなく配信の時刻で計算する。ff_time_by_board[board] は各 PWM メッセージの FF の時刻 [s]
    （配信番号 j に対して (j − j0)·0.010 − 0.010。PWM は POT より1周回古いので 10 ms 前の FF を引く近似）。
    """
    fi = lambda v: '' if not np.isfinite(v) else str(int(round(v)))
    ff = lambda v: '' if not np.isfinite(v) else '%.7g' % v
    blocks = []
    extra = []
    for k, (board, offset, count) in enumerate(BOARD_LAYOUT, start=1):
        raw = rec.raw.get(board, {})
        dofs = list(range(offset, offset + count))
        t0 = rec.t_target
        t, V = raw.get('pot', (np.zeros(0), np.zeros((0, 2 * count))))
        blocks.append((_record_header(k, 'pot', offset, count),
                       [[f"{t[i] - t0:.6f}"] + [fi(x) for x in V[i]] for i in range(len(t))]))
        t, V = raw.get('pwm', (np.zeros(0), np.zeros((0, count))))
        tff = ff_time_by_board.get(board)
        FFv = np.zeros((len(t), count))
        if tff is not None and len(t):
            for j, d in enumerate(dofs):
                if d in FF_INDEX:
                    FFv[:, j] = ff_eval(rec.ff[FF_INDEX[d]], rec.T, np.asarray(tff, dtype=float))
        header = _record_header(k, 'pwm', offset, count) + [f"FF{d}" for d in dofs] + [f"ADRC{d}" for d in dofs]
        blocks.append((header, [[f"{t[i] - t0:.6f}"] + [ff(x) for x in V[i]] + [ff(x) for x in FFv[i]]
                                + [ff(x) for x in V[i] - FFv[i]] for i in range(len(t))]))
        t, V = raw.get('z3', (np.zeros(0), np.zeros((0, count))))
        blocks.append((_record_header(k, 'z3', offset, count),
                       [[f"{t[i] - t0:.6f}"] + [ff(x) for x in V[i]] for i in range(len(t))]))
        for kind in ('z1', 'z2', 'der'):
            t, V = raw.get(kind, (np.zeros(0), np.zeros((0, count))))
            extra.append((_record_header(k, kind, offset, count),
                          [[f"{t[i] - t0:.6f}"] + [ff(x) for x in V[i]] for i in range(len(t))]))
    blocks += extra
    blocks.append((["Time_cmd"] + [f"cmd{d}" for d in range(N_DOF)],
                   [["0.000000"] + [ff(float(v)) for v in rec.target]]))
    n_rows = max((len(r) for _, r in blocks), default=0)
    with open(path, 'w', newline='', encoding=CSV_ENCODING) as f:
        w = csv.writer(f)
        w.writerow([name for h, _ in blocks for name in h])
        for i in range(n_rows):
            row = []
            for h, rows in blocks:
                row.extend(rows[i] if i < len(rows) else [''] * len(h))
            w.writerow(row)


def ff_times_for_csv(rec, info):
    """write_trial_timeseries に渡す、PWM メッセージごとの FF の時刻 [s]（配信の時間軸。10 ms 前の FF）"""
    out = {}
    for board, bi in info.items():
        kinds = rec.raw.get(board, {})
        if 'pwm' not in kinds or 'pot' not in kinds or not len(kinds['pwm'][0]):
            continue
        j_tk, _u, _n = assign_ticks(kinds['pot'][0])
        jp = align_ticks(kinds['pwm'][0], kinds['pot'][0], j_tk)
        out[board] = (jp - bi['j0']) * TICK_DT - TICK_DT
    return out


def save_trial_npz_ext(path, rec, extra):
    """移植した save_trial_npz の内容に、配信番号・送った float32 の FF などを x_ で始まる名前で足す（load_trial_npz で読める）。

    ADRC パラメータは試行開始時を param_{DOF}（移植元と同じ名前）、試行の終わり（初期姿勢へ戻した後）を x_param_after_{DOF}
    に [kp, kd, ic, λ] で保存する。2つを比べれば、試行中に他のプログラムがパラメータを変えたかがわかる。
    """
    data = {'initial': np.array(rec.initial, dtype=float), 'target': np.array(rec.target, dtype=float), 'ff': rec.ff,
            'T': rec.T, 'L': rec.L, 'started_at': rec.started_at, 'aborted': rec.aborted, 't_target': rec.t_target}
    for d, p in (rec.params or {}).items():
        if p is not None:
            data[f'param_{d}'] = np.array(p.as_list())
    for d, p in (rec.params_after or {}).items():
        if p is not None:
            data[f'x_param_after_{d}'] = np.array(p.as_list())
    for board, kinds in (rec.raw or {}).items():
        for kind, (t, V) in kinds.items():
            data[f'{board}_{kind}_t'] = t
            data[f'{board}_{kind}_v'] = V
    for k, v in (extra or {}).items():
        data['x_' + k] = np.asarray(v) if not isinstance(v, str) else v
    tmp = path + '.tmp'
    with open(tmp, 'wb') as f:
        np.savez(f, **data)
    os.replace(tmp, path)


def save_raw_attempt(path, raw, initial, target, ff_rows, T, t_send, note, params=None, params_after=None):
    """測り直した回・中断した回の生データ（TrialRecord がない場合）。

    完了した試行の npz と同じく、送った FF（float32 に丸めた値）を x_ff_sent_rows に、試行開始時・終了時の ADRC パラメータを
    param_{DOF}・x_param_after_{DOF} に入れる（ADRC パラメータを読む前に失敗した回は入らない）。
    """
    ff = np.asarray(ff_rows, dtype=float)
    rec = TrialRecord(raw=raw or {}, initial=list(initial), target=list(target), ff=ff,
                      T=float(T), L=TICK_DT, params=params or {}, params_after=params_after or {},
                      t_target=float(t_send or 0.0), started_at=time.strftime('%Y-%m-%d %H:%M:%S'), aborted=True,
                      note=note)
    save_trial_npz_ext(path, rec, {'note': note, 'ff_sent_rows': ff.astype(np.float32).astype(float)})


def write_json_atomic(path, data):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2, default=_json_default)
    os.replace(tmp, path)


def _json_default(o):
    if isinstance(o, np.generic):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(type(o))


def write_status(folder, state, reason='', **extra):
    """status.json（running / completed / stopped / error と理由）を一時ファイル経由で置き換える"""
    data = {'state': state, 'reason': reason, 'updated': time.strftime('%Y-%m-%d %H:%M:%S'), 'pid': os.getpid()}
    data.update(extra)
    write_json_atomic(os.path.join(folder, 'status.json'), data)


class TrialsCsv:
    """trials.csv（全試行。FAIL も含む）。試行ごとに1行追記して flush する"""

    def __init__(self, path, columns):
        self.path = path
        if os.path.exists(path) and os.path.getsize(path) > 0:
            with open(path, newline='', encoding='utf-8-sig') as f:
                head = next(csv.reader(f), None)
            self.columns = head or list(columns)
            for c_ in columns:
                if c_ not in self.columns:
                    self.columns.append(c_)
        else:
            self.columns = list(columns)
            with open(path, 'w', newline='', encoding=CSV_ENCODING) as f:
                csv.writer(f).writerow(self.columns)

    def append(self, row):
        with open(self.path, 'a', newline='', encoding='utf-8') as f:
            csv.writer(f).writerow([_cell(row.get(k)) for k in self.columns])
            f.flush()
            os.fsync(f.fileno())


def trial_columns(dofs):
    cols = ['trial', 'state', 'fail_class', 'fail_reason', 'fail_stage', 'source', 'retry_of', 'confirm_of',
            'remeasure', 'started_at', 'finished_at', 'J', 'log10J', 'RMS_percent', 'best_trial_so_far',
            'best_J_so_far', 'J_partial', 'missing_max']
    for d in dofs:
        p = f'd{d}_'
        cols += [p + k for k in ('u1', 'u2', 'u3', 'A', 's1', 's2', 'r', 't1', 'y1', 't2', 'y2',
                                 'a', 'b', 'c', 'd', 'e', 'T', 't1_sent', 'y1_sent', 't2_sent', 'y2_sent',
                                 'J_i', 'P0', 'delta_P0', 'missing', 'ff_max', 'ff_end', 'ff_step')]
    return cols


def write_grid_csv(path, series, targets, ff_rows, T):
    """配信の時間軸の解析用データ（j − j0、t と、自由度ごとの POT・Target・PWM・FF・ADRC・z1・z2・z3・derivatives・有効）"""
    dofs = sorted(series)
    if not dofs:
        return
    s0 = series[dofs[0]]
    head = ['j', 't']
    for d in dofs:
        head += [f'POT{d}', f'Target{d}', f'PWM{d}', f'FF{d}', f'ADRC{d}', f'z1_{d}', f'z2_{d}', f'z3_{d}',
                 f'derivatives_{d}', f'valid{d}']
    rows = []
    cols = {}
    for d in dofs:
        s = series[d]
        tgt = np.full(len(s.jrel), np.nan)
        tgt[s.win()] = targets[d].seq() if d in targets else np.nan
        ffv = ff_eval(ff_rows[FF_INDEX[d]], T, s.t - TICK_DT)
        cols[d] = (s.pot, tgt, s.pwm, ffv, s.pwm - ffv, s.z1, s.z2, s.z3, s.der, np.isfinite(s.pot))
    for i in range(len(s0.jrel)):
        r = [int(s0.jrel[i]), f"{s0.jrel[i] * TICK_DT:.3f}"]
        for d in dofs:
            vals = cols[d]
            r += [_cell(float(v[i])) for v in vals[:-1]] + [int(vals[-1][i])]
        rows.append(r)
    with open(path, 'w', newline='', encoding=CSV_ENCODING) as f:
        w = csv.writer(f)
        w.writerow(head)
        w.writerows(rows)


# ==============================================================================
# Optuna の study（計画書 §6）
# ==============================================================================
SAMPLERS = ('TPE', 'CMA-ES', 'GP', 'Random', 'QMC')


def torch_available():
    try:
        import torch  # noqa: F401
        return True, ''
    except Exception as exc:
        return False, f"torch を import できません（{exc}）"


def condition_hash(cond):
    return hashlib.sha256(json.dumps(cond, sort_keys=True, ensure_ascii=False, default=_json_default)
                          .encode('utf-8')).hexdigest()


def make_sampler(name, seed, n_startup, x0=None):
    import optuna
    if name == 'TPE':
        return optuna.samplers.TPESampler(multivariate=True, n_startup_trials=n_startup, seed=seed), \
            {'multivariate': True, 'n_startup_trials': n_startup, 'seed': seed}
    if name == 'CMA-ES':
        return optuna.samplers.CmaEsSampler(x0=x0, sigma0=0.2, seed=seed), {'x0': x0, 'sigma0': 0.2, 'seed': seed}
    if name == 'GP':
        ok, why = torch_available()
        if not ok:
            raise RuntimeError(why)
        return optuna.samplers.GPSampler(seed=seed, n_startup_trials=n_startup, deterministic_objective=False), \
            {'seed': seed, 'n_startup_trials': n_startup, 'deterministic_objective': False}
    if name == 'Random':
        return optuna.samplers.RandomSampler(seed=seed), {'seed': seed}
    if name == 'QMC':
        return optuna.samplers.QMCSampler(qmc_type='sobol', scramble=True, seed=seed), \
            {'qmc_type': 'sobol', 'scramble': True, 'seed': seed}
    raise ValueError(f"不明な Sampler: {name}")


def resume_seed(name, seed, n_complete):
    """再開時の seed（§6.6）: QMC・CMA-ES は元の seed（列の位置・状態が study に保存されている）、他は seed + 完了数"""
    return int(seed) if name in ('QMC', 'CMA-ES') else int(seed) + int(n_complete)


def _jsonable(v):
    if isinstance(v, (np.floating, np.integer)):
        return v.item()
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, float) and not math.isfinite(v):
        return None
    if isinstance(v, dict):
        return {str(k): _jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    return v


class FFStudy:
    """journal に保存する study と、失敗試行の管理（§6.7）・近傍棄却（§6.8）・確認試行（§5.5）"""

    def __init__(self, folder, dofs, A_max, sampler_name, seed, n_startup, ffmap, rho=0.05, tabu_on=True,
                 resume=False, x0=None, log=print):
        import optuna
        from optuna.storages import JournalStorage
        from optuna.storages.journal import JournalFileBackend
        optuna.logging.set_verbosity(optuna.logging.WARNING)
        self.optuna = optuna
        self.folder = folder
        self.dofs = list(dofs)
        self.A_max = float(A_max)
        self.ffmap = ffmap
        self.rho = float(rho)
        self.tabu_on = bool(tabu_on)
        self.log = log
        self.sampler_name = sampler_name
        self.seed0 = int(seed)
        self.n_startup = int(n_startup)
        self.dists = {}
        for d in self.dofs:
            for k in ('u1', 'u2', 'u3'):
                self.dists[f'd{d}_{k}'] = optuna.distributions.FloatDistribution(0.0, 1.0)
            self.dists[f'd{d}_A'] = optuna.distributions.FloatDistribution(-self.A_max, self.A_max)
        self.storage = JournalStorage(JournalFileBackend(os.path.join(folder, 'study.journal')))
        if resume:
            probe = optuna.load_study(study_name=STUDY_NAME, storage=self.storage)
            n_c = len(probe.get_trials(deepcopy=False, states=(optuna.trial.TrialState.COMPLETE,)))
            self.seed = resume_seed(sampler_name, seed, n_c)
            self.sampler, self.sampler_args = make_sampler(sampler_name, self.seed, self.n_startup, x0)
            self.study = optuna.load_study(study_name=STUDY_NAME, storage=self.storage, sampler=self.sampler)
        else:
            self.seed = int(seed)
            self.sampler, self.sampler_args = make_sampler(sampler_name, self.seed, self.n_startup, x0)
            self.study = optuna.create_study(study_name=STUDY_NAME, storage=self.storage, sampler=self.sampler,
                                             direction='minimize')
        self.tabu = []
        self._rebuild_tabu()

    # ---------------- 補助 ----------------
    def TS(self):
        return self.optuna.trial.TrialState

    def u_of(self, params):
        return {d: {k: float(params[f'd{d}_{k}']) for k in ('u1', 'u2', 'u3', 'A')} for d in self.dofs}

    def params_of(self, u):
        return {f'd{d}_{k}': float(u[d][k]) for d in self.dofs for k in ('u1', 'u2', 'u3', 'A')}

    def trials(self, states=None):
        return self.study.get_trials(deepcopy=False, states=states)

    def n_complete(self):
        """実機で測った COMPLETE の数（最大試行数の判定に使う。過去の試行の add_trial（source='past'）は数えない）"""
        return sum(1 for t in self.trials((self.TS().COMPLETE,)) if t.user_attrs.get('source') != 'past')

    def _rebuild_tabu(self):
        self.tabu = []
        for t in self.trials((self.TS().FAIL,)):
            if t.user_attrs.get('fail_class') == 'unsafe' and t.params:
                self.add_tabu(self.u_of(t.params))

    def add_tabu(self, u):
        self.tabu.append({d: self.ffmap.normalized(u[d]) for d in self.dofs})

    def tabu_hit(self, u):
        """正規化した空間で、DOF ごとのユークリッド距離の最大値が ρ 以下の登録点があれば、その距離を返す"""
        if not self.tabu_on:
            return None
        x = {d: self.ffmap.normalized(u[d]) for d in self.dofs}
        for p in self.tabu:
            dist = max(float(np.linalg.norm(x[d] - p[d])) for d in self.dofs)
            if dist <= self.rho:
                return dist
        return None

    # ---------------- ask / tell ----------------
    def ask(self):
        trial = self.study.ask(fixed_distributions=self.dists)
        trial.set_user_attr('u', _jsonable(self.u_of(trial.params)))
        trial.set_user_attr('stage', 'presend')
        return trial

    def stage(self, trial, stage):
        trial.set_user_attr('stage', stage)

    def tell_complete(self, trial, J, attrs):
        for k, v in attrs.items():
            trial.set_user_attr(k, _jsonable(v))
        trial.set_user_attr('J', float(J))
        trial.set_user_attr('stage', 'done')
        self.study.tell(trial, math.log10(max(float(J), 1e-12)))

    def tell_fail(self, trial, fail_class, reason, attrs=None):
        """失敗（§6.7）。属性を先に書いてから tell(state=FAIL)（値は渡さない）"""
        assert fail_class in FAIL_CLASSES
        trial.set_user_attr('fail_stage', trial.user_attrs.get('stage', ''))
        for k, v in (attrs or {}).items():
            trial.set_user_attr(k, _jsonable(v))
        trial.set_user_attr('fail_class', fail_class)
        trial.set_user_attr('fail_reason', str(reason)[:2000])
        self.study.tell(trial, state=self.TS().FAIL)
        if fail_class == 'unsafe':
            self.add_tabu(self.u_of(trial.params))

    def add_confirm(self, params, J=None, attrs=None, fail_class=None, reason=''):
        """確認試行（§5.5）を ask を通さずに記録する（成功: COMPLETE、失敗: FAIL）"""
        ua = {k: _jsonable(v) for k, v in (attrs or {}).items()}
        if fail_class is None:
            ua['J'] = float(J)
            ft = self.optuna.trial.create_trial(params=params, distributions=self.dists,
                                                value=math.log10(max(float(J), 1e-12)), user_attrs=ua)
        else:
            ua.update({'fail_class': fail_class, 'fail_reason': str(reason)[:2000]})
            ft = self.optuna.trial.create_trial(state=self.TS().FAIL, params=params, distributions=self.dists,
                                                user_attrs=ua)
        self.study.add_trial(ft)
        return self.trials()[-1]

    def enqueue(self, u, attrs):
        self.study.enqueue_trial(self.params_of(u), user_attrs={k: _jsonable(v) for k, v in attrs.items()})

    def add_past(self, u, J, attrs):
        ua = {k: _jsonable(v) for k, v in attrs.items()}
        ua['J'] = float(J)
        ua['source'] = 'past'
        self.study.add_trial(self.optuna.trial.create_trial(params=self.params_of(u), distributions=self.dists,
                                                            value=math.log10(max(float(J), 1e-12)), user_attrs=ua))

    # ---------------- 再投入（同じ u を1回だけ） ----------------
    def retry(self, frozen, reason):
        if frozen.user_attrs.get('fail_class') not in RETRY_CLASSES or not frozen.params:
            return False
        if frozen.user_attrs.get('retry_of') is not None:
            return False                                  # 再投入した試行が再び失敗したら、もう入れない
        done = set(self.study.user_attrs.get('retried', []))
        if frozen.number in done:
            return False
        self.study.enqueue_trial(dict(frozen.params), user_attrs={'retry_of': frozen.number, 'source': 'retry'})
        done.add(frozen.number)
        self.study.set_user_attr('retried', sorted(done))
        self.log(f"  試行 {frozen.number} と同じ u を再投入しました（{reason}）")
        return True

    def handle_orphans(self):
        """再開時：RUNNING のまま残った試行を orphan の FAIL にし、同じ u を再投入する。stopped・comm も再投入する"""
        n = 0
        for t in self.trials((self.TS().RUNNING,)):
            tr = self.optuna.trial.Trial(self.study, t._trial_id)
            tr.set_user_attr('fail_stage', t.user_attrs.get('stage', ''))
            tr.set_user_attr('fail_class', 'orphan')
            tr.set_user_attr('fail_reason', '異常終了で RUNNING のまま残っていた')
            self.study.tell(t.number, state=self.TS().FAIL)
            n += 1
        for t in self.trials((self.TS().FAIL,)):
            if t.user_attrs.get('fail_class') in ('orphan', 'stopped', 'comm'):
                self.retry(t, f"再開時（{t.user_attrs.get('fail_class')}）")
        return n

    # ---------------- 結果 ----------------
    def completed(self):
        return [t for t in self.trials((self.TS().COMPLETE,)) if t.user_attrs.get('J') is not None]

    def best(self):
        """COMPLETE で J を持つ試行の中で J が最小のもの（FAIL は構造的に入らない）。
        過去の試行（source='past'）は測定データがこの保存先にないので除く"""
        done = [t for t in self.completed() if t.user_attrs.get('source') != 'past']
        return min(done, key=lambda t: t.user_attrs['J']) if done else None
