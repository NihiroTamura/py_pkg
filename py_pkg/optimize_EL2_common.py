#!/usr/bin/env python3
"""ADRC＋FF最適制御（_2 系）の共通部

optimize_EL_ff_ROS2_2.py（最適化本体）・optimize_TuningEL_ff_ROS2_2.py（重みの自動調整）・
view_optimization_2.py（ビューア）が共通に使う計算・通信・保存の処理。3つのプログラムで
モデル同定・FF関数・評価関数・ADRC内部シミュレーション・重み・時間刻み・データ形式を一致させるため、
同じ処理を各プログラムに書かず、必ずここを使うこと。

■ 実機の構成（ADRC_ROS2_tk_FF_ADRCparamChange-board1〜5.ino。ファームウェアは変更しない）
  制御スレッド thread_callback の1周（以下「周回」。間隔 LOOP_DT ≒ 0.011 s）で、次を1回ずつ行う。
    1. POT_realized = 直近18回のADC値の移動平均（以下 y）
    2. derivatives  = RCローパス(係数0.52)[ (25y_k − 48y_{k−1} + 36y_{k−2} − 16y_{k−3} + 3y_{k−4}) / (12h) ]、h = 0.001
    3. 配信用の配列へ格納（POT・目標値・derivatives はこの周回 k の値、PWM・z1・z2・z3 は前の周回 k−1 の計算結果）
    4. ESO（前進オイラー、式の中の dt = 0.001。入力項は前の周回の u_ADRC。FFは入らない）
    5. u_ADRC = (−z3 + kp(r − z1) − kd·derivatives) / ic（z1・z3 は更新後）、出力 = u_ADRC + FF(t)
       FF(t) は目標値が変わった周回を t=0 とする実時間（millis）の5次関数で、t ≥ T では 0
  ・dt = 0.001 と h = 0.001 は式の中の係数であって、実際の周回間隔ではない（0.011 に置き換えてはいけない）。
  ・ROS2 の配信（100 Hz）は制御とは別の時計で動いており、配信周期を周回間隔として使ってはいけない。
  ・周回間隔 LOOP_DT は実機から正確に測れないため 0.011 s を仮定値として使う（GUIで変更できる）。
    周回間隔の変動が再現結果に与える影響は loop_dt_sensitivity() で評価する。

■ 実測の時間軸（受信時刻を使わない）
  PC（WSL2）の受信時刻は使わない。WSL2 の時計は Windows の時計より数%遅れて進むことがあり（2026-10-22 に
  約5%を確認）、受信時刻で時間軸を作ると、同定する時定数やFFとの位置合わせがその分ずれるためである。
  代わりに「値が変わったメッセージ＝制御の1周」として周回番号を復元し（配信 10 ms < 周回 11 ms なので、
  どの周回の値も1回以上配信される）、周回番号 k を時刻 t = k·LOOP_DT とみなす。目標値が変わった周回を k=0 とする。
  POT の浮動小数点値 y は、z1・z2 の配信値から ESO の更新式を逆に解いて復元する（配信POTは整数に丸められているため）。

■ 座標
  z = P − P0（P0 は目標値切替直前に静止していた位置の実測平均）。システム同定・ADRC内部シミュレーション・EL はこの系。
  y = P − Pf（目標値を原点にして 0 へ収束する系）。目標軌道（ζ=1 の3次遅れ系）の同定はこの系で、z = y + (Pf − P0)。
  システムモデルへの入力は u = (u_ADRC + FF) − u_hold（u_hold は切替直前の静止中の PWM 平均）。

■ 内部シミュレーションの状態（1周＝1ステップ, 12次元）
  s = [x0, x1, x2,  ζ1, ζ2, ζ3,  y_{k−1}, y_{k−2}, y_{k−3}, y_{k−4},  D_{k−1},  u_ADRC,{k−1}]
    x : システムモデル（可制御正準形、x0 = z）   ζ : ESO の状態（ζ1 = z1 − P0）
  1周の更新は firmware_step() にファームと同じ順序で書き、その写像から s' = A s + B u_FF + c を数値的に取り出す。
"""
import csv
import json
import math
import os
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np
import scipy.optimize
from scipy.signal import cont2discrete


# ==============================================================================
# 自由度・ボード・トピック（GUI_android_tk.py / GUI_android_tk_FFsetting.py と同じ規則）
# ==============================================================================
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

# ==============================================================================
# ファームウェアの定数（変更しない。.ino と同じ値）
# ==============================================================================
ESO_DT = 0.001              # ESO の式の中の dt（周回間隔ではない）
DIFF_H = 0.001              # derivatives の後退差分の h（周回間隔ではない）
OMEGA_LPF = 0.52            # derivatives の RCローパスの係数 coef_lpf_omega（filter = 前回·coef + 今回·(1−coef)）
POT_MA_N = 18               # POT の移動平均の点数（LPF_KOSUU。システムモデルの一部として同定される）
LOOP_DT_ASSUMED = 0.011     # 制御の周回間隔 [s]（仮定値。実機から正確には測れない）
PUBLISH_DT = 0.010          # ROS2 の配信周期 [s]（周回番号の復元で欠落を数えるときの目安にだけ使う）
PWM_LIMIT = 255.0           # 弁へ出る入力の上限（128 ± u/2 を [0,255] でクリップ → 実効 ±255）
EVAL_FACTOR = 1.5           # 測定・評価区間 = max(EVAL_FACTOR × T, EVAL_MIN_TIME)
EVAL_MIN_TIME = 5.0         # 測定・評価区間の最短 [s]（1.5T が 5 秒未満なら 5 秒評価する）

# ==============================================================================
# 既定の設定（GUIの初期値）
# ==============================================================================
DEFAULT_WEIGHTS = {'Q': [1.0, 1e-2, 1e-4], 'Sf': [0.0, 0.0, 0.0], 'R': 0.05}
DEFAULT_ID_BOUNDS = {
    'sys_T1': (2e-3, 20.0), 'sys_zeta': (-0.4, 5.0), 'sys_wn': (0.5, 50.0), 'sys_b0': (-1e5, 1e5),
    'tgt_T1': (1e-3, 10.0), 'tgt_wn': (0.1, 100.0),
}
DEFAULT_ID_WEIGHTS = {'w_pwm': 1.0, 'w_z3': 1.0}
CL_GROWTH_MAX = 10.0        # 評価区間で許す内部閉ループの増幅率 ρ^N（超えたらそのDOFのFFは更新しない）
HOLD_ITER = 40              # 目標値切替直前の静止区間として平均を取る周回数（約0.44 s）
STEP_WARN = 10.0            # |ΔP| がこれ未満なら同定が不安定になりうると警告する [count]


# ==============================================================================
# 区間の長さ（round() を使わない）
# ==============================================================================
def eval_time(T):
    """測定・評価区間の長さ t_f = max(1.5T, 5 s) [s]"""
    return max(EVAL_FACTOR * float(T), EVAL_MIN_TIME)


def eval_steps(T, L):
    """評価区間の最終ステップ N = max{k : k·L ≤ t_f}、t_f = max(1.5T, 5 s)。測定・評価は k = 0..N の N+1 点"""
    return int(np.floor(eval_time(T) / float(L) + 1e-9))


def ff_steps(T, L):
    """FFが掛かる周回数 N_ff = #{k : k·L < T}（k = 0..N_ff−1。ファームは t ≥ T で 0）"""
    return int(np.ceil(float(T) / float(L) - 1e-9))


# ==============================================================================
# ADRCパラメータ
# ==============================================================================
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


# ==============================================================================
# ファームウェアの1周（複製の唯一の定義）
# ==============================================================================
def firmware_step(zeta, yh, Dp, up, y, ref, prm):
    """ADRC() と ESO() の1周を実機と同じ順序で計算する。

    引数: zeta = (z1, z2, z3)（前の周回の ESO 更新後）, yh = (y_{k−1}, …, y_{k−4}),
          Dp = 前の周回の derivatives, up = 前の周回の u_ADRC, y = この周回の POT_realized, ref = 目標値
    戻り値: (zeta', D, uA)  zeta' はこの周回の ESO 更新後、D はこの周回の derivatives、uA はこの周回の u_ADRC
    z1 と ref は同じ原点で与えること（z 形式なら z1 − P0 と Pf − P0）。y も同じ原点。
    """
    b1, b2, b3 = prm.betas()
    raw = (25.0 * y - 48.0 * yh[0] + 36.0 * yh[1] - 16.0 * yh[2] + 3.0 * yh[3]) / (12.0 * DIFF_H)
    D = OMEGA_LPF * Dp + (1.0 - OMEGA_LPF) * raw                             # RC_LPF_float(derivatives, 前回, …, 0.52)
    inn = y - zeta[0]                                                       # 推定誤差（更新前の z1 で計算）
    z1 = zeta[0] + ESO_DT * (zeta[1] + b1 * inn)                            # dz は更新前の値から同時に計算する
    z2 = zeta[1] + ESO_DT * (zeta[2] + b2 * inn + prm.ic * up)
    z3 = zeta[2] + ESO_DT * (b3 * inn)
    uA = (-z3 + prm.kp * (ref - z1) - prm.kd * D) / prm.ic                  # 更新後の z1・z3 と、この周回の derivatives
    return (z1, z2, z3), D, uA


# ==============================================================================
# システムモデル（3次遅れ系）と閉ループ
# ==============================================================================
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


NS = 12                                     # 閉ループの状態数
IX = slice(0, 3)                            # システムモデル x
IZ = slice(3, 6)                            # ESO ζ
IYH = slice(6, 10)                          # y_{k−1..k−4}
I_DP, I_UP = 10, 11                         # D_{k−1}, u_ADRC,{k−1}
SEL_X = np.hstack([np.eye(3), np.zeros((3, NS - 3))])     # s → x


class ClosedLoop:
    """システムモデル＋実機ADRCの1周の写像 s' = A s + B u_FF + c（z 形式）

    plant = (T1, zeta, wn, b0), prm = AdrcParams, L = 周回間隔 [s], ref0 = Pf − P0, u_hold = 保持PWM。
    firmware_step() を使って写像を数値的に組むので、式はファームの複製と常に一致する。
    出力の取り出し（ステップ k の入力 u_FF(k) に対して）
        z(k) = s_k[0]                    … 周回 k の POT（z 形式）
        u_ADRC(k) = s_{k+1}[11], z3(k) = s_{k+1}[5], D(k) = s_{k+1}[10], z1(k) = s_{k+1}[3], z2(k) = s_{k+1}[4]
        PWM(k) = u_ADRC(k) + u_FF(k)      … 配信される PWM は1周遅れでこの値になる
    """

    def __init__(self, plant, prm, L, ref0, u_hold):
        self.plant = tuple(float(v) for v in plant)
        self.prm = prm
        self.L = float(L)
        self.ref0 = float(ref0)
        self.u_hold = float(u_hold)
        a2, a1, a0 = system_coeffs(*self.plant[:3])
        self.Ad, self.Bd = discretize(a2, a1, a0, self.plant[3], self.L)
        c = self._step(np.zeros(NS), 0.0)
        self.B = self._step(np.zeros(NS), 1.0) - c
        self.A = np.column_stack([self._step(e, 0.0) - c for e in np.eye(NS)])
        self.c = c

    def _step(self, s, u_ff):
        """1周（ファームと同じ順序）。最後にシステムモデルを u = u_ADRC + FF − u_hold で1周進める"""
        x = s[IX]
        y = x[0]
        zeta, D, uA = firmware_step(s[IZ], s[IYH], s[I_DP], s[I_UP], y, self.ref0, self.prm)
        x_next = self.Ad @ x + self.Bd * (uA + u_ff - self.u_hold)
        return np.concatenate([x_next, zeta, [y, s[6], s[7], s[8]], [D], [uA]])

    def spectral_radius(self):
        try:
            return float(np.max(np.abs(np.linalg.eigvals(self.A))))
        except (np.linalg.LinAlgError, ValueError):
            return float('nan')

    def simulate(self, u_ff, s0, n, saturate=False):
        """k = 0..n−1 を進め、状態列 S (n+1, NS) を返す。u_ff は長さ n 以上（足りない分は 0）

        saturate=True のときは弁の飽和（|u_ADRC + FF| ≤ 255）を入れる（同定・再現評価用。EL は線形のまま）。
        """
        u = np.zeros(n)
        uf = np.asarray(u_ff, dtype=float)[:n]
        u[:len(uf)] = uf
        S = np.zeros((n + 1, NS))
        S[0] = s0
        A, B, c, Bd = self.A, self.B, self.c, self.Bd
        with np.errstate(all='ignore'):
            for k in range(n):
                s = A @ S[k] + B * u[k] + c
                if saturate:
                    v = s[I_UP] + u[k]
                    if abs(v) > PWM_LIMIT:
                        s[IX] += Bd * (np.clip(v, -PWM_LIMIT, PWM_LIMIT) - v)
                S[k + 1] = s
        return S

    def simulate_batch(self, U, n):
        """ゼロ初期状態・定数項なしの応答（線形部分だけ）をまとめて計算する。U: (n, B) → (n+1, NS, B)"""
        Bn = U.shape[1]
        S = np.zeros((n + 1, NS, Bn))
        for k in range(n):
            S[k + 1] = self.A @ S[k] + np.outer(self.B, U[k])
        return S

    @staticmethod
    def outputs(S, u_ff):
        """状態列から（周回 k = 0..n−1 の）z, u_ADRC, PWM, z1, z2, z3, D を取り出す"""
        n = S.shape[0] - 1
        u = np.zeros(n)
        uf = np.asarray(u_ff, dtype=float)[:n]
        u[:len(uf)] = uf
        uA = S[1:, I_UP]
        return {'z': S[:-1, 0], 'uA': uA, 'pwm': uA + u, 'z1': S[1:, 3], 'z2': S[1:, 4],
                'z3': S[1:, 5], 'D': S[1:, I_DP], 'u_ff': u}


def rest_state(z3_hold, u_hold, z2_hold=0.0, zeta1_hold=0.0):
    """目標値切替直前に静止していたとみなした初期状態（次の試行の予測・EL に使う）"""
    s = np.zeros(NS)
    s[3], s[4], s[5] = zeta1_hold, z2_hold, z3_hold
    s[I_UP] = u_hold
    return s


# ==============================================================================
# 目標軌道（ζ=1 の3次遅れ系、入力0）
# ==============================================================================
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


def target_xz(T1, wn, P0, Pf, L, n):
    """目標軌道を z 系（原点 P0）の状態列 (n+1, 3) で返す。位置成分だけ Pf − P0 だけずらす"""
    X = target_states(T1, wn, P0 - Pf, L, n)
    X[:, 0] += Pf - P0
    return X


# ==============================================================================
# 累積誤差（相談 C の C2）
# ==============================================================================
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


# ==============================================================================
# 5次関数FF（極値で表す。相談 B の B4）
#   f(t) = a t⁵ + b t⁴ + c t³ + d t² + e t,  e = −(aT⁴ + bT³ + cT² + dT)（f(0)=f(T)=0）
#   自由度は4つで、極値 (t1, y1, t2, y2) と1対1に対応する（f(T)=0, f'(t1)=f'(t2)=0, f(t1)=y1, f(t2)=y2）。
#   (0,T) 内の極値がちょうど2個になるのは高さの比 r = −y2/y1 が (t1,t2) ごとの区間に入るときだけ。
# ==============================================================================
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


def dquintic_dp(T, p):
    """∂[a..e]/∂(t1, t2, y1, y2) を (5, 4) で返す（連鎖律の ∂u/∂p に使う）"""
    t1, t2, y1, y2 = [float(v) for v in p]
    M = quintic_matrix(T, t1, t2)
    co = np.linalg.solve(M, np.array([0.0, 0.0, 0.0, y1, y2]))
    fpp1 = _rfpp(t1) @ co
    fpp2 = _rfpp(t2) @ co
    rhs = np.zeros((5, 4))
    rhs[1, 0] = -fpp1                   # ∂M/∂t1 · c = [0, f''(t1), 0, f'(t1)=0, 0]
    rhs[2, 1] = -fpp2                   # ∂M/∂t2 · c = [0, 0, f''(t2), 0, f'(t2)=0]
    rhs[3, 2] = 1.0                     # ∂b/∂y1
    rhs[4, 3] = 1.0                     # ∂b/∂y2
    return co, np.linalg.solve(M, rhs)


def ratio_margin(T, p):
    """極値ちょうど2個の条件の余裕 h（> 0 なら内部）。h = min(log(r/lo), log(hi/r))、r = −y2/y1、[lo, hi] は r を含む区間。

    r がどの区間にも入らなければ、最も近い区間までの距離を負で返す。区間が無ければ −inf。
    """
    t1, t2, y1, y2 = [float(v) for v in p]
    if not (0.0 < t1 < t2 < T) or y1 == 0.0:
        return -np.inf, None
    r = -y2 / y1
    if r <= 0:
        return -np.inf, None
    ivs = ratio_intervals(T, t1, t2, r_hint=r)
    if not ivs:
        return -np.inf, None
    best = None
    for lo, hi in ivs:
        h = min(np.log(r / lo), np.log(hi / r))
        if best is None or h > best[0]:
            best = (h, (lo, hi))
    return best


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
# 評価関数の重み
# ==============================================================================
@dataclass
class Weights:
    """J = e_Nᵀ Sf e_N + Σ_{k=0}^{N−1} e_kᵀ Q e_k·L + Σ_{k=0}^{N_ff−1} R u_k²·L

    e = [位置, 速度, 加速度] の誤差（評価区間 t_f = max(1.5T, 5 s) 全体）、u = FF（0 ≤ t < T だけ。T 以降は誤差の項だけ）
    """
    Q: list = field(default_factory=lambda: list(DEFAULT_WEIGHTS['Q']))
    Sf: list = field(default_factory=lambda: list(DEFAULT_WEIGHTS['Sf']))
    R: float = DEFAULT_WEIGHTS['R']

    def as_dict(self):
        return {'Q': [float(v) for v in self.Q], 'Sf': [float(v) for v in self.Sf], 'R': float(self.R)}

    @staticmethod
    def from_dict(d):
        return Weights(Q=[float(v) for v in d['Q']], Sf=[float(v) for v in d['Sf']], R=float(d['R']))


# ==============================================================================
# 離散時間オイラー・ラグランジュ法（極値座標 p = (t1, t2, y1, y2) を変数とする。相談 B の B4）
# ==============================================================================
@dataclass
class ELResult:
    status: str                         # 'converged'(_boundary) / 'stalled'(_boundary) / 'max_iter' / 'zero' / 'aborted' / 'failed'
                                        # _boundary は「極値ちょうど2個」の境界で止まった（制約が効いている）
    coeffs: np.ndarray                  # 送る [a, b, c, d, e]
    extrema: tuple                      # (t1, y1, t2, y2)
    u_ff: np.ndarray                    # FF入力（k = 0..N−1）
    J: float                            # 評価関数 J_EL
    J_parts: dict                       # {'state': …, 'input': …, 'terminal': …}
    iters: int
    proj_grad2: float                   # ∫‖Π ∂H/∂u‖² dt（5次関数の接空間へ射影した勾配。収束判定に使う）
    free_grad2: float                   # ∫‖∂H/∂u‖² dt（射影しない値。参考）
    history: list                       # 反復ごとの (J, proj_grad2, α)
    starts: list                        # 初期値の候補 [(J, p), ...]
    pred: dict                          # この FF での予測（ClosedLoop.outputs の内容）


class ELSolver:
    """ADRCを含む閉ループ s' = A s + B u + c に対して、5次関数FF（極値座標）を勾配法で最適化する。

    ● 評価関数とハミルトニアン（e_k = S s_k − x_tgt,k）
        J   = e_Nᵀ Sf e_N + Σ_{k=0}^{N−1} e_kᵀ Q e_k L + Σ_{k=0}^{N_ff−1} R u_k² L   （N: t_f = max(1.5T, 5 s), N_ff: t < T）
        H_k = (e_kᵀ Q e_k + R u_k²) L + λ_{k+1}ᵀ (A s_k + B u_k + c)   （k ≥ N_ff は u_k = 0 なので誤差の項だけ）
    ● 終端条件・共役方程式・勾配
        λ_N = 2 Sᵀ Sf e_N
        λ_k = ∂H_k/∂s_k = 2L Sᵀ Q e_k + Aᵀ λ_{k+1}          （k = N−1..1。s_0 は固定なので λ_0 は使わない）
        ∂J/∂u_k = ∂H_k/∂u_k = 2RL u_k + Bᵀ λ_{k+1}         （k < N_ff。FF は t ≥ T で 0 に固定）
        連続時間の ∂H/∂u(t_k) に当たる量は g_k = (∂H_k/∂u_k)/L
    ● 5次関数への制約（u_k = f(k L; p)）
        ∂J/∂p_i = Σ_k (∂H_k/∂u_k) ψ_i(t_k),  ψ_i = ∂f/∂p_i
        u の空間で勾配 g を5次関数の接空間へ L² 射影した Πg = Σ c_i ψ_i は G c = ∂J/∂p（G = Σ ψψᵀ L）で求まる。
        更新は u_new = u − α Πg に当たる p_new = p − α c（自然勾配）。収束判定は ∫‖Πg‖² dt = cᵀ G c ≤ ε。
        α は u について J が厳密に2次式であることを使い、方向 −Πg に沿った最小点 α* を応答1回で求め、
        p が非線形なぶんはバックトラックで補う。「極値ちょうど2個」を破る点へは進まない（刻みを縮める）。
    ● 初期化（非凸なので多点開始）
        (t1, t2) の格子ごとに J は (y1, y2) の2次式になるので、高さの比 r の実行可能区間（ratio_intervals）
        の中で最小点を閉形式で求め、J の小さい順に n_starts 個から勾配法を始める。
    """

    def __init__(self, model, s0, x_tgt, T, L, weights, eps=1e-6, max_iter=300, grid_n=22, n_starts=3,
                 stop_event=None, log=None):
        self.m = model
        self.s0 = np.asarray(s0, dtype=float)
        self.x_tgt = np.asarray(x_tgt, dtype=float)
        self.N = len(self.x_tgt) - 1
        self.T = float(T)
        self.L = float(L)
        self.N_ff = min(ff_steps(T, L), self.N)
        self.t_ff = np.arange(self.N_ff) * self.L
        self.Q = np.diag(np.asarray(weights.Q, dtype=float))
        self.Sf = np.diag(np.asarray(weights.Sf, dtype=float))
        self.R = float(weights.R)
        self.eps = float(eps)
        self.max_iter = int(max_iter)
        self.grid_n = int(grid_n)
        self.n_starts = int(n_starts)
        self.stop_event = stop_event
        self.log = log or (lambda msg: None)
        self.S0 = self.m.simulate(np.zeros(self.N), self.s0, self.N)      # FF=0 の応答（公称軌道）

    # ---------------- 順計算・評価関数・随伴 ----------------
    def u_of(self, p):
        co = quintic_from_extrema(self.T, *p)
        u = np.zeros(self.N)
        u[:self.N_ff] = ff_eval(co, self.T, self.t_ff)
        return co, u

    def forward(self, u):
        return self.m.simulate(u, self.s0, self.N)

    def errors(self, S):
        return S[:, IX] - self.x_tgt

    def cost(self, e, u):
        st = float(np.einsum('ki,ij,kj->', e[:-1], self.Q, e[:-1])) * self.L
        inp = self.R * float(u @ u) * self.L
        term = float(e[-1] @ self.Sf @ e[-1])
        return st + inp + term, {'state': st, 'input': inp, 'terminal': term}

    def adjoint(self, e):
        lam = np.zeros((self.N + 1, NS))
        lam[self.N, :3] = 2.0 * (self.Sf @ e[self.N])                           # λ_N = 2 Sᵀ Sf e_N
        AT = self.m.A.T
        q = np.zeros(NS)
        for k in range(self.N - 1, 0, -1):
            q[:3] = 2.0 * self.L * (self.Q @ e[k])
            lam[k] = q + AT @ lam[k + 1]                                        # λ_k = 2L SᵀQ e_k + Aᵀ λ_{k+1}
        return lam

    def dJdu(self, u, lam):
        """∂H_k/∂u_k（k = 0..N−1）。k ≥ N_ff は変数ではないが、参考の勾配ノルムのために全区間を返す"""
        return 2.0 * self.R * self.L * u + lam[1:self.N + 1] @ self.m.B

    def evaluate(self, p):
        co, u = self.u_of(p)
        S = self.forward(u)
        e = self.errors(S)
        J, parts = self.cost(e, u)
        return co, u, S, e, J, parts

    def dir_curvature(self, du):
        """方向 du に沿った J の2階微分 d²J/dα²（J は u の2次式。応答は線形部分だけ）"""
        Sd = self.m.simulate_batch(du[:, None], self.N)[:, :, 0]
        ed = Sd[:, IX]
        st = float(np.einsum('ki,ij,kj->', ed[:-1], self.Q, ed[:-1])) * self.L
        return 2.0 * (st + self.R * float(du @ du) * self.L + float(ed[-1] @ self.Sf @ ed[-1]))

    # ---------------- 初期化（格子＋閉形式） ----------------
    def initial_candidates(self, warm=None):
        T, N, Nf = self.T, self.N, self.N_ff
        e0 = self.errors(self.S0)
        cands = []
        if warm is not None and feasible(T, warm):
            co, u, S, e, J, _ = self.evaluate(warm)
            cands.append((J, np.asarray(warm, dtype=float)))
        t1g = np.linspace(0.03, 0.60, self.grid_n) * T
        t2g = np.linspace(0.12, 0.96, self.grid_n) * T
        pts, g1s, g2s = [], [], []
        for t1 in t1g:
            for t2 in t2g:
                if t2 - t1 < 0.08 * T:
                    continue
                try:
                    g1 = quintic_from_extrema(T, t1, t2, 1.0, 0.0)
                    g2 = quintic_from_extrema(T, t1, t2, 0.0, 1.0)
                except np.linalg.LinAlgError:
                    continue
                pts.append((t1, t2))
                g1s.append(ff_eval(g1, T, self.t_ff))
                g2s.append(ff_eval(g2, T, self.t_ff))
        if not pts:
            return cands
        U = np.zeros((N, 2 * len(pts)))
        U[:Nf, 0::2] = np.array(g1s).T
        U[:Nf, 1::2] = np.array(g2s).T
        R = self.m.simulate_batch(U, N)[:, IX, :]                                # (N+1, 3, 2P) 線形応答
        Lq = self.L
        for i, (t1, t2) in enumerate(pts):
            E1, E2 = R[:, :, 2 * i], R[:, :, 2 * i + 1]
            u1, u2 = U[:, 2 * i], U[:, 2 * i + 1]

            def ip(Ea, Eb, ua, ub):
                return (float(np.einsum('ki,ij,kj->', Ea[:-1], self.Q, Eb[:-1])) * Lq
                        + float(Ea[-1] @ self.Sf @ Eb[-1]) + self.R * float(ua @ ub) * Lq)
            P = np.array([[ip(E1, E1, u1, u1), ip(E1, E2, u1, u2)], [ip(E1, E2, u1, u2), ip(E2, E2, u2, u2)]])
            qv = np.array([ip(e0, E1, np.zeros(N), u1), ip(e0, E2, np.zeros(N), u2)])
            J0 = ip(e0, e0, np.zeros(N), np.zeros(N))
            best = None
            for lo, hi in ratio_intervals(T, t1, t2):
                lo, hi = lo * 1.002, hi * 0.998
                if lo >= hi:
                    continue
                for sgn in (1.0, -1.0):                                         # y1 > 0（山→谷）と y1 < 0（谷→山）
                    for r in (lo, hi, None):
                        if r is None:                                           # 錐の内部の停留点
                            try:
                                y = -np.linalg.solve(P, qv)
                            except np.linalg.LinAlgError:
                                continue
                            if not (y[0] * sgn > 0 and y[1] * sgn < 0 and lo <= -y[1] / y[0] <= hi):
                                continue
                        else:                                                   # 錐の境界の半直線 y = a·(sgn, −sgn·r)
                            dvec = np.array([sgn, -sgn * r])
                            den = dvec @ P @ dvec
                            a = max(0.0, -(qv @ dvec) / den) if den > 0 else 0.0
                            y = a * dvec
                        if np.max(np.abs(y)) < 1e-6:
                            continue
                        J = J0 + 2 * qv @ y + y @ P @ y
                        if best is None or J < best[0]:
                            best = (float(J), np.array([t1, t2, y[0], y[1]]))
            if best is not None:
                cands.append(best)
        cands.sort(key=lambda c: c[0])
        out = []
        for J, p in cands:                                                      # 近い開始点は1つにまとめる
            if all(np.hypot((p[0] - q[0]) / T, (p[1] - q[1]) / T) > 0.06 for _, q in out):
                if feasible(T, p):
                    out.append((J, p))
            if len(out) >= self.n_starts:
                break
        return out

    # ---------------- 勾配法 ----------------
    def descend(self, p0):
        """自然勾配法（極値ちょうど2個の条件が効いているときは、その境界に沿う方向へ射影する）

        制約 h(p) ≥ 0（ratio_margin）が効いている（h ≈ 0）とき、KKT 条件に合わせて勾配の係数 c から
        制約の法線 n の成分を G の計量で除く: c_t = c − G⁻¹n (nᵀc)/(nᵀG⁻¹n)（外向きに進むときだけ）。
        収束判定はこの c_t による ∫‖Π ∂H/∂u‖² dt = c_tᵀ G c_t ≤ ε。1歩進んだ点が実行可能でなければ retract() で戻す。
        """
        T, L = self.T, self.L
        p = np.asarray(p0, dtype=float)
        co, u, S, e, J, parts = self.evaluate(p)
        hist = []
        status = 'max_iter'
        pg2 = fg2 = float('nan')
        active = False
        it = 0
        for it in range(1, self.max_iter + 1):
            if self.stop_event is not None and self.stop_event.is_set():
                status = 'aborted'
                break
            lam = self.adjoint(e)
            g = self.dJdu(u, lam)
            fg2 = float(np.sum(g ** 2) / L)                                     # ∫‖∂H/∂u‖² dt（参考。制約なしの停留条件）
            _, dc = dquintic_dp(T, p)
            Psi = np.zeros((self.N, 4))
            Psi[:self.N_ff] = np.stack([ff_eval(dc[:, i], T, self.t_ff) for i in range(4)], axis=1)
            b = Psi.T @ g                                                       # ∂J/∂p
            G = L * (Psi.T @ Psi)
            G += 1e-12 * np.trace(G) * np.eye(4)
            try:
                c = np.linalg.solve(G, b)
            except np.linalg.LinAlgError:
                status = 'failed'
                break
            h, _iv = ratio_margin(T, p)
            active = bool(np.isfinite(h) and h < 1e-4)
            if active:
                n = self._margin_normal(p)
                Gn = np.linalg.solve(G, n)
                if n @ c > 0 and (n @ Gn) > 0:                                  # −c が外向き（h を減らす）なら接線へ射影
                    c = c - Gn * (n @ c) / (n @ Gn)
            pg2 = float(c @ G @ c)                                              # ∫‖Π ∂H/∂u‖² dt
            hist.append((J, pg2, float('nan')))
            if pg2 <= self.eps:
                status = 'converged'
                break
            du = -(Psi @ c)                                                     # u の空間での方向 −Πg
            curv = self.dir_curvature(du)
            slope = float(g @ du)                                               # dJ/dα (< 0)
            if slope >= 0:
                status = 'stalled'
                break
            alpha = -slope / curv if curv > 0 else 1.0
            accepted = False
            for _ in range(40):
                p_new = retract(T, p - alpha * c)
                if p_new is not None:
                    co_n, u_n, S_n, e_n, J_n, parts_n = self.evaluate(p_new)
                    if J_n < J + 1e-4 * alpha * slope or (J_n < J and active):
                        accepted = True
                        break
                alpha *= 0.5
            if not accepted:
                status = 'stalled'
                break
            rel = (J - J_n) / max(abs(J), 1e-300)
            p, co, u, S, e, J, parts = p_new, co_n, u_n, S_n, e_n, J_n, parts_n
            hist[-1] = (hist[-1][0], hist[-1][1], float(alpha))
            if rel < 1e-10:
                status = 'stalled'                                          # 改善が尽きた（ε には届いていない）
                break
        if status in ('converged', 'stalled') and active:
            status += '_boundary'
        return {'p': p, 'coeffs': co, 'u': u, 'S': S, 'J': J, 'parts': parts, 'iters': it, 'status': status,
                'pg2': pg2, 'fg2': fg2, 'hist': hist, 'active': active}

    def _margin_normal(self, p):
        """ratio_margin の p についての勾配（中心差分）"""
        T = self.T
        n = np.zeros(4)
        for i in range(4):
            h = 1e-5 * (T if i < 2 else max(abs(p[i]), 1e-3))
            pp, pm = p.copy(), p.copy()
            pp[i] += h
            pm[i] -= h
            hp, hm = ratio_margin(T, pp)[0], ratio_margin(T, pm)[0]
            if np.isfinite(hp) and np.isfinite(hm):
                n[i] = (hp - hm) / (2 * h)
        return n

    def solve(self, warm=None):
        e0 = self.errors(self.S0)
        J_zero, parts_zero = self.cost(e0, np.zeros(self.N))
        starts = self.initial_candidates(warm)
        if not starts:
            self.log("  EL: 極値ちょうど2個の5次関数の初期値が見つかりません（FF=0 のまま）")
            return self._zero_result('failed', J_zero, parts_zero, starts)
        best = None
        for J0, p0 in starts:
            if self.stop_event is not None and self.stop_event.is_set():
                break
            r = self.descend(p0)
            self.log(f"  EL: 開始 t1={p0[0]:.3f} y1={p0[2]:+.2f} t2={p0[1]:.3f} y2={p0[3]:+.2f} (J={J0:.4g}) → "
                     f"J={r['J']:.4g}, {r['iters']}反復, ∫‖Π∂H/∂u‖²dt={r['pg2']:.3g} [{r['status']}]")
            if best is None or r['J'] < best['J']:
                best = r
        if best is None:
            return self._zero_result('aborted', J_zero, parts_zero, starts)
        if self.stop_event is not None and self.stop_event.is_set():
            best['status'] = 'aborted'
        if best['J'] >= J_zero or max(abs(best['p'][2]), abs(best['p'][3])) < 1e-3:
            self.log(f"  EL: FF=0（J={J_zero:.4g}）より良い5次関数がありません。FF は 0 にします")
            return self._zero_result('zero', J_zero, parts_zero, starts)
        t1, t2, y1, y2 = [float(v) for v in best['p']]
        return ELResult(status=best['status'], coeffs=np.asarray(best['coeffs'], dtype=float),
                        extrema=(t1, y1, t2, y2), u_ff=best['u'], J=float(best['J']), J_parts=best['parts'],
                        iters=int(best['iters']), proj_grad2=float(best['pg2']), free_grad2=float(best['fg2']),
                        history=best['hist'], starts=[(float(J), list(map(float, p))) for J, p in starts],
                        pred=ClosedLoop.outputs(best['S'], best['u']))

    def _zero_result(self, status, J, parts, starts):
        u = np.zeros(self.N)
        return ELResult(status=status, coeffs=np.zeros(5), extrema=(self.T * 0.33, 0.0, self.T * 0.66, 0.0),
                        u_ff=u, J=float(J), J_parts=parts, iters=0, proj_grad2=float('nan'),
                        free_grad2=float('nan'), history=[],
                        starts=[(float(J0), list(map(float, p))) for J0, p in starts],
                        pred=ClosedLoop.outputs(self.S0, u))


# ==============================================================================
# 実測の復元（周回番号で並べる。受信時刻は欠落を数えるときだけ使う）
# ==============================================================================
@dataclass
class BoardIterations:
    """1ボードぶんの、周回番号 i で並べた実測（列はボード内の番号 0..count−1）

    pot・des・der は周回 i の値、pwm・z1・z2・z3 は周回 i の計算結果、y は周回 i の POT_realized（浮動小数点）。
    欠落・対応が取れなかった周回は NaN。method は 'exact'（式で対応を確かめた）か 'time'（受信時刻で組んだ）。
    """
    count: int
    pot: np.ndarray
    des: np.ndarray
    der: np.ndarray
    pwm: np.ndarray
    z1: np.ndarray
    z2: np.ndarray
    z3: np.ndarray
    y: np.ndarray
    t_first: np.ndarray             # その周回の PWM を最初に受信した時刻（PCの時計。目安としてだけ使う）
    n_drops: int                    # 配信の欠落を推定した箇所の数
    n_messages: int
    method: str = 'time'
    n_unmatched: int = 0            # 式で対応が取れなかった周回の数（method='exact' のとき）


def _events(t, V, pub=None):
    """連続して同じ値のメッセージを1つにまとめ、周回ごとの値の列にする。

    配信 10 ms < 周回 11 ms なので、欠落がなければどの周回の値も1回以上届き、値が変わるたびに1周進む。
    受信間隔が配信周期の1.5倍を超えたら欠落とみなし、その間に過ぎた周回の数だけ NaN の行を入れる。
    戻り値 (各周回の値 (n, c), その値を最初に受信した時刻 (n,), 最後に受信した時刻 (n,), 欠落箇所の数)
    """
    n = len(t)
    if n == 0:
        return np.zeros((0, V.shape[1] if V.ndim == 2 else 0)), np.zeros(0), np.zeros(0), 0
    pub = float(np.median(np.diff(t))) if (pub is None and n > 2) else (pub or PUBLISH_DT)
    per_iter = pub * LOOP_DT_ASSUMED / PUBLISH_DT
    vals, first, last, drops = [V[0]], [t[0]], [t[0]], 0
    for m in range(1, n):
        same = np.array_equal(V[m], V[m - 1], equal_nan=True)
        gap = t[m] - t[m - 1]
        if gap > 1.5 * pub:
            drops += 1
            k = max(1, int(round(gap / per_iter))) if not same else int(round(gap / per_iter))
            for _ in range(max(0, k - 1)):
                vals.append(np.full(V.shape[1], np.nan))
                first.append(np.nan)
                last.append(np.nan)
            if k >= 1 and not same:
                vals.append(V[m])
                first.append(t[m])
                last.append(t[m])
            else:
                last[-1] = t[m]
        elif not same:
            vals.append(V[m])
            first.append(t[m])
            last.append(t[m])
        else:
            last[-1] = t[m]
    return np.array(vals, dtype=float), np.array(first, dtype=float), np.array(last, dtype=float), drops


def _adrc_residual(pwm, z3, z1, der, prm, ref):
    """PWM − (−z3 + kp(r − z1) − kd·derivatives)/ic（FF=0 の自由度なら、周回の対応が正しいときほぼ 0）"""
    return pwm - (-z3 + prm.kp * (ref - z1) - prm.kd * der) / prm.ic


def reconstruct_board(raw, count, check=None, tol=5e-3, prms=None):
    """受信メッセージから、周回番号で並べた実測を作る。

    raw  : {kind: (t, V)}。t は受信時刻（昇順）、V は値（pot は 2*count 列 = POT, 目標値、他は count 列）。
    check: {ボード内の番号: (AdrcParams, 目標値)} … 試行中に目標値もFFも変わらない自由度（式で周回の対応を確かめる）。
    prms : {ボード内の番号: AdrcParams} … POT_realized を ESO の式から復元する自由度（最適化対象も含める）。
    z1・z3・derivatives が届いていて check があれば 'exact'（式で確かめる）、なければ 'time'（受信時刻で組む）で復元する。
    """
    if 'pwm' not in raw or len(raw['pwm'][0]) == 0:
        return None
    floats_ok = all(k in raw and len(raw[k][0]) > 0 for k in ('z3', 'z1', 'der'))
    check = {l: v for l, v in (check or {}).items() if v[0].valid()}
    if floats_ok and check:
        bi = _reconstruct_exact(raw, count, check, tol, prms)
        if bi is not None:
            return bi
    return _reconstruct_by_time(raw, count)


def _partner(tk, t_ref, side, tol=0.006):
    """同じ配信（タイマ1回）で送られた他のトピックのメッセージ。PWM より先に送られるもの（POT）は直前、
    後に送られるもの（z3, z1, derivatives, z2）は直後に届いたものを選ぶ。見つからなければ −1"""
    if side == 'before':
        j = np.searchsorted(tk, t_ref + 1e-6, 'right') - 1
    else:
        j = np.searchsorted(tk, t_ref - 1e-6, 'left')
    j = np.clip(j, 0, len(tk) - 1)
    return np.where(np.abs(tk[j] - t_ref) <= tol, j, -1)


def _pot_by_time(raw, count, t_last):
    """POT・目標値（整数）は値が変わらないことが多く周回に区切れないので、受信時刻で割り当てる。

    タイマ1回の配信の中で POT は PWM より先にコピーされるため、その間に制御の周回が割り込むと「POT は1周前、
    PWM は新しい周回」の組になる（周回が変わった最初の配信で起きる）。そこで各周回の最後の配信の POT を使う。
    t_last[i] は「周回 i の POT と組になる PWM（周回 i−1 の値）」を最後に受信した時刻。
    """
    K = len(t_last)
    pot = np.full((K, count), np.nan)
    des = np.full((K, count), np.nan)
    if 'pot' not in raw or len(raw['pot'][0]) == 0:
        return pot, des
    tp, Vp = raw['pot']
    ok = np.isfinite(t_last)
    j = np.full(K, -1)
    j[ok] = _partner(tp, t_last[ok], 'before')
    for i in np.flatnonzero(j >= 0):
        pot[i] = Vp[j[i]][:count]
        des[i] = Vp[j[i]][count:2 * count]
    return pot, des


def _reconstruct_by_time(raw, count):
    """z1・derivatives が無い記録用。PWM の値の変化で周回に区切り、他のトピックは受信時刻で組む"""
    ta, Va = raw['pwm']
    E, t_first, t_last_ev, drops = _events(ta, Va)
    K = len(E) + 1
    nanm = lambda: np.full((K, count), np.nan)
    out = dict(pwm=nanm(), z1=nanm(), z2=nanm(), z3=nanm(), der=nanm())
    t_iter = np.full(K, np.nan)
    t_last = np.full(K, np.nan)
    t_iter[1:] = t_first                                                # PWM が周回 i の値になった配信は周回 i+1 のもの
    t_last[1:] = t_last_ev
    out['pwm'][:-1] = E[:, :count]
    # PWM より後に送られるトピックは、割り込みがあると新しい周回の値になる（周回が変わる直前の配信で起きる）。
    # そこで各周回の最初の配信（PWM が新しい値になった配信）に続いて届いた値を使う。
    for kind in ('z1', 'z2', 'z3', 'der'):
        if kind not in raw or len(raw[kind][0]) == 0:
            continue
        tk, Vk = raw[kind]
        okm = np.isfinite(t_iter)
        j = np.full(K, -1)
        j[okm] = _partner(tk, t_iter[okm], 'after')
        for i in np.flatnonzero(j >= 0):
            if kind == 'der':
                out['der'][i] = Vk[j[i]][:count]                        # derivatives はその配信の周回の値
            else:
                out[kind][i - 1] = Vk[j[i]][:count]                     # z1〜z3 は1周前の計算結果
    pot, des = _pot_by_time(raw, count, t_last)
    return BoardIterations(count=count, pot=pot, des=des, der=out['der'], pwm=out['pwm'], z1=out['z1'],
                           z2=out['z2'], z3=out['z3'], y=pot.copy(), t_first=t_iter, n_drops=drops,
                           n_messages=len(ta), method='time')


def _reconstruct_exact(raw, count, check, tol, prms=None):
    """各トピックの値の列（_events）を、ADRCの式が成り立つずれで重ねる。欠落でずれが変わったら合わせ直す"""
    seq = {k: _events(*raw[k]) for k in ('pwm', 'z3', 'z1', 'der', 'z2') if k in raw and len(raw[k][0])}
    n = len(seq['pwm'][0])
    if n < 20:
        return None
    check = _consistent_locals(seq, check, tol)
    if not check:
        return None
    locs = sorted(check)

    def get(kind, j, o):
        E = seq[kind][0]
        i = j + o
        return E[i] if 0 <= i < len(E) else None

    def ok(j, o):
        p, z3, z1, d = get('pwm', j, 0), get('z3', j, o[0]), get('z1', j, o[1]), get('der', j, o[2])
        if any(v is None for v in (p, z3, z1, d)):
            return None
        for l in locs:
            prm, ref = check[l]
            r = _adrc_residual(p[l], z3[l], z1[l], d[l], prm, ref)
            if not np.isfinite(r):
                return None
            if abs(r) > tol:
                return False
        return True

    rng = range(-3, 4)
    offs = sorted(((a, b, c) for a in rng for b in rng for c in rng), key=lambda o: sum(map(abs, o)))

    def search(j0, around=None, span=6):
        if around is None:
            cands = offs
        else:
            near = {(around[0] + a, around[1] + b, around[2] + c) for a in range(-2, 3) for b in range(-2, 3)
                    for c in range(-2, 3)}
            cands = sorted(near, key=lambda o: sum(abs(o[i] - around[i]) for i in range(3)))
        for o in cands:
            res = [ok(j, o) for j in range(j0, min(n, j0 + span))]
            good = [r for r in res if r is not None]
            if len(good) >= 3 and all(good):
                return o
        return None

    o, j0 = None, 0
    while o is None and j0 < n - 6:
        o = search(j0)
        j0 += 3
    if o is None:
        return None
    K = n + 1
    nanm = lambda: np.full((K, count), np.nan)
    out = dict(pwm=nanm(), z1=nanm(), z3=nanm(), der=nanm(), z2=nanm())
    t_iter = np.full(K, np.nan)
    t_last = np.full(K, np.nan)
    unmatched = 0
    for j in range(n):
        r = ok(j, o)
        if r is False:
            o2 = search(j, around=o)
            if o2 is not None:
                o, r = o2, True
        if r is not True:
            unmatched += 1
            continue
        out['pwm'][j] = get('pwm', j, 0)[:count]
        out['z3'][j] = get('z3', j, o[0])[:count]
        out['z1'][j] = get('z1', j, o[1])[:count]
        out['der'][j] = get('der', j, o[2])[:count]
        t_iter[j + 1] = seq['pwm'][1][j]
        t_last[j + 1] = seq['pwm'][2][j]
    if 'z2' in seq:
        _align_z2(seq['z2'][0], out, check, count)
    # POT_realized を ESO の z3 更新式から復元する: z3_i = z3_{i−1} + dt·β3·(y_i − z1_{i−1})
    y = np.full((K, count), np.nan)
    ys = {l: prm for l, (prm, _ref) in check.items()}
    ys.update({l: p for l, p in (prms or {}).items() if p is not None and p.valid()})
    for l, prm in ys.items():
        y[1:, l] = out['z1'][:-1, l] + (out['z3'][1:, l] - out['z3'][:-1, l]) / (ESO_DT * prm.betas()[2])
    pot, des = _pot_by_time(raw, count, t_last)
    drops = sum(seq[k][3] for k in seq)
    return BoardIterations(count=count, pot=pot, des=des, der=out['der'], pwm=out['pwm'], z1=out['z1'],
                           z2=out['z2'], z3=out['z3'], y=y, t_first=t_iter, n_drops=drops,
                           n_messages=len(raw['pwm'][0]), method='exact', n_unmatched=unmatched)


def _consistent_locals(seq, check, tol, n_try=60):
    """式で周回の対応を確かめるのに使える自由度だけを残す（PWM を 0 に固定している制御対象外の枠などを除く）。

    自由度ごとに、先頭 n_try 周回の中で ADRC の式が成り立つずれ (z3, z1, derivatives) があるかを調べ、
    最も多くの自由度が一致したずれを持つ自由度を返す。
    """
    found = {}
    rng = range(-3, 4)
    for l, (prm, ref) in check.items():
        for o in sorted(((a, b, c) for a in rng for b in rng for c in rng), key=lambda o: sum(map(abs, o))):
            hits = 0
            for j in range(min(n_try, len(seq['pwm'][0]))):
                vals = []
                for kind, off in (('pwm', 0), ('z3', o[0]), ('z1', o[1]), ('der', o[2])):
                    E = seq[kind][0]
                    vals.append(E[j + off][l] if 0 <= j + off < len(E) else np.nan)
                r = _adrc_residual(*vals, prm, ref)
                if np.isfinite(r) and abs(r) <= tol:
                    hits += 1
            if hits >= 10:
                found[l] = o
                break
    if not found:
        return {}
    votes = {}
    for o in found.values():
        votes[o] = votes.get(o, 0) + 1
    best = max(votes, key=votes.get)
    return {l: check[l] for l, o in found.items() if o == best}


def _align_z2(E2, out, check, count):
    """z2 の列のずれを ESO の更新式 z2_i = z2_{i−1} + dt(z3_{i−1} + β2(y_i − z1_{i−1}) + ic·u_{i−1}) で決める"""
    K = out['pwm'].shape[0]
    best, best_err = None, np.inf
    for o in range(-3, 4):
        Z2 = np.full((K, count), np.nan)
        for j in range(K):
            if 0 <= j + o < len(E2):
                Z2[j] = E2[j + o][:count]
        errs = []
        for l, (prm, _ref) in check.items():
            _b1, b2, b3 = prm.betas()
            z1, z3, u = out['z1'][:, l], out['z3'][:, l], out['pwm'][:, l]
            yv = z1[:-1] + (z3[1:] - z3[:-1]) / (ESO_DT * b3)
            e = Z2[:-1, l] + ESO_DT * (z3[:-1] + b2 * (yv - z1[:-1]) + prm.ic * u[:-1]) - Z2[1:, l]
            errs.append(np.nanmedian(np.abs(e)) if np.isfinite(e).any() else np.inf)
        err = max(errs) if errs else np.inf
        if err < best_err:
            best, best_err = Z2, err
    if best is not None:
        out['z2'][:] = best


def find_change_iteration(bi, changes, prms=None, tol=5e-3):
    """目標値が変わった周回 kc。changes = {ボード内の番号: (Pi, Pf)}, prms = {ボード内の番号: AdrcParams}

    method='exact' でパラメータがあれば、ADRC の式が r = Pf で成り立つ最初の周回（比例キックの周回）で決める
    （その周回の FF は f(0) = 0 なので式に FF は入らない）。そうでなければ配信の目標値が新しい値になった最初の周回。
    """
    if bi.method == 'exact' and prms:
        for i in range(len(bi.pwm)):
            hits = []
            for local, (Pi, Pf) in changes.items():
                prm = prms.get(local)
                if prm is None or not prm.valid():
                    continue
                args = (bi.pwm[i, local], bi.z3[i, local], bi.z1[i, local], bi.der[i, local], prm)
                r_new, r_old = _adrc_residual(*args, Pf), _adrc_residual(*args, Pi)
                if np.isfinite(r_new):
                    hits.append(abs(r_new) <= tol and abs(r_old) > tol)
            if hits and all(hits):
                return i
    for i in range(1, len(bi.des)):
        for local, (Pi, Pf) in changes.items():
            prev, cur = bi.des[i - 1, local], bi.des[i, local]
            if np.isfinite(cur) and abs(cur - round(Pf)) < 0.5 and not (np.isfinite(prev) and abs(prev - round(Pf)) < 0.5):
                return i
    return None


@dataclass
class DofTrial:
    """1試行・1自由度の実測（周回 k = 0..N。k=0 が目標値の変わった周回）。z は z = P − P0"""
    dof: int
    Pi: float
    Pf: float
    P0: float
    u_hold: float
    z3_hold: float
    z2_hold: float
    T: float
    L: float
    N: int
    prm: AdrcParams
    coeffs: np.ndarray              # 送ったFFの係数 [a, b, c, d, e]
    y: np.ndarray                   # POT（浮動小数点。z1・z2 から復元できない周回は配信の整数値）
    y_float: np.ndarray             # 復元できた周回は True
    pwm: np.ndarray                 # u_ADRC + FF（周回 k の出力）
    ff: np.ndarray                  # 送ったFF（t = k·L）
    z1: np.ndarray
    z2: np.ndarray
    z3: np.ndarray
    D: np.ndarray
    pre: dict                       # 切替前 HOLD_ITER 周回ぶん（y, pwm, z1, z2, z3, D）
    s0: np.ndarray                  # 実測から作った初期状態（z 形式）
    n_drops: int

    @property
    def dP(self):
        return self.Pf - self.Pi

    @property
    def uA(self):
        return self.pwm - self.ff

    @property
    def z(self):
        return self.y - self.P0

    @property
    def valid(self):
        return np.isfinite(self.y)

    def ff_at(self, L=None):
        """周回 k = 0..N の FF（周回間隔 L で t = k·L。省略時は self.L）"""
        L = self.L if L is None else float(L)
        return ff_eval(self.coeffs, self.T, np.arange(self.N + 1) * L)


def extract_dof(bi, local, dof, kc, Pi, Pf, prm, coeffs, T, L, hold=HOLD_ITER):
    """ボードの周回データから1自由度ぶんの DofTrial を作る（kc は目標値が変わった周回）"""
    N = eval_steps(T, L)
    if kc < 1:
        raise ValueError(f"DOF {dof}: 目標値切替前の周回がありません")
    col = lambda a: a[:, local]
    yr = col(bi.pot)
    yf = col(bi.y) if bi.method == 'exact' else np.full(len(yr), np.nan)   # ESO の式から復元した浮動小数点の POT
    good = np.isfinite(yf)
    y_all = np.where(good, yf, yr)

    def seg(a, s, e):
        out = np.full(e - s, np.nan)
        s2, e2 = max(s, 0), min(e, len(a))
        if e2 > s2:
            out[s2 - s:e2 - s] = a[s2:e2]
        return out

    ff = np.zeros(N + 1)
    ff[:] = ff_eval(coeffs, T, np.arange(N + 1) * L)
    pre = {name: seg(arr, kc - hold, kc) for name, arr in
           (('y', y_all), ('pwm', col(bi.pwm)), ('z1', col(bi.z1)), ('z2', col(bi.z2)),
            ('z3', col(bi.z3)), ('D', col(bi.der)))}
    # 原点 P0: ESO から復元した浮動小数点の POT があれば切替周回の値（実測の履歴とモデルの初期状態をつなげるため）、
    # なければ（整数の POT しかない）静止区間の平均
    with np.errstate(all='ignore'):
        if kc < len(y_all) and good[kc] and np.isfinite(y_all[kc]):
            P0 = float(y_all[kc])
        else:
            P0 = float(np.nanmean(pre['y'])) if np.isfinite(pre['y']).any() else float(y_all[kc])
    u_hold = float(np.nanmean(pre['pwm']))
    z3_hold = float(np.nanmean(pre['z3']))
    z2_hold = float(np.nanmean(pre['z2'])) if np.isfinite(pre['z2']).any() else 0.0
    # 実測から作る初期状態: 周回 −1 の ESO 更新後・u_ADRC、周回 −1..−4 の y、周回 −1 の derivatives
    s0 = np.zeros(NS)
    zl = [seg(col(a), kc - 1, kc)[0] for a in (bi.z1, bi.z2, bi.z3)]
    s0[3] = (zl[0] - P0) if np.isfinite(zl[0]) else 0.0
    s0[4] = zl[1] if np.isfinite(zl[1]) else z2_hold
    s0[5] = zl[2] if np.isfinite(zl[2]) else z3_hold
    yh = seg(y_all, kc - 4, kc)[::-1] - P0
    s0[IYH] = np.where(np.isfinite(yh), yh, 0.0)
    Dm = seg(col(bi.der), kc - 1, kc)[0]
    s0[I_DP] = Dm if np.isfinite(Dm) else 0.0
    um = seg(col(bi.pwm), kc - 1, kc)[0]
    s0[I_UP] = um if np.isfinite(um) else u_hold
    s0[0] = 0.0                                                     # システムモデルは静止（P0 で u_hold と釣り合う）
    return DofTrial(dof=dof, Pi=float(Pi), Pf=float(Pf), P0=P0, u_hold=u_hold, z3_hold=z3_hold, z2_hold=z2_hold,
                    T=float(T), L=float(L), N=N, prm=prm, coeffs=np.asarray(coeffs, dtype=float),
                    y=seg(y_all, kc, kc + N + 1), y_float=seg(good.astype(float), kc, kc + N + 1) > 0.5,
                    pwm=seg(col(bi.pwm), kc, kc + N + 1), ff=ff,
                    z1=seg(col(bi.z1), kc, kc + N + 1), z2=seg(col(bi.z2), kc, kc + N + 1),
                    z3=seg(col(bi.z3), kc, kc + N + 1), D=seg(col(bi.der), kc, kc + N + 1),
                    pre=pre, s0=s0, n_drops=bi.n_drops)


# ==============================================================================
# ファーム複製の検証（実測 POT を入れて、u_ADRC と z3 が実測と一致するか）
# ==============================================================================
def replay_check(tr):
    """実測 y を firmware_step() に入れ、複製の u_ADRC・z3 を実測と比べる（システムモデルを使わない検証）

    周回間隔 L にも依存しない（ESO・derivatives・ADRC は周回ごとの式なので）。y を z1・z2 から復元できていれば、
    一致は浮動小数点の誤差程度になるはずで、ずれていればゲイン・式・周回の対応のどれかが実機と違う。
    """
    prm = tr.prm
    H = len(tr.pre['y'])
    cat = lambda name, post: np.concatenate([tr.pre[name], post])
    yy = cat('y', tr.y)
    uAc = np.concatenate([tr.pre['pwm'], tr.uA])
    z1c, z2c, z3c, Dc = cat('z1', tr.z1), cat('z2', tr.z2), cat('z3', tr.z3), cat('D', tr.D)
    idx = np.arange(len(yy))
    good = np.isfinite(yy)
    nan_out = {'uA_rms': float('nan'), 'z3_rms': float('nan'), 'z3_rms_pwm': float('nan'), 'n': 0,
               'float_ratio': float(np.mean(tr.y_float)), 'uA_replay': np.full(tr.N + 1, np.nan),
               'z3_replay': np.full(tr.N + 1, np.nan)}
    if good.sum() < 10:
        return nan_out
    yy = np.interp(idx, idx[good], yy[good]) - tr.P0
    start = next((i for i in range(1, H + 1) if np.all(np.isfinite([z3c[i - 1], uAc[i - 1]]))), None)
    if start is None:
        return nan_out
    # z1・z2 を配信しないファーム・記録では、静止中とみなして z1 ≈ POT, z2 ≈ 0 から始める
    zeta = ((z1c[start - 1] - tr.P0) if np.isfinite(z1c[start - 1]) else yy[start - 1],
            z2c[start - 1] if np.isfinite(z2c[start - 1]) else 0.0, z3c[start - 1])
    yh = [yy[max(start - 1 - j, 0)] for j in range(4)]               # y_{start−1..start−4}（足りない分は最初の値）
    Dp = Dc[start - 1] if np.isfinite(Dc[start - 1]) else 0.0
    up = uAc[start - 1]
    uA = np.full(len(yy), np.nan)
    z3 = np.full(len(yy), np.nan)
    for i in range(start, len(yy)):
        ref = (tr.Pi if i < H else tr.Pf) - tr.P0
        zeta, D, u = firmware_step(zeta, yh, Dp, up, yy[i], ref, prm)
        uA[i], z3[i] = u, zeta[2]
        yh = [yy[i], yh[0], yh[1], yh[2]]
        Dp, up = D, u
    m = (idx >= H) & np.isfinite(uAc) & np.isfinite(uA)
    m3 = (idx >= H) & np.isfinite(z3c) & np.isfinite(z3)
    eu = uA[m] - uAc[m]
    e3 = z3[m3] - z3c[m3]
    out = dict(nan_out)
    out.update({'uA_rms': float(np.sqrt(np.mean(eu ** 2))) if m.any() else float('nan'),
                'z3_rms': float(np.sqrt(np.mean(e3 ** 2))) if m3.any() else float('nan'),
                'z3_rms_pwm': float(np.sqrt(np.mean(e3 ** 2)) / prm.ic) if m3.any() else float('nan'),
                'n': int(m.sum()), 'uA_replay': uA[H:], 'z3_replay': z3[H:]})
    return out


# ==============================================================================
# システムモデルの同定
# ==============================================================================
def _plant_from_q(q):
    return (float(np.exp(q[0])), float(q[1]), float(np.exp(q[2])), float(q[3]))


def _bounds_q(bounds):
    lo = [np.log(bounds['sys_T1'][0]), bounds['sys_zeta'][0], np.log(bounds['sys_wn'][0]), bounds['sys_b0'][0]]
    hi = [np.log(bounds['sys_T1'][1]), bounds['sys_zeta'][1], np.log(bounds['sys_wn'][1]), bounds['sys_b0'][1]]
    return np.array(lo, dtype=float), np.array(hi, dtype=float)


def _q_from_plant(p, lo, hi):
    T1, zeta, wn, b0 = [float(v) for v in p]
    if not (T1 > 0 and wn > 0 and np.isfinite(zeta) and np.isfinite(b0)):
        return None
    return np.clip([np.log(T1), zeta, np.log(wn), b0], lo + 1e-9, hi - 1e-9)


def simulate_plant(plant, u_in, L, n):
    """システムモデル単体の応答 z(0..n)（x(0)=0、入力 u_in(0..n−1)）"""
    a2, a1, a0 = system_coeffs(*plant[:3])
    Ad, Bd = discretize(a2, a1, a0, plant[3], L)
    x = np.zeros(3)
    z = np.zeros(n + 1)
    with np.errstate(all='ignore'):
        for k in range(n):
            x = Ad @ x + Bd * u_in[k]
            z[k + 1] = x[0]
    return z


def _spectral_seed(z, L):
    n = np.arange(len(z))
    osc = z - np.polyval(np.polyfit(n, z, 1), n)
    spec = np.abs(np.fft.rfft(osc * np.hanning(len(osc))))
    freq = np.fft.rfftfreq(len(osc), L)
    sel = freq > 0.1
    if not np.any(sel) or spec[sel].max() <= 0:
        return None
    return 2 * np.pi * float(freq[sel][np.argmax(spec[sel])])


def fit_open_loop(tr, bounds, prev=None):
    """実測 PWM（弁の飽和でクリップ）を入力、実測 POT を出力として、システムモデル単体を同定する（閉ループ同定の初期値用）"""
    N, L = tr.N, tr.L
    pwm = tr.pwm.copy()
    idx = np.arange(N + 1)
    g = np.isfinite(pwm)
    pwm = np.interp(idx, idx[g], pwm[g]) if g.any() else np.full(N + 1, tr.u_hold)
    u_in = np.clip(pwm, -PWM_LIMIT, PWM_LIMIT) - tr.u_hold
    z = tr.z
    m = np.isfinite(z)
    lo, hi = _bounds_q(bounds)

    def res(q):
        r = (simulate_plant(_plant_from_q(q), u_in, L, N) - np.nan_to_num(z))[m]
        return np.clip(np.nan_to_num(r, nan=1e5, posinf=1e5, neginf=-1e5), -1e5, 1e5)

    starts = []
    if prev is not None:
        q = _q_from_plant(prev, lo, hi)
        if q is not None:
            starts.append(q)
    b0_init = float(starts[0][3]) if starts else 100.0
    wn_seed = _spectral_seed(np.nan_to_num(z), L)
    for ratio in ((0.8, 1.0, 1.25) if wn_seed else ()):
        starts.append(np.clip([np.log(0.3), 0.1, np.log(wn_seed * ratio), b0_init], lo + 1e-9, hi - 1e-9))
    starts.append(np.clip([np.log(0.3), 0.7, np.log(5.0), b0_init], lo + 1e-9, hi - 1e-9))
    best = None
    for q0 in starts:
        try:
            r = scipy.optimize.least_squares(res, q0, bounds=(lo, hi), method='trf', x_scale='jac', max_nfev=400)
        except Exception:
            continue
        if best is None or r.cost < best.cost:
            best = r
    if best is None:
        return (0.3, 0.7, 5.0, b0_init)
    return _plant_from_q(best.x)


def closed_loop_problem(tr, L=None, w_pwm=1.0, w_z3=1.0, saturate=True):
    """3チャネル（POT・u_ADRC・z3）の残差関数と、チャネル別の一致度の関数を返す

    同定モデルを実測と同じ条件（実測の初期状態・送ったFF・目標変位・保持PWM）で閉ループのまま回し、
        r = [ (z_sim − z)/σ_pot,  √w_pwm (u_sim − u)/σ_pwm,  √w_z3 (z3_sim − z3)/σ_z3 ]
        σ_pot = max(|ΔP|,1), σ_pwm = max(rms(u − u_hold),1), σ_z3 = max(rms(z3 − z3_hold),1)
    """
    L = tr.L if L is None else float(L)
    N = tr.N
    z, uA, z3 = tr.z, tr.uA, tr.z3
    mp, mu, m3 = np.isfinite(z), np.isfinite(uA), np.isfinite(z3)
    mu = mu if (w_pwm > 0 and mu.mean() >= 0.5) else np.zeros(N + 1, dtype=bool)
    m3 = m3 if (w_z3 > 0 and m3.mean() >= 0.5) else np.zeros(N + 1, dtype=bool)
    rms = lambda a: float(np.sqrt(np.mean(a ** 2))) if a.size else float('nan')
    sp = max(abs(tr.dP), 1.0)
    su = max(rms(uA[mu] - tr.u_hold), 1.0) if mu.any() else 1.0
    s3 = max(rms(z3[m3] - tr.z3_hold), 1.0) if m3.any() else 1.0
    wu, w3 = np.sqrt(max(w_pwm, 0.0)), np.sqrt(max(w_z3, 0.0))
    ff = tr.ff_at(L)[:N]                                            # FF は実時間で掛かるので t = k·L で作り直す

    def simulate(plant):
        cl = ClosedLoop(plant, tr.prm, L, tr.Pf - tr.P0, tr.u_hold)
        S = cl.simulate(ff, tr.s0, N + 1, saturate=saturate)
        o = ClosedLoop.outputs(S, np.append(ff, 0.0))
        return o, cl

    def residuals(plant):
        with np.errstate(all='ignore'):
            o, _ = simulate(plant)
            parts = [(o['z'] - z)[mp] / sp]
            if mu.any():
                parts.append(wu * (o['uA'] - uA)[mu] / su)
            if m3.any():
                parts.append(w3 * (o['z3'] - z3)[m3] / s3)
            r = np.concatenate(parts)
        return np.clip(np.nan_to_num(r, nan=1e5, posinf=1e5, neginf=-1e5), -1e5, 1e5)

    def metrics(plant):
        with np.errstate(all='ignore'):
            o, cl = simulate(plant)
            r = residuals(plant)
            out = {'cost': float(r @ r) / max(r.size, 1),
                   'pot': rms((o['z'] - z)[mp]), 'pwm': rms((o['uA'] - uA)[mu]) if mu.any() else float('nan'),
                   'z3': rms((o['z3'] - z3)[m3]) if m3.any() else float('nan'),
                   'rho': cl.spectral_radius()}
        out['z3_pwm'] = out['z3'] / tr.prm.ic
        return out, o

    channels = ('POT',) + (('u_ADRC',) if mu.any() else ()) + (('z3',) if m3.any() else ())
    return residuals, metrics, channels


def fit_closed_loop(tr, bounds, starts, w_pwm=1.0, w_z3=1.0, L=None, max_nfev=300):
    """開始点のリスト（開ループ同定の結果・前回のモデル）から3チャネルの閉ループ同定を行い、最もよいものを返す"""
    residuals, metrics, channels = closed_loop_problem(tr, L, w_pwm, w_z3)
    lo, hi = _bounds_q(bounds)
    best, best_m = None, None
    for p in starts:
        if p is None:
            continue
        q0 = _q_from_plant(p, lo, hi)
        if q0 is None:
            continue
        try:
            r = scipy.optimize.least_squares(lambda q: residuals(_plant_from_q(q)), q0, bounds=(lo, hi),
                                             method='trf', x_scale='jac', max_nfev=max_nfev)
        except Exception:
            continue
        cand = _plant_from_q(r.x)
        m, _ = metrics(cand)
        if best_m is None or m['cost'] < best_m['cost']:
            best, best_m = cand, m
    return best, best_m, channels


def identify_system(tr, bounds, prev=None, w_pwm=1.0, w_z3=1.0, L=None):
    """Step 3: 開ループ同定 → 3チャネル閉ループ同定。戻り値 dict（plant, metrics, open 側の結果, 使ったチャネル, 境界への張り付き）"""
    L = tr.L if L is None else float(L)
    p_open = fit_open_loop(tr if L == tr.L else with_loop_dt(tr, L), bounds, prev)
    _, metrics, channels = closed_loop_problem(tr, L, w_pwm, w_z3)
    m_open, _ = metrics(p_open)
    p_cl, m_cl, _ = fit_closed_loop(tr, bounds, [p_open, prev], w_pwm, w_z3, L)
    use_cl = p_cl is not None and m_cl['cost'] < m_open['cost']
    plant, m = (p_cl, m_cl) if use_cl else (p_open, m_open)
    _, sim = metrics(plant)
    pinned = []
    for name, v in zip(('T1', 'zeta', 'wn', 'b0'), plant):
        lo_b, hi_b = bounds['sys_' + name]
        if v <= lo_b * (1.001 if lo_b > 0 else 0.999) + 1e-12 or v >= hi_b * 0.999:
            pinned.append(name)
    return {'plant': plant, 'metrics': m, 'metrics_open': m_open, 'plant_open': p_open, 'closed_loop': bool(use_cl),
            'channels': channels, 'pinned': pinned, 'sim': sim, 'L': L}


def with_loop_dt(tr, L):
    """周回間隔だけを変えた DofTrial の写し（FF の時刻も t = k·L で作り直す）"""
    out = DofTrial(**{k: getattr(tr, k) for k in tr.__dataclass_fields__})
    out.L = float(L)
    out.ff = tr.ff_at(L)
    return out


def loop_dt_sensitivity(tr, plant, bounds, L_list, refit=False, jitter_sd=0.0, n_jitter=0, w_pwm=1.0, w_z3=1.0,
                        seed=0):
    """周回間隔 L の変動が再現結果（POT・u_ADRC・z3 の rms）に与える影響を評価する。

    ・L_list の各値で、同じシステムモデル（連続時間のパラメータ）のまま回した場合
    ・refit=True なら、各 L でシステムモデルを同定し直した場合（L が違っていても同定で吸収できるか）
    ・jitter_sd > 0 なら、周回ごとに L を N(L, jitter_sd²) でばらつかせた場合（n_jitter 回の平均）
    FF は実機と同じく実時間で掛かるので、周回 k の FF は f(Σ L_j) で与える。
    """
    rows = []
    for Lx in L_list:
        _, metrics, _ = closed_loop_problem(tr, Lx, w_pwm, w_z3)
        m_same, _ = metrics(plant)
        row = {'L': float(Lx), 'same_plant': m_same}
        if refit:
            res = identify_system(tr, bounds, prev=plant, w_pwm=w_pwm, w_z3=w_z3, L=Lx)
            row['refit'] = res['metrics']
            row['refit_plant'] = res['plant']
        rows.append(row)
    jit = None
    if jitter_sd > 0 and n_jitter > 0:
        rng = np.random.default_rng(seed)
        acc = []
        for _ in range(n_jitter):
            Ls = np.clip(rng.normal(tr.L, jitter_sd, tr.N + 1), 0.5 * tr.L, 1.5 * tr.L)
            acc.append(_simulate_jitter(tr, plant, Ls))
        jit = {key: float(np.mean([a[key] for a in acc])) for key in acc[0]}
        jit['sd'] = float(jitter_sd)
    return rows, jit


def _simulate_jitter(tr, plant, Ls):
    """周回ごとに間隔が違う場合の閉ループ（システムモデルは各周回の間隔で離散化）"""
    a2, a1, a0 = system_coeffs(*plant[:3])
    cache = {}
    t = np.concatenate([[0.0], np.cumsum(Ls[:-1])])
    ff = ff_eval(tr.coeffs, tr.T, t)
    s = tr.s0.copy()
    zs, us, z3s = [], [], []
    for k in range(tr.N + 1):
        key = round(float(Ls[k]), 6)
        if key not in cache:
            cache[key] = discretize(a2, a1, a0, plant[3], key)
        Ad, Bd = cache[key]
        x = s[IX]
        zs.append(x[0])
        zeta, D, uA = firmware_step(s[IZ], s[IYH], s[I_DP], s[I_UP], x[0], tr.Pf - tr.P0, tr.prm)
        v = uA + ff[k]
        x_next = Ad @ x + Bd * (np.clip(v, -PWM_LIMIT, PWM_LIMIT) - tr.u_hold)
        s = np.concatenate([x_next, zeta, [x[0], s[6], s[7], s[8]], [D], [uA]])
        us.append(uA)
        z3s.append(zeta[2])
    rms = lambda a: float(np.sqrt(np.nanmean(a ** 2)))
    return {'pot': rms(np.array(zs) - tr.z), 'pwm': rms(np.array(us) - tr.uA),
            'z3': rms(np.array(z3s) - tr.z3), 'z3_pwm': rms(np.array(z3s) - tr.z3) / tr.prm.ic}


# ==============================================================================
# 記録CSVの読み込み（GUI_android_tk.py / GUI_android_tk_FFsetting.py / 本プログラムの時系列CSV）
# ==============================================================================
CSV_KIND = {'pot': 'pot', 'pwm': 'pwm', 'z3': 'z3', 'z1': 'z1', 'z2': 'z2', 'der': 'derivatives'}


def load_record_csv(path):
    """記録CSV（ボードごとに Time{k}, POT{d}…, POTdesired{d}…, Time{k}_PWM, PWM{d}…, Time{k}_z3, z3_{d}… の形式）を
    {board: {kind: (t, V)}} に読む。z1・z2・derivatives の列があればそれも読む。"""
    with open(path, newline='', encoding='utf-8-sig') as f:
        rows = list(csv.reader(f))
    head, body = rows[0], rows[1:]
    pos = {name: i for i, name in enumerate(head)}
    out = {}
    for k, (board, offset, count) in enumerate(BOARD_LAYOUT, start=1):
        dofs = range(offset, offset + count)
        groups = {
            'pot': (f"Time{k}", [f"POT{d}" for d in dofs] + [f"POTdesired{d}" for d in dofs]),
            'pwm': (f"Time{k}_PWM", [f"PWM{d}" for d in dofs]),
            'z3': (f"Time{k}_z3", [f"z3_{d}" for d in dofs]),
            'z1': (f"Time{k}_z1", [f"z1_{d}" for d in dofs]),
            'z2': (f"Time{k}_z2", [f"z2_{d}" for d in dofs]),
            'der': (f"Time{k}_derivatives", [f"derivatives_{d}" for d in dofs]),
        }
        raw = {}
        for kind, (tname, cols) in groups.items():
            if tname not in pos or not all(c in pos for c in cols):
                continue
            ti, ci = pos[tname], [pos[c] for c in cols]
            t, V = [], []
            for r in body:
                if ti < len(r) and r[ti] != '':
                    t.append(float(r[ti]))
                    V.append([float(r[c]) if c < len(r) and r[c] != '' else np.nan for c in ci])
            raw[kind] = (np.array(t), np.array(V, dtype=float).reshape(len(t), len(cols)))
        out[board] = raw
    return out


def load_param_sheet(path):
    """<記録>_ADRCparam.csv から {DOF: AdrcParams}（記録開始時の値）を読む"""
    out = {}
    with open(path, newline='', encoding='utf-8-sig') as f:
        for r in csv.DictReader(f):
            try:
                vals = [float(r[k + '_start']) for k in ('kp', 'kd', 'input_coef', 'lambda_0')]
            except (KeyError, ValueError):
                continue
            out[int(r['DOF'])] = AdrcParams(*vals)
    return out


# ==============================================================================
# 実機との通信（ROS2）。rclpy はこのクラスを使うときだけ読み込む（ビューアは ROS なしで動かすため）
# ==============================================================================
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


# ==============================================================================
# 1回の試行（安全な順序）
# ==============================================================================
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


def return_to_initial(link, initial, T, log=None):
    """FF=0 を送ってから初期姿勢を送る（FF は目標値が変わるたびに t=0 から掛かるため、戻りで掛からないようにする）"""
    link.send_ff(np.zeros((N_FF, 5)), T)
    time.sleep(0.1)
    link.send_targets(initial)
    link.last_return = time.monotonic()
    if log:
        log("  FF=0 を送ってから初期姿勢へ戻しました")


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


def run_trial(link, initial, target, ff_rows, T, L, wait_s, stop_event, log, opt_dofs, margin_iter=15):
    """1試行: FF=0 → 初期姿勢 → 待機 → FF送信 → 目標値 → max(1.5T, 5 s) 測定 → FF=0 → 初期姿勢。

    待機は「初期姿勢を送ってから max(wait_s, T + 0.5) 秒」。初期姿勢への移動でもFFのタイマは t=0 から動くので、
    次のFFを送ったときに戻りのタイマで掛かり始めないよう T より長く待つ。停止要求が来たら FF=0 → 初期姿勢で抜ける。
    """
    N = eval_steps(T, L)
    need = N + 1 + margin_iter
    boards = sorted({DOF_BOARD[d][1] for d in opt_dofs})
    started = time.strftime('%Y-%m-%d %H:%M:%S')
    try:
        cur = link.board_targets()
        if link.last_return is None or any(v is None or abs(v - float(i)) >= 0.5 for v, i in zip(cur, initial)):
            return_to_initial(link, initial, T, log)
        wait = max(float(wait_s), float(T) + 0.5)
        remain = wait - (time.monotonic() - link.last_return)
        if remain > 0:
            log(f"  初期姿勢で待機 {remain:.1f} 秒")
            _sleep_checked(remain, stop_event)
        wait_rest(link, initial, opt_dofs, stop_event, log)
        params = link.adrc_params()
        missing = [d for d in opt_dofs if params.get(d) is None or not params[d].valid()]
        if missing:
            raise RuntimeError(f"ADRCパラメータを実機から受信できない自由度があります: DOF {missing}")
        link.send_ff(ff_rows, T)
        time.sleep(0.15)                                                # FF が届いてから目標値を送る（FF は目標値の切替で t=0）
        link.start_capture()
        time.sleep(0.05)
        link.send_targets(target)
        t0 = time.monotonic()
        log(f"  目標値を送信しました。max({EVAL_FACTOR:g}T, {EVAL_MIN_TIME:g} s) = {eval_time(T):.2f} 秒（{N + 1} 周回）測定します")
        timeout = need * L * 1.3 + 3.0
        while True:
            _sleep_checked(0.05, stop_event)
            cnt = link.capture_counts(t0)
            if all(cnt.get(b, 0) >= need for b in boards):
                break
            if time.monotonic() - t0 > timeout:
                log(f"  警告: 周回数がそろわないまま測定を終えます（{cnt}）")
                break
        raw = link.stop_capture()
        return_to_initial(link, initial, T, log)
        return TrialRecord(raw=raw, initial=list(initial), target=list(target), ff=np.asarray(ff_rows, dtype=float),
                           T=float(T), L=float(L), params=params, params_after=link.adrc_params(), t_target=t0,
                           started_at=started)
    except StopRequested:
        raw = link.stop_capture() if link.capture is not None else {}
        return_to_initial(link, initial, T, log)
        return TrialRecord(raw=raw, initial=list(initial), target=list(target), ff=np.asarray(ff_rows, dtype=float),
                           T=float(T), L=float(L), params=link.adrc_params(), params_after=link.adrc_params(),
                           t_target=time.monotonic(), started_at=started, aborted=True, note='停止ボタン')
    except Exception:
        if link.capture is not None:
            link.stop_capture()
        return_to_initial(link, initial, T, log)
        raise


# ==============================================================================
# 試行の解析（Step 2・3・4.8: 周回の復元 → ファーム複製の検証 → 目標軌道 → J → システム同定）
# ==============================================================================
@dataclass
class DofAnalysis:
    dof: int
    trial: DofTrial
    replay: dict
    tgt: tuple                      # (T1, wn)
    tgt_rms: float
    y_tgt: np.ndarray               # 目標軌道（絶対POT, k = 0..N）
    J_i: float
    ident: dict                     # identify_system() の戻り値
    sens: list = None               # loop_dt_sensitivity() の結果
    jitter: dict = None
    board_method: str = ''
    board_note: str = ''


def opt_dofs_from(initial, target):
    """|ΔP| ≠ 0 の自由度（FF の枠がない DOF 21・25 は除く）"""
    return [d for d in range(N_DOF) if d not in NO_FF_DOFS and float(target[d]) != float(initial[d])]


def analyze_trial(rec, opt_dofs, bounds, id_weights, prev_tgt, prev_sys, log, sens_L=None, jitter_sd=0.0, n_jitter=0):
    """試行1回ぶんを解析する。prev_tgt / prev_sys は {DOF: 前回の値}（同定の初期値）"""
    out = {}
    L, T = rec.L, rec.T
    for k, (board, offset, count) in enumerate(BOARD_LAYOUT, start=1):
        dofs = [d for d in opt_dofs if offset <= d < offset + count]
        if not dofs:
            continue
        raw = rec.raw.get(board, {})
        check = {}
        for j in range(count):
            d = offset + j
            p = rec.params.get(d)
            if d in dofs or p is None or not p.valid():
                continue
            if float(rec.initial[d]) == float(rec.target[d]):
                check[j] = (p, float(rec.target[d]))
        prms_b = {j: rec.params.get(offset + j) for j in range(count)}
        bi = reconstruct_board(raw, count, check, prms=prms_b)
        if bi is None:
            log(f"  {board}: 受信データがありません")
            continue
        note = ''
        if bi.method != 'exact':
            note = ("z1・derivatives が無い、または式で周回を合わせられる自由度がないため、周回の対応は受信時刻で決めました"
                    if not check else
                    "ADRC の式で周回を合わせられませんでした（.ino と違う版のファームが動いている可能性）。受信時刻で組みました")
            log(f"  {board}: {note}")
        elif bi.n_unmatched > 0.05 * len(bi.pwm):
            note = f"式で対応が取れなかった周回が {bi.n_unmatched}/{len(bi.pwm)} あります"
            log(f"  {board}: {note}")
        changes = {d - offset: (float(rec.initial[d]), float(rec.target[d])) for d in dofs}
        prms_local = {d - offset: rec.params[d] for d in dofs}
        kc = find_change_iteration(bi, changes, prms_local)
        if kc is None:
            log(f"  {board}: 目標値が変わった周回が見つかりません（DOF {dofs}）")
            continue
        for d in dofs:
            n_ff = FF_INDEX[d]
            tr = extract_dof(bi, d - offset, d, kc, rec.initial[d], rec.target[d], rec.params[d], rec.ff[n_ff], T, L)
            if len(tr.y) < tr.N + 1 or np.isfinite(tr.y).mean() < 0.8:
                log(f"  DOF {d}: 測定区間の実測が足りません（有効 {np.isfinite(tr.y).mean() * 100:.0f}%）")
            rp = replay_check(tr)
            yv = tr.y - tr.Pf
            (T1, wn), trms = fit_target(yv, np.isfinite(yv), tr.P0 - tr.Pf, L, bounds, prev_tgt.get(d))
            y_tgt = target_states(T1, wn, tr.P0 - tr.Pf, L, tr.N)[:, 0] + tr.Pf
            J_i = normalized_error(tr.y, y_tgt, tr.dP, np.isfinite(tr.y))
            ident = identify_system(tr, bounds, prev_sys.get(d), id_weights.get('w_pwm', 1.0), id_weights.get('w_z3', 1.0))
            da = DofAnalysis(dof=d, trial=tr, replay=rp, tgt=(T1, wn), tgt_rms=trms, y_tgt=y_tgt, J_i=J_i, ident=ident,
                             board_method=bi.method, board_note=note)
            if sens_L:
                da.sens, da.jitter = loop_dt_sensitivity(tr, ident['plant'], bounds, sens_L, refit=False,
                                                         jitter_sd=jitter_sd, n_jitter=n_jitter,
                                                         w_pwm=id_weights.get('w_pwm', 1.0),
                                                         w_z3=id_weights.get('w_z3', 1.0))
            m = ident['metrics']
            log(f"  DOF {d}: J_i = {J_i:.4g}（RMS {rms_percent(J_i):.2f}% of ΔP）  目標軌道 T1={T1:.4f} ωn={wn:.3f}")
            log(f"     ファーム複製（実測POTを入力）: u_ADRC 誤差 {rp['uA_rms']:.3g} PWM, z3 誤差 {rp['z3_rms_pwm']:.3g} PWM相当"
                f"（POT を ESO から復元 {rp['float_ratio'] * 100:.0f}%）")
            log(f"     システムモデル [{'・'.join(ident['channels'])}{'' if ident['closed_loop'] else '・開ループ結果'}] "
                f"T1={ident['plant'][0]:.4f} ζ={ident['plant'][1]:.4f} ωn={ident['plant'][2]:.3f} b0={ident['plant'][3]:.4g}  "
                f"再現: POT {m['pot']:.2f} count, u_ADRC {m['pwm']:.2f} PWM, z3 {m['z3_pwm']:.2f} PWM相当, ρ={m['rho']:.5f}")
            if ident['pinned']:
                log(f"     警告: 同定結果が上下限に張り付いています（{', '.join(ident['pinned'])}）")
            if abs(tr.dP) < STEP_WARN:
                log(f"     警告: |ΔP| = {abs(tr.dP):.1f} < {STEP_WARN:g} なので同定が不安定になることがあります")
            if da.sens:
                txt = ", ".join(f"L={r['L'] * 1000:.1f}ms: POT {r['same_plant']['pot']:.2f}/uA {r['same_plant']['pwm']:.2f}"
                                for r in da.sens)
                log(f"     周回間隔を変えた再現（同じモデル）: {txt}")
            out[d] = da
    return out


def mean_J(analysis):
    vals = [a.J_i for a in analysis.values() if np.isfinite(a.J_i)]
    return float(np.mean(vals)) if vals else float('nan')


def optimize_ff(da, weights, T, L, eps, max_iter, stop_event, log, warm=None, grid_n=22, n_starts=3):
    """Step 4: 同定したシステムモデル＋実機ADRC の閉ループで、目標軌道に近づける5次関数FF を EL で求める"""
    tr = da.trial
    model = ClosedLoop(da.ident['plant'], tr.prm, L, tr.Pf - tr.P0, tr.u_hold)
    N = eval_steps(T, L)
    rho = model.spectral_radius()
    growth = rho ** N if np.isfinite(rho) else float('inf')
    s0 = rest_state(tr.z3_hold, tr.u_hold, tr.z2_hold)
    x_tgt = target_xz(da.tgt[0], da.tgt[1], tr.P0, tr.Pf, L, N)
    solver = ELSolver(model, s0, x_tgt, T, L, weights, eps=eps, max_iter=max_iter, grid_n=grid_n, n_starts=n_starts,
                      stop_event=stop_event, log=log)
    res = solver.solve(warm)
    pwm_peak = float(np.max(np.abs(res.pred['pwm']))) if len(res.pred['pwm']) else float('nan')
    return res, {'rho': rho, 'growth': growth, 'gate_ok': bool(growth <= CL_GROWTH_MAX), 'pwm_peak': pwm_peak,
                 'J_zero': float(solver.cost(solver.errors(solver.S0), np.zeros(solver.N))[0])}


def predicted_J(da, coeffs, T, L, bounds):
    """そのFFを与えたときのモデルの応答に ζ=1 の目標軌道を当てはめ直して J_i を予測する（重みに依存しない指標）"""
    tr = da.trial
    model = ClosedLoop(da.ident['plant'], tr.prm, L, tr.Pf - tr.P0, tr.u_hold)
    N = eval_steps(T, L)
    s0 = rest_state(tr.z3_hold, tr.u_hold, tr.z2_hold)
    u = ff_samples(coeffs, T, L, N)
    S = model.simulate(u, s0, N)
    y = S[:, 0] + tr.P0
    (T1, wn), _ = fit_target(y - tr.Pf, np.ones(N + 1, dtype=bool), tr.P0 - tr.Pf, L, bounds, da.tgt)
    y_tgt = target_states(T1, wn, tr.P0 - tr.Pf, L, N)[:, 0] + tr.Pf
    return normalized_error(y, y_tgt, tr.dP, np.ones(N + 1, dtype=bool)), y, y_tgt


# ==============================================================================
# 保存（CSV・ビューア用 npz）
# ==============================================================================
CSV_ENCODING = 'utf-8-sig'      # CSV は BOM 付き UTF-8（Excel で開いても日本語が文字化けしないように）
POINTER_FILE = '/tmp/el2_optimization_current.json'     # ビューアに実行中の保存先を知らせるファイル
LOOP_DIR = 'loops'                                      # 保存先の中の、ループごとの npz の置き場
FINISHED_FILE = 'finished.json'                         # 最適化が終わったことをビューアへ知らせるファイル


def make_run_folder(base, prefix='optimize'):
    """<base>/<prefix>_YYYYMMDD_HHMM を作る（同じ名前があれば _2, _3 … を付ける）"""
    name = time.strftime(f'{prefix}_%Y%m%d_%H%M')
    path = os.path.join(base, name)
    n = 2
    while os.path.exists(path):
        path = os.path.join(base, f"{name}_{n}")
        n += 1
    os.makedirs(os.path.join(path, LOOP_DIR))
    return path


def write_pointer(folder, session, state='running'):
    data = {'folder': os.path.abspath(folder), 'session': session, 'state': state, 'updated': time.time()}
    tmp = POINTER_FILE + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(data, f)
    os.replace(tmp, POINTER_FILE)


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


def write_trial_timeseries(path, rec, kc_by_board):
    """試行の受信データそのもの（GUI_android_tk.py の記録CSVと同じ列 ＋ PWM の組に FF{d}・ADRC{d}、
    その後ろに z1・z2・derivatives の組、最後に送った目標値 Time_cmd, cmd{d}）。

    時刻は目標値を送った瞬間からの秒数（PCの時計。WSL2 では数%遅れることがある）。
    FF{d} は各 PWM メッセージが属する周回 i から k = i − kc を求め、ファームと同じく f(k·L) で計算した値。
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
        labels = _message_iterations(V) if len(t) else np.zeros(0, dtype=int)
        kc = kc_by_board.get(board)
        FFv = np.zeros((len(t), count))
        if kc is not None and len(t):
            kk = (labels - kc) * rec.L
            for j, d in enumerate(dofs):
                if d in FF_INDEX:
                    FFv[:, j] = ff_eval(rec.ff[FF_INDEX[d]], rec.T, kk)
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


def _message_iterations(V):
    """PWM メッセージそれぞれが、どの周回の出力か（値が変わるたびに1つ進む番号。_events と同じ数え方）"""
    lab = np.zeros(len(V), dtype=int)
    for m in range(1, len(V)):
        lab[m] = lab[m - 1] + (0 if np.array_equal(V[m], V[m - 1], equal_nan=True) else 1)
    return lab


def write_trial_grid(path, analysis, L):
    """周回番号でそろえた解析用の時系列（k, t = k·L と、自由度ごとの実測・目標軌道・モデル・入力・z3 など）"""
    if not analysis:
        return
    N = max(a.trial.N for a in analysis.values())
    cols = [('k', np.arange(N + 1)), ('t', np.arange(N + 1) * L)]
    for d, a in sorted(analysis.items()):
        tr, sim = a.trial, a.ident['sim']
        pad = lambda v: np.concatenate([np.asarray(v, dtype=float), np.full(N + 1 - len(v), np.nan)])[:N + 1]
        cols += [(f'POT{d}', pad(tr.y)), (f'POTfromESO{d}', pad(tr.y_float.astype(float))),
                 (f'target{d}', pad(a.y_tgt)), (f'model{d}', pad(sim['z'] + tr.P0)),
                 (f'PWM{d}', pad(tr.pwm)), (f'FF{d}', pad(tr.ff)), (f'ADRC{d}', pad(tr.uA)),
                 (f'ADRCmodel{d}', pad(sim['uA'])), (f'PWMmodel{d}', pad(sim['pwm'])),
                 (f'z3_{d}', pad(tr.z3)), (f'z3model_{d}', pad(sim['z3'])), (f'z1_{d}', pad(tr.z1)),
                 (f'z2_{d}', pad(tr.z2)), (f'derivatives_{d}', pad(tr.D))]
    with open(path, 'w', newline='', encoding=CSV_ENCODING) as f:
        w = csv.writer(f)
        w.writerow([c for c, _ in cols])
        for i in range(N + 1):
            w.writerow([_cell(float(v[i])) for _, v in cols])


def save_loop_npz(folder, loop, payload):
    """ループ1回ぶんのデータを <folder>/loops/loop_NN.npz に書く（一時ファイルに書いてから置き換える）"""
    path = os.path.join(folder, LOOP_DIR, f'loop_{loop:02d}.npz')
    tmp = path + '.tmp'
    with open(tmp, 'wb') as f:
        np.savez(f, **payload)
    os.replace(tmp, path)
    return path


def loop_payload(loop, rec, analysis, J, el_results=None, session='', is_best=False, label=''):
    """ビューアが使う配列をまとめる（対象自由度の順に積む）"""
    dofs = sorted(analysis)
    N = max((a.trial.N for a in analysis.values()), default=0)
    pad = lambda v: np.concatenate([np.asarray(v, dtype=float), np.full(N + 1 - len(v), np.nan)])[:N + 1]
    stack = lambda f: np.array([pad(f(analysis[d])) for d in dofs]) if dofs else np.zeros((0, N + 1))
    vec = lambda f, n: np.array([np.asarray(f(analysis[d]), dtype=float) for d in dofs]).reshape(len(dofs), n)
    p = {
        'loop': loop, 'label': label, 'session': session, 'time': time.strftime('%Y-%m-%d %H:%M:%S'),
        'T': rec.T, 'L': rec.L, 'N': N, 'J': J, 'is_best': is_best, 'dofs': np.array(dofs, dtype=int),
        't': np.arange(N + 1) * rec.L,
        'pot': stack(lambda a: a.trial.y), 'pot_float': stack(lambda a: a.trial.y_float.astype(float)),
        'tgt': stack(lambda a: a.y_tgt), 'sys': stack(lambda a: a.ident['sim']['z'] + a.trial.P0),
        'pwm': stack(lambda a: a.trial.pwm), 'ff': stack(lambda a: a.trial.ff), 'uA': stack(lambda a: a.trial.uA),
        'uA_sim': stack(lambda a: a.ident['sim']['uA']), 'pwm_sim': stack(lambda a: a.ident['sim']['pwm']),
        'z3': stack(lambda a: a.trial.z3), 'z3_sim': stack(lambda a: a.ident['sim']['z3']),
        'Pi': vec(lambda a: [a.trial.Pi], 1)[:, 0] if dofs else np.zeros(0),
        'Pf': vec(lambda a: [a.trial.Pf], 1)[:, 0] if dofs else np.zeros(0),
        'P0': vec(lambda a: [a.trial.P0], 1)[:, 0] if dofs else np.zeros(0),
        'ic': vec(lambda a: [a.trial.prm.ic], 1)[:, 0] if dofs else np.zeros(0),
        'J_i': vec(lambda a: [a.J_i], 1)[:, 0] if dofs else np.zeros(0),
        'tgt_params': vec(lambda a: a.tgt, 2), 'sys_params': vec(lambda a: a.ident['plant'], 4),
        'id_metrics': vec(lambda a: [a.ident['metrics']['pot'], a.ident['metrics']['pwm'],
                                     a.ident['metrics']['z3_pwm'], a.ident['metrics']['rho']], 4),
        'replay': vec(lambda a: [a.replay['uA_rms'], a.replay['z3_rms_pwm']], 2),
        'coeffs': vec(lambda a: a.trial.coeffs, 5),
        'extrema': vec(lambda a: extrema_of(a.trial.coeffs, rec.T), 4),
    }
    if el_results:
        p['next_coeffs'] = np.array([el_results[d][0].coeffs if d in el_results else np.zeros(5) for d in dofs])
        p['next_extrema'] = np.array([el_results[d][0].extrema if d in el_results else (np.nan,) * 4 for d in dofs])
        p['next_status'] = np.array([el_results[d][0].status if d in el_results else '' for d in dofs])
        p['next_J_el'] = np.array([el_results[d][0].J if d in el_results else np.nan for d in dofs])
        p['next_pred_pot'] = np.array([pad(el_results[d][0].pred['z'].tolist() + [np.nan]) + analysis[d].trial.P0
                                       if d in el_results else np.full(N + 1, np.nan) for d in dofs])
    return p


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
