#!/usr/bin/env python3
#2026/08/04までのオイラーラグランジュ法プログラム
import os                                                       # OSライブラリ
import sys                                                      # Pythonを扱うライブラリ
import time                                                     # 時間
import threading                                                # スレッド処理
import traceback                                                # エラー内容表示
import warnings                                                 # 警告表示を制御

import matplotlib                                               # グラフライブラリ
matplotlib.use('Agg')                                           # 画像保存専用モードに変更
import matplotlib.pyplot as plt                                 # プロット
import numpy as np                                              # 数学計算
import openpyxl                                                 # Excel書き込み
from openpyxl.drawing.image import Image as OpenpyxlImage       # Excelへ画像保存
import pandas as pd                                             # csv保存

import rclpy                                                    # ROS2ライブラリ
from rclpy.node import Node                                     # Nodeクラスの読み込み
from std_msgs.msg import Float32MultiArray, UInt16MultiArray    # ROS2メッセージ型

import scipy.optimize                                           # 最適化ライブラリ
from scipy.signal import cont2discrete                          # 連続時間→離散時間(ZOH)厳密離散化
from cmaes import CMA                                           # CMA-ES（進化戦略）最適化ライブラリ

import tkinter as tk                                            # GUIライブラリ
from tkinter import filedialog                                  # GUIでフォルダ選択

warnings.simplefilter('ignore', RuntimeWarning)                 # RuntimeWarningを非表示
np.seterr(all='ignore')                                         # Numpyのエラーを無地

# ==============================================================================
# 評価関数の重み行列（チューニング要素）
#   コスト関数 J = Σ (eᵀQe + Ru²) の重み（e は目標軌道との誤差状態, u は制御入力）
# ==============================================================================
COST_Q = np.diag([8858.79, 0.191027, 0.0711133])     # 状態誤差の重み（大きいほど誤差を抑える）
COST_R = np.array([[10.7164]])             # 制御入力の重み（大きいほど入力を抑える）

# ==============================================================================
# シミュレーションおよび最適化のパラメータ（チューニング要素）
# ==============================================================================
SIM_TIME = 5.0              # シミュレーション時間および実測データ収集時間（秒）
SIM_DT = 0.01               # シミュレーションのサンプル刻み幅（秒）
INIT_WAIT_TIME = 10.0       # 初期姿勢への移動後の待機時間（秒）
MAX_INNER_ITER = 60         # 内側ループの最大反復回数
THRESHOLD_J = 1000000.0     # 内側ループの収束判定閾値（評価関数Jがこの値以下になれば収束）

# ==============================================================================
# 離散時間オイラー・ラグランジュ（随伴／勾配）法のパラメータ（チューニング要素）
# ==============================================================================
EL_MAX_ITER = 200           # 勾配法の最大反復回数
EL_EPS = 1e-3               # 勾配ノルム Σ||∂H/∂u||² の収束判定閾値 ε
EL_LS_MAX = 40              # ステップ幅 α のバックトラッキング最大試行回数

# ==============================================================================
# 初回システム同定用の初期FF入力の振幅（チューニング要素）
#   ゼロ入力ではシステムモデルの b0 が同定不能（上限に張り付く）ため、十分な励振を
#   与える非ゼロFFを初期値とする。2つの極値の大きさ |f(t1)|=|f(t2)| がこの値になる。
# ==============================================================================
INIT_FF_PEAK = 50.0         # 初期FFの極値の大きさ [PWM]（f(t1)=+50≥40, f(t2)=-50≤-40 を満たす）

# ==============================================================================
# 5次多項式FFフィット（CMA-ES）のパラメータ（チューニング要素）
#   最適入力 u_opt を f(t) = a t^5 + b t^4 + c t^3 + d t^2 - (aT^4+bT^3+cT^2+dT) t で近似する。
#   極値条件（区間(0,T)の内部に f'(t)=0 の実単純根がちょうど2つ）はペナルティで妥協せず、
#   条件を満たした個体だけを採用し、満たす解が出なければ初期値を変えて再探索する。
# ==============================================================================
CMA_POP_SIZE = 16           # CMA-ESの1世代あたりの個体数
CMA_MAX_GEN = 200           # 1回の探索あたりの最大世代数
CMA_SIGMA_RATIO = 0.3       # CMA-ESの初期ステップ幅（u_optの最大振幅に対する比）
CMA_MAX_RESTART = 20        # 極値条件を満たす解が見つかるまでの最大再探索回数
EXTREMA_IMAG_TOL = 1e-6     # f'(t)=0 の根を実根とみなす虚部の許容値
EXTREMA_EPS = 1e-9          # |f''(ti)| > ε で単純根（重根でない）と判定する閾値 ε
FIT_PWM_PENALTY = 1e6       # FF入力を -255～255 に収めるためのペナルティ係数
INFEASIBLE_PENALTY = 1e12   # 極値条件を満たさない個体を必ず劣後させる定数オフセット（採用は条件判定のみで行う）

# ==============================================================================
# 最適化ビューア（別プロセス）へ渡すスナップショットの設定
#   内側ループが1回終わるごとに、24自由度分の波形とパラメータ履歴を1つの .npz へ書き出す。
#   ビューア（view_optimization.py）はこのファイルの更新時刻を監視して自動で再描画する。
# ==============================================================================
ENABLE_SNAPSHOT = True                                      # ビューア用スナップショット出力のON/OFF
SNAPSHOT_PATH = "/tmp/el_optimization_snapshot.npz"         # スナップショットの保存先（ビューア側と同じパスにすること）


# ==============================================================================
# 数学ソルバー (System ID & 離散時間オイラー・ラグランジュ最適制御)
# ==============================================================================
class MathematicalSolver:
    # コンストラクタ
    def __init__(self, T, dt=SIM_DT, Q=None, R=None):       # 引数(FF制御入力時間, シミュレーションステップ時間, 状態重み行列, 入力重み行列)
        self.T = T                                          # FF制御入力時間を保存
        self.dt = dt                                        # シミュレーションステップ時間を保存
        self.t_eval = np.arange(0, SIM_TIME, self.dt)       # シミュレーション時間配列を作成
        self.Q = Q if Q is not None else COST_Q.copy()      # 状態重み行列の保存（引数 Q が与えられていればそれを使用し、与えられていなければ COST_Q をコピー）
        self.R = R if R is not None else COST_R.copy()      # 入力重み行列の保存（引数 R が与えられていればそれを使用し、与えられていなければ COST_R をコピー）
        self.last_fit_restart = 0                           # 直近の fit_ff_poly が何回目の再探索で条件を満たしたか（-1は退避。ビューア表示用）

    # ------------------------------------------------------------------
    # 離散化・シミュレーション
    # ------------------------------------------------------------------

    # 3次遅れ系（可制御正準形）を scipy.signal.cont2discrete() でZOH厳密離散化する関数
    def _discretize(self, a2, a1, a0, b0):                          # 引数(3次遅れ系の係数 a2・a1・a0・b0)
        """連続時間状態方程式 ẋ = A_c x + B_c u をZOH（Zero-Order Hold）で厳密離散化する。

            連続系（可制御正準形, 出力 y = x0）
                A_c = [[0,1,0],[0,0,1],[-a0,-a1,-a2]],  B_c = [0,0,b0]ᵀ
                → 状態 x = [位置 y, 速度 ẏ, 加速度 ÿ]
            離散系（ZOH厳密離散化）
                x(k+1) = A_d x(k) + B_d u(k),  A_d = expm(A_c·dt),  B_d = (∫₀^{dt} expm(A_c τ)dτ) B_c
        """
        A_c = np.array([[0.0,   1.0,   0.0],
                        [0.0,   0.0,   1.0],
                        [-a0,   -a1,   -a2]])                        # 連続時間 A（可制御正準形）
        B_c = np.array([[0.0], [0.0], [b0]])                        # 連続時間 B
        C_d = np.zeros((1, 3))                                      # 出力行列（状態フィードバックのためダミー）
        D_d = np.zeros((1, 1))                                      # 直達行列（ダミー）
        A_d, B_d, _, _, _ = cont2discrete((A_c, B_c, C_d, D_d), self.dt, method="zoh")   # ZOH厳密離散化
        return A_d, B_d.flatten()                                   # A_d (3x3), B_d (3,) を返す

    # 目標モデルの自由応答を計算する関数
    def simulate_unforced(self, t_array, a2, a1, a0, y0):           # 引数(シミュレーション時間, 3次遅れ系の係数 a2・a1・a0, 初期偏差)
        """目標モデル（入力なし）のZOH離散状態方程式 x(k+1)=A_d x(k) によるシミュレーション"""
        N = len(t_array)                                            # シミュレーション時間サンプル数を取得
        A_d, _ = self._discretize(a2, a1, a0, 0.0)                  # 目標モデルをZOH離散化（b0=0）
        x = np.zeros((N, 3))                                        # 状態列 x(k) = [位置, 速度, 加速度]
        x[0] = np.array([y0, 0.0, 0.0])                             # 初期状態 x(0)=[y0,0,0]（位置 y0, 速度0, 加速度0）
        for k in range(N - 1):                                     # 自由応答を逐次計算
            x[k + 1] = A_d @ x[k]                                   # x(k+1) = A_d x(k)
        y_traj = x[:, 0]                                            # 出力（位置成分）
        dy_traj = x[:, 1]                                           # 速度成分
        ddy_traj = x[:, 2]                                          # 加速度成分
        dddy_traj = -a0 * x[:, 0] - a1 * x[:, 1] - a2 * x[:, 2]     # 加加速度（連続時間の ẋ2 = -a0 x0 - a1 x1 - a2 x2）
        return y_traj, dy_traj, ddy_traj, dddy_traj

    # システムモデルの応答(FF入力)を計算する関数
    def simulate_forced(self, t_array, a2, a1, a0, b0, u_array, y0):    # 引数(シミュレーション時間, 3次遅れ系の係数 a2・a1・a0・b0, FF入力, 初期偏差)
        """システムモデル（FF入力あり）のZOH離散状態方程式 x(k+1)=A_d x(k)+B_d u(k) によるシミュレーション"""
        N = len(t_array)                                                # シミュレーション時間サンプル数を取得
        A_d, B_d = self._discretize(a2, a1, a0, b0)                     # システムモデルをZOH離散化
        x = np.zeros((N, 3))                                            # 状態列 x(k) = [位置, 速度, 加速度]
        x[0] = np.array([y0, 0.0, 0.0])                                 # 初期状態 x(0)=[y0,0,0]
        for k in range(N - 1):                                         # FF入力ありの応答を逐次計算
            x[k + 1] = A_d @ x[k] + B_d * u_array[k]                    # x(k+1) = A_d x(k) + B_d u(k)
        y_traj = x[:, 0]                                                # 出力（位置成分）
        dy_traj = x[:, 1]                                               # 速度成分
        ddy_traj = x[:, 2]                                              # 加速度成分
        return y_traj, dy_traj, ddy_traj

    # 目標モデルの3次遅れ系係数を計算する関数（ "_"が付いているので外から直接呼べない内部専用関数）
    def _target_coeffs(self, T1, wn):           # 引数(1次遅れ系の時定数, 固有振動数)
        """減衰係数1固定の目標モデル係数"""
        a2 = (2 * wn * T1 + 1) / T1
        a1 = (wn ** 2 * T1 + 2 * wn) / T1
        a0 = (wn ** 2) / T1
        return a2, a1, a0

    # 同定の初期値を決める関数（ "_"が付いているので外から直接呼べない内部専用関数）
    @staticmethod
    def _init_guess(prev_params, default):                                          # 引数(前回の同定結果, 前回結果が無いときの既定初期値)
        """同定（least_squares）の初期値 x0 を決める。

        前回の同定結果があればそれを初期値に使う（ウォームスタート）。内側ループごとに
        モデルはわずかしか変化しないため、前回値から始めた方が収束が速く、同定値が
        ループ間で飛びにくい。前回値が無い（初回）か、要素数が合わない・NaN/Infを含む
        場合は既定の初期値へ戻す。
        """
        if prev_params is not None and len(prev_params) == len(default) and np.all(np.isfinite(prev_params)):
            return [float(v) for v in prev_params]                                  # 前回の同定結果を初期値にする
        return list(default)                                                        # 初回・異常値のときは既定の初期値にする

    # 目標モデル同定関数
    def fit_target_model(self, y_data, y0, prev_params=None):                               # 引数(実測データ, 初期偏差, 前回の同定結果[T1, wn])
        """目標モデル同定: 1 / ((T1*s + 1)(s^2 + 2*wn*s + wn^2))（減衰係数 zeta=1 固定）

        勾配ベースの信頼領域反射法（scipy.optimize.least_squares, method='trf'）で
        各時刻の残差 r(k)=y_sim(k)-y_data(k) の二乗和 Σr² を最小化する（Nelder-Mead は使用しない）。
        初期値 x0 は前回の同定結果（prev_params）があればそれを使う（ウォームスタート）。
        """
        # 各時刻の残差ベクトルを返す関数（least_squares は Σr² を最小化する）
        def residuals(p):
            T1, wn = p                                                              # 最適化変数の取り出し
            a2, a1, a0 = self._target_coeffs(T1, wn)                                # 減衰係数1固定の3次遅れ系の係数を計算
            y_sim, _, _, _ = self.simulate_unforced(self.t_eval, a2, a1, a0, y0)    # 目標モデルの自由応答を計算
            return y_sim - y_data                                                   # 残差ベクトルを返す

        res = scipy.optimize.least_squares(                                                 # Σr² が最小になる変数[T1, wn]を最適化する
            residuals, x0=self._init_guess(prev_params, [0.1, 10.0]),                       # 初期値（前回の同定結果 or 既定値）
            bounds=([1e-6, 1e-6], [np.inf, np.inf]),                                        # T1>0, wn>0（負の極を排除）
            method='trf',
        )
        T1, wn = res.x                                                                      # 最適変数を取り出す
        return T1, wn

    # システムモデル同定関数
    def fit_system_model(self, y_data, u_ff, y0, prev_params=None):                                 # 引数(実測データ, FF制御入力, 初期偏差, 前回の同定結果[a2, a1, a0, b0])
        """システムモデル同定: b0 / (s^3 + a2*s^2 + a1*s + a0)

        目標モデルと同じく勾配ベースの信頼領域反射法（least_squares, method='trf'）で
        残差二乗和を最小化する。b0は励振（非ゼロFF入力）があってはじめて同定できるため、
        初回同定では INIT_FF_PEAK 振幅の初期FFで励振する。
        初期値 x0 は前回の同定結果（prev_params）があればそれを使う（ウォームスタート）。
        """
        # 各時刻の残差ベクトルを返す関数
        def residuals(p):
            a2, a1, a0, b0 = p                                                          # 最適化変数の取り出し
            y_sim, _, _ = self.simulate_forced(self.t_eval, a2, a1, a0, b0, u_ff, y0)   # システムモデルの応答を計算
            return y_sim - y_data                                                       # 残差ベクトルを返す

        res = scipy.optimize.least_squares(                                                         # Σr² が最小になる変数[a2, a1, a0, b0]を最適化する
            residuals, x0=self._init_guess(prev_params, [10.0, 100.0, 1000.0, 1000.0]),             # 初期値（前回の同定結果 or 既定値）
            bounds=([1e-6, 1e-6, 1e-6, -np.inf], [np.inf, np.inf, np.inf, np.inf]),                 # a2,a1,a0>0（負の極を排除）, b0は符号自由
            method='trf',
        )
        return res.x                                                                                # 最適変数を返す

    # 極値計算関数
    def calc_extrema_from_ff(self, ff_params):                                                                  # 引数(FFパラメータ)
        a, b, c, d, e = ff_params

        roots = np.roots([5*a, 4*b, 3*c, 2*d, e])                                                               # 極値を計算

        real_roots = sorted(
            r.real for r in roots if abs(r.imag) < 1e-6 and 0 < r.real < self.T                                 # FF入力時間における実部の極を取り出し、小さい順に並べる
        )

        if len(real_roots) >= 2:                                                                                # 極値が2つ以上なら、一番早い極値と二番目の極値を使う
            t1, t2 = real_roots[:2]
        elif len(real_roots) == 1:                                                                              # 極値が1つなら、その極値とFF入力時間の半分の値を使う
            t1 = real_roots[0]
            t2 = self.T / 2
        else:                                                                                                   # 極値が0なら、FF入力時間から算出する
            t1 = self.T * 0.33
            t2 = self.T * 0.66
        
        # 5次関数計算
        def poly(t):
            return a*t**5 + b*t**4 + c*t**3 + d*t**2 + e*t

        y1 = poly(t1)                                                                                           # t1における5次関数値y1を計算
        y2 = poly(t2)                                                                                           # t2における5次関数値y2を計算

        return (t1, y1, t2, y2)

    # 初回システム同定用の初期FF入力（5次係数[a,b,c,d,e]）を生成する関数
    @staticmethod
    def initial_ff_params(T, peak=INIT_FF_PEAK):                                                        # 引数(FF制御入力時間, 極値の大きさ)
        """初回同定用の初期FF入力 f(t) = C·t(t-T/2)(t-T) の5次係数 [a,b,c,d,e] を返す。

        ゼロ入力ではシステムモデルの b0 が同定不能（上限に張り付く）ため、十分な励振を
        与える非ゼロFFを初期値とする。この関数の f(t) は必須条件を厳密に満たす：
          ・f(0) = f(T) = 0                （因子 t, (t-T) より）
          ・0<t<T に極値ちょうど2個         （t1=T(3-√3)/6≈0.211T, t2=T(3+√3)/6≈0.789T）
          ・f(t1) = +peak ≥ 40, f(t2) = -peak ≤ -40   （t=T/2 まわりで反対称、|極値|=peak）
        f(t) は t^5・t^4 の項を持たない退化5次（a=b=0）で、標準形 f = c t³ + d t² + e t に展開する。
        極値の大きさは |f(t1)| = (√3/36)·C·T³ = peak より C = 12√3·peak / T³。
        """
        C = 12.0 * np.sqrt(3.0) * peak / (T ** 3)                                                       # 極値の大きさが peak になる振幅
        a = 0.0                                                                                         # t^5 の係数
        b = 0.0                                                                                         # t^4 の係数
        c = C                                                                                           # t^3 の係数
        d = -1.5 * C * T                                                                                # t^2 の係数
        e = 0.5 * C * T ** 2                                                                            # t^1 の係数（f(T)=0 を満たす）
        return [a, b, c, d, e]

    # 5次関数が満たすべき極値条件を判定する関数
    def check_extrema_condition(self, a, b, c, d):                                      # 引数(5次関数パラメータ a・b・c・d)
        """区間 (0,T) の内部に f'(t)=0 の実単純根がちょうど2つ存在するかを判定する。

        判定する条件
            ∃ t1,t2 ∈ (0,T), 0 < t1 < t2 < T,  f'(t1)=f'(t2)=0,  f''(t1)≠0, f''(t2)≠0
            かつ ∀t∈(0,T), f'(t)=0 ⇔ (t=t1)∨(t=t2)
        条件は区間 (0,T) の内部だけに適用する。t<0 や t>T にある実根、および複素根は
        いっさい制約しない（区間外に極値があってもよい）。

        手順
            1. 導関数 f'(t) = 5a t^4 + 4b t^3 + 3c t^2 + 2d t + e の根を求める
            2. |Im(root)| ≤ EXTREMA_IMAG_TOL の実根だけを抽出する
            3. 0 < t < T の内部にある実根だけを対象にする
            4. その個数がちょうど2個であることを判定する
            5. 各根で |f''(ti)| > EXTREMA_EPS を確認し、単純根（重根でない）であることを保証する

        条件を満たせば極値時刻 (t1, t2)（t1<t2）を返し、満たさなければ None を返す。
        """
        e = -(a * self.T ** 4 + b * self.T ** 3 + c * self.T ** 2 + d * self.T)         # f(T)=0 から決まる5次関数パラメータe
        if not np.all(np.isfinite([a, b, c, d, e])):                                    # 係数が発散・NaNなら不採用
            return None
        roots = np.roots([5 * a, 4 * b, 3 * c, 2 * d, e])                               # 手順1: 導関数 f'(t) の根を求める
        real_roots = sorted(                                                            # 手順2,3: 実根かつ 0<t<T の内部にあるものだけを取り出す
            r.real for r in roots if abs(r.imag) <= EXTREMA_IMAG_TOL and 0.0 < r.real < self.T
        )
        if len(real_roots) != 2:                                                        # 手順4: 内部の実根がちょうど2個でなければ不採用
            return None
        t1, t2 = real_roots                                                             # 0 < t1 < t2 < T
        for ti in (t1, t2):                                                             # 手順5: 各根が単純根であることを確認
            ddf = 20 * a * ti ** 3 + 12 * b * ti ** 2 + 6 * c * ti + 2 * d              # f''(t) = 20a t^3 + 12b t^2 + 6c t + 2d
            if abs(ddf) <= EXTREMA_EPS:                                                 # |f''(ti)| ≤ ε は重根とみなして不採用
                return None
        return t1, t2                                                                   # 条件を満たした極値時刻を返す

    # 最適入力を5次関数で近似する関数（CMA-ES + 極値条件を満たす解のみ採用）
    def fit_ff_poly(self, t_ff, u_opt_ff):                                              # 引数(FF入力時間の配列, その区間の最適入力)
        """最適入力 u_opt を f(t)=a t^5+b t^4+c t^3+d t^2-(aT^4+bT^3+cT^2+dT) t で近似する。

        探索アルゴリズムは CMA-ES（cmaes.CMA）。Nelder-Mead は局所解に陥りやすく、
        極値条件を満たす解へ到達しにくいため置き換えた。

        極値条件はペナルティで妥協せず、check_extrema_condition() を満たした個体だけを
        採用候補にする（満たさない個体は INFEASIBLE_PENALTY を加えて必ず劣後させるが、
        たとえ最良個体でも採用しない）。1回の探索で条件を満たす個体が1つも出なければ、
        新しい初期値（平均ベクトル）と広げたステップ幅で最初から再探索し、条件を満たす
        5次関数が見つかるまで最大 CMA_MAX_RESTART 回まで繰り返す。

        探索空間は端点条件 f(0)=f(T)=0 を満たす基底 φ_j(t)=t^(j+2)-T^(j+1) t を
        最大値1へ正規化した係数（4次元）。T や振幅が変わっても探索スケールが一定になり、
        CMA-ES が安定して働く。
        """
        # 端点条件 f(0)=f(T)=0 を満たす基底 φ_j を作り、最大値が1になるよう正規化する
        basis = np.stack([
            t_ff ** 5 - self.T ** 4 * t_ff,                                             # φ_a(t)（係数a に対応）
            t_ff ** 4 - self.T ** 3 * t_ff,                                             # φ_b(t)（係数b に対応）
            t_ff ** 3 - self.T ** 2 * t_ff,                                             # φ_c(t)（係数c に対応）
            t_ff ** 2 - self.T * t_ff,                                                  # φ_d(t)（係数d に対応）
        ])
        scale = np.max(np.abs(basis), axis=1)                                           # 各基底の最大値（正規化の分母）
        scale[scale < 1e-300] = 1.0                                                     # 0除算の回避
        basis_n = basis / scale[:, None]                                                # 正規化した基底（最大値1）

        amp = float(np.max(np.abs(u_opt_ff)))                                           # 最適入力の振幅（探索スケールの基準）
        if not np.isfinite(amp) or amp < 1e-6:                                          # 最適入力がほぼ0なら初期FFの振幅を基準にする
            amp = INIT_FF_PEAK

        # 探索の初期値: 極値条件を必ず満たす形 f(t)=C t(t-T/2)(t-T) を u_opt へ最小二乗で当てたもの
        #   この形の極値は t=T(3±√3)/6 の2点のみで常に単純根なので、実行可能な点から探索を始められる
        g = t_ff ** 3 - 1.5 * self.T * t_ff ** 2 + 0.5 * self.T ** 2 * t_ff             # 形状 t(t-T/2)(t-T)
        denom = float(g @ g)                                                            # 最小二乗の分母
        C = float(g @ u_opt_ff) / denom if denom > 1e-300 else 0.0                      # 二乗誤差が最小になる振幅C
        if not np.isfinite(C) or abs(C) < 1e-12:                                        # u_optがほぼ0のときは初期励振FFと同じ振幅にする
            C = 12.0 * np.sqrt(3.0) * INIT_FF_PEAK / (self.T ** 3)
        params_init = np.array([0.0, 0.0, C, -1.5 * C * self.T])                        # f(t)=C t(t-T/2)(t-T) の係数 [a,b,c,d]
        z_init = params_init * scale                                                    # 正規化した探索空間での初期値

        # 最適入力との二乗和誤差 + PWM範囲のペナルティ（極値条件はここには入れない）
        def fit_loss(z):                                                                # 引数(正規化した5次関数パラメータ)
            u_pred = z @ basis_n                                                        # 5次関数で計算したFF入力
            mse = np.sum((u_pred - u_opt_ff) ** 2)                                      # 最適入力と近似したFF入力との二乗和誤差
            penalty_pwm = (                                                             # -255～255の間に収めるためのペナルティ
                np.sum(np.maximum(0, u_pred - 255) ** 2)
                + np.sum(np.maximum(0, -255 - u_pred) ** 2)
            )
            return mse + FIT_PWM_PENALTY * penalty_pwm

        rng = np.random.default_rng(0)                                                  # 再探索の初期値生成用（再現性のため固定シード）

        # 条件を満たす5次関数が見つかるまで、初期値を変えて CMA-ES による探索を繰り返す
        for restart in range(CMA_MAX_RESTART):
            if restart == 0:                                                            # 初回は実行可能な初期値から探索する
                mean = z_init.copy()
            else:                                                                       # 再探索は新しい初期値・広げたステップ幅でやり直す
                mean = z_init + rng.normal(0.0, amp, 4)
            optimizer = CMA(                                                            # CMA-ESの生成
                mean=mean,
                sigma=CMA_SIGMA_RATIO * amp * (1.0 + restart),                          # 初期ステップ幅（再探索ごとに広げる）
                population_size=CMA_POP_SIZE,
                seed=restart + 1,
            )

            best_params = None                                                          # 極値条件を満たした中で最良の [a,b,c,d]
            best_extrema = None                                                         # そのときの極値時刻 (t1,t2)
            best_loss = np.inf                                                          # そのときの評価値

            for _gen in range(CMA_MAX_GEN):                                             # 世代ループ
                solutions = []                                                          # (個体, 評価値) のリスト
                for _ in range(optimizer.population_size):                              # 個体ループ
                    z = optimizer.ask()                                                 # 個体を生成
                    value = fit_loss(z)                                                 # 近似誤差を計算
                    a, b, c, d = z / scale                                              # 元の5次関数パラメータへ戻す
                    extrema = self.check_extrema_condition(a, b, c, d)                  # 極値条件を判定
                    if extrema is None:                                                 # 条件を満たさない個体は採用せず、必ず劣後させる
                        value += INFEASIBLE_PENALTY
                    elif value < best_loss:                                             # 条件を満たした個体のみ採用候補にする
                        best_loss = value
                        best_params = (a, b, c, d)
                        best_extrema = extrema
                    solutions.append((z, value))                                        # 評価結果を格納
                optimizer.tell(solutions)                                               # CMA-ESの分布を更新
                if optimizer.should_stop():                                             # 収束したら世代ループを抜ける
                    break

            if best_params is not None:                                                 # 条件を満たす5次関数が見つかったので採用する
                self.last_fit_restart = restart                                         # 何回目の再探索で見つかったか（ビューア表示用）
                return best_params, best_extrema

        # 最大再探索回数でも見つからない場合は、極値条件を必ず満たす形 f(t)=C t(t-T/2)(t-T) へ退避する
        self.last_fit_restart = -1                                                      # 退避したことを示す値（ビューア表示用）
        print(f"[warn] 極値条件を満たす5次関数が {CMA_MAX_RESTART} 回の再探索で見つからず、C·t(t-T/2)(t-T) で代替します")
        params = (0.0, 0.0, C, -1.5 * C * self.T)                                       # この形の極値は t=T(3±√3)/6 の2点のみ（常に単純根）
        return params, self.check_extrema_condition(*params)

    # ------------------------------------------------------------------
    # 離散時間オイラー・ラグランジュ（随伴／勾配）法による最適制御 + 5次多項式フィット
    # ------------------------------------------------------------------
    def calculate_el_ff(self, target_params, sys_params, u_ff, y0):                                          # 引数(目標モデルのパラメータ[T1, wn], システムモデルのパラメータ[a2, a1, a0, b0], FF制御入力, 初期偏差)
        """
        離散時間オイラー・ラグランジュ（随伴／勾配）法で最適制御入力を計算し、
        5次多項式 FF = a*t^5 + ... + e*t にフィットする。

        目的：システムモデルの軌道を、入力 u=0 で生成した目標軌道 x_tgt へ一致させる。
          入力ホライズン : u(k) は [0,T]（k=0..N_ff-1）のみ最適化し、それ以降は0
          評価ホライズン : コスト J はシミュレーション全体（5秒, k=0..M-1）で評価する

        ● 離散状態方程式（可制御正準形・ZOH厳密離散化）
            システム : x_sys(k+1) = A_sys x_sys(k) + B_sys u(k)
            目標(u=0): x_tgt(k+1) = A_tgt x_tgt(k)

        ● 誤差状態 x(k)=x_sys(k)-x_tgt(k) を状態変数とした状態方程式（e と表記, e(0)=0）
            e(k+1) = x_sys(k+1) - x_tgt(k+1)
                   = A_sys( e(k)+x_tgt(k) ) + B_sys u(k) - A_tgt x_tgt(k)
                   = A_sys e(k) + B_sys u(k) + d(k),   d(k) = (A_sys - A_tgt) x_tgt(k)  （既知の入力項）

        ● コスト関数（終端コストなし）／ハミルトニアン
            J = Σ_{k=0}^{M-1} ( e(k)ᵀ Q e(k) + R u(k)² )
            H(k) = e(k)ᵀ Q e(k) + R u(k)² + λ(k+1)ᵀ [ A_sys e(k) + B_sys u(k) + d(k) ]

        ● 随伴方程式（既知項 d は ∂d/∂e = 0 より随伴に現れない）
            λ(N) = λ(M) = 0
            λ(k) = ∂H/∂e(k) = 2 Q e(k) + A_sysᵀ λ(k+1)

        ● 勾配（入力ホライズン k=0..N_ff-1）
            ∂H/∂u(k) = 2 R u(k) + B_sysᵀ λ(k+1)

        解法（未知変数は [0,T] の入力列 u(k)、k≥N_ff では u(k)=0）：
          1. u(k) を初期化する（零入力から開始）
          2. 誤差状態 e を状態変数とした状態方程式から e(k) を順方向計算（5秒全体）
          3. λ(N) = 0
          4. 随伴方程式を逆方向計算（5秒全体）
          5. ∂H/∂u を計算（k=0..N_ff-1）
          6. Σ||∂H/∂u||² < ε なら終了
          7. そうでなければ u ← clip(u - α ∂H/∂u, -255, 255) として 2 へ戻る
        """
        T1, wn = target_params                                                                          # 目標モデルのパラメータ取得
        a2_tgt, a1_tgt, a0_tgt = self._target_coeffs(T1, wn)                                            # 目標モデルの3次遅れ系の係数を計算
        a2_sys, a1_sys, a0_sys, b0_sys = sys_params                                                     # システムモデルのパラメータ取得

        dt = self.dt                                                                                    # シミュレーションのサンプル刻み幅
        M = len(self.t_eval)                                                                            # 評価ホライズン（5秒全体）のステップ数
        N_ff = int(round(self.T / dt))                                                                  # 入力ホライズン [0,T] のステップ数
        n_x = 3                                                                                         # 状態次元 (可制御正準形: x0=位置, x1=速度, x2=加速度)

        # -----------------------------------------------------------
        # 同定結果から離散時間状態方程式を構築（cont2discrete によるZOH厳密離散化）
        #   システム : x_sys(k+1) = A_sys x_sys(k) + B_sys u(k)
        #   目標(u=0): x_tgt(k+1) = A_tgt x_tgt(k)
        # -----------------------------------------------------------
        A_sys, B_sys = self._discretize(a2_sys, a1_sys, a0_sys, b0_sys)                                 # システムモデルのZOH離散状態方程式
        A_tgt, _ = self._discretize(a2_tgt, a1_tgt, a0_tgt, 0.0)                                        # 目標モデルのZOH離散状態方程式（u=0）

        # 目標軌道 x_tgt(k)（u=0 の目標モデル自由応答）を A_tgt の再帰で生成
        #   x_tgt(0) = [y0, 0, 0],  x_tgt(k+1) = A_tgt x_tgt(k)
        x_tgt = np.zeros((M, n_x))                                                                      # 目標状態列 (M, 3)
        x_tgt[0] = np.array([y0, 0.0, 0.0])                                                             # 初期状態
        for k in range(M - 1):                                                                          # 目標モデルの自由応答を逐次計算
            x_tgt[k + 1] = A_tgt @ x_tgt[k]                                                             # x_tgt(k+1) = A_tgt x_tgt(k)
        y_tgt = x_tgt[:, 0]                                                                             # 目標出力（位置成分）

        # 誤差状態方程式の既知入力項 d(k) = (A_sys - A_tgt) x_tgt(k)
        D = x_tgt @ (A_sys - A_tgt).T                                                                   # D[k] = (A_sys - A_tgt) x_tgt(k) → 形状 (M, 3)

        Q = self.Q                                                                                      # 状態誤差の重み行列 (3x3)
        R = float(self.R[0, 0])                                                                         # 入力の重みスカラー

        # ---- 手順2: 誤差状態 e を順方向に計算する関数（e(0)=x_sys(0)-x_tgt(0)=0） ----
        #   e(k+1) = A_sys e(k) + B_sys u(k) + d(k)。入力は [0,T](k<N_ff) のみ、それ以降は0。
        def forward(u_seq):
            e = np.zeros((M + 1, n_x))                                                                  # 誤差状態列 e(0..M)（e(0)=0）
            for k in range(M):                                                                          # k=0..M-1 を順方向に更新
                uk = u_seq[k] if k < N_ff else 0.0                                                      # 入力は [0,T] のみ、それ以降は0
                e[k + 1] = A_sys @ e[k] + B_sys * uk + D[k]                                             # e(k+1)=A_sys e(k)+B_sys u(k)+d(k)
            return e

        # ---- コスト関数 J = Σ_{k=0}^{M-1} ( e(k)ᵀ Q e(k) + R u(k)² )（5秒全体で評価） ----
        def cost(e_traj, u_seq):
            state_cost = np.einsum('ki,ij,kj->', e_traj[:M], Q, e_traj[:M])                             # Σ e(k)ᵀ Q e(k)（5秒全体）
            return float(state_cost + R * np.sum(u_seq ** 2))                                           # + R Σ u(k)²（[0,T]）

        # ---- 手順3,4: 随伴方程式を逆方向計算し λ(1..M) を求める関数 ----
        #   λ(M)=0（手順3, 終端コストなし）, λ(k)=2 Q e(k)+A_sysᵀ λ(k+1)（手順4）
        def backward(e_traj):
            lam = np.zeros((M + 1, n_x))                                                                # 随伴変数列 λ(0..M)（λ(M)=0）
            for k in range(M - 1, 0, -1):                                                               # k=M-1..1 を逆方向に更新
                lam[k] = 2.0 * (Q @ e_traj[k]) + A_sys.T @ lam[k + 1]                                   # λ(k)=2 Q e(k)+A_sysᵀ λ(k+1)
            return lam

        # ---- 手順5: 勾配 ∂H/∂u(k)=2 R u(k)+B_sysᵀ λ(k+1)（入力ホライズン k=0..N_ff-1） ----
        def gradient(u_seq, lam):
            return 2.0 * R * u_seq + lam[1:N_ff + 1] @ B_sys                                            # k=0..N_ff-1 の勾配ベクトル（λ(k+1)=lam[k+1]）

        # ---- 手順1: 入力列 u(k) を初期化（零入力から開始） ----
        u = np.zeros(N_ff)                                                                              # 決定変数 u(0..N_ff-1)（[0,T] のみ）

        # ---- 手順2〜7: 勾配降下の反復 ----
        for _ in range(EL_MAX_ITER):
            e_traj = forward(u)                                                                         # 手順2: 誤差状態を順方向計算
            J0 = cost(e_traj, u)                                                                        # 現在のコスト
            lam = backward(e_traj)                                                                      # 手順3,4: 随伴方程式を逆方向計算
            grad = gradient(u, lam)                                                                     # 手順5: 勾配 ∂H/∂u を計算

            gnorm2 = float(grad @ grad)                                                                 # 勾配ノルム Σ||∂H/∂u||²
            if gnorm2 < EL_EPS:                                                                         # 手順6: Σ||∂H/∂u||² < ε なら収束
                break

            # 手順7: u ← clip(u - α ∂H/∂u, -255, 255)
            #   固定 α では発散しうるため、コストが減少する α をバックトラッキングで選ぶ。
            #   J は u の2次形式なので、有界プローブ点から厳密最小ステップ α*=||g||²/(gᵀH g) を推定して初期値にする。
            gmax = float(np.max(np.abs(grad)))                                                          # 勾配の最大成分（プローブ幅の基準）
            if gmax < 1e-12:                                                                            # 勾配がほぼ0なら収束
                break
            s = 255.0 / gmax                                                                            # 過大ステップを避ける有界プローブ幅（最大成分が255動く）
            Js = cost(forward(u - s * grad), u - s * grad)                                              # 方向 -grad 上の1点でコストを評価
            gHg = 2.0 * (Js - J0 + s * gnorm2) / (s * s)                                                # 2次曲率 gᵀH g（Jはuの2次形式）
            alpha = gnorm2 / gHg if gHg > 1e-30 else s                                                  # 厳密最小ステップ α*=||g||²/(gᵀH g)

            accepted = False                                                                            # コストを減少させる α が見つかったか
            for _ls in range(EL_LS_MAX):                                                                # バックトラッキング
                u_cand = np.clip(u - alpha * grad, -255.0, 255.0)                                       # PWM制約 -255≤u≤255 を満たすようclip
                J_cand = cost(forward(u_cand), u_cand)                                                  # 候補入力のコスト
                if J_cand < J0:                                                                         # コストが減少したら採用
                    accepted = True
                    break
                alpha *= 0.5                                                                            # 減少しなければステップ幅を半分にして再試行
            if not accepted:                                                                            # どのステップ幅でも改善しなければ収束
                break
            u = u_cand                                                                                  # 入力列を更新
            if J0 - J_cand < 1e-9 * (abs(J0) + 1.0):                                                    # 改善が微小なら収束
                break

        # 最適入力をシミュレーション時間全体の配列へ格納（FF入力区間以外は0）
        u_opt = np.zeros(len(self.t_eval))                                                              # 最適入力を保存する配列
        u_opt[:N_ff] = u                                                                                # FF入力区間 (0..N_ff-1) の最適入力を格納

        # 5次多項式フィット（0≤t≤T, 端点0, 極値2個）
        t_ff = self.t_eval[self.t_eval <= self.T]                                                       # FF入力を与える時間だけ取り出す
        u_opt_ff = u_opt[: len(t_ff)]                                                                   # FF入力を与える区間だけの最適入力を取り出す

        # 5次関数を定義する関数
        def poly(t, a, b, c, d):                                                                        # 引数(時間, 5次関数パラメータa・b・c・d)
            e = -(a * self.T ** 4 + b * self.T ** 3 + c * self.T ** 2 + d * self.T)     # 5次関数パラメータeを計算
            return a * t ** 5 + b * t ** 4 + c * t ** 3 + d * t ** 2 + e * t            # 5次関数FF入力値を返す

        # CMA-ESで5次関数近似（極値条件を満たす解のみ採用し、満たすまで初期値を変えて再探索）
        (a, b, c, d), (t1, t2) = self.fit_ff_poly(t_ff, u_opt_ff)                                       # 5次関数パラメータと、条件を満たす極値時刻を取得
        e = -(a * self.T ** 4 + b * self.T ** 3 + c * self.T ** 2 + d * self.T)                         # 5次関数パラメータeを計算
        
        y1 = poly(t1, a, b, c, d)                                                                       # t1における5次関数値y1を計算
        y2 = poly(t2, a, b, c, d)                                                                       # t2における5次関数値y2を計算

        u_pred_full = np.zeros_like(self.t_eval)                                                        # シミュレーション時間全体におけるFF入力全体を保存する配列
        u_pred_full[: len(t_ff)] = poly(t_ff, a, b, c, d)                                               # FF制御入力を格納

        return [a, b, c, d, e], (t1, y1, t2, y2), u_pred_full, y_tgt, u_opt                             # 5次関数のパラメータ、極値、完成したFF制御入力、目標モデル、最適入力を返す
    
    # 最終的な制御性能を評価する関数
    def compute_J(self, y_tgt, y_sys):                                                                  # 引数(目標モデルの応答, システムモデルの応答)
        """評価関数 J（目標軌道とシステムモデル出力の二乗誤差）"""
        return float(np.sum((y_tgt - y_sys) ** 2))                                                      # 評価値（二乗和誤差）を返す


# ==============================================================================
# ROS2 ノード
# ==============================================================================
class OptimalControlSequencer(Node):
    # ★ デバッグExcel出力の切り替え (True: 有効, False: 無効)
    DEBUG_EXCEL = True

    # コンストラクタ
    def __init__(self, csv_path, T, max_iter, target_mode):                                                                                     # 引数(保存csv情報, FF制御入力時間, 外側ループ最大回数, 目標値の与え方がランダムorプリセット)
        super().__init__('optimal_control_sequencer_el')                                                                                        # ROS2ノードとして登録
        self.csv_path = csv_path                                                                                                                # csv情報を格納
        self.T = T                                                                                                                              # FF制御入力時間を格納
        self.max_outer_iter = max_iter                                                                                                          # 外側ループ最大回数を格納
        self.target_mode = target_mode                                                                                                          # 目標値の与え方のモードを格納

        # ビューア用スナップショットの実行識別
        #   スナップショットは実行が終わってもファイルとして残るため、そのままだとビューアが
        #   前回の実行の残りを読んでしまう。実行ごとのIDを埋め込み、起動時に古いファイルを消す。
        self.session_id = time.strftime('%Y%m%d_%H%M%S')                                                                                        # この実行を識別するID
        if ENABLE_SNAPSHOT and os.path.exists(SNAPSHOT_PATH):                                                                                   # 前回の実行が残したスナップショットを削除する
            try:
                os.remove(SNAPSHOT_PATH)
            except OSError as exc:
                self.get_logger().warn(f"古いスナップショットを削除できません: {exc}")

        self.current_outer = 0                                                                                                                  # 外側ループ回数
        self.current_inner = 0                                                                                                                  # 内側ループ回数
        self.max_inner_iter = MAX_INNER_ITER                                                                                                    # 内側ループ回数
        self.threshold_J = THRESHOLD_J                                                                                                          # 内側ループの収束判定の閾値

        self.initial_stabilize_time = INIT_WAIT_TIME                                                                                            # 初期姿勢への移動後の待機時間
        self.data_collection_time = SIM_TIME                                                                                                    # データ取得時間
        self.dt = SIM_DT                                                                                                                        # シミュレーションステップ時間

        self.state = "INIT_ROBOT"                                                                                                               # ロボット状態（初期位置へ送る状態）

        # デバックExcelを行うときの処理
        if self.DEBUG_EXCEL:
            self.debug_wb = openpyxl.Workbook()                                                 # 新しいExcelファイルをメモリ上に作成
            self.default_sheet = self.debug_wb.active                                           # ExcelのSheet情報を取得
            self.excel_path = os.path.splitext(self.csv_path)[0] + "_debug_models.xlsx"         # csvの拡張子を除き、Excelのファイル名を作成

        # ポテンショメータ値の範囲 (26 elements)
        self.pot_bounds = [
            (450, 700), (135, 550), (500, 680), (250, 700), (66, 259), (192, 389),
            (70, 200), (60, 465), (115, 200), (100, 550), (239, 430), (205, 395),
            (30, 660), (30, 690), (110, 830), (3, 630), (3, 700), (9, 660),
            (275, 360), (115, 785), (192, 440), (284, 557),
            (323, 580), (188, 630), (375, 500), (300, 490),
        ]

        # 初期姿勢
        self.initial_pot = [
            500.0, 200.0, 500.0, 300.0, 170.0, 300.0,
            160.0, 410.0, 200.0, 500.0, 350.0, 220.0,
            300.0, 250.0, 400.0, 350.0, 420.0, 400.0,
            325.0, 370.0, 280.0, 420.0,
            360.0, 390.0, 420.0, 390.0,
        ]

        # ホームポジション
        self.initial_pot_fin = [
            500.0, 200.0, 500.0, 300.0, 170.0, 300.0,
            160.0, 410.0, 200.0, 500.0, 350.0, 220.0,
            300.0, 250.0, 400.0, 350.0, 420.0, 400.0,
            325.0, 370.0, 280.0, 420.0,
            360.0, 390.0, 420.0, 390.0,
        ]
        self.home_pot = list(self.initial_pot_fin)                                                                  # オリジナルの初期姿勢（非常停止・終了用安定位置）を保存

        # プリセットした目標値
        self.preset_targets = [
            [501, 201, 501, 700, 171, 301, 161, 411, 201, 501, 351, 221, 301, 251, 401, 351, 421, 401, 326, 371, 281, 300, 361, 391, 421, 300],
        ]

        self.current_ff_matrix = [list(MathematicalSolver.initial_ff_params(self.T)) for _ in range(24)]                                        # 24自由度分のFF係数（初回同定の励振用に非ゼロ初期値: f(0)=f(T)=0, f(t1)≥40, f(t2)≤-40）
        self.best_ff_matrix = None                                                                                                              # 今までの最良FF係数
        self.best_extrema = None                                                                                                                # 今までの最良FFの極値
        self.min_J_sum = float('inf')                                                                                                           # 評価関数Jの初期化
        self.best_debug_data = None                                                                                                             # デバック情報
        self.prev_u_pred_full = [None] * 24                                                                                                      # 前回ループで計算した5次関数FF入力（今回の実測データを生成した入力）
        self.prev_u_opt = [None] * 24                                                                                                            # 前回ループで計算した最適制御入力（今回の実測データを生成した入力）
        self.prev_tgt_params = [None] * 24                                                                                                       # 前回の目標モデル同定結果[T1, wn]（次回同定の初期値に使う）
        self.prev_sys_params = [None] * 24                                                                                                       # 前回のシステムモデル同定結果[a2, a1, a0, b0]（次回同定の初期値に使う）
        self.prev_fit_restart = [0] * 24                                                                                                         # 前回ループのCMA-ES再探索回数（ビューア用）
        self.param_history = []                                                                                                                  # 内側ループごとのパラメータ・評価値の履歴（ビューア用、外側ループごとにリセット）

        self.buffer_time_series = {f'board{i}': [] for i in range(1, 6)}                                                                        # 5board分のデータバッファ
        self.target_pot = self.get_next_target_positions()                                                                                      # 最初にロボットへ送る目標値を決定

        # Publisher作成
        self.pub_target = self.create_publisher(Float32MultiArray, '/board_android_float/sub', 10)                                              # 目標値パブリッシャーを作成
        self.pub_ff = {}                                                                                                                        # FFパラメータ係数を送信するための辞書を初期化
        for i in range(1, 6):                                                                                                                   # boardごとにFFパラメータ係数パブリッシャーを作成
            self.pub_ff[i] = self.create_publisher(Float32MultiArray, f'/board{i}_FFparam_float/sub', 10)

        # Subscriber作成
        self.create_subscription(UInt16MultiArray, '/board1_tk/pub', lambda m: self.cb_board(m, 1), 10)
        self.create_subscription(UInt16MultiArray, '/board2_tk/pub', lambda m: self.cb_board(m, 2), 10)
        self.create_subscription(UInt16MultiArray, '/board3_tk/pub', lambda m: self.cb_board(m, 3), 10)
        self.create_subscription(UInt16MultiArray, '/board4_tk/pub', lambda m: self.cb_board(m, 4), 10)
        self.create_subscription(UInt16MultiArray, '/board5_tk/pub', lambda m: self.cb_board(m, 5), 10)

        # タイマー作成
        self.control_timer = self.create_timer(0.1, self.sequencer_loop)

        self.state_start_time = self.get_clock().now()                                                                                          # 状態開始時刻の保存

    @staticmethod                                                                                                                               # Pythonデコレータ（静的メソッド：selfを使わない）
    def build_dof_map():
        """24自由度 → 26要素配列インデックス"""
        dof_map = []                                                                                                                            # 空リスト作成
        for i in range(1, 4):                                                                                                                   # board1, board2, board3用
            for j in range(6):
                dof_map.append((i - 1) * 6 + j)
        for i in [4, 5]:                                                                                                                        # board4, board5用
            for j in range(3):
                dof_map.append(18 + (i - 4) * 4 + j)
        return dof_map

    # 送信する目標値を決める関数
    def get_next_target_positions(self):
        if self.target_mode == "1" and self.current_outer < len(self.preset_targets):                                                           # 目標値送信モードが1で、外側ループ回数がプリセットの数より小さければ、プリセットした目標値を送る
            return self.preset_targets[self.current_outer]
        return [float(np.random.randint(b[0], b[1] + 1)) for b in self.pot_bounds]                                                              # それ以外はランダムにPOT範囲から送る

    # ROS2 Subscriberのコールバック関数
    def cb_board(self, msg, board_id):                                                                                                          # 引数(受信メッセージ, board番号)
        if self.state == "COLLECTING":                                                                                                          # 目標値受信状態のみ実行
            if board_id in [4, 5]:                                                                                                              # baord4と5は、3要素のみ受信してバッファに格納
                selected = [msg.data[idx] for idx in [0, 1, 2, 6, 7, 8]]
                self.buffer_time_series[f'board{board_id}'].append(selected)
            else:
                self.buffer_time_series[f'board{board_id}'].append(list(msg.data))                                                              # board1と2と3は、そのままバッファに格納
    
    # 目標値のPublish関数
    def publish_target_positions(self, pot_list):                                                                                               # 引数(送信する目標値)
        msg = Float32MultiArray()                                                                                                               # メッセージの型
        msg.data = [float(x) for x in pot_list]                                                                                                 # 目標値をメッセージに格納
        self.pub_target.publish(msg)                                                                                                            # メッセージ送信
        self.get_logger().info("=== 送信した目標値 (Target POT) ===")                                                                            # ログを出力
        for i in range(0, len(pot_list), 6):
            chunk = [f"{x:.1f}" for x in pot_list[i:i + 6]]
            self.get_logger().info(
                f"  [{i + 1:02d}-{min(i + 6, len(pot_list)):02d}]: " + " | ".join(chunk)
            )
        self.get_logger().info("===================================")

    # FFパラメータのPublish関数
    def publish_all_ff_parameters(self):
        dof_idx = 0                                                                                                                             # DOF番号（最初は0）
        solver = MathematicalSolver(self.T, self.dt)                                                                                            # 極値計算用のソルバー
        self.get_logger().info("=== 送信したFFパラメータ ===")
        for b_id in range(1, 6):                                                                                                                # board1からboard5の順に実行
            msg = Float32MultiArray()                                                                                            # メッセージの型
            data = []                                                                                                            # 空のリスト
            for _ in range(6):                                                                                                   # DOFループ
                if b_id in [4, 5] and _ >= 3:                                                                           # board4とboard5、かつ、DOF4以降       
                    data.extend([0.0, 0.0, 0.0, 0.0, 0.0, self.T])                                              # すべてのFFパラメータを0にする。
                else:                                                                                                   # board1とboard2とboard3とboar4・5の3DOFまで
                    a, b, c, d, e = self.current_ff_matrix[dof_idx]                                             # FFパラメータを取り出す
                    t1, y1, t2, y2 = solver.calc_extrema_from_ff([a, b, c, d, e])                               # 極値を計算
                    data.extend([float(a), float(b), float(c), float(d), float(e), float(self.T)])              # データに格納
                    self.get_logger().info(                                                                     # ログ出力
                        f"  B{b_id}-D{_ + 1} (DOF {dof_idx + 1:02d}): "
                        f"a={a:.6e}, b={b:.6e}, c={c:.6e}, d={d:.6e}, e={e:.6e} | "
                        f"t1={t1:.3f}, y1={y1:.2f}, t2={t2:.3f}, y2={y2:.2f}"
                    )
                    dof_idx += 1                                                                                # DOF番号更新
            msg.data = data                                                                                     # ROS2メッセージに格納
            self.pub_ff[b_id].publish(msg)                                                                      # メッセージ送信
        self.get_logger().info("============================")

    # ROS2のタイマーのコールバック関数
    def sequencer_loop(self):
        now = self.get_clock().now()                                                                                                            # 現在の時刻を取得
        elapsed = (now - self.state_start_time).nanoseconds / 1e9                                                                               # 経過時間を計算

        # 初期姿勢へ戻す状態
        if self.state == "INIT_ROBOT":
            self.get_logger().info(                                                                                     # ログ出力
                f"=== 最適制御 外側ループ {self.current_outer + 1} / {self.max_outer_iter} "
                f"(内側ループ {self.current_inner + 1}/{self.max_inner_iter}) ==="
            )
            self.publish_target_positions(self.initial_pot)                                                             # 初期姿勢を送信
            self.state = "WAIT_INITIAL_STABILIZE"                                                                       # 状態変更
            self.state_start_time = now                                                                                 # 現在時刻を取得

        # 初期姿勢への収束を待っている状態
        elif self.state == "WAIT_INITIAL_STABILIZE":
            if elapsed >= self.initial_stabilize_time:                                                                  # 待ち時間が終了したか判定
                self.publish_all_ff_parameters()                                                                # FFパラメータを送信
                time.sleep(0.1)                                                                                 # 0.1秒待機
                self.publish_target_positions(self.target_pot)                                                  # 目標値送信
                for k in self.buffer_time_series:                                                               # 各boardが持つデータバッファを初期化
                    self.buffer_time_series[k].clear()
                self.state = "COLLECTING"                                                                       # 状態変更
                self.state_start_time = now                                                                     # 現在時刻を取得

        # ロボットの実測データを収集している状態
        elif self.state == "COLLECTING":
            if elapsed >= self.data_collection_time:                                                                    # データ収集時間が終了したか判定
                self.state = "PROCESSING"                                                                       # 状態変更
                self.publish_target_positions(self.initial_pot)                                                 # 初期姿勢を送信
                threading.Thread(target=self.dispatch_optimization_pipeline, daemon=True).start()               # dispatch_optimization_pipeline関数を新しいスレッドに追加し実行する

        # プログラムの最終状態
        elif self.state == "FINISHED":
            # ロボットを安定化させるために初期位置をPublish
            self.get_logger().info("=== 最終安定化: home_pot を送信します ===")
            self.publish_target_positions(self.home_pot)                                                                # オリジナルの初期姿勢を送信
            if self.DEBUG_EXCEL:                                                                                        # Excel出力判定
                try:
                    if self.default_sheet in self.debug_wb.worksheets and len(self.debug_wb.worksheets) > 1:    # デフォルトシート(Sheet)がある、かつ、他にもシートがある 
                        self.debug_wb.remove(self.default_sheet)                                            # デフォルトシート削除                                         
                    self.debug_wb.save(self.excel_path)                                                         # Excel保存
                    self.get_logger().info(f"デバッグExcel保存: {self.excel_path}")                              # ログ出力
                except Exception as exc:
                    self.get_logger().error(f"Excel保存失敗: {exc}")                                             # エラーログ
            self.get_logger().info("すべての実験試行が正常終了しました。")
            self.control_timer.cancel()                                                                                 # タイマー停止
            raise SystemExit(0)                                                                                         # プログラム終了
    
    # 実測データから次回のFF入力を計算する関数（スレッド関数）
    def dispatch_optimization_pipeline(self):
        try:
            # 26要素と最適化する24自由度の対応表を作成
            data_24_dof = []                                                                                            # 24自由度分の空リスト
            dof_map = []                                                                                                # 26要素→24自由度への対応表
            for i in range(1, 4):                                                                                       # board1、board2、board3
                for j in range(6):
                    dof_map.append((i - 1) * 6 + j)                                                         # 0~17 DOF               
            for i in [4, 5]:                                                                                            # board4、board5
                for j in range(3):
                    dof_map.append(18 + (i - 4) * 4 + j)                                                    # 18, 19, 20, 22, 23, 24 DOF

            # Boardごとに保存されている実測データを24自由度の時系列データへ変換する
            for i in range(1, 4):                                                                                       # board1、board2、board3
                arr = np.array(self.buffer_time_series[f'board{i}'])                                        # 各boardに対応する実測データを取りだす
                for j in range(6):                                                                          # DOF1～6の順に処理
                    data_24_dof.append(arr[:, j] if arr.size > 0 else [])                               # データがあることを確認して、そのDOFのデータを格納
            for i in [4, 5]:                                                                                            # board4、board5
                arr = np.array(self.buffer_time_series[f'board{i}'])                                        # 各boardに対応する実測データを取りだす
                for j in range(3):                                                                          # DOF1～3の順に処理
                    data_24_dof.append(arr[:, j] if arr.size > 0 else [])                           # データがあることを確認して、そのDOFのデータを格納

            # 24自由度それぞれに対して、目標モデル同定・システムモデル同定・オイラーラグランジュ最適制御を実行する
            solver = MathematicalSolver(self.T, self.dt)                                                                # MathematicalSolverクラスの生成
            N_samples = int(SIM_TIME / self.dt)                                                                         # サンプル数の計算

            J_array = []                                                                                                # 24自由度それぞれの評価関数Jの空リスト
            debug_info = []                                                                                             # デバッグExcelへ保存する情報の空リスト
            extrema_list = []                                                                                           # 保存する極大値と極小値の空リスト
            used_extrema_list = []                                                                                      # 極値のリスト
            snap_dof = []                                                                                               # ビューア用データ（自由度ごと）の空リスト

            # 今回ロボットへ送信したFF（更新前）を保存
            ff_used_this_iteration = [row[:] for row in self.current_ff_matrix]                                         # パラメータの保存
            used_extrema_list =  [solver.calc_extrema_from_ff(ff) for ff in ff_used_this_iteration]                     # 極値の保存

            for dof_idx in range(24):                                                                                   # 24自由度分のループ
                raw_y = np.array(data_24_dof[dof_idx])[:N_samples]                                                      # 5秒間分の実測データを取りだす
                if len(raw_y) < N_samples:                                                                              # データ数が不足している場合
                    raw_y = np.pad(raw_y, (0, max(0, N_samples - len(raw_y))), mode='edge')                 # 測定できたデータの最後の値をコピーして、不足分だけ後ろに追加する

                # Initial and Target values
                Pi = self.initial_pot[dof_map[dof_idx]]                                                     # 現在のDOFの初期位置を取得
                Pf = self.target_pot[dof_map[dof_idx]]                                                      # 現在のDOFの目標位置を取得
                y0 = Pi - Pf                                                                                # 初期偏差を計算

                # Shift data so it converges to 0
                y_shifted = raw_y - Pf                                                                      # 目標位置を原点（0）に移動するための処理

                # ビューア表示用に「今回印加したFFを作るのに使った値」を、同定で上書きされる前に控えておく
                #   prev_u_opt が None のときは初回同定（最適制御前）なので、モデルパラメータもu_optも無い
                if self.prev_u_opt[dof_idx] is None:                                                         # 初回同定（印加したのは初期励振FF）
                    u_opt_used = np.full(N_samples, np.nan)                                                  # 表示なし（NaNはグラフに描かれない）
                    tgt_used = [np.nan, np.nan]                                                              # 表示なし
                    sys_used = [np.nan] * 4                                                                  # 表示なし
                    restart_used = -2                                                                        # 表示なしを示す値
                else:                                                                                        # 2回目以降（前回の最適制御結果を印加している）
                    u_opt_used = self.prev_u_opt[dof_idx]                                                    # 今回のFFの元になったu_opt
                    tgt_used = list(self.prev_tgt_params[dof_idx])                                           # そのu_optを計算するのに使った目標モデル
                    sys_used = list(self.prev_sys_params[dof_idx])                                           # そのu_optを計算するのに使ったシステムモデル
                    restart_used = self.prev_fit_restart[dof_idx]                                            # そのFFを作ったときのCMA-ES再探索回数

                # 1. Target Model ID
                tgt_params = solver.fit_target_model(y_shifted, y0, self.prev_tgt_params[dof_idx])          # 目標モデルの同定をして、パラメータを取得（前回の同定結果を初期値に使う）
                self.prev_tgt_params[dof_idx] = list(tgt_params)                                            # 次回同定の初期値として保存

                # 2. System Model ID
                a, b, c, d, e = self.current_ff_matrix[dof_idx]                                             # 現在のDOFのFFパラメータを取り出す
                t_ff = solver.t_eval[solver.t_eval <= self.T]                                               # 0秒～FF入力時間までの時間配列を取り出す
                u_ff = np.zeros(N_samples)                                                                  # 入力ベクトルの生成
                u_ff[:len(t_ff)] = a * t_ff ** 5 + b * t_ff ** 4 + c * t_ff ** 3 + d * t_ff ** 2 + e * t_ff # FF入力時間だけ、そのときのFF入力を格納する

                sys_params = solver.fit_system_model(y_shifted, u_ff, y0, self.prev_sys_params[dof_idx])    # システムモデルの同定をして、パラメータを取得（前回の同定結果を初期値に使う）
                self.prev_sys_params[dof_idx] = list(sys_params)                                            # 次回同定の初期値として保存

                # 3. Calculate squared error J
                a2_sys, a1_sys, a0_sys, b0_sys = sys_params                                                 # システムモデル係数を取り出す
                y_sys_sim, _, _ = solver.simulate_forced(solver.t_eval, a2_sys, a1_sys, a0_sys, b0_sys, u_ff, y0)   # 同定したシステムモデルの応答を取り出す

                # 4. 離散時間オイラー・ラグランジュ最適制御入力計算 + 5次多項式フィット
                new_ff, extrema, u_pred_full, y_tgt, u_opt = solver.calculate_el_ff(tgt_params, sys_params, u_ff, y0)     # 最適入力を計算して、その結果の、FFパラメータ、極値、5次関数のFF制御入力、目標モデルの応答、最適入力を格納
                self.current_ff_matrix[dof_idx] = new_ff                                                    # FFパラメータの更新
                extrema_list.append(extrema)                                                                # 極値を保存

                # 内側ループの判定用評価関数J（実測データと目標モデルとの差）
                J = np.sum((y_tgt - y_shifted) ** 2)                                                        # 二乗和誤差を計算
                J_array.append(J)                                                                           # 24自由度それぞれの評価関数Jの空リストに追加

                # 今回の実測データ（＝今回のJ）を生成した入力は、前回ループで計算した u_pred_full / u_opt
                # 初回ループは前回値が無いため、実際に印加したFF入力(u_ff)で代用する
                u_pred_full_for_J = self.prev_u_pred_full[dof_idx] if self.prev_u_pred_full[dof_idx] is not None else u_ff.copy()   # 今回のJを生成した5次関数FF入力
                u_opt_for_J = self.prev_u_opt[dof_idx] if self.prev_u_opt[dof_idx] is not None else u_ff.copy()                     # 今回のJを生成した最適制御入力

                # デバック用データを保存
                debug_info.append({
                    't': solver.t_eval,             # 実測時間
                    'y_data': raw_y,                # 実測POT値
                    'y_sys': y_sys_sim + Pf,        # システムモデル応答
                    'y_tgt': y_tgt + Pf,            # 目標モデル応答
                    'J': J,
                    'u_pred_full': u_pred_full_for_J,   # 今回のJを生成した5次関数FF入力（最小J時の値）
                    'u_opt': u_opt_for_J,               # 今回のJを生成した最適制御入力（最小J時の値）
                })

                # ビューア用データを保存
                #   波形と極値・モデルパラメータは「この内側ループで実際に使った値」に揃える。
                #   ・応答(Time-POT) は今回の実測と今回同定したモデル
                #   ・入力(Time-PWM) は今回印加したFFと、その元になった前回のu_opt
                #   ・モデルパラメータ履歴は、そのu_optを計算するのに使った前回の同定値
                if ENABLE_SNAPSHOT:
                    snap_dof.append({
                        'y_data': raw_y,                                                    # 実測POT値
                        'y_sys': y_sys_sim + Pf,                                            # 今回同定したシステムモデルの応答
                        'y_tgt': y_tgt + Pf,                                                # 今回同定した目標モデルの応答
                        'u_opt_used': u_opt_used,                                           # 今回印加したFFの元になったu_opt（初回同定時はNaN）
                        'u_ff_applied': u_ff,                                               # 今回ロボットへ送信したFF入力（この実測データを生成した入力）
                        'tgt_params_id': list(tgt_params),                                  # 今回同定した目標モデル[T1, wn]（Time-POTの破線に対応）
                        'sys_params_id': list(sys_params),                                  # 今回同定したシステムモデル[a2, a1, a0, b0]（Time-POTの点線に対応）
                        'tgt_params_used': tgt_used,                                        # 今回のFFを作るのに使った目標モデル（初回同定時はNaN）
                        'sys_params_used': sys_used,                                        # 今回のFFを作るのに使ったシステムモデル（初回同定時はNaN）
                        'ff': list(ff_used_this_iteration[dof_idx]),                         # 今回印加したFFパラメータ[a, b, c, d, e]
                        'extrema': list(used_extrema_list[dof_idx]),                         # 今回印加したFFの極値[t1, y1, t2, y2]
                        'J': float(J),                                                      # このDOFの評価関数値（実測と目標モデルの差）
                        'Pi': float(Pi),                                                    # 初期位置
                        'Pf': float(Pf),                                                    # 目標位置
                        'pot_idx': dof_map[dof_idx],                                        # 26要素配列でのインデックス
                        'fit_res': float(np.sum((u_ff - u_opt_used) ** 2)),                  # 印加したFFの近似残差 Σ(FF-u_opt)²（初回同定時はNaN）
                        'fit_restart': restart_used,                                        # そのFFを作ったときのCMA-ES再探索回数（-1は退避, -2は該当なし）
                        'id_res_sys': float(np.sum((y_sys_sim - y_shifted) ** 2)),           # 今回のシステムモデル同定の残差
                    })

                # 次回ループで印加する（＝次回の実測データを生成する）入力を保存
                self.prev_u_pred_full[dof_idx] = u_pred_full                                                 # 次回ループ用の5次関数FF入力を保存
                self.prev_u_opt[dof_idx] = u_opt                                                             # 次回ループ用の最適制御入力を保存
                self.prev_fit_restart[dof_idx] = solver.last_fit_restart                                     # 次回ループ用のCMA-ES再探索回数を保存

            total_J = sum(J_array)                                                                                      # 各自由度の評価関数値Jを足す

            # 評価関数値の値の変化確認
            if self.min_J_sum == float('inf'):                                                                          # 初回の場合
                diff_msg = "(初回)"
            else:                                                                                                       # 2回目以降
                diff = total_J - self.min_J_sum                                                             # これまでのbestJと比較
                if diff < 0:
                    diff_msg = f"(これまでのベストより {-diff:.2f} 改善！)"
                else:
                    diff_msg = f"(これまでのベストより {diff:.2f} 悪化)"

            self.get_logger().info(                                                                                     # ログ出力
                f"内側ループ {self.current_inner + 1} 完了: 今回のJ = {total_J:.2f}, "
                f"これまでのベストJ = {self.min_J_sum if self.min_J_sum != float('inf') else total_J:.2f} {diff_msg}"
            )

            # ベストかどうか判定
            if total_J < self.min_J_sum:
                self.min_J_sum = total_J                                                                    # ベストJ更新
                self.best_ff_matrix = [row[:] for row in ff_used_this_iteration]                            # ベストFFパラメータを格納
                self.best_extrema = used_extrema_list[:]                                                    # ベストJのときの極値を保存
                self.best_debug_data = debug_info                                                           # ベストJのときのデバック情報を保存

            self.current_inner += 1                                                                                     # 内側ループ回数更新

            if ENABLE_SNAPSHOT:                                                                                         # ビューア用スナップショットを書き出す
                self.save_snapshot(snap_dof, total_J)

            # 内側ループ終了判定
            if total_J < self.threshold_J or self.current_inner >= self.max_inner_iter:                                             # 評価関数Jが閾値より小さくなったか、or、内側ループの最大回数を超えたか
                self.get_logger().info(                                                                                 # ログ出力
                    f"外側ループ {self.current_outer + 1} 完了！最高結果をCSV/Excelに保存します。"
                )
                self.save_optimal_results_to_csv()                                                                      # ベストFF入力の情報をCSVへ保存
                if self.DEBUG_EXCEL:                                                                                    # Excelにも保存する場合は保存
                    self.save_debug_to_excel()

                # 最良パラメータ表示
                self._print_best_params(dof_map)                                                                        # 最良パラメータを表示

                self.current_outer += 1                                                                                 # 外側ループ回数を更新
                self.current_inner = 0                                                                                  # 内側ループ回数を0に戻す
                self.min_J_sum = float('inf')                                                                           # 内側ループの評価関数値の初期値を無限に変える

                # 外側ループを終えた後の判定
                if self.current_outer >= self.max_outer_iter:
                    self.state = "FINISHED"                                                                             # 終了状態に変更
                    self.state_start_time = self.get_clock().now()                                                      # 現在時刻を取得
                    return                                                                                              # dispatch_optimization_pipeline()を終了

                # Next outer loop target
                self.initial_pot = list(self.target_pot)                                                                # 1つ前の目標値を次の初期姿勢にする
                self.target_pot = self.get_next_target_positions()                                                      # 次の目標値を決める
                # reset ff for new target
                self.current_ff_matrix = [list(MathematicalSolver.initial_ff_params(self.T)) for _ in range(24)]        # FFパラメータを初回励振用の非ゼロ初期値へリセット
                self.prev_u_pred_full = [None] * 24                                                                     # FFリセットに伴い前回入力もリセット（初回はu_ffで代用）
                self.prev_u_opt = [None] * 24                                                                           # FFリセットに伴い前回最適入力もリセット（初回はu_ffで代用）
                self.param_history = []                                                                                 # ビューアの横軸を新しい外側ループの内側ループ番号に戻す

            self.state = "INIT_ROBOT"                                                                                   # 初期状態に戻す
            self.state_start_time = self.get_clock().now()                                                              # 現在時刻を取得

        except Exception as exc:                                                                                                                    # 何かエラーが出たときの処理
            self.get_logger().error(f"最適化パイプライン異常: {exc}\n{traceback.format_exc()}")
            self.state = "FINISHED"
            self.state_start_time = self.get_clock().now()

    # ビューア用スナップショットを書き出す関数
    def save_snapshot(self, snap_dof, total_J):                                                                                                 # 引数(自由度ごとのビューア用データ, 今回の評価関数値の合計)
        """最新の波形と、内側ループごとのパラメータ履歴を1つの .npz にまとめて保存する。

        別プロセスのビューア（view_optimization.py）がこのファイルの更新時刻を監視し、
        更新されていれば読み直して再描画する。一時ファイルへ書いてから os.replace で
        アトミックに差し替えるため、ビューアが書き込み途中のファイルを掴むことはない。
        保存に失敗しても最適化は続行する（ビューアは実験の必須要素ではないため）。
        """
        if not snap_dof:                                                                                                # データが無ければ何もしない
            return

        # 自由度方向に積んだ配列を作る関数（24行の2次元配列になる）
        def stack(key):
            return np.array([np.asarray(d[key], dtype=np.float32) for d in snap_dof], dtype=np.float32)

        # 内側ループごとのスカラー値を履歴へ追加する（横軸 Inner loop = 0 が初回同定）
        self.param_history.append({
            'tgt': stack('tgt_params_used'), 'sys': stack('sys_params_used'),                                           # そのループのu_optを計算するのに使ったモデル
            'ext': stack('extrema'),                                                                                    # そのループで印加したFFの極値
            'J': stack('J'),                                                                                            # DOF別評価関数値（実測と目標モデルの差）
            'total_J': float(total_J), 'best_J': float(self.min_J_sum),                                                 # 全DOF合計J・ベストJ
        })
        hist = self.param_history                                                                                       # 履歴の参照

        snap = {
            # --- 現在の状態 ---
            'session': self.session_id,                                                                                 # この実行の識別ID（ビューアが前回の実行と区別するために使う）
            'outer': self.current_outer + 1, 'max_outer': self.max_outer_iter,                                          # 外側ループ番号
            'inner': self.current_inner, 'max_inner': self.max_inner_iter,                                              # 内側ループ番号
            'total_J': float(total_J), 'best_J': float(self.min_J_sum),                                                 # 今回のJ・ベストJ
            'T': float(self.T), 'time': time.strftime('%Y-%m-%d %H:%M:%S'),                                             # FF制御時間・更新時刻
            # --- 最新の波形（24自由度 × 時系列） ---
            't': np.arange(0, SIM_TIME, self.dt, dtype=np.float32),                                                     # 時間軸
            'y_data': stack('y_data'), 'y_sys': stack('y_sys'), 'y_tgt': stack('y_tgt'),                                # 実測・システムモデル・目標モデル
            'u_opt_used': stack('u_opt_used'), 'u_ff_applied': stack('u_ff_applied'),                                   # 使用したu_opt・印加したFF
            # --- 最新のパラメータ（24自由度分） ---
            'tgt_params_id': stack('tgt_params_id'), 'sys_params_id': stack('sys_params_id'),                           # 今回同定したモデル
            'tgt_params_used': stack('tgt_params_used'), 'sys_params_used': stack('sys_params_used'),                   # 今回のFFを作るのに使ったモデル
            'ff': stack('ff'), 'extrema': stack('extrema'),                                                             # 印加したFFのパラメータ・極値
            'J': stack('J'), 'Pi': stack('Pi'), 'Pf': stack('Pf'),                                                      # 評価関数値・初期位置・目標位置
            'pot_idx': np.array([d['pot_idx'] for d in snap_dof], dtype=np.int32),                                      # 26要素配列でのインデックス
            'fit_res': stack('fit_res'), 'fit_restart': np.array([d['fit_restart'] for d in snap_dof], dtype=np.int32),  # 近似残差・CMA-ES再探索回数
            'id_res_sys': stack('id_res_sys'),                                                                          # システムモデル同定の残差
            # --- 履歴（内側ループ × 24自由度） ---
            'hist_tgt': np.array([r['tgt'] for r in hist]), 'hist_sys': np.array([r['sys'] for r in hist]),             # 使用した目標モデル・システムモデルの推移
            'hist_ext': np.array([r['ext'] for r in hist]),                                                             # 印加したFFの極値の推移
            'hist_J': np.array([r['J'] for r in hist]),                                                                 # 評価関数値の推移
            'hist_total_J': np.array([r['total_J'] for r in hist], dtype=np.float32),                                   # 全DOF合計Jの推移
            'hist_best_J': np.array([r['best_J'] for r in hist], dtype=np.float32),                                     # ベストJの推移
        }

        try:
            tmp_path = SNAPSHOT_PATH + ".tmp"                                                                           # 一時ファイル名
            with open(tmp_path, 'wb') as fp:                                                                            # 拡張子を勝手に付けられないようファイルオブジェクトで渡す
                np.savez(fp, **snap)
            os.replace(tmp_path, SNAPSHOT_PATH)                                                                         # アトミックに差し替える
        except Exception as exc:
            self.get_logger().warn(f"スナップショット保存失敗: {exc}")                                                    # 失敗しても最適化は続行する

    # 外側ループが終了した時点で得られた最良のFFパラメータを一覧表示する関数
    def _print_best_params(self, dof_map):
        """外側ループ完了時に最良パラメータa,b,c,d,eとt1,t2,y1,y2を表示する"""
        self.get_logger().info("")
        self.get_logger().info("=" * 90)
        self.get_logger().info(
            f"  外側ループ {self.current_outer + 1} 最良結果 (ベストJ = {self.min_J_sum:.2f})"
        )
        self.get_logger().info("=" * 90)

        board_names = ["Board1", "Board2", "Board3", "Board4", "Board5"]
        dof_idx = 0                                                                                                                                     # 24自由度全体を数える番号
        for b_id in range(1, 6):                                                                                                                        # board1～board5の順に処理
            # 表の見出しを作成
            n_dof = 6 if b_id <= 3 else 3
            self.get_logger().info(f"")
            self.get_logger().info(f"--- {board_names[b_id - 1]} ({n_dof}DOF) ---")
            self.get_logger().info(
                f"  {'DOF':>4s} | {'a':>12s}  {'b':>12s}  {'c':>12s}  {'d':>12s}  {'e':>12s}"
                f"  | {'t1':>6s}  {'y1':>8s}  {'t2':>6s}  {'y2':>8s}"
            )
            self.get_logger().info("  " + "-" * 106)

            # 表に値をいれて表示
            for local_d in range(n_dof):
                a, b, c, d, e = self.best_ff_matrix[dof_idx]                                                            # パラメータを取得
                t1, y1, t2, y2 = self.best_extrema[dof_idx]                                                             # 極値取得
                self.get_logger().info(
                    f"  {dof_idx + 1:4d} | {a:>12.4e}  {b:>12.4e}  {c:>12.4e}  {d:>12.4e}  {e:>12.4e}"
                    f"  | {t1:6.3f}  {y1:8.2f}  {t2:6.3f}  {y2:8.2f}"
                )
                dof_idx += 1                                                                                            # 自由度番号の更新

        self.get_logger().info("=" * 90)
        self.get_logger().info("")
    
    # 外側ループで最終的に得られた最良結果をCSVへ保存する関数
    def save_optimal_results_to_csv(self):
        dof_map = self.build_dof_map()                                                                                                                  # 26要素→24自由度対応表の作成
        row_data = {}                                                                                                                              # 空の辞書作成
        for dof_idx in range(24):                                                                                                                  # 24自由度ループ
            Pi = self.initial_pot[dof_map[dof_idx]]                                                                                                # 初期位置取得
            Pf = self.target_pot[dof_map[dof_idx]]                                                                                                 # 目標位置取得
            t1, y1, t2, y2 = self.best_extrema[dof_idx]                                                                                            # 極値取得
            
            # csvへ保存するデータを追加
            row_data[f'Init_{dof_idx + 1}'] = Pi
            row_data[f'Target_{dof_idx + 1}'] = Pf
            row_data[f'T_{dof_idx + 1}'] = self.T
            row_data[f't1_{dof_idx + 1}'] = t1
            row_data[f'y1_{dof_idx + 1}'] = y1
            row_data[f't2_{dof_idx + 1}'] = t2
            row_data[f'y2_{dof_idx + 1}'] = y2

        df = pd.DataFrame([row_data])                                                                                                                   # データを横並びにする
        header = not os.path.exists(self.csv_path)                                                                                                      # 既存ファイルがあるか判定
        df.to_csv(self.csv_path, mode='a', header=header, index=False)                                                                                  # csvへ保存
        self.get_logger().info(f"CSV保存完了: {self.csv_path}")

    # デバック用Excelを作成する関数
    def save_debug_to_excel(self):
        sheet_name = f"Iter_{self.current_outer + 1}"                                                                                                   # シート名作成
        ws = self.debug_wb.create_sheet(title=sheet_name)                                                                                               # シート作成

        ws.cell(row=1, column=1, value="DOF")                                                                                                           # 1列目のタイトル作成
        ws.cell(row=1, column=2, value="Best J")                                                                                                        # 2列目のタイトル作成
        for dof_idx in range(24):                                                                                                                       # 24自由度ループ
            ws.cell(row=dof_idx + 2, column=1, value=dof_idx + 1)                                                                           # A列へDOF番号を格納
            ws.cell(row=dof_idx + 2, column=2, value=self.best_debug_data[dof_idx]['J'])                                                    # B列へ評価関数Jを格納
        
        # 各DOFのグラフを作成し、それをExcelへ貼り付ける処理
        for dof_idx in range(24):                                                                                                                       # 24自由度ループ
            data = self.best_debug_data[dof_idx]                                                                                            # デバック用データを取りだす
            plt.figure(figsize=(6, 4))                                                                                                      # 新しいグラフを作成
            plt.plot(data['t'], data['y_data'], label='Actual Data')                                                                        # 実測データを描画
            plt.plot(data['t'], data['y_tgt'], '--', label='Target Model (zeta=1)')                                                         # 目標モデルを描画
            plt.plot(data['t'], data['y_sys'], ':', label='System Model')                                                                   # システムモデルを描画
            plt.title(f"DOF {dof_idx + 1} (J = {data['J']:.2f})")                                                                           # グラフのタイトルを設定
            plt.legend()                                                                                                                    # 凡例を表示
            img_path = f"/tmp/el_plot_iter{self.current_outer + 1}_dof{dof_idx + 1}.png"                                                    # 画像ファイル名を作成
            plt.savefig(img_path)                                                                                                           # PNG画像として保存
            plt.close()                                                                                                                     # グラフを閉じる
            img = OpenpyxlImage(img_path)                                                                                                   # PNG画像をOpenPyXLが扱える画像オブジェクトへ変換
            col = "D" if dof_idx % 2 == 0 else "M"                                                                                          # 偶数自由度は左に、奇数自由度は右に配置
            row_idx = 2 + (dof_idx // 2) * 22                                                                                               # 張り付ける高さを指定
            ws.add_image(img, f"{col}{row_idx}")                                                                                            # 画像を張り付ける

        # 新しく入力比較用のシートを追加
        input_sheet_name = f"Input_Iter_{self.current_outer + 1}"
        ws_input = self.debug_wb.create_sheet(title=input_sheet_name)
        headers = ["DOF", "a", "b", "c", "d", "e", "t1", "y1", "t2", "y2"]
        for col_idx, h in enumerate(headers, start=1):
            ws_input.cell(row=1, column=col_idx, value=h)
            
        for dof_idx in range(24):
            a, b, c, d, e = self.best_ff_matrix[dof_idx]
            t1, y1, t2, y2 = self.best_extrema[dof_idx]
            row_data = [dof_idx + 1, a, b, c, d, e, t1, y1, t2, y2]
            for col_idx, val in enumerate(row_data, start=1):
                ws_input.cell(row=dof_idx + 2, column=col_idx, value=val)
        
        # 入力比較用グラフの作成と貼り付け
        for dof_idx in range(24):
            data = self.best_debug_data[dof_idx]
            plt.figure(figsize=(6, 4))
            plt.plot(data['t'], data['u_opt'], label='Optimal Control (u_opt)')
            plt.plot(data['t'], data['u_pred_full'], '--', label='5th-order FF (u_pred_full)')
            plt.title(f"Input Comparison DOF {dof_idx + 1}")
            plt.xlabel("Time [s]")
            plt.ylabel("Input value")
            plt.legend()
            img_path = f"/tmp/input_plot_iter{self.current_outer + 1}_dof{dof_idx + 1}.png"
            plt.savefig(img_path)
            plt.close()
            
            img = OpenpyxlImage(img_path)
            col = "D" if dof_idx % 2 == 0 else "M"
            row_idx = 2 + (dof_idx // 2) * 22
            ws_input.add_image(img, f"{col}{row_idx}")


# ==============================================================================
# エントリーポイント
# ==============================================================================

# CSVファイル選択関数
def resolve_csv_file():
    root = tk.Tk()                                                                                                                          # Tkinterを起動
    root.withdraw()                                                                                                                         # 親ウィンドウは表示しない
    root.attributes("-topmost", True)                                                                                                       # ダイアログを最前面に表示

    print("====================================================")
    print("【最適制御実施前手順 1】結果記録用CSVの選択および生成")
    print("1: 既存の結果CSVファイルを選択して追記する")
    print("2: 新規保存先フォルダを選択してCSVファイルを生成する")
    print("====================================================")
    choice = input("モードを選択してください (1 または 2): ").strip()                                                                           # モードの入力

    # 追記の場合
    if choice == '1':
        file_path = filedialog.askopenfilename(                                                                             # ファイル選択ダイアログを表示
            title="既存の結果CSVファイルを選択してください",
            filetypes=[("CSV Files", "*.csv")],
        )
        if not file_path:
            print("ファイル未選択のため終了します。")
            sys.exit(1)
        return file_path

    # 新規作成の場合
    folder_path = filedialog.askdirectory(title="結果のCSVファイルを保存するフォルダを選択してください")                                            # フォルダ選択ダイアログを表示
    if not folder_path:
        print("フォルダ未選択のため終了します。")
        sys.exit(1)
    file_name = input("新規作成するCSVファイル名を入力してください (例: result.csv): ").strip()                                                     # ファイル名の入力
    if not file_name.endswith(".csv"):
        file_name += ".csv"
    return os.path.join(folder_path, file_name)


def main(args=None):
    csv_path = resolve_csv_file()
    print(f"【保存先CSVパス】 {csv_path}")

    print("【FF制御時間 T を入力してください】")
    T = float(input("> "))
    print("【最適制御実行回数を入力してください】")
    max_iter = int(input("> "))
    print("【モードを選択してください (1: プリセット後ランダム, 2: 最初からランダム)】")
    mode = input("> ").strip()
    while mode not in ['1', '2']:
        mode = input("無効な入力です。1 または 2 を入力してください: ").strip()

    rclpy.init(args=args)
    node = OptimalControlSequencer(csv_path, T, max_iter, mode)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
