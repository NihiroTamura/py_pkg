#!/usr/bin/env python3
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

# COST_Q = np.diag([724837, 13949.9, 0.184584])
# COST_R = np.array([[0.0526736]])

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
# 最適入力の5次多項式フィットのパラメータ（チューニング要素）
#   f(t) = a t^5 + b t^4 + c t^3 + d t^2 - (a T_FF^4 + b T_FF^3 + c T_FF^2 + d T_FF) t
#   ・1次の係数を e = -(a T_FF^4 + b T_FF^3 + c T_FF^2 + d T_FF) に固定しているため
#     f(0) = f(T_FF) = 0 が常に厳密に成立する
#   ・極値は「探してから数える」のではなく、f'(t) を
#         f'(t) = (t-t1)(t-t2)·q(t),  q(t) = α(t-m)² + β （α,βは同符号 → q は符号一定）
#     と因数分解した形で表すことで、0<t<T_FF に山1つ・谷1つだけを構造的に保証する
#   ・α=0 のとき f は3次関数 C·t(t-tm)(t-T_FF) に退化する（理想形）
#   ・その条件を満たす形の中で、u_opt との二乗和誤差が最小のものを採用する
# ==============================================================================
FIT_EXTREMA_WEIGHT = 1e12   # q(t)が符号一定にならない（極値が増える）ときのペナルティ重み（大きな重み）
FIT_PWM_WEIGHT = 1e6        # PWM上下限(±255)を超えたときのペナルティ重み
FIT_QUAD_MARGIN = 0.05      # |q(t)|が区間内で最大値のこの割合を下回らないようにする余裕（平坦部＝準重根を防ぐ）
FIT_EXTREMA_MARGIN = 0.02   # 極値を端点(0, T_FF)から離しておく余裕（T_FFに対する割合。端点に張り付いた極値を防ぐ）
FIT_EXTREMA_GAP = 0.05      # 山と谷を離しておく最小間隔（T_FFに対する割合。2つの極値が重なるのを防ぐ）
FIT_MAX_RETRY = 100          # 極値2個の判定を満たさなかった場合のフィット再試行回数
FIT_MIN_TRIAL = 5           # 条件を満たす解が得られても最低限試す初期値の個数（初期値依存の悪い解の採用を防ぐ）
FIT_ROOT_TOL = 1e-6         # f'(t)=0 の実根判定の許容誤差（虚部の大きさおよび重根の同一視）

# ==============================================================================
# 5次多項式フィットの極値アンカー（トラストリージョン）のパラメータ（チューニング要素）
#   u_opt はインパルス状になりやすく、f(0)=f(T_FF)=0 を課した5次多項式では表現しきれない。
#   その結果フィットの残差曲面が平坦になり、振幅・極値位置がほぼ不定となって
#   内側ループごとに極値が大きく飛ぶ（＝実機に入る入力が毎回別物になる）。
#   そこで「そのDOFで最小のJを出したFFの極値」をアンカーとし、
#     ・fit_loss に正規化した近接項 w_prox·prox を加えて最適化そのものを引き寄せる
#     ・アンカーの極値から作った形状パラメータを初期値の1つに加える（ウォームスタート）
#     ・フィット精度が実質同等な候補（mse ≤ (1+ε)·mse_min）の中から最も近い解を採用する
#     ・改善したら重みを半分（半径拡大）、悪化したら倍（半径縮小）に更新する
#   ことで、極値の連続性とフィット精度を両立させる。
# ==============================================================================
FIT_PROX_W0 = 1.0           # 近接項の重みの初期値（アンカーを効かせ始めるときの探索半径）
FIT_PROX_W_MIN = 1e-3       # 近接項の重みの下限（改善が続いたときに許す最大の探索半径）
FIT_PROX_W_MAX = 1e3        # 近接項の重みの上限（悪化が続いたときの最小の探索半径）
FIT_PROX_WARMUP = 3         # アンカーを効かせ始めるまでの内側ループ回数（初期FFの振幅に縛られるのを防ぐ）
FIT_MSE_TOL = 0.10          # フィット精度の許容劣化率 ε（mse ≤ (1+ε)·mse_min の候補だけをアンカー選択の対象にする）
FIT_PWM_SCALE = 255.0       # 極値の大きさ [PWM] の正規化スケール（時間 t は T_FF で正規化する）


# ==============================================================================
# 5次多項式フィットの極値アンカー（DOFごとのトラストリージョン）
# ==============================================================================
class ExtremaAnchor:
    """1自由度分の「最小Jを出したFFの極値」を保持し、フィットの探索半径を適応させるクラス。

    合計Jではなく**DOFごとのJ**でベストを持つ（1軸が支配的なとき、他の軸が
    「たまたま合計が良かった回」の極値にアンカーされるのを防ぐ）。

    ・pending_extrema : 今回ロボットへ印加したFFの極値。そのFFが生んだJが確定した時点で
                        改善していれば best_extrema へ昇格する（J とFFの対応をずらさないため）
    ・w_prox          : 近接項の重み。改善で半分（半径拡大）、悪化で倍（半径縮小）。
                        古典的なトラストリージョンと同じ更新で「良くなっている間は自由に動き、
                        悪化し始めたらベストへ引き戻される」挙動になる
    ・n_update        : update() の呼び出し回数。FIT_PROX_WARMUP 未満はアンカーを無効にする
    """

    # コンストラクタ
    def __init__(self):
        self.best_J = float('inf')          # このDOFのこれまでの最小J
        self.best_extrema = None            # 最小Jを出したFFの極値 (t1, y1, t2, y2)
        self.pending_extrema = None         # 今回印加したFFの極値（Jが確定したら昇格させる候補）
        self.w_prox = FIT_PROX_W0           # 近接項の重み（トラストリージョン半径に相当）
        self.n_update = 0                   # update() の呼び出し回数（＝Jが確定した回数）

    # 今回のJでベスト極値と探索半径を更新する関数
    def update(self, J):                                                            # 引数(今回印加したFFが生んだ評価関数J)
        improved = J < self.best_J                                                  # このDOFのベストを更新したか
        if improved:                                                                # 改善していれば
            self.best_J = J                                                         # ベストJを更新
            if self.pending_extrema is not None:                                    # 今回印加したFFの極値を
                self.best_extrema = tuple(self.pending_extrema)                     # ベスト極値へ昇格
        if self.n_update >= FIT_PROX_WARMUP:                                        # アンカーが有効な区間だけ半径を更新する
            self.w_prox = (max(self.w_prox * 0.5, FIT_PROX_W_MIN) if improved       # 改善 → 重みを半分にして探索半径を広げる
                           else min(self.w_prox * 2.0, FIT_PROX_W_MAX))             # 悪化 → 重みを倍にしてベスト極値へ引き戻す
        self.n_update += 1                                                          # 呼び出し回数を更新

    # アンカーとして使う極値を返す関数（ウォームアップ中・ベスト未確定なら None）
    def target(self):
        if self.n_update < FIT_PROX_WARMUP:                                         # 初回数ループはアンカーを効かせない
            return None
        return self.best_extrema                                                    # ベスト極値（未確定なら None）

    # 近接項の重みを返す関数（アンカーが無効なら0 → 従来どおりの純粋なフィットになる）
    def weight(self):
        return 0.0 if self.target() is None else self.w_prox


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

    # 目標モデル同定関数
    def fit_target_model(self, y_data, y0):                                                 # 引数(実測データ, 初期偏差)
        """目標モデル同定: 1 / ((T1*s + 1)(s^2 + 2*wn*s + wn^2))（減衰係数 zeta=1 固定）

        勾配ベースの信頼領域反射法（scipy.optimize.least_squares, method='trf'）で
        各時刻の残差 r(k)=y_sim(k)-y_data(k) の二乗和 Σr² を最小化する（Nelder-Mead は使用しない）。
        """
        # 各時刻の残差ベクトルを返す関数（least_squares は Σr² を最小化する）
        def residuals(p):
            T1, wn = p                                                              # 最適化変数の取り出し
            a2, a1, a0 = self._target_coeffs(T1, wn)                                # 減衰係数1固定の3次遅れ系の係数を計算
            y_sim, _, _, _ = self.simulate_unforced(self.t_eval, a2, a1, a0, y0)    # 目標モデルの自由応答を計算
            return y_sim - y_data                                                   # 残差ベクトルを返す

        res = scipy.optimize.least_squares(                                                 # Σr² が最小になる変数[T1, wn]を最適化する
            residuals, x0=[0.1, 10.0],
            bounds=([1e-6, 1e-6], [np.inf, np.inf]),                                        # T1>0, wn>0（負の極を排除）
            method='trf',
        )
        T1, wn = res.x                                                                      # 最適変数を取り出す
        return T1, wn

    # システムモデル同定関数
    def fit_system_model(self, y_data, u_ff, y0):                                                   # 引数(実測データ, FF制御入力, 初期偏差)
        """システムモデル同定: b0 / (s^3 + a2*s^2 + a1*s + a0)

        目標モデルと同じく勾配ベースの信頼領域反射法（least_squares, method='trf'）で
        残差二乗和を最小化する。b0は励振（非ゼロFF入力）があってはじめて同定できるため、
        初回同定では INIT_FF_PEAK 振幅の初期FFで励振する。
        """
        # 各時刻の残差ベクトルを返す関数
        def residuals(p):
            a2, a1, a0, b0 = p                                                          # 最適化変数の取り出し
            y_sim, _, _ = self.simulate_forced(self.t_eval, a2, a1, a0, b0, u_ff, y0)   # システムモデルの応答を計算
            return y_sim - y_data                                                       # 残差ベクトルを返す

        res = scipy.optimize.least_squares(                                                         # Σr² が最小になる変数[a2, a1, a0, b0]を最適化する
            residuals, x0=[10.0, 100.0, 1000.0, 1000.0],
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

    # ------------------------------------------------------------------
    # 離散時間オイラー・ラグランジュ（随伴／勾配）法による最適制御 + 5次多項式フィット
    # ------------------------------------------------------------------
    def calculate_el_ff(self, target_params, sys_params, u_ff, y0, y_meas=None, anchor=None):                # 引数(目標モデルのパラメータ[T1, wn], システムモデルのパラメータ[a2, a1, a0, b0], FF制御入力, 初期偏差, 今回の実測データ, 極値アンカー)
        """
        離散時間オイラー・ラグランジュ（随伴／勾配）法で最適制御入力を計算し、
        5次多項式 FF = a*t^5 + ... + e*t にフィットする。

        y_meas / anchor を渡すと、5次多項式フィットに極値アンカー（トラストリージョン）が働く。
        y_meas は今回の実測データ（目標位置を原点へ移した y_shifted）で、
        目標モデル応答 y_tgt との二乗和誤差 J をフィット前に anchor へ渡すために使う
        （＝今回測ったJを反映したベスト極値・探索半径でフィットできる）。
        どちらも None のときは近接項の重みが0になり、従来どおりの純粋なフィットになる。

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

        # -----------------------------------------------------------------------------
        # 5次多項式フィット
        #   f(t) = a t^5 + b t^4 + c t^3 + d t^2 - (a T^4 + b T^3 + c T^2 + d T) t   (0≤t≤T)
        #   ・1次の係数を e = -(a T^4 + b T^3 + c T^2 + d T) に固定 → f(0)=f(T)=0 を必ず満たす
        #   ・f'(t)=(t-t1)(t-t2)·q(t)（q(t)は符号一定）の形で係数を作ることで、
        #     0<t<T の極値が必ず2個（山1つ・谷1つ）になることを構造的に保証する
        #   ・その条件を満たす形の中で u_opt との二乗和誤差が最小のものを選ぶ
        #   ・さらに極値アンカー（そのDOFで最小のJを出したFFの極値）が与えられている場合は、
        #     フィット精度が実質同等な候補の中でアンカーに最も近い解を選ぶ
        #   極値 (t1,y1),(t2,y2)・FF入力波形・返す係数はすべて上式から計算する。
        # -----------------------------------------------------------------------------
        t_ff = self.t_eval[self.t_eval <= self.T]                                                       # FF入力を与える時間だけ取り出す
        u_opt_ff = u_opt[: len(t_ff)]                                                                   # FF入力を与える区間だけの最適入力を取り出す

        # -----------------------------------------------------------------------------
        # 極値アンカーの更新（フィットの前に今回のJを反映させる）
        #   今回の実測データ y_meas を生んだのは前回ループで計算したFF（＝anchor.pending_extrema の極値）。
        #   そのJをここで確定させることで、フィットは「今わかっている最良の極値」を基準にできる。
        # -----------------------------------------------------------------------------
        if anchor is not None and y_meas is not None:                                                   # アンカーと実測データが与えられている場合
            anchor.update(float(np.sum((y_tgt - np.asarray(y_meas)) ** 2)))                             # 今回のJ（＝内側ループの判定用Jと同じ値）でベスト極値と探索半径を更新
        anchor_extrema = anchor.target() if anchor is not None else None                                # アンカーとして使う極値（無効なら None）
        w_prox = anchor.weight() if anchor is not None else 0.0                                         # 近接項の重み（無効なら0 → 従来どおりの純粋なフィット）

        # 5次関数パラメータeを計算する関数（f(T)=0 の条件）
        def calc_e(a, b, c, d):                                                                         # 引数(5次関数パラメータa・b・c・d)
            return -(a * self.T ** 4 + b * self.T ** 3 + c * self.T ** 2 + d * self.T)                  # 5次関数パラメータeを返す

        # 5次関数を定義する関数
        def poly(t, a, b, c, d):                                                                        # 引数(時間, 5次関数パラメータa・b・c・d)
            e = calc_e(a, b, c, d)                                                                      # 5次関数パラメータeを計算
            return a * t ** 5 + b * t ** 4 + c * t ** 3 + d * t ** 2 + e * t                            # 5次関数FF入力値を返す

        # f'(t)=0 の実根のうち 0<t<T にあるものを時間[s]の昇順で返す関数（重根は1個として数える）
        def extrema_times(a, b, c, d):                                                                  # 引数(5次関数パラメータa・b・c・d)
            e = calc_e(a, b, c, d)                                                                      # 5次関数パラメータeを計算
            coeffs = [5.0 * a, 4.0 * b, 3.0 * c, 2.0 * d, e]                                            # f'(t)=5a t^4+4b t^3+3c t^2+2d t+e の係数
            if not np.all(np.isfinite(coeffs)):                                                         # 発散した係数は極値なしとして扱う
                return []
            roots = np.roots(coeffs)                                                                    # f'(t)=0 の根（極値）を計算
            real_roots = sorted(
                r.real for r in roots if abs(r.imag) < 1e-6 and 0 < r.real < self.T                     # FF入力時間における実部の極を取り出し、小さい順に並べる
            )
            uniq = []                                                                                   # 重根を1個にまとめた実根リスト
            for r in real_roots:                                                                        # 実根を昇順に走査
                if not uniq or (r - uniq[-1]) > FIT_ROOT_TOL * self.T:                                  # 直前の根と十分離れていれば別の極値として採用
                    uniq.append(r)
            return uniq

        # -----------------------------------------------------------------------------
        # 極値を「山1つ・谷1つ」に構造的に固定する形状パラメータ表現
        #   f'(t) = (t-t1)(t-t2)·q(t),   q(t) = α(t-m)² + β    （α,β が同符号 → q(t) は符号一定）
        #   ・q(t) は0にならないので、f'(t) の符号が変わるのは t1, t2 の2点だけ
        #     → 0<t<T の極値は必ず2個（t1が山、t2が谷、またはその逆）。根を数えて判定する必要がない。
        #   ・0<t1<t2<T はシグモイド変換で必ず満たす
        #   ・f(T)=∫₀ᵀf'(t)dt = α·J1 + β·J2 = 0 は α=-J2·λ, β=J1·λ と置けば恒等的に成立する
        #       J1 = ∫₀ᵀ(t-t1)(t-t2)(t-m)²dt,  J2 = ∫₀ᵀ(t-t1)(t-t2)dt,  λ は振幅
        #   ・α=0（J2=0）のとき f は3次関数 C·t(t-tm)(t-T) に退化する（山1つ谷1つの理想形）
        #   最適化変数は z=[λ, z1, z2, m] の4個（a,b,c,d と同じ自由度）
        # -----------------------------------------------------------------------------
        def integrals(t1, t2, m):                                                                       # 引数(1つ目の極値, 2つ目の極値, q(t)の中心)
            A = t1 + t2                                                                                 # (t-t1)(t-t2)=t²-At+B の A
            B = t1 * t2                                                                                 # 同じく B
            J2 = self.T ** 3 / 3.0 - A * self.T ** 2 / 2.0 + B * self.T                                 # ∫₀ᵀ(t-t1)(t-t2)dt
            J1 = (self.T ** 5 / 5.0 + (-2.0 * m - A) * self.T ** 4 / 4.0                                # ∫₀ᵀ(t-t1)(t-t2)(t-m)²dt
                  + (m * m + 2.0 * m * A + B) * self.T ** 3 / 3.0
                  + (-A * m * m - 2.0 * m * B) * self.T ** 2 / 2.0
                  + B * m * m * self.T)
            return J1, J2, A, B

        # 形状パラメータ z=[λ,z1,z2,m] から5次関数の係数[a,b,c,d]と制約違反量を計算する関数
        def shape_to_coeffs(z):                                                                         # 引数(形状パラメータ[λ, z1, z2, m])
            if not np.all(np.isfinite(z)):                                                              # 発散した場合は無効とする
                return None, np.inf, 0.0, 0.0
            lam, z1, z2, m = z                                                                          # 振幅・極値位置・q(t)の中心を取り出す
            s1 = 1.0 / (1.0 + np.exp(-np.clip(z1, -50.0, 50.0)))                                        # 0<s1<1（シグモイド）
            s2 = 1.0 / (1.0 + np.exp(-np.clip(z2, -50.0, 50.0)))                                        # 0<s2<1（シグモイド）
            lo = FIT_EXTREMA_MARGIN * self.T                                                            # 極値の下限（端点0から離す）
            hi = (1.0 - FIT_EXTREMA_MARGIN) * self.T                                                    # 極値の上限（端点Tから離す）
            gap = FIT_EXTREMA_GAP * self.T                                                              # 山と谷の最小間隔
            t1 = lo + max(hi - gap - lo, 0.0) * s1                                                      # 1つ目の極値（lo≤t1≤hi-gap を必ず満たす）
            t2 = t1 + gap + max(hi - t1 - gap, 0.0) * s2                                                # 2つ目の極値（t1+gap≤t2≤hi を必ず満たす）
            J1, J2, A, B = integrals(t1, t2, m)                                                         # 端点条件に必要な積分値を計算
            alpha = -J2 * lam                                                                           # q(t)の2次の係数（この置き方で f(T)=0 が恒等的に成立）
            beta = J1 * lam                                                                             # q(t)の定数項

            # q(t) が 0≤t≤T で符号を変えない（＝極値が2個から増えない）条件を違反量として計算する
            q_min = abs(alpha * (min(max(m, 0.0), self.T) - m) ** 2 + beta)                             # 区間内の |q(t)| の最小値
            q_max = max(abs(alpha * m ** 2 + beta), abs(alpha * (self.T - m) ** 2 + beta))              # 区間内の |q(t)| の最大値
            violation = 0.0                                                                             # 制約違反量（0なら条件を満たす）
            if alpha * beta < 0.0:                                                                      # α,βが異符号 → q(t)が実根を持ち極値が4個になる
                violation += 1.0
            if q_max > 0.0:                                                                             # |q(t)|が区間内でほぼ0まで落ち込まない余裕を持たせる
                violation += max(0.0, FIT_QUAD_MARGIN - q_min / q_max)                                  # （準重根による平坦部と、float32変換での極値増加を防ぐ）
            else:
                violation += 1.0

            Pq = -2.0 * alpha * m                                                                       # q(t)=α t²+Pq t+Qq へ展開
            Qq = alpha * m * m + beta
            a = alpha / 5.0                                                                             # f'(t)の係数から f(t) の係数へ変換
            b = (Pq - alpha * A) / 4.0
            c = (Qq - A * Pq + alpha * B) / 3.0
            d = (-A * Qq + B * Pq) / 2.0
            return np.array([a, b, c, d]), violation, t1, t2

        # 5次関数の「フィット精度」だけを返す関数（近接項・極値制約を含まない ＝ 候補のガード判定に使う）
        def fit_error(p):                                                                               # 引数(5次関数パラメータ[a, b, c, d])
            u_pred = poly(t_ff, *p)                                                                     # 5次関数で計算したFF入力
            if not np.all(np.isfinite(u_pred)):                                                         # 発散した場合は無限大を返す
                return np.inf
            mse = np.sum((u_pred - u_opt_ff) ** 2)                                                      # 最適入力と近似したFF入力との二乗和誤差
            penalty_pwm = (                                                                             # -255～255の間に収めるためのペナルティ
                np.sum(np.maximum(0, u_pred - 255) ** 2)
                + np.sum(np.maximum(0, -255 - u_pred) ** 2)
            )
            return float(mse + FIT_PWM_WEIGHT * penalty_pwm)

        # アンカー極値からの距離（近接項）を返す関数
        #   時間は T、極値の大きさは FIT_PWM_SCALE[PWM] で正規化する。
        #   t は 0〜T[s]、y は 0〜255[PWM] とスケールが2桁違うため、生の距離では y だけで
        #   距離が決まってしまい極値時刻の連続性が全く効かない。
        #   t1<t2 は shape_to_coeffs が構造的に保証しているので、アンカーとの対応付けは常に一意。
        def prox_of(t1c, t2c, p):                                                                       # 引数(候補の1つ目の極値時刻, 2つ目の極値時刻, 5次関数パラメータ[a,b,c,d])
            if anchor_extrema is None:                                                                  # アンカーが無効なら近接項なし
                return 0.0
            t1a, y1a, t2a, y2a = anchor_extrema                                                         # アンカー極値（最小Jを出したFFの極値）
            y1c = poly(t1c, *p)                                                                         # 候補の1つ目の極値の大きさ
            y2c = poly(t2c, *p)                                                                         # 候補の2つ目の極値の大きさ
            if not (np.isfinite(y1c) and np.isfinite(y2c)):                                             # 発散した場合は非常に大きな値を返す
                return 1e30
            return float(((t1c - t1a) / self.T) ** 2 + ((t2c - t2a) / self.T) ** 2                      # 極値時刻の差（Tで正規化）
                         + ((y1c - y1a) / FIT_PWM_SCALE) ** 2                                           # 極値の大きさの差（PWMスケールで正規化）
                         + ((y2c - y2a) / FIT_PWM_SCALE) ** 2)

        # 最適入力を5次関数で近似するための評価関数（近接項を含めて最適化そのものをアンカーへ引き寄せる）
        #   候補の中から近いものを「選ぶ」だけでは、ランダム初期値から偶然出てきた解の集合が
        #   アンカー近傍を含まない限り効かない。損失に入れて探索自体を引き寄せる。
        def fit_loss(z):                                                                                # 引数(形状パラメータ[λ, z1, z2, m])
            p, violation, t1c, t2c = shape_to_coeffs(z)                                                 # 形状パラメータから係数と制約違反量を計算
            if p is None:                                                                               # 無効な形状は非常に大きな値を返す
                return 1e30
            err = fit_error(p)                                                                          # フィット精度（二乗和誤差＋PWMペナルティ）
            if not np.isfinite(err):                                                                    # 発散した場合は非常に大きな値を返す
                return 1e30
            return err + FIT_EXTREMA_WEIGHT * violation + w_prox * prox_of(t1c, t2c, p)                 # 精度＋極値制約＋近接項（w_prox=0なら従来と同一）

        peak_ref = float(np.max(np.abs(u_opt_ff))) if u_opt_ff.size else 0.0                            # 最適入力の振幅（初期値の基準）
        if peak_ref < 1e-9:                                                                             # 最適入力がほぼ0なら初期FFの振幅で代用
            peak_ref = INIT_FF_PEAK
        i_peak = int(np.argmax(np.abs(u_opt_ff))) if u_opt_ff.size else 0                               # 最適入力の振幅が最大となる位置
        sgn = 1.0 if (u_opt_ff.size == 0 or u_opt_ff[i_peak] >= 0) else -1.0                            # 最適入力の向き（第1極値の符号に合わせる）

        # 3次関数 f(t)=C t(t-T/2)(t-T)（山1つ谷1つの理想形）に対応する形状パラメータを返す関数
        def cubic_seed(peak):                                                                           # 引数(極値の大きさ)
            t1 = self.T * (3.0 - np.sqrt(3.0)) / 6.0                                                    # 3次関数の1つ目の極値
            t2 = self.T * (3.0 + np.sqrt(3.0)) / 6.0                                                    # 3次関数の2つ目の極値
            m = self.T / 2.0                                                                            # q(t)の中心
            J1, _, _, _ = integrals(t1, t2, m)                                                          # 積分値を計算（この配置では J2=0 → α=0 の3次関数）
            C = 12.0 * np.sqrt(3.0) * peak / (self.T ** 3)                                              # |f(t1)|=(√3/36)C T³=peak となる振幅
            lam = 3.0 * C / J1 if abs(J1) > 1e-30 else 0.0                                              # f'(t)=3C(t-t1)(t-t2) となる λ
            lo = FIT_EXTREMA_MARGIN * self.T                                                            # shape_to_coeffs と同じ範囲でシグモイドを逆変換する
            hi = (1.0 - FIT_EXTREMA_MARGIN) * self.T
            gap = FIT_EXTREMA_GAP * self.T
            s1 = np.clip((t1 - lo) / max(hi - gap - lo, 1e-30), 1e-6, 1.0 - 1e-6)                       # シグモイドの逆変換
            s2 = np.clip((t2 - t1 - gap) / max(hi - t1 - gap, 1e-30), 1e-6, 1.0 - 1e-6)
            return np.array([lam, np.log(s1 / (1.0 - s1)), np.log(s2 / (1.0 - s2)), m])

        # アンカー極値 (t1,y1,t2,y2) に対応する形状パラメータを返す関数（ウォームスタート用のシード）
        #   ・極値時刻 t1,t2 → シグモイドの逆変換で z1,z2 が一意に決まる
        #   ・q(t)の中心は極値の中点 m=(t1+t2)/2 に置く（cubic_seed の対称配置と整合する）
        #   ・f は λ について線形（α=-J2λ, β=J1λ より係数 a,b,c,d すべてがλに比例）なので、
        #     λ=1 の波形 g(t) を作れば 2つの極値の大きさに最小二乗で合わせる振幅が閉形式で求まる
        #       λ* = (y1·g(t1) + y2·g(t2)) / (g(t1)² + g(t2)²)
        def seed_from_extrema(ext):                                                                     # 引数(アンカー極値(t1, y1, t2, y2))
            t1a, y1a, t2a, y2a = ext                                                                    # アンカー極値を取り出す
            lo = FIT_EXTREMA_MARGIN * self.T                                                            # shape_to_coeffs と同じ可動域
            hi = (1.0 - FIT_EXTREMA_MARGIN) * self.T
            gap = FIT_EXTREMA_GAP * self.T
            t1c = min(max(t1a, lo), max(hi - gap, lo))                                                  # アンカーの極値時刻を可動域へ収める
            t2c = min(max(t2a, t1c + gap), hi)
            s1 = np.clip((t1c - lo) / max(hi - gap - lo, 1e-30), 1e-6, 1.0 - 1e-6)                      # シグモイドの逆変換
            s2 = np.clip((t2c - t1c - gap) / max(hi - t1c - gap, 1e-30), 1e-6, 1.0 - 1e-6)
            z1 = float(np.log(s1 / (1.0 - s1)))
            z2 = float(np.log(s2 / (1.0 - s2)))
            m = 0.5 * (t1c + t2c)                                                                       # q(t)の中心は2つの極値の中点に置く
            p_unit, _, tu1, tu2 = shape_to_coeffs(np.array([1.0, z1, z2, m]))                           # λ=1 のときの5次関数係数
            if p_unit is None:                                                                          # 無効な形状ならシードを作れない
                return None
            g1 = poly(tu1, *p_unit)                                                                     # λ=1 のときの1つ目の極値の大きさ
            g2 = poly(tu2, *p_unit)                                                                     # λ=1 のときの2つ目の極値の大きさ
            den = float(g1 * g1 + g2 * g2)
            if not np.isfinite(den) or den < 1e-30:                                                     # 退化した形状ならシードを作れない
                return None
            lam = float((y1a * g1 + y2a * g2) / den)                                                    # 極値の大きさを最小二乗で合わせる振幅λ
            if not np.isfinite(lam):
                return None
            return np.array([lam, z1, z2, m])

        # 3次関数族 f(t)=C t(t-tm)(t-T) の最小二乗フィット（山1つ谷1つを必ず満たす保険）
        def fit_cubic_family():
            best_c, best_res = None, np.inf                                                             # 最良の係数と残差
            for tm in np.linspace(0.05 * self.T, 0.95 * self.T, 91):                                    # 中間の零点 tm を走査
                g = t_ff * (t_ff - tm) * (t_ff - self.T)                                                # 形が決まれば振幅Cについて線形
                gg = float(g @ g)
                if gg < 1e-30:
                    continue
                C = float(g @ u_opt_ff) / gg                                                            # 二乗和誤差が最小となる振幅
                res_val = float(np.sum((C * g - u_opt_ff) ** 2))                                        # そのときの残差
                if res_val < best_res:                                                                  # より誤差が小さければ更新
                    best_res = res_val
                    best_c = np.array([0.0, 0.0, C, -C * (tm + self.T)])                                # C t³ - C(tm+T) t² の係数
            return best_c

        # 決定論的な初期値（この順に試し、以降はランダム初期値）
        #   アンカーが有効なら、その極値から作ったシードを最初に試して確実にベスト近傍を探索させる。
        z_seeds = []                                                                                    # 決定論的な初期値のリスト
        if anchor_extrema is not None:                                                                  # アンカーが有効な場合
            z_anchor = seed_from_extrema(anchor_extrema)                                                 # ベスト極値からのウォームスタート用シード
            if z_anchor is not None:
                z_seeds.append(z_anchor)
        z_seeds.append(cubic_seed(sgn * peak_ref))                                                      # 従来どおりの3次関数シード

        # 山1つ谷1つを必ず満たす形の中で、u_optとの二乗和誤差が最小のフィットを探す
        #   ・形状パラメータ表現により、探索中の候補はすべて極値2個を構造的に満たす
        #   ・初期値依存の悪い解を掴まないよう最低 FIT_MIN_TRIAL 個の初期値を試して最良を採用する
        #   ・条件を満たした候補は (フィット精度, アンカーからの距離, 係数) の組で全部ためておき、
        #     最後にガード付きで選ぶ
        cand_list = []                                                                                  # 条件を満たした候補 [(フィット精度, 近接項, 係数)]
        for attempt in range(FIT_MAX_RETRY):                                                            # フィット再試行ループ
            if attempt < len(z_seeds):                                                                  # 決定論的な初期値がある間はそれを使う
                z0 = z_seeds[attempt]
            else:                                                                                       # 以降は振幅・極値位置・q(t)の中心をランダムに変えて別の解を狙う
                z0 = cubic_seed(sgn * peak_ref * np.random.uniform(0.2, 1.5))
                z0 = z0 + np.array([
                    z0[0] * np.random.uniform(-0.5, 0.5),
                    np.random.normal(0.0, 0.8),
                    np.random.normal(0.0, 0.8),
                    np.random.uniform(-0.5, 0.5) * self.T,
                ])

            res = scipy.optimize.minimize(                                                              # fit_lossが最小になる形状パラメータを取得
                fit_loss, z0, method='Nelder-Mead',
                options={'maxiter': 2000, 'maxfev': 2000, 'xatol': 1e-8, 'fatol': 1e-8},
            )
            p_cand, violation, t1_cand, t2_cand = shape_to_coeffs(res.x)                                # 得られた形状から係数を計算

            if p_cand is not None and violation <= 0.0 and len(extrema_times(*p_cand)) == 2:            # 念のため実際の係数でも極値2個を確認
                err_cand = fit_error(p_cand)                                                            # 近接項を含まない純粋なフィット精度
                if np.isfinite(err_cand):                                                               # 有効な候補として保存
                    cand_list.append((err_cand, prox_of(t1_cand, t2_cand, p_cand), p_cand))
                    if attempt + 1 >= FIT_MIN_TRIAL:                                                    # 最低試行回数を満たしていればフィット完了
                        break

        # 候補の選択（フィット品質のガード付き）
        #   近接項は「u_optへの近似精度をわざと捨てる」操作なので、精度が実質同等な候補に限って
        #   アンカーに近いものを選ぶ。u_optが本当に大きく変わったとき（＝精度が明確に落ちるとき）は
        #   アンカーに縛らず精度優先の解を採用し、必要な変化を殺さないようにする。
        if cand_list:                                                                                   # 条件を満たす候補がある場合
            err_min = min(c[0] for c in cand_list)                                                      # 最良のフィット精度
            tol = err_min * (1.0 + FIT_MSE_TOL) + 1e-12                                                 # 許容するフィット精度の劣化（εは FIT_MSE_TOL）
            best_p = min((c for c in cand_list if c[0] <= tol), key=lambda c: c[1])[2]                  # 同等な候補の中でアンカーに最も近い解を採用
        else:                                                                                           # 条件を満たす解が得られなかった場合は3次関数族で近似する
            best_p = fit_cubic_family()
            print(f"[FF fit] 警告: 5次関数で条件を満たす解が得られなかったため3次関数で近似しました。")

        a, b, c, d = best_p                                                                             # 最適化した5次関数パラメータを取得
        e = calc_e(a, b, c, d)                                                                          # 5次関数パラメータeを計算

        # 極値 (t1,y1), (t2,y2) を抽出
        real_roots = extrema_times(a, b, c, d)                                                          # 極値を計算
        if len(real_roots) >= 2:                                                                        # 極値が2つ以上なら、一番早い極値と二番目の極値を使う
            t1, t2 = real_roots[0], real_roots[1]
        elif len(real_roots) == 1:                                                                      # 極値が1つなら、その極値とFF入力時間の半分の値を使う
            t1 = real_roots[0]
            t2 = self.T / 2.0
        else:                                                                                           # 極値が0なら、FF入力時間から算出する
            t1 = self.T * 0.33
            t2 = self.T * 0.66

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
        self.fit_anchors = [ExtremaAnchor() for _ in range(24)]                                                                                 # 5次多項式フィットの極値アンカー（DOFごとのベストJ・ベスト極値・探索半径）

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
                    f_T = a * self.T ** 5 + b * self.T ** 4 + c * self.T ** 3 + d * self.T ** 2 + e * self.T    # 端点 t=T でのFF入力値（0になることの確認用）
                    self.get_logger().info(                                                                     # ログ出力（丸めた値では f(T)=0 を確認できないため有効数字を多く表示する）
                        f"  B{b_id}-D{_ + 1} (DOF {dof_idx + 1:02d}): "
                        f"a={a:.6e}, b={b:.6e}, c={c:.6e}, d={d:.6e}, e={e:.6e} | "
                        f"t1={t1:.3f}, y1={y1:.2f}, t2={t2:.3f}, y2={y2:.2f} | f(T)={f_T:.2e}"
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

                # 1. Target Model ID
                tgt_params = solver.fit_target_model(y_shifted, y0)                                         # 目標モデルの同定をして、パラメータを取得

                # 2. System Model ID
                a, b, c, d, e = self.current_ff_matrix[dof_idx]                                             # 現在のDOFのFFパラメータを取り出す
                t_ff = solver.t_eval[solver.t_eval <= self.T]                                               # 0秒～FF入力時間までの時間配列を取り出す
                u_ff = np.zeros(N_samples)                                                                  # 入力ベクトルの生成
                u_ff[:len(t_ff)] = a * t_ff ** 5 + b * t_ff ** 4 + c * t_ff ** 3 + d * t_ff ** 2 + e * t_ff # FF入力時間だけ、そのときのFF入力を格納する

                sys_params = solver.fit_system_model(y_shifted, u_ff, y0)                                   # システムモデルの同定をして、パラメータを取得

                # 3. Calculate squared error J
                a2_sys, a1_sys, a0_sys, b0_sys = sys_params                                                 # システムモデル係数を取り出す
                y_sys_sim, _, _ = solver.simulate_forced(solver.t_eval, a2_sys, a1_sys, a0_sys, b0_sys, u_ff, y0)   # 同定したシステムモデルの応答を取り出す

                # 4. 離散時間オイラー・ラグランジュ最適制御入力計算 + 5次多項式フィット
                #    今回の実測データ y_shifted を生んだFFの極値をアンカー候補として渡す
                #    （calculate_el_ff 内で今回のJが確定し、ベスト極値と探索半径が更新される）
                self.fit_anchors[dof_idx].pending_extrema = used_extrema_list[dof_idx]                   # 今回印加したFFの極値をアンカー候補に設定
                new_ff, extrema, u_pred_full, y_tgt, u_opt = solver.calculate_el_ff(                     # 最適入力を計算して、その結果の、FFパラメータ、極値、5次関数のFF制御入力、目標モデルの応答、最適入力を格納
                    tgt_params, sys_params, u_ff, y0,
                    y_meas=y_shifted, anchor=self.fit_anchors[dof_idx],
                )
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

                # 次回ループで印加する（＝次回の実測データを生成する）入力を保存
                self.prev_u_pred_full[dof_idx] = u_pred_full                                                 # 次回ループ用の5次関数FF入力を保存
                self.prev_u_opt[dof_idx] = u_opt                                                             # 次回ループ用の最適制御入力を保存

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
                self.fit_anchors = [ExtremaAnchor() for _ in range(24)]                                                 # 目標値が変わりJの基準も変わるため極値アンカーもリセット

            self.state = "INIT_ROBOT"                                                                                   # 初期状態に戻す
            self.state_start_time = self.get_clock().now()                                                              # 現在時刻を取得

        except Exception as exc:                                                                                                                    # 何かエラーが出たときの処理
            self.get_logger().error(f"最適化パイプライン異常: {exc}\n{traceback.format_exc()}")
            self.state = "FINISHED"
            self.state_start_time = self.get_clock().now()

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
