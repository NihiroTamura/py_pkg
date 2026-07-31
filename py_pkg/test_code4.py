#!/usr/bin/env python3
"""評価関数の重み Q, R を自動チューニングするプログラム（test_code3.py ベース）

test_code3.py は J = Σ(eᵀQe + Ru²) の重み Q, R を手動で調整していたため、
最適制御入力 u_opt が小さくなり過ぎて実際には意味のある入力にならないことがあった。
本プログラムは Q, R を Optuna で自動探索する。

【ロボットは1回しか動かさない】
  1. 初期姿勢へ移動 → 励振用の初期FFを印加して5秒間の実測データを1回だけ取得する
  2. その実測データから目標モデル・システムモデルを同定し、
     同定したシステムモデルを以降「真のモデル」とみなす
  3. 以降はロボットを一切動かさず、すべてシミュレーションのみで Q, R を探索する
     （同定結果はJSONへ保存されるので、モード2で読み込めば再探索でもロボットは不要）

【1候補(Q,R)の評価手順】
  1. 真のモデル（システムモデル）と 2. 目標モデル を使い、
     離散時間オイラー・ラグランジュ法（calculate_el_ff, アルゴリズム変更なし）で u_opt を計算
  3. u_opt を真のモデルへ入力して応答を計算
  4. 目標軌道（目標モデル）との差 J = Σ(y_tgt - y_sys)² を計算
  5. 対象DOF（DOF4）の u_opt が max(u_opt) ≥ 60 または min(u_opt) ≤ -60 を満たすことを必須条件とし、
     満たさない候補には大きなペナルティを与える

得られた Q, R は test_code3.py の COST_Q, COST_R へ設定して使用する。
"""
import os                                                       # OSライブラリ
import sys                                                      # Pythonを扱うライブラリ
import json                                                     # 同定結果の保存・読み込み
import time                                                     # 時間
import threading                                                # スレッド処理
import traceback                                                # エラー内容表示
import warnings                                                 # 警告表示を制御

import matplotlib                                               # グラフライブラリ
matplotlib.use('Agg')                                           # 画像保存専用モードに変更
import matplotlib.pyplot as plt                                 # プロット
import numpy as np                                              # 数学計算
import pandas as pd                                             # csv保存

import rclpy                                                    # ROS2ライブラリ
from rclpy.node import Node                                     # Nodeクラスの読み込み
from std_msgs.msg import Float32MultiArray, UInt16MultiArray    # ROS2メッセージ型

import scipy.optimize                                           # 最適化ライブラリ
from scipy.signal import cont2discrete                          # 連続時間→離散時間(ZOH)厳密離散化

import optuna                                                   # ブラックボックス最適化ライブラリ（Q, R の自動探索）

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
# Q, R 自動チューニングのパラメータ（チューニング要素）
# ==============================================================================
TUNE_TARGET_DOF = 4         # 振幅の必須条件を課し、追従誤差Jを評価する対象DOF（1始まり。プリセット目標値で大きく動く自由度）
TUNE_U_THRESHOLD = 60.0     # 必須条件の入力振幅閾値 [PWM]（max(u_opt)≥60 または min(u_opt)≤-60）
TUNE_N_TRIALS = 200         # Optunaの探索回数の既定値（実行時に引数で指定可能）
TUNE_SAMPLER = 'tpe'        # Optunaのサンプラー（'tpe': TPESampler / 'cmaes': CmaEsSampler）
TUNE_SEED = 0               # サンプラーの乱数シード（Noneで毎回変化）
TUNE_Q_RANGE = (1e-3, 1e6)  # 探索範囲 q1, q2, q3（対数スケール）
TUNE_R_RANGE = (1e-6, 1e3)  # 探索範囲 r（対数スケール）
TUNE_PENALTY = 1e12         # 必須条件（入力振幅）を満たさない候補へ与える大ペナルティ（実現しうる最大のJより十分大きい値）
TUNE_SEED_MANUAL = True     # Trueなら手動値(COST_Q, COST_R)を初回トライアルとして必ず評価し、比較対象にする

# ------------------------------------------------------------------------------
# 評価に使う入力の選択（'u_opt' or 'ff'）
#   'u_opt' : 仕様どおり、最適制御入力 u_opt そのもので J と振幅条件を評価する。
#   'ff'    : u_opt を5次多項式近似したFF入力（＝実際にロボットへ送る入力）で
#             J と振幅条件を評価する。
#   ※ u_opt は f(0)=0 の制約が無いため t=0 近傍のインパルス状になりやすく、
#      その場合 5次多項式（f(0)=f(T)=0, 極値2個）では表現できず、実機へ送るFFの
#      振幅がほぼ0まで縮んでしまう。実機で意味のある入力を得たい場合は 'ff' にする。
#      どちらを選んでも両方の結果（J, J_ff, u_optとFFの振幅）は表示・保存される。
# ------------------------------------------------------------------------------
TUNE_EVAL_TARGET = 'u_opt'

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

        # 最適入力を5次関数で近似するための評価関数
        def fit_loss(p):
            a, b, c, d = p                                                                          # 最適化5次関数パラメータ変数を取り出す                                                                              
            u_pred = poly(t_ff, a, b, c, d)                                                         # 5次関数で計算したFF入力
            mse = np.sum((u_pred - u_opt_ff) ** 2)                                                  # 最適入力と近似したFF入力との二乗和誤差
            penalty_pwm = (                                                                         # -255～255の間に収めるためのペナルティ
                np.sum(np.maximum(0, u_pred - 255) ** 2)
                + np.sum(np.maximum(0, -255 - u_pred) ** 2)
            )
            e_val = -(a * self.T ** 4 + b * self.T ** 3 + c * self.T ** 2 + d * self.T)             # 5次関数パラメータeを計算
            dp = 5 * a * t_ff ** 4 + 4 * b * t_ff ** 3 + 3 * c * t_ff ** 2 + 2 * d * t_ff + e_val   # 微分値を計算
            sign_changes = np.count_nonzero(np.diff(dp > 0))                                        # 傾き（微分値）の符号が変わった回数を取得
            penalty_extrema = abs(sign_changes - 2) * 1e7                                           # 極値が2つになるためのペナルティ
            return mse + 1e6 * penalty_pwm + penalty_extrema

        initial_guess = [0.0, 0.0, 0.01, -0.015 * self.T]                                               # 最適化5次関数パラメータの初期値
        res = scipy.optimize.minimize(fit_loss, initial_guess, method='Nelder-Mead')                    # fit_lossが最小になる5次関数パラメータを取得
        a, b, c, d = res.x                                                                              # 最適化した5次関数パラメータを取得
        e = -(a * self.T ** 4 + b * self.T ** 3 + c * self.T ** 2 + d * self.T)                         # 5次関数パラメータeを計算

        # 極値 (t1,y1), (t2,y2) を抽出
        roots = np.roots([5 * a, 4 * b, 3 * c, 2 * d, e])                                               # 極値を計算
        real_roots = sorted(
            r.real for r in roots if abs(r.imag) < 1e-6 and 0 < r.real < self.T                         # FF入力時間における実部の極を取り出し、小さい順に並べる
        )
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
# 評価関数の重み Q, R の自動探索クラス（Optuna）
#   同定済みの「真のモデル」と「目標モデル」だけを使い、ロボットを動かさず
#   シミュレーションのみで Q = diag(q1,q2,q3), R = [[r]] の4変数を探索する。
#
#   ※ コスト J = Σ(eᵀQe + Ru²) は (Q,R) → (cQ,cR)（c>0）に対して同じ u_opt を与える
#      （スケール不変）。つまり本質的に効くのは Q と R の「比」であり、4変数のうち
#      1自由度は冗長である。仕様どおり4変数を探索するが、結果として複数の (Q,R) が
#      同じ性能を示すのはこの性質による。
# ==============================================================================
class QRTuner:
    # コンストラクタ
    def __init__(self, T, dt, model, n_trials=TUNE_N_TRIALS, sampler_name=TUNE_SAMPLER, logger=None):    # 引数(FF制御入力時間, ステップ時間, 対象DOFの同定済みモデル, 探索回数, サンプラー名, ROS2ロガー)
        self.T = T                                                          # FF制御入力時間を保存
        self.dt = dt                                                        # シミュレーションステップ時間を保存
        self.model = model                                                  # 対象DOFの同定済みモデル（tgt_params, sys_params, u_ff, y0, Pf …）
        self.n_trials = n_trials                                            # 探索回数を保存
        self.sampler_name = sampler_name                                    # サンプラー名を保存
        self.logger = logger                                               # ROS2ロガー（無ければprintで代用）
        self.t_eval = np.arange(0, SIM_TIME, dt)                            # シミュレーション時間配列（プロット用）
        self.study = None                                                   # Optunaのstudy
        self.best = None                                                    # 最良トライアルの評価結果（配列を含む）
        self.best_score = float('inf')                                      # 最良トライアルの目的関数値
        self.manual = None                                                  # 手動値(COST_Q, COST_R)の評価結果（比較用）

    # ログ出力関数（ROS2ロガーがあればinfo、無ければprintで代用）
    def _log(self, msg):
        if self.logger is not None:
            self.logger.info(msg)
        else:
            print(msg)

    # ------------------------------------------------------------------
    # 評価関数：候補(Q,R)から u_opt を計算し、真のモデルへ入力して追従誤差Jを求める
    # ------------------------------------------------------------------
    def evaluate(self, Q, R):                                                                                       # 引数(状態重み行列, 入力重み行列)
        """候補(Q,R)を1つ評価する（ロボットは動かさず、シミュレーションのみ）"""
        m = self.model                                                                                              # 対象DOFの同定済みモデルを取り出す
        a2, a1, a0, b0 = m['sys_params']                                                                            # 真のモデル（システムモデル）の係数を取り出す
        solver = MathematicalSolver(self.T, self.dt, Q=Q, R=R)                                                      # 候補(Q,R)を持つソルバーを生成（calculate_el_ff は一切変更しない）

        # 手順1,2: 真のモデル（システムモデル）と目標モデルを使い、オイラー・ラグランジュ法で最適制御入力を計算
        ff_params, extrema, u_pred_full, y_tgt, u_opt = solver.calculate_el_ff(                                      # 5次関数パラメータ、極値、5次関数FF入力、目標モデル応答、最適入力
            m['tgt_params'], m['sys_params'], m['u_ff'], m['y0']
        )

        # 手順3: u_opt を真のモデルへ入力して応答を計算
        y_sys, _, _ = solver.simulate_forced(solver.t_eval, a2, a1, a0, b0, u_opt, m['y0'])                          # 真のモデルの応答（最適入力 u_opt を印加）

        # 手順4: 目標軌道（目標モデル）との差 J を計算
        J = solver.compute_J(y_tgt, y_sys)                                                                          # 追従誤差 J = Σ(y_tgt - y_sys)²

        # 参考情報: 実際にロボットへ与えるのは u_opt を5次多項式近似したFF入力なので、その応答も評価しておく
        y_ff, _, _ = solver.simulate_forced(solver.t_eval, a2, a1, a0, b0, u_pred_full, m['y0'])                     # 真のモデルの応答（5次関数FF入力を印加）
        J_ff = solver.compute_J(y_tgt, y_ff)                                                                         # 5次関数FF入力での追従誤差

        u_max = float(np.max(u_opt))                                                                                # u_opt の最大値
        u_min = float(np.min(u_opt))                                                                                # u_opt の最小値
        ff_max = float(np.max(u_pred_full))                                                                          # 5次関数FF入力の最大値
        ff_min = float(np.min(u_pred_full))                                                                          # 5次関数FF入力の最小値
        # 手順5: 必須条件（対象DOFの入力振幅）の判定
        satisfied = (u_max >= TUNE_U_THRESHOLD) or (u_min <= -TUNE_U_THRESHOLD)                                     # max(u_opt)≥60 または min(u_opt)≤-60
        satisfied_ff = (ff_max >= TUNE_U_THRESHOLD) or (ff_min <= -TUNE_U_THRESHOLD)                                 # 5次関数FF入力についての同じ判定
        sat_ratio = float(np.mean(np.abs(u_opt) >= 254.9))                                                           # PWM上限(±255)に張り付いている割合

        return {
            'J': J, 'J_ff': J_ff,                                               # 追従誤差（u_opt印加時 / 5次関数FF印加時）
            'u_max': u_max, 'u_min': u_min,                                     # u_opt の振幅
            'ff_max': ff_max, 'ff_min': ff_min,                                 # 5次関数FF入力の振幅
            'satisfied': satisfied, 'satisfied_ff': satisfied_ff,               # 必須条件の判定
            'sat_ratio': sat_ratio,                                             # PWM飽和割合
            'ff_params': list(ff_params), 'extrema': tuple(extrema),            # 5次関数パラメータと極値
            'u_opt': u_opt, 'u_pred_full': u_pred_full,                         # 最適入力と5次関数FF入力
            'y_tgt': y_tgt, 'y_sys': y_sys, 'y_ff': y_ff,                       # 目標モデル応答と真のモデル応答
            'Q': np.array(Q, dtype=float), 'R': np.array(R, dtype=float),       # 評価した重み行列
        }

    # ------------------------------------------------------------------
    # Optunaの目的関数（最小化）
    # ------------------------------------------------------------------
    def objective(self, trial):                                                                             # 引数(Optunaのトライアル)
        q1 = trial.suggest_float('q1', *TUNE_Q_RANGE, log=True)                                             # 状態誤差の重み q1（位置誤差, 対数スケール）
        q2 = trial.suggest_float('q2', *TUNE_Q_RANGE, log=True)                                             # 状態誤差の重み q2（速度誤差, 対数スケール）
        q3 = trial.suggest_float('q3', *TUNE_Q_RANGE, log=True)                                             # 状態誤差の重み q3（加速度誤差, 対数スケール）
        r = trial.suggest_float('r', *TUNE_R_RANGE, log=True)                                                # 制御入力の重み r（対数スケール）
        Q = np.diag([q1, q2, q3])                                                                           # 対角の状態重み行列 Q = diag(q1,q2,q3)
        R = np.array([[r]])                                                                                 # 入力重み行列 R = [[r]]

        # 候補(Q,R)を評価（数値的に破綻した候補は大ペナルティで棄却する）
        try:
            res = self.evaluate(Q, R)                                                                       # 追従誤差Jと最適入力の情報を取得
        except Exception as exc:                                                                            # 評価に失敗した候補
            self._log(f"  [trial {trial.number:4d}] 評価失敗のため棄却: {exc}")
            return TUNE_PENALTY * 10.0                                                                      # 最大級のペナルティを返す

        # 評価に使う入力を選択（'u_opt': 最適制御入力そのもの / 'ff': 実機へ送る5次関数近似FF入力）
        if TUNE_EVAL_TARGET == 'ff':
            J = res['J_ff']                                                                                 # 5次関数FF入力を印加したときの追従誤差
            satisfied = res['satisfied_ff']                                                                 # 5次関数FF入力についての振幅条件
            peak = max(res['ff_max'], -res['ff_min'])                                                        # 5次関数FF入力の振幅ピーク
        else:
            J = res['J']                                                                                    # 最適制御入力を印加したときの追従誤差
            satisfied = res['satisfied']                                                                    # u_opt についての振幅条件
            peak = max(res['u_max'], -res['u_min'])                                                          # u_opt の振幅ピーク

        if not np.isfinite(J):                                                                              # 発散した候補
            return TUNE_PENALTY * 10.0                                                                      # 最大級のペナルティを返す

        # 必須条件を満たさない候補には大ペナルティを与える
        #   閾値までの不足量に応じて連続的に増やし、探索が振幅を大きくする方向へ進むようにする
        if satisfied:
            score = J                                                                                       # 条件を満たすので追従誤差そのものを目的関数値とする
        else:
            score = TUNE_PENALTY * (2.0 - max(0.0, peak) / TUNE_U_THRESHOLD)                                 # 不足が大きいほど大きいペナルティ（1e12～2e12）

        # 結果表示用の付加情報をトライアルへ保存
        trial.set_user_attr('J', res['J'])                                                                  # u_opt印加時の追従誤差
        trial.set_user_attr('J_ff', res['J_ff'])                                                            # 5次関数FF入力印加時の追従誤差
        trial.set_user_attr('u_max', res['u_max'])                                                          # u_opt最大値
        trial.set_user_attr('u_min', res['u_min'])                                                          # u_opt最小値
        trial.set_user_attr('ff_max', res['ff_max'])                                                        # 5次関数FF入力の最大値
        trial.set_user_attr('ff_min', res['ff_min'])                                                        # 5次関数FF入力の最小値
        trial.set_user_attr('satisfied', bool(res['satisfied']))                                            # u_optについての必須条件の判定
        trial.set_user_attr('satisfied_ff', bool(res['satisfied_ff']))                                      # 5次関数FF入力についての必須条件の判定
        trial.set_user_attr('sat_ratio', res['sat_ratio'])                                                  # PWM飽和割合

        # 手動値(COST_Q, COST_R)のトライアル（初回）は比較用に保存する
        if self.manual is None and TUNE_SEED_MANUAL and trial.number == 0:
            self.manual = res                                                                               # 手動値の評価結果を保存

        # 最良トライアルの結果（プロット用の配列を含む）を保持する
        if score < self.best_score:
            self.best_score = score                                                                         # 最良の目的関数値を更新
            self.best = res                                                                                 # 最良の評価結果を更新
            mark = "  <-- best"                                                                             # 最良更新の印
        else:
            mark = ""

        self._log(                                                                                          # トライアルごとの進捗ログ
            f"  [trial {trial.number:4d}/{self.n_trials}] "
            f"q=({q1:.3e},{q2:.3e},{q3:.3e}) r={r:.3e} | "
            f"J={J:.4e} u_opt=[{res['u_min']:8.2f},{res['u_max']:8.2f}] "
            f"FF=[{res['ff_min']:7.2f},{res['ff_max']:7.2f}] "
            f"{'OK ' if satisfied else 'NG '}{mark}"
        )
        return score                                                                                        # 目的関数値（追従誤差 + ペナルティ）を返す

    # ------------------------------------------------------------------
    # 探索の実行
    # ------------------------------------------------------------------
    def run(self):
        self._log("=" * 100)
        self._log(
            f"  Q, R 自動探索開始  (Optuna/{self.sampler_name.upper()}, 探索回数={self.n_trials}, "
            f"対象DOF={self.model['dof']}, 評価入力={TUNE_EVAL_TARGET}, 必須条件|入力|>={TUNE_U_THRESHOLD:.0f})"
        )
        self._log(f"    真のモデル(システムモデル): a2={self.model['sys_params'][0]:.4f}, a1={self.model['sys_params'][1]:.4f}, "
                  f"a0={self.model['sys_params'][2]:.4f}, b0={self.model['sys_params'][3]:.4f}")
        self._log(f"    目標モデル: T1={self.model['tgt_params'][0]:.4f}, wn={self.model['tgt_params'][1]:.4f}, y0={self.model['y0']:.2f}")
        self._log("=" * 100)

        optuna.logging.set_verbosity(optuna.logging.WARNING)                                                 # Optuna自身のログは抑制する（進捗は自前で出力）
        if self.sampler_name == 'cmaes':                                                                     # CMA-ESサンプラー
            sampler = optuna.samplers.CmaEsSampler(seed=TUNE_SEED, n_startup_trials=10)
        else:                                                                                                # TPEサンプラー（既定）
            sampler = optuna.samplers.TPESampler(seed=TUNE_SEED, multivariate=True, n_startup_trials=20)
        self.study = optuna.create_study(direction='minimize', sampler=sampler)                              # 最小化のstudyを作成

        if TUNE_SEED_MANUAL:                                                                                 # 手動値を初回トライアルとして必ず評価する
            self.study.enqueue_trial({
                'q1': float(COST_Q[0, 0]), 'q2': float(COST_Q[1, 1]),
                'q3': float(COST_Q[2, 2]), 'r': float(COST_R[0, 0]),
            })

        t_start = time.time()                                                                                # 探索開始時刻
        self.study.optimize(self.objective, n_trials=self.n_trials, show_progress_bar=False)                  # 探索実行
        elapsed = time.time() - t_start                                                                       # 探索所要時間

        if self.best is None:                                                                                 # 全トライアルが失敗した場合
            self._log("すべてのトライアルが評価に失敗しました。探索範囲を見直してください。")
            return None

        self.print_result(elapsed)                                                                            # 結果表示
        return self.best

    # 探索結果を表示する関数
    def print_result(self, elapsed=None):                                                                     # 引数(探索所要時間)
        b = self.best                                                                                         # 最良の評価結果
        q1, q2, q3 = float(b['Q'][0, 0]), float(b['Q'][1, 1]), float(b['Q'][2, 2])                            # 最良のQの対角成分
        r = float(b['R'][0, 0])                                                                               # 最良のR
        key = 'satisfied_ff' if TUNE_EVAL_TARGET == 'ff' else 'satisfied'                                      # 評価対象に応じた必須条件のキー
        n_ok = sum(1 for t in self.study.trials if t.user_attrs.get(key, False))                               # 必須条件を満たしたトライアル数
        J_best = b['J_ff'] if TUNE_EVAL_TARGET == 'ff' else b['J']                                             # 評価対象に応じた最終J

        self._log("")
        self._log("=" * 100)
        self._log(f"  Q, R 自動探索の結果  (評価対象の入力: {TUNE_EVAL_TARGET})")
        self._log("=" * 100)
        self._log(f"  Best Q        = np.diag([{q1:.6g}, {q2:.6g}, {q3:.6g}])")
        self._log(f"  Best R        = np.array([[{r:.6g}]])")
        self._log(f"  最終J         = {J_best:.6f}   （目標軌道と真のモデル応答の二乗和誤差）")
        self._log(f"  u_opt最大値   = {b['u_max']:.4f}")
        self._log(f"  u_opt最小値   = {b['u_min']:.4f}")
        self._log("-" * 100)
        self._log(
            f"  必須条件 (DOF{self.model['dof']}: max>={TUNE_U_THRESHOLD:.0f} または min<=-{TUNE_U_THRESHOLD:.0f})"
        )
        self._log(
            "    u_opt（最適制御入力）         : "
            + ("満たす" if b['satisfied'] else "★満たさない★")
            + f"   [min {b['u_min']:.2f}, max {b['u_max']:.2f}]"
        )
        self._log(
            "    5次関数FF（実機へ送る入力）   : "
            + ("満たす" if b['satisfied_ff'] else "★満たさない★")
            + f"   [min {b['ff_min']:.2f}, max {b['ff_max']:.2f}]"
        )
        self._log(f"  条件を満たしたトライアル数 : {n_ok} / {len(self.study.trials)}"
                  + ("" if n_ok > 0 else "  ← 0件です。探索回数を増やすか、探索範囲/閾値を見直してください"))
        self._log(f"  PWM上限(±255)への飽和割合  : {b['sat_ratio'] * 100:.1f} %")
        self._log(f"  u_opt印加時のJ              : {b['J']:.6f}")
        self._log(f"  5次関数FF印加時のJ          : {b['J_ff']:.6f}  "
                  f"（極値 t1={b['extrema'][0]:.3f}, y1={b['extrema'][1]:.2f}, t2={b['extrema'][2]:.3f}, y2={b['extrema'][3]:.2f}）")
        if self.manual is not None:                                                                            # 手動値との比較
            self._log("-" * 100)
            self._log(
                f"  比較: 手動値 Q=diag({float(COST_Q[0,0]):.6g}, {float(COST_Q[1,1]):.6g}, {float(COST_Q[2,2]):.6g}), "
                f"R=[[{float(COST_R[0,0]):.6g}]]"
            )
            self._log(
                f"        → J={self.manual['J']:.6f} (5次関数FF: {self.manual['J_ff']:.6f}), "
                f"u_opt=[{self.manual['u_min']:.4f}, {self.manual['u_max']:.4f}], "
                f"FF=[{self.manual['ff_min']:.4f}, {self.manual['ff_max']:.4f}]"
            )
        if elapsed is not None:
            self._log(f"  探索所要時間 : {elapsed:.1f} 秒")
        self._log("=" * 100)
        self._log("  ↓ この2行を test_code3.py の COST_Q, COST_R に貼り替えてください")
        self._log(f"    COST_Q = np.diag([{q1:.6g}, {q2:.6g}, {q3:.6g}])")
        self._log(f"    COST_R = np.array([[{r:.6g}]])")
        self._log("=" * 100)
        self._log("")

    # 全トライアルの結果をCSVへ保存する関数
    def save_trials_csv(self, path):                                                                          # 引数(保存先パス)
        df = self.study.trials_dataframe()                                                                    # 全トライアルの結果をDataFrameへ変換
        df.to_csv(path, index=False)                                                                           # CSVへ保存
        self._log(f"全トライアル結果を保存: {path}")

    # 最良結果の応答と入力をグラフ化して保存する関数
    def save_result_plot(self, path):                                                                         # 引数(保存先パス)
        b = self.best                                                                                          # 最良の評価結果
        Pf = self.model['Pf']                                                                                  # 目標位置（POT値へ戻すためのオフセット）
        fig, axes = plt.subplots(2, 1, figsize=(9, 8))                                                         # 応答用と入力用の2段グラフ

        axes[0].plot(self.t_eval, b['y_tgt'] + Pf, '--', label='Target model (y_tgt)')                          # 目標軌道
        axes[0].plot(self.t_eval, b['y_sys'] + Pf, '-', label='True model with u_opt')                          # 最適入力を印加した真のモデルの応答
        axes[0].plot(self.t_eval, b['y_ff'] + Pf, ':', label='True model with 5th-order FF')                     # 5次関数FF入力を印加した応答
        axes[0].set_title(f"DOF {self.model['dof']} response (J = {b['J']:.2f})")
        axes[0].set_xlabel('Time [s]')
        axes[0].set_ylabel('POT value')
        axes[0].legend()
        axes[0].grid(True)

        axes[1].plot(self.t_eval, b['u_opt'], '-', label='Optimal input (u_opt)')                               # 最適入力
        axes[1].plot(self.t_eval, b['u_pred_full'], '--', label='5th-order FF (u_pred_full)')                     # 5次関数FF入力
        axes[1].axhline(TUNE_U_THRESHOLD, color='gray', lw=0.8, ls='-.')                                         # 必須条件の上側閾値
        axes[1].axhline(-TUNE_U_THRESHOLD, color='gray', lw=0.8, ls='-.')                                        # 必須条件の下側閾値
        axes[1].set_title(f"Input (max = {b['u_max']:.2f}, min = {b['u_min']:.2f})")
        axes[1].set_xlabel('Time [s]')
        axes[1].set_ylabel('Input value [PWM]')
        axes[1].legend()
        axes[1].grid(True)

        fig.tight_layout()                                                                                       # レイアウト調整
        fig.savefig(path)                                                                                        # PNGへ保存
        plt.close(fig)                                                                                           # グラフを閉じる
        self._log(f"最良結果のグラフを保存: {path}")

    # 最良のQ, Rと評価結果をCSVへ1行追記する関数
    def save_best_to_csv(self, path):                                                                            # 引数(保存先パス)
        b = self.best                                                                                            # 最良の評価結果
        row = {
            'dof': self.model['dof'], 'T': self.T, 'n_trials': self.n_trials, 'sampler': self.sampler_name,
            'eval_target': TUNE_EVAL_TARGET,
            'q1': float(b['Q'][0, 0]), 'q2': float(b['Q'][1, 1]), 'q3': float(b['Q'][2, 2]), 'r': float(b['R'][0, 0]),
            'J': b['J'], 'J_5th_order_ff': b['J_ff'], 'u_opt_max': b['u_max'], 'u_opt_min': b['u_min'],
            'ff_max': b['ff_max'], 'ff_min': b['ff_min'],
            'satisfied': b['satisfied'], 'satisfied_ff': b['satisfied_ff'], 'sat_ratio': b['sat_ratio'],
            'ff_a': b['ff_params'][0], 'ff_b': b['ff_params'][1], 'ff_c': b['ff_params'][2],
            'ff_d': b['ff_params'][3], 'ff_e': b['ff_params'][4],
            't1': b['extrema'][0], 'y1': b['extrema'][1], 't2': b['extrema'][2], 'y2': b['extrema'][3],
            'Pi': self.model['Pi'], 'Pf': self.model['Pf'], 'y0': self.model['y0'],
            'a2': self.model['sys_params'][0], 'a1': self.model['sys_params'][1],
            'a0': self.model['sys_params'][2], 'b0': self.model['sys_params'][3],
            'T1': self.model['tgt_params'][0], 'wn': self.model['tgt_params'][1],
        }
        df = pd.DataFrame([row])                                                                                 # データを横並びにする
        header = not os.path.exists(path)                                                                        # 既存ファイルがあるか判定
        df.to_csv(path, mode='a', header=header, index=False)                                                    # CSVへ追記
        self._log(f"最良のQ, RをCSVへ保存: {path}")


# ==============================================================================
# 同定結果（真のモデル）の保存・読み込み
#   ロボットを1回動かして得た同定結果をJSONへ保存しておき、以降の再探索では
#   これを読み込むことでロボットを一切動かさずに済むようにする。
# ==============================================================================

# 同定結果をJSONへ保存する関数
def save_identified_models(path, T, dt, models, initial_pot, target_pot):                                        # 引数(保存先パス, FF制御入力時間, ステップ時間, 24自由度分の同定結果, 初期姿勢, 目標値)
    payload = {
        'T': T, 'dt': dt, 'sim_time': SIM_TIME,                                                                  # 実験条件
        'initial_pot': list(initial_pot), 'target_pot': list(target_pot),                                         # 初期姿勢と目標値
        'models': [
            {
                'dof': m['dof'], 'Pi': m['Pi'], 'Pf': m['Pf'], 'y0': m['y0'],                                     # 位置情報
                'tgt_params': [float(v) for v in m['tgt_params']],                                                # 目標モデル [T1, wn]
                'sys_params': [float(v) for v in m['sys_params']],                                                # システムモデル（真のモデル）[a2, a1, a0, b0]
                'ff_params': [float(v) for v in m['ff_params']],                                                  # 実測時に印加した励振FFの5次係数
                'y_data': [float(v) for v in m['y_data']],                                                        # 実測POT値（再同定用）
            }
            for m in models
        ],
    }
    with open(path, 'w') as f:
        json.dump(payload, f, indent=2)                                                                           # JSONへ保存

# 同定結果をJSONから読み込む関数
def load_identified_models(path):                                                                                 # 引数(読み込みパス)
    with open(path, 'r') as f:
        payload = json.load(f)                                                                                     # JSONを読み込む
    T = float(payload['T'])                                                                                        # FF制御入力時間
    dt = float(payload['dt'])                                                                                      # シミュレーションステップ時間
    t_eval = np.arange(0, SIM_TIME, dt)                                                                            # シミュレーション時間配列
    t_ff = t_eval[t_eval <= T]                                                                                     # FF入力を与える時間配列
    models = []                                                                                                    # 同定結果のリスト
    for m in payload['models']:
        a, b, c, d, e = m['ff_params']                                                                              # 実測時に印加した励振FFの5次係数
        u_ff = np.zeros(len(t_eval))                                                                                # 入力ベクトルの生成
        u_ff[:len(t_ff)] = a * t_ff ** 5 + b * t_ff ** 4 + c * t_ff ** 3 + d * t_ff ** 2 + e * t_ff                 # FF入力時間だけFF入力を格納
        models.append({
            'dof': int(m['dof']), 'Pi': float(m['Pi']), 'Pf': float(m['Pf']), 'y0': float(m['y0']),
            'tgt_params': np.array(m['tgt_params'], dtype=float),
            'sys_params': np.array(m['sys_params'], dtype=float),
            'ff_params': list(m['ff_params']),
            'y_data': np.array(m['y_data'], dtype=float),
            'u_ff': u_ff,
        })
    return T, dt, models


# ==============================================================================
# ROS2 ノード
# ==============================================================================
class OptimalControlSequencer(Node):
    # ★ 探索結果グラフ出力の切り替え (True: 有効, False: 無効)
    SAVE_RESULT_PLOT = True

    # コンストラクタ
    def __init__(self, csv_path, T, n_trials=TUNE_N_TRIALS, max_iter=1, target_mode="1"):                                                       # 引数(保存csv情報, FF制御入力時間, Q,R探索回数, 外側ループ最大回数=1固定, 目標値の与え方=プリセット固定)
        super().__init__('qr_tuning_sequencer_el')                                                                                              # ROS2ノードとして登録
        self.csv_path = csv_path                                                                                                                # csv情報を格納
        self.T = T                                                                                                                              # FF制御入力時間を格納
        self.n_trials = n_trials                                                                                                                # Q,R探索回数を格納
        self.max_outer_iter = max_iter                                                                                                          # 外側ループ最大回数を格納（ロボットは1回しか動かさないので1）
        self.target_mode = target_mode                                                                                                          # 目標値の与え方のモードを格納

        self.current_outer = 0                                                                                                                  # 外側ループ回数
        self.current_inner = 0                                                                                                                  # 内側ループ回数
        self.max_inner_iter = MAX_INNER_ITER                                                                                                    # 内側ループ回数
        self.threshold_J = THRESHOLD_J                                                                                                          # 内側ループの収束判定の閾値

        self.initial_stabilize_time = INIT_WAIT_TIME                                                                                            # 初期姿勢への移動後の待機時間
        self.data_collection_time = SIM_TIME                                                                                                    # データ取得時間
        self.dt = SIM_DT                                                                                                                        # シミュレーションステップ時間

        self.state = "INIT_ROBOT"                                                                                                               # ロボット状態（初期位置へ送る状態）

        # 出力ファイルのパスを作成（選択したCSVパスを基準にする）
        base_path = os.path.splitext(self.csv_path)[0]                                          # csvの拡張子を除いたパス
        self.model_path = base_path + "_identified_models.json"                                 # 同定結果（真のモデル）の保存先
        self.trials_csv_path = base_path + "_qr_trials.csv"                                     # 全トライアル結果の保存先
        self.plot_path = base_path + "_qr_best.png"                                             # 最良結果のグラフの保存先

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
                        f"a={a:.1e}, b={b:.1e}, c={c:.1e}, d={d:.1e}, e={e:.1e} | "
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
                "=== システム同定用データ取得（ロボットを動かすのはこの1回だけ） ==="
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
            self.get_logger().info("Q, R の自動チューニングが正常終了しました。")
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

            # 24自由度それぞれに対して、目標モデル同定・システムモデル同定を実行する
            #   ここで同定したシステムモデルを、以降「真のモデル」とみなす（ロボットはもう動かさない）
            solver = MathematicalSolver(self.T, self.dt)                                                                # MathematicalSolverクラスの生成
            N_samples = int(SIM_TIME / self.dt)                                                                         # サンプル数の計算

            models = []                                                                                                 # 24自由度分の同定結果の空リスト

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

                # 2. System Model ID（＝真のモデル）
                a, b, c, d, e = self.current_ff_matrix[dof_idx]                                             # 現在のDOFのFFパラメータ（励振用の初期FF）を取り出す
                t_ff = solver.t_eval[solver.t_eval <= self.T]                                               # 0秒～FF入力時間までの時間配列を取り出す
                u_ff = np.zeros(N_samples)                                                                  # 入力ベクトルの生成
                u_ff[:len(t_ff)] = a * t_ff ** 5 + b * t_ff ** 4 + c * t_ff ** 3 + d * t_ff ** 2 + e * t_ff # FF入力時間だけ、そのときのFF入力を格納する

                sys_params = solver.fit_system_model(y_shifted, u_ff, y0)                                    # システムモデルの同定をして、パラメータを取得

                # 同定結果を保存（この1回の実測データから得られた「真のモデル」）
                models.append({
                    'dof': dof_idx + 1,                                                                     # DOF番号（1始まり）
                    'Pi': float(Pi), 'Pf': float(Pf), 'y0': float(y0),                                      # 初期位置・目標位置・初期偏差
                    'tgt_params': np.asarray(tgt_params, dtype=float),                                      # 目標モデル [T1, wn]
                    'sys_params': np.asarray(sys_params, dtype=float),                                      # 真のモデル [a2, a1, a0, b0]
                    'ff_params': [float(v) for v in self.current_ff_matrix[dof_idx]],                        # 実測時に印加した励振FFの5次係数
                    'u_ff': u_ff,                                                                           # 実測時に印加した励振FF入力
                    'y_data': raw_y,                                                                        # 実測POT値
                })

                self.get_logger().info(                                                                     # 同定結果のログ出力
                    f"  DOF {dof_idx + 1:02d}: 目標モデル T1={tgt_params[0]:.4f}, wn={tgt_params[1]:.4f} | "
                    f"真のモデル a2={sys_params[0]:.4f}, a1={sys_params[1]:.4f}, a0={sys_params[2]:.4f}, b0={sys_params[3]:.4f}"
                )

            # 同定結果をJSONへ保存する（再探索時はこのJSONを読み込めばロボットを動かす必要がない）
            save_identified_models(self.model_path, self.T, self.dt, models, self.initial_pot, self.target_pot)          # 同定結果を保存
            self.get_logger().info(f"同定結果（真のモデル）を保存: {self.model_path}")                                     # ログ出力

            # ---------------------------------------------------------------------
            # ここから先はロボットを一切動かさず、シミュレーションのみで Q, R を探索する
            # ---------------------------------------------------------------------
            model = models[TUNE_TARGET_DOF - 1]                                                                         # 対象DOF（DOF4）の同定済みモデルを取り出す
            tuner = QRTuner(self.T, self.dt, model, n_trials=self.n_trials, logger=self.get_logger())                    # Q,R探索クラスを生成
            best = tuner.run()                                                                                          # 探索を実行して最良のQ,Rを取得

            if best is not None:                                                                                        # 探索が成功した場合は結果を保存
                tuner.save_best_to_csv(self.csv_path)                                                                   # 最良のQ,RをCSVへ追記
                tuner.save_trials_csv(self.trials_csv_path)                                                              # 全トライアル結果をCSVへ保存
                if self.SAVE_RESULT_PLOT:                                                                                # グラフ出力判定
                    tuner.save_result_plot(self.plot_path)                                                               # 最良結果のグラフを保存

            self.state = "FINISHED"                                                                                     # 終了状態に変更（ロボットはホームポジションへ戻す）
            self.state_start_time = self.get_clock().now()                                                              # 現在時刻を取得

        except Exception as exc:                                                                                                                    # 何かエラーが出たときの処理
            self.get_logger().error(f"最適化パイプライン異常: {exc}\n{traceback.format_exc()}")
            self.state = "FINISHED"
            self.state_start_time = self.get_clock().now()


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


# Q,R探索回数を決める関数（コマンドライン引数 --trials N が最優先、無ければキーボード入力）
def resolve_n_trials(argv=None):                                                                    # 引数(コマンドライン引数リスト)
    argv = list(sys.argv[1:]) if argv is None else list(argv)                                       # コマンドライン引数を取得
    for i, arg in enumerate(argv):                                                                  # 引数を順に確認
        if arg in ('--trials', '-n') and i + 1 < len(argv):                                         # 「--trials 200」形式
            try:
                return max(1, int(argv[i + 1]))                                                     # 探索回数を返す
            except ValueError:
                break
        if arg.startswith('--trials='):                                                             # 「--trials=200」形式
            try:
                return max(1, int(arg.split('=', 1)[1]))                                            # 探索回数を返す
            except ValueError:
                break
    print(f"【Q,R探索回数を入力してください（未入力なら既定値 {TUNE_N_TRIALS}）】")                        # 入力を促す
    text = input("> ").strip()                                                                      # 探索回数の入力
    return max(1, int(text)) if text else TUNE_N_TRIALS                                             # 探索回数を返す


# 保存済みの同定結果を読み込み、ロボットを動かさずQ,Rを探索する関数
def run_tuning_from_saved_models(n_trials):                                                         # 引数(Q,R探索回数)
    root = tk.Tk()                                                                                  # Tkinterを起動
    root.withdraw()                                                                                 # 親ウィンドウは表示しない
    root.attributes("-topmost", True)                                                                # ダイアログを最前面に表示

    json_path = filedialog.askopenfilename(                                                          # ファイル選択ダイアログを表示
        title="同定結果JSON（*_identified_models.json）を選択してください",
        filetypes=[("JSON Files", "*.json")],
    )
    if not json_path:
        print("ファイル未選択のため終了します。")
        sys.exit(1)

    T, dt, models = load_identified_models(json_path)                                                # 同定結果（真のモデル）を読み込む
    print(f"【同定結果を読み込みました】{json_path}  (T={T}, dt={dt}, DOF数={len(models)})")

    tuner = QRTuner(T, dt, models[TUNE_TARGET_DOF - 1], n_trials=n_trials)                           # Q,R探索クラスを生成（対象DOFの同定済みモデルを使用）
    best = tuner.run()                                                                               # 探索を実行

    if best is not None:                                                                             # 探索が成功した場合は結果を保存
        base_path = os.path.splitext(json_path)[0]                                                   # JSONの拡張子を除いたパス
        tuner.save_best_to_csv(base_path + "_qr_best.csv")                                           # 最良のQ,RをCSVへ追記
        tuner.save_trials_csv(base_path + "_qr_trials.csv")                                          # 全トライアル結果をCSVへ保存
        tuner.save_result_plot(base_path + "_qr_best.png")                                           # 最良結果のグラフを保存


def main(args=None):
    print("========================================================================")
    print("  評価関数の重み Q, R の自動チューニング (Optuna)")
    print("  1: ロボットを1回だけ動かして同定し、そのモデルでQ,Rを探索する")
    print("  2: 保存済みの同定結果(JSON)を読み込んでQ,Rのみ探索する（ロボットは動かさない）")
    print("========================================================================")
    run_mode = input("モードを選択してください (1 または 2): ").strip()                                    # 実行モードの入力
    while run_mode not in ['1', '2']:
        run_mode = input("無効な入力です。1 または 2 を入力してください: ").strip()

    n_trials = resolve_n_trials()                                                                    # Q,R探索回数を決定
    print(f"【Q,R探索回数】 {n_trials}")

    # 保存済み同定結果を使う場合（ロボット・ROS2は不要）
    if run_mode == '2':
        run_tuning_from_saved_models(n_trials)                                                       # 探索のみ実行
        return

    csv_path = resolve_csv_file()                                                                    # 結果保存先CSVを決定
    print(f"【保存先CSVパス】 {csv_path}")

    print("【FF制御時間 T を入力してください】")
    T = float(input("> "))

    rclpy.init(args=args)
    node = OptimalControlSequencer(csv_path, T, n_trials)                                             # ロボットを1回動かして同定→Q,R探索
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
