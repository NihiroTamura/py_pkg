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
from scipy.signal import cont2discrete                          # 連続時間→離散時間(ZOH)変換

import tkinter as tk                                            # GUIライブラリ
from tkinter import filedialog                                  # GUIでフォルダ選択

warnings.simplefilter('ignore', RuntimeWarning)                 # RuntimeWarningを非表示
np.seterr(all='ignore')                                         # Numpyのエラーを無地

# ==============================================================================
# 評価関数の重み行列（チューニング要素）
#   コスト関数 J = Σ (xᵀQx + uᵀRu) の重み（x は目標軌道との誤差状態、u は制御入力）
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
# 離散時間オイラー・ラグランジュ（勾配）法のパラメータ（チューニング要素）
# ==============================================================================
EL_MAX_ITER = 200           # 勾配法の最大反復回数
EL_EPS = 1e-3               # 勾配ノルム Σ||∂H/∂u||² の収束判定閾値
EL_ALPHA = 1.0             # 勾配降下ステップ幅 α の初期値（バックトラッキングで調整）
EL_LS_MAX = 30             # ステップ幅バックトラッキングの最大試行回数


# ==============================================================================
# 数学ソルバー (System ID & Euler-Lagrange Optimal Control)
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
    # シミュレーション
    # ------------------------------------------------------------------

    # 目標モデルの自由応答を計算する関数
    def simulate_unforced(self, t_array, a2, a1, a0, y0):           # 引数(シミュレーション時間, 3次遅れ系の係数 a2・a1・a0, 初期偏差)
        """目標モデル（入力なし）のシミュレーション"""
        N = len(t_array)                                            # シミュレーション時間サンプル数を取得
        x = np.array([y0, a2 * y0, a1 * y0], dtype=float)           # 状態変数の初期化 x0 = y ( y(0) = y0 ), x1 = dy + a2*y ( dy(0) = 0 ), x2 = ddy + a2*dy + a1*y ( ddy(0) = 0 ),
        y_traj = np.zeros(N)                                        # 出力を保存する配列
        dy_traj = np.zeros(N)                                       # 出力の1階微分を保存する配列
        ddy_traj = np.zeros(N)                                      # 出力の2階微分を保存する配列
        dddy_traj = np.zeros(N)                                     # 出力の3階微分を保存する配列
        for i in range(N):                                          # シミュレーション時間ごとにオイラー積分
            # 可観測正準系の状態空間表現          
            dx0 = x[1] - a2 * x[0]
            dx1 = x[2] - a1 * x[0]
            dx2 = -a0 * x[0]
            
            y_traj[i] = x[0]                                    # 出力の保存
            dy_traj[i] = dx0                                    # 速度の保存
            ddy_traj[i] = dx1 - a2 * dx0                        # 加速度の保存
            dddy_traj[i] = dx2 - a1 * dx0 - a2 * ddy_traj[i]    # 加加速度の保存
            
            # 状態変数の更新
            x[0] += dx0 * self.dt
            x[1] += dx1 * self.dt
            x[2] += dx2 * self.dt
        return y_traj, dy_traj, ddy_traj, dddy_traj

    # システムモデルの応答(FF入力)を計算する関数
    def simulate_forced(self, t_array, a2, a1, a0, b0, u_array, y0):    # 引数(シミュレーション時間, 3次遅れ系の係数 a2・a1・a0・b0, FF入力, 初期偏差)
        """システムモデル（FF入力あり）のシミュレーション"""
        N = len(t_array)                                                # シミュレーション時間サンプル数を取得
        x = np.array([y0, a2 * y0, a1 * y0], dtype=float)               # 状態変数の初期化 x0 = y ( y(0) = y0 ), x1 = dy + a2*y ( dy(0) = 0 ), x2 = ddy + a2*dy + a1*y ( ddy(0) = 0 ),
        y_traj = np.zeros(N)                                            # 出力を保存する配列
        dy_traj = np.zeros(N)                                           # 出力の1階微分を保存する配列
        ddy_traj = np.zeros(N)                                          # 出力の2階微分を保存する配列
        for i in range(N):                                              # シミュレーション時間ごとにオイラー積分
            # 可観測正準系の状態空間表現
            dx0 = x[1] - a2 * x[0]
            dx1 = x[2] - a1 * x[0]
            dx2 = b0 * u_array[i] - a0 * x[0]
            
            y_traj[i] = x[0]                        # 出力の保存
            dy_traj[i] = dx0                        # 速度の保存
            ddy_traj[i] = dx1 - a2 * dx0            # 加速度の保存
            
            # 状態変数の更新
            x[0] += dx0 * self.dt
            x[1] += dx1 * self.dt
            x[2] += dx2 * self.dt
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

        Nelder-Mead を用いず、勾配ベースの信頼領域法（Levenberg-Marquardt 系の
        scipy.optimize.least_squares, method='trf'）で残差二乗和を最小化する。
        """
        # 各時刻の残差 r(k) = y_sim(k) - y_data(k) を返す関数（least_squares は Σr² を最小化）
        def residuals(p):
            T1, wn = p                                                              # 最適化変数の取り出し
            a2, a1, a0 = self._target_coeffs(T1, wn)                                # 減衰係数1固定の3次遅れ系の係数を計算
            y_sim, _, _, _ = self.simulate_unforced(self.t_eval, a2, a1, a0, y0)    # 目標モデルの自由応答を計算
            return y_sim - y_data                                                   # 残差ベクトルを返す

        res = scipy.optimize.least_squares(                                         # 残差二乗和が最小になる変数[T1, wn]を最適化する
            residuals, x0=[0.1, 10.0],
            bounds=([1e-6, 1e-6], [np.inf, np.inf]),                                # T1>0, wn>0（負の極を排除）
            method='trf',
        )
        T1, wn = res.x                                                                      # 最適変数を取り出す
        return T1, wn

    # システムモデル同定関数
    def fit_system_model(self, y_data, u_ff, y0):                                                   # 引数(実測データ, FF制御入力, 初期偏差)
        """システムモデル同定: b0 / (s^3 + a2*s^2 + a1*s + a0)

        目標モデルと同じく勾配ベースの信頼領域法（least_squares, method='trf'）で
        残差二乗和を最小化する。
        """
        # 各時刻の残差 r(k) = y_sim(k) - y_data(k) を返す関数
        def residuals(p):
            a2, a1, a0, b0 = p                                                          # 最適化変数の取り出し
            y_sim, _, _ = self.simulate_forced(self.t_eval, a2, a1, a0, b0, u_ff, y0)   # システムモデルの応答を計算
            return y_sim - y_data                                                       # 残差ベクトルを返す

        res = scipy.optimize.least_squares(                                             # 残差二乗和が最小になる変数[a2, a1, a0, b0]を最適化する
            residuals, x0=[10.0, 100.0, 1000.0, 1000.0],
            bounds=([1e-6, 1e-6, 1e-6, -np.inf], [np.inf, np.inf, np.inf, np.inf]),     # a2,a1,a0>0（負の極を排除）, b0は符号自由
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

    # ------------------------------------------------------------------
    # 離散時間オイラー・ラグランジュ（勾配）法による最適制御 + 5次多項式フィット
    # ------------------------------------------------------------------
    def calculate_el_ff(self, target_params, sys_params, u_ff, y0):                                          # 引数(目標モデルのパラメータ[T1, wn], システムモデルのパラメータ[a2, a1, a0, b0], FF制御入力, 初期偏差)
        """
        離散時間オイラー・ラグランジュ（随伴／勾配）法で最適制御入力を計算し、
        5次多項式 FF = a*t^5 + ... + e*t にフィットする。

        目的：システムモデルの軌道を、入力 u=0 で生成した目標軌道 x_tgt へ追従させる。
          入力ホライズン : u(k) は [0,T]（k=0..N_ff-1）のみ最適化し、それ以降は0
          評価ホライズン : コスト J はシミュレーション全体（5秒, k=0..M-1）で評価する

        ● 離散状態方程式（可制御正準形・前進オイラー）
            システム : x_sys(k+1) = A_sys x_sys(k) + B_sys u(k)
            目標(u=0): x_tgt(k+1) = A_tgt x_tgt(k)

        ● 誤差状態と誤差ダイナミクス（e = x_sys - x_tgt を状態変数とする）
            e(k) = x_sys(k) - x_tgt(k),  e(0) = 0
            e(k+1) = x_sys(k+1) - x_tgt(k+1)
                   = A_sys( e(k) + x_tgt(k) ) + B_sys u(k) - A_tgt x_tgt(k)
                   = A_sys e(k) + B_sys u(k) + d(k),   d(k) = (A_sys - A_tgt) x_tgt(k)  （既知入力, u_tgt=0）

        ● ラグランジュ関数（終端コストなし）／ハミルトニアン
            J = Σ_{k=0}^{M-1} L(k),   L(k) = e(k)ᵀ Q e(k) + R u(k)²
            H(k) = L(k) + λ(k+1)ᵀ [ A_sys e(k) + B_sys u(k) + d(k) ]

        ● 随伴方程式（∂d/∂e = 0 より既知入力 d は随伴に現れない）
            λ(M) = 0
            λ(k) = ∂H/∂e(k) = 2 Q e(k) + A_sysᵀ λ(k+1)

        ● 勾配（入力ホライズン k=0..N_ff-1）
            ∂H/∂u(k) = 2 R u(k) + B_sysᵀ λ(k+1)

        解法（[0,T] の入力列 u(k) が未知変数、k≥N_ff では u(k)=0）：
          1. u(k) を初期化する
          2. 誤差ダイナミクス e(k+1)=A_sys e(k)+B_sys u(k)+d(k) を順方向に積分（5秒全体）
          3. 終端随伴変数 λ(M) = 0
          4. 随伴方程式 λ(k)=2 Q e(k)+A_sysᵀ λ(k+1) を逆方向に計算（5秒全体）
          5. 勾配 ∂H/∂u(k)=2 R u(k)+B_sysᵀ λ(k+1) を計算（k=0..N_ff-1）
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
        # 同定結果から連続時間状態方程式（可制御正準形）を構築し、
        # cont2discrete() による ZOH（Zero-Order Hold）厳密離散化で離散時間モデルを得る。
        #   連続系      : ẋ = A x + B u
        #     A = [[0,1,0],[0,0,1],[-a0,-a1,-a2]],  B_sys = [0,0,b0]ᵀ
        #   離散系(ZOH) : x(k+1) = A_d x(k) + B_d u(k)
        #     A_d = expm(A*dt),  B_d = (∫₀^{dt} expm(A τ) dτ) B
        #   システム : x_sys(k+1) = A_sys x_sys(k) + B_sys u(k)
        #   目標(u=0): x_tgt(k+1) = A_tgt x_tgt(k)
        # -----------------------------------------------------------
        C_dummy = np.zeros((1, n_x))                                                                    # 出力行列（状態フィードバックのためダミー）
        D_dummy = np.zeros((1, 1))                                                                      # 直達行列（ダミー）

        # システムモデルの連続時間状態方程式 → ZOH離散化
        A_sys_c = np.array([[0.0,     1.0,     0.0],
                            [0.0,     0.0,     1.0],
                            [-a0_sys, -a1_sys, -a2_sys]])                                               # 連続時間 A（可制御正準形）
        B_sys_c = np.array([[0.0], [0.0], [b0_sys]])                                                    # 連続時間 B
        A_sys, B_sys_mat, _, _, _ = cont2discrete((A_sys_c, B_sys_c, C_dummy, D_dummy), dt, method="zoh")   # ZOH離散化で A_sys, B_sys を取得
        B_sys = B_sys_mat.flatten()                                                                     # 入力ベクトル (3,) へ整形

        # 目標モデル(u=0)の連続時間状態方程式 → ZOH離散化（A_tgt のみ使用、B はダミー）
        A_tgt_c = np.array([[0.0,     1.0,     0.0],
                            [0.0,     0.0,     1.0],
                            [-a0_tgt, -a1_tgt, -a2_tgt]])                                               # 連続時間 A（可制御正準形）
        B_tgt_c = np.zeros((n_x, 1))                                                                    # 連続時間 B（目標は u=0 なのでダミー）
        A_tgt, _, _, _, _ = cont2discrete((A_tgt_c, B_tgt_c, C_dummy, D_dummy), dt, method="zoh")       # ZOH離散化で A_tgt = expm(A_tgt_c*dt) を取得

        # 目標軌道 x_tgt(k)（u=0 の目標モデル自由応答）を A_tgt の再帰で生成
        #   x_tgt(0) = [y0, 0, 0],  x_tgt(k+1) = A_tgt x_tgt(k)
        x_tgt = np.zeros((M, n_x))                                                                      # 目標状態列 (M, 3)
        x_tgt[0] = np.array([y0, 0.0, 0.0])                                                             # 初期状態
        for k in range(M - 1):                                                                          # 目標モデルの自由応答を逐次計算
            x_tgt[k + 1] = A_tgt @ x_tgt[k]
        y_tgt = x_tgt[:, 0]                                                                             # 目標出力（位置成分）

        # 誤差ダイナミクスの既知入力 d(k) = (A_sys - A_tgt) x_tgt(k)（forward でのみ扱う）
        D = x_tgt @ (A_sys - A_tgt).T                                                                   # D[k] = (A_sys - A_tgt) x_tgt(k) → 形状 (M, 3)

        Q = self.Q                                                                                      # 状態誤差の重み行列 (3x3)
        R = float(self.R[0, 0])                                                                         # 入力の重みスカラー

        # ---- 手順2: 誤差状態 e を順方向に積分する関数（e(0) = x_sys(0)-x_tgt(0) = 0） ----
        #   誤差ダイナミクス e(k+1) = A_sys e(k) + B_sys u(k) + d(k)
        #   入力は [0,T] (k<N_ff) のみ与え、それ以降は0（自由応答）とする。
        def forward(u_seq):
            e = np.zeros((M + 1, n_x))                                                                  # 誤差状態列 e(0..M)（e(0)=0）
            for k in range(M):                                                                          # k=0..M-1 を順方向に更新
                uk = u_seq[k] if k < N_ff else 0.0                                                      # 入力は [0,T] のみ、それ以降は0
                e[k + 1] = A_sys @ e[k] + B_sys * uk + D[k]                                             # e(k+1)=A_sys e(k)+B_sys u(k)+d(k)
            return e

        # ---- コスト関数 J = Σ_{k=0}^{M-1} ( e(k)ᵀ Q e(k) + R u(k)² )（5秒全体で評価） ----
        def cost(e_traj, u_seq):
            e = e_traj[:M]                                                                              # 誤差状態 e(0..M-1)
            return float(np.einsum('ki,ij,kj->', e, Q, e) + R * np.sum(u_seq ** 2))                    # 状態コスト（5秒全体） + 入力コスト（[0,T]）

        # ---- 手順3,4: 随伴方程式を逆方向に計算し λ(1..M) を求める関数（5秒全体） ----
        #   ∂d/∂e = 0 より既知入力 d は随伴方程式に現れない。
        def backward(e_traj):
            lam = np.zeros((M + 1, n_x))                                                                # 随伴変数列 λ(0..M)
            # 手順3: 終端条件 λ(M) = 0（終端コストなし） → lam[M] は 0 のまま
            for k in range(M - 1, 0, -1):                                                               # 手順4: k=M-1..1 を逆方向に更新
                lam[k] = 2.0 * (Q @ e_traj[k]) + A_sys.T @ lam[k + 1]                                   # λ(k) = 2 Q e(k) + A_sysᵀ λ(k+1)
            return lam

        # ---- 手順5: 勾配 ∂H/∂u(k) = 2 R u(k) + B_sysᵀ λ(k+1)（入力ホライズン k=0..N_ff-1 のみ） ----
        def gradient(u_seq, lam):
            return 2.0 * R * u_seq + lam[1:N_ff + 1] @ B_sys                                            # [0,T] の各時刻 k=0..N_ff-1 の勾配ベクトル

        # ---- 手順1: 入力列 u(k) を初期化（零入力から開始） ----
        u = np.zeros(N_ff)                                                                              # 決定変数 u(0..N_ff-1)（[0,T] のみ）

        # ---- 手順2〜7: 勾配降下の反復 ----
        for _ in range(EL_MAX_ITER):
            e_traj = forward(u)                                                                         # 手順2: 誤差状態を順方向積分
            lam = backward(e_traj)                                                                      # 手順3,4: 随伴方程式を逆方向計算
            grad = gradient(u, lam)                                                                     # 手順5: 勾配 ∂H/∂u を計算

            if np.sum(grad ** 2) < EL_EPS:                                                              # 手順6: Σ||∂H/∂u||² < ε なら収束とみなし終了
                break

            # 手順7: u ← clip(u - α ∂H/∂u, -255, 255)
            #   固定 α では不安定になりうるため、コストが減少する α をバックトラッキングで選ぶ
            J0 = cost(e_traj, u)                                                                        # 現在のコスト
            step = EL_ALPHA                                                                             # ステップ幅 α の初期値
            u_new = u                                                                                   # 更新結果の初期化
            for _ls in range(EL_LS_MAX):
                u_cand = np.clip(u - step * grad, -255.0, 255.0)                                        # PWM制約 -255≤u≤255 を満たすようclip
                if cost(forward(u_cand), u_cand) < J0:                                                  # コストが減少したら採用
                    u_new = u_cand
                    break
                step *= 0.5                                                                             # 減少しなければステップ幅を半分にして再試行
            else:
                break                                                                                   # どのステップ幅でも改善しなければ収束とみなし終了
            u = u_new                                                                                   # 入力列を更新

        # 最適入力をシミュレーション時間全体の配列へ格納（FF入力区間以外は0）
        u_opt = np.zeros(len(self.t_eval))                                                              # 最適入力を保存する配列
        u_opt[:N_ff] = u                                                                                # FF入力区間 (0..N_ff-1) の最適入力を格納

        # 5次多項式フィット（必須条件: f(0)=0, f(T)=0, 0<t<T で極値ちょうど2個）
        t_ff = self.t_eval[self.t_eval <= self.T]                                                       # FF入力を与える時間だけ取り出す
        u_opt_ff = u_opt[: len(t_ff)]                                                                   # FF入力を与える区間だけの最適入力を取り出す

        # 5次関数を定義する関数（端点条件 f(0)=0, f(T)=0 を構造的に満たす）
        def poly(t, a, b, c, d):                                                                        # 引数(時間, 5次関数パラメータa・b・c・d)
            e = -(a * self.T ** 4 + b * self.T ** 3 + c * self.T ** 2 + d * self.T)     # f(T)=0 となるように e を決定
            return a * t ** 5 + b * t ** 4 + c * t ** 3 + d * t ** 2 + e * t            # 5次関数FF入力値を返す（f(0)=0 は定数項なしで保証）

        # 0<t<T における極値（f'(t)=0 の実根）を小さい順に返す関数
        def interior_extrema(a, b, c, d, e):                                                            # 引数(5次関数パラメータa・b・c・d・e)
            droots = np.roots([5 * a, 4 * b, 3 * c, 2 * d, e])                                          # f'(t)=0 の根を計算
            return sorted(r.real for r in droots if abs(r.imag) < 1e-6 and 0 < r.real < self.T)         # (0,T)内の実根のみを小さい順に返す

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

        """
        # 【必須条件の検証】5次フィットが「0<t<T で極値ちょうど2個」を満たさなければ採用しない。
        #   満たさない場合は、自由度4のフィット形式
        #       f(t) = t (t - T) (A t^3 + B t^2 + C t + D)
        #   へ u_opt を再フィットして代用する。因子 t, (t-T) により f(0)=0, f(T)=0 を必ず満たす。
        #
        #   ＜なぜこの形式だと u_opt のピーク値・ピーク位置を維持しやすいか＞
        #   旧来の C·t(t-r)(t-T) は自由度2しかなく、u_opt の2つのピークの位置と高さを同時に
        #   合わせられずずれが大きかった。本形式は3次因子 (A,B,C,D) の自由度4を持つため、
        #   2ピークの位置・値を同時に再現できる。さらに f は (A,B,C,D) について線形なので
        #   二乗誤差 J1 の大域最小を最小二乗で一発計算でき、加えて f(tpi)=vpi・f'(tpi)=0 の
        #   ピーク一致条件を評価関数に入れることで、ピーク位置・値を明示的に保持できる。
        if len(interior_extrema(a, b, c, d, e)) != 2:                                                   # 極値がちょうど2個でなければ再フィット
            # 新フィット形式とその微分（局所定義。外側の poly / interior_extrema は変更しない）
            def poly_fac(t, A, B, C, D):                                                                # f(t) = t(t-T)(A t^3+B t^2+C t+D)
                return t * (t - self.T) * (A * t ** 3 + B * t ** 2 + C * t + D)

            def poly_fac_deriv(t, A, B, C, D):                                                          # f'(t) = (2t-T)g + t(t-T)g',  g=A t^3+B t^2+C t+D
                g = A * t ** 3 + B * t ** 2 + C * t + D
                dg = 3 * A * t ** 2 + 2 * B * t + C
                return (2 * t - self.T) * g + t * (t - self.T) * dg

            def fac_to_quintic(A, B, C, D):                                                             # 因子形式 → 標準5次係数 (a,b,c,d,e)
                T = self.T
                return A, B - A * T, C - B * T, D - C * T, -D * T

            # u_opt の2つのピーク（位置 tp・値 vp）を検出する
            du = np.diff(u_opt_ff)                                                                       # 隣接差分
            ext = [i for i in range(1, len(u_opt_ff) - 1) if du[i - 1] * du[i] < 0.0]                    # 内部の局所極値インデックス
            if len(ext) >= 2:
                ext = sorted(sorted(ext, key=lambda i: abs(u_opt_ff[i]), reverse=True)[:2])            # |値|が大きい順に顕著な2つ
            elif len(ext) == 1:
                ext = sorted({ext[0], int(np.argmax(np.abs(u_opt_ff)))})                                # もう一方は|値|最大点
            else:
                ext = sorted({int(np.argmax(u_opt_ff)), int(np.argmin(u_opt_ff))})                      # 極値が無ければ最大・最小点
            while len(ext) < 2:                                                                          # 念のため2点を確保
                ext.append(min(len(u_opt_ff) - 1, (ext[-1] if ext else 0) + 1))
            tp1, vp1 = t_ff[ext[0]], u_opt_ff[ext[0]]
            tp2, vp2 = t_ff[ext[1]], u_opt_ff[ext[1]]

            # f は (A,B,C,D) について線形 → 基底 φk(t)=t(t-T)t^k を並べた計画行列
            Phi = np.stack([
                t_ff * (t_ff - self.T) * t_ff ** 3,                                                     # A の基底 φ3
                t_ff * (t_ff - self.T) * t_ff ** 2,                                                     # B の基底 φ2
                t_ff * (t_ff - self.T) * t_ff,                                                           # C の基底 φ1
                t_ff * (t_ff - self.T),                                                                  # D の基底 φ0
            ], axis=1)

            W_PWM = 1e6                                                                                  # PWM制約ペナルティの重み
            W_PEAK_VAL = 5.0                                                                             # ピーク値一致の重み（J1と同スケール）
            W_PEAK_POS = 1e-2                                                                            # ピーク位置一致（傾き0）の重み

            # 評価関数: J1 + PWM制約 + 極値数制約 + ピーク位置・値の一致
            def fac_loss(p):
                A, B, C, D = p
                f = poly_fac(t_ff, A, B, C, D)
                J1 = np.sum((f - u_opt_ff) ** 2)                                                        # 条件1: u_optとの二乗誤差
                penalty_pwm = (                                                                        # 条件4: PWM制約を最適化時の制約(罰則)として扱う
                    np.sum(np.maximum(0.0, f - 255.0) ** 2)
                    + np.sum(np.maximum(0.0, -255.0 - f) ** 2)
                )
                sc = np.count_nonzero(np.diff(poly_fac_deriv(t_ff, A, B, C, D) > 0))                    # 条件2: f'の符号反転回数=極値数
                penalty_extrema = abs(sc - 2) * 1e7                                                     #        極値が2個になるようペナルティ
                penalty_peak = (                                                                        # 条件3: ピーク位置(傾き0)・値の一致
                    W_PEAK_POS * (poly_fac_deriv(tp1, A, B, C, D) ** 2 + poly_fac_deriv(tp2, A, B, C, D) ** 2)   # f'(tpi)=0 → tpi を極値に
                    + W_PEAK_VAL * ((poly_fac(tp1, A, B, C, D) - vp1) ** 2 + (poly_fac(tp2, A, B, C, D) - vp2) ** 2)  # f(tpi)=vpi
                )
                return J1 + W_PWM * penalty_pwm + penalty_extrema + penalty_peak

            # 「(0,T)内の極値がちょうど2個」かつ「PWM制約 |f|<=255」を満たすか（外側 interior_extrema を再利用）
            def fac_valid(A, B, C, D):
                if len(interior_extrema(*fac_to_quintic(A, B, C, D))) != 2:                              # 条件2: 極値ちょうど2個
                    return False
                return float(np.max(np.abs(poly_fac(t_ff, A, B, C, D)))) <= 255.0 + 1e-6                 # 条件4: PWM制約

            # 2つのピークに極値をピン留めする初期値 [f(tpi)=vpi, f'(tpi)=0 の4元1次連立を解く]
            def pin_guess():
                M, rhs = [], []
                for tp, vp in [(tp1, vp1), (tp2, vp2)]:
                    M.append([tp * (tp - self.T) * tp ** 3, tp * (tp - self.T) * tp ** 2,
                              tp * (tp - self.T) * tp, tp * (tp - self.T)])                              # f(tp)=vp の行
                    rhs.append(vp)
                for tp, _v in [(tp1, vp1), (tp2, vp2)]:
                    M.append([5 * tp ** 4 - 4 * self.T * tp ** 3, 4 * tp ** 3 - 3 * self.T * tp ** 2,
                              3 * tp ** 2 - 2 * self.T * tp, 2 * tp - self.T])                           # f'(tp)=0 の行（φk'）
                    rhs.append(0.0)
                try:
                    sol = np.linalg.solve(np.array(M), np.array(rhs))                                   # 4元1次連立を解く
                    return sol if np.all(np.isfinite(sol)) else None
                except np.linalg.LinAlgError:
                    return None

            # 有効な候補（J1, (A,B,C,D)）を収集する
            candidates = []

            def consider(p):
                A, B, C, D = float(p[0]), float(p[1]), float(p[2]), float(p[3])
                if fac_valid(A, B, C, D):
                    candidates.append((float(np.sum((poly_fac(t_ff, A, B, C, D) - u_opt_ff) ** 2)), (A, B, C, D)))

            p_ls = np.linalg.lstsq(Phi, u_opt_ff, rcond=None)[0]                                         # (1) J1大域最小（最小二乗）
            consider(p_ls)
            starts = [p_ls]
            p_pin = pin_guess()                                                                          # (2) ピークピン留め初期値
            if p_pin is not None:
                starts.append(p_pin)
                consider(p_pin)
            for p0 in starts:                                                                            # 罰則付きで最適化して調整
                r = scipy.optimize.minimize(fac_loss, p0, method='Nelder-Mead',
                                            options={'maxiter': 3000, 'xatol': 1e-9, 'fatol': 1e-6})
                consider(r.x)

            if candidates:                                                                               # 有効解のうち J1 最小を採用
                candidates.sort(key=lambda cc: cc[0])
                A, B, C, D = candidates[0][1]
            else:
                # 最終手段: 極値ちょうど2個を構造的に保証する退化形 f = C·t(t-r)(t-T)（本形式で A=B=0）
                best_res, best_C, best_r = np.inf, 0.0, self.T / 2.0
                for r in np.linspace(0.05 * self.T, 0.95 * self.T, 91):                                  # 中間根 r を (0,T) 内で走査
                    phi = t_ff * (t_ff - r) * (t_ff - self.T)                                            # 基底 t(t-r)(t-T)
                    denom = float(phi @ phi)
                    if denom <= 0.0:
                        continue
                    Cc = float(phi @ u_opt_ff) / denom                                                   # 振幅 C の最小二乗解
                    resid = float(np.sum((Cc * phi - u_opt_ff) ** 2))
                    if resid < best_res:
                        best_res, best_C, best_r = resid, Cc, r
                peak = float(np.max(np.abs(best_C * (t_ff * (t_ff - best_r) * (t_ff - self.T)))))
                if peak > 255.0:                                                                         # 最終手段のみPWM超過時に振幅縮小
                    best_C *= 255.0 / peak
                A, B, C, D = 0.0, 0.0, best_C, -best_C * best_r                                          # 因子形式へ

            a, b, c, d, e = fac_to_quintic(A, B, C, D)                                                   # 標準5次係数へ変換して採用

        """
        # 極値 (t1,y1), (t2,y2) を抽出（上記処理により (0,T) 内の極値はちょうど2個）
        real_roots = interior_extrema(a, b, c, d, e)                                                    # (0,T)内の極値を取得
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
        super().__init__('optimal_control_sequencer_lqr')                                                                                       # ROS2ノードとして登録
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

        self.current_ff_matrix = [[0.0, 0.0, 0.0, 0.0, 0.0] for _ in range(24)]                                                                 # 24自由度分のFF係数
        self.best_ff_matrix = None                                                                                                              # 今までの最良FF係数
        self.best_extrema = None                                                                                                                # 今までの最良FFの極値
        self.min_J_sum = float('inf')                                                                                                           # 評価関数Jの初期化
        self.best_debug_data = None                                                                                                             # デバック情報

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

            # 24自由度それぞれに対して、目標モデル同定・システムモデル同定・オイラー・ラグランジュ法最適制御を実行する
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

                # 4. オイラー・ラグランジュ法最適制御入力計算 + 5次多項式フィット
                new_ff, extrema, u_pred_full, y_tgt, u_opt = solver.calculate_el_ff(tgt_params, sys_params, u_ff, y0)    # 最適入力を計算して、その結果の、FFパラメータ、極値、5次関数のFF制御入力、目標モデルの応答、最適入力を格納
                self.current_ff_matrix[dof_idx] = new_ff                                                    # FFパラメータの更新
                extrema_list.append(extrema)                                                                # 極値を保存

                # 内側ループの判定用評価関数J（実測データと目標モデルとの差）
                J = np.sum((y_tgt - y_shifted) ** 2)                                                        # 二乗和誤差を計算
                J_array.append(J)                                                                           # 24自由度それぞれの評価関数Jの空リストに追加

                # デバック用データを保存
                debug_info.append({
                    't': solver.t_eval,         # 実測時間
                    'y_data': raw_y,            # 実測POT値
                    'y_sys': y_sys_sim + Pf,    # システムモデル応答
                    'y_tgt': y_tgt + Pf,        # 目標モデル応答
                    'J': J,
                    'u_pred_full': u_pred_full, # 近似した5次関数FF入力
                    'u_opt': u_opt,             # 最適制御入力
                })

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
                self.current_ff_matrix = [[0.0] * 5 for _ in range(24)]                                                 # FFパラメータをリセット

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
            img_path = f"/tmp/lqr_plot_iter{self.current_outer + 1}_dof{dof_idx + 1}.png"                                                   # 画像ファイル名を作成
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
