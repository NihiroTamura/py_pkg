#!/usr/bin/env python3
import os
import sys
import time
import threading
import traceback
import warnings

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt16MultiArray

import scipy.optimize
import casadi as ca

warnings.simplefilter('ignore', RuntimeWarning)
np.seterr(all='ignore')

# ==============================================================================
# LQR重み行列（初期推定値用）
# ==============================================================================
LQR_Q_INITIAL = np.diag([80.0, 10.0, 0.1])
LQR_R_INITIAL = np.array([[0.000001]])

# ==============================================================================
# シミュレーションおよび最適化のパラメータ（チューニング要素）
# ==============================================================================
SIM_TIME = 5.0              # シミュレーション時間および実測データ収集時間（秒）
SIM_DT = 0.01               # シミュレーションのサンプル刻み幅（秒）
INIT_WAIT_TIME = 10.0       # 初期姿勢への移動後の待機時間（秒）


# ==============================================================================
# 数学ソルバー (System ID & LQR Optimal Control)
# ==============================================================================
class MathematicalSolver:
    # コンストラクタ
    def __init__(self, T, dt=SIM_DT, Q=None, R=None):       # 引数(FF制御入力時間, シミュレーションステップ時間, LQR状態重み行列, LQR入力重み行列)
        self.T = T                                          # FF制御入力時間を保存
        self.dt = dt                                        # シミュレーションステップ時間を保存
        self.t_eval = np.arange(0, SIM_TIME, self.dt)       # シミュレーション時間配列を作成
        self.Q = Q if Q is not None else LQR_Q_INITIAL.copy()       # 状態重み行列の保存（引数 Q が与えられていればそれを使用し、与えられていなければ LQR_Q をコピー）
        self.R = R if R is not None else LQR_R_INITIAL.copy()       # 入力重み行列の保存（引数 R が与えられていればそれを使用し、与えられていなければ LQR_R をコピー）

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
        """目標モデル同定: 1 / ((T1*s + 1)(s^2 + 2*wn*s + wn^2))"""
        # 同定に使用する評価関数
        def loss(p):
            T1, wn = p                                                              # 最適化変数の取り出し
            if T1 <= 0 or wn <= 0:                                                  # 制約条件（負の極を排除）
                return float('inf')

            # 3次遅れ系の係数を計算
            a2 = (2 * wn * T1 + 1) / T1
            a1 = (wn ** 2 * T1 + 2 * wn) / T1
            a0 = (wn ** 2) / T1

            y_sim, _, _, _ = self.simulate_unforced(self.t_eval, a2, a1, a0, y0)    # シミュレーション開始（"_"はその変数を使わないという意味）
            return np.sum((y_sim - y_data) ** 2)                                    # 評価関数値（二乗和誤差）を返す

        res = scipy.optimize.minimize(loss, [0.1, 10.0], method='Nelder-Mead')              # lossが最小になる変数[T1, wn]を最適化する
        T1, wn = res.x                                                                      # 最適変数を取り出す
        return T1, wn

    # システムモデル同定関数
    def fit_system_model(self, y_data, u_ff, y0):                                                   # 引数(実測データ, FF制御入力, 初期偏差)
        """システムモデル同定: b0 / (s^3 + a2*s^2 + a1*s + a0)"""
        # 同定に使用する評価関数
        def loss(p):
            a2, a1, a0, b0 = p                                                          # 最適化変数の取り出し
            if a0 <= 0 or a1 <= 0 or a2 <= 0:                                           # 制約条件（負の極を排除）
                return float('inf')
            
            y_sim, _, _ = self.simulate_forced(self.t_eval, a2, a1, a0, b0, u_ff, y0)   # シミュレーション開始（"_"はその変数を使わないという意味）
            return np.sum((y_sim - y_data) ** 2)                                        # 評価関数値（二乗和誤差）を返す

        res = scipy.optimize.minimize(loss, [10.0, 100.0, 1000.0, 20.0], method='Nelder-Mead')    # lossが最小になる変数[T1, wn]を最適化する
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
    # OCP最適制御（CasADi直接コロケーション法） + 5次多項式フィット
    # ------------------------------------------------------------------
    def calculate_ocp_ff(self, target_params, sys_params, u_ff, y0):                                          # 引数(目標モデルのパラメータ[T1, wn], システムモデルのパラメータ[a2, a1, a0, b0], FF制御入力, 初期偏差)
        """
        CasADi による最適制御問題（OCP）で最適制御入力を計算し、
        5次多項式 FF = a*t^5 + ... + e*t にフィットする。
        オイラー・ラグランジュ法に基づく変分問題を直接コロケーション法で離散化し、
        IPOPT ソルバーで求解する。入力制約 ±255 を直接扱える。
        """
        T1, wn = target_params                                                                          # 目標モデルのパラメータ取得
        a2_tgt, a1_tgt, a0_tgt = self._target_coeffs(T1, wn)                                            # 目標モデルの3次遅れ系の係数を計算
        a2_sys, a1_sys, a0_sys, b0_sys = sys_params                                                     # システムモデルのパラメータ取得

        # 目標軌道
        y_tgt, dy_tgt, ddy_tgt, _ = self.simulate_unforced(                                             # 目標モデルの応答を計算
            self.t_eval, a2_tgt, a1_tgt, a0_tgt, y0
        )

        # -----------------------------------------------------------
        # CasADi OCP の定式化（直接コロケーション法）
        # -----------------------------------------------------------
        dt = self.dt                                                                                    # シミュレーションのサンプル刻み幅
        N_ff = int(round(self.T / dt))                                                                  # FF入力区間のステップ数
        n_x = 3                                                                                         # 状態次元 (可制御正準形: x0=位置, x1=速度, x2=加速度)

        try:
            # CasADiシンボリック変数で状態方程式を定義
            x_sym = ca.SX.sym('x', n_x)                                                                 # 状態ベクトル x = [x0, x1, x2]
            u_sym = ca.SX.sym('u')                                                                      # 入力スカラー u

            # 状態方程式 ẋ = f(x, u) — 可制御正準形
            xdot = ca.vertcat(
                x_sym[1],                                                                               # dx0 = x1
                x_sym[2],                                                                               # dx1 = x2
                b0_sys * u_sym - a0_sys * x_sym[0] - a1_sys * x_sym[1] - a2_sys * x_sym[2]              # dx2 = b0*u - a0*x0 - a1*x1 - a2*x2
            )
            f_dyn = ca.Function('f', [x_sym, u_sym], [xdot])                                            # 動力学関数を定義

            # NLP変数・制約・目的関数の蓄積リスト
            w = []                                                                                      # NLP決定変数のリスト
            w0 = []                                                                                     # NLP決定変数の初期推定値リスト
            lbw = []                                                                                    # NLP決定変数の下界リスト
            ubw = []                                                                                    # NLP決定変数の上界リスト
            g = []                                                                                      # NLP等式制約リスト
            lbg = []                                                                                    # NLP等式制約の下界リスト
            ubg = []                                                                                    # NLP等式制約の上界リスト
            J_cost = 0                                                                                  # 目的関数の初期値

            # Q, R の重み行列をCasADi形式に変換
            Q_ca = ca.DM(self.Q)                                                                        # 状態誤差の重み行列
            R_val = float(self.R[0, 0])                                                                 # 入力の重みスカラー

            # 初期状態 (可制御正準形)
            x0_val = np.array([y0, 0.0, 0.0])                                                           # x(0) = [y0, 0, 0]

            # 各時刻の状態変数と入力変数をNLPに追加する
            X_vars = []                                                                                 # 各時刻の状態変数を保持するリスト
            U_vars = []                                                                                 # 各時刻の入力変数を保持するリスト

            for k in range(N_ff + 1):                                                                   # ステップ0～N_ffまでの状態変数
                Xk = ca.SX.sym(f'X_{k}', n_x)                                                          # 時刻kの状態変数を作成
                w.append(Xk)                                                                            # NLP変数に追加
                X_vars.append(Xk)                                                                       # 状態変数リストに追加

                if k == 0:                                                                              # 初期状態の制約
                    lbw.extend(x0_val.tolist())                                                          # 下界 = 初期値
                    ubw.extend(x0_val.tolist())                                                          # 上界 = 初期値（等式制約）
                    w0.extend(x0_val.tolist())                                                           # 初期推定値 = 初期値
                else:                                                                                   # 自由な状態変数
                    lbw.extend([-1e6] * n_x)                                                            # 下界（十分大きな範囲）
                    ubw.extend([1e6] * n_x)                                                             # 上界
                    w0.extend([y_tgt[min(k, len(y_tgt)-1)],                                             # 初期推定値（目標軌道を使用）
                               dy_tgt[min(k, len(dy_tgt)-1)],
                               ddy_tgt[min(k, len(ddy_tgt)-1)]])

                if k < N_ff:                                                                            # 入力変数はステップ0～N_ff-1まで
                    Uk = ca.SX.sym(f'U_{k}')                                                            # 時刻kの入力変数を作成
                    w.append(Uk)                                                                        # NLP変数に追加
                    U_vars.append(Uk)                                                                   # 入力変数リストに追加
                    lbw.append(-255.0)                                                                  # 入力の下界（PWM制約）
                    ubw.append(255.0)                                                                   # 入力の上界（PWM制約）
                    w0.append(0.0)                                                                      # 入力の初期推定値

            # 動力学制約（台形公式による離散化）と目的関数の構築
            for k in range(N_ff):                                                                       # 各ステップごとにコロケーション制約を追加
                Xk = X_vars[k]                                                                          # 時刻kの状態
                Xk1 = X_vars[k + 1]                                                                     # 時刻k+1の状態
                Uk = U_vars[k]                                                                          # 時刻kの入力

                # 台形公式: X_{k+1} = X_k + dt/2 * (f(X_k, U_k) + f(X_{k+1}, U_k))
                f_k = f_dyn(Xk, Uk)                                                                    # 時刻kの状態微分
                f_k1 = f_dyn(Xk1, Uk)                                                                   # 時刻k+1の状態微分（入力はZOH）
                gap = Xk1 - Xk - (dt / 2.0) * (f_k + f_k1)                                              # コロケーション残差
                g.append(gap)                                                                           # 等式制約として追加
                lbg.extend([0.0] * n_x)                                                                 # 残差 = 0（等式制約の下界）
                ubg.extend([0.0] * n_x)                                                                 # 残差 = 0（等式制約の上界）

                # 目的関数: Σ (x_err^T Q x_err + u^T R u) * dt
                # オイラー・ラグランジュ法に基づく汎関数 J = ∫₀ᵀ L(x,u) dt の離散近似
                idx = min(k, len(y_tgt) - 1)                                                            # 目標軌道のインデックス
                x_tgt_k = ca.DM([y_tgt[idx], dy_tgt[idx], ddy_tgt[idx]])                                # 時刻kの目標状態
                x_err_k = Xk - x_tgt_k                                                                 # 状態誤差
                J_cost += (ca.mtimes([x_err_k.T, Q_ca, x_err_k]) + R_val * Uk**2) * dt                  # ラグランジアン L(x,u) の積分

            # NLP問題を構築
            w_cat = ca.vertcat(*w)                                                                      # 全NLP変数を結合
            g_cat = ca.vertcat(*g)                                                                      # 全制約を結合
            nlp = {'f': J_cost, 'x': w_cat, 'g': g_cat}                                                 # NLP辞書を定義

            # IPOPTソルバーの設定
            opts = {
                'ipopt.print_level': 0,                                                                 # IPOPTの出力を抑制
                'ipopt.max_iter': 500,                                                                  # 最大反復回数
                'ipopt.tol': 1e-6,                                                                      # 収束許容誤差
                'print_time': 0,                                                                        # CasADiのタイミング出力を抑制
            }
            solver = ca.nlpsol('ocp_solver', 'ipopt', nlp, opts)                                         # IPOPTソルバーを生成

            # NLPを求解
            sol = solver(
                x0=w0,                                                                                  # 初期推定値
                lbx=lbw, ubx=ubw,                                                                       # 決定変数の上下界
                lbg=lbg, ubg=ubg                                                                        # 制約の上下界
            )
            w_opt = sol['x'].full().flatten()                                                            # 最適解を取り出す

            # 最適解から入力列を抽出
            u_opt = np.zeros(len(self.t_eval))                                                          # 最適入力を保存する配列
            offset = 0                                                                                  # 最適解ベクトル内のオフセット
            for k in range(N_ff + 1):                                                                   # 各ステップの変数を順番に取り出す
                offset += n_x                                                                           # 状態変数(3要素)をスキップ
                if k < N_ff:                                                                            # 入力変数を取り出す
                    u_opt[k] = w_opt[offset]                                                            # 最適入力を格納
                    offset += 1                                                                         # 入力変数(1要素)をスキップ

        except Exception:
            # IPOPTが失敗した場合のフォールバック：簡易的な比例制御で近似
            u_opt = np.zeros(len(self.t_eval))                                                          # 最適入力を保存する配列
            x_sys = np.array([y0, 0.0, 0.0], dtype=float)                                               # システムモデルの初期状態
            K_fallback = np.array([1.0, 10.0, 100.0])                                                   # フォールバック用の簡易ゲイン
            for i in range(len(self.t_eval)):
                if self.t_eval[i] <= self.T:
                    x_tgt_i = np.array([y_tgt[i], dy_tgt[i], ddy_tgt[i]])
                    x_err_i = x_sys - x_tgt_i
                    u_val = -float(K_fallback @ x_err_i)
                    u_val = np.clip(u_val, -255.0, 255.0)
                    u_opt[i] = u_val
                    dx0 = x_sys[1]
                    dx1 = x_sys[2]
                    dx2 = b0_sys * u_val - a0_sys * x_sys[0] - a1_sys * x_sys[1] - a2_sys * x_sys[2]
                    x_sys[0] += dx0 * dt
                    x_sys[1] += dx1 * dt
                    x_sys[2] += dx2 * dt

        # 5次多項式フィット（0≤t≤T, 端点0, 極値2個）
        t_ff = self.t_eval[self.t_eval <= self.T]                                                       # FF入力を与える時間だけ取り出す
        u_opt_ff = u_opt[: len(t_ff)]                                                                   # FF入力を与える区間だけの最適入力を取り出す

        # 5次関数を定義する関数
        def poly(t, a, b, c, d):                                                                        # 引数(時間, 5次関数パラメータa・b・c・d)
            e = -(a * self.T ** 4 + b * self.T ** 3 + c * self.T ** 2 + d * self.T)     # 5次関数パラメータeを計算
            return a * t ** 5 + b * t ** 4 + c * t ** 3 + d * t ** 2 + e * t            # 5次関数FF入力値を返す

        # OCP最適入力を5次関数で近似するための評価関数
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
# ROS2 ノード
# ==============================================================================
class OptimalControlSequencer(Node):
    def __init__(self, T):
        super().__init__('optimal_control_sequencer_lqr')
        self.T = T

        self.initial_stabilize_time = INIT_WAIT_TIME
        self.data_collection_time = SIM_TIME
        self.dt = SIM_DT

        self.state = "INIT_ROBOT"

        # ポテンショメータ値の範囲 (26 elements) - 参照用
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

        self.home_pot = list(self.initial_pot)

        # 4自由度のみステップサイズ400、他は1
        self.target_pot = list(self.initial_pot)
        for i in range(len(self.target_pot)):
            if i == 3: # 4自由度はインデックス3 (0-indexed)
                self.target_pot[i] += 400.0
            else:
                self.target_pot[i] += 1.0

        self.current_ff_matrix = [[0.0, 0.0, 0.0, 0.0, 0.0] for _ in range(24)]
        self.best_ff_matrix = [[0.0, 0.0, 0.0, 0.0, 0.0] for _ in range(24)]
        self.best_extrema = [(0,0,0,0) for _ in range(24)]

        self.buffer_time_series = {f'board{i}': [] for i in range(1, 6)}

        # Publisher作成
        self.pub_target = self.create_publisher(Float32MultiArray, '/board_android_float/sub', 10)
        self.pub_ff = {}
        for i in range(1, 6):
            self.pub_ff[i] = self.create_publisher(Float32MultiArray, f'/board{i}_FFparam_float/sub', 10)

        # Subscriber作成
        self.create_subscription(UInt16MultiArray, '/board1_tk/pub', lambda m: self.cb_board(m, 1), 10)
        self.create_subscription(UInt16MultiArray, '/board2_tk/pub', lambda m: self.cb_board(m, 2), 10)
        self.create_subscription(UInt16MultiArray, '/board3_tk/pub', lambda m: self.cb_board(m, 3), 10)
        self.create_subscription(UInt16MultiArray, '/board4_tk/pub', lambda m: self.cb_board(m, 4), 10)
        self.create_subscription(UInt16MultiArray, '/board5_tk/pub', lambda m: self.cb_board(m, 5), 10)

        # タイマー作成
        self.control_timer = self.create_timer(0.1, self.sequencer_loop)

        self.state_start_time = self.get_clock().now()

    @staticmethod
    def build_dof_map():
        """24自由度 → 26要素配列インデックス"""
        dof_map = []
        for i in range(1, 4):
            for j in range(6):
                dof_map.append((i - 1) * 6 + j)
        for i in [4, 5]:
            for j in range(3):
                dof_map.append(18 + (i - 4) * 4 + j)
        return dof_map

    # ROS2 Subscriberのコールバック関数
    def cb_board(self, msg, board_id):
        if self.state == "COLLECTING":
            if board_id in [4, 5]:
                selected = [msg.data[idx] for idx in [0, 1, 2, 6, 7, 8]]
                self.buffer_time_series[f'board{board_id}'].append(selected)
            else:
                self.buffer_time_series[f'board{board_id}'].append(list(msg.data))
    
    # 目標値のPublish関数
    def publish_target_positions(self, pot_list):
        msg = Float32MultiArray()
        msg.data = [float(x) for x in pot_list]
        self.pub_target.publish(msg)
        self.get_logger().info("=== 送信した目標値 (Target POT) ===")
        for i in range(0, len(pot_list), 6):
            chunk = [f"{x:.1f}" for x in pot_list[i:i + 6]]
            self.get_logger().info(f"  [{i + 1:02d}-{min(i + 6, len(pot_list)):02d}]: " + " | ".join(chunk))
        self.get_logger().info("===================================")

    # FFパラメータのPublish関数
    def publish_all_ff_parameters(self):
        dof_idx = 0
        solver = MathematicalSolver(self.T, self.dt)
        self.get_logger().info("=== 送信したFFパラメータ ===")
        for b_id in range(1, 6):
            msg = Float32MultiArray()
            data = []
            for _ in range(6):
                if b_id in [4, 5] and _ >= 3:
                    data.extend([0.0, 0.0, 0.0, 0.0, 0.0, self.T])
                else:
                    a, b, c, d, e = self.current_ff_matrix[dof_idx]
                    t1, y1, t2, y2 = solver.calc_extrema_from_ff([a, b, c, d, e])
                    data.extend([float(a), float(b), float(c), float(d), float(e), float(self.T)])
                    self.get_logger().info(
                        f"  B{b_id}-D{_ + 1} (DOF {dof_idx + 1:02d}): "
                        f"a={a:.1e}, b={b:.1e}, c={c:.1e}, d={d:.1e}, e={e:.1e} | "
                        f"t1={t1:.3f}, y1={y1:.2f}, t2={t2:.3f}, y2={y2:.2f}"
                    )
                    dof_idx += 1
            msg.data = data
            self.pub_ff[b_id].publish(msg)
        self.get_logger().info("============================")

    # ROS2のタイマーのコールバック関数
    def sequencer_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds / 1e9

        # 初期姿勢へ戻す状態
        if self.state == "INIT_ROBOT":
            self.get_logger().info("=== 実機データ収集のための初期姿勢移動 ===")
            self.publish_target_positions(self.initial_pot)
            self.state = "WAIT_INITIAL_STABILIZE"
            self.state_start_time = now

        # 初期姿勢への収束を待っている状態
        elif self.state == "WAIT_INITIAL_STABILIZE":
            if elapsed >= self.initial_stabilize_time:
                self.publish_all_ff_parameters()
                time.sleep(0.1)
                self.publish_target_positions(self.target_pot)
                for k in self.buffer_time_series:
                    self.buffer_time_series[k].clear()
                self.state = "COLLECTING"
                self.state_start_time = now

        # ロボットの実測データを収集している状態
        elif self.state == "COLLECTING":
            if elapsed >= self.data_collection_time:
                self.state = "PROCESSING"
                threading.Thread(target=self.dispatch_optimization_pipeline, daemon=True).start()

        # プログラムの最終状態
        elif self.state == "FINISHED":
            self.get_logger().info("=== 最終安定化: home_pot を送信します ===")
            self.publish_target_positions(self.home_pot)
            self.get_logger().info("すべての実験試行が正常終了しました。")
            self.control_timer.cancel()
            raise SystemExit(0)
    
    # 実測データから次回のFF入力を計算する関数（スレッド関数）
    def dispatch_optimization_pipeline(self):
        try:
            # 26要素と最適化する24自由度の対応表を作成
            data_24_dof = []
            dof_map = self.build_dof_map()

            # Boardごとに保存されている実測データを24自由度の時系列データへ変換する
            for i in range(1, 4):
                arr = np.array(self.buffer_time_series[f'board{i}'])
                for j in range(6):
                    data_24_dof.append(arr[:, j] if arr.size > 0 else [])
            for i in [4, 5]:
                arr = np.array(self.buffer_time_series[f'board{i}'])
                for j in range(3):
                    data_24_dof.append(arr[:, j] if arr.size > 0 else [])

            # 24自由度それぞれに対して、目標モデル同定・システムモデル同定・LQR最適制御(Q/R探索)を実行する
            solver = MathematicalSolver(self.T, self.dt)
            N_samples = int(SIM_TIME / self.dt)

            for dof_idx in range(24):
                self.get_logger().info(f"--- DOF {dof_idx + 1} の処理開始 ---")
                raw_y = np.array(data_24_dof[dof_idx])[:N_samples]
                if len(raw_y) < N_samples:
                    raw_y = np.pad(raw_y, (0, max(0, N_samples - len(raw_y))), mode='edge')

                Pi = self.initial_pot[dof_map[dof_idx]]
                Pf = self.target_pot[dof_map[dof_idx]]
                y0 = Pi - Pf

                y_shifted = raw_y - Pf

                # 1. Target Model ID
                tgt_params = solver.fit_target_model(y_shifted, y0)

                # 2. System Model ID
                a, b, c, d, e = 0.0, 0.0, 0.0, 0.0, 0.0
                t_ff = solver.t_eval[solver.t_eval <= self.T]
                u_ff = np.zeros(N_samples)
                sys_params = solver.fit_system_model(y_shifted, u_ff, y0)

                # 3. オフラインでのQ/R探索ループ
                is_large_step = abs(y0) > 10.0  # ステップサイズが1より大きい（例えば400）かどうか
                
                # 最適化のための目的関数（1次元探索で最小化）
                def weight_optimization_loss(log10_r):
                    try:
                        q0 = LQR_Q_INITIAL[0, 0]
                        q1 = LQR_Q_INITIAL[1, 1]
                        q2 = LQR_Q_INITIAL[2, 2]
                        r  = 10 ** log10_r
                        
                        solver.Q = np.diag([q0, q1, q2])
                        solver.R = np.array([[r]])
                        
                        _, _, _, y_tgt, u_opt = solver.calculate_ocp_ff(tgt_params, sys_params, u_ff, y0)
                        
                        a2_sys, a1_sys, a0_sys, b0_sys = sys_params
                        y_sys_sim, _, _ = solver.simulate_forced(solver.t_eval, a2_sys, a1_sys, a0_sys, b0_sys, u_opt, y0)
                        
                        # 目標軌道との誤差
                        J_track = np.sum((y_tgt - y_sys_sim)**2)
                        
                        max_u = np.max(u_opt)
                        min_u = np.min(u_opt)
                        peak_u = max(abs(max_u), abs(min_u))
                        
                        penalty = 0.0
                        if is_large_step:
                            # ステップサイズが大きい場合、制御入力のピークを約50に近づける
                            if peak_u < 40:
                                penalty += 1e5 * (40 - peak_u)**2
                            elif peak_u > 60:
                                penalty += 1e5 * (peak_u - 60)**2
                        else:
                            # ステップサイズが小さい場合、過大な入力にならないように制限
                            if peak_u > 255:
                                penalty += 1e5 * (peak_u - 255)**2
                                
                        return J_track + penalty
                    except Exception as e:
                        return float('inf')

                self.get_logger().info("Q・Rの自動探索を実行中...")
                res = scipy.optimize.minimize_scalar(
                    weight_optimization_loss, 
                    bounds=(-10.0, 2.0),
                    method='bounded',
                    options={'maxiter': 50}
                )
                
                best_log10_r = res.x
                best_q0 = LQR_Q_INITIAL[0, 0]
                best_q1 = LQR_Q_INITIAL[1, 1]
                best_q2 = LQR_Q_INITIAL[2, 2]
                best_r  = 10 ** best_log10_r
                
                # 最終的な最適なQ/RでFFパラメータと極値を計算
                solver.Q = np.diag([best_q0, best_q1, best_q2])
                solver.R = np.array([[best_r]])
                new_ff, extrema, u_pred_full, y_tgt, u_opt = solver.calculate_ocp_ff(tgt_params, sys_params, u_ff, y0)
                
                self.current_ff_matrix[dof_idx] = new_ff
                self.best_ff_matrix[dof_idx] = new_ff
                self.best_extrema[dof_idx] = extrema
                
                max_u = np.max(u_opt)
                min_u = np.min(u_opt)
                
                self.get_logger().info(f"--- DOF {dof_idx + 1} 最適化結果 ---")
                self.get_logger().info(f"最適重み: Q = diag([{best_q0:.2e}, {best_q1:.2e}, {best_q2:.2e}]), R = {best_r:.2e}")
                self.get_logger().info(f"制御入力: Max = {max_u:.2f}, Min = {min_u:.2f}")

            # すべてのDOFの処理が終わったら、最終パラメータを表示して終了状態へ
            self._print_best_params(dof_map)
            self.state = "FINISHED"
            self.state_start_time = self.get_clock().now()

        except Exception as exc:
            self.get_logger().error(f"最適化パイプライン異常: {exc}\\n{traceback.format_exc()}")
            self.state = "FINISHED"
            self.state_start_time = self.get_clock().now()

    # 最良のFFパラメータを一覧表示する関数
    def _print_best_params(self, dof_map):
        """最良パラメータa,b,c,d,eとt1,t2,y1,y2を表示する"""
        self.get_logger().info("")
        self.get_logger().info("=" * 90)
        self.get_logger().info("  最適化完了 最終結果一覧")
        self.get_logger().info("=" * 90)

        board_names = ["Board1", "Board2", "Board3", "Board4", "Board5"]
        dof_idx = 0
        for b_id in range(1, 6):
            n_dof = 6 if b_id <= 3 else 3
            self.get_logger().info(f"")
            self.get_logger().info(f"--- {board_names[b_id - 1]} ({n_dof}DOF) ---")
            self.get_logger().info(
                f"  {'DOF':>4s} | {'a':>12s}  {'b':>12s}  {'c':>12s}  {'d':>12s}  {'e':>12s}"
                f"  | {'t1':>6s}  {'y1':>8s}  {'t2':>6s}  {'y2':>8s}"
            )
            self.get_logger().info("  " + "-" * 106)

            for local_d in range(n_dof):
                a, b, c, d, e = self.best_ff_matrix[dof_idx]
                t1, y1, t2, y2 = self.best_extrema[dof_idx]
                self.get_logger().info(
                    f"  {dof_idx + 1:4d} | {a:>12.4e}  {b:>12.4e}  {c:>12.4e}  {d:>12.4e}  {e:>12.4e}"
                    f"  | {t1:6.3f}  {y1:8.2f}  {t2:6.3f}  {y2:8.2f}"
                )
                dof_idx += 1

        self.get_logger().info("=" * 90)
        self.get_logger().info("")

# ==============================================================================
# エントリーポイント
# ==============================================================================

def main(args=None):
    print("【FF制御時間 T を入力してください】")
    T = float(input("> "))

    rclpy.init(args=args)
    node = OptimalControlSequencer(T)
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
