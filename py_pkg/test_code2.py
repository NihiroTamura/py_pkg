#!/usr/bin/env python3
import rclpy                                                    # ROS2
from rclpy.node import Node                                     # ROS2 ノード
from std_msgs.msg import Float32MultiArray, UInt16MultiArray    # ROS2メッセージ型
import numpy as np                                              # 数値計算ライブラリ
import scipy.optimize                                           # 最適化
import pandas as pd                                             # データフレーム
import openpyxl                                                 # エクセル
from openpyxl.drawing.image import Image as OpenpyxlImage       # 画像
import matplotlib                                               # プロット
matplotlib.use('Agg')                                           # 画面を使わず画像だけ保存するモードに変更
import matplotlib.pyplot as plt                                 # プロット
import threading                                                # スレッド処理
import time                                                     # 時間
import os                                                       # OS関係
import sys                                                      # Pythonシステム関係
import warnings                                                 # Pythonの警告表示を制御
warnings.simplefilter('ignore', RuntimeWarning)                 # RuntimeWarningを表示しない
np.seterr(all='ignore')                                         # NumPy版のWarning無効化

# ==========================================
# 数学ソルバー (System ID & Optimal Control)
# ==========================================
class MathematicalSolver:
    def __init__(self, T, dt=0.01):
        self.T = T                                  # FF入力時間
        self.dt = dt                                # シミュレーション刻み
        self.t_eval = np.arange(0, 5.0, self.dt)    # 5秒間分のシミュレーション時間データ

    #　目標モデル生成(入力なし)関数
    def simulate_unforced(self, t_array, a2, a1, a0, y0):   # 引数（時間, 3次遅れ系の係数, 初期値）
        N = len(t_array)                                    # サンプル数（シミュレーション時間）
        x = np.array([y0, a2*y0, a1*y0], dtype=float)       # 状態変数の初期化
        y_traj = np.zeros(N)                                # 出力を保存する配列
        dy_traj = np.zeros(N)                               # 一次微分を保存する配列
        ddy_traj = np.zeros(N)                              # 二次微分を保存する配列
        dddy_traj = np.zeros(N)                             # 三次微分を保存する配列
        for i in range(N):                  # 時間方向に1サンプルずつ積分
            y_traj[i] = x[0]                # 状態変数 x0 を出力
            dy_traj[i] = x[1] - a2*x[0]     # 一次微分を計算
            ddy_traj[i] = x[2] - a1*x[0]    # 二次微分を計算
            dddy_traj[i] = -a0*x[0]         # 三次微分を計算（入力は0）
            
            dx0 = x[1] - a2*x[0]            # 状態変数 x0 の微分
            dx1 = x[2] - a1*x[0]            # 状態変数 x1 の微分
            dx2 = -a0*x[0]                  # 状態変数 x2 の微分
            
            x[0] += dx0 * self.dt           # オイラー法で積分
            x[1] += dx1 * self.dt
            x[2] += dx2 * self.dt
        return y_traj, dy_traj, ddy_traj, dddy_traj

    # システムモデル生成(入力あり)関数
    def simulate_forced(self, t_array, a2, a1, a0, b0, u_array, y0):    # 引数（時間, 3次遅れ系の係数, 入力波形, 初期値）
        N = len(t_array)                                                # サンプル数（シミュレーション時間）
        x = np.array([y0, a2*y0, a1*y0], dtype=float)                   # 状態変数の初期化
        y_traj = np.zeros(N)                                            # 出力を保存する配列
        for i in range(N):                  # 時間方向に1サンプルずつ積分
            y_traj[i] = x[0]                # 状態変数 x0 を出力
            
            dx0 = x[1] - a2*x[0]            # 状態変数 x0 の微分
            dx1 = x[2] - a1*x[0]            # 状態変数 x1 の微分
            dx2 = b0*u_array[i] - a0*x[0]   # 状態変数 x2 の微分
            
            x[0] += dx0 * self.dt           # オイラー法で積分
            x[1] += dx1 * self.dt
            x[2] += dx2 * self.dt
        return y_traj
    
    # 目標モデルを同定する関数
    def fit_target_model(self, y_data, y0):                                         # 引数（実測データ, 初期位置）
        # Target Model: 1 / ((T1*s + 1)(s^2 + 2*wn*s + wn^2))
        # 評価関数
        def loss(p):
            T1, wn = p                                                              # 最適化変数               
            if T1 <= 0 or wn <= 0:                                                  # 制約（負になる条件を排除）
                return float('inf')
            
            a2 = (2*wn*T1 + 1)/T1                                                   # 3次遅れ系の係数変換
            a1 = (wn**2*T1 + 2*wn)/T1
            a0 = (wn**2)/T1
            
            y_sim, _, _, _ = self.simulate_unforced(self.t_eval, a2, a1, a0, y0)    # 目標モデルのシミュレーション開始
            return np.sum((y_sim - y_data)**2)                                      # 二乗和誤差を返す

        res = scipy.optimize.minimize(loss, [0.1, 10.0], method='Nelder-Mead')      # lossを最小にする最適化開始（Nelder–Mead法)
        T1, wn = res.x                                                              # 最適化終了後のパラメータ取得
        return T1, wn

    # システムモデルを同定する関数
    def fit_system_model(self, y_data, u_ff, y0):                                                   # 引数（実測データ, FF入力, 初期位置）
        # System Model: b0 / (s^3 + a2*s^2 + a1*s + a0)
        # 評価関数
        def loss(p):
            a2, a1, a0, b0 = p                                                                      # 最適化変数
            if a0 <= 0 or a1 <= 0 or a2 <= 0:                                                       # 制約（負になる条件を排除）
                return float('inf')
            
            y_sim = self.simulate_forced(self.t_eval, a2, a1, a0, b0, u_ff, y0)                     # システムモデルのシミュレーション開始
            return np.sum((y_sim - y_data)**2)                                                      # 二乗和誤差を返す

        res = scipy.optimize.minimize(loss, [10.0, 100.0, 1000.0, 1000.0], method='Nelder-Mead')    # lossを最小にする最適化開始（Nelder–Mead法)
        return res.x # a2, a1, a0, b0

    # 目標モデル(Target Model)を実現するために必要なFF入力を計算する関数
    def calculate_optimal_ff(self, target_params, sys_params, y0):                                              # 引数（fit_target_model()で求めたT1・ωn, fit_system_model()で求めたa2・a1・a0・b0, 初期位置）
        T1, wn = target_params                                                                                  # システム同定後の目標モデルのパラメータ取得
        a2_tgt = (2*wn*T1 + 1)/T1                                                                               # 目標モデルの3次遅れ系の係数変換
        a1_tgt = (wn**2*T1 + 2*wn)/T1
        a0_tgt = (wn**2)/T1

        a2_sys, a1_sys, a0_sys, b0_sys = sys_params                                                             # システム同定後のシステムモデルのパラメータ取得

        # Get target trajectory and its derivatives
        y_tgt, dy_tgt, ddy_tgt, dddy_tgt = self.simulate_unforced(self.t_eval, a2_tgt, a1_tgt, a0_tgt, y0)      # 目標モデルのシミュレーション開始

        # Inverse Dynamics to find optimal control input (and clip to PWM bounds)
        u_opt = (dddy_tgt + a2_sys * ddy_tgt + a1_sys * dy_tgt + a0_sys * y_tgt) / b0_sys                       # 逆ダイナミクスで最適入力計算（目標位置→入力）
        u_opt = np.clip(u_opt, -255, 255)                                                                       # 入力をクリッピング

        # Fit 5th-order polynomial to u_opt for t in [0, T]
        t_ff = self.t_eval[self.t_eval <= self.T]                                                               # FF制御入力時間分だけ時間データを取得
        u_opt_ff = u_opt[:len(t_ff)]                                                                            # FF制御入力時間分だけの最適入力を取得

        # 5次多項式を定義する関数
        def poly(t, a, b, c, d):
            e = -(a*self.T**4 + b*self.T**3 + c*self.T**2 + d*self.T)
            return a*t**5 + b*t**4 + c*t**3 + d*t**2 + e*t
        
        # 5次多項式の係数を最適化するための評価関数
        def fit_loss(p):
            a, b, c, d = p                                                                              # FFパラメータを取得
            u_pred = poly(t_ff, a, b, c, d)                                                             # FF入力を生成
            mse = np.sum((u_pred - u_opt_ff)**2)                                                        # 二乗誤差（最適入力とフィッティングする5次多項式の差）
            
            # PWM limit constraint (-255 to 255) using a heavy penalty
            penalty = np.sum(np.maximum(0, u_pred - 255)**2) + np.sum(np.maximum(0, -255 - u_pred)**2)  # -255から255の間に収めるためのペナルティ
            
            return mse + 1e6 * penalty
        
        res = scipy.optimize.minimize(fit_loss, [0.0, 0.0, 0.0, 0.0], method='BFGS')                    # 5次関数の係数の最適化を開始
        a, b, c, d = res.x                                                                              # 結果のパラメータを取得
        e = -(a*self.T**4 + b*self.T**3 + c*self.T**2 + d*self.T)

        # Calculate extrema
        roots = np.roots([5*a, 4*b, 3*c, 2*d, e])                                                       # 5次多項式を微分したときの係数から極値を計算
        real_roots = [r.real for r in roots if abs(r.imag) < 1e-6 and 0 < r.real < self.T]              # 実数かつ0<t<Tの間の極値を取得
        
        if len(real_roots) >= 2:
            real_roots.sort()
            t1, t2 = real_roots[0], real_roots[1]
        elif len(real_roots) == 1:
            t1 = real_roots[0]
            t2 = self.T / 2.0
        else:
            t1 = self.T * 0.33
            t2 = self.T * 0.66
            
        y1 = a*t1**5 + b*t1**4 + c*t1**3 + d*t1**2 + e*t1
        y2 = a*t2**5 + b*t2**4 + c*t2**3 + d*t2**2 + e*t2

        # Return full time series for debug, and the parameters
        u_pred_full = np.zeros_like(self.t_eval)
        u_pred_full[:len(t_ff)] = poly(t_ff, a, b, c, d)

        return [a, b, c, d, e], (t1, y1, t2, y2), u_pred_full, y_tgt

# ==========================================
# ROS2 ノード
# ==========================================
class OptimalControlSequencer(Node):
    def __init__(self, csv_path, T, max_iter, target_mode):
        super().__init__('optimal_control_sequencer_adrc')
        self.csv_path = csv_path
        self.T = T
        self.max_outer_iter = max_iter
        self.target_mode = target_mode # "1": Preset then Random, "2": Random only
        
        self.current_outer = 0
        self.current_inner = 0
        self.max_inner_iter = 10 # 評価関数Jが最小になるまで繰り返す最大回数

        self.initial_stabilize_time = 10.0
        self.data_collection_time = 5.0
        self.dt = 0.01

        self.state = "INIT_ROBOT"
        
        self.DEBUG_EXCEL = True
        if self.DEBUG_EXCEL:
            self.debug_wb = openpyxl.Workbook()
            self.default_sheet = self.debug_wb.active
            self.excel_path = os.path.splitext(self.csv_path)[0] + "_debug_models.xlsx"

        # ポテンショメータ値の範囲 (26 elements)
        self.pot_bounds = [
            (450, 700), (135, 550), (500, 680), (250, 700), (66, 259), (192, 389), # board1
            (70, 200),  (60, 465),  (115, 200), (100, 550), (239, 430), (205, 395), # board2
            (30, 660),  (30, 690),  (110, 830), (3, 630),   (3, 700),   (9, 660),   # board3
            (275, 360), (115, 785), (192, 440), (284, 557), # board4 (0-2 and dummy)
            (323, 580), (188, 630), (375, 500), (300, 490)  # board5 (0-2 and dummy)
        ]
        
        self.initial_pot = [
            500.0, 200.0, 500.0, 300.0, 170.0, 300.0,
            160.0, 410.0, 200.0, 500.0, 350.0, 220.0,
            300.0, 250.0, 400.0, 350.0, 420.0, 400.0,
            325.0, 370.0, 280.0, 420.0,
            360.0, 390.0, 420.0, 390.0
        ]

        # プリセット目標値 (26 elements)
        self.preset_targets = [
            [501, 201, 501, 700, 171, 301, 161, 411, 201, 501, 351, 221, 301, 251, 401, 351, 421, 401, 326, 371, 281, 300, 361, 391, 421, 300],
            [600, 300, 600, 500, 200, 250, 120, 350, 150, 400, 300, 300, 400, 350, 500, 450, 520, 500, 300, 400, 300, 300, 400, 450, 400, 300]
        ]

        self.current_ff_matrix = [[0.0, 0.0, 0.0, 0.0, 0.0] for _ in range(24)]
        self.best_ff_matrix = None
        self.best_extrema = None
        self.min_J_sum = float('inf')
        self.best_debug_data = None

        self.buffer_time_series = {f'board{i}': [] for i in range(1, 6)}
        self.target_pot = self.get_next_target_positions()

        # Publishers
        self.pub_target = self.create_publisher(Float32MultiArray, '/board_android_float/sub', 10)
        self.pub_ff = {}
        for i in range(1, 6):
            self.pub_ff[i] = self.create_publisher(Float32MultiArray, f'/board{i}_FFparam_float/sub', 10)

        # Subscribers
        self.sub_board1 = self.create_subscription(UInt16MultiArray, '/board1_tk/pub', lambda msg: self.cb_board(msg, 1), 10)
        self.sub_board2 = self.create_subscription(UInt16MultiArray, '/board2_tk/pub', lambda msg: self.cb_board(msg, 2), 10)
        self.sub_board3 = self.create_subscription(UInt16MultiArray, '/board3_tk/pub', lambda msg: self.cb_board(msg, 3), 10)
        self.sub_board4 = self.create_subscription(UInt16MultiArray, '/board4_tk/pub', lambda msg: self.cb_board(msg, 4), 10)
        self.sub_board5 = self.create_subscription(UInt16MultiArray, '/board5_tk/pub', lambda msg: self.cb_board(msg, 5), 10)

        # Timer
        self.control_timer = self.create_timer(0.1, self.sequencer_loop)
        self.state_start_time = self.get_clock().now()

    def get_next_target_positions(self):
        if self.target_mode == "1" and self.current_outer < len(self.preset_targets):
            return self.preset_targets[self.current_outer]
        else:
            return [float(np.random.randint(b[0], b[1]+1)) for b in self.pot_bounds]

    def cb_board(self, msg, board_id):
        if self.state == "COLLECTING":
            if board_id in [4, 5]:
                selected = [msg.data[idx] for idx in [0, 1, 2, 6, 7, 8]]
                self.buffer_time_series[f'board{board_id}'].append(selected)
            else:
                self.buffer_time_series[f'board{board_id}'].append(list(msg.data))

    def publish_target_positions(self, pot_list):
        msg = Float32MultiArray()
        msg.data = [float(x) for x in pot_list]
        self.pub_target.publish(msg)
        
        self.get_logger().info("=== 送信した目標値 (Target POT) ===")
        for i in range(0, len(pot_list), 6):
            chunk = [f"{x:.1f}" for x in pot_list[i:i+6]]
            self.get_logger().info(f"  [{i+1:02d}-{min(i+6, len(pot_list)):02d}]: " + " | ".join(chunk))
        self.get_logger().info("===================================")

    def publish_all_ff_parameters(self):
        dof_idx = 0
        self.get_logger().info("=== 送信したFFパラメータ ===")
        for b_id in range(1, 6):
            msg = Float32MultiArray()
            data = []
            for _ in range(6): # 6 DOF per board
                if b_id in [4, 5] and _ >= 3:
                    data.extend([0.0, 0.0, 0.0, 0.0, 0.0, self.T])
                else:
                    a, b, c, d, e = self.current_ff_matrix[dof_idx]
                    data.extend([float(a), float(b), float(c), float(d), float(e), float(self.T)])
                    self.get_logger().info(f"  B{b_id}-D{_+1} (DOF {dof_idx+1:02d}): a={a: .1e}, b={b: .1e}, c={c: .1e}, d={d: .1e}, e={e: .1e}")
                    dof_idx += 1
            msg.data = data
            self.pub_ff[b_id].publish(msg)
        self.get_logger().info("============================")

    def sequencer_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds / 1e9

        if self.state == "INIT_ROBOT":
            self.get_logger().info(f"=== 最適制御 外側ループ {self.current_outer + 1} / {self.max_outer_iter} (内側ループ {self.current_inner+1}/{self.max_inner_iter}) ===")
            self.publish_target_positions(self.initial_pot)
            self.state = "WAIT_INITIAL_STABILIZE"
            self.state_start_time = now

        elif self.state == "WAIT_INITIAL_STABILIZE":
            if elapsed >= self.initial_stabilize_time:
                self.publish_all_ff_parameters()
                time.sleep(0.1)
                self.publish_target_positions(self.target_pot)
                for k in self.buffer_time_series: 
                    self.buffer_time_series[k].clear()
                self.state = "COLLECTING"
                self.state_start_time = now

        elif self.state == "COLLECTING":
            if elapsed >= self.data_collection_time:
                self.state = "PROCESSING"
                threading.Thread(target=self.dispatch_optimization_pipeline).start()

        elif self.state == "FINISHED":
            if self.DEBUG_EXCEL:
                if self.default_sheet in self.debug_wb.worksheets and len(self.debug_wb.worksheets) > 1:
                    self.debug_wb.remove(self.default_sheet)
                self.debug_wb.save(self.excel_path)
            self.get_logger().info("すべての実験試行が正常終了しました。")
            self.control_timer.cancel()
            raise SystemExit(0)

    def dispatch_optimization_pipeline(self):
        # Build 24-DOF array
        data_24_dof = []
        dof_map = [] # To extract targets easily
        for i in range(1, 4):
            for j in range(6):
                dof_map.append((i-1)*6 + j)
        for i in [4, 5]:
            for j in range(3):
                dof_map.append(18 + (i-4)*4 + j) # mapping to 26 element array

        for i in range(1, 4):
            arr = np.array(self.buffer_time_series[f'board{i}']) # N x 12
            for j in range(6):
                data_24_dof.append(arr[:, j] if arr.size > 0 else [])
        for i in [4, 5]:
            arr = np.array(self.buffer_time_series[f'board{i}']) # N x 6
            for j in range(3):
                data_24_dof.append(arr[:, j] if arr.size > 0 else [])
        
        solver = MathematicalSolver(self.T, self.dt)
        N_samples = int(5.0 / self.dt)

        J_array = []
        debug_info = []
        extrema_list = []

        for dof_idx in range(24):
            raw_y = np.array(data_24_dof[dof_idx])[:N_samples]
            if len(raw_y) < N_samples:
                raw_y = np.pad(raw_y, (0, max(0, N_samples - len(raw_y))), 'edge')

            # Initial and Target values
            Pi = self.initial_pot[dof_map[dof_idx]]
            Pf = self.target_pot[dof_map[dof_idx]]
            y0 = Pi - Pf

            # Shift data so it converges to 0
            y_shifted = raw_y - Pf

            # 1. Target Model ID
            tgt_params = solver.fit_target_model(y_shifted, y0)

            # 2. System Model ID
            a, b, c, d, e = self.current_ff_matrix[dof_idx]
            t_ff = solver.t_eval[solver.t_eval <= self.T]
            u_ff = np.zeros(N_samples)
            u_ff[:len(t_ff)] = a*t_ff**5 + b*t_ff**4 + c*t_ff**3 + d*t_ff**2 + e*t_ff
            
            sys_params = solver.fit_system_model(y_shifted, u_ff, y0)

            # 3. Calculate squared error J
            a2_sys, a1_sys, a0_sys, b0_sys = sys_params
            y_sys_sim = solver.simulate_forced(solver.t_eval, a2_sys, a1_sys, a0_sys, b0_sys, u_ff, y0)
            
            J = np.sum((y_sys_sim - y_shifted)**2)
            J_array.append(J)

            # 4. Optimal FF computation (using Inverse Dynamics)
            new_ff, extrema, u_opt_full, y_tgt = solver.calculate_optimal_ff(tgt_params, sys_params, y0)
            self.current_ff_matrix[dof_idx] = new_ff
            extrema_list.append(extrema)

            debug_info.append({
                't': solver.t_eval,
                'y_data': raw_y,
                'y_sys': y_sys_sim + Pf,
                'y_tgt': y_tgt + Pf,
                'J': J
            })

        total_J = sum(J_array)
        
        if self.min_J_sum == float('inf'):
            diff_msg = "(初回)"
        else:
            diff = total_J - self.min_J_sum
            if diff < 0:
                diff_msg = f"(前回ベストより {-diff:.2f} 改善！)"
            else:
                diff_msg = f"(前回ベストより {diff:.2f} 悪化)"

        self.get_logger().info(f"内側ループ {self.current_inner+1} 完了: 今回のJ = {total_J:.2f}, これまでのベストJ = {self.min_J_sum if self.min_J_sum != float('inf') else total_J:.2f} {diff_msg}")

        if total_J < self.min_J_sum:
            self.min_J_sum = total_J
            self.best_ff_matrix = [row[:] for row in self.current_ff_matrix]
            self.best_extrema = extrema_list[:]
            self.best_debug_data = debug_info

        self.current_inner += 1
        
        # 内側ループ終了判定
        if total_J < 10.0 or self.current_inner >= self.max_inner_iter:
            self.get_logger().info(f"外側ループ {self.current_outer+1} 完了！最高結果をCSV/Excelに保存します。")
            self.save_optimal_results_to_csv()
            if self.DEBUG_EXCEL:
                self.save_debug_to_excel()
            
            self.current_outer += 1
            self.current_inner = 0
            self.min_J_sum = float('inf')
            
            if self.current_outer >= self.max_outer_iter:
                self.state = "FINISHED"
                return
            else:
                # Next outer loop target
                self.target_pot = self.get_next_target_positions()
                # reset ff for new target
                self.current_ff_matrix = [[0.0]*5 for _ in range(24)]
                
        self.state = "INIT_ROBOT"

    def save_optimal_results_to_csv(self):
        dof_map = []
        for i in range(1, 4):
            for j in range(6): dof_map.append((i-1)*6 + j)
        for i in [4, 5]:
            for j in range(3): dof_map.append(18 + (i-4)*4 + j)

        row_data = {}
        for dof_idx in range(24):
            Pi = self.initial_pot[dof_map[dof_idx]]
            Pf = self.target_pot[dof_map[dof_idx]]
            t1, y1, t2, y2 = self.best_extrema[dof_idx]
            
            row_data[f'Init_{dof_idx+1}'] = Pi
            row_data[f'Target_{dof_idx+1}'] = Pf
            row_data[f'T_{dof_idx+1}'] = self.T
            row_data[f't1_{dof_idx+1}'] = t1
            row_data[f'y1_{dof_idx+1}'] = y1
            row_data[f't2_{dof_idx+1}'] = t2
            row_data[f'y2_{dof_idx+1}'] = y2
        
        df = pd.DataFrame([row_data])
        header = not os.path.exists(self.csv_path)
        df.to_csv(self.csv_path, mode='a', header=header, index=False)

    def save_debug_to_excel(self):
        sheet_name = f"Iter_{self.current_outer+1}"
        ws = self.debug_wb.create_sheet(title=sheet_name)
        
        # Simple table for J values
        ws.cell(row=1, column=1, value="DOF")
        ws.cell(row=1, column=2, value="Best J")
        for dof_idx in range(24):
            ws.cell(row=dof_idx+2, column=1, value=dof_idx+1)
            ws.cell(row=dof_idx+2, column=2, value=self.best_debug_data[dof_idx]['J'])

        # Plot for all DOFs
        for dof_idx in range(24):
            data = self.best_debug_data[dof_idx]
            
            plt.figure(figsize=(6, 4))
            plt.plot(data['t'], data['y_data'], label='Actual Data')
            plt.plot(data['t'], data['y_tgt'], '--', label='Target Model (zeta=1)')
            plt.plot(data['t'], data['y_sys'], ':', label='System Model')
            plt.title(f"DOF {dof_idx+1} (J = {data['J']:.2f})")
            plt.legend()
            
            img_path = f"/tmp/plot_iter{self.current_outer+1}_dof{dof_idx+1}.png"
            plt.savefig(img_path)
            plt.close()
            
            img = OpenpyxlImage(img_path)
            # 2列に分けて配置する (例: D列とM列)
            col = "D" if dof_idx % 2 == 0 else "M"
            row_idx = 2 + (dof_idx // 2) * 22
            ws.add_image(img, f"{col}{row_idx}")

def resolve_csv_file():
    import tkinter as tk
    from tkinter import filedialog
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)

    print("====================================================")
    print("【最適制御実施前手順 1】結果記録用CSVの選択および生成")
    print("1: 既存の結果CSVファイルを選択して追記する")
    print("2: 新規保存先フォルダを選択してCSVファイルを生成する")
    print("====================================================")
    choice = input("モードを選択してください (1 または 2): ").strip()

    if choice == '1':
        file_path = filedialog.askopenfilename(
            title="既存の結果CSVファイルを選択してください",
            filetypes=[("CSV Files", "*.csv")]
        )
        if not file_path:
            print("ファイル未選択のため終了します。")
            sys.exit(1)
        return file_path
    else:
        folder_path = filedialog.askdirectory(title="結果のCSVファイルを保存するフォルダを選択してください")
        if not folder_path:
            print("フォルダ未選択のため終了します。")
            sys.exit(1)
        file_name = input("新規作成するCSVファイル名を入力してください (例: result.csv): ").strip()
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
    mode = input("> ")

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

# 2026/06/26で一番良い（フィッティングするFF入力には制約はなし）