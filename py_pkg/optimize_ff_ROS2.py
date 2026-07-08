#!/usr/bin/env python3
import os # ファイルパスの操作(OSとのやり取り)
import sys # システム関連の操作(システムの設定や環境変数の操作)
import csv # CSVファイルの読み書き
import time # 時間関連の操作(時間の取得や待機)
import traceback # エラー情報の取得(エラー内容を文字列として取得)
import threading # スレッド関連の操作(スレッドの作成や管理)
import tkinter as tk # GUIライブラリ(ウィンドウの作成や管理)
from tkinter import filedialog # GUIの操作(ファイルの選択や保存)
from concurrent.futures import ProcessPoolExecutor # 並行処理の実行(並行処理の実行)

import rclpy # ROS2の初期化(ROS2の初期化)
from rclpy.node import Node # ROS 2のノードの作成(ROS 2のノードの作成)
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy # ROS 2のQoSの設定
from std_msgs.msg import UInt16MultiArray, Float32MultiArray # ROS 2のメッセージの作成(ROS 2のメッセージの作成)

import numpy as np # 数値計算のライブラリ(数値計算のライブラリ)
from scipy.optimize import curve_fit, minimize, NonlinearConstraint, differential_evolution # curve_fit：モデル同定、minimize：最適化、NonlinearConstraint：制約条件、differential_evolution：大域最適化

# Excelデバッグ出力用のインポート
import openpyxl
from openpyxl.chart import LineChart, Reference

# ==============================================================================
# 1. 数値計算・最適化エンジン (マルチプロセス用独立クラス)
# ==============================================================================
# モデル同定、シミュレーション、最適制御、FF生成を行う
class MathematicalSolver:
    def __init__(self, T):
        self.T = T # 入力したFF制御入力時間
        self.dt = 0.01  # 100Hz想定のサンプリングステップ
        self.t_eval = np.arange(0, 5.0, self.dt) # 0から5.0までの時間を0.01秒ごとに分割した配列を作成

    # def check_system_stability(self, sys_params, dof_idx):
    #     """
    #     推定伝達関数の極を解析して
    #     爆発の原因候補を表示する
    #     """

    #     a0, a1, a2, _, _, _ = sys_params

    #     # 分母多項式
    #     den = [1.0, a2, a1, a0]

    #     # 極
    #     poles = np.roots(den)

    #     print("\n================================================")
    #     print(f"DOF {dof_idx+1} : Pole Analysis")
    #     print("poles =", poles)

    #     real_parts = np.real(poles)

    #     # ------------------------------------------------
    #     # 1. 不安定判定
    #     # ------------------------------------------------
    #     if np.any(real_parts > 0):
    #         print("[WARNING]")
    #         print("推定された伝達関数が不安定です")
    #         print("→ 最有力原因")
    #         print("→ 正の実部を持つ極があります")
    #         print("→ オイラー積分で必ず発散します")

    #     # ------------------------------------------------
    #     # 2. 極が速すぎる
    #     # ------------------------------------------------
    #     max_speed = np.max(np.abs(real_parts))

    #     if max_speed > 100:
    #         tau = 1.0 / max_speed

    #         print("[WARNING]")
    #         print("極が非常に速いです")
    #         print(f"最大極速度 = {max_speed:.2f}")
    #         print(f"時定数 τ ≈ {tau:.6f} s")

    #         if tau < self.dt:
    #             print("→ 時定数がdtより小さい")
    #             print("→ 離散化誤差で爆発する可能性があります")

    #     # ------------------------------------------------
    #     # 3. Euler安定条件
    #     # ------------------------------------------------
    #     for p in poles:

    #         if np.real(p) < 0:

    #             dt_limit = 2.0 / abs(np.real(p))

    #             if self.dt > dt_limit:
    #                 print("[WARNING]")
    #                 print("dt が大きすぎる可能性があります")
    #                 print(f"極 = {p}")
    #                 print(f"Euler安定限界 ≈ {dt_limit:.6f}")
    #                 print(f"現在 dt = {self.dt}")

    #     print("================================================\n")

    # ある3次遅れ系を時間方向へシミュレーションするための関数（目標モデルとシステムモデル）
    def _simulate_core(self, t_array, params, is_target_model, init_pot, target_pot, ff_params=None): # 引数（シミュレーション時間、モデルのパラメータ[目標モデル[K, T1, wn]、システムモデル[a0,a1,a2,b0,b1,b2]]、Trueなら目標モデル・Falseならシステムモデル、初期位置、目標位置、FF入力係数）
        """
        誤差空間（偏差系）での3次遅れシミュレーション
        """
        # 初期誤差
        e0 = float(init_pot - target_pot)

        # シミュレーション結果を保存するリスト
        y_sim = []

        # 目標モデルおよびシステムモデル共通の3次遅れ系物理パラメータ展開
        if is_target_model:
            K, T1, wn = params 
            zeta_val = 1.0 # 目標モデルは減衰係数を 1.0 に固定
        else:
            K, T1, wn, zeta_val = params # システムモデルは zeta_val もパラメータから受け取る

        # 3次遅れ系の分母・分子係数を物理パラメータから計算
        a0 = (wn ** 2) / T1 # 定数項
        a1 = (wn ** 2) + (2.0 * zeta_val * wn) / T1 # sの係数
        a2 = (2.0 * zeta_val * wn) + 1.0 / T1 # s^2の係数

        # 3次遅れ系の分子係数
        b0 = (K * (wn ** 2)) / T1

        # 状態空間表現（可制御正準系）
        A = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [-a0, -a1, -a2]]) # A行列
        B = np.array([[0.0], [0.0], [1.0]]) # B行列
        C = np.array([[b0, 0.0, 0.0]]) # 出力行列
        
        # 出力方程式 Cx0 = e0 を満たす最小ノルム解による厳密な初期化
        x = np.linalg.pinv(C) @ np.array([[e0]])  # ムーア・ペンローズ擬似逆行列(「初期誤差 e0 を正しく再現しながら、一番無理のない初期状態を計算する」)

        # シミュレーション開始
        for ts in t_array:
            if is_target_model:
                u = 0.0 # 自由応答
            else:
                if ff_params is not None: # FF入力が用意されているなら以下を実行
                    a, b, c, d, e = ff_params 
                    u = a*(ts**5) + b*(ts**4) + c*(ts**3) + d*(ts**2) + e*ts if ts <= self.T else 0.0  # FF入力を計算（FF制御入力時間以内のみ）
                else: 
                    u = 0.0
            
            # 4次ルンゲ・クッタ法 (RK4) による高精度数値積分
            k1 = np.dot(A, x) + B * u
            k2 = np.dot(A, x + 0.5 * self.dt * k1) + B * u
            k3 = np.dot(A, x + 0.5 * self.dt * k2) + B * u
            k4 = np.dot(A, x + self.dt * k3) + B * u
            x = x + (self.dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

            # if not np.all(np.isfinite(x)):
            #     raise RuntimeError("simulation diverged")

            # if np.max(np.abs(x)) > 1e6:
            #     raise RuntimeError("simulation exploded")

            e_val = np.dot(C, x)[0, 0]  # 状態ベクトルから出力（誤差）を計算
            y_sim.append(e_val)  # シミュレーションした誤差を保存

        return np.array(y_sim) # PythonのリストをNumpy配列へ変換

    # 測定データから3次遅れ系のパラメータを同定する関数
    def fit_3rd_order_system(self, t, y_data, is_step_input, init_pot, target_pot, ff_params=None, dof_idx=None): # 引数（時間データ、測定データ、Trueなら目標モデル・Falseならシステムモデル、初期位置、目標位置、FF入力係数、自由度番号）
        """
        Multi-start（複数初期値探索）による完全一般化システム同定（複数の初期値から最適化を開始し、一番良かったものを採用する）
        """
        # 目標位置を原点(0)に平行移動
        adjusted_e_data = y_data - target_pot

        # 目標モデルの場合
        if is_step_input:
            # curve_fitに渡すための関数を作成（curve_fitはf(x,param1,param2,...)という形式の関数しか受け付けない）
            def simulate_target(t_array, K, T1, wn): # 引数（時間配列、ゲインK、一次遅れ系時定数、二次遅れ系固有角振動数）
                return self._simulate_core(t_array, [K, T1, wn], True, init_pot, target_pot) # 実際のデータからシミュレーションを実行（返り値は誤差軌道）
            
            # パラメータ[K, T1, ωn]の探索範囲
            bounds_low = [0.01, 0.001, 0.1] # 下限
            bounds_high = [10.0, 2.0, 100.0] # 上限
            
            # 3種類の異なるパラメータ[K, T1, ωn]初期値からマルチスタート
            seeds = [
                [1.0, 0.1, 15.0],
                [0.5, 0.5, 5.0],
                [2.0, 0.02, 40.0]
            ]
            
            # これまでで一番良かったパラメータを保存
            best_popt = None

            # RESで用いる初期の最小値である正の無限
            min_res = float('inf')

            # print(f"\n========== DOF {dof_idx+1} : Target Model Fitting ==========")
            
            # seedsパラメータからcurve_fitを実行
            for seed in seeds:
                try:
                    # seedsパラメータからcurve_fitを実行
                    p0 = np.clip(seed, bounds_low, bounds_high).tolist()

                    # シミュレーション結果と測定データとの差が最小になるパラメータを探す
                    popt, _ = curve_fit(simulate_target, t, adjusted_e_data, p0=p0, bounds=(bounds_low, bounds_high), maxfev=5000) # 引数（誤差軌道、時間、実際に測定した誤差、探索開始位置、探索範囲、最大評価回数）

                    # 最最適化で求めたパラメータを使って、もう一度シミュレーションを実行
                    y_sim = simulate_target(t, *popt) # poptは133行目で得られた最適パラメータ

                    # シミュレーションと実測データのズレRSSを計算（残差平方和）
                    res = np.sum((adjusted_e_data - y_sim) ** 2)
                    
                    # ログを表示
                    # print(f"[DOF {dof_idx+1}]" f"[Target Model] Seed={seed}")
                    # print(f"[DOF {dof_idx+1}]" f"[Target Model] Parameters={popt}")
                    # print(f"[DOF {dof_idx+1}]" f"[Target Model] RSS={res:.6f}")

                    K, T1, wn = popt
                    zeta = 1.0

                    a0 = (wn**2) / T1
                    a1 = (wn**2) + (2.0 * zeta * wn) / T1
                    a2 = (2.0 * zeta * wn) + (1.0 / T1)

                    poles = np.roots([1, a2, a1, a0])

                    if np.any(np.real(poles) >= 0):
                        print(f"[DOF {dof_idx+1}][Target Model] unstable model rejected")
                        continue

                    # 収束判定を評価（各seedで評価）
                    if res < min_res:
                        min_res = res # 現在の最小残差
                        best_popt = popt # 現在の一番良かったパラメータ
                except Exception as e:
                    #print(f"[DOF {dof_idx+1}]" f"[Target Model][ERROR] Seed {seed} failed: {e}")
                    continue
            
            # どの seed でもフィッティングに成功しなかった場合の処理
            if best_popt is None:
                print(f"[DOF {dof_idx+1}]" "[Target Model][WARNING] Target model fitting failed for all seeds.")
                print(f"[DOF {dof_idx+1}]" "[Target Model][WARNING] Using default parameters: [1.0, 0.1, 15.0]")
                best_popt = np.array([1.0, 0.1, 15.0]) # すべて失敗したら適当な初期値を返す
            #else:
                #print(f"[DOF {dof_idx+1}]" f"[Target Model] Best parameters = {best_popt}")
                # print(f"[DOF {dof_idx+1}]" f"[Target Model] Minimum RSS = {min_res:.6f}")
            
            # パラメータ[K,T1,ωn]を返す
            return best_popt

        # システムモデルの場合   
        else:
            # curve_fitに渡すための関数を作成（curve_fitはf(x,param1,param2,...)という形式の関数しか受け付けない）
            def simulate_system(t_array, K, T1, wn, zeta_val):  # 引数（時間配列、３次遅れ系パラメータ）
                return self._simulate_core(t_array, [K, T1, wn, zeta_val], False, init_pot, target_pot, ff_params=ff_params) # 実際のデータからシミュレーションを実行（返り値は誤差軌道）
            
            # パラメータ[K, T1, wn, zeta_val]の探索範囲（広めに設定）
            bounds_low = [0.01, 0.001, 0.5, 0.1]  # 下限
            bounds_high = [5.0, 2.0, 80.0, 5.0]   # 上限

            # ========================================================
            # Stage 1: Differential Evolution（大域探索）
            # ========================================================
            # 集団ベースの大域最適化で、局所解を回避して良好な初期値を発見する
            def residual_func(params):
                """大域探索用の残差平方和を計算する目的関数"""
                try:
                    with np.errstate(over='ignore', invalid='ignore'):  # DE探索中の発散警告を抑制
                        y_sim = self._simulate_core(t, list(params), False, init_pot, target_pot, ff_params=ff_params)
                        if not np.all(np.isfinite(y_sim)):
                            return 1e15  # 発散した場合は巨大ペナルティ
                        rss = float(np.sum((adjusted_e_data - y_sim) ** 2))

                    # 安定性チェック（不安定モデルや高速すぎる極を持つモデルにペナルティ）
                    K_est, T1_est, wn_est, zeta_est = params
                    a0_chk = (wn_est ** 2) / T1_est
                    a1_chk = (wn_est ** 2) + (2.0 * zeta_est * wn_est) / T1_est
                    a2_chk = (2.0 * zeta_est * wn_est) + 1.0 / T1_est
                    poles = np.roots([1, a2_chk, a1_chk, a0_chk])

                    if np.any(np.real(poles) >= 0):
                        return 1e15  # 不安定モデルにペナルティ
                    if np.max(np.abs(np.real(poles))) > 100:
                        return 1e15  # 高速すぎる極にペナルティ

                    return rss
                except Exception:
                    return 1e15  # 例外発生時は巨大ペナルティ

            de_bounds = list(zip(bounds_low, bounds_high))
            de_result = differential_evolution(
                residual_func,
                bounds=de_bounds,
                seed=42,         # 再現性のための乱数シード
                maxiter=300,     # 最大反復回数
                tol=1e-8,        # 収束判定閾値
                polish=False,    # polishing は Stage 2 の curve_fit で行う
                popsize=20       # 集団サイズ（探索の多様性を確保）
            )

            # ========================================================
            # Stage 2: curve_fit（局所リファイン）
            # ========================================================
            # DE結果を起点として、勾配ベースの局所最適化で精密フィッティングを行う
            best_popt = None
            min_res = float('inf')

            # DE結果を最優先シードとし、従来のマルチスタートシードも追加（多様性確保）
            refine_seeds = [de_result.x.tolist()]
            refine_seeds.extend([
                [1.0, 0.1, 15.0, 1.0],   # 標準（目標モデルに近い臨界制動系）
                [0.5, 0.5, 5.0,  0.5],   # 緩慢・低減衰系
                [2.0, 0.02, 40.0, 1.5],  # 高速・過制動系
                [1.0, 0.2, 10.0, 0.2],   # 強く振動する系
            ])

            for seed in refine_seeds:
                try:
                    p0 = np.clip(seed, bounds_low, bounds_high).tolist()

                    # シミュレーション結果と測定データとの差が最小になるパラメータを探す
                    popt, _ = curve_fit(simulate_system, t, adjusted_e_data, p0=p0, bounds=(bounds_low, bounds_high), maxfev=5000)

                    # ------------------------
                    # 安定性および高速極チェック
                    # ------------------------
                    K_est, T1_est, wn_est, zeta_est = popt
                    a0 = (wn_est ** 2) / T1_est
                    a1 = (wn_est ** 2) + (2.0 * zeta_est * wn_est) / T1_est
                    a2 = (2.0 * zeta_est * wn_est) + 1.0 / T1_est

                    poles = np.roots([1, a2, a1, a0])

                    if np.any(np.real(poles) >= 0):
                        print(f"[DOF {dof_idx+1}][System Model] unstable model rejected")
                        continue

                    if np.max(np.abs(np.real(poles))) > 100:
                        print(f"[DOF {dof_idx+1}][System Model] too fast pole rejected")
                        continue

                    # シミュレーション実行
                    y_sim = simulate_system(t, *popt)

                    # シミュレーションと実測データのズレRSSを計算（残差平方和）
                    res = np.sum((adjusted_e_data - y_sim) ** 2)

                    # 収束判定を評価（各seedで評価）
                    if res < min_res:
                        min_res = res
                        best_popt = popt
                except Exception as e:
                    continue

            # フォールバック: curve_fit が全失敗した場合、DE結果を直接採用
            if best_popt is None:
                if de_result.fun < 1e14:
                    best_popt = np.array(de_result.x)
                    print(f"[DOF {dof_idx+1}][System Model] curve_fit refinement failed. Using DE result directly (RSS={de_result.fun:.6f})")
                else:
                    print(f"[DOF {dof_idx+1}][System Model][WARNING] All fitting methods failed. Using default parameters: [1.0, 0.1, 15.0, 1.0]")
                    best_popt = np.array([1.0, 0.1, 15.0, 1.0])

            # パラメータ[K, T1, wn, zeta_val]を返す
            return best_popt
    
    # 最適なFF入力を計算する関数
    def solve_optimal_control(self, tgt_params, sys_params, init_pot, target_pot): # 引数（目標モデル、システムモデル、初期位置、目標位置）
        """
        随伴変数法を用いた最適制御入力の反復導出（オイラー・ラグランジュ方程式（Pontryaginの最小原理）を数値的に解く方法）
        """
        # 1. 目標モデルから理想誤差軌道を生成
        y_target = self._simulate_core(self.t_eval, tgt_params, True, init_pot, target_pot) # 引数(時間、目標モデルのパラメータ、True、初期位置、目標位置)
        
        # 2. システムモデルの状態空間行列を再構築
        K, T1, wn, zeta = sys_params
        a0 = (wn **2) / T1 # 定数項
        a1 = (wn **2) + (2.0 * zeta * wn) /T1 # sの係数
        a2 = (2.0 * zeta * wn) + 1.0 / T1 # s^2の係数

        # 3次遅れ系の分子係数
        b0 = (K*(wn**2))/T1
        b1 = 0.0
        b2 = 0.0

        A = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [-a0, -a1, -a2]]) # A行列
        B = np.array([[0.0], [0.0], [1.0]]) # B行列
        C = np.array([[b0, b1, b2]]) # C行列
        
        # 初期誤差
        e0 = float(init_pot - target_pot)

        # 初期状態
        x0 = np.linalg.pinv(C) @ np.array([[e0]])
        
        # サンプル数
        N = len(self.t_eval)

        # FF入力の初期値
        u_iter = np.zeros(N)

        # 入力ペナルティ（正規化定数）
        rho = 0.1
        
        # 3. 反復最適化ループ (収束判定を入れて条件を満たすまで実行)
        max_adj_iter = 200 # 最大反復回数
        adj_iter_cnt = 0 # 現在の反復回数
        error_criterion = float('inf') # 収束基準の初期値
        
        # 収束条件を満たさない、かつ、200回未満なら繰り返す
        while error_criterion > 1e-6 and adj_iter_cnt < max_adj_iter:
            # 前回の入力を保存
            u_old = u_iter.copy()
            
            # A. 順方向シミュレーション
            x_traj = np.zeros((3, N)) # 状態軌道 x(t) を保存する配列
            x_curr = x0.copy() # 現在の状態を初期状態にする
            for k in range(N): # 0秒から最後まで1サンプルずつ計算
                x_traj[:, k] = x_curr.ravel() # 現在の状態
                dxdt = np.dot(A, x_curr) + B * u_iter[k] # 状態方程式
                x_curr += dxdt * self.dt # オイラー法で積分
            
            # B. 随伴方程式の逆方向積分: lambda(T)=0 から開始（「現在のFF入力で動いた結果に対して、どのように入力を修正すれば評価関数が小さくなるか」を計算）
            lam_traj = np.zeros((3, N)) # λ(t) = [λ1, λ2, λ3]を保存する配列
            lam_curr = np.zeros((3, 1)) # 終端条件
            for k in reversed(range(N)): # 随伴方程式であるため、逆向きに積分
                # 誤差の勾配を計算
                err = np.dot(C, x_traj[:, k].reshape(3, 1))[0, 0] - y_target[k]
                # x_traj[:, k].reshape(-1,1)：順方向シミュレーションで保存していたx_trajから時刻kの状態を取り出し、(3,1)の列ベクトルにする
                # y = float(C @ x)：出力方程式を計算
                # err = y - y_target[k]：目標軌道との差を計算

                # 随伴方程式: dλ/dt = -A^T * λ - 2 * C^T * (y - y_ref)
                dl_dt = -np.dot(A.T, lam_curr) - 2.0 * C.T * err
                lam_curr -= dl_dt * self.dt # 逆時間積分（オイラー法）
                lam_traj[:, k] = lam_curr.ravel() # その状態軌道に対する随伴変数の時間変化（逆方向積分）を保存
            
            # C. 入力更新: u = -0.5/rho * B^T * λ
            u_iter = -0.5 / rho * (B.T @ lam_traj).flatten()
            u_iter = np.clip(u_iter, -255, 255) # 入力制限 ※改善の余地あり
            
            # 収束判定の計算 (入力ベクトルの差分のL2ノルム) 
            # 「今回更新した入力」が「前回の入力」とほとんど同じになったかを調べる
            error_criterion = float(np.linalg.norm(u_iter - u_old) / (np.linalg.norm(u_old) + 1e-10))
            adj_iter_cnt += 1 # 1回反復したのでカウントを1増やす
        
        # 4. 5次多項式フィッティング (非線形制約条件および極値条件の統合)
        t_ff = self.t_eval[self.t_eval <= self.T] # FF入力が有効な時間だけ取り出す
        u_target_ff = u_iter[:len(t_ff)] # FF入力が有効な時間の波形を取り出す
        
        # 数値計算上の極値点情報（目標軌道から初期探索用として抽出）
        # 元のシーケンサが期待する出力形式に合わせるため、極値の位置をここで仮計測
        u_grad = np.gradient(u_target_ff, self.dt) # 勾配を計算
        zero_crossings = np.where(np.diff(np.sign(u_grad)))[0] # 勾配の符号が変化したところを探す
        valid_idx = [idx for idx in zero_crossings if 0 < t_ff[idx] < self.T] # 開始点や終了点は除外し、内部だけの極値を採用

        # 1つ目の極値（山または谷）が現れる時刻の初期値を決める
        t1_init = t_ff[valid_idx[0]] if len(valid_idx) > 0 else self.T * 0.33 # 極値が見つかればその時間を使い、見つからなければ全体時間の約1/3を仮の極値時刻とする

        # 2つ目の極値（山または谷）が現れる時刻の初期値を決める
        t2_init = t_ff[valid_idx[1]] if len(valid_idx) > 1 else self.T * 0.66 # 極値が見つかればその時間を使い、見つからなければ全体時間の約2/3を仮の極値時刻とする

        # 山と谷の順序を判定
        idx_max = np.argmax(u_target_ff) # 山部分
        idx_min = np.argmin(u_target_ff) # 谷部分

        if idx_max < idx_min: # 山が先，谷が後
            y1_default = np.max(u_target_ff)
            y2_default = np.min(u_target_ff)
        else: # 谷が先，山が後
            y1_default = np.min(u_target_ff)
            y2_default = np.max(u_target_ff)

        # 1つ目の極値の高さを決める
        y1_init = (u_target_ff[valid_idx[0]] if len(valid_idx) > 0 else y1_default * 0.5) # 極値が見つかればその高さを使い、見つからなければ最大or最小値の1/2を極値とする

        # 2つ目の極値の高さを決める
        y2_init = (u_target_ff[valid_idx[1]] if len(valid_idx) > 1 else y2_default * 0.5) # 極値が見つかればその高さを使い、見つからなければ最大or最小値の1/2を極値とする
        
        # SLSQPが最小化する評価関数
        def polynomial_fit_obj(p):
            a, b, c, d = p # パラメータを取得
            # 条件2. 点(0,0), (T,0)を通る拘束を満たすための末端係数eの決定
            e = -(a * (self.T ** 4) + b * (self.T ** 3) + c * (self.T ** 2) + d * self.T)

            # 多項式を計算
            u_poly = a*(t_ff**5) + b*(t_ff**4) + c*(t_ff**3) + d*(t_ff**2) + e*t_ff
            
            # 最小二乗誤差の計算（この値を最小化）
            loss = np.sum((u_target_ff - u_poly) ** 2)
            
            # 条件1. 0 <= t <= T で極値が2つしか持たない（導関数の零点が2つのみ）
            # 導関数 5at^4 + 4bt^3 + 3ct^2 + 2dt + e = 0 の0からTの範囲の根を評価
            poly_deriv_coeffs = [5*a, 4*b, 3*c, 2*d, e] # 導関数係数
            roots = np.roots(poly_deriv_coeffs) # u'(t)=0を解く
            real_roots_in_range = [r.real for r in roots if np.isreal(r) and 0 < r < self.T] # 実数解だけ抽出
            # 極値が2個か確認（極値が2個以外ならペナルティを与える）
            if len(real_roots_in_range) != 2:
                loss += 1e7 * (abs(len(real_roots_in_range) - 2) + 1)

            # 評価値を返す
            return loss

        # NonlinearConstraint 用の関数定義 (時間軸 t_ff 上の全点の多項式出力を返す)
        # 「この係数で作られる5次多項式は、全時間で入力制限を守っているか？」を判定
        def input_constraint_func(p):
            a, b, c, d = p # SLSQPが現在試している係数を取得
            e = -(a * (self.T ** 4) + b * (self.T ** 3) + c * (self.T ** 2) + d * self.T) # eを計算
            return a*(t_ff**5) + b*(t_ff**4) + c*(t_ff**3) + d*(t_ff**2) + e*t_ff # 5次多項式を計算

        # すべての時間点で -255.0 <= u_poly <= 255.0 となる非線形制約を定義
        const_nonlinear = NonlinearConstraint(input_constraint_func, -255.0, 255.0) # 引数（制約値を返す関数、下限、上限）
            
        # 制約付き最適化をサポートする SLSQP メソッドを使用
        res = minimize(polynomial_fit_obj, x0=[0.0, 0.0, 0.0, 0.0], method='SLSQP', constraints=[const_nonlinear]) # 引数（目的関数、係数の初期値、使用するアルゴリズム、制約条件）

        # 最適パラメータ
        a, b, c, d = res.x # 最適パラメータを取得
        e = -(a * (self.T ** 4) + b * (self.T ** 3) + c * (self.T ** 2) + d * self.T) # eを計算
        
        # FF入力の5次関数のパラメータを返す
        return [a, b, c, d, e]

# 1自由度(DOF)分の最適化処理をまとめたメイン関数
def execute_dof_pipeline(args): # 引数（外から値を受け取るための変数：コマンドラインから入力した情報）
    """単一の自由度（DOF）に対する計算サンドボックス（他のDOFとは独立して計算する）関数"""
    # アンパック（args[タプル]を分解して格納）
    dof_idx, time_series_y, current_ff, T, init_pot, target_pot = args # 自由度番号、5秒間取得した角度データ、現在使っているFFパラメータ、FF入力時間、初期値謂、目標位置
    try:
        # MathematicalSolverクラスのインスタンスを作成
        solver = MathematicalSolver(T)

        # シミュレーション用の時間軸を作成
        t = np.linspace(0, 5.0, len(time_series_y))

        # 目標モデル（減衰係数=1）のパラメータを推定する
        tgt_params_init = solver.fit_3rd_order_system(t, time_series_y, is_step_input=True, init_pot=init_pot, target_pot=target_pot, dof_idx=dof_idx) # 引数（時間軸、実際のデータ、目標モデルを指定、初期位置、目標位置、自由度番号）

        # システムモデルの推定
        sys_params_init = solver.fit_3rd_order_system(t, time_series_y, is_step_input=False, init_pot=init_pot, target_pot=target_pot, ff_params=current_ff, dof_idx=dof_idx) # 引数（時間軸、実際のデータ、システムモデルを指定、初期位置、目標位置、FF制御パラメータ、自由度番号）

        # 目標モデルとシステムモデルの差が最小になる5次多項式の係数を探索
        opt_ff = solver.solve_optimal_control(tgt_params_init, sys_params_init, init_pot=init_pot, target_pot=target_pot) # 引数（目標モデル、システムモデル、初期位置、目標位置）
        
        # 最適5次多項式のパラメータを取得
        a, b, c, d, e = opt_ff

        # 極値（山・谷）の時刻を求める
        roots = np.roots([5*a, 4*b, 3*c, 2*d, e]) # 極値を計算
        valid_roots = sorted([r.real for r in roots if np.isreal(r) and 0 < r < T]) # 実数かつ時間範囲内だけ残す
        
        # 極値時刻を取り出す
        t1, t2 = (valid_roots[0], valid_roots[1]) if len(valid_roots) == 2 else (0.0, 0.0) # 何もなければ0にする

        # 1つ目の極値の高さを計算
        y1 = a*(t1**5) + b*(t1**4) + c*(t1**3) + d*(t1**2) + e*t1

        # 2つ目の極値の高さを計算
        y2 = a*(t2**5) + b*(t2**4) + c*(t2**3) + d*(t2**2) + e*t2

        # ① 目標モデルfinalの同定（実測データ2に対して、減衰係数1固定で同定）
        tgt_params_final = solver.fit_3rd_order_system(t, time_series_y, is_step_input=True, init_pot=init_pot, target_pot=target_pot, dof_idx=dof_idx)
        
        # ② システムモデルfinalの同定（実測データ2に対して、5次関数FF入力を考慮して同定）
        sys_params_final = solver.fit_3rd_order_system(t, time_series_y, is_step_input=False, init_pot=init_pot, target_pot=target_pot, ff_params=opt_ff, dof_idx=dof_idx)
        
        # ターゲットモデルをもう一度シミュレーション
        y_target_final = solver._simulate_core(solver.t_eval, tgt_params_final, True, init_pot, target_pot)

        # 実システムもシミュレーション
        y_sys_final = solver._simulate_core(solver.t_eval, sys_params_final, False, init_pot, target_pot, opt_ff)

        # 評価関数を計算
        J = float(np.sum((y_target_final - y_sys_final) ** 2))

        # デバッグ用として、実測データ・モデルデータをExcel出力のためにリサンプリングして保持
        # データの長さミスマッチを防ぐため、t_evalと同じ長さ(500点)にリサンプリング、またはt_eval上で評価
        t_eval_len = len(solver.t_eval)
        measured_resampled = np.interp(solver.t_eval, t, time_series_y)
        
        # 結果を返す
        return {
            "status": "SUCCESS", 
            "idx": dof_idx, 
            "params": opt_ff, 
            "extr": [t1, t2, y1, y2], 
            "J": J,
            "debug_data": {
                "t_eval": solver.t_eval.tolist(),
                "measured": measured_resampled.tolist(),                # 実測データ2
                "target_model": (y_target_final + target_pot).tolist(), # 目標モデルfinal ★修正完了
                "system_model": (y_sys_final + target_pot).tolist()     # システムモデルfinal
            }
        }
    except Exception:
        return {"status": "ERROR", "idx": dof_idx, "error": traceback.format_exc()}


# ==============================================================================
# 2. ROS 2 メイン制御シーケンサノード
# ==============================================================================
# ROS2のNodeを継承した、最適制御実験を管理するクラス
class OptimalControlSequencer(Node):
    # ★★★ 簡単な切り替え用フラグ (True: Excel保存有効, False: 無効) ★★★
    DEBUG_EXCEL = True

    # ROS2ノードが起動した瞬間に一度だけ実行される初期化処理
    def __init__(self, csv_path, T, max_iter, target_mode): # 引数（csv_path、FF入力時間T、実験回数、モード選択）
        # 親クラス(Node)の初期化
        super().__init__('optimal_control_sequencer')

        # CSV保存先を保存
        self.csv_path = csv_path

        # FF入力時間を保存
        self.T = T

        # 実験回数を保存
        self.max_iter = max_iter
        
        # "1": プリセット優先, "2": 完全ランダム
        self.target_mode = target_mode 
        
        # 現在の試行回数
        self.current_iteration = 0
        
        # 定常化待機時間
        self.initial_stabilize_time = 10.0

        # データ収集時間
        self.data_collection_time = 5.0

        # シーケンサ状態
        self.state = "INIT_ROBOT"

        # 最小評価関数の初期値
        self.min_J_global = float('inf')

        # 最良結果
        self.best_results_to_save = None

        # デバッグ用 Excel ブックの初期化
        if self.DEBUG_EXCEL:
            self.debug_wb = openpyxl.Workbook()
            # 初期作成されるシートを削除するための参照保持
            self.default_sheet = self.debug_wb.active
            self.excel_path = os.path.splitext(self.csv_path)[0] + "_debug_models.xlsx"

        # ポテンショメータ値の範囲
        self.pot_bounds = [
            (450, 700), (135, 550), (500, 680), (250, 700), (66, 259), (192, 389),
            (70, 200),  (60, 465),  (115, 200), (100, 550), (239, 430), (205, 395),
            (30, 660),  (30, 690),  (110, 830), (3, 630),   (3, 700),   (9, 660),
            (275, 360), (115, 785), (192, 440), (284, 557), (323, 580), (188, 630),
            (375, 500), (300, 490)
        ]
        
        # ロボットが起動した直後の初期姿勢（ホームポジション）
        self.initial_desired_raw = [
            500, 200, 500, 300, 170, 300,
            160, 410, 200, 500, 350, 220,
            300, 250, 400, 350, 420, 400,
            325, 370, 280, 420,
            360, 390, 420, 390
        ]

        # ユーザー定義のプリセット目標姿勢のリスト(例として2つプリセット、24自由度分を定義)
        self.preset_targets = [
            [
                501, 201, 501, 700, 171, 301,
                161, 411, 201, 501, 351, 221,
                301, 251, 401, 351, 421, 401,
                326, 371, 281,
                361, 391, 421
            ]
        ]

        # 最適化対象24自由度のみの最小値
        self.pot_min = (
            [b[0] for b in self.pot_bounds[0:18]] +
            [b[0] for b in self.pot_bounds[18:21]] +
            [b[0] for b in self.pot_bounds[22:25]]
        )

        # 最適化対象24自由度のみの最大値
        self.pot_max = (
            [b[1] for b in self.pot_bounds[0:18]] +
            [b[1] for b in self.pot_bounds[18:21]] +
            [b[1] for b in self.pot_bounds[22:25]]
        )
        
        # 最適化対象24自由度のみの初期姿勢
        self.initial_pot = [float(int(x)) for x in (
            self.initial_desired_raw[0:18] +   
            self.initial_desired_raw[18:21] +  
            self.initial_desired_raw[22:25]    
        )]
        
        # 初回の目標値選択
        self.target_pot = self.get_next_target_positions()

        # 24自由度分のFFパラメータを初期化
        self.current_ff_matrix = [[0.1, 0.0, 0.0, 0.0, 0.0] for _ in range(24)]

        # 受信した時系列データを保存するバッファを生成
        self.buffer_time_series = {f'board{i}': [] for i in range(1, 6)}

        # ROS2通信の品質設定を作成開始
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        # Subscriber生成
        self.sub_b1 = self.create_subscription(UInt16MultiArray, '/board1_tk/pub', lambda msg: self.cb_board(msg, 1), qos_profile)
        self.sub_b2 = self.create_subscription(UInt16MultiArray, '/board2_tk/pub', lambda msg: self.cb_board(msg, 2), qos_profile)
        self.sub_b3 = self.create_subscription(UInt16MultiArray, '/board3_tk/pub', lambda msg: self.cb_board(msg, 3), qos_profile)
        self.sub_b4 = self.create_subscription(UInt16MultiArray, '/board4_tk/pub', lambda msg: self.cb_board(msg, 4), qos_profile)
        self.sub_b5 = self.create_subscription(UInt16MultiArray, '/board5_tk/pub', lambda msg: self.cb_board(msg, 5), qos_profile)

        # Publisher生成
        self.pub_android = self.create_publisher(Float32MultiArray, '/board_android_float/sub', qos_profile) # アンドロイドへ目標位置を送るPublisher
        self.pubs_ff = {i: self.create_publisher(Float32MultiArray, f'/board{i}_FFparam_float/sub', qos_profile) for i in range(1, 6)} # boardごとのFFパラメータ送信用Publisher

        # Timer生成
        self.control_timer = self.create_timer(0.1, self.sequencer_loop)

        # 現在状態に入った時刻を保存
        self.state_start_time = self.get_clock().now()
    
    def get_next_target_positions(self):
        """
        現在の試行回数(current_iteration)と選択されたモードに応じて、次の目標姿勢を取得する
        """
        # モード1 (プリセット優先) かつ プリセットリスト内にまだインデックスがある場合
        if self.target_mode == '1' and self.current_iteration < len(self.preset_targets):
            self.get_logger().info(f"【目標値選択】プリセットパターン {self.current_iteration + 1} を適用します。")
            return [float(int(x)) for x in self.preset_targets[self.current_iteration]]
        else:
            # モード2（完全ランダム）、またはモード1でプリセットをすべて消化し終えた場合
            if self.target_mode == '1':
                self.get_logger().info("【目標値選択】プリセットをすべて消化したため、ここからはランダム目標値を適用します。")
            else:
                self.get_logger().info("【目標値選択】完全ランダム目標値を生成・適用します。")
            return [float(int(x)) for x in np.floor(np.random.uniform(self.pot_min, self.pot_max))]
    
    # 各boardのデータ受信時に呼ばれる関数
    def cb_board(self, msg, board_id):
        if self.state == "COLLECTING": # データ収集中だけ保存する
            if board_id in [4, 5]: # board4,5は6自由度分送られるが、実際に最適化するのは3自由度
                # 不要自由度を除外
                selected = [msg.data[idx] for idx in [0, 1, 2, 6, 7, 8]]

                # 時系列データとして保存
                self.buffer_time_series[f'board{board_id}'].append(selected)
            else:
                # 時系列データとして保存
                self.buffer_time_series[f'board{board_id}'].append(msg.data)

    def sequencer_loop(self):
        # 現在時刻取得
        now = self.get_clock().now()

        # 状態遷移後の経過時間[s]
        elapsed = (now - self.state_start_time).nanoseconds / 1e9

        # 新しい試行開始状態
        if self.state == "INIT_ROBOT":
            self.get_logger().info(f"=== 最最適制御サイクル {self.current_iteration + 1} / {self.max_iter} ===") # 現在試行回数表示
            self.get_logger().info("初期位置データをロボットに送信して安定化待機中...")
            self.publish_target_positions(self.initial_pot) # ホーム姿勢へ移動指令
            self.state = "WAIT_INITIAL_STABILIZE" # 安定化待機へ移行
            self.state_start_time = now # 開始時刻更新

        # ロボットが落ち着くのを待つ状態
        elif self.state == "WAIT_INITIAL_STABILIZE":
            if elapsed >= self.initial_stabilize_time: # 指定時間待機したら次へ
                self.get_logger().info("定常状態を確認。ランダム目標値と現在のFFパラメータを適用します。")
                self.publish_all_ff_parameters() # 現在のFFパラメータを送信
                time.sleep(0.1)
                self.publish_target_positions(self.target_pot) # ランダム目標姿勢を送信
                
                # 全バッファ走査
                for k in self.buffer_time_series: 
                    # 前回データ削除
                    self.buffer_time_series[k].clear()

                # データ収集開始
                self.state = "COLLECTING"

                # 収集開始時刻記録
                self.state_start_time = now

        # 動作データ収集中の状態
        elif self.state == "COLLECTING":
            if elapsed >= self.data_collection_time: # 指定時間取得したら終了
                # ===== デバッグ追加 =====
                for i in range(1, 6):
                    count = len(self.buffer_time_series[f'board{i}'])
                    self.get_logger().info(
                        f"board{i} received data count = {count}"
                    )
                # =======================

                self.state = "PROCESSING" # 解析状態へ
                self.get_logger().info("5秒間の動作データを取得完了。最適化パイプラインを展開します。")
                threading.Thread(target=self.dispatch_optimization_pipeline).start() # 最適化処理を別スレッドで開始

        # 最適化終了待ち状態
        elif self.state == "PROCESSING":
            pass

        # 全試行終了
        elif self.state == "FINISHED":
            # ★ 1. 強制終了する前に、まず確実にエクセルファイルを保存する
            if self.DEBUG_EXCEL:
                try:
                    self.get_logger().info("【デバッグ】エクセルファイルを最終保存中...")
                    if self.default_sheet in self.debug_wb.worksheets and len(self.debug_wb.worksheets) > 1:
                        self.debug_wb.remove(self.default_sheet)
                    self.debug_wb.save(self.excel_path)
                    self.get_logger().info(f"デバッグ用エクセルファイルを保存しました: {self.excel_path}")
                except Exception as e:
                    self.get_logger().error(f"デバッグ用エクセル保存に失敗しました: {e}")

            self.get_logger().info("すべての実験試行が正常終了しました。ノードを落とします。")

            self.control_timer.cancel() # タイマーを破棄
            #sys.exit(0) # プログラム終了
            # 安全かつ確実に spin を強制離脱させてコマンドラインに戻す
            raise SystemExit(0)

    # 目標値をロボットへ送信する関数
    def publish_target_positions(self, pot_list): # 引数（24自由度分の目標値）
        # ROS2送信用メッセージを生成
        msg = Float32MultiArray()

        # 長さ26の配列を生成
        data = [0.0] * 26
        
        # 1番目から21番目：そのまま格納 (インデックス 0〜20)
        data[0:21] = pot_list[0:21]     
        
        # 22番目 (インデックス 21)：除外しているので適当な値を入れる
        if self.current_iteration == 0 and self.state == "INIT_ROBOT": # 最初の試行かつ初期化状態か確認
            data[21] = 0.0
        else:
            data[21] = 0.0
        
        # 22自由度目を23番目に、23自由度目を24番目に、24自由度目を25番目に格納 (インデックス 22〜24)
        data[22:25] = pot_list[21:24]  
        
        # 26番目 (インデックス 25)：除外しているので適当な値を入れる
        if self.current_iteration == 0 and self.state == "INIT_ROBOT": # 最初の試行かつ初期化状態か確認
            data[25] = 0.0
        else:
            data[25] = 0.0
        
        # 全要素をfloat型へ変換
        msg.data = [float(int(x)) for x in data]

        # アンドロイドへ送信
        self.pub_android.publish(msg)

        self.get_logger().info(
            f"[TARGET] Sent target positions = {msg.data}"
        )
    
    # 24自由度のFFパラメータを送信する関数
    def publish_all_ff_parameters(self):
        # board1 (DOF1〜6)
        b1_data = [] # 送信用配列
        for i in range(6):
            b1_data.extend(self.current_ff_matrix[0 + i]) # 各自由度のパラメータa,b,c,d,e
            b1_data.append(self.T) # 制御入力時間
        msg1 = Float32MultiArray(data=[float(x) for x in b1_data]) # ROSメッセージ化
        self.pubs_ff[1].publish(msg1) # board1へ送信

        self.get_logger().info(
            f"[FF][board1] {msg1.data}"
        )

        # board2 (DOF7〜12)
        b2_data = [] # 送信用配列
        for i in range(6):
            b2_data.extend(self.current_ff_matrix[6 + i]) # 各自由度のパラメータa,b,c,d,e
            b2_data.append(self.T) # 制御入力時間
        msg2 = Float32MultiArray(data=[float(x) for x in b2_data]) # ROSメッセージ化
        self.pubs_ff[2].publish(msg2) # board2へ送信

        self.get_logger().info(
            f"[FF][board2] {msg2.data}"
        )

        # board3 (DOF13〜18)
        b3_data = [] # 送信用配列
        for i in range(6):
            b3_data.extend(self.current_ff_matrix[12 + i]) # 各自由度のパラメータa,b,c,d,e
            b3_data.append(self.T) # 制御入力時間
        msg3 = Float32MultiArray(data=[float(x) for x in b3_data]) # ROSメッセージ化
        self.pubs_ff[3].publish(msg3) # board3へ送信

        self.get_logger().info(
            f"[FF][board3] {msg3.data}"
        )

        # board4 (DOF19〜21 + 最適化対象外×3)
        b4_data = [] # 送信用配列
        # 1〜18番目: DOF19, 20, 21 (パラメータと時間)
        for i in range(3):
            b4_data.extend(self.current_ff_matrix[18 + i]) # 各自由度のパラメータa,b,c,d,e
            b4_data.append(self.T) # 制御入力時間
        # 19〜23番目: 最適化対象外 (すべて0) -> 24番目: 制御入力時間
        b4_data.extend([0.0] * 5)
        b4_data.append(self.T)
        # 25〜29番目: 最適化対象外 (すべて0) -> 30番目: 制御入力時間
        b4_data.extend([0.0] * 5)
        b4_data.append(self.T)
        # 31〜35番目: 最最適化対象外 (すべて0) -> 36番目: 制御入力時間
        b4_data.extend([0.0] * 5)
        b4_data.append(self.T)
        msg4 = Float32MultiArray(data=[float(x) for x in b4_data]) # ROSメッセージ化
        self.pubs_ff[4].publish(msg4) # board4へ送信

        self.get_logger().info(
            f"[FF][board4] {msg4.data}"
        )

        # board5 (DOF22〜24 + 最適化対象外×3)
        b5_data = [] # 送信用配列
        # 1〜18番目: DOF22, 23, 24 (パラメータと時間)
        for i in range(3):
            b5_data.extend(self.current_ff_matrix[21 + i]) # 各自由度のパラメータa,b,c,d,e
            b5_data.append(self.T) # 制御入力時間
        # 19〜23番目: 最適化対象外 (すべて0) -> 24番目: 制御入力時間
        b5_data.extend([0.0] * 5)
        b5_data.append(self.T)
        # 25〜29番目: 最適化対象外 (すべて0) -> 30番目: 制御入力時間
        b5_data.extend([0.0] * 5)
        b5_data.append(self.T)
        # 31〜35番目: 最適化対象外 (すべて0) -> 36番目: 制御入力時間
        b5_data.extend([0.0] * 5)
        b5_data.append(self.T)
        msg5 = Float32MultiArray(data=[float(x) for x in b5_data]) # ROSメッセージ化
        self.pubs_ff[5].publish(msg5) # board5へ送信

        self.get_logger().info(
            f"[FF][board5] {msg5.data}"
        )
    
    # 最適化パイプライン全体を実行する関数
    def dispatch_optimization_pipeline(self):
        try:
            # 24自由度分の時系列データ格納用
            dof_time_series = [[] for _ in range(24)]

            # 各boardの受信データ数を調べ、最小数を取得（全boardで共通に使える長さに揃える）
            min_length = min(len(self.buffer_time_series[f'board{i}']) for i in range(1, 6))
            
            # データが少なすぎるか判定
            if min_length < 10:
                raise RuntimeError("受信データ点数が極端に不足しています。") # 正常な最適化ができないので中断

            # boardデータを24自由度へ再構成
            for t_idx in range(min_length): # 時刻サンプルを順番に処理
                # board1〜3用
                for b in range(1, 4):
                    # 各boardの6自由度
                    for d in range(6):
                        # 対応自由度へ格納(そのまま格納)
                        dof_time_series[(b-1)*6 + d].append(self.buffer_time_series[f'board{b}'][t_idx][d]) # self.buffer_time_series[f'board{b}'][t_idx][d])：その時刻のポテンショメータ値を追加
                for d in range(3):
                    # board4用
                    dof_time_series[18 + d].append(self.buffer_time_series['board4'][t_idx][d]) # 最適化対象3自由度のみ
                    # board5用
                    dof_time_series[21 + d].append(self.buffer_time_series['board5'][t_idx][d]) # 最適化対象3自由度のみ

            # 並列計算用入力リスト
            tasks = []
            # 24自由度全て処理
            for idx in range(24):
                # 1自由度分の入力をまとめる
                tasks.append((
                    idx, # 自由度番号
                    np.array(dof_time_series[idx], dtype=np.float64), # その自由度の時系列データ
                    self.current_ff_matrix[idx], # 現在のFFパラメータ
                    self.T, # FF制御入力時間
                    self.initial_pot[idx], # 初期位置
                    self.target_pot[idx] # 目標位置
                ))

            # 並列演算開始
            self.get_logger().info("ProcessPoolExecutorを展開。演算中...")
            with ProcessPoolExecutor() as executor: # CPU並列処理開始
                results = list(executor.map(execute_dof_pipeline, tasks)) # 各自由度ごとにexecute_dof_pipeline()を実行

            # 評価関数集計
            errors_detected = False # エラー検知フラグ
            round_J = 0.0 # 今回の24自由度全体の評価関数
            round_results = [] # 正常結果保存用

            # 24自由度結果を順番に確認
            for r in results:
                if r["status"] == "ERROR": # 計算失敗判定
                    self.get_logger().error(f"【自由度 {r['idx']} 演算エラー】\n{r['error']}")
                    errors_detected = True # 異常フラグON
                else: # 正常時
                    round_J += r["J"] # 評価関数加算
                    round_results.append(r) # 正常結果保存

            # エラー確認
            if errors_detected:
                raise ArithmeticError("計算パイプライン中に致命的エラーを検知しました。") # 最適化中断

            # 結果出力
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"Iteration {self.current_iteration + 1}")
            self.get_logger().info(f"Current J = {round_J:.6f}")

            if self.min_J_global != float('inf'):
                ratio = 100.0 * (round_J - self.min_J_global) / self.min_J_global

                if round_J < self.min_J_global:
                    self.get_logger().info(f"BEST J = {self.min_J_global:.6f}")
                    self.get_logger().info(f"Improved by {-ratio:.2f}%")
                else:
                    self.get_logger().info(f"BEST J = {self.min_J_global:.6f}")
                    self.get_logger().info(f"Worse by {ratio:.2f}%")
            else:
                self.get_logger().info("First trial")

            self.get_logger().info("=" * 60)

            # グローバル最良結果更新
            if round_J < self.min_J_global: # 今回の評価関数が過去最小か判定
                self.min_J_global = round_J # 最小値更新
                self.best_results_to_save = round_results # 最良パラメータ保存
                self.get_logger().info(f"==> グローバル最小評価関数 J 更新: {self.min_J_global}") # 更新通知

            # FFパラメータ更新（Jの成否に関わらず、次回の実験のために最新の最適化パラメータを反映）
            for r in round_results: # 24自由度分の計算結果を順番に取り出す
                self.current_ff_matrix[r["idx"]] = r["params"] # 最適化後のFFパラメータで更新する

            # --- 収束判定としきい値・ループ制限の設定 ---
            j_threshold = 100000.0 # 収束とみなす評価関数のしきい値
            max_loops_per_target = 10   # 同一目標姿勢での最大ループ回数（安全弁）
            
            # 同一目標でのカウンタ変数がまだ存在しない場合は初期化
            if not hasattr(self, 'same_target_loop_count'):
                self.same_target_loop_count = 0
            
            # 今回の試行をカウント
            self.same_target_loop_count += 1

            # 条件A: しきい値以下に収束したか
            is_converged = round_J <= j_threshold
            # 条件B: 同一目標での最大ループ回数に達したか
            is_loop_limit = self.same_target_loop_count >= max_loops_per_target

            if is_converged or is_loop_limit:
                if is_converged:
                    self.get_logger().info(f"【収束達成】round_J ({round_J:.4f}) <= しきい値 ({j_threshold})。次のランダム目標へ移行します。")
                else:
                    self.get_logger().warn(f"【ループ上限到達】同一目標での試行が {max_loops_per_target} 回に達したため、収束を諦めて次のランダム目標へ強制移行します。")
                
                # 同一目標カウンターをリセット
                self.same_target_loop_count = 0

                # csv用に保存
                self.csv_initial_pot = self.initial_pot.copy()
                self.csv_target_pot  = self.target_pot.copy()

                # ★ 最もJが小さかったベスト結果をExcelの各シート(Trial)へ記録
                if self.DEBUG_EXCEL and self.best_results_to_save is not None:
                    self.save_debug_to_excel()

                # csvに保存
                self.save_optimal_results_to_csv()

                # 初期位置更新
                self.initial_pot = list(self.target_pot) # 今回の目標位置を、次回の開始位置にする

                # 次の目標値（次の試行）に進むため、Best J 管理用の変数をリセット
                self.min_J_global = float('inf')
                self.best_results_to_save = None
                
                # 試行回数更新（目標をクリア、またはスキップした時のみ全体の進捗を進める）
                self.current_iteration += 1 # 実験回数を1増やす

                # 次の目標値を取得 (進捗current_iterationに応じた値を割り当てる)
                if self.current_iteration < self.max_iter:
                    # 次の目標位置生成
                    self.target_pot = self.get_next_target_positions()

            else:
                self.get_logger().info(f"【未収束】round_J ({round_J:.4f}) > しきい値 ({j_threshold}) [同一目標内での試行: {self.same_target_loop_count}/{max_loops_per_target} 回]。同じ目標位置でFFを再適用します。")
                # initial_pot, target_pot, current_iteration は更新せずそのまま維持

            # 終了判定
            if self.current_iteration >= self.max_iter: # 設定回数に達したか確認
                self.state = "FINISHED" # 終了状態へ
            else: # まだ試行回数に達していない場合
                self.state = "INIT_ROBOT" # 最初の状態へ戻る

        # 途中で何らかのエラーが発生した場合    
        except Exception as e:
            self.get_logger().error(f"シーケンサープロセスが異常停止しました: {e}") # エラー内容出力
            self.state = "FINISHED" # 安全のため終了状態へ

        # エラーの有無に関係なく必ず実行
        finally:
            # 状態開始時刻を更新
            self.state_start_time = self.get_clock().now()

    # 最適化結果をCSVへ保存する関数
    def save_optimal_results_to_csv(self):
        self.get_logger().info(f"最小J結果をCSVに永続化保存中: {self.csv_path}") # ROSログ出力

        # CSV保存中のエラー対策
        try:
            row_data = [] # CSVへ書き込む前の一時保存領域

            # 24自由度を順番に処理
            for idx in range(24):
                r = next(item for item in self.best_results_to_save if item["idx"] == idx) # self.best_results_to_saveの中からidxと一致する自由度の結果を取り出す

                # 特徴量取得
                t1, t2, y1, y2 = r["extr"] # 最適化関数内で抽出した特徴量

                # csvファイルに書き込む初期値・目標値も確実に小数点以下0のfloat型にする
                init_val_fixed = float(int(self.csv_initial_pot[idx]))
                tgt_val_fixed = float(int(self.csv_target_pot[idx]))

                # 保存データ作成
                # 1自由度分の保存データを作る
                dof_tuple = ( 
                    init_val_fixed, # 初期値
                    tgt_val_fixed, # 目標値
                    self.T, # FF制御入力時間
                    t1, t2, y1, y2 # 各特徴量
                )

                # 保存リストへ追加
                row_data.append(dof_tuple)

            # CSVファイルを開く
            # CSVファイルを追記モードで開く
            with open(self.csv_path, 'a', newline='') as f:
                # CSV書き込みオブジェクト作成
                writer = csv.writer(f)

                # CSV1行分を格納
                flat_row = []

                # 24自由度を順番に処理
                for t in row_data:
                    # 要素を横方向へ展開
                    flat_row.extend(t)
                
                # CSVへ1行追加
                writer.writerow(flat_row)

            # 保存完了メッセージ
            self.get_logger().info("CSVファイルの保存に成功しました。")
        except Exception as e: # 保存失敗時
            self.get_logger().error(f"CSV保存中にI/Oエラーが発生しました: {e}")

    def save_debug_to_excel(self):
        """
        [超高速・一括書き込み版] 自由度ごとに完全独立した時間軸を持つマトリクス
        """
        self.get_logger().info(f"【デバッグ】波形データをExcelシートに高速一括展開中... (試行 {self.current_iteration + 1})")
        try:
            sheet_name = f"Trial_{self.current_iteration + 1}"
            ws = self.debug_wb.create_sheet(title=sheet_name)

            # 1. J値サマリー表の作成
            ws.append(["自由度 (DOF)", "評価関数値 J"])
            sorted_results = sorted(self.best_results_to_save, key=lambda x: x["idx"])
            for r in sorted_results:
                ws.append([f"DOF {r['idx'] + 1}", r["J"]])
            
            total_row = len(sorted_results) + 2
            ws.cell(row=total_row, column=1, value="Total J")
            ws.cell(row=total_row, column=2, value=f"=SUM(B2:B{total_row-1})")

            # 2. ヘッダー行の一括構築 (自由度ごとに独立した 4列[Time, Measured, Target, System] を並べる)
            headers = ["", ""]  # A, B列(サマリー用) のスペースをスキップ
            for r in sorted_results:
                dof_num = r['idx'] + 1
                headers.extend([f"Time_DOF{dof_num}", f"Measured_DOF{dof_num}", f"TargetModel_DOF{dof_num}", f"SystemModel_DOF{dof_num}"])
            ws.append(headers)

            # 3. 時系列データの一括パッキングと ws.append による超高速流し込み
            data_len = len(sorted_results[0]["debug_data"]["t_eval"])
            for t_step in range(data_len):
                row_cells = [None, None]  # A, B列用
                for r in sorted_results:
                    debug_data = r["debug_data"]
                    row_cells.append(debug_data["t_eval"][t_step])
                    row_cells.append(debug_data["measured"][t_step])
                    row_cells.append(debug_data["target_model"][t_step])
                    row_cells.append(debug_data["system_model"][t_step])
                ws.append(row_cells)

            # 4. 各自由度の独立グラフを生成してレイアウト
            # データ行はヘッダー（1行目サマリー、total_row行目合計、さらにヘッダー行）を挟むため、実データは total_row + 2 行目からスタート
            start_data_row = total_row + 2
            end_data_row = start_data_row + data_len - 1

            for d_idx, r in enumerate(sorted_results):
                # 自由度ごとに4列ずつずれる (C列が3番目なので index=3 からスタート)
                start_col = 3 + (d_idx * 4)

                # 時間軸 (Time_DOF X) への個別参照
                cats_ref = Reference(ws, min_col=start_col, min_row=start_data_row, max_row=end_data_row)
                
                # データ範囲 (Measured, TargetModel, SystemModel) の個別参照
                data_ref = Reference(ws, min_col=start_col+1, min_row=total_row+1, max_col=start_col+3, max_row=end_data_row)

                chart = LineChart()
                chart.title = f"Model Identification - DOF {r['idx'] + 1}"
                chart.style = 13
                chart.y_axis.title = "POT Value"
                chart.x_axis.title = "Time (s)"
                chart.width = 15
                chart.height = 9
                
                chart.add_data(data_ref, titles_from_data=True)
                chart.set_categories(cats_ref)

                # A列のJ値サマリーの下方に縦並びでグラフを並べる
                insert_cell = f"A{total_row + 3 + (d_idx * 18)}"
                ws.add_chart(chart, insert_cell)

            # 各試行が終了するごとに自動中間保存 (安全確保)
            self.debug_wb.save(self.excel_path)
            self.get_logger().info(f"【デバッグ】独立軸Excelシートの中間保存に成功しました: {self.excel_path}")
        except Exception as e:
            self.get_logger().error(f"【デバッグ】Excel処理中にエラーが発生しました: {e}")


# ==============================================================================
# 3. GUIファイルシステムダイアログ & エントリーポイント
# ==============================================================================
# CSVファイルの保存先を決める関数
def resolve_csv_file():
    root = tk.Tk() # TkinterのGUIウィンドウ生成
    root.withdraw() # メインウィンドウを非表示にする
    root.attributes("-topmost", True) # ファイル選択ダイアログを最前面表示にする

    # 説明表示
    print("====================================================")
    print("【最適制御実施前手順 1】結果記録用CSVの選択および生成")
    print("1: 既存の結果CSVファイルを選択して追記する")
    print("2: 新規保存先フォルダを選択してCSVファイルを生成する")
    print("====================================================")
    choice = input("モードを選択してください (1 または 2): ").strip()

    # モード1：既存CSVへ追記
    if choice == '1':
        file_path = filedialog.askopenfilename( # ファイル選択ダイアログを開く。
            title="既存の結果CSVファイルを選択してください",
            filetypes=[("CSV Files", "*.csv")] # CSVだけ表示
        )
        # キャンセルされたか確認
        if not file_path:
            print("ファイル未選択のため終了します。") # 終了メッセージ
            sys.exit(1) # 異常終了
        return file_path # 選択したCSVパスを返す
    # モード2：新規作成モード
    else:
        folder_path = filedialog.askdirectory(title="結果のCSVファイルを保存するフォルダを選択してください") # フォルダ選択ダイアログ
        if not folder_path: # キャンセル確認
            print("フォルダ未選択のため終了します。")
            sys.exit(1)
        
        # ファイル名生成
        file_path = os.path.join(folder_path, "optimal_control_results.csv")
        
        # ファイル存在確認
        if not os.path.exists(file_path): # まだ存在しない場合のみ作成
            with open(file_path, 'w', newline='') as f: # 新規CSV作成
                writer = csv.writer(f) # CSVライター作成
                headers = [] # ヘッダ格納用

                # 24自由度分ヘッダ作成
                for i in range(1, 25):
                    headers.extend([
                        f'Init_POT_dof{i}', f'Target_POT_dof{i}', f'T_dof{i}', 
                        f't1_dof{i}', f't2_dof{i}', f'y1_dof{i}', f'y2_dof{i}'
                    ])

                writer.writerow(headers) # ヘッダ行を書き込む
        return file_path # 作成したCSVパスを返す

# プログラムのエントリーポイント
def main(args=None):
    # 保存先決定
    csv_file_path = resolve_csv_file()
    
    print("\n【最適制御実施前手順 2】")
    T = float(input("FF制御入力時間 T (秒) を入力してください: ")) # FF入力時間取得
    
    print("\n【最適制御実施前手順 3】")
    max_iterations = int(input("最適制御の総実行回数を入力してください: ")) # 総試行回数取得

    # 目標値指定モードの選択 ---
    print("\n【最適制御実施前手順 4】目標値指定モードの選択")
    print("1: プリセット目標値を優先して与え、足りない分をランダム補填する")
    print("2: 最初からすべてランダムに目標値を与える")
    target_mode = input("モードを選択してください (1 または 2): ").strip()
    while target_mode not in ['1', '2']:
        target_mode = input("無効な入力です。1 または 2 を選択してください: ").strip()

    print("\n【最適制御実施前手順 5】ROS 2 最適制御プログラムをスピンアップします...")

    # ROS2初期化
    rclpy.init(args=args)
    
    # コンストラクタを実行
    node = OptimalControlSequencer(csv_file_path, T, max_iterations, target_mode)
    
    try:
        # ROS2イベントループ開始
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit): # Ctrl+C対応
        print("\nユーザーによるシグナル遮断を検知しました。")
    finally: # 必ず実行
        node.destroy_node() # ROS2ノード破棄
        rclpy.shutdown() # ROS2終了

if __name__ == '__main__':
    main()