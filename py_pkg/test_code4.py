#!/usr/bin/env python3
"""評価関数の重み COST_Q, COST_R, COST_R_DU を自動チューニングするプログラム（test_code3.py ベース）

test_code3.py は離散時間オイラー・ラグランジュ法のコスト
    J = Σ eᵀQe + Σ R u² + Σ R_du (Δu)²
の重みを手動で調整していた。欲しい最適制御入力（振幅が実機で意味を持つ大きさになる u_opt）を
手作業で見つけるのは難しいため、本プログラムは COST_Q, COST_R, COST_R_DU を Optuna で自動探索する。

【ロボットは1回しか動かさない】
  1. 初期姿勢へ移動 → 励振用の初期FFを印加して SIM_TIME 秒間の実測データを1回だけ取得する
  2. その実測データから目標モデルとシステムモデルを同定し、
     同定したシステムモデルを以降ずっと「真のモデル」とみなす
  3. 以降はロボットを一切動かさず、すべてシミュレーションのみで重みを探索する
     （同定結果はJSONへ保存されるので、モード2で読み込めば再探索にロボットは不要）

【1候補 (COST_Q, COST_R, COST_R_DU) の評価手順】
  1. 真のモデル（システムモデル）と 2. 目標モデル を使い、
     離散時間オイラー・ラグランジュ法（calculate_el_ff。アルゴリズムは一切変更しない）で u_opt を計算
  3. u_opt を真のモデルへ入力して応答を計算する
  4. 目標軌道（目標モデル）との差 J = Σ(y_tgt - y_sys)² を計算する
  5. 対象DOF（TUNE_TARGET_DOF = DOF4）の u_opt が
         10 < max(u_opt) <= 30   または   -30 <= min(u_opt) <= -10
     を満たすことを必須条件とし、満たさない候補には大きなペナルティを与える

【探索変数】 q1, q2, q3, r, du の5変数のみ（すべて対数スケール）
    COST_Q = diag(q1, q2, q3),  COST_R = [[r]],  COST_R_DU = du

得られた重みは test_code3.py の COST_Q, COST_R, COST_R_DU へ貼り替えて使用する。

【test_code3.py からの変更点】
  ・MathematicalSolver（同定・オイラーラグランジュ最適制御・5次多項式フィット）は完全に同一
  ・ROS2通信とデータ収集（Publisher/Subscriber・補間・欠測判定・保持PWM）も完全に同一
  ・追加したのは WeightTuner クラス（探索クラス＋評価関数）と探索実行部分、同定結果のJSON保存/読込
  ・削除したのは内側／外側ループに固有の処理（ビューア用スナップショット、デバッグExcel、
    ベストFFのCSV保存）のみ。ロボットを1回しか動かさない本プログラムでは動作しないため
"""
import os                                                       # OSライブラリ
import sys                                                      # Pythonを扱うライブラリ
import json                                                     # 同定結果（真のモデル）の保存・読み込み
from collections import deque                                   # 固定長リング（保持PWMの直近サンプル保存用）
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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # コールバックの並列実行グループ
from rclpy.executors import MultiThreadedExecutor               # 購読とタイマーを別スレッドで回す実行器
from rclpy.node import Node                                     # Nodeクラスの読み込み
from std_msgs.msg import Float32MultiArray, UInt16MultiArray    # ROS2メッセージ型

import scipy.optimize                                           # 最適化ライブラリ
from scipy.signal import cont2discrete                          # 連続時間→離散時間(ZOH)厳密離散化
from cmaes import CMA                                           # CMA-ES（進化戦略）最適化ライブラリ

import optuna                                                   # ブラックボックス最適化ライブラリ（重みの自動探索）

import tkinter as tk                                            # GUIライブラリ
from tkinter import filedialog                                  # GUIでフォルダ選択

warnings.simplefilter('ignore', RuntimeWarning)                 # RuntimeWarningを非表示
np.seterr(all='ignore')                                         # Numpyのエラーを無地

# ==============================================================================
# 評価関数の重み行列（チューニング要素）
#   コスト関数 J = Σ (eᵀQe + Ru²) + R_du Σ(Δu)² の重み
#   （e は目標軌道との誤差状態, u は制御入力, Δu(k) = u(k) - u(k-1) は入力の変化量）
# ==============================================================================
COST_Q = np.diag([0.900272, 8.38989, 0.122944])     # 状態誤差の重み（大きいほど誤差を抑える）
COST_R = np.array([[34.4762]])             # 制御入力の重み（大きいほど入力を抑える）

# ==============================================================================
# 入力レート罰則の重み（チューニング要素）
#   R_du Σ_{k=0}^{N_ff} ( u(k) - u(k-1) )² を J に加える。境界を u(-1)=0, u(N_ff)=0 と
#   定義することで、この1項だけで次の3つが同時に誘導される。
#     ・u(0)   → 0 に近づく（Δu(0) = u(0) - 0 が罰せられる）
#     ・u(T)   → 0 に近づく（Δu(N_ff) = 0 - u(N_ff-1) が罰せられる）
#     ・途中も滑らか（隣り合うサンプルの段差が罰せられる）
#   これは端点をハード制約で0にするのではなくソフトに引き寄せる方式なので、R_du を上げるほど
#   端点は0へ近づくが追従性能は落ちる。R_du = 0 とすれば従来の定式化に完全に戻る。
#
#   スケールの目安
#     滑らかな入力では |Δu| ~ |u|/N_ff なので Σ(Δu)² ~ Σu²/N_ff²（N_ff = T/dt）。よって
#         R_du = ρ * R * N_ff²
#     と置くと、無次元量 ρ がそのまま「レート項と入力項の比」になる。T や dt を変えたときは
#     ρ を保つように N_ff² に比例させて調整すること。
#     実際には状態コスト Σ eᵀQe が入力項より桁で大きいことが多く、レート項はそれと競合する
#     必要があるため、ρ は 1〜10 程度（入力項と同等〜10倍）が実用域になる。
#     既定値は T=1.5s, dt=0.01s（N_ff=150）, R=10.7 に対する ρ=1。
#
#   チューニング手順
#     R_du を10倍刻みで振り、u_opt(T)（FF区間終端の値）が十分0に近づく最小の値を選ぶ。
#     max|Δu| は中間領域で一時的に増えることがある（終端を下げる過程で急降下が生じるため）
#     ので、判断は max|Δu| ではなく u_opt(0), u_opt(T) の大きさで行うこと。
# ==============================================================================
COST_R_DU = 2.4e6           # 入力の変化量Δuの重み（大きいほど入力が滑らかになり、端点が0へ寄る。0で従来どおり無効）

# ==============================================================================
# 重み (COST_Q, COST_R, COST_R_DU) 自動探索のパラメータ（チューニング要素）
#   探索変数は q1, q2, q3, r, du の5個のみ。
#       COST_Q = diag(q1, q2, q3),  COST_R = [[r]],  COST_R_DU = du
#   いずれも桁が大きく違うため、すべて対数スケールで探索する。
#
#   ● 必須条件（対象DOFの最適入力の振幅）
#       10 < max(u_opt) <= 30   または   -30 <= min(u_opt) <= -10
#     「大きすぎても小さすぎてもいけない」帯状の条件なので、単なる下限ではない。
#     条件を満たさない候補は、帯までの距離に応じた大ペナルティで必ず劣後させる。
#
#   ● スケール不変性について
#     コスト J = Σ eᵀQe + Σ R u² + Σ R_du (Δu)² は (Q,R,R_du) → c·(Q,R,R_du) （c>0）に
#     対して同じ u_opt を与える。つまり本質的に効くのは3者の「比」であり、5変数のうち
#     1自由度は冗長である。仕様どおり5変数を探索するが、結果として複数の重みの組が
#     まったく同じ性能を示すのはこの性質による。
# ==============================================================================
TUNE_TARGET_DOF = 4         # 必須条件を課し、追従誤差Jを評価する対象DOF（1始まり。プリセット目標値で大きく動く自由度）
TUNE_U_BAND_LO = 10.0       # 必須条件の振幅の下限 [PWM]（max(u_opt) はこれ「より大きい」、min(u_opt) は -これ「以下」）
TUNE_U_BAND_HI = 30.0       # 必須条件の振幅の上限 [PWM]（max(u_opt) はこれ以下、min(u_opt) は -これ以上）
TUNE_N_TRIALS = 200         # Optunaの探索回数の既定値（実行時に引数またはキーボードで指定可能）
TUNE_SAMPLER = 'tpe'        # Optunaのサンプラー（'tpe': TPESampler / 'cmaes': CmaEsSampler）
TUNE_SEED = 0               # サンプラーの乱数シード（Noneで毎回変化）
TUNE_Q_RANGE = (1e-3, 1e6)  # 探索範囲 q1, q2, q3（対数スケール）
TUNE_R_RANGE = (1e-6, 1e3)  # 探索範囲 r（対数スケール）
TUNE_R_DU_RANGE = (1e-4, 1e10)   # 探索範囲 du（対数スケール）。目安 du ≈ ρ·r·N_ff²（N_ff=T/dt, ρ=1〜10）を含む広さにする
TUNE_U_ABS_LIMIT = 60     # 追加条件（任意）: |u_opt| 全体の上限 [PWM]。None なら仕様どおり片側の帯だけで判定する
#   必須条件は「max または min」のOR条件なので、min が帯に入っていれば max がいくら大きくても
#   条件を満たしてしまう（例: min=-20 で条件成立、しかし max=249）。片側だけ極端に大きい
#   u_opt を除きたい場合は、ここに 30.0 などを設定すると |u_opt| 全体にも上限が掛かる。
TUNE_PENALTY = 1e12         # 必須条件を満たさない候補へ与える大ペナルティ（実現しうる最大のJより十分大きい値）
TUNE_SEED_MANUAL = True     # Trueなら手動値(COST_Q, COST_R, COST_R_DU)を初回トライアルとして必ず評価し、比較対象にする
TUNE_SAVE_PLOT = True       # 最良結果のグラフ（応答・入力）をPNGへ保存するか

# ==============================================================================
# シミュレーションおよび最適化のパラメータ（チューニング要素）
# ==============================================================================
SIM_TIME = 5.0              # シミュレーション時間および実測データ収集時間（秒）
SIM_DT = 0.01               # シミュレーションのサンプル刻み幅（秒）
COLLECT_EXTRA_WAIT = 3.0    # SIM_TIME分のデータが揃うまで追加で待つ最大時間（秒）。これを超えたら警告して先へ進む
HOLD_PWM_WINDOW = 50        # 保持PWM（同定入力の基準値）の推定に使う直近サンプル数（100Hzなら約0.5秒分）
MAX_SAMPLE_GAP = 0.05       # 受信間隔がこれを超えた区間は「欠測」とみなす（秒）。補間で作った直線を実測として扱わないための閾値
GAP_WARN_RATIO = 0.05       # 欠測がこの割合を超えたDOFは警告する（同定結果が信用できないため）
INIT_WAIT_TIME = 10.0       # 初期姿勢への移動後の待機時間（秒）

# ==============================================================================
# 離散時間オイラー・ラグランジュ（随伴／勾配）法のパラメータ（チューニング要素）
# ==============================================================================
EL_MAX_ITER = 200           # 勾配法の最大反復回数
EL_EPS = 1e-3               # 勾配ノルム Σ||∂H/∂u||² の収束判定閾値 ε
EL_LS_MAX = 40              # ステップ幅 α のバックトラッキング最大試行回数

# ==============================================================================
# システムモデル同定のパラメータ（チューニング要素）
#   モデル  b0 / ((T1*s + 1)(s^2 + 2*zeta*wn*s + wn^2))   同定変数は (T1, zeta, wn, b0) の4個
#   極形式で持つことで、係数(a2,a1,a0)を直接探索する場合に比べて
#     ・T1>0, wn>0 なので実極 -1/T1 が必ず安定側に入り、探索の条件数も改善する
#     ・zeta を自由変数（負も許容）にすることで、減衰振動も発散振動も表現できる
#     ・wn に直接上限を課せるので、ナイキスト周波数を超える無意味な極へ逃げない
# ==============================================================================
SYS_T1_MIN = 2e-3           # 1次遅れ時定数 T1 の下限 [s]
SYS_T1_MAX = 20.0           # 1次遅れ時定数 T1 の上限 [s]
SYS_ZETA_MIN = -0.4         # 減衰比 zeta の下限（負を許容し、振幅が増大する振動を表現できるようにする）
SYS_ZETA_MAX = 5.0          # 減衰比 zeta の上限（強い過減衰の軸も表現できるよう広めに取る）
SYS_WN_MIN = 0.5            # 固有振動数 wn の下限 [rad/s]
SYS_WN_NYQ_RATIO = 5.0      # 固有振動数 wn の上限をナイキスト角周波数(π/dt)の 1/この値 にする
SYS_SPEC_FMIN = 0.1         # スペクトルピーク探索の下限周波数 [Hz]（DC近傍のトレンド成分を除外する）
SYS_WN_SEED_RATIOS = (0.8, 0.9, 1.0, 1.1, 1.25)     # スペクトル推定値に掛ける多点開始の倍率
SYS_RESID_CLIP = 1.0e5      # 残差の打ち切り値（発散したモデルでinf/nanが出て最適化が止まるのを防ぐ）

# ==============================================================================
# 初回システム同定用の初期FF入力の振幅（チューニング要素）
#   ゼロ入力ではシステムモデルの b0 が同定不能（上限に張り付く）ため、十分な励振を
#   与える非ゼロFFを初期値とする。2つの極値の大きさ |f(t1)|=|f(t2)| がこの値になる。
# ==============================================================================
INIT_FF_PEAK = 0.0         # 初期FFの極値の大きさ [PWM]（f(t1)=+50≥40, f(t2)=-50≤-40 を満たす）

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
FIT_ZERO_AMP = 1e-6         # max|u_opt| がこの値未満なら「動かす必要がないDOF」とみなし、探索せず f≡0 を返す閾値 [PWM]


# ==============================================================================
# 数学ソルバー (System ID & 離散時間オイラー・ラグランジュ最適制御)
# ==============================================================================
class MathematicalSolver:
    # コンストラクタ
    def __init__(self, T, dt=SIM_DT, Q=None, R=None, R_du=None):    # 引数(FF制御入力時間, シミュレーションステップ時間, 状態重み行列, 入力重み行列, 入力レート重み)
        self.T = T                                          # FF制御入力時間を保存
        self.dt = dt                                        # シミュレーションステップ時間を保存
        self.t_eval = np.arange(0, SIM_TIME, self.dt)       # シミュレーション時間配列を作成
        self.Q = Q if Q is not None else COST_Q.copy()      # 状態重み行列の保存（引数 Q が与えられていればそれを使用し、与えられていなければ COST_Q をコピー）
        self.R = R if R is not None else COST_R.copy()      # 入力重み行列の保存（引数 R が与えられていればそれを使用し、与えられていなければ COST_R をコピー）
        self.R_du = float(R_du) if R_du is not None else float(COST_R_DU)   # 入力レート重みの保存（引数 R_du が与えられていればそれを使用し、与えられていなければ COST_R_DU）
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

    # 極形式のパラメータを3次遅れ系の係数へ展開する関数（最適化パイプラインからも呼ぶので公開関数）
    @staticmethod
    def system_coeffs(T1, zeta, wn):            # 引数(1次遅れ系の時定数, 減衰比, 固有振動数)
        """(T1*s + 1)(s^2 + 2*zeta*wn*s + wn^2) の分母を s^3 + a2*s^2 + a1*s + a0 の形へ展開する。

            (T1 s + 1)(s^2 + 2 zeta wn s + wn^2)
              = T1 s^3 + (2 zeta wn T1 + 1) s^2 + (wn^2 T1 + 2 zeta wn) s + wn^2
            → 可制御正準形で使うため、T1 で割って最高次の係数を1に正規化する
        """
        a2 = (2 * zeta * wn * T1 + 1) / T1
        a1 = (wn ** 2 * T1 + 2 * zeta * wn) / T1
        a0 = (wn ** 2) / T1
        return a2, a1, a0

    # 目標モデルの3次遅れ系係数を計算する関数（ "_"が付いているので外から直接呼べない内部専用関数）
    def _target_coeffs(self, T1, wn):           # 引数(1次遅れ系の時定数, 固有振動数)
        """減衰係数1固定の目標モデル係数（system_coeffs の zeta=1 の特殊形）"""
        return self.system_coeffs(T1, 1.0, wn)

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

    # 実測データの支配振動数を推定する関数（ "_"が付いているので外から直接呼べない内部専用関数）
    def _spectral_wn(self, y_data):                                                     # 引数(実測データ)
        """実測波形のスペクトルピークから支配振動数 [rad/s] を推定する。

        出力誤差法（シミュレーション全体の残差二乗和）のコストは振動数について
        櫛状の局所解を持つ。5秒間に数周期あるため、モデルの振動数が数%ずれるだけで
        位相が1周回り、残差は「振動を全く再現しない場合」より大きくなる。そのため
        過減衰な初期値から出発した勾配には正解の振動数を指す情報が無い。
        ここで実測波形から wn の当たりを付け、正解の谷の中から探索を始められるようにする。
        """
        n = np.arange(len(y_data))                                                      # サンプル番号
        trend = np.polyval(np.polyfit(n, y_data, 1), n)                                 # 1次トレンド（立ち上がり成分）
        osc = y_data - trend                                                            # トレンドを除いた振動成分
        spec = np.abs(np.fft.rfft(osc * np.hanning(len(osc))))                          # 窓を掛けて振幅スペクトルを計算
        freq = np.fft.rfftfreq(len(osc), self.dt)                                       # 周波数軸 [Hz]
        sel = freq > SYS_SPEC_FMIN                                                      # DC近傍を除外（残ったトレンドを拾わないため）
        if not np.any(sel) or not np.isfinite(spec[sel]).any() or spec[sel].max() <= 0:  # 振動成分が無い（静止データ）場合
            return None                                                                 # Noneを返して呼び出し側で既定値を使わせる
        return 2 * np.pi * float(freq[sel][np.argmax(spec[sel])])                       # ピーク周波数を角周波数 [rad/s] にして返す

    # システムモデル同定関数
    def fit_system_model(self, y_data, u_ff, y0, prev_params=None):                                 # 引数(実測データ, 同定入力（実際に印加された制御入力PWM）, 初期偏差, 前回の同定結果[T1, zeta, wn, b0])
        """システムモデル同定: b0 / ((T1*s + 1)(s^2 + 2*zeta*wn*s + wn^2))

        同定変数は (T1, zeta, wn, b0) の4個。係数(a2,a1,a0,b0)を直接探索していた従来方式と
        パラメータ数は同じだが、極形式で持つことで次の利点がある。
          ・T1>0, wn>0 なので実極 -1/T1 が必ず安定側に入る
            （従来の a2,a1,a0>0 は3次系の安定性を保証せず、不安定なモデルが採用されていた）
          ・zeta を自由変数にすることで複素極を作れる。zeta<0 まで許すので、実機で観測される
            振幅が増大する振動も表現できる（zeta=1 に固定すると三重実極しか作れず再現不能）
          ・wn に直接上限を課せるので、ナイキスト周波数を大きく超える無意味な極へ逃げない
        T1・wn は対数で探索し、桁の違うパラメータが混在することによる信頼領域の破綻を防ぐ。
        また zeta を自由にすると振動数の局所解問題が生じるため、前回結果のウォームスタートに
        加えてスペクトル推定値まわりの多点開始を行い、最も残差の小さい解を採用する。
        b0は励振（非ゼロFF入力）があってはじめて同定できるため、初回同定では
        INIT_FF_PEAK 振幅の初期FFで励振する。
        """
        wn_max = np.pi / self.dt / SYS_WN_NYQ_RATIO                                     # 固有振動数の上限（ナイキスト角周波数の1/SYS_WN_NYQ_RATIO）

        # 各時刻の残差ベクトルを返す関数（T1, wn は対数で受け取る）
        def residuals(q):
            log_T1, zeta, log_wn, b0 = q                                                            # 最適化変数の取り出し
            a2, a1, a0 = self.system_coeffs(np.exp(log_T1), zeta, np.exp(log_wn))                   # 3次遅れ系の係数へ展開
            with np.errstate(over='ignore', invalid='ignore'):                                      # 発散したモデルのオーバーフロー警告を抑制
                y_sim, _, _ = self.simulate_forced(self.t_eval, a2, a1, a0, b0, u_ff, y0)           # システムモデルの応答を計算
                r = y_sim - y_data                                                                  # 残差ベクトル
            r = np.nan_to_num(r, nan=SYS_RESID_CLIP, posinf=SYS_RESID_CLIP, neginf=-SYS_RESID_CLIP)  # inf/nanを有限値に置換
            return np.clip(r, -SYS_RESID_CLIP, SYS_RESID_CLIP)                                      # 打ち切って最適化が止まらないようにする

        lower = [np.log(SYS_T1_MIN), SYS_ZETA_MIN, np.log(SYS_WN_MIN), -np.inf]                     # 探索変数の下限
        upper = [np.log(SYS_T1_MAX), SYS_ZETA_MAX, np.log(wn_max),      np.inf]                     # 探索変数の上限

        # 探索の開始点を集める（ウォームスタート + スペクトル推定値まわりの多点開始）
        starts = []                                                                                 # 開始点の空リスト
        if prev_params is not None and len(prev_params) == 4 and np.all(np.isfinite(prev_params)):  # 前回の同定結果が使える場合
            T1_p, zeta_p, wn_p, b0_p = [float(v) for v in prev_params]                              # 前回の同定結果を取り出す
            if T1_p > 0 and wn_p > 0:                                                               # 対数を取れる値であることを確認
                starts.append([np.log(T1_p), zeta_p, np.log(wn_p), b0_p])                           # ウォームスタートの開始点を追加
        b0_init = starts[0][3] if starts else 100.0                                                 # b0の初期値（前回値があればそれを流用）
        wn_seed = self._spectral_wn(y_data)                                                         # 実測スペクトルからの振動数推定
        if wn_seed is not None:                                                                     # 振動成分が検出できた場合
            for ratio in SYS_WN_SEED_RATIOS:                                                        # 櫛状の局所解に落ちないよう振動数を振って多点開始
                starts.append([np.log(0.5), -0.05, np.log(wn_seed * ratio), b0_init])               # わずかに発散側(zeta=-0.05)から開始する
        if not starts:                                                                              # 前回結果もスペクトル推定も無い場合
            starts.append([np.log(0.5), 0.7, np.log(5.0), b0_init])                                 # 既定の開始点を使う

        # 開始点ごとに最適化し、最も残差二乗和の小さい解を採用する
        best = None                                                                                 # 最良の最適化結果
        for x0 in starts:                                                                           # 開始点のループ
            try:
                res = scipy.optimize.least_squares(                                                 # Σr² が最小になる変数[log T1, zeta, log wn, b0]を最適化する
                    residuals, x0=np.clip(x0, lower, upper),                                        # 開始点を境界内に収めてから渡す
                    bounds=(lower, upper),
                    method='trf', x_scale='jac',                                                    # x_scale='jac' で変数間のスケール差を吸収する
                )
            except Exception:                                                                       # 数値的に破綻した開始点は捨てる
                continue
            if best is None or res.cost < best.cost:                                                # より残差の小さい解が見つかった場合
                best = res                                                                          # 最良解を更新

        if best is None:                                                                            # すべての開始点で失敗した場合
            return np.array([0.5, 0.7, 5.0, b0_init])                                               # 既定値を返して同定を継続させる

        log_T1, zeta, log_wn, b0 = best.x                                                           # 最適変数を取り出す
        return np.array([np.exp(log_T1), zeta, np.exp(log_wn), b0])                                 # (T1, zeta, wn, b0) を返す

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

        最適入力が実質ゼロのDOF（目標値がそのDOFを動かさない＝偏差y0が0で勾配が立たない場合）は
        f≡0 が正解なので、探索せずゼロ係数を返す。
        """
        # 動かす必要がないDOFの早期リターン
        #   max|u_opt|=0 のまま探索へ進むと sigma = CMA_SIGMA_RATIO*amp*(1+restart) = 0 となり、
        #   cmaes 側の assert sigma > 0 で例外になる。そもそも極値条件を満たす非ゼロな5次関数を
        #   探しても意味がない（ゼロ入力へ近づけるほど誤差が減る）ため、ここで f≡0 を返す。
        amp = float(np.max(np.abs(u_opt_ff))) if len(u_opt_ff) > 0 else 0.0             # 最適入力の振幅（探索スケールの基準）
        if not np.isfinite(amp) or amp < FIT_ZERO_AMP:                                  # 最適入力が実質ゼロなら探索しない
            self.last_fit_restart = -3                                                  # 探索不要だったことを示す値（ビューア表示用）
            return (0.0, 0.0, 0.0, 0.0), (self.T * 0.33, self.T * 0.66)                 # f≡0（極値時刻は calc_extrema_from_ff の既定値に合わせた表示用の値）

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

        # 探索の初期値: 極値条件を必ず満たす形 f(t)=C t(t-T/2)(t-T) を u_opt へ最小二乗で当てたもの
        #   この形の極値は t=T(3±√3)/6 の2点のみで常に単純根なので、実行可能な点から探索を始められる
        g = t_ff ** 3 - 1.5 * self.T * t_ff ** 2 + 0.5 * self.T ** 2 * t_ff             # 形状 t(t-T/2)(t-T)
        denom = float(g @ g)                                                            # 最小二乗の分母
        C = float(g @ u_opt_ff) / denom if denom > 1e-300 else 0.0                      # 二乗誤差が最小になる振幅C
        if not np.isfinite(C) or abs(C) < 1e-12:                                        # u_optとこの形状がほぼ直交するときは極値がu_optと同振幅になるCにする
            C = 12.0 * np.sqrt(3.0) * amp / (self.T ** 3)                               # |f(t1)|=|f(t2)|=amp となる振幅（amp>0 は早期リターンで保証済み）
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
    def calculate_el_ff(self, target_params, sys_params, u_ff, y0, u_adrc=None):                             # 引数(目標モデルのパラメータ[T1, wn], システムモデルのパラメータ[T1, zeta, wn, b0], FF制御入力, 初期偏差, 実測から復元したADRC入力)
        """
        離散時間オイラー・ラグランジュ（随伴／勾配）法で最適制御入力を計算し、
        FF分 u_FF_opt を5次多項式 FF = a*t^5 + ... + e*t にフィットする。

        目的：システムモデルの軌道を、目標モデルが描く目標軌道 x_tgt へ一致させる。
          座標系       : z = P - Pi（初期位置を原点とする系）。システムモデル同定と同じ規約で、
                         入力は u = u_total - u_hold。x_sys(0)=x_tgt(0)=0 から step = Pf - Pi へ立ち上がる
          入力の分解   : u(k) = u_ADRC(k) + u_FF_opt(k)
                         u_ADRC は実測から復元した固定の時系列（最適化中は一切変化させない）で、
                         u_FF_opt だけが最適化変数。目標位置を保持するための定常入力は u_ADRC が
                         担うので、u_FF_opt は移動の過渡補正だけを受け持つ。
          入力ホライズン : u_FF_opt(k) は [0,T]（k=0..N_ff-1）のみ最適化し、それ以降は0。
                         k ≥ N_ff では u(k) = u_ADRC(k) となり、ADRCが姿勢を保持し続ける
          端点条件     : u_FF_opt(0) = 0, u_FF_opt(N_ff) = 0 をハード制約として固定する。
                         したがって u_opt(0) = u_ADRC(0), u_opt(T) = u_ADRC(T) になる
                         （実測入力は t=0 で 0 なので u_ADRC(0)=0、つまり u_opt(0)=0）
          評価ホライズン : コスト J はシミュレーション全体（5秒, k=0..M-1）で評価する

        ● 実測データからの u_ADRC の復元（呼び出し側で行い、この関数へ渡す）
            同定に使った実測総入力 u_actual = u_pwm - u_hold は、ADRCの出力と実際に送った
            FF入力の和になっている。
                u_actual(t) = u_ADRC(t) + u_FF_actual(t)
            u_FF_actual は自分が送った5次関数FFなので既知。よって
                u_ADRC(t) = u_actual(t) - u_FF_actual(t)
            としてADRC入力の時系列を復元する。t > T では u_FF_actual = 0 なので u_ADRC = u_actual。

        ● 離散状態方程式（可制御正準形・ZOH厳密離散化）
            システム : x_sys(k+1) = A_sys x_sys(k) + B_sys u(k),  x_sys(0) = 0
                       u(k) = u_ADRC(k) + u_FF_opt(k)
            目標     : x_tgt(k) は A_tgt の自由応答（誤差系 y0→0）を z 形式へ定数オフセットして生成する
                       （詳細は下の「目標軌道 x_tgt(k) を z 形式で生成する」ブロックを参照）

        ● 誤差状態 x(k)=x_sys(k)-x_tgt(k) を状態変数とした状態方程式（e と表記, e(0)=0）
            e(k+1) = x_sys(k+1) - x_tgt(k+1)
                   = A_sys( e(k)+x_tgt(k) ) + B_sys u(k) - x_tgt(k+1)
                   = A_sys e(k) + B_sys u(k) + d(k),   d(k) = A_sys x_tgt(k) - x_tgt(k+1)  （既知の入力項）
            目標軌道が A_tgt の自由応答であれば d(k) = (A_sys - A_tgt) x_tgt(k) に一致する（従来式）。
            誤差状態の定義も式の形もこれまでと同じで、変わったのは u(k) が u_ADRC + u_FF_opt に
            分解された点だけである。

        ● コスト関数（終端コストなし）／ハミルトニアン
            J = Σ_{k=0}^{M-1} ( e(k)ᵀ Q e(k) )
              + Σ_{k=0}^{N_ff-1} R u_FF_opt(k)²
              + Σ_{k=0}^{N_ff}  R_du ( u_FF_opt(k) - u_FF_opt(k-1) )²,  u_FF_opt(-1)=u_FF_opt(N_ff)=0
            入力罰則・レート罰則は最適化変数 u_FF_opt に掛ける。固定の u_ADRC を罰しても最適化には
            効かないうえ、総入力に掛けるとレート罰則の境界0が「終端でADRCの保持入力まで0へ引き戻す」
            意味になり、端点条件 u_opt(T)=u_ADRC(T) と矛盾するためである。u_FF_opt に掛ければ
            境界0はそのまま端点条件 u_FF_opt(0)=u_FF_opt(N_ff)=0 と一致する。

            段コスト L(k) = e(k)ᵀ Q e(k) + R u_FF_opt(k)² + R_du ( u_FF_opt(k)-u_FF_opt(k-1) )²
            H(k) = L(k) + λ(k+1)ᵀ [ A_sys e(k) + B_sys u(k) + d(k) ]

        ● 随伴方程式（u_ADRC は固定、レート罰則は u_FF_opt のみの関数で ∂/∂e = 0 なので不変）
            λ(N) = λ(M) = 0
            λ(k) = ∂H/∂e(k) = 2 Q e(k) + A_sysᵀ λ(k+1)

        ● 勾配（最適化変数 u_FF_opt, k=0..N_ff-1）
            u(k) = u_ADRC(k) + u_FF_opt(k) で u_ADRC は定数なので ∂u(k)/∂u_FF_opt(k) = 1。
            よって状態方程式からの寄与は従来どおり B_sysᵀ λ(k+1) のままで、勾配の式は変わらない。
                ∂J/∂u_FF_opt(k) = ∂H(k)/∂u_FF_opt(k) + ∂L(k+1)/∂u_FF_opt(k)
                                 = 2 R u_FF_opt(k) + B_sysᵀ λ(k+1)
                                   + 2 R_du ( 2u_FF_opt(k) - u_FF_opt(k-1) - u_FF_opt(k+1) )
            末尾の括弧は Dirichlet境界条件 u_FF_opt(-1)=u_FF_opt(N_ff)=0 を課した2階差分
            （3重対角行列 L=tridiag(-1,2,-1)）で、(L u_FF_opt)(k) と書ける。
            端点 k=0 は u_FF_opt(0)=0 のハード制約なので、勾配の第0成分を0にして更新しない（射影）。
            k=N_ff は決定変数の外なので u_FF_opt(N_ff)=0 は構造的に満たされる。

        解法（未知変数は [0,T] の u_FF_opt(k)、k≥N_ff では u_FF_opt(k)=0）：
          1. u_FF_opt(k) を初期化する（零入力から開始。u_FF_opt(0)=0 は以後ずっと保たれる）
          2. 誤差状態 e を状態変数とした状態方程式から e(k) を順方向計算（5秒全体, u=u_ADRC+u_FF_opt）
          3. λ(N) = 0
          4. 随伴方程式を逆方向計算（5秒全体）
          5. ∂H/∂u_FF_opt を計算（k=0..N_ff-1）し、端点条件のため第0成分を0にする
          6. Σ||∂H/∂u_FF_opt||² < ε なら終了
          7. そうでなければ u_FF_opt ← clip(u_FF_opt - α ∂H/∂u_FF_opt, -255, 255) として 2 へ戻る

        最後に総最適入力 u_opt = u_ADRC + u_FF_opt を作り、その差 u_FF_opt = u_opt - u_ADRC を
        5次多項式で近似する。近似対象は総入力 u_opt ではなくFF分 u_FF_opt であり、
        u_FF_opt(0)=u_FF_opt(T)=0 は5次関数の端点条件 f(0)=f(T)=0 とそのまま整合する。
        """
        T1, wn = target_params                                                                          # 目標モデルのパラメータ取得
        a2_tgt, a1_tgt, a0_tgt = self._target_coeffs(T1, wn)                                            # 目標モデルの3次遅れ系の係数を計算
        T1_sys, zeta_sys, wn_sys, b0_sys = sys_params                                                   # システムモデルのパラメータ取得
        a2_sys, a1_sys, a0_sys = self.system_coeffs(T1_sys, zeta_sys, wn_sys)                           # システムモデルの3次遅れ系の係数を計算

        dt = self.dt                                                                                    # シミュレーションのサンプル刻み幅
        M = len(self.t_eval)                                                                            # 評価ホライズン（5秒全体）のステップ数
        N_ff = int(round(self.T / dt))                                                                  # 入力ホライズン [0,T] のステップ数
        n_x = 3                                                                                         # 状態次元 (可制御正準形: x0=位置, x1=速度, x2=加速度)

        # -----------------------------------------------------------
        # 同定結果から離散時間状態方程式を構築（cont2discrete によるZOH厳密離散化）
        #   システム : x_sys(k+1) = A_sys x_sys(k) + B_sys u(k)
        #   目標     : A_tgt は誤差系（y→0）の自由応答行列。同定はこのまま変更しない
        # -----------------------------------------------------------
        A_sys, B_sys = self._discretize(a2_sys, a1_sys, a0_sys, b0_sys)                                 # システムモデルのZOH離散状態方程式
        A_tgt, _ = self._discretize(a2_tgt, a1_tgt, a0_tgt, 0.0)                                        # 目標モデルのZOH離散状態方程式（u=0）

        # 目標軌道 x_tgt(k) を z 形式（初期位置を原点とする系）で生成する
        #   A_tgt の同定は従来どおり誤差系（y = P - Pf, y0 → 0 へ収束する自由応答）で行うが、
        #   最適制御ではシステムモデル同定と同じ z = P - Pi の系（z(0)=0 → step へ立ち上がる）で扱う。
        #   両者は定数オフセットの関係にあり、z_tgt(k) = y_tgt(k) - y0 が厳密に成り立つ。
        #   （3次系は相対次数3なので、step応答 z は自由応答 y の1-補数 z = step(1 - y/y0), step = -y0。
        #     位置成分だけがオフセットされ、速度・加速度成分は自由応答のまま変わらない。）
        #   この座標系では x_sys(0)=x_tgt(0)=0 となり、入力規約も同定と同じ u = u_total - u_hold で
        #   そろう。姿勢保持のための定常入力は u_ADRC が担うので、最適化する u_FF_opt には混入しない。
        #   d(M-1) の計算に x_tgt(M) が要るので、M+1 点分を生成する。
        step = -y0                                                                                      # 目標変位 step = Pf - Pi（z形式での最終値）
        x_tgt = np.zeros((M + 1, n_x))                                                                  # 目標状態列 x_tgt(0..M) (M+1, 3)
        x_tgt[0] = np.array([y0, 0.0, 0.0])                                                             # 誤差系での初期状態（自由応答の初期値）
        for k in range(M):                                                                              # 目標モデルの自由応答を逐次計算
            x_tgt[k + 1] = A_tgt @ x_tgt[k]                                                             # x_tgt(k+1) = A_tgt x_tgt(k)
        y_tgt = x_tgt[:M, 0].copy()                                                                     # 目標出力（誤差系 y0→0）。呼び出し側の規約は従来どおり維持する
        x_tgt[:, 0] += step                                                                             # 位置成分だけを z 形式へオフセット（x_tgt(0)の位置=0, 最終値=step）

        # 誤差状態方程式の既知入力項 d(k) = A_sys x_tgt(k) - x_tgt(k+1)
        #   e(k+1) = x_sys(k+1) - x_tgt(k+1) を厳密に満たす一般形。目標軌道が自由応答
        #   （x_tgt(k+1) = A_tgt x_tgt(k)）のときは従来式 (A_sys - A_tgt) x_tgt(k) に一致する。
        #   z 形式の目標軌道は定数オフセットを含むぶん A_tgt の自由応答ではない
        #   （x_tgt(k+1) = A_tgt x_tgt(k) + (I - A_tgt)[step,0,0]ᵀ）ため、この一般形で計算する。
        D = x_tgt[:M] @ A_sys.T - x_tgt[1:M + 1]                                                        # D[k] = A_sys x_tgt(k) - x_tgt(k+1) → 形状 (M, 3)

        Q = self.Q                                                                                      # 状態誤差の重み行列 (3x3)
        R = float(self.R[0, 0])                                                                         # 入力の重みスカラー
        R_du = float(self.R_du)                                                                         # 入力レート（Δu）の重みスカラー

        # 実測から復元したADRC入力（最適化中は固定。未指定なら0＝従来どおりFFのみの定式化になる）
        u_adrc = np.zeros(M) if u_adrc is None else np.asarray(u_adrc, dtype=float)[:M]                 # ADRC入力の時系列 u_ADRC(0..M-1)
        if u_adrc.size < M:                                                                             # 長さが足りない場合（受信不足など）
            u_adrc = np.pad(u_adrc, (0, M - u_adrc.size), mode='edge')                                  # 最後の値で埋めて長さをそろえる

        # ---- 手順2: 誤差状態 e を順方向に計算する関数（e(0)=x_sys(0)-x_tgt(0)=0） ----
        #   e(k+1) = A_sys e(k) + B_sys u(k) + d(k)。
        #   入力は総入力 u(k) = u_ADRC(k) + u_FF_opt(k)。u_FF_opt は [0,T](k<N_ff) のみで、
        #   それ以降は0（＝ k ≥ N_ff では u(k) = u_ADRC(k) となりADRCが姿勢を保持する）。
        def forward(u_seq):                                                                             # 引数(最適化変数 u_FF_opt の列)
            e = np.zeros((M + 1, n_x))                                                                  # 誤差状態列 e(0..M)（e(0)=0）
            for k in range(M):                                                                          # k=0..M-1 を順方向に更新
                uk = u_adrc[k] + (u_seq[k] if k < N_ff else 0.0)                                        # 総入力 u(k)=u_ADRC(k)+u_FF_opt(k)
                e[k + 1] = A_sys @ e[k] + B_sys * uk + D[k]                                             # e(k+1)=A_sys e(k)+B_sys u(k)+d(k)
            return e

        # ---- 入力レート罰則 Σ_{k=0}^{N_ff} ( u(k)-u(k-1) )² を計算する関数（u(-1)=u(N_ff)=0） ----
        #   先頭と末尾に0を1個ずつ挟んでから階差を取ることで、境界条件をそのまま式に埋め込む。
        #   Δu(0)=u(0)-0 と Δu(N_ff)=0-u(N_ff-1) が入るので、両端が0から離れるほど罰せられる。
        def rate_sq(u_seq):
            du = np.diff(np.concatenate(([0.0], u_seq, [0.0])))                                         # Δu(k)=u(k)-u(k-1), k=0..N_ff （長さ N_ff+1）
            return float(du @ du)                                                                       # Σ Δu(k)²

        # ---- 入力レート罰則の勾配 (L u)(k) = 2u(k)-u(k-1)-u(k+1) を計算する関数（u(-1)=u(N_ff)=0） ----
        #   Dirichlet境界条件を課した2階差分行列 L=tridiag(-1,2,-1) との積。境界の項を足さない
        #   ことが u(-1)=0, u(N_ff)=0 を課すことに等しい。
        def rate_grad(u_seq):
            lu = 2.0 * u_seq                                                                            # 2u(k)
            lu[1:] -= u_seq[:-1]                                                                        # -u(k-1)（k=0 は u(-1)=0 なので引かない）
            lu[:-1] -= u_seq[1:]                                                                        # -u(k+1)（k=N_ff-1 は u(N_ff)=0 なので引かない）
            return lu

        # ---- コスト関数 J = Σ_{k=0}^{M-1} e(k)ᵀQe(k) + R Σ u(k)² + R_du Σ ( u(k)-u(k-1) )² ----
        def cost(e_traj, u_seq):
            state_cost = np.einsum('ki,ij,kj->', e_traj[:M], Q, e_traj[:M])                             # Σ e(k)ᵀ Q e(k)（5秒全体）
            input_cost = R * np.sum(u_seq ** 2)                                                         # R Σ u(k)²（[0,T]）
            rate_cost = R_du * rate_sq(u_seq)                                                           # R_du Σ Δu(k)²（両端は0とみなす）
            return float(state_cost + input_cost + rate_cost)

        # ---- 手順3,4: 随伴方程式を逆方向計算し λ(1..M) を求める関数 ----
        #   λ(M)=0（手順3, 終端コストなし）, λ(k)=2 Q e(k)+A_sysᵀ λ(k+1)（手順4）
        #   入力レート罰則は u だけの関数（∂/∂e = 0）なので、随伴方程式は従来のまま変わらない。
        def backward(e_traj):
            lam = np.zeros((M + 1, n_x))                                                                # 随伴変数列 λ(0..M)（λ(M)=0）
            for k in range(M - 1, 0, -1):                                                               # k=M-1..1 を逆方向に更新
                lam[k] = 2.0 * (Q @ e_traj[k]) + A_sys.T @ lam[k + 1]                                   # λ(k)=2 Q e(k)+A_sysᵀ λ(k+1)
            return lam

        # ---- 手順5: 勾配（入力ホライズン k=0..N_ff-1） ----
        #   ∂J/∂u(k) = 2 R u(k) + B_sysᵀ λ(k+1) + 2 R_du ( 2u(k)-u(k-1)-u(k+1) )
        #   第3項がレート罰則による寄与。u(k) が段 k と段 k+1 の両方の段コストに現れるため、
        #   H(k) からの +2R_du(u(k)-u(k-1)) と L(k+1) からの -2R_du(u(k+1)-u(k)) の和になる。
        def gradient(u_seq, lam):
            g = (2.0 * R * u_seq                                                                        # 入力そのものの罰則 2 R u_FF_opt(k)
                 + lam[1:N_ff + 1] @ B_sys                                                              # 随伴からの寄与 B_sysᵀ λ(k+1)（u_ADRCは定数なので係数は従来どおり1）
                 + 2.0 * R_du * rate_grad(u_seq))                                                       # レート罰則 2 R_du (L u_FF_opt)(k)
            g[0] = 0.0                                                                                  # 端点条件 u_FF_opt(0)=0 は固定なので更新しない（実行可能集合への射影）
            return g

        # ---- 手順1: 最適化変数 u_FF_opt(k) を初期化（零入力から開始） ----
        #   決定変数はFF分 u_FF_opt(0..N_ff-1) のみ。0で初期化するので端点条件 u_FF_opt(0)=0 が
        #   最初から満たされ、勾配の第0成分を0にしているので以後もずっと0のまま保たれる。
        #   u_FF_opt(N_ff)=0（t=T）は決定変数の外なので構造的に満たされる。
        u = np.zeros(N_ff)                                                                              # 決定変数 u_FF_opt(0..N_ff-1)（[0,T] のみ）

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
            #   （レート罰則 R_du Σ(Δu)² も u の2次形式なので、この曲率推定はそのまま成り立つ）
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

        # 総最適入力 u_opt = u_ADRC + u_FF_opt をシミュレーション時間全体の配列へ格納
        #   u_FF_opt は [0,T] のみ非ゼロなので、それ以降は u_opt = u_ADRC（ADRCが姿勢を保持する）。
        u_opt = u_adrc.copy()                                                                           # 総最適入力を保存する配列（まずADRC分）
        u_opt[:N_ff] += u                                                                               # FF入力区間 (0..N_ff-1) に最適化したFF分を加える

        # FF分を総入力との差として取り出す（u_FF_opt(t) = u_opt(t) - u_ADRC(t)）
        #   端点は u_FF_opt(0)=0（ハード制約）, u_FF_opt(N_ff)=0（決定変数の外）で、
        #   5次関数の端点条件 f(0)=f(T)=0 とそのまま整合する。
        u_ff_opt = u_opt - u_adrc                                                                       # ELで最適化したFF入力 u_FF_opt

        # 5次多項式フィット（0≤t≤T, 端点0, 極値2個）。近似対象は総入力ではなくFF分 u_FF_opt。
        t_ff = self.t_eval[self.t_eval <= self.T]                                                       # FF入力を与える時間だけ取り出す
        u_opt_ff = u_ff_opt[: len(t_ff)]                                                                # FF入力を与える区間だけの u_FF_opt を取り出す

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

        return [a, b, c, d, e], (t1, y1, t2, y2), u_pred_full, y_tgt, u_opt, u_ff_opt                   # 5次関数のパラメータ、極値、5次関数FF入力、目標モデル、総最適入力、ELのFF入力を返す
    
    # 最終的な制御性能を評価する関数
    def compute_J(self, y_tgt, y_sys):                                                                  # 引数(目標モデルの応答, システムモデルの応答)
        """評価関数 J（目標軌道とシステムモデル出力の二乗誤差）"""
        return float(np.sum((y_tgt - y_sys) ** 2))                                                      # 評価値（二乗和誤差）を返す


# ==============================================================================
# 評価関数の重み COST_Q, COST_R, COST_R_DU の自動探索クラス（Optuna）
#   ロボットを1回だけ動かして同定した「真のモデル」と「目標モデル」だけを使い、
#   以降はロボットを一切動かさずシミュレーションのみで
#       COST_Q = diag(q1,q2,q3),  COST_R = [[r]],  COST_R_DU = du
#   の5変数を探索する。MathematicalSolver は一切変更せず、重みだけを差し替えて呼び出す。
# ==============================================================================
class WeightTuner:
    # コンストラクタ
    def __init__(self, T, dt, model, n_trials=TUNE_N_TRIALS, sampler_name=TUNE_SAMPLER, logger=None):    # 引数(FF制御入力時間, ステップ時間, 対象DOFの同定済みモデル, 探索回数, サンプラー名, ROS2ロガー)
        self.T = T                                                          # FF制御入力時間を保存
        self.dt = dt                                                        # シミュレーションステップ時間を保存
        self.model = model                                                  # 対象DOFの同定済みモデル（tgt_params, sys_params, u_ff, y0, Pi, Pf …）
        self.n_trials = n_trials                                            # 探索回数を保存
        self.sampler_name = sampler_name                                    # サンプラー名を保存
        self.logger = logger                                                # ROS2ロガー（無ければprintで代用）
        self.t_eval = np.arange(0, SIM_TIME, dt)                            # シミュレーション時間配列（プロット用）
        self.study = None                                                   # Optunaのstudy
        self.best = None                                                    # 最良トライアルの評価結果（波形の配列を含む）
        self.best_score = float('inf')                                      # 最良トライアルの目的関数値
        self.manual = None                                                  # 手動値(COST_Q, COST_R, COST_R_DU)の評価結果（比較用）

    # ログ出力関数（ROS2ロガーがあればinfo、無ければprintで代用）
    def _log(self, msg):
        if self.logger is not None:
            self.logger.info(msg)
        else:
            print(msg)

    # ------------------------------------------------------------------
    # 必須条件（対象DOFの最適入力の振幅）の判定
    # ------------------------------------------------------------------
    @staticmethod
    def amplitude_satisfied(u_max, u_min):                                                              # 引数(u_optの最大値, u_optの最小値)
        """必須条件を満たすかどうかを厳密に判定する。

            TUNE_U_BAND_LO < max(u_opt) <= TUNE_U_BAND_HI
            または
            -TUNE_U_BAND_HI <= min(u_opt) <= -TUNE_U_BAND_LO

        正側は下限の等号なし・上限の等号あり、負側は両端とも等号ありで、仕様どおりに判定する。
        探索の誘導には連続量が必要なので、帯までの距離は amplitude_violation() を別に用意している。
        """
        pos_ok = (TUNE_U_BAND_LO < u_max <= TUNE_U_BAND_HI)                                             # 正側が帯の中にあるか
        neg_ok = (-TUNE_U_BAND_HI <= u_min <= -TUNE_U_BAND_LO)                                          # 負側が帯の中にあるか
        if TUNE_U_ABS_LIMIT is not None and max(abs(u_max), abs(u_min)) > TUNE_U_ABS_LIMIT:             # 追加条件（設定時のみ）: |u_opt| 全体の上限
            return False
        return bool(pos_ok or neg_ok)

    @staticmethod
    def amplitude_violation(u_max, u_min):                                                              # 引数(u_optの最大値, u_optの最小値)
        """必須条件の帯までの距離を返す（0なら帯の中）。

        必須条件は「大きすぎても小さすぎてもいけない」帯状の条件なので、下限だけの判定にはできない。
        正側・負側それぞれについて帯までの距離を測り、近いほうの距離を返す。どちらか一方の帯に
        入れば条件を満たすので、探索はこの距離が小さくなる方向へ進めばよい。
        （採否の判定そのものは等号の扱いまで含めて amplitude_satisfied() で行う）
        """
        lo, hi = TUNE_U_BAND_LO, TUNE_U_BAND_HI                                                         # 帯の下限・上限
        if u_max <= lo:                                                                                 # 正側が小さすぎる（帯の下）
            d_pos = lo - u_max
        elif u_max > hi:                                                                                # 正側が大きすぎる（帯の上）
            d_pos = u_max - hi
        else:                                                                                           # 正側が帯の中にある
            d_pos = 0.0
        if u_min >= -lo:                                                                                # 負側が小さすぎる（帯の上）
            d_neg = u_min + lo
        elif u_min < -hi:                                                                               # 負側が大きすぎる（帯の下）
            d_neg = -hi - u_min
        else:                                                                                           # 負側が帯の中にある
            d_neg = 0.0
        dist = min(d_pos, d_neg)                                                                        # どちらか一方を満たせばよいので小さいほう
        if TUNE_U_ABS_LIMIT is not None:                                                                # 追加条件（設定時のみ）: |u_opt| 全体の上限
            dist += max(0.0, max(abs(u_max), abs(u_min)) - TUNE_U_ABS_LIMIT)                            # 上限の超過分を距離へ加える
        return float(dist)

    # ------------------------------------------------------------------
    # 評価関数：候補の重みから u_opt を計算し、真のモデルへ入力して追従誤差Jを求める
    # ------------------------------------------------------------------
    def evaluate(self, Q, R, R_du):                                                                     # 引数(状態重み行列, 入力重み行列, 入力レート重み)
        """候補 (COST_Q, COST_R, COST_R_DU) を1つ評価する（ロボットは動かさず、シミュレーションのみ）"""
        m = self.model                                                                                  # 対象DOFの同定済みモデルを取り出す
        T1_sys, zeta_sys, wn_sys, b0_sys = m['sys_params']                                              # 真のモデル（システムモデル）のパラメータを取り出す
        a2, a1, a0 = MathematicalSolver.system_coeffs(T1_sys, zeta_sys, wn_sys)                         # 3次遅れ系の係数へ展開
        solver = MathematicalSolver(self.T, self.dt, Q=Q, R=R, R_du=R_du)                               # 候補の重みを持つソルバーを生成（calculate_el_ff は一切変更しない）

        # 手順1,2: 真のモデル（システムモデル）と目標モデルを使い、オイラー・ラグランジュ法で最適制御入力を計算
        ff_params, extrema, u_pred_full, y_tgt, u_opt, u_ff_opt = solver.calculate_el_ff(                # 5次関数パラメータ、極値、5次関数FF入力、目標モデル応答、総最適入力、ELのFF入力
            m['tgt_params'], m['sys_params'], m['u_ff'], m['y0'], m['u_adrc']
        )

        # 手順3: u_opt を真のモデルへ入力して応答を計算
        #   calculate_el_ff と同じ z = P - Pi の系（z(0)=0）でシミュレーションする。u_opt は
        #   総入力 u_ADRC + u_FF_opt なので、そのまま印加すれば実機に対応した応答になる。
        #   目標軌道 y_tgt は偏差系（Pf基準）で返るので、z に y0 を足して偏差系へそろえてから比較する。
        z_sys, _, _ = solver.simulate_forced(solver.t_eval, a2, a1, a0, b0_sys, u_opt, 0.0)              # 真のモデルの応答（総最適入力 u_opt を印加）
        y_sys = z_sys + m['y0']                                                                          # 偏差系（Pf基準）へ戻す

        # 手順4: 目標軌道（目標モデル）との差 J を計算
        J = solver.compute_J(y_tgt, y_sys)                                                               # 追従誤差 J = Σ(y_tgt - y_sys)²

        # 参考情報: 実機へ実際に送るのは u_FF_opt を5次多項式近似したFF入力なので、その応答も評価しておく
        #   このときの総入力は u_ADRC + （5次関数FF）になる。
        z_ff, _, _ = solver.simulate_forced(solver.t_eval, a2, a1, a0, b0_sys, m['u_adrc'] + u_pred_full, 0.0)   # 真のモデルの応答（ADRC + 5次関数FF を印加）
        y_ff = z_ff + m['y0']                                                                            # 偏差系（Pf基準）へ戻す
        J_ff = solver.compute_J(y_tgt, y_ff)                                                             # 5次関数FF入力での追従誤差

        # 振幅の必須条件は、実機へ送るFF入力そのもの（＝ELが最適化した u_FF_opt）に対して判定する
        #   u_opt は u_ADRC を含む総入力なので、FFの振幅条件の判定対象としては適切でない。
        u_max = float(np.max(u_ff_opt))                                                                  # u_FF_opt の最大値
        u_min = float(np.min(u_ff_opt))                                                                  # u_FF_opt の最小値
        ff_max = float(np.max(u_pred_full))                                                              # 5次関数FF入力の最大値
        ff_min = float(np.min(u_pred_full))                                                              # 5次関数FF入力の最小値

        # 手順5: 必須条件（対象DOFの入力振幅）の判定
        violation = self.amplitude_violation(u_max, u_min)                                               # 必須条件からの距離（0なら条件を満たす）
        violation_ff = self.amplitude_violation(ff_max, ff_min)                                          # 5次関数FF入力についての同じ判定

        return {
            'J': J, 'J_ff': J_ff,                                               # 追従誤差（u_opt印加時 / 5次関数FF印加時）
            'u_max': u_max, 'u_min': u_min,                                     # u_FF_opt の振幅（必須条件の判定対象）
            'ff_max': ff_max, 'ff_min': ff_min,                                 # 5次関数FF入力の振幅
            'violation': violation, 'violation_ff': violation_ff,               # 必須条件の帯までの距離（0なら帯の中）
            'satisfied': self.amplitude_satisfied(u_max, u_min),                # u_FF_opt が必須条件を満たすか（等号の扱いまで厳密に判定）
            'satisfied_ff': self.amplitude_satisfied(ff_max, ff_min),           # 5次関数FF入力が必須条件を満たすか
            'sat_ratio': float(np.mean(np.abs(u_opt) >= 254.9)),                # 総入力がPWM上限(±255)に張り付いている割合
            'u_end': float(u_ff_opt[int(round(self.T / self.dt))]),             # FF区間終端 u_FF_opt(T) の値（端点条件どおり0になる）
            'ff_params': list(ff_params), 'extrema': tuple(extrema),            # 5次関数パラメータと極値
            'u_opt': u_opt, 'u_ff_opt': u_ff_opt,                               # 総最適入力とELのFF入力
            'u_adrc': m['u_adrc'], 'u_pred_full': u_pred_full,                  # 固定のADRC入力と5次関数FF入力
            'y_tgt': y_tgt, 'y_sys': y_sys, 'y_ff': y_ff,                       # 目標モデル応答と真のモデル応答
            'Q': np.array(Q, dtype=float), 'R': np.array(R, dtype=float),       # 評価した重み行列
            'R_du': float(R_du),                                                # 評価した入力レート重み
        }

    # ------------------------------------------------------------------
    # Optunaの目的関数（最小化）
    # ------------------------------------------------------------------
    def objective(self, trial):                                                                         # 引数(Optunaのトライアル)
        q1 = trial.suggest_float('q1', *TUNE_Q_RANGE, log=True)                                         # 状態誤差の重み q1（位置誤差, 対数スケール）
        q2 = trial.suggest_float('q2', *TUNE_Q_RANGE, log=True)                                         # 状態誤差の重み q2（速度誤差, 対数スケール）
        q3 = trial.suggest_float('q3', *TUNE_Q_RANGE, log=True)                                         # 状態誤差の重み q3（加速度誤差, 対数スケール）
        r = trial.suggest_float('r', *TUNE_R_RANGE, log=True)                                           # 制御入力の重み r（対数スケール）
        du = trial.suggest_float('du', *TUNE_R_DU_RANGE, log=True)                                      # 入力レート（Δu）の重み du（対数スケール）
        Q = np.diag([q1, q2, q3])                                                                       # 対角の状態重み行列 Q = diag(q1,q2,q3)
        R = np.array([[r]])                                                                             # 入力重み行列 R = [[r]]

        # 候補を評価（数値的に破綻した候補は大ペナルティで棄却する）
        try:
            res = self.evaluate(Q, R, du)                                                               # 追従誤差Jと最適入力の情報を取得
        except Exception as exc:                                                                        # 評価に失敗した候補
            self._log(f"  [trial {trial.number:4d}] 評価失敗のため棄却: {exc}")
            return TUNE_PENALTY * 100.0                                                                 # 最大級のペナルティを返す

        J = res['J']                                                                                    # 最適制御入力を印加したときの追従誤差
        if not np.isfinite(J):                                                                          # 発散した候補
            return TUNE_PENALTY * 100.0                                                                 # 最大級のペナルティを返す

        # 必須条件を満たさない候補には大ペナルティを与える
        #   帯までの距離に応じて連続的に増やし、探索が条件を満たす方向へ進むようにする。
        #   条件を満たす候補（score = J）は必ずこのペナルティより小さくなる。
        if res['satisfied']:
            score = J                                                                                   # 条件を満たすので追従誤差そのものを目的関数値とする
        else:
            score = TUNE_PENALTY * (1.0 + res['violation'] / TUNE_U_BAND_HI)                            # 帯から遠いほど大きいペナルティ

        # 結果表示用の付加情報をトライアルへ保存
        trial.set_user_attr('J', res['J'])                                                              # u_opt印加時の追従誤差
        trial.set_user_attr('J_ff', res['J_ff'])                                                        # 5次関数FF入力印加時の追従誤差
        trial.set_user_attr('u_max', res['u_max'])                                                      # u_FF_opt最大値
        trial.set_user_attr('u_min', res['u_min'])                                                      # u_FF_opt最小値
        trial.set_user_attr('ff_max', res['ff_max'])                                                    # 5次関数FF入力の最大値
        trial.set_user_attr('ff_min', res['ff_min'])                                                    # 5次関数FF入力の最小値
        trial.set_user_attr('satisfied', bool(res['satisfied']))                                        # u_optについての必須条件の判定
        trial.set_user_attr('satisfied_ff', bool(res['satisfied_ff']))                                  # 5次関数FF入力についての必須条件の判定
        trial.set_user_attr('violation', res['violation'])                                              # 必須条件からの距離
        trial.set_user_attr('sat_ratio', res['sat_ratio'])                                              # PWM飽和割合
        trial.set_user_attr('u_end', res['u_end'])                                                      # FF区間終端 u(T) の値

        # 手動値(COST_Q, COST_R, COST_R_DU)のトライアル（初回）は比較用に保存する
        if self.manual is None and TUNE_SEED_MANUAL and trial.number == 0:
            self.manual = res                                                                           # 手動値の評価結果を保存

        # 最良トライアルの結果（グラフ用の配列を含む）を保持する
        if score < self.best_score:
            self.best_score = score                                                                     # 最良の目的関数値を更新
            self.best = res                                                                             # 最良の評価結果を更新
            mark = "  <-- best"                                                                         # 最良更新の印
        else:
            mark = ""

        self._log(                                                                                      # トライアルごとの進捗ログ
            f"  [trial {trial.number:4d}/{self.n_trials}] "
            f"q=({q1:.3e},{q2:.3e},{q3:.3e}) r={r:.3e} du={du:.3e} | "
            f"J={J:.4e} u_FF_opt=[{res['u_min']:8.2f},{res['u_max']:8.2f}] "
            f"FF=[{res['ff_min']:7.2f},{res['ff_max']:7.2f}] "
            + ("OK " if res['satisfied'] else f"NG(d={res['violation']:.1f}) ")
            + mark
        )
        return score                                                                                    # 目的関数値（追従誤差 or ペナルティ）を返す

    # ------------------------------------------------------------------
    # 探索の実行
    # ------------------------------------------------------------------
    def run(self):
        m = self.model                                                                                  # 対象DOFの同定済みモデル
        self._log("=" * 110)
        self._log(
            f"  COST_Q, COST_R, COST_R_DU 自動探索開始  (Optuna/{self.sampler_name.upper()}, "
            f"探索回数={self.n_trials}, 対象DOF={m['dof']})"
        )
        self._log(
            f"    必須条件: {TUNE_U_BAND_LO:.0f} < max(u_opt) <= {TUNE_U_BAND_HI:.0f} "
            f"または -{TUNE_U_BAND_HI:.0f} <= min(u_opt) <= -{TUNE_U_BAND_LO:.0f}"
        )
        self._log(
            f"    真のモデル(システムモデル): T1={m['sys_params'][0]:.4f}, zeta={m['sys_params'][1]:.4f}, "
            f"wn={m['sys_params'][2]:.4f}, b0={m['sys_params'][3]:.4f}"
        )
        self._log(
            f"    目標モデル: T1={m['tgt_params'][0]:.4f}, wn={m['tgt_params'][1]:.4f} | "
            f"Pi={m['Pi']:.1f}, Pf={m['Pf']:.1f}, y0={m['y0']:.2f}, T={self.T:.2f}s"
        )
        self._log("=" * 110)

        optuna.logging.set_verbosity(optuna.logging.WARNING)                                             # Optuna自身のログは抑制する（進捗は自前で出力）
        if self.sampler_name == 'cmaes':                                                                 # CMA-ESサンプラー
            sampler = optuna.samplers.CmaEsSampler(seed=TUNE_SEED, n_startup_trials=10)
        else:                                                                                            # TPEサンプラー（既定）
            sampler = optuna.samplers.TPESampler(seed=TUNE_SEED, multivariate=True, n_startup_trials=20)
        self.study = optuna.create_study(direction='minimize', sampler=sampler)                          # 最小化のstudyを作成

        if TUNE_SEED_MANUAL:                                                                             # 手動値を初回トライアルとして必ず評価する
            self.study.enqueue_trial({
                'q1': float(COST_Q[0, 0]), 'q2': float(COST_Q[1, 1]), 'q3': float(COST_Q[2, 2]),
                'r': float(COST_R[0, 0]), 'du': float(COST_R_DU),
            })

        t_start = time.time()                                                                            # 探索開始時刻
        self.study.optimize(self.objective, n_trials=self.n_trials, show_progress_bar=False)             # 探索実行
        elapsed = time.time() - t_start                                                                  # 探索所要時間

        if self.best is None:                                                                            # 全トライアルが失敗した場合
            self._log("すべてのトライアルが評価に失敗しました。探索範囲を見直してください。")
            return None

        self.print_result(elapsed)                                                                       # 結果表示
        return self.best

    # 探索結果を表示する関数
    def print_result(self, elapsed=None):                                                                # 引数(探索所要時間)
        b = self.best                                                                                    # 最良の評価結果
        q1, q2, q3 = float(b['Q'][0, 0]), float(b['Q'][1, 1]), float(b['Q'][2, 2])                       # 最良のQの対角成分
        r = float(b['R'][0, 0])                                                                          # 最良のR
        du = float(b['R_du'])                                                                            # 最良のR_du
        n_ok = sum(1 for t in self.study.trials if t.user_attrs.get('satisfied', False))                  # 必須条件を満たしたトライアル数

        self._log("")
        self._log("=" * 110)
        self._log("  COST_Q, COST_R, COST_R_DU 自動探索の結果")
        self._log("=" * 110)
        self._log(f"  Best COST_Q    = np.diag([{q1:.6g}, {q2:.6g}, {q3:.6g}])")
        self._log(f"  Best COST_R    = np.array([[{r:.6g}]])")
        self._log(f"  Best COST_R_DU = {du:.6g}")
        self._log(f"  最終J          = {b['J']:.6f}   （目標軌道と真のモデル応答の二乗和誤差）")
        self._log(f"  u_FF_opt最大値 = {b['u_max']:.4f}")
        self._log(f"  u_FF_opt最小値 = {b['u_min']:.4f}")
        self._log("-" * 110)
        self._log(
            f"  必須条件 (DOF{self.model['dof']}: {TUNE_U_BAND_LO:.0f} < max <= {TUNE_U_BAND_HI:.0f} "
            f"または -{TUNE_U_BAND_HI:.0f} <= min <= -{TUNE_U_BAND_LO:.0f})"
        )
        self._log(
            "    u_FF_opt（ELのFF入力）       : "
            + ("満たす" if b['satisfied'] else f"★満たさない★（帯までの距離 {b['violation']:.2f}）")
            + f"   [min {b['u_min']:.2f}, max {b['u_max']:.2f}]"
        )
        self._log(
            "    5次関数FF（実機へ送る入力）  : "
            + ("満たす" if b['satisfied_ff'] else f"★満たさない★（帯までの距離 {b['violation_ff']:.2f}）")
            + f"   [min {b['ff_min']:.2f}, max {b['ff_max']:.2f}]"
        )
        self._log(f"  条件を満たしたトライアル数 : {n_ok} / {len(self.study.trials)}"
                  + ("" if n_ok > 0 else "  ← 0件です。探索回数を増やすか、探索範囲/帯の設定を見直してください"))
        self._log(f"  PWM上限(±255)への飽和割合  : {b['sat_ratio'] * 100:.1f} %")
        self._log(f"  FF区間終端 u_FF_opt(T)     : {b['u_end']:.4f}   （端点条件により0になる）")
        self._log(f"  5次関数FF印加時のJ         : {b['J_ff']:.6f}  "
                  f"（極値 t1={b['extrema'][0]:.3f}, y1={b['extrema'][1]:.2f}, "
                  f"t2={b['extrema'][2]:.3f}, y2={b['extrema'][3]:.2f}）")
        if self.manual is not None:                                                                      # 手動値との比較
            self._log("-" * 110)
            self._log(
                f"  比較: 手動値 COST_Q=diag({float(COST_Q[0,0]):.6g}, {float(COST_Q[1,1]):.6g}, "
                f"{float(COST_Q[2,2]):.6g}), COST_R=[[{float(COST_R[0,0]):.6g}]], COST_R_DU={float(COST_R_DU):.6g}"
            )
            self._log(
                f"        → J={self.manual['J']:.6f} (5次関数FF: {self.manual['J_ff']:.6f}), "
                f"u_FF_opt=[{self.manual['u_min']:.4f}, {self.manual['u_max']:.4f}], "
                f"FF=[{self.manual['ff_min']:.4f}, {self.manual['ff_max']:.4f}], "
                + ("条件を満たす" if self.manual['satisfied'] else "条件を満たさない")
            )
        if elapsed is not None:
            self._log(f"  探索所要時間 : {elapsed:.1f} 秒")
        self._log("=" * 110)
        self._log("  ↓ この3行を test_code3.py の COST_Q, COST_R, COST_R_DU に貼り替えてください")
        self._log(f"    COST_Q = np.diag([{q1:.6g}, {q2:.6g}, {q3:.6g}])")
        self._log(f"    COST_R = np.array([[{r:.6g}]])")
        self._log(f"    COST_R_DU = {du:.6g}")
        self._log("=" * 110)
        self._log("")

    # 全トライアルの結果をCSVへ保存する関数
    def save_trials_csv(self, path):                                                                     # 引数(保存先パス)
        df = self.study.trials_dataframe()                                                               # 全トライアルの結果をDataFrameへ変換
        df.to_csv(path, index=False)                                                                     # CSVへ保存
        self._log(f"全トライアル結果を保存: {path}")

    # 最良結果の応答と入力をグラフ化して保存する関数
    def save_result_plot(self, path):                                                                    # 引数(保存先パス)
        b = self.best                                                                                    # 最良の評価結果
        Pf = self.model['Pf']                                                                            # 目標位置（POT値へ戻すためのオフセット）
        fig, axes = plt.subplots(2, 1, figsize=(9, 8))                                                    # 応答用と入力用の2段グラフ

        axes[0].plot(self.t_eval, b['y_tgt'] + Pf, '--', label='Target model (y_tgt)')                    # 目標軌道
        axes[0].plot(self.t_eval, b['y_sys'] + Pf, '-', label='True model with u_opt')                    # 最適入力を印加した真のモデルの応答
        axes[0].plot(self.t_eval, b['y_ff'] + Pf, ':', label='True model with 5th-order FF')              # 5次関数FF入力を印加した応答
        axes[0].set_title(f"DOF {self.model['dof']} response (J = {b['J']:.2f})")
        axes[0].set_xlabel('Time [s]')
        axes[0].set_ylabel('POT value')
        axes[0].legend()
        axes[0].grid(True)

        axes[1].plot(self.t_eval, b['u_ff_opt'], '-', label='EL FF input (u_FF_opt)')                     # ELが最適化したFF入力
        axes[1].plot(self.t_eval, b['u_pred_full'], '--', label='5th-order FF (u_FF)')                    # u_FF_optを5次関数で近似したFF入力
        for level in (TUNE_U_BAND_LO, TUNE_U_BAND_HI, -TUNE_U_BAND_LO, -TUNE_U_BAND_HI):                  # 必須条件の帯
            axes[1].axhline(level, color='gray', lw=0.8, ls='-.')
        axes[1].set_title(f"Input (max = {b['u_max']:.2f}, min = {b['u_min']:.2f})")
        axes[1].set_xlabel('Time [s]')
        axes[1].set_ylabel('Input value [PWM]')
        axes[1].legend()
        axes[1].grid(True)

        fig.tight_layout()                                                                                # レイアウト調整
        fig.savefig(path)                                                                                 # PNGへ保存
        plt.close(fig)                                                                                    # グラフを閉じる
        self._log(f"最良結果のグラフを保存: {path}")

    # 最良の重みと評価結果をCSVへ1行追記する関数
    def save_best_to_csv(self, path):                                                                     # 引数(保存先パス)
        b = self.best                                                                                     # 最良の評価結果
        row = {
            'dof': self.model['dof'], 'T': self.T, 'n_trials': self.n_trials, 'sampler': self.sampler_name,
            'band_lo': TUNE_U_BAND_LO, 'band_hi': TUNE_U_BAND_HI,
            'q1': float(b['Q'][0, 0]), 'q2': float(b['Q'][1, 1]), 'q3': float(b['Q'][2, 2]),
            'r': float(b['R'][0, 0]), 'du': float(b['R_du']),
            'J': b['J'], 'J_5th_order_ff': b['J_ff'], 'u_opt_max': b['u_max'], 'u_opt_min': b['u_min'],
            'ff_max': b['ff_max'], 'ff_min': b['ff_min'], 'u_end': b['u_end'],
            'satisfied': b['satisfied'], 'satisfied_ff': b['satisfied_ff'], 'sat_ratio': b['sat_ratio'],
            'ff_a': b['ff_params'][0], 'ff_b': b['ff_params'][1], 'ff_c': b['ff_params'][2],
            'ff_d': b['ff_params'][3], 'ff_e': b['ff_params'][4],
            't1': b['extrema'][0], 'y1': b['extrema'][1], 't2': b['extrema'][2], 'y2': b['extrema'][3],
            'Pi': self.model['Pi'], 'Pf': self.model['Pf'], 'y0': self.model['y0'],
            'sys_T1': self.model['sys_params'][0], 'sys_zeta': self.model['sys_params'][1],
            'sys_wn': self.model['sys_params'][2], 'sys_b0': self.model['sys_params'][3],
            'tgt_T1': self.model['tgt_params'][0], 'tgt_wn': self.model['tgt_params'][1],
        }
        df = pd.DataFrame([row])                                                                          # データを横並びにする
        header = not os.path.exists(path)                                                                 # 既存ファイルがあるか判定
        df.to_csv(path, mode='a', header=header, index=False)                                             # CSVへ追記
        self._log(f"最良の重みをCSVへ保存: {path}")


# ==============================================================================
# 同定結果（真のモデル）の保存・読み込み
#   ロボットを1回動かして得た同定結果をJSONへ保存しておき、以降の再探索では
#   これを読み込むことでロボットを一切動かさずに済むようにする。
# ==============================================================================

# 同定結果をJSONへ保存する関数
def save_identified_models(path, T, dt, models, initial_pot, target_pot):                                  # 引数(保存先パス, FF制御入力時間, ステップ時間, 24自由度分の同定結果, 初期姿勢, 目標値)
    payload = {
        'T': float(T), 'dt': float(dt), 'sim_time': SIM_TIME,                                             # 実験条件
        'initial_pot': [float(v) for v in initial_pot],                                                    # 初期姿勢
        'target_pot': [float(v) for v in target_pot],                                                      # 目標値
        'models': [
            {
                'dof': int(m['dof']), 'Pi': float(m['Pi']), 'Pf': float(m['Pf']), 'y0': float(m['y0']),    # 位置情報
                'tgt_params': [float(v) for v in m['tgt_params']],                                         # 目標モデル [T1, wn]
                'sys_params': [float(v) for v in m['sys_params']],                                         # 真のモデル [T1, zeta, wn, b0]
                'ff_params': [float(v) for v in m['ff_params']],                                           # 実測時に印加した励振FFの5次係数
                'u_hold': float(m['u_hold']),                                                              # 同定入力の基準値（初期姿勢の保持PWM）
                'y_data': [float(v) for v in m['y_data']],                                                 # 実測POT値（同定結果の確認用）
                'u_ident': [float(v) for v in m['u_ident']],                                               # 同定に使った入力 u_pwm - u_hold（同定結果の確認用）
                'u_adrc': [float(v) for v in m['u_adrc']],                                                 # 実測から復元したADRC入力（EL最適化中は固定）
            }
            for m in models
        ],
    }
    with open(path, 'w') as f:
        json.dump(payload, f, indent=2)                                                                    # JSONへ保存


# 同定結果をJSONから読み込む関数
def load_identified_models(path):                                                                          # 引数(読み込みパス)
    with open(path, 'r') as f:
        payload = json.load(f)                                                                             # JSONを読み込む
    T = float(payload['T'])                                                                                # FF制御入力時間
    dt = float(payload['dt'])                                                                              # シミュレーションステップ時間
    t_eval = np.arange(0, SIM_TIME, dt)                                                                    # シミュレーション時間配列
    t_ff = t_eval[t_eval <= T]                                                                             # FF入力を与える時間配列
    models = []                                                                                            # 同定結果のリスト
    for m in payload['models']:
        a, b, c, d, e = m['ff_params']                                                                     # 実測時に印加した励振FFの5次係数
        u_ff = np.zeros(len(t_eval))                                                                       # 入力ベクトルの生成
        u_ff[:len(t_ff)] = a * t_ff ** 5 + b * t_ff ** 4 + c * t_ff ** 3 + d * t_ff ** 2 + e * t_ff        # FF入力時間だけFF入力を格納
        models.append({
            'dof': int(m['dof']), 'Pi': float(m['Pi']), 'Pf': float(m['Pf']), 'y0': float(m['y0']),
            'tgt_params': np.array(m['tgt_params'], dtype=float),                                          # 目標モデル [T1, wn]
            'sys_params': np.array(m['sys_params'], dtype=float),                                          # 真のモデル [T1, zeta, wn, b0]
            'ff_params': list(m['ff_params']),
            'u_hold': float(m.get('u_hold', 0.0)),
            'y_data': np.array(m.get('y_data', []), dtype=float),
            'u_ident': np.array(m.get('u_ident', []), dtype=float),
            'u_adrc': np.array(m.get('u_adrc', np.zeros(len(t_eval))), dtype=float),                        # 固定のADRC入力（旧JSONには無いので0で代用）
            'u_ff': u_ff,
        })
    return T, dt, models


# ==============================================================================
# ROS2 ノード
# ==============================================================================
class OptimalControlSequencer(Node):
    # ★ 探索結果グラフ出力の切り替え (True: 有効, False: 無効)
    SAVE_RESULT_PLOT = TUNE_SAVE_PLOT

    # コンストラクタ
    def __init__(self, csv_path, T, n_trials=TUNE_N_TRIALS, target_mode="1"):                                                                   # 引数(保存csv情報, FF制御入力時間, 重みの探索回数, 目標値の与え方がランダムorプリセット)
        super().__init__('weight_tuning_sequencer_el')                                                                                          # ROS2ノードとして登録
        self.csv_path = csv_path                                                                                                                # csv情報を格納
        self.T = T                                                                                                                              # FF制御入力時間を格納
        self.n_trials = n_trials                                                                                                                # 重みの探索回数を格納
        self.target_mode = target_mode                                                                                                          # 目標値の与え方のモードを格納

        self.current_outer = 0                                                                                                                  # 目標値の選択に使う試行番号（ロボットは1回しか動かさないので0のまま）

        self.initial_stabilize_time = INIT_WAIT_TIME                                                                                            # 初期姿勢への移動後の待機時間
        self.data_collection_time = SIM_TIME                                                                                                    # データ取得時間
        self.dt = SIM_DT                                                                                                                        # シミュレーションステップ時間

        self.state = "INIT_ROBOT"                                                                                                               # ロボット状態（初期位置へ送る状態）

        # 出力ファイルのパスを作成（選択したCSVパスを基準にする）
        base_path = os.path.splitext(self.csv_path)[0]                                          # csvの拡張子を除いたパス
        self.model_path = base_path + "_identified_models.json"                                 # 同定結果（真のモデル）の保存先
        self.trials_csv_path = base_path + "_weight_trials.csv"                                 # 全トライアル結果の保存先
        self.plot_path = base_path + "_weight_best.png"                                         # 最良結果のグラフの保存先
        self.ident_plot_path = base_path + "_identification.png"                                # 対象DOFの同定結果グラフの保存先

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

        self.current_ff_matrix = [list(MathematicalSolver.initial_ff_params(self.T)) for _ in range(24)]                                        # 24自由度分のFF係数（同定の励振用に非ゼロ初期値: f(0)=f(T)=0, f(t1)=+peak, f(t2)=-peak）

        # 受信バッファは (受信時刻, 値リスト) の組を1回のappendで積む。
        #   時刻と値を別々のリストへ積むと、購読コールバックを別スレッドで動かしたときに
        #   「値だけ入って時刻がまだ」という中間状態が読み出され、時刻と値の対応がずれる。
        self.buffer_pot = {f'board{i}': [] for i in range(1, 6)}                                                                                # 5board分のPOT受信データ [(経過秒, [POT値, ...]), ...]
        self.buffer_pwm = {f'board{i}': [] for i in range(1, 6)}                                                                                # 5board分の実制御入力（PWM）受信データ（システムモデル同定の入力に使う）
        self.collect_start_time = None                                                                                                          # データ収集を開始した時刻（受信時刻の基準）
        self.shutdown_event = threading.Event()                                                                                                 # 全試行の終了要求（main()の実行ループを抜けるために使う）
        self.pwm_hold_window = {f'board{i}': deque(maxlen=HOLD_PWM_WINDOW) for i in range(1, 6)}                                                # 直近PWMのリング（保持PWMの推定に使う。状態によらず常時更新）
        self.hold_pwm_24 = None                                                                                                                 # 目標値切替直前の保持PWM（24自由度分。同定入力の基準値）
        self.target_pot = self.get_next_target_positions()                                                                                      # 最初にロボットへ送る目標値を決定

        # Publisher作成
        self.pub_target = self.create_publisher(Float32MultiArray, '/board_android_float/sub', 10)                                              # 目標値パブリッシャーを作成
        self.pub_ff = {}                                                                                                                        # FFパラメータ係数を送信するための辞書を初期化
        for i in range(1, 6):                                                                                                                   # boardごとにFFパラメータ係数パブリッシャーを作成
            self.pub_ff[i] = self.create_publisher(Float32MultiArray, f'/board{i}_FFparam_float/sub', 10)

        # Subscriber作成
        #   受信時刻は「コールバックが動いた時刻」なので、実行器が詰まるとその分だけ時刻がずれ、
        #   詰まっている間に届いたデータは受信キュー（深さ10）から溢れて消える。消えた区間は
        #   補間で直線に化けるため、購読はシーケンサ（FF送信・ログ出力・sleep）と同じスレッドで
        #   動かしてはいけない。購読ごとに別のコールバックグループを与え、MultiThreadedExecutor
        #   （main参照）で並列に動かすことで、シーケンサの処理時間が受信時刻に混入しないようにする。
        for i in range(1, 6):                                                                                                                   # POT値のSubscriberをboardごとに作成
            self.create_subscription(UInt16MultiArray, f'/board{i}_tk/pub',
                                     lambda m, i=i: self.cb_board(m, i), 10,
                                     callback_group=MutuallyExclusiveCallbackGroup())

        # 実制御入力（PWM）のSubscriber作成（各board 6要素のFloat32MultiArray）
        for i in range(1, 6):                                                                                                                   # PWM値のSubscriberをboardごとに作成
            self.create_subscription(Float32MultiArray, f'/board{i}_tk_PWM_float/pub',
                                     lambda m, i=i: self.cb_pwm(m, i), 10,
                                     callback_group=MutuallyExclusiveCallbackGroup())

        # タイマー作成（購読とは別グループにして、シーケンサの処理が受信を止めないようにする）
        self.control_timer = self.create_timer(0.1, self.sequencer_loop, callback_group=MutuallyExclusiveCallbackGroup())

        self.state_start_time = self.get_clock().now()                                                                                          # 状態開始時刻の保存

    # 送信する目標値を決める関数
    def get_next_target_positions(self):
        if self.target_mode == "1" and self.current_outer < len(self.preset_targets):                                                           # 目標値送信モードが1で、外側ループ回数がプリセットの数より小さければ、プリセットした目標値を送る
            return self.preset_targets[self.current_outer]
        return [float(np.random.randint(b[0], b[1] + 1)) for b in self.pot_bounds]                                                              # それ以外はランダムにPOT範囲から送る

    # ROS2 Subscriberのコールバック関数
    def cb_board(self, msg, board_id):                                                                                                          # 引数(受信メッセージ, board番号)
        if self.state != "COLLECTING":                                                                                                          # 実測データ収集状態のみ蓄積する（それ以外は受信キューを空けるだけ）
            return
        stamp = self.elapsed_since_collect_start()                                                                                              # 受信時刻（データ収集開始からの経過秒）はメッセージを触る前に取る
        idx_use = [0, 1, 2, 6, 7, 8] if board_id in [4, 5] else list(range(6))                                                                  # board4と5は3自由度ぶん（index0～2, 6～8）だけ使う
        if len(msg.data) <= idx_use[-1]:                                                                                                        # 要素数が足りない受信データは捨てる（例外でコールバックが止まるのを防ぐ）
            return
        values = [float(msg.data[k]) for k in idx_use]                                                                                          # 使用する自由度分のPOT値を取り出す
        self.buffer_pot[f'board{board_id}'].append((stamp, values))                                                                             # 受信時刻と値を1組にしてバッファへ格納（appendは1回なのでスレッド間でずれない）

    # 実制御入力（PWM）Subscriberのコールバック関数
    def cb_pwm(self, msg, board_id):                                                                                                            # 引数(受信メッセージ, board番号)
        n_use = 3 if board_id in [4, 5] else 6                                                                                                  # board4と5はindex0～2のみ最適化対象（index3～5は届くが使用しない）
        if len(msg.data) < n_use:                                                                                                               # 必要な要素数に満たない受信データは捨てる
            return
        stamp = self.elapsed_since_collect_start() if self.state == "COLLECTING" else None                                                       # 受信時刻（データ収集開始からの経過秒）はメッセージを触る前に取る
        values = [float(v) for v in msg.data[:n_use]]                                                                                           # 使用する自由度分のPWMを取り出す
        self.pwm_hold_window[f'board{board_id}'].append(values)                                                                                 # 保持PWM推定用のリングは状態によらず常に更新する
        if stamp is not None:                                                                                                                   # 実測データ収集状態のみバッファへ蓄積
            self.buffer_pwm[f'board{board_id}'].append((stamp, values))                                                                         # 受信時刻と値を1組にしてバッファへ格納

    # 目標値切替直前の保持PWM（同定入力の基準値）を24自由度分スナップショットする関数
    def capture_hold_pwm(self):
        """初期姿勢で静止しているあいだに印加されているPWMを24自由度分そろえて返す。

        z（初期位置を0とする）形式の同定では、入力の基準は「初期姿勢を保持していたPWM」でなければ
        ならない。目標値を切り替えた後の値を基準にすると、切替でPIDが跳ねた分だけ基準がずれ、b0が
        大きく化ける。切替前に静止しているあいだの直近サンプルを平均して基準値とする。
        """
        hold = []                                                                                                                               # 24自由度分の空リスト
        for b_id in range(1, 6):                                                                                                                # board1からboard5の順に処理
            window = list(self.pwm_hold_window[f'board{b_id}'])                                                                                 # 直近サンプルを取り出す
            n_dof = 3 if b_id in [4, 5] else 6                                                                                                  # board4と5は3自由度、それ以外は6自由度
            if window:                                                                                                                          # 直近サンプルがある場合
                mean = np.mean(np.array(window, dtype=float), axis=0)                                                                           # 静止中なので平均を取ってノイズを落とす
                hold.extend(float(mean[j]) for j in range(n_dof))                                                                               # 自由度ごとの保持PWMを格納
            else:                                                                                                                               # 1点も受信できていない場合
                hold.extend([None] * n_dof)                                                                                                     # 基準値なしとして格納
        return hold

    # データ収集開始からの経過時間を返す関数
    def elapsed_since_collect_start(self):
        if self.collect_start_time is None:                                                                                                     # 収集開始時刻が未設定の場合
            return 0.0
        return (self.get_clock().now() - self.collect_start_time).nanoseconds / 1e9                                                             # 経過秒を返す

    # 各boardが何秒分のデータを受信できたかを返す関数
    def collected_spans(self):
        """POT・PWMそれぞれについて {board名: 受信できた時間長[秒]} を返す（1点も受信していなければ0）。"""
        spans = {}                                                                                                                              # 空の辞書
        for i in range(1, 6):                                                                                                                   # board1からboard5
            key = f'board{i}'
            spans[f'POT-{key}'] = self.buffer_pot[key][-1][0] if self.buffer_pot[key] else 0.0                                                  # POTの最終受信時刻＝受信できた時間長
            spans[f'PWM-{key}'] = self.buffer_pwm[key][-1][0] if self.buffer_pwm[key] else 0.0                                                  # PWMの最終受信時刻＝受信できた時間長
        return spans

    # 受信時刻付きの時系列を、シミュレーションの時間軸へ線形補間して揃える関数
    @staticmethod                                                                                                                               # Pythonデコレータ（静的メソッド：selfを使わない）
    def resample_by_time(stamps, values, t_grid):                                                                                               # 引数(受信時刻の配列, 値の配列, 揃えたい時間軸)
        """配信周期のばらつきや取りこぼしがあっても、実際の受信時刻を使って t_grid 上の時系列へ復元する。

        受信データを「1サンプル = SIM_DT 秒」と決め打ちすると、配信が遅れたり取りこぼしたりした分だけ
        時間軸が縮んでしまう（500点届くはずが300点しか無いと3秒分のデータとして扱われる）。
        受信時刻で補間することで、届いた点数によらず必ず [0, SIM_TIME] 全体のデータになる。

        ただし補間はデータが無い区間を直線で埋めてしまう（範囲外は端の値で水平に伸ばす）。
        その区間が本物の実測に見えないよう、欠測の判定は gap_mask() で別に行うこと。
        """
        ts = np.asarray(stamps, dtype=float)                                                                                                    # 受信時刻をnumpy配列へ変換
        vs = np.asarray(values, dtype=float)                                                                                                    # 値をnumpy配列へ変換
        if ts.size == 0 or vs.size == 0:                                                                                                        # 1点も受信できていない場合
            return None                                                                                                                         # Noneを返して呼び出し側で処理させる
        order = np.argsort(ts, kind='stable')                                                                                                   # 受信時刻の昇順に並べ替える（np.interpは単調増加が前提）
        ts, vs = ts[order], vs[order]
        uniq_ts, uniq_idx = np.unique(ts, return_index=True)                                                                                    # 同一時刻の重複を取り除く
        return np.interp(t_grid, uniq_ts, vs[uniq_idx])                                                                                         # t_grid上へ線形補間した時系列を返す（範囲外は端の値で保持）

    # 補間で作られた（実測が無い）区間を判定する関数
    @staticmethod                                                                                                                               # Pythonデコレータ（静的メソッド：selfを使わない）
    def gap_mask(stamps, t_grid, max_gap=MAX_SAMPLE_GAP):                                                                                       # 引数(受信時刻の配列, 揃えたい時間軸, 欠測とみなす受信間隔)
        """t_grid の各点について「近くに実測サンプルが無い＝補間で作った値」かどうかを返す。

        受信が途切れると resample_by_time() はその間を直線で結ぶ。実機が振動していても
        グラフは水平な直線になり、しかもその直線がそのままモデル同定の入力になってしまう。
        最も近い実測サンプルまでの距離が max_gap を超える点を欠測として印を付け、
        表示（直線を実測として描かない）と警告（同定結果の信頼性）に使う。
        受信開始前・受信終了後（端の値で水平に伸ばされる区間）もここで欠測になる。
        """
        ts = np.unique(np.asarray(stamps, dtype=float))                                                                                         # 受信時刻を昇順・重複なしにする
        t = np.asarray(t_grid, dtype=float)
        if ts.size == 0:                                                                                                                        # 1点も受信できていない場合
            return np.ones(t.size, dtype=bool)                                                                                                  # 全点が欠測
        pos = np.searchsorted(ts, t)                                                                                                            # 各グリッド点が入る受信時刻の区間
        d_prev = np.where(pos > 0, t - ts[np.clip(pos - 1, 0, ts.size - 1)], np.inf)                                                             # 直前の実測サンプルまでの時間
        d_next = np.where(pos < ts.size, ts[np.clip(pos, 0, ts.size - 1)] - t, np.inf)                                                           # 直後の実測サンプルまでの時間
        return np.minimum(d_prev, d_next) > max_gap                                                                                             # 前後どちらも遠ければ補間で作った値

    # 受信の欠測を警告ログに出す関数
    def report_gaps(self, label, board_id, stamps, mask, t_grid):                                                                               # 引数(POT/PWMの別, board番号, 受信時刻, 欠測マスク, 時間軸)
        """受信が途切れた区間を警告する。

        補間は欠測を直線で埋めてしまうため、黙っていると「実機は振動しているのにグラフは水平」
        というデータがそのままモデル同定にも使われる。どのboardのどの時刻で何秒抜けたのかを
        必ずログに残し、収集をやり直すか通信を見直すかを判断できるようにする。
        """
        if not mask.any():                                                                                                                      # 欠測が無ければ何も言わない
            return
        interval = np.diff(np.unique(stamps))                                                                                                   # 実際の受信間隔
        rate = f"{len(stamps)}点, 平均{interval.mean() * 1000:.1f}ms間隔" if interval.size else f"{len(stamps)}点"                                 # 受信状況の要約
        spans = self.gap_spans(mask, t_grid)                                                                                                    # 欠測区間の一覧
        lost = float(mask.sum()) * self.dt                                                                                                      # 欠測の合計時間
        worst = max(spans, key=lambda s: s[1] - s[0])                                                                                           # 最も長い欠測区間
        self.get_logger().warn(
            f"{label}-board{board_id}: 受信が途切れました（{rate}）。"
            f"欠測 {len(spans)}箇所 / 合計 {lost:.2f}s / 最長 {worst[1] - worst[0]:.2f}s (t={worst[0]:.2f}～{worst[1]:.2f}s)。"
            f"この区間は補間の直線であり実測ではありません"
        )

    # 欠測マスクを (開始時刻, 終了時刻) の区間リストへ変換する関数
    @staticmethod                                                                                                                               # Pythonデコレータ（静的メソッド：selfを使わない）
    def gap_spans(mask, t_grid):                                                                                                                # 引数(欠測マスク, 時間軸)
        spans = []                                                                                                                              # 空のリスト
        edges = np.flatnonzero(np.diff(np.concatenate(([0], np.asarray(mask, dtype=np.int8), [0]))))                                             # 立ち上がり・立ち下がりの位置
        for lo, hi in zip(edges[0::2], edges[1::2]):                                                                                            # 2つずつ取り出して1区間にする
            spans.append((float(t_grid[lo]), float(t_grid[min(hi, t_grid.size - 1)])))
        return spans

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
                "=== システム同定用データ取得（ロボットを動かすのはこの1回だけ） ==="
            )
            self.publish_target_positions(self.initial_pot)                                                             # 初期姿勢を送信
            self.state = "WAIT_INITIAL_STABILIZE"                                                                       # 状態変更
            self.state_start_time = now                                                                                 # 現在時刻を取得

        # 初期姿勢への収束を待っている状態
        elif self.state == "WAIT_INITIAL_STABILIZE":
            if elapsed >= self.initial_stabilize_time:                                                                  # 待ち時間が終了したか判定
                # 同定入力の基準となる保持PWMは、FF・目標値を送る前（初期姿勢で静止しているうち）に取る
                self.hold_pwm_24 = self.capture_hold_pwm()                                                      # 目標値切替直前の保持PWMを24自由度分取得
                self.publish_all_ff_parameters()                                                                # FFパラメータを送信
                time.sleep(0.1)                                                                                 # 0.1秒待機
                self.publish_target_positions(self.target_pot)                                                  # 目標値送信
                # バッファは clear() ではなく新しい辞書へ差し替える。購読コールバックは別スレッドで
                # 動いているため、clear() だと「消した直後に古いデータが1点入る」ことがある。
                self.buffer_pot = {f'board{i}': [] for i in range(1, 6)}                                        # 各boardが持つPOTデータバッファを初期化
                self.buffer_pwm = {f'board{i}': [] for i in range(1, 6)}                                        # 各boardが持つPWMデータバッファを初期化
                # 収集開始時刻は、FF・目標値の送信（24自由度分の極値計算とログ出力を含む）が終わった後の
                # 時刻に取り直す。タイマー先頭の now を使うと、送信にかかった時間だけ収集時間が短くなる。
                self.collect_start_time = self.get_clock().now()                                                # 受信時刻の基準となるデータ収集開始時刻
                self.state = "COLLECTING"                                                                       # 状態変更
                self.state_start_time = self.collect_start_time                                                 # 収集開始時刻を状態開始時刻にも使う

        # ロボットの実測データを収集している状態
        elif self.state == "COLLECTING":
            spans = self.collected_spans()                                                                              # 各boardが何秒分のデータを受信できたかを取得
            min_span = min(spans.values())                                                                              # 最も受信が遅れているboardの時間長
            timeout = elapsed >= self.data_collection_time + COLLECT_EXTRA_WAIT                                          # 追加待機の上限を超えたか判定

            # 経過時間だけでなく「全boardが指定時間分のデータを受信できたか」も条件にする。
            # 配信の遅れや取りこぼしで短いデータのまま次へ進んでしまうのを防ぐ。
            if (elapsed >= self.data_collection_time and min_span >= self.data_collection_time - self.dt) or timeout:
                if timeout and min_span < self.data_collection_time - self.dt:                                   # 指定時間分そろわないまま打ち切った場合
                    short = {k: v for k, v in spans.items() if v < self.data_collection_time - self.dt}          # 不足しているboardを抽出
                    self.get_logger().warn(                                                                     # 警告ログを出力
                        f"データ収集が {self.data_collection_time:.1f}s 分そろいませんでした（"
                        + ", ".join(f"{k}={v:.2f}s" for k, v in sorted(short.items())) + "）"
                    )
                else:                                                                                           # 指定時間分そろった場合
                    n_pot = [len(self.buffer_pot[f'board{i}']) for i in range(1, 6)]                            # boardごとのPOT受信点数
                    n_pwm = [len(self.buffer_pwm[f'board{i}']) for i in range(1, 6)]                            # boardごとのPWM受信点数
                    self.get_logger().info(                                                                     # ログ出力
                        f"データ収集完了: {elapsed:.2f}s 経過 / 全board {min_span:.2f}s 分を受信 "
                        f"(受信点数 POT={n_pot}, PWM={n_pwm} → {int(SIM_TIME / self.dt)}点へ補間)"
                    )
                self.state = "PROCESSING"                                                                       # 状態変更
                self.publish_target_positions(self.initial_pot)                                                 # 初期姿勢を送信
                threading.Thread(target=self.dispatch_optimization_pipeline, daemon=True).start()               # dispatch_optimization_pipeline関数を新しいスレッドに追加し実行する

        # プログラムの最終状態
        elif self.state == "FINISHED":
            # ロボットを安定化させるために初期位置をPublish
            self.get_logger().info("=== 最終安定化: home_pot を送信します ===")
            self.publish_target_positions(self.home_pot)                                                                # オリジナルの初期姿勢を送信
            self.get_logger().info("COST_Q, COST_R, COST_R_DU の自動チューニングが正常終了しました。")
            self.control_timer.cancel()                                                                                 # タイマー停止
            self.shutdown_event.set()                                                                                   # main()の実行ループへ終了を伝える（MultiThreadedExecutorは例外を外へ出さない）
    
    # 実測データから次回のFF入力を計算する関数（スレッド関数）
    def dispatch_optimization_pipeline(self):
        try:
            # 26要素と最適化する24自由度の対応表を作成
            dof_map = []                                                                                                # 26要素→24自由度への対応表
            for i in range(1, 4):                                                                                       # board1、board2、board3
                for j in range(6):
                    dof_map.append((i - 1) * 6 + j)                                                         # 0~17 DOF               
            for i in [4, 5]:                                                                                            # board4、board5
                for j in range(3):
                    dof_map.append(18 + (i - 4) * 4 + j)                                                    # 18, 19, 20, 22, 23, 24 DOF

            # 24自由度それぞれに対して、目標モデル同定・システムモデル同定・オイラーラグランジュ最適制御を実行する
            solver = MathematicalSolver(self.T, self.dt)                                                                # MathematicalSolverクラスの生成
            N_samples = int(SIM_TIME / self.dt)                                                                         # サンプル数の計算
            t_grid = solver.t_eval                                                                                      # シミュレーションの時間軸（0秒～SIM_TIME, N_samples点）

            # Boardごとに保存されている時系列を、受信時刻を使って24自由度分のシミュレーション時間軸へ揃える関数
            #   1サンプル=SIM_DT秒と決め打ちせず実際の受信時刻で補間するため、配信の遅れや取りこぼしが
            #   あっても必ず [0, SIM_TIME] 全体をカバーした N_samples 点のデータになる。
            #   ただし受信が途切れた区間は補間が直線で埋めてしまうので、その位置も一緒に返して
            #   表示（実測として描かない）と警告（同定結果の信頼性）に使えるようにする。
            def build_24_dof(buffer, label):                                                                            # 引数(受信バッファ, ログ表示名)
                out, gaps = [], []                                                                                      # 24自由度分の値と欠測マスクの空リスト
                for b_id in range(1, 6):                                                                                # board1からboard5の順に処理
                    samples = list(buffer[f'board{b_id}'])                                                  # 各boardに対応する受信データ（受信時刻, 値）を取りだす
                    n_dof = 3 if b_id in [4, 5] else 6                                                      # board4と5は3自由度、それ以外は6自由度
                    if not samples:                                                                         # 1点も受信できなかった場合
                        out.extend([None] * n_dof); gaps.extend([None] * n_dof)                             # 未受信として格納
                        self.get_logger().warn(f"{label}-board{b_id}: 1点も受信できませんでした")
                        continue
                    stamps = np.array([s[0] for s in samples], dtype=float)                                 # 受信時刻の配列
                    arr = np.array([s[1] for s in samples], dtype=float)                                    # 受信値の配列
                    gap = self.gap_mask(stamps, t_grid)                                                     # 欠測マスク（受信時刻はboard内の全自由度で共通）
                    self.report_gaps(label, b_id, stamps, gap, t_grid)                                      # 欠測があれば警告ログを出す
                    for j in range(n_dof):                                                                  # そのboardの自由度の順に処理
                        out.append(self.resample_by_time(stamps, arr[:, j], t_grid))                        # 受信時刻で補間して格納
                        gaps.append(gap)
                return out, gaps

            data_24_dof, gap_24_dof = build_24_dof(self.buffer_pot, "POT")                                              # 実測POT値を24自由度分へ変換
            pwm_24_dof, _ = build_24_dof(self.buffer_pwm, "PWM")                                                        # 実制御入力（PWM）を24自由度分へ変換

            models = []                                                                                                 # 24自由度分の同定結果（＝真のモデル）の空リスト

            for dof_idx in range(24):                                                                                   # 24自由度分のループ
                if data_24_dof[dof_idx] is None:                                                                        # POT値を1点も受信できなかった場合
                    raise RuntimeError(f"DOF {dof_idx + 1:02d}: POT値を1点も受信できませんでした")                        # 同定できないため異常として扱う
                raw_y = data_24_dof[dof_idx]                                                                            # 5秒間分（N_samples点）に揃え済みの実測データ
                y_gap = gap_24_dof[dof_idx]                                                                             # 実測が無く補間の直線になっている区間

                # 欠測が多いと、水平な直線をそのまま同定してしまい振動しないモデルになる
                if y_gap.mean() > GAP_WARN_RATIO:
                    self.get_logger().warn(
                        f"DOF {dof_idx + 1:02d}: 実測データの {y_gap.mean() * 100:.0f}% が補間です。"
                        f"同定結果（特に zeta・wn）は信用できません"
                    )

                # Initial and Target values
                Pi = self.initial_pot[dof_map[dof_idx]]                                                     # 現在のDOFの初期位置を取得
                Pf = self.target_pot[dof_map[dof_idx]]                                                      # 現在のDOFの目標位置を取得
                y0 = Pi - Pf                                                                                # 初期偏差を計算

                # Shift data so it converges to 0
                y_shifted = raw_y - Pf                                                                      # 目標位置を原点（0）に移動するための処理

                # 1. Target Model ID
                tgt_params = solver.fit_target_model(y_shifted, y0)                                          # 目標モデルの同定をして、パラメータを取得（同定は1回だけなのでウォームスタートは無し）

                # 2. System Model ID
                a, b, c, d, e = self.current_ff_matrix[dof_idx]                                             # 現在のDOFのFFパラメータを取り出す
                t_ff = solver.t_eval[solver.t_eval <= self.T]                                               # 0秒～FF入力時間までの時間配列を取り出す
                u_ff = np.zeros(N_samples)                                                                  # 入力ベクトルの生成
                u_ff[:len(t_ff)] = a * t_ff ** 5 + b * t_ff ** 4 + c * t_ff ** 3 + d * t_ff ** 2 + e * t_ff # FF入力時間だけ、そのときのFF入力を格納する

                # 同定入力は「送信したFF入力」ではなく「実際にそのDOFを動かした制御入力（PWM）」を使う
                u_pwm = pwm_24_dof[dof_idx]                                                                 # 受信時刻でシミュレーションの時間軸へ揃え済みの実測PWM
                if u_pwm is None:                                                                           # PWMを1点も受信できなかった場合
                    self.get_logger().warn(                                                                 # 警告ログを出力
                        f"DOF {dof_idx + 1:02d}: PWMを受信できませんでした。FF入力でシステムモデルを同定します。"
                    )
                    u_pwm = u_ff.copy()                                                                     # 従来どおりFF入力で代用する
                    u_hold = 0.0                                                                            # FF入力は0から始まるので基準値も0
                else:                                                                                       # PWMを受信できた場合
                    u_hold = self.hold_pwm_24[dof_idx] if self.hold_pwm_24 is not None else None            # 目標値切替直前の保持PWM（同定入力の基準値）
                    if u_hold is None:                                                                      # 保持PWMを取得できなかった場合
                        self.get_logger().warn(                                                             # 警告ログを出力
                            f"DOF {dof_idx + 1:02d}: 保持PWMを取得できませんでした。収集開始時のPWMで代用します。"
                        )
                        u_hold = float(u_pwm[0])                                                            # 収集開始時のPWMで代用する（精度は落ちる）

                # システムモデル同定は「初期位置を0として目標値へ変化する系」 z = P - Pi で行う
                #   偏差系 y = P - Pf（y→0）だと、姿勢を保持するための定常入力が残るせいで
                #   「定常入力があるのに出力0」となり、b0 が0へ潰れて同定できない。
                #   z 形式では次式が厳密に成り立ち、姿勢依存の重力保持分 u_g が式から消える。
                #     z''' + a2z'' + a1z' + a0z = b0 (u - u_hold),  z(0)=ż(0)=z̈(0)=0
                #     u_hold = 初期姿勢 Pi を保持していたPWM（= u_g + a0*y0/b0）
                z_data = raw_y - Pi                                                                         # 初期位置を原点（0）にした実測データ
                u_ident = u_pwm - u_hold                                                                    # 同定に使用する入力（初期姿勢の保持分を除去）

                sys_params = solver.fit_system_model(z_data, u_ident, 0.0)                                  # システムモデルの同定をして、パラメータを取得（初期値0。同定は1回だけなのでウォームスタートは無し）

                # 実測総入力からADRC入力の時系列を復元する（u_actual = u_ADRC + u_FF_actual）
                #   実際にロボットへ送ったFF入力 u_ff を引けばADRC分だけが残る。t > T では u_ff = 0。
                #   この時系列は以降の重み探索中ずっと固定し、一切変化させない。
                u_adrc = u_ident - u_ff                                                                     # ADRC入力 u_ADRC = u_actual - u_FF_actual

                # 3. 同定結果の確認（同定した「真のモデル」が実測をどれだけ再現できているか）
                T1_sys, zeta_sys, wn_sys, b0_sys = sys_params                                               # システムモデルのパラメータ[T1, zeta, wn, b0]を取り出す
                a2_sys, a1_sys, a0_sys = solver.system_coeffs(T1_sys, zeta_sys, wn_sys)                     # 3次遅れ系の係数へ展開
                z_sys_sim, _, _ = solver.simulate_forced(solver.t_eval, a2_sys, a1_sys, a0_sys, b0_sys, u_ident, 0.0)   # 同定したシステムモデルの応答を取り出す（z形式：初期値0）
                y_sys_sim = z_sys_sim + Pi - Pf                                                             # 表示・残差評価用に偏差系（Pf基準）へ戻す
                a2_tgt, a1_tgt, a0_tgt = solver.system_coeffs(tgt_params[0], 1.0, tgt_params[1])            # 目標モデル（zeta=1固定）の3次遅れ系の係数へ展開
                y_tgt_sim, _, _, _ = solver.simulate_unforced(solver.t_eval, a2_tgt, a1_tgt, a0_tgt, y0)    # 同定した目標モデルの自由応答（偏差系 y0→0）
                id_res_sys = float(np.sum((y_sys_sim - y_shifted) ** 2))                                    # システムモデル同定の残差
                id_res_tgt = float(np.sum((y_tgt_sim - y_shifted) ** 2))                                    # 目標モデル同定の残差

                # 4. 同定結果を保存（この1回の実測データから得られた「真のモデル」。以降ロボットは動かさない）
                models.append({
                    'dof': dof_idx + 1,                                                                     # DOF番号（1始まり）
                    'Pi': float(Pi), 'Pf': float(Pf), 'y0': float(y0),                                      # 初期位置・目標位置・初期偏差
                    'tgt_params': np.asarray(tgt_params, dtype=float),                                      # 目標モデル [T1, wn]
                    'sys_params': np.asarray(sys_params, dtype=float),                                      # 真のモデル [T1, zeta, wn, b0]
                    'ff_params': [float(v) for v in self.current_ff_matrix[dof_idx]],                        # 実測時に印加した励振FFの5次係数
                    'u_hold': float(u_hold),                                                                # 同定入力の基準値（初期姿勢の保持PWM）
                    'u_ff': u_ff,                                                                           # 実測時に印加した励振FF入力（calculate_el_ff の引数用）
                    'u_ident': u_ident,                                                                     # 同定に使った入力 u_pwm - u_hold
                    'u_adrc': u_adrc,                                                                       # 実測から復元したADRC入力（EL最適化中は固定）
                    'y_data': raw_y,                                                                        # 実測POT値
                    'y_sys_sim': y_sys_sim + Pf,                                                            # 同定したシステムモデルの応答（POT値）
                    'y_tgt_sim': y_tgt_sim + Pf,                                                            # 同定した目標モデルの応答（POT値）
                    'id_res_sys': id_res_sys, 'id_res_tgt': id_res_tgt,                                     # 同定残差
                })

                self.get_logger().info(                                                                     # 同定結果のログ出力
                    f"  DOF {dof_idx + 1:02d}: 目標モデル T1={tgt_params[0]:.4f}, wn={tgt_params[1]:.4f} (残差 {id_res_tgt:.1f}) | "
                    f"真のモデル T1={T1_sys:.4f}, zeta={zeta_sys:.4f}, wn={wn_sys:.4f}, b0={b0_sys:.4f} (残差 {id_res_sys:.1f})"
                )

            # 同定結果をJSONへ保存する（再探索時はこのJSONを読み込めばロボットを動かす必要がない）
            save_identified_models(self.model_path, self.T, self.dt, models, self.initial_pot, self.target_pot)          # 同定結果を保存
            self.get_logger().info(f"同定結果（真のモデル）を保存: {self.model_path}")                                    # ログ出力

            model = models[TUNE_TARGET_DOF - 1]                                                                          # 対象DOF（DOF4）の同定済みモデルを取り出す
            if self.SAVE_RESULT_PLOT:                                                                                    # 対象DOFの同定結果をグラフで確認できるようにする
                self.save_identification_plot(model, self.ident_plot_path)                                               # 実測・システムモデル・目標モデルの比較グラフを保存

            # ---------------------------------------------------------------------
            # ここから先はロボットを一切動かさず、シミュレーションのみで
            # COST_Q, COST_R, COST_R_DU を探索する
            # ---------------------------------------------------------------------
            tuner = WeightTuner(self.T, self.dt, model, n_trials=self.n_trials, logger=self.get_logger())                 # 重み探索クラスを生成
            best = tuner.run()                                                                                           # 探索を実行して最良の重みを取得

            if best is not None:                                                                                         # 探索が成功した場合は結果を保存
                tuner.save_best_to_csv(self.csv_path)                                                                    # 最良の重みをCSVへ追記
                tuner.save_trials_csv(self.trials_csv_path)                                                              # 全トライアル結果をCSVへ保存
                if self.SAVE_RESULT_PLOT:                                                                                # グラフ出力判定
                    tuner.save_result_plot(self.plot_path)                                                               # 最良結果のグラフを保存

            self.state = "FINISHED"                                                                                      # 終了状態に変更（ロボットはホームポジションへ戻す）
            self.state_start_time = self.get_clock().now()                                                               # 現在時刻を取得

        except Exception as exc:                                                                                                                    # 何かエラーが出たときの処理
            self.get_logger().error(f"最適化パイプライン異常: {exc}\n{traceback.format_exc()}")
            self.state = "FINISHED"
            self.state_start_time = self.get_clock().now()

    # 対象DOFの同定結果（実測 vs 同定モデル）をグラフへ保存する関数
    def save_identification_plot(self, model, path):                                                                                            # 引数(対象DOFの同定結果, 保存先パス)
        """同定した「真のモデル」が実測をどれだけ再現できているかを目で確認するためのグラフ。

        探索の前提はここで同定したモデルが実機と合っていることなので、この波形が実測と
        大きくずれている場合は、探索して得た重みも意味を持たない。
        """
        try:
            t = np.arange(0, SIM_TIME, self.dt)                                                                          # 時間軸
            fig, axes = plt.subplots(2, 1, figsize=(9, 8))                                                               # 応答用と入力用の2段グラフ
            axes[0].plot(t, model['y_data'], label='Measured (POT)')                                                     # 実測POT値
            axes[0].plot(t, model['y_tgt_sim'], '--', label='Target model (zeta=1)')                                     # 同定した目標モデル
            axes[0].plot(t, model['y_sys_sim'], ':', label='System model (= true model)')                                # 同定したシステムモデル
            axes[0].set_title(f"DOF {model['dof']} identification "
                              f"(res sys = {model['id_res_sys']:.1f}, res tgt = {model['id_res_tgt']:.1f})")
            axes[0].set_xlabel('Time [s]')
            axes[0].set_ylabel('POT value')
            axes[0].legend()
            axes[0].grid(True)

            axes[1].plot(t, model['u_ident'], label='Identification input (u_pwm - u_hold)')                              # 同定に使った入力
            axes[1].plot(t, model['u_ff'], '--', label='Excitation FF (sent to robot)')                                   # 送信した励振FF
            axes[1].set_title(f"Identification input (u_hold = {model['u_hold']:.2f})")
            axes[1].set_xlabel('Time [s]')
            axes[1].set_ylabel('PWM')
            axes[1].legend()
            axes[1].grid(True)

            fig.tight_layout()                                                                                           # レイアウト調整
            fig.savefig(path)                                                                                            # PNGへ保存
            plt.close(fig)                                                                                               # グラフを閉じる
            self.get_logger().info(f"対象DOFの同定結果グラフを保存: {path}")
        except Exception as exc:                                                                                         # グラフ保存の失敗で探索を止めない
            self.get_logger().warn(f"同定結果グラフの保存に失敗: {exc}")


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


# 重みの探索回数を決める関数（コマンドライン引数 --trials N が最優先、無ければキーボード入力）
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
    print(f"【探索回数を入力してください（未入力なら既定値 {TUNE_N_TRIALS}）】")                          # 入力を促す
    text = input("> ").strip()                                                                      # 探索回数の入力
    return max(1, int(text)) if text else TUNE_N_TRIALS                                             # 探索回数を返す


# 保存済みの同定結果を読み込み、ロボットを動かさず重みを探索する関数
def run_tuning_from_saved_models(n_trials):                                                         # 引数(重みの探索回数)
    root = tk.Tk()                                                                                  # Tkinterを起動
    root.withdraw()                                                                                 # 親ウィンドウは表示しない
    root.attributes("-topmost", True)                                                               # ダイアログを最前面に表示

    json_path = filedialog.askopenfilename(                                                         # ファイル選択ダイアログを表示
        title="同定結果JSON（*_identified_models.json）を選択してください",
        filetypes=[("JSON Files", "*.json")],
    )
    if not json_path:
        print("ファイル未選択のため終了します。")
        sys.exit(1)

    T, dt, models = load_identified_models(json_path)                                               # 同定結果（真のモデル）を読み込む
    print(f"【同定結果を読み込みました】{json_path}  (T={T}, dt={dt}, DOF数={len(models)})")

    tuner = WeightTuner(T, dt, models[TUNE_TARGET_DOF - 1], n_trials=n_trials)                       # 重み探索クラスを生成（対象DOFの同定済みモデルを使用）
    best = tuner.run()                                                                              # 探索を実行

    if best is not None:                                                                            # 探索が成功した場合は結果を保存
        base_path = os.path.splitext(json_path)[0]                                                  # JSONの拡張子を除いたパス
        tuner.save_best_to_csv(base_path + "_weight_best.csv")                                      # 最良の重みをCSVへ追記
        tuner.save_trials_csv(base_path + "_weight_trials.csv")                                     # 全トライアル結果をCSVへ保存
        if TUNE_SAVE_PLOT:                                                                          # グラフ出力判定
            tuner.save_result_plot(base_path + "_weight_best.png")                                  # 最良結果のグラフを保存


def main(args=None):
    print("=================================================================================")
    print("  評価関数の重み COST_Q, COST_R, COST_R_DU の自動チューニング (Optuna)")
    print("  1: ロボットを1回だけ動かして同定し、そのモデル（真のモデル）で重みを探索する")
    print("  2: 保存済みの同定結果(JSON)を読み込んで重みのみ探索する（ロボットは動かさない）")
    print("=================================================================================")
    run_mode = input("モードを選択してください (1 または 2): ").strip()                                   # 実行モードの入力
    while run_mode not in ['1', '2']:
        run_mode = input("無効な入力です。1 または 2 を入力してください: ").strip()

    n_trials = resolve_n_trials()                                                                   # 重みの探索回数を決定
    print(f"【探索回数】 {n_trials}")

    # 保存済み同定結果を使う場合（ロボット・ROS2は不要）
    if run_mode == '2':
        run_tuning_from_saved_models(n_trials)                                                      # 探索のみ実行
        return

    csv_path = resolve_csv_file()
    print(f"【保存先CSVパス】 {csv_path}")

    print("【FF制御時間 T を入力してください】")
    T = float(input("> "))
    print("【目標値のモードを選択してください (1: プリセット, 2: ランダム)】")
    mode = input("> ").strip()
    while mode not in ['1', '2']:
        mode = input("無効な入力です。1 または 2 を入力してください: ").strip()

    rclpy.init(args=args)
    node = OptimalControlSequencer(csv_path, T, n_trials, mode)
    # 購読（10トピック）とシーケンサのタイマーを別スレッドで動かす。
    #   単一スレッドだと、FF送信・24自由度分のログ出力・sleep のあいだ受信が止まり、
    #   その間のデータが受信キューから溢れて消える。消えた区間は補間で直線に化けるため、
    #   実機が振動していてもグラフが水平になってしまう。
    executor = MultiThreadedExecutor(num_threads=12)                                            # 購読10 + タイマー1 + 余裕1
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.shutdown_event.is_set():                                  # 終了要求が来るまで回し続ける
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
