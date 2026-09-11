# 2026/08/07　19:30までのEL方程式プログラム。計算されるu_optに対して、直接FFを近似している。view_optimizationはtest_code5に対応。
#!/usr/bin/env python3
import os                                                       # OSライブラリ
import sys                                                      # Pythonを扱うライブラリ
from collections import deque                                   # 固定長リング（保持PWMの直近サンプル保存用）
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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # コールバックの並列実行グループ
from rclpy.executors import MultiThreadedExecutor               # 購読とタイマーを別スレッドで回す実行器
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
# 最適制御の対象自由度と内側ループの収束判定（チューニング要素）
#   OPT_DOF_IDS : 最適制御を行う自由度番号のリスト（1始まり, 1～24）。ここに書いた自由度だけが
#                 ・初期FF入力（初回同定用の励振）を受け取る
#                 ・目標モデル／システムモデルの同定と、オイラーラグランジュ最適制御の対象になる
#                 ・内側ループの評価値 J（の総和）に算入される
#                 ここに書かなかった自由度は「最適化対象外」として扱い、FF入力を常に0
#                 （a=b=c=d=e=0）にしたまま実測POT値だけを記録する。同定・最適入力計算・
#                 5次関数フィットはいっさい行わないので、その分の計算時間もかからない。
#                 ビューア（test_code5.py）も、対象外の自由度は表示枠を保ったまま
#                 実測POT値のみを描く。
#   THRESHOLD_J : 内側ループの収束判定閾値。OPT_DOF_IDS に含まれる自由度の J の総和が
#                 この値より小さくなれば、その外側ループを終了する。
#   ※ 対象自由度を変えると J の総和のスケールも変わるため、THRESHOLD_J も併せて見直すこと。
# ==============================================================================
OPT_DOF_IDS = [4]    # 最適制御を行う自由度番号（1始まり）
THRESHOLD_J = 250000.0             # 内側ループの収束判定閾値（対象自由度のJの総和がこの値以下になれば収束）

N_DOF_ALL = 24                                                                      # 最適制御パイプラインが扱う自由度の総数
OPT_DOF_IDX = sorted({int(v) - 1 for v in OPT_DOF_IDS})                             # 0始まりの添字へ変換（重複は除く）
if not OPT_DOF_IDX or OPT_DOF_IDX[0] < 0 or OPT_DOF_IDX[-1] >= N_DOF_ALL:           # 書き間違いに気付かないまま実験するのを防ぐ
    raise ValueError(f"OPT_DOF_IDS は 1～{N_DOF_ALL} の自由度番号を1つ以上指定してください: {OPT_DOF_IDS!r}")
OPT_DOF_MASK = np.zeros(N_DOF_ALL, dtype=bool)                                      # 自由度ごとの最適化対象フラグ
OPT_DOF_MASK[OPT_DOF_IDX] = True                                                    # 指定された自由度だけTrueにする

# ==============================================================================
# 評価関数の重み行列（チューニング要素）
#   コスト関数 J = Σ (eᵀQe + Ru²) + R_du Σ(Δu)² の重み
#   （e は目標軌道との誤差状態, u は制御入力, Δu(k) = u(k) - u(k-1) は入力の変化量）
# ==============================================================================
# COST_Q = np.diag([0.900272, 8.38989, 0.122944])     # 状態誤差の重み（大きいほど誤差を抑える）
# COST_R = np.array([[34.4762]])             # 制御入力の重み（大きいほど入力を抑える）

COST_Q = np.diag([8858.79, 0.191027, 0.0711133])     # 状態誤差の重み（大きいほど誤差を抑える）
COST_R = np.array([[10.7164]])             # 制御入力の重み（大きいほど入力を抑える）

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
COST_R_DU = 2.4e6           # 入力の変化量Δuの重み（大きいほど入力が滑らかになり、端点が0へ寄る。0で従来どおり無効）15.6961

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
MAX_INNER_ITER = 60         # 内側ループの最大反復回数
                            # ※ 内側ループの収束判定閾値 THRESHOLD_J はファイル上部（OPT_DOF_IDS と同じブロック）に移動した

# ==============================================================================
# システムモデル同定に使う入力の選択（チューニング要素）
#   'actual_pwm' : 実際にロボットへ印加されたPWM（/board{i}_tk_PWM_float/pub）から、
#                  目標値切替直前の保持PWM u_hold を引いた値を同定入力にする。
#                  FFだけでなくPIDが供給した分も含んだ「本当に加わった入力」で同定できる。
#                  座標系は z = P - Pi（初期位置を0とする系, z(0)=ż(0)=z̈(0)=0）。
#                  入力の基準を Pi の保持PWMに置くことで、姿勢保持に必要な定常入力が
#                  式から消え、b0 が0へ潰れない。
#   'ff_input'   : 実PWMは使わず、その測定で与えた5次関数FF入力そのものを同定入力にする。
#                  座標系は y = P - Pf（最終位置＝目標位置を0とする偏差系, y(0)=Pi-Pf）。
#                  FF入力は0から始まり区間外も0なので、入力の基準値は0でよい。
#                  PIDが供給する分はモデル化されず外乱として残るが、同定入力と自分が
#                  与えた入力が完全に一致するため、入力規約のずれが起きない。
# ==============================================================================
IDENTIFICATION_INPUT_MODE = 'ff_input'    # 同定入力の選択（'actual_pwm' または 'ff_input'）

if IDENTIFICATION_INPUT_MODE not in ('actual_pwm', 'ff_input'):                                  # 綴り違いに気付かないまま実験するのを防ぐ
    raise ValueError(f"IDENTIFICATION_INPUT_MODE は 'actual_pwm' か 'ff_input' です: {IDENTIFICATION_INPUT_MODE!r}")

# ==============================================================================
# 測定失敗時の自動再測定の設定（チューニング要素）
#   受信が途切れた区間は resample_by_time() が直線で埋めてしまうため、その測定データを
#   同定に使うと「実機は振動しているのに水平な直線」をモデルに写し取ってしまう。
#   そこで受信途切れを検出した測定は失敗とみなしてデータを捨て、
#     初期姿勢へ戻す → INIT_WAIT_TIME 待機 → 直前と同じFF入力で測定し直す
#   を MAX_MEASURE_RETRY 回まで繰り返す。失敗した測定では同定・FF更新・評価関数Jの計算・
#   ビューア用スナップショットのいずれも行わないので、内側ループ番号も進まない。
# ==============================================================================
RETRY_ON_GAP = True         # 受信途切れを測定失敗とみなして自動再測定するか（Falseなら従来どおり警告のみで続行）
RETRY_GAP_RATIO = 0.0       # 欠測の割合がこの値を超えるDOFが1つでもあれば測定失敗とみなす（0.0なら1点でも途切れたら失敗。再測定が多すぎるなら0.01程度まで上げる）
MAX_MEASURE_RETRY = 5       # 同じFF入力での連続再測定の上限回数（超えたら警告してそのデータのまま続行する）
RETRY_ON_PWM_GAP = True     # PWMの受信途切れも測定失敗とみなすか（IDENTIFICATION_INPUT_MODE='actual_pwm' のときだけ判定する）

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
INIT_FF_PEAK = 5.0         # 初期FFの極値の大きさ [PWM]（f(t1)=+50≥40, f(t2)=-50≤-40 を満たす）

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
    def calculate_el_ff(self, target_params, sys_params, u_ff, y0):                                          # 引数(目標モデルのパラメータ[T1, wn], システムモデルのパラメータ[T1, zeta, wn, b0], FF制御入力, 初期偏差)
        """
        離散時間オイラー・ラグランジュ（随伴／勾配）法で最適制御入力を計算し、
        5次多項式 FF = a*t^5 + ... + e*t にフィットする。

        目的：システムモデルの軌道を、目標モデルが描く目標軌道 x_tgt へ一致させる。
          座標系       : y = P - Pf（目標位置を原点とする系）。x_sys(0)=x_tgt(0)=y0 から 0 へ収束する
          入力の基準   : ũ = u_total - u_hold(Pf)。「目標位置 Pf を保持するPWM」からの偏差。
                         したがって k ≥ N_ff の ũ(k)=0 は「PWMが Pf 保持値にある」＝腕が目標位置に
                         留まる状態を意味する（保持に必要な定常分はフィードバック側が供給する前提）。
                         最適化されるのは移動の過渡分だけになるので、加速側と制動側の両符号が出る。
          入力ホライズン : ũ(k) は [0,T]（k=0..N_ff-1）のみ最適化し、それ以降は0
          評価ホライズン : コスト J はシミュレーション全体（5秒, k=0..M-1）で評価する

        ● 同定との関係（同定の式・手順は一切変更しない）
            システムモデル同定は z = P - Pi, w = u_total - u_hold(Pi) の規約で行う。
                z''' + a2 z'' + a1 z' + a0 z = b0 w
            ここへ y = z - step （step = Pf - Pi = -y0）を代入すると
                y''' + a2 y'' + a1 y' + a0 y = b0 w - a0·step = b0 ( w - a0·step/b0 )
                                                                  └──────┬──────┘
                                                                       ũ = w - u_ss
            となり、係数 (a2,a1,a0,b0) は同じまま、入力の基準だけが u_ss = a0·step/b0 ずれた形になる。
            u_ss は「Pi 保持PWMから見た Pf 保持PWM」なので、ũ はちょうど Pf 保持PWM からの偏差。
            つまり同定結果（z形式）をそのまま使え、同定側を変える必要はない。
            なお u_opt の意味も「Pf 保持PWMからの偏差」になるため、実機へ送るFFは
            フィードバックが供給する定常分に上乗せする過渡分として解釈すること。

        ● 離散状態方程式（可制御正準形・ZOH厳密離散化）
            システム : x_sys(k+1) = A_sys x_sys(k) + B_sys ũ(k),  x_sys(0) = [y0, 0, 0]ᵀ
            目標     : x_tgt(k) は A_tgt の自由応答（y0 → 0）そのもの

        ● 誤差状態 x(k)=x_sys(k)-x_tgt(k) を状態変数とした状態方程式（e と表記, e(0)=0）
            e(k+1) = x_sys(k+1) - x_tgt(k+1)
                   = A_sys( e(k)+x_tgt(k) ) + B_sys ũ(k) - x_tgt(k+1)
                   = A_sys e(k) + B_sys ũ(k) + d(k),   d(k) = A_sys x_tgt(k) - x_tgt(k+1)  （既知の入力項）
            この座標系では目標軌道が A_tgt の自由応答そのものなので、d(k) は
            d(k) = (A_sys - A_tgt) x_tgt(k) に一致する。

        ● コスト関数（終端コストなし）／ハミルトニアン
            J = Σ_{k=0}^{M-1} ( e(k)ᵀ Q e(k) )
              + Σ_{k=0}^{N_ff-1} R u(k)²
              + Σ_{k=0}^{N_ff}  R_du ( u(k) - u(k-1) )²,     u(-1) = 0,  u(N_ff) = 0
            第3項が入力レート罰則。境界を u(-1)=u(N_ff)=0 と置くことで、u(0)→0・u(T)→0・
            途中の滑らかさの3つを1項で同時に誘導する。R_du=0 とすれば従来の定式化に戻る。

            段コスト L(k) = e(k)ᵀ Q e(k) + R u(k)² + R_du ( u(k) - u(k-1) )²
            H(k) = L(k) + λ(k+1)ᵀ [ A_sys e(k) + B_sys u(k) + d(k) ]

        ● 随伴方程式（レート罰則は u のみの関数で ∂/∂e = 0 なので、随伴方程式は変わらない）
            λ(N) = λ(M) = 0
            λ(k) = ∂H/∂e(k) = 2 Q e(k) + A_sysᵀ λ(k+1)

        ● 勾配（入力ホライズン k=0..N_ff-1）
            レート罰則は u(k) と u(k-1) を結合させるため、u(k) は段 k と段 k+1 の両方の
            段コストに現れる。したがって勾配は H(k) だけでなく L(k+1) からも寄与を受ける。
                ∂J/∂u(k) = ∂H(k)/∂u(k) + ∂L(k+1)/∂u(k)
                         = [ 2 R u(k) + 2 R_du ( u(k) - u(k-1) ) + B_sysᵀ λ(k+1) ]
                           + [ -2 R_du ( u(k+1) - u(k) ) ]
                         = 2 R u(k) + B_sysᵀ λ(k+1) + 2 R_du ( 2u(k) - u(k-1) - u(k+1) )
            末尾の括弧は Dirichlet境界条件 u(-1)=u(N_ff)=0 を課した2階差分（3重対角行列
            L=tridiag(-1,2,-1)）で、(L u)(k) と書ける。k=0 では u(-1)=0、k=N_ff-1 では
            u(N_ff)=0 として評価するため、端点だけ自動的に「0へ引き戻す」向きの力が働く。

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

        # 目標軌道 x_tgt(k) を y 形式（目標位置を原点とする系）で生成する
        #   A_tgt の同定は従来どおり誤差系（y = P - Pf, y0 → 0 へ収束する自由応答）で行い、
        #   最適制御でもその誤差系のまま扱う（定数オフセットを掛けない）。
        #   こうすると x_sys(0)=x_tgt(0)=[y0,0,0]ᵀ で e(0)=0 は変わらないまま、入力の基準だけが
        #   「Pi を保持するPWM」から「Pf を保持するPWM」へ移る。k ≥ N_ff で ũ(k)=0 とすることが
        #   「腕が目標位置に留まる」を意味するようになり、最適化は移動の過渡分だけを担当する。
        #   （z 形式のままだと ũ(k)=0 は「PWMが Pi 保持値へ戻る」＝腕が初期位置へ戻る意味になり、
        #     FFだけで変位を保持し続けろという無理な要求になって入力が片側に張り付いていた）
        #   d(M-1) の計算に x_tgt(M) が要るので、M+1 点分を生成する。
        x_tgt = np.zeros((M + 1, n_x))                                                                  # 目標状態列 x_tgt(0..M) (M+1, 3)
        x_tgt[0] = np.array([y0, 0.0, 0.0])                                                             # 誤差系での初期状態（自由応答の初期値）
        for k in range(M):                                                                              # 目標モデルの自由応答を逐次計算
            x_tgt[k + 1] = A_tgt @ x_tgt[k]                                                             # x_tgt(k+1) = A_tgt x_tgt(k)
        y_tgt = x_tgt[:M, 0].copy()                                                                     # 目標出力（誤差系 y0→0）。呼び出し側の規約は従来どおり維持する

        # 誤差状態方程式の既知入力項 d(k) = A_sys x_tgt(k) - x_tgt(k+1)
        #   e(k+1) = x_sys(k+1) - x_tgt(k+1) を厳密に満たす一般形。この座標系では目標軌道が
        #   A_tgt の自由応答（x_tgt(k+1) = A_tgt x_tgt(k)）そのものなので、この式は
        #   d(k) = (A_sys - A_tgt) x_tgt(k) と一致する。
        D = x_tgt[:M] @ A_sys.T - x_tgt[1:M + 1]                                                        # D[k] = A_sys x_tgt(k) - x_tgt(k+1) → 形状 (M, 3)

        Q = self.Q                                                                                      # 状態誤差の重み行列 (3x3)
        R = float(self.R[0, 0])                                                                         # 入力の重みスカラー
        R_du = float(self.R_du)                                                                         # 入力レート（Δu）の重みスカラー

        # ---- 手順2: 誤差状態 e を順方向に計算する関数（e(0)=x_sys(0)-x_tgt(0)=0） ----
        #   e(k+1) = A_sys e(k) + B_sys u(k) + d(k)。入力は [0,T](k<N_ff) のみ、それ以降は0。
        def forward(u_seq):
            e = np.zeros((M + 1, n_x))                                                                  # 誤差状態列 e(0..M)（e(0)=0）
            for k in range(M):                                                                          # k=0..M-1 を順方向に更新
                uk = u_seq[k] if k < N_ff else 0.0                                                      # 入力は [0,T] のみ、それ以降は0
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
            return (2.0 * R * u_seq                                                                     # 入力そのものの罰則 2 R u(k)
                    + lam[1:N_ff + 1] @ B_sys                                                           # 随伴からの寄与 B_sysᵀ λ(k+1)
                    + 2.0 * R_du * rate_grad(u_seq))                                                    # レート罰則 2 R_du (L u)(k)

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
            [500, 200, 500, 700, 170, 300, 160, 410, 200, 500, 350, 220, 300, 250, 400, 350, 420, 400, 325, 370, 280, 420, 360, 390, 420, 390],
        ]

        self.current_ff_matrix = self.build_initial_ff_matrix(self.T)                                                                            # 24自由度分のFF係数（対象自由度のみ初回励振用の非ゼロ初期値、対象外は0のまま）
        self.best_ff_matrix = None                                                                                                              # 今までの最良FF係数
        self.best_extrema = None                                                                                                                # 今までの最良FFの極値
        self.min_J_sum = float('inf')                                                                                                           # 評価関数Jの初期化
        self.best_debug_data = None                                                                                                             # デバック情報
        self.prev_u_pred_full = [None] * 24                                                                                                      # 前回ループで計算した5次関数FF入力（今回の実測データを生成した入力）
        self.prev_u_opt = [None] * 24                                                                                                            # 前回ループで計算した最適制御入力（今回の実測データを生成した入力）
        self.prev_tgt_params = [None] * 24                                                                                                       # 前回の目標モデル同定結果[T1, wn]（次回同定の初期値に使う）
        self.prev_sys_params = [None] * 24                                                                                                       # 前回のシステムモデル同定結果[T1, zeta, wn, b0]（次回同定の初期値に使う）
        self.prev_fit_restart = [0] * 24                                                                                                         # 前回ループのCMA-ES再探索回数（ビューア用）
        self.param_history = []                                                                                                                  # 内側ループごとのパラメータ・評価値の履歴（ビューア用、外側ループごとにリセット）

        # 測定失敗（受信途切れ）による再測定の管理
        #   FFの更新は測定が成功したときだけ行うので、再測定では self.current_ff_matrix が
        #   前回のまま残り、自動的に「直前に測定していたものと同じFF入力」で測り直せる。
        self.measure_retry = 0                                                                                                                   # 同じFF入力での連続再測定回数（測定に成功したら0へ戻す）
        self.last_measure_retry = 0                                                                                                              # 直近で採用した測定が何回の再測定を要したか（ビューア用）
        self.retry_total = 0                                                                                                                     # この実行での再測定の総回数（ビューア用）

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

    @staticmethod                                                                                                                               # Pythonデコレータ（静的メソッド：selfを使わない）
    def build_initial_ff_matrix(T):                                                                                                             # 引数(FF制御入力時間)
        """24自由度分のFF係数の初期値を作る。

        OPT_DOF_IDS に指定した自由度だけに、初回同定の励振用の非ゼロ初期FF
        （f(0)=f(T)=0, 極値ちょうど2個, |極値|=INIT_FF_PEAK）を与える。
        指定していない自由度は最適化対象外なので、FF係数を全て0にしてロボットへも
        0のFFを送り続ける（＝FFを与えない）。
        """
        return [list(MathematicalSolver.initial_ff_params(T)) if OPT_DOF_MASK[i]                                                                # 対象自由度は初回励振用の非ゼロ初期FF
                else [0.0, 0.0, 0.0, 0.0, 0.0]                                                                                                  # 対象外の自由度はFFを与えない
                for i in range(N_DOF_ALL)]

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

    # 今回の測定を同定に使ってよいか判定する関数
    def check_measurement(self, pot_data, pot_gaps, pwm_data, pwm_gaps):                                                                        # 引数(24自由度分のPOT, その欠測マスク, 24自由度分のPWM, その欠測マスク)
        """受信が途切れていれば、その測定を失敗とみなして理由の文字列を返す（問題なければ None）。

        補間は欠測を直線で埋めてしまうため、途切れたまま同定すると「実機は振動しているのに
        水平な直線」をモデルへ写し取ってしまう。そのデータを使うより測り直すほうが確実なので、
        ここで失敗と判定してデータを捨て、同じFF入力で再測定させる。
        PWMは同定入力に使うときだけ判定する（'ff_input' モードでは同定に使わないため）。
        判定するのは OPT_DOF_IDS の自由度だけ。最適化対象外の自由度は同定にも評価関数Jにも
        使わないので、そこが途切れたことを理由に測定をやり直すと時間を無駄にするため。
        """
        checks = [("POT", pot_data, pot_gaps)]                                                                                                  # POTは常に判定する
        if IDENTIFICATION_INPUT_MODE == 'actual_pwm' and RETRY_ON_PWM_GAP:                                                                      # 実PWMを同定入力に使う場合
            checks.append(("PWM", pwm_data, pwm_gaps))                                                                                          # PWMの欠測も測定失敗の対象にする

        worst_ratio, worst_dof, worst_label = -1.0, -1, ''                                                                                      # 最も欠測が多かったDOFの記録
        for label, data, gaps in checks:                                                                                                        # POT / PWM の順に判定
            for dof_idx in OPT_DOF_IDX:                                                                                                         # 最適化対象の自由度だけを判定する
                if data[dof_idx] is None:                                                                                                       # 1点も受信できなかった場合
                    return f"{label} DOF {dof_idx + 1:02d}: 1点も受信できませんでした"                                                             # 即座に失敗とする
                ratio = float(np.asarray(gaps[dof_idx], dtype=bool).mean())                                                                     # そのDOFの欠測の割合
                if ratio > worst_ratio:                                                                                                         # これまでで最も欠測が多い場合
                    worst_ratio, worst_dof, worst_label = ratio, dof_idx, label                                                                 # 記録を更新
        if worst_ratio > RETRY_GAP_RATIO:                                                                                                       # 許容割合を超える欠測があった場合
            return (f"{worst_label} DOF {worst_dof + 1:02d}: 受信が途切れました"
                    f"（欠測 {worst_ratio * 100:.1f}% > 許容 {RETRY_GAP_RATIO * 100:.1f}%）")                                                     # 失敗の理由を返す
        return None                                                                                                                             # 問題なし（この測定は同定に使える）

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
                    opt_mark = "" if OPT_DOF_MASK[dof_idx] else "  [最適化対象外: FF=0]"                       # 最適化対象外の自由度が一目で分かるようにする
                    self.get_logger().info(                                                                     # ログ出力
                        f"  B{b_id}-D{_ + 1} (DOF {dof_idx + 1:02d}): "
                        f"a={a:.6e}, b={b:.6e}, c={c:.6e}, d={d:.6e}, e={e:.6e} | "
                        f"t1={t1:.3f}, y1={y1:.2f}, t2={t2:.3f}, y2={y2:.2f}{opt_mark}"
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
            retry_msg = (f" [再測定 {self.measure_retry}/{MAX_MEASURE_RETRY}: 同じFF入力で測り直します]"
                         if self.measure_retry > 0 else "")                                                             # 再測定中はその旨を添える
            self.get_logger().info(                                                                                     # ログ出力
                f"=== 最適制御 外側ループ {self.current_outer + 1} / {self.max_outer_iter} "
                f"(内側ループ {self.current_inner + 1}/{self.max_inner_iter}){retry_msg} ==="
            )
            self.publish_target_positions(self.initial_pot)                                                             # 初期姿勢を送信（再測定のときもここでロボットを初期位置へ戻す）
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
            pwm_24_dof, pwm_gap_24_dof = build_24_dof(self.buffer_pwm, "PWM")                                           # 実制御入力（PWM）を24自由度分へ変換

            # 測定の成否判定（受信が途切れていたらこの測定データは捨てて、同じFF入力で測り直す）
            #   ここで return すれば self.current_ff_matrix・prev_*・param_history・内側ループ番号は
            #   いずれも更新されないため、次の測定は「直前に測定していたものと同じFF入力」になる。
            fail_reason = self.check_measurement(data_24_dof, gap_24_dof, pwm_24_dof, pwm_gap_24_dof) if RETRY_ON_GAP else None
            if fail_reason is not None:                                                                                 # 測定が失敗していた場合
                if self.measure_retry < MAX_MEASURE_RETRY:                                                              # まだ再測定の上限に達していない場合
                    self.measure_retry += 1                                                                             # 連続再測定回数を更新
                    self.retry_total += 1                                                                               # 通算の再測定回数を更新
                    self.get_logger().warn(                                                                             # 警告ログを出力
                        f"測定失敗（{fail_reason}）。このデータは同定に使いません。"
                        f"初期姿勢へ戻して {self.initial_stabilize_time:.1f}s 待機し、同じFF入力で再測定します "
                        f"(再測定 {self.measure_retry}/{MAX_MEASURE_RETRY})"
                    )
                    self.state = "INIT_ROBOT"                                                                           # 初期姿勢へ戻す状態に戻す（待機→同じFFで再測定へ進む）
                    self.state_start_time = self.get_clock().now()                                                      # 現在時刻を取得
                    return                                                                                              # 同定・FF更新・J計算・スナップショットは一切行わない
                self.get_logger().error(                                                                                # エラーログを出力
                    f"測定失敗（{fail_reason}）が再測定の上限 {MAX_MEASURE_RETRY} 回まで続きました。"
                    f"やむを得ずこのデータで同定を続行します（同定結果は信用できません）"
                )
            self.last_measure_retry = self.measure_retry                                                                # この測定に要した再測定回数を控える（ビューア用）
            self.measure_retry = 0                                                                                      # 測定を採用したので連続再測定回数を戻す

            J_array = []                                                                                                # 24自由度それぞれの評価関数Jの空リスト
            debug_info = []                                                                                             # デバッグExcelへ保存する情報の空リスト
            extrema_list = []                                                                                           # 保存する極大値と極小値の空リスト
            used_extrema_list = []                                                                                      # 極値のリスト
            snap_dof = []                                                                                               # ビューア用データ（自由度ごと）の空リスト

            # 今回ロボットへ送信したFF（更新前）を保存
            ff_used_this_iteration = [row[:] for row in self.current_ff_matrix]                                         # パラメータの保存
            used_extrema_list =  [solver.calc_extrema_from_ff(ff) for ff in ff_used_this_iteration]                     # 極値の保存

            nan_series = np.full(N_samples, np.nan)                                                                     # 最適化対象外の自由度に入れる「値なし」の時系列（グラフには描かれない）

            for dof_idx in range(24):                                                                                   # 24自由度分のループ
                is_opt = bool(OPT_DOF_MASK[dof_idx])                                                                    # この自由度が最適制御の対象かどうか
                if data_24_dof[dof_idx] is None:                                                                        # POT値を1点も受信できなかった場合
                    if is_opt:                                                                                          # 最適化対象なら同定できないため異常として扱う
                        raise RuntimeError(f"DOF {dof_idx + 1:02d}: POT値を1点も受信できませんでした")
                    self.get_logger().warn(                                                                             # 最適化対象外なら表示できないだけなので警告にとどめる
                        f"DOF {dof_idx + 1:02d}: POT値を1点も受信できませんでした（最適化対象外のため続行します）"
                    )
                    raw_y = nan_series.copy()                                                                           # 実測なしとして扱う
                    y_gap = np.ones(N_samples, dtype=bool)                                                              # 全区間が欠測
                else:
                    raw_y = data_24_dof[dof_idx]                                                                        # 5秒間分（N_samples点）に揃え済みの実測データ
                    y_gap = gap_24_dof[dof_idx]                                                                         # 実測が無く補間の直線になっている区間

                # Initial and Target values
                Pi = self.initial_pot[dof_map[dof_idx]]                                                     # 現在のDOFの初期位置を取得
                Pf = self.target_pot[dof_map[dof_idx]]                                                      # 現在のDOFの目標位置を取得
                y0 = Pi - Pf                                                                                # 初期偏差を計算

                # 最適化対象外の自由度は、実測POT値だけを記録して次の自由度へ進む。
                #   目標モデル同定・システムモデル同定・最適入力計算・5次関数フィットはいっさい行わず、
                #   評価関数Jにも算入しない（total_J は OPT_DOF_IDS の自由度の合計）。
                #   データ構造（24自由度分の並び）は保ったまま、計算していない項目をNaNで埋めるので、
                #   ビューア側は表示枠を維持したまま実測POT値だけを描ける。
                if not is_opt:
                    J_array.append(np.nan)                                                                  # Jは計算しない（合計には含めない）
                    debug_info.append({
                        't': solver.t_eval,             # 実測時間
                        'y_data': raw_y,                # 実測POT値
                        'y_sys': nan_series.copy(),     # システムモデル応答（計算しない）
                        'y_tgt': nan_series.copy(),     # 目標モデル応答（計算しない）
                        'J': float('nan'),
                        'u_pred_full': nan_series.copy(),   # 5次関数FF入力（計算しない）
                        'u_opt': nan_series.copy(),         # 最適制御入力（計算しない）
                    })
                    if ENABLE_SNAPSHOT:
                        snap_dof.append({
                            'y_data': raw_y,                                                # 実測POT値（ビューアが描くのはこれだけ）
                            'y_gap': y_gap,                                                 # 実測が無く補間の直線になっている区間
                            'y_sys': nan_series.copy(), 'y_tgt': nan_series.copy(),          # モデル応答は計算しない
                            'u_opt_used': nan_series.copy(),                                 # u_optは計算しない
                            'u_ff_applied': nan_series.copy(),                               # FFは0を送っている（表示しない）
                            'u_ident': nan_series.copy(),                                    # 同定入力も無い
                            'tgt_params_id': [np.nan] * 2, 'sys_params_id': [np.nan] * 4,     # 同定していない
                            'tgt_params_used': [np.nan] * 2, 'sys_params_used': [np.nan] * 4,  # 同定していない
                            'ff': [0.0] * 5,                                                 # 送信しているFF係数（全て0）
                            'extrema': [np.nan] * 4,                                         # 極値は無い
                            'J': float('nan'),                                               # Jは計算しない
                            'Pi': float(Pi), 'Pf': float(Pf),                                # 初期位置・目標位置
                            'pot_idx': dof_map[dof_idx],                                     # 26要素配列でのインデックス
                            'fit_res': float('nan'),                                         # 近似残差は無い
                            'fit_restart': -4,                                               # 最適化対象外を示す値
                            'id_res_sys': float('nan'),                                      # 同定残差は無い
                        })
                    continue                                                                                # 同定・最適制御は行わない

                # 欠測が多いと、水平な直線をそのまま同定してしまい振動しないモデルになる
                if y_gap.mean() > GAP_WARN_RATIO:
                    self.get_logger().warn(
                        f"DOF {dof_idx + 1:02d}: 実測データの {y_gap.mean() * 100:.0f}% が補間です。"
                        f"同定結果（特に zeta・wn）は信用できません"
                    )

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

                # 同定入力の選択（IDENTIFICATION_INPUT_MODE で切り替える）
                #   'actual_pwm' : 実際にそのDOFを動かした制御入力（PWM）から保持PWMを引いた値
                #   'ff_input'   : 実PWMを使わず、その測定で与えた5次関数FF入力そのもの
                u_pwm = pwm_24_dof[dof_idx]                                                                 # 受信時刻でシミュレーションの時間軸へ揃え済みの実測PWM（未受信ならNone）
                if IDENTIFICATION_INPUT_MODE == 'ff_input':                                                 # FF入力のみで同定する場合
                    u_hold = 0.0                                                                            # FF入力は0から始まるので基準値も0
                elif u_pwm is None:                                                                         # 実PWMで同定したいがPWMを1点も受信できなかった場合
                    self.get_logger().warn(                                                                 # 警告ログを出力
                        f"DOF {dof_idx + 1:02d}: PWMを受信できませんでした。FF入力でシステムモデルを同定します。"
                    )
                    u_hold = 0.0                                                                            # FF入力は0から始まるので基準値も0（u_pwm は None のままなので下でFF入力へ切り替わる）
                else:                                                                                       # PWMを受信できた場合
                    u_hold = self.hold_pwm_24[dof_idx] if self.hold_pwm_24 is not None else None            # 目標値切替直前の保持PWM（同定入力の基準値）
                    if u_hold is None:                                                                      # 保持PWMを取得できなかった場合
                        self.get_logger().warn(                                                             # 警告ログを出力
                            f"DOF {dof_idx + 1:02d}: 保持PWMを取得できませんでした。収集開始時のPWMで代用します。"
                        )
                        u_hold = float(u_pwm[0])                                                            # 収集開始時のPWMで代用する（精度は落ちる）

                # 同定の座標系は同定入力に合わせて変える（入力の零点と出力の原点をそろえるため）
                #   'actual_pwm' : z = P - Pi（初期位置を0とする系）で同定する。
                #       偏差系 y = P - Pf（y→0）だと、姿勢を保持するための定常入力が残るせいで
                #       「定常入力があるのに出力0」となり、b0 が0へ潰れて同定できない。
                #       z 形式では次式が厳密に成り立ち、姿勢依存の重力保持分 u_g が式から消える。
                #         z''' + a2z'' + a1z' + a0z = b0 (u - u_hold),  z(0)=ż(0)=z̈(0)=0
                #         u_hold = 初期姿勢 Pi を保持していたPWM（= u_g + a0*y0/b0）
                #   'ff_input'   : y = P - Pf（最終位置＝目標位置を0とする偏差系）で同定する。
                #       入力はFF（0から始まり区間外も0）なので基準値の推定が不要で、出力は
                #       目標位置へ収束する y → 0。初期値は y(0) = Pi - Pf = y0 を与える。
                if u_pwm is None or IDENTIFICATION_INPUT_MODE == 'ff_input':                                # FF入力を同定入力にする場合
                    ident_y = y_shifted                                                                     # 偏差系（最終位置を原点にした実測データ）
                    ident_y0 = y0                                                                           # 偏差系の初期値 y(0) = Pi - Pf
                    u_ident = u_ff.copy()                                                                   # 同定に使用する入力（印加したFF入力そのもの）
                    ident_offset = Pf                                                                       # 応答をPOT値へ戻すためのオフセット
                else:                                                                                       # 実PWMを同定入力にする場合
                    ident_y = raw_y - Pi                                                                    # z形式（初期位置を原点にした実測データ）
                    ident_y0 = 0.0                                                                          # z形式の初期値 z(0)=0
                    u_ident = u_pwm - u_hold                                                                # 同定に使用する入力（初期姿勢の保持分を除去）
                    ident_offset = Pi                                                                       # 応答をPOT値へ戻すためのオフセット

                sys_params = solver.fit_system_model(ident_y, u_ident, ident_y0, self.prev_sys_params[dof_idx])  # システムモデルの同定をして、パラメータを取得（前回の同定結果を初期値に使う）
                self.prev_sys_params[dof_idx] = list(sys_params)                                            # 次回同定の初期値として保存

                # 3. Calculate squared error J
                T1_sys, zeta_sys, wn_sys, b0_sys = sys_params                                               # システムモデルのパラメータ[T1, zeta, wn, b0]を取り出す
                a2_sys, a1_sys, a0_sys = solver.system_coeffs(T1_sys, zeta_sys, wn_sys)                     # 3次遅れ系の係数へ展開
                y_sys_ident, _, _ = solver.simulate_forced(solver.t_eval, a2_sys, a1_sys, a0_sys, b0_sys, u_ident, ident_y0)  # 同定したシステムモデルの応答を取り出す（同定した座標系のまま）
                y_sys_sim = y_sys_ident + ident_offset - Pf                                                 # 表示・残差評価用に偏差系（Pf基準）へそろえる

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
                        'y_gap': y_gap,                                                     # 実測が無く補間の直線になっている区間（ビューアはここを実測として描かない）
                        'y_sys': y_sys_sim + Pf,                                            # 今回同定したシステムモデルの応答
                        'y_tgt': y_tgt + Pf,                                                # 今回同定した目標モデルの応答
                        'u_opt_used': u_opt_used,                                           # 今回印加したFFの元になったu_opt（初回同定時はNaN）
                        'u_ff_applied': u_ff,                                               # 今回ロボットへ送信したFF入力（この実測データを生成した入力）
                        'u_ident': u_ident,                                                 # システムモデル同定に実際に使った入力（IDENTIFICATION_INPUT_MODEで中身が変わる）
                        'tgt_params_id': list(tgt_params),                                  # 今回同定した目標モデル[T1, wn]（Time-POTの破線に対応）
                        'sys_params_id': list(sys_params),                                  # 今回同定したシステムモデル[T1, zeta, wn, b0]（Time-POTの点線に対応）
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

            total_J = float(sum(J_array[dof_idx] for dof_idx in OPT_DOF_IDX))                                           # 最適化対象の自由度の評価関数値Jだけを足す（対象外はNaNのまま算入しない）

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
                f"これまでのベストJ = {self.min_J_sum if self.min_J_sum != float('inf') else total_J:.2f} {diff_msg} "
                f"[対象DOF {', '.join(str(k + 1) for k in OPT_DOF_IDX)} の合計 / 閾値 {self.threshold_J:.2f}]"
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
                self.current_ff_matrix = self.build_initial_ff_matrix(self.T)                                           # FFパラメータをリセット（対象自由度のみ初回励振用の非ゼロ初期値）
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
            'retry': int(self.last_measure_retry),                                                                      # そのループの測定に要した再測定回数（0なら1回で成功）
        })
        hist = self.param_history                                                                                       # 履歴の参照

        snap = {
            # --- 現在の状態 ---
            'session': self.session_id,                                                                                 # この実行の識別ID（ビューアが前回の実行と区別するために使う）
            'outer': self.current_outer + 1, 'max_outer': self.max_outer_iter,                                          # 外側ループ番号
            'inner': self.current_inner, 'max_inner': self.max_inner_iter,                                              # 内側ループ番号
            'total_J': float(total_J), 'best_J': float(self.min_J_sum),                                                 # 今回のJ・ベストJ
            'T': float(self.T), 'time': time.strftime('%Y-%m-%d %H:%M:%S'),                                             # FF制御時間・更新時刻
            'ident_mode': IDENTIFICATION_INPUT_MODE,                                                                    # システムモデル同定に使った入力（'actual_pwm' / 'ff_input'）
            'opt_dof': OPT_DOF_MASK.copy(),                                                                             # 自由度ごとの最適化対象フラグ（Falseの自由度は実測POT以外を計算していない）
            'threshold_J': float(self.threshold_J),                                                                     # 内側ループの収束判定閾値
            'retry_used': int(self.last_measure_retry), 'retry_total': int(self.retry_total),                           # この測定に要した再測定回数・この実行での通算再測定回数
            'max_retry': int(MAX_MEASURE_RETRY),                                                                        # 再測定の上限回数
            # --- 最新の波形（24自由度 × 時系列） ---
            't': np.arange(0, SIM_TIME, self.dt, dtype=np.float32),                                                     # 時間軸
            'y_data': stack('y_data'), 'y_sys': stack('y_sys'), 'y_tgt': stack('y_tgt'),                                # 実測・システムモデル・目標モデル
            'y_gap': np.array([np.asarray(d['y_gap'], dtype=bool) for d in snap_dof], dtype=bool),                      # 実測が無く補間で作った区間
            'u_opt_used': stack('u_opt_used'), 'u_ff_applied': stack('u_ff_applied'),                                   # 使用したu_opt・印加したFF
            'u_ident': stack('u_ident'),                                                                                # システムモデル同定に実際に使った入力
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
            'hist_retry': np.array([r['retry'] for r in hist], dtype=np.int32),                                         # 各内側ループの測定に要した再測定回数の推移
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
            J_dof = self.best_debug_data[dof_idx]['J']                                                                                      # そのDOFの評価関数J（最適化対象外はNaN）
            ws.cell(row=dof_idx + 2, column=1, value=dof_idx + 1)                                                                           # A列へDOF番号を格納
            ws.cell(row=dof_idx + 2, column=2,                                                                                              # B列へ評価関数Jを格納
                    value=float(J_dof) if np.isfinite(J_dof) else None)                                                                     # 最適化対象外は空欄（NaNはExcelが読めないため）
        
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
