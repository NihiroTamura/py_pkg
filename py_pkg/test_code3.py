#!/usr/bin/env python3
import os                                                       # ファイルパス操作や環境変数、ファイル削除・置換を行う標準ライブラリ
import sys                                                      # プログラム終了（sys.exit）やシステムパラメータを扱う標準ライブラリ
from collections import deque                                   # 固定長リング（保持PWMの直近サンプル保存用）
import time                                                     # 時刻の取得や待機処理（sleep）を行う標準ライブラリ
import threading                                                # スレッド処理ライブラリ
import traceback                                                # エラー内容表示するライブラリ
import warnings                                                 # 警告表示を非表示にするライブラリ

import matplotlib                                               # グラフ描画用ライブラリ
matplotlib.use('Agg')                                           # GUIウィンドウを表示せず、バックグラウンドでの画像ファイル（PNG）書き出し専用バックエンドに設定
import matplotlib.pyplot as plt                                 # matplotlibの描画インタフェースモジュール
import numpy as np                                              # 数値計算ライブラリ
import openpyxl                                                 # Excelファイル（.xlsx）の作成・読み書きを行うライブラリ
from openpyxl.drawing.image import Image as OpenpyxlImage       # openpyxlのExcelシート上に画像を貼り付けるためのクラス
import pandas as pd                                             # データフレーム操作およびCSVへの保存を行うライブラリ

import rclpy                                                    # ROS2ライブラリ
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # ROS 2コールバックの排他実行グループ
from rclpy.executors import MultiThreadedExecutor               # トピック購読やタイマーをマルチスレッドで並列実行する実行器
from rclpy.node import Node                                     # ROS 2ノードを作成するためのベースクラス
from std_msgs.msg import Float32MultiArray, UInt16MultiArray    # ROS2メッセージ型

import scipy.optimize                                           # 最小二乗法（least_squares）等の最適化用サブモジュール
from scipy.signal import cont2discrete                          # 連続時間→離散時間(ZOH)厳密離散化
from cmaes import CMA                                           # CMA-ES（進化戦略）最適化ライブラリ

import tkinter as tk                                            # GUIライブラリ
from tkinter import filedialog                                  # GUIでファイルやフォルダを選択するためのモジュール

warnings.simplefilter('ignore', RuntimeWarning)                 # 浮動小数点演算のオーバーフローなどの RuntimeWarning を非表示に設定
np.seterr(all='ignore')                                         # Numpyのエラーを無視

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
THRESHOLD_J = 1000000.0     # 内側ループの収束判定閾値（評価関数Jがこの値以下になれば収束）

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
# 閉ループ同定（仕上げ）のパラメータ（チューニング要素）
#   開ループ同定（fit_system_model）は「実測PWMを入力として与えたときの実測POT」に合わせるだけで、
#   ADRCがその入力をどう作ったかを見ていない。そのため同定結果をADRCで閉じ直すと発散すること
#   があり（実測データでは 21/24 自由度で ρ(A_cl) > 1）、最適制御の内部で計算される u_ADRC が
#   実測PWMとまったく違う値になってしまう。
#
#   そこで開ループ同定の結果を初期値として、
#     「ADRC込みの閉ループを実際に印加したFFで回し、実測POTと実測PWMの両方に合わせる」
#   仕上げ最適化を行う。実測PWMを残差に入れることで u_ADRC が実測に一致し、同時に発散する
#   モデルは残差が跳ね上がるため自動的に安定領域へ追い込まれる。
# ==============================================================================
#   PWM残差の重み CL_ID_W_PWM について（実測DOF4: 目標変位400カウント で調べた結果）
#     W=0（POT残差だけ）にすると、位置は 10 カウントで合うが b0/a0 = 0.22 count/PWM という
#     モデルになる。これは「400カウント動かすのに 1840 PWM 要る」という意味で、PWMの上限が
#     ±255 である以上ありえない。位置だけに合わせると、入力のスケールがいくらでも小さい方向へ
#     逃げられてしまうためこうなる。
#     W≥0.1 にすると b0/a0 = 7.4 count/PWM（400カウントに 54 PWM）となり、実測の入力振れ幅
#     rms 54 PWM とぴたりと一致する。つまりPWM残差は「同定した入力ゲインを物理的に正しい
#     大きさに保つ」ために必要な項であって、単なる検証用の飾りではない。
#     引き換えに位置残差は 10 → 127 カウントへ悪化し、wn は上限（π/dt/SYS_WN_NYQ_RATIO）に
#     張り付く。実測データが3次遅れモデルで表せるより速い応答を要求しているためで、位置の
#     過渡形状をもっと合わせたい場合は SYS_WN_NYQ_RATIO を緩めるか W を下げて調整する。
CL_ID_ENABLE = True         # 閉ループ同定の仕上げを行うか（Falseで従来どおり開ループ同定のみ）
CL_ID_W_PWM = 1.0           # PWM残差の重み（POT残差との比。どちらも各自のスケールで正規化した後に掛ける）
CL_ID_MAX_NFEV = 300        # 仕上げ最適化の関数評価回数の上限（1評価あたり約1〜2ms）
CL_ID_RHO_MAX = 1.0         # 仕上げ結果を採用する閉ループスペクトル半径の上限（これを超えたら開ループ結果へ戻す）

# ==============================================================================
# 目標値切替時の比例キック検査のパラメータ（チューニング要素）
#   目標値が切り替わった瞬間は、オブザーバ状態 z1,z2,z3 がまだ更新されておらず、5次関数FFも
#   f(0)=0 なので、印加PWMの跳びは次式で厳密に決まる。
#       Δu_pwm = kp · step / input_coef
#   オブザーバもプラントも関与しないので、これは実測PWMだけで
#     ・ADRCゲイン表（ADRC_KP / ADRC_INPUT_COEF）が実機と一致しているか
#     ・目標値がロボットへ届くまでの通信遅れ δ が何サンプルか
#   を高い分解能で検証できる唯一の点になる。
# ==============================================================================
ADRC_KICK_MIN = 5.0         # 理論キックがこの大きさ[PWM]未満の自由度は判定しない（目標値変化が小さすぎて埋もれる）
ADRC_KICK_MAX_DELAY = 8     # 通信遅れ δ を探すサンプル数の上限
ADRC_KICK_WARN = 0.25       # 実測キック/理論キックがこの割合以上ずれたら警告する

# ==============================================================================
# 初回システム同定用の初期FF入力の振幅（チューニング要素）
#   ゼロ入力ではシステムモデルの b0 が同定不能（上限に張り付く）ため、十分な励振を
#   与える非ゼロFFを初期値とする。2つの極値の大きさ |f(t1)|=|f(t2)| がこの値になる。
# ==============================================================================
INIT_FF_PEAK = 40.0         # 初期FFの極値の大きさ [PWM]（f(t1)=+50≥40, f(t2)=-50≤-40 を満たす）

# ==============================================================================
# 実機ADRCのパラメータ（最適化しない固定値）
#   最適制御の内部でADRCを逐次計算するため、実機のTeensyスケッチ
#   （ADRC_ROS2_tk_FFoptimize-board1〜5.ino）に書かれている値をそのまま24自由度分並べる。
#   最適化の対象は u_FF_opt だけであり、ここの値は一切最適化しない。
#   実機側のゲインを変更したら、この表も必ず同じ値へ更新すること。
#
#   並び順は build_dof_map() と同じ24自由度
#     DOF  1- 6 : board1 の index0〜5
#     DOF  7-12 : board2 の index0〜5
#     DOF 13-18 : board3 の index0〜5
#     DOF 19-21 : board4 の index0〜2
#     DOF 22-24 : board5 の index0〜2
#
#   実機ADRCの構成（各自由度は対角ゲインなので完全に独立。1自由度ぶんを抜き出して使う）
#     ESO（拡張状態オブザーバ, 前進オイラー・制御周期 ADRC_DT）
#         z1 ← z1 + dt( z2 + β1(y - z1) )                        z1: 角度の推定値
#         z2 ← z2 + dt( z3 + β2(y - z1) + input_coef·u_ADRC )    z2: 角速度の推定値
#         z3 ← z3 + dt( β3(y - z1) )                             z3: 外乱（動特性）の推定値
#       ※ ESOの入力項に入るのは u_ADRC だけで、FF入力 u_FF は入らない（実機 ESO() と同じ）。
#          FF入力はESOから見れば外乱であり、z3 が推定する側にまわる。
#     ADRC制御則
#         u_ADRC = ( -z3 + kp(P_desired - z1) - kd·z2 ) / input_coef
#     オブザーバゲイン（-λ₀ の三重極配置, 実機 computeBeta() と同じ）
#         β1 = 3λ₀,  β2 = 3λ₀²,  β3 = λ₀³
# ==============================================================================
ADRC_DT = 0.001             # 実機ADRCの制御周期 [s]（Teensy の CONTROL_PERIOD_MS = 1ms）

# PDゲイン kp（実機 kp_1?[]）
ADRC_KP = np.array([
     2200.0, 12000.0,  8000.0,  9000.0,  6000.0,  6000.0,        # board1 DOF1〜6
     2000.0, 14000.0,  9000.0,  9000.0,  6000.0,  6000.0,        # board2 DOF7〜12
    10000.0, 10000.0,  8000.0, 10000.0,  8000.0,  8500.0,        # board3 DOF13〜18
     7000.0,  6000.0,  6000.0,                                   # board4 DOF19〜21
     4000.0, 19000.0,  8000.0,                                   # board5 DOF22〜24
])

# PDゲイン kd（実機 kd_1?[]）
ADRC_KD = np.array([
      115.0,   650.0,   225.0,   500.0,   420.0,   520.0,        # board1 DOF1〜6
      100.0,   600.0,   300.0,   500.0,   300.0,   450.0,        # board2 DOF7〜12
      230.0,   230.0,   400.0,   230.0,   250.0,   230.0,        # board3 DOF13〜18
      300.0,   280.0,   230.0,                                   # board4 DOF19〜21
      150.0,   240.0,   200.0,                                   # board5 DOF22〜24
])

# 制御入力係数 input_coef（実機 input_coef_1?[]）
ADRC_INPUT_COEF = np.array([
    40000.0, 20000.0, 30000.0, 200000.0, 30000.0, 30000.0,       # board1 DOF1〜6
    40000.0, 20000.0, 30000.0, 200000.0, 30000.0, 30000.0,       # board2 DOF7〜12
    40000.0, 54000.0, 30000.0,  40000.0, 40000.0, 50000.0,       # board3 DOF13〜18
    30000.0, 50000.0, 20000.0,                                   # board4 DOF19〜21
    60000.0, 30000.0, 60000.0,                                   # board5 DOF22〜24
])

# オブザーバの極 λ₀（実機 lamda_0[]。極は -λ₀ の三重根）
ADRC_OBS_POLE = np.array([
      800.0,   300.0,   300.0,   700.0,   300.0,   300.0,        # board1 DOF1〜6
      800.0,   300.0,   300.0,   700.0,   300.0,   300.0,        # board2 DOF7〜12
      300.0,   300.0,   300.0,   300.0,   300.0,   500.0,        # board3 DOF13〜18
      300.0,   500.0,   500.0,                                   # board4 DOF19〜21
      800.0,   300.0,   800.0,                                   # board5 DOF22〜24
])

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
        self.last_cl_radius = float('nan')                  # 直近に組んだADRC閉ループのスペクトル半径（1を超えると内部ADRCが発散している）
        self.last_u_adrc = None                             # 直近の最適化で内部ADRCが出した入力の時系列（絶対PWM）

    # ------------------------------------------------------------------
    # 離散化・シミュレーション
    # ------------------------------------------------------------------

    # 3次遅れ系（可制御正準形）を scipy.signal.cont2discrete() でZOH厳密離散化する関数
    def _discretize(self, a2, a1, a0, b0, dt=None):                 # 引数(3次遅れ系の係数 a2・a1・a0・b0, 離散化の刻み幅)
        """連続時間状態方程式 ẋ = A_c x + B_c u をZOH（Zero-Order Hold）で厳密離散化する。

            連続系（可制御正準形, 出力 y = x0）
                A_c = [[0,1,0],[0,0,1],[-a0,-a1,-a2]],  B_c = [0,0,b0]ᵀ
                → 状態 x = [位置 y, 速度 ẏ, 加速度 ÿ]
            離散系（ZOH厳密離散化）
                x(k+1) = A_d x(k) + B_d u(k),  A_d = expm(A_c·dt),  B_d = (∫₀^{dt} expm(A_c τ)dτ) B_c

            dt を省略するとシミュレーション刻み self.dt を使う（従来どおり）。ADRCを内部に含む
            閉ループを組むときだけ、実機の制御周期 ADRC_DT を指定して呼ぶ。
        """
        dt = self.dt if dt is None else float(dt)                   # 離散化の刻み幅（既定はシミュレーション刻み）
        A_c = np.array([[0.0,   1.0,   0.0],
                        [0.0,   0.0,   1.0],
                        [-a0,   -a1,   -a2]])                        # 連続時間 A（可制御正準形）
        B_c = np.array([[0.0], [0.0], [b0]])                        # 連続時間 B
        C_d = np.zeros((1, 3))                                      # 出力行列（状態フィードバックのためダミー）
        D_d = np.zeros((1, 1))                                      # 直達行列（ダミー）
        A_d, B_d, _, _, _ = cont2discrete((A_c, B_c, C_d, D_d), dt, method="zoh")   # ZOH厳密離散化
        return A_d, B_d.flatten()                                   # A_d (3x3), B_d (3,) を返す

    # 実機と同じ三重極配置でオブザーバゲインを計算する関数
    @staticmethod
    def observer_gains(pole):                                       # 引数(オブザーバの極 λ₀)
        """β1 = 3λ₀, β2 = 3λ₀², β3 = λ₀³（実機 computeBeta() と同じ）"""
        return 3.0 * pole, 3.0 * pole ** 2, pole ** 3

    # 実機ADRCの1制御周期ぶんの式を、オブザーバ状態 ζ だけの行列として返す関数
    @classmethod
    def _adrc_step_matrices(cls, adrc_params, step, dt=ADRC_DT):    # 引数(ADRCパラメータ(kp,kd,input_coef,λ₀), 目標変位 Pf-Pi, 制御周期)
        """実機の ESO() と ADRC() を、オブザーバ状態 ζ だけのアフィン写像として書き下す。

            ζ(k+1)    = M_open ζ(k) + L_y·y_z(k) + B_u·u_ADRC(k-1)
            u_ADRC(k) = g_z·ζ(k+1) + g_c

          ζ = [ζ1, ζ2, ζ3] は実機の z1,z2,z3 を z 形式へ平行移動したもの（ζ1 = z1 - Pi）。
          y_z は「実測値に相当する出力」の z 形式（y_z = P - Pi）で、ここでは外から与える量として
          扱う。最適制御の内部では y_z = x0（システムモデルの出力）を代入し、実測との照合では
          実測POTを代入する。どちらも同じこの関数を通すことで、両者が必ず同じ式になる。

              M_open = [[1-dt·β1, dt,   0 ]   L_y = [dt·β1, dt·β2, dt·β3]  （推定誤差 y_z - ζ1 の係数）
                        [ -dt·β2,  1,  dt ]   B_u = [0, dt·ic, 0]          （ESOの入力項 ic·u_ADRC）
                        [ -dt·β3,  0,   1 ]]  g_z = [-kp/ic, -kd/ic, -1/ic]（制御則 u_ADRC の係数）
                                              g_c = kp·step/ic             （制御則の目標値ぶん）

          M_open = I + dt(A - LC) はESOの誤差ダイナミクスそのもので、固有値は 1-dt·λ₀ の三重根。
          ADRCが無効な自由度（input_coef ≤ 0 または λ₀ ≤ 0）は None を返す。
        """
        kp, kd, ic, pole = [float(v) for v in adrc_params]                                      # ADRCパラメータ（PDゲイン, 制御入力係数, オブザーバの極）
        if not (ic > 0.0 and pole > 0.0):                                                       # ADRCが無効な自由度（ゲイン未設定）の場合
            return None                                                                         # 呼び出し側で開ループへ縮退させる
        b1, b2, b3 = cls.observer_gains(pole)                                                   # オブザーバゲイン β1, β2, β3
        M_open = np.array([[1.0 - dt * b1, dt,  0.0],                                           # ζ1 ← ζ1 + dt( ζ2 + β1·innov )
                           [-dt * b2,      1.0, dt],                                            # ζ2 ← ζ2 + dt( ζ3 + β2·innov + ic·u_ADRC⁻ )
                           [-dt * b3,      0.0, 1.0]])                                          # ζ3 ← ζ3 + dt( β3·innov )
        L_y = np.array([dt * b1, dt * b2, dt * b3])                                             # 推定誤差 innov = y_z - ζ1 のうち y_z 側の係数
        B_u = np.array([0.0, dt * ic, 0.0])                                                     # ESOの入力項（実機 sum_input = input_coef·outputADRC）
        g_z = np.array([-kp / ic, -kd / ic, -1.0 / ic])                                         # 制御則 u_ADRC = (-ζ3 + kp(step-ζ1) - kd ζ2)/ic の係数
        g_c = kp * step / ic                                                                    # 制御則の定数項（目標値ぶん）
        return M_open, L_y, B_u, g_z, float(g_c)

    # 上の1周期写像を「u_ADRC(k-1) も ζ(k) から決まる」自励式へ閉じる関数
    @classmethod
    def _adrc_closed_step(cls, adrc_params, step, dt=ADRC_DT):      # 引数(ADRCパラメータ(kp,kd,input_coef,λ₀), 目標変位 Pf-Pi, 制御周期)
        """ESOの入力項に入る u_ADRC(k-1) を、実機どおり ζ(k) から自己無撞着に決めた形。

            u_ADRC(k-1) = g_z·ζ(k) + g_c  を代入して
                ζ(k+1) = M_zz ζ(k) + L_y·y_z(k) + m_z,   M_zz = M_open + B_u g_zᵀ,  m_z = B_u·g_c

          実機は ESO() → ADRC() の順に呼ぶので、ESOが使う u_ADRC は前周期の値であり、それは
          「前周期の ADRC() が更新後の ζ から計算した値」＝ 今周期の入口の ζ の関数になる。

          注意: この形では ESO の +dt·ζ3 と ic·u_ADRC の中の -ζ3 が厳密に相殺し、M_zz の
          3列目が [0,0,1]ᵀ になる。すなわち ζ3 は推定誤差の純粋な積分器で、M_zz は固有値1を
          必ず持つ。プラントで閉じない限り（実測POTをそのまま流し込むような使い方では）
          この積分器が入力の再構成誤差を溜め込むため、長時間の再現には使えない。
        """
        mats = cls._adrc_step_matrices(adrc_params, step, dt)                                   # 1周期ぶんの開いた形を取得
        if mats is None:                                                                        # ADRCが無効な自由度の場合
            return None
        M_open, L_y, B_u, g_z, g_c = mats                                                       # 開いた形を取り出す
        M_zz = M_open + np.outer(B_u, g_z)                                                      # u_ADRC(k-1) = g_z·ζ(k) + g_c を代入して閉じる
        m_z = B_u * g_c                                                                         # その定数項
        return M_zz, L_y, m_z, g_z, g_c

    # 目標値切替時の比例キックから、通信遅れとADRCゲインの整合を実測する関数
    @staticmethod
    def detect_adrc_kick(u_pwm, u_hold, kp, ic, step,                                           # 引数(実測PWM, 初期姿勢の保持PWM, PDゲインkp, 制御入力係数, 目標変位 Pf-Pi)
                         max_delay=ADRC_KICK_MAX_DELAY, min_kick=ADRC_KICK_MIN):                # 引数(遅れ探索の上限サンプル数, 判定に必要な最小キック)
        """目標値がロボットへ届いた時刻 δ と、比例キックの実測/理論比を返す。

        目標値が切り替わった瞬間、オブザーバ状態はまだ1度も更新されておらず（z1=Pi, z2=0,
        z3=-ic·u_hold のまま）、5次関数FFも f(0)=0 なので、印加PWMの跳びは
            Δu_pwm = ( -z3 + kp(Pf - z1) - kd·z2 )/ic - u_hold = kp · step / ic
        となる。オブザーバの収束にもプラントの応答にも依存しない厳密式なので、
        実測PWMの立ち上がり1点だけで ADRC_KP / ADRC_INPUT_COEF の整合と通信遅れを検証できる。

        戻り値: (delay [サンプル], ratio = 実測キック / 理論キック)
                目標値変化が小さすぎる、または立ち上がりを見つけられない場合は (nan, nan)。
        """
        kick = float(kp) * float(step) / float(ic) if ic else 0.0                               # 理論キック kp·step/input_coef [PWM]
        if not np.isfinite(kick) or abs(kick) < min_kick:                                       # 目標値変化が小さく、ノイズに埋もれる自由度
            return float('nan'), float('nan')                                                   # 判定不能として返す
        u = np.asarray(u_pwm, dtype=float)                                                      # 実測PWM
        for k in range(min(int(max_delay), u.size)):                                            # 立ち上がりを先頭から探す
            if abs(u[k] - u_hold) > 0.5 * abs(kick):                                            # 理論キックの半分を超えたサンプルを立ち上がりとみなす
                return float(k), float((u[k] - u_hold) / kick)                                  # 遅れ[サンプル]と実測/理論比を返す
        return float('nan'), float('nan')                                                       # 上限まで見つからなければ判定不能

    # ADRCを内部に含む閉ループ系の離散状態方程式を構築する関数
    def _adrc_closed_loop(self, sys_params, adrc_params, step, u_hold):     # 引数(システムモデルのパラメータ[T1,zeta,wn,b0], ADRCパラメータ(kp,kd,input_coef,λ₀), 目標変位 Pf-Pi, 初期姿勢の保持PWM)
        """同定済みシステムモデルとADRCを結合し、シミュレーション1刻みぶんのアフィン写像を作る。

            ξ(k+1) = A_cl ξ(k) + B_cl u_FF_opt(k) + c_cl

        ● 拡大状態 ξ（6次元）
            ξ = [ x0, x1, x2, ζ1, ζ2, ζ3 ]ᵀ
              x = システムモデルの状態（z形式・可制御正準形）  x0 = z = P - Pi
              ζ = ADRCオブザーバの状態。実機の z1,z2,z3 をそのまま持つが、角度推定 z1 だけ
                  z形式へ平行移動して ζ1 = z1 - Pi とする。ESOに入る量は
                      推定誤差 y - z1 = (z + Pi) - (ζ1 + Pi) = x0 - ζ1
                      目標偏差 P_desired - z1 = Pf - (ζ1 + Pi) = step - ζ1
                  のどちらも Pi が相殺するので、式は実機と同一のまま絶対位置が消える
                  （数値も 500 前後のオフセットを持ち回らずに済む）。

        ● 1制御周期（ADRC_DT = 1ms, 実機の制御周期）の更新。実機 loop() の呼び出し順
          「ESO() を回してから ADRC() を回す」をそのまま再現する
            0) 出力（＝実測値に相当する量）        y - z1 = x0 - ζ1
            1) 前周期のADRC出力                    u_ADRC⁻ = ( -ζ3 + kp(step - ζ1) - kd ζ2 ) / ic
            2) ESO更新（前進オイラー）             ζ' = M_obs ξ + m_obs
            3) 今周期のADRC出力（更新後のζで計算）  u_ADRC = ( -ζ3' + kp(step - ζ1') - kd ζ2' ) / ic
                                                          = u_row·ξ + u_c
            4) システムモデルへ与える総入力         u = ( u_ADRC - u_hold ) + u_FF_opt
               （同定と同じ z 形式の入力規約 u = u_total - u_hold。ADRCの絶対PWM出力から
                 初期姿勢の保持分を引いたものが、z形式で見たADRCの寄与になる）
            5) システムモデル更新                   x' = A_s x + B_s u

        ● シミュレーション刻み SIM_DT への合成
            u_FF_opt は SIM_DT ごとの区分一定（ZOH）なので、1msの写像 Â を N_sub = SIM_DT/ADRC_DT 回
            合成すれば SIM_DT ぶんの写像になる。全体が線形（アフィン）なので厳密に合成できる。
                A_cl = Â^N,  B_cl = (Σ_{i<N} Â^i) B̂,  c_cl = (Σ_{i<N} Â^i) ĉ
            この合成は最適化の前に1回行うだけなので、順方向計算の負荷は従来（3次系）とほぼ同じ。

        ● 初期状態 ξ(0)
            t=0 の直前まで、ロボットは目標値 Pi で静止しADRCが u_hold を出し続けていた。
            その定常状態は  ζ1 = 0（＝z1 = Pi）, ζ2 = 0, u_ADRC = -ζ3/ic = u_hold  なので
                ζ3(0) = -ic · u_hold
            とする。t=0 で目標値が Pf へ跳ぶため、直後のADRC出力は
                u_ADRC(0) = u_hold + kp·step/ic
            となり、実機どおり比例分のキックから始まる。

        ADRCが無効な自由度（input_coef = 0 など）は u_ADRC ≡ u_hold（z形式では0）として扱い、
        従来のFFのみの開ループ定式化へそのまま縮退させる。
        """
        n_x, n_z, n_xi = 3, 3, 6                                                                # 状態次元（システム, オブザーバ, 拡大系）
        T1_sys, zeta_sys, wn_sys, b0_sys = sys_params                                           # システムモデルのパラメータ取得
        a2_sys, a1_sys, a0_sys = self.system_coeffs(T1_sys, zeta_sys, wn_sys)                   # 3次遅れ系の係数へ展開
        A_s, B_s = self._discretize(a2_sys, a1_sys, a0_sys, b0_sys, dt=ADRC_DT)                 # 実機の制御周期でZOH離散化（1msごとにADRCが動くため）

        ic = float(adrc_params[2])                                                              # 制御入力係数（無効判定と ζ3(0) に使う）
        S = np.hstack([np.eye(n_x), np.zeros((n_x, n_z))])                                      # 拡大状態からシステム状態を取り出す行列 x = S ξ
        closed = self._adrc_closed_step(adrc_params, step)                                      # ADRC1周期ぶんの自励式（実測との照合と完全に同じ関数を通す）

        if closed is None:                                                                      # ADRCが無効な自由度（ゲイン未設定）の場合
            A_hat = np.zeros((n_xi, n_xi)); A_hat[:n_x] = A_s @ S                               # オブザーバは動かさず、システムモデルだけを更新する
            B_hat = np.concatenate([B_s, np.zeros(n_z)])                                        # 入力はFFのみ
            c_hat = np.zeros(n_xi)                                                              # 定数項なし（u_ADRC - u_hold = 0 とみなす）
            u_row, u_c = np.zeros(n_xi), u_hold                                                 # u_ADRC ≡ u_hold（z形式では寄与0）
        else:
            M_zz, L_y, m_obs, g_z, g_c = closed                                                 # ζだけの1周期写像と制御則の係数

            # ζ の式を拡大状態 ξ の式へ持ち上げる（外部入力 y_z にシステムモデルの出力 x0 を代入する）
            M_obs = np.zeros((n_z, n_xi))                                                       # ζ' = M_obs ξ + m_obs の係数行列
            M_obs[:, 0] = L_y                                                                   # y_z = x0 を代入（推定誤差 innov = x0 - ζ1 の x0 側）
            M_obs[:, n_x:] = M_zz                                                               # ζ 側はそのまま

            # 今周期のADRC出力（実機は ESO() の後に ADRC() を呼ぶので、更新後の ζ' で計算される）
            u_row = g_z @ M_obs                                                                 # u_ADRC(k) = u_row·ξ(k) + u_c（絶対PWM）
            u_c = float(g_z @ m_obs + g_c)                                                      # その定数項

            # 1制御周期ぶんの拡大系アフィン写像 ξ ← Â ξ + B̂ u_FF_opt + ĉ
            A_hat = np.vstack([
                A_s @ S + np.outer(B_s, u_row),                                                 # x' = A_s x + B_s( u_ADRC - u_hold + u_FF_opt )
                M_obs,                                                                          # ζ' = M_obs ξ + m_obs
            ])
            B_hat = np.concatenate([B_s, np.zeros(n_z)])                                        # u_FF_opt の係数
            c_hat = np.concatenate([B_s * (u_c - u_hold), m_obs])                               # 定数項

        # シミュレーション刻み SIM_DT ぶん（N_sub 個の1msステップ）へ合成する
        n_sub = max(1, int(round(self.dt / ADRC_DT)))                                           # 1シミュレーション刻みに入るADRC制御周期の数
        A_pow = np.eye(n_xi)                                                                    # Â^i を積み上げる作業変数
        G = np.zeros((n_xi, n_xi))                                                              # Σ_{i<N} Â^i
        for _ in range(n_sub):                                                                  # N_sub 回合成する
            G += A_pow                                                                          # Σ へ Â^i を加える
            A_pow = A_hat @ A_pow                                                               # Â^(i+1) へ更新
        A_cl, B_cl, c_cl = A_pow, G @ B_hat, G @ c_hat                                          # SIM_DT ぶんの閉ループ写像

        # 初期状態（初期姿勢 Pi で静止しADRCが u_hold を出し続けていた定常状態）
        xi0 = np.zeros(n_xi)                                                                    # x(0)=0（z形式なので初期位置が原点）, ζ1(0)=ζ2(0)=0
        xi0[n_x + 2] = -ic * u_hold if ic > 0.0 else 0.0                                        # ζ3(0) = -ic·u_hold（保持入力と釣り合う外乱推定）

        return A_cl, B_cl, c_cl, xi0, u_row, u_c                                                # 閉ループ写像・初期状態・u_ADRC再構成用の係数を返す

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

    # 同定済みシステムモデルとADRCを結合した閉ループを、実測POTと実測PWMの両方に合わせて回す関数
    def simulate_closed_loop(self, sys_params, adrc_params, step, u_hold, u_ff):                    # 引数(システムモデルのパラメータ, ADRCパラメータ, 目標変位 Pf-Pi, 保持PWM, 印加したFF入力)
        """ADRCを内部に含む閉ループを、実際に印加したFF入力で順方向に回す。

        最適制御（calculate_el_ff）が u_FF_opt を最適化しながら回しているのと同じ写像を、
        「今回ロボットへ実際に送ったFF」で1回だけ回したもの。したがって出力は実測と直接比較できる。
        戻り値: (z_sim (M,), u_adrc_sim (M,), rho)
          z_sim      : z形式のシステムモデル出力（= P - Pi）。実測 raw_y - Pi と比較する
          u_adrc_sim : 内部ADRCが各時刻で出した入力（絶対PWM）。実測 u_pwm - u_ff と比較する
          rho        : 閉ループのスペクトル半径（1を超えると発散している）
        """
        M = len(self.t_eval)                                                                        # シミュレーションのステップ数
        A_cl, B_cl, c_cl, xi0, u_row, u_c = self._adrc_closed_loop(sys_params, adrc_params, step, u_hold)   # 閉ループのZOH離散状態方程式
        uff = np.asarray(u_ff, dtype=float)                                                         # 印加したFF入力
        xi = np.zeros((M, 6))                                                                       # 拡大状態列 ξ(0..M-1)
        xi[0] = xi0                                                                                 # 初期状態（初期姿勢で静止しADRCが u_hold を出していた定常状態）
        with np.errstate(all='ignore'):                                                             # 発散モデルのオーバーフロー警告を抑制
            for k in range(M - 1):                                                                  # k=0..M-2 を順方向に更新
                xi[k + 1] = A_cl @ xi[k] + B_cl * uff[k] + c_cl                                     # ξ(k+1)=A_cl ξ(k)+B_cl u_FF(k)+c_cl
            try:
                rho = float(np.max(np.abs(np.linalg.eigvals(A_cl))))                                # 閉ループのスペクトル半径
            except np.linalg.LinAlgError:                                                           # 固有値が求まらない場合
                rho = float('nan')
        return xi[:, 0], xi @ u_row + u_c, rho                                                      # z形式の出力・内部ADRC出力（絶対PWM）・スペクトル半径

    # 開ループ同定の結果を、ADRC込みの閉ループで実測POT・実測PWMの両方に合うよう仕上げる関数
    def refine_system_model_closed_loop(self, sys_params, adrc_params, step, u_hold,                # 引数(開ループ同定の結果, ADRCパラメータ, 目標変位 Pf-Pi, 保持PWM)
                                        z_meas, u_adrc_meas, u_ff):                                 # 引数(z形式の実測POT, 実測ADRC出力(絶対PWM), 印加したFF入力)
        """開ループ同定 (fit_system_model) の結果を初期値に、閉ループで再フィットする。

        ● なぜ必要か
            開ループ同定は「実測PWMを入力として与えたときの実測POT」に合わせるだけで、その
            PWMをADRCがどう作ったかを一切見ていない。閉ループデータに出力誤差法を当てる形なので
            結果は偏り、同定したモデルをADRCで閉じ直すと発散することがある（実測データでは
            21/24 自由度で ρ(A_cl) > 1）。その状態では最適制御の内部で計算される u_ADRC が
            実測PWMとまったく違う値になり、u_FF_opt の最適化が意味を失う。

        ● 何をするか
            探索変数は開ループ同定と同じ (T1, zeta, wn, b0)（対数で持つのも境界も同じ）。
            残差は、ADRC込みの閉ループを「実際に印加したFF」で回した結果と実測の差を、
            POTとPWMの両方について取る。
                r = [ (z_sim - z_meas)/σ_pot ,  √W (u_adrc_sim - u_adrc_meas)/σ_pwm ]
                σ_pot = max(|step|, 1)                     位置の代表スケール [count]
                σ_pwm = max(rms(u_adrc_meas - u_hold), 1)  入力の代表スケール [PWM]
            それぞれ自分のスケールで割ってから重み W を掛けるので、W は「PWMをPOTの何倍
            重視するか」という無次元量になる。
            発散するモデルは残差が跳ね上がる（打ち切り前の序盤サンプルに勾配が残る）ので、
            最適化は自然に安定領域へ向かう。

        ● 採用条件
            仕上げ結果は次の両方を満たしたときだけ採用する。片方でも欠けたら開ループ結果を返す。
              1. PWM残差が開ループ初期値より改善している … u_ADRC が実測へ近づいた
              2. 安定性を悪化させていない … ρ(A_cl) ≤ CL_ID_RHO_MAX か、少なくとも仕上げ前以下
            2つ目を「ρ ≤ 1 でなければ不採用」にしてはいけない。開ループ結果も ρ > 1 のことが
            多く、その場合に不採用にすると「同じく発散するのに残差だけ桁違いに悪いモデル」を
            わざわざ選ぶことになるため。悪化させないことだけを条件にする。
            励振不足で b0 が負に同定されるような自由度では改善が得られず、自動的に従来どおりの
            結果へフォールバックする。

        ● この残差が示すこと・示さないこと（読み違えないための注意）
            仕上げ後の残差 res_pwm は「同定したモデルが実機を再現できているか」の指標であって、
            ADRCパラメータ表（ADRC_KP など）が実機と一致しているかの検証にはならない。
            仕上げはシステムモデル側を自由に動かせるので、ADRC側のパラメータが多少ずれていても
            モデルがそれを吸収してしまうためである（実測で確認: ADRC_DT を 1ms→5ms と誤らせても
            残差は 11.5→8.5 PWM とむしろ下がる）。
            ADRCパラメータ表の検証は detect_adrc_kick（目標値切替時の比例キック）が担う。あちらは
            オブザーバもプラントも関与しない厳密式なので、kp/input_coef のずれに正確に比例する
            （実測で確認: kp を 1.5倍 に誤らせるとキック比が 1.054→0.703 = 1.054/1.5 へ動く）。

        戻り値: (sys_params, accepted, res_pwm_before, res_pwm_used)
                res_pwm_* は rms(u_adrc_sim - u_adrc_meas) [PWM]。res_pwm_used は
                実際に返したパラメータの残差なので、不採用時は res_pwm_before と一致する。
        """
        sys_params = np.asarray(sys_params, dtype=float)                                            # 開ループ同定の結果
        z_meas = np.asarray(z_meas, dtype=float)                                                    # z形式の実測POT
        u_meas = np.asarray(u_adrc_meas, dtype=float)                                               # 実測ADRC出力（絶対PWM）

        # 残差の正規化スケール（POTはcount、PWMはPWMなので、そのまま足すと片方しか効かない）
        sig_pot = max(abs(float(step)), 1.0)                                                        # 位置の代表スケール（目標変位）
        sig_pwm = max(float(np.sqrt(np.mean((u_meas - u_hold) ** 2))), 1.0)                         # 入力の代表スケール（保持分からの振れ幅）
        w_pwm = np.sqrt(float(CL_ID_W_PWM))                                                         # PWM残差の重み（残差ベクトルへは√で掛ける）

        # PWM残差 rms（採用判定と表示に使う）を計算する関数
        def pwm_rms(params):                                                                        # 引数(システムモデルのパラメータ)
            _, u_sim, rho = self.simulate_closed_loop(params, adrc_params, step, u_hold, u_ff)      # 閉ループを印加FFで回す
            with np.errstate(all='ignore'):
                r = float(np.sqrt(np.mean((u_sim - u_meas) ** 2)))                                  # 実測ADRC出力との差のrms
            return (r if np.isfinite(r) else float('inf')), rho                                     # 発散したモデルはinf扱い

        res_before, rho_before = pwm_rms(sys_params)                                                # 仕上げ前（開ループ同定のまま）の残差
        if not CL_ID_ENABLE:                                                                        # 仕上げを無効にしている場合
            return sys_params, False, res_before, res_before                                        # 開ループ結果をそのまま返す

        wn_max = np.pi / self.dt / SYS_WN_NYQ_RATIO                                                 # 固有振動数の上限（開ループ同定と同じ）
        lower = [np.log(SYS_T1_MIN), SYS_ZETA_MIN, np.log(SYS_WN_MIN), -np.inf]                     # 探索変数の下限（開ループ同定と同じ）
        upper = [np.log(SYS_T1_MAX), SYS_ZETA_MAX, np.log(wn_max),      np.inf]                     # 探索変数の上限（開ループ同定と同じ）

        # 各時刻の残差ベクトルを返す関数（T1, wn は対数で受け取る。開ループ同定と同じ規約）
        def residuals(q):
            log_T1, zeta, log_wn, b0 = q                                                            # 最適化変数の取り出し
            params = (np.exp(log_T1), zeta, np.exp(log_wn), b0)                                     # 極形式のパラメータへ戻す
            with np.errstate(all='ignore'):                                                         # 発散したモデルのオーバーフロー警告を抑制
                z_sim, u_sim, _ = self.simulate_closed_loop(params, adrc_params, step, u_hold, u_ff)    # 閉ループを印加FFで回す
                r = np.concatenate([(z_sim - z_meas) / sig_pot,                                     # POT残差（位置スケールで正規化）
                                    w_pwm * (u_sim - u_meas) / sig_pwm])                            # PWM残差（入力スケールで正規化し重みを掛ける）
            r = np.nan_to_num(r, nan=SYS_RESID_CLIP, posinf=SYS_RESID_CLIP, neginf=-SYS_RESID_CLIP)  # inf/nanを有限値に置換（開ループ同定と同じ）
            return np.clip(r, -SYS_RESID_CLIP, SYS_RESID_CLIP)                                      # 打ち切って最適化が止まらないようにする

        T1_0, zeta_0, wn_0, b0_0 = [float(v) for v in sys_params]                                   # 開ループ同定の結果を開始点にする
        if not (T1_0 > 0 and wn_0 > 0):                                                             # 対数を取れない開始点の場合
            return sys_params, False, res_before, res_before                                        # 仕上げを諦めて開ループ結果を返す
        x0 = np.clip([np.log(T1_0), zeta_0, np.log(wn_0), b0_0], lower, upper)                      # 開始点を境界内に収めてから渡す

        try:
            res = scipy.optimize.least_squares(                                                     # Σr² が最小になる変数[log T1, zeta, log wn, b0]を最適化する
                residuals, x0=x0, bounds=(lower, upper),
                method='trf', x_scale='jac', max_nfev=CL_ID_MAX_NFEV,                               # 評価回数に上限を掛けて1内側ループの時間を抑える
            )
        except Exception:                                                                           # 数値的に破綻した場合
            return sys_params, False, res_before, res_before                                        # 開ループ結果へフォールバック

        log_T1, zeta, log_wn, b0 = res.x                                                            # 最適変数を取り出す
        refined = np.array([np.exp(log_T1), zeta, np.exp(log_wn), b0])                              # (T1, zeta, wn, b0) へ戻す
        res_after, rho_after = pwm_rms(refined)                                                     # 仕上げ後の残差とスペクトル半径

        stable_ok = np.isfinite(rho_after) and (rho_after <= CL_ID_RHO_MAX                          # 内部ADRCが発散しないか、
                                                or rho_after <= rho_before)                         # 少なくとも仕上げ前より悪化していない
        accepted = bool(np.all(np.isfinite(refined))                                                # パラメータが有限で
                        and res_after < res_before                                                  # 実測PWMへ近づいていて
                        and stable_ok)                                                              # 安定性を悪化させていない場合のみ採用
        if accepted:
            return refined, True, res_before, res_after                                             # 仕上げ結果を採用
        return sys_params, False, res_before, res_before                                            # 条件を満たさなければ開ループ結果と、その残差を返す

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
    def calculate_el_ff(self, target_params, sys_params, y0, adrc_params, u_hold):                           # 引数(目標モデルのパラメータ[T1, wn], システムモデルのパラメータ[T1, zeta, wn, b0], 初期偏差, ADRCパラメータ(kp,kd,input_coef,λ₀), 初期姿勢の保持PWM)
        """
        離散時間オイラー・ラグランジュ（随伴／勾配）法で最適制御入力を計算し、
        FF分 u_FF_opt を5次多項式 FF = a*t^5 + ... + e*t にフィットする。

        目的：システムモデルの軌道を、目標モデルが描く目標軌道 x_tgt へ一致させる。
          座標系       : z = P - Pi（初期位置を原点とする系）。システムモデル同定と同じ規約で、
                         入力は u = u_total - u_hold。x_sys(0)=x_tgt(0)=0 から step = Pf - Pi へ立ち上がる
          入力の分解   : u(k) = ( u_ADRC(k) - u_hold ) + u_FF_opt(k)
                         u_ADRC は実測から与えるのではなく、この関数の中でADRCを逐次計算して得る
                         （ESOがシステムモデルの出力を「実測値」として受け取り、状態推定を更新し、
                           その推定値からPD＋外乱補償で u_ADRC を作る）。最適化変数は u_FF_opt だけ。
                         目標位置を保持するための定常入力はADRCが担うので、u_FF_opt は移動の
                         過渡補正だけを受け持つ。
          入力ホライズン : u_FF_opt(k) は [0,T]（k=0..N_ff-1）のみ最適化し、それ以降は0。
                         k ≥ N_ff では u(k) = u_ADRC(k) - u_hold となり、ADRCが姿勢を保持し続ける
          端点条件     : u_FF_opt(0) = 0, u_FF_opt(N_ff) = 0 をハード制約として固定する
          評価ホライズン : コスト J はシミュレーション全体（5秒, k=0..M-1）で評価する

        ● ADRCを内部に含む閉ループ系（詳細は _adrc_closed_loop() を参照）
            拡大状態 ξ(k) = [ x_sys(k) ; ζ(k) ]（システムモデル3状態 ＋ ADRCオブザーバ3状態）を
            とると、ADRCはPD＋線形オブザーバなので閉ループ全体がアフィン系になり、
            シミュレーション1刻みぶんの写像を厳密に合成できる。
                ξ(k+1) = A_cl ξ(k) + B_cl u_FF_opt(k) + c_cl,   x_sys(k) = S ξ(k)
            A_cl の中で毎刻み次の順に計算が回っている（実機の1ms制御周期を SIM_DT/1ms 回ぶん合成）。
                1. システムモデルへ u_ADRC + u_FF_opt を入力する
                2. その出力（角度）を「実測値」に相当する量としてESOへ渡す
                3. 出力とオブザーバ推定値との差 (y - z1) から状態推定 z1,z2,z3 を更新する
                4. 更新した推定値から u_ADRC = ( -z3 + kp(Pf - z1) - kd z2 ) / input_coef を計算する
                5. u_ADRC と u_FF_opt を合わせて次の刻みのシステムモデル入力にする
            ADRCの状態は k から k+1 へそのまま引き継がれる（ξ に含まれているため）。

        ● 目標軌道
            目標     : x_tgt(k) は A_tgt の自由応答（誤差系 y0→0）を z 形式へ定数オフセットして生成する
                       （詳細は下の「目標軌道 x_tgt(k) を z 形式で生成する」ブロックを参照）
            誤差     : e(k) = x_sys(k) - x_tgt(k) = S ξ(k) - x_tgt(k)
            従来は誤差 e 自体を状態変数にして e(k+1)=A_sys e(k)+B_sys u(k)+d(k) と書いていたが、
            ADRCが絶対的な状態（オブザーバ推定値）を持つようになったため、状態は拡大状態 ξ の
            まま進め、目標との差はコストの中で引く形に整理した。式の中身は等価である。

        ● コスト関数（終端コストなし）／ハミルトニアン
            J = Σ_{k=0}^{M-1} ( e(k)ᵀ Q e(k) )
              + Σ_{k=0}^{N_ff-1} R u_FF_opt(k)²
              + Σ_{k=0}^{N_ff}  R_du ( u_FF_opt(k) - u_FF_opt(k-1) )²,  u_FF_opt(-1)=u_FF_opt(N_ff)=0
            入力罰則・レート罰則は最適化変数 u_FF_opt にだけ掛ける。ADRCが出す u_ADRC を罰しても
            最適化には効かないうえ、総入力に掛けるとレート罰則の境界0が「終端でADRCの保持入力まで
            0へ引き戻す」意味になってしまう。u_FF_opt に掛ければ境界0はそのまま端点条件
            u_FF_opt(0)=u_FF_opt(N_ff)=0 と一致する。

            段コスト L(k) = e(k)ᵀ Q e(k) + R u_FF_opt(k)² + R_du ( u_FF_opt(k)-u_FF_opt(k-1) )²
            H(k) = L(k) + λ(k+1)ᵀ [ A_cl ξ(k) + B_cl u_FF_opt(k) + c_cl ]

        ● 随伴方程式（λ は拡大状態と同じ6次元。レート罰則は u_FF_opt のみの関数で ∂/∂ξ = 0）
            λ(N) = λ(M) = 0
            λ(k) = ∂H/∂ξ(k) = 2 Sᵀ Q e(k) + A_clᵀ λ(k+1)
            A_cl にはADRCのフィードバック経路が含まれるので、随伴を逆向きに解くだけで
            「u_FF_opt を動かすとADRCがどう反応し、その反応が軌道をどう変えるか」まで勾配に入る。

        ● 勾配（最適化変数 u_FF_opt, k=0..N_ff-1）
                ∂J/∂u_FF_opt(k) = ∂H(k)/∂u_FF_opt(k) + ∂L(k+1)/∂u_FF_opt(k)
                                 = 2 R u_FF_opt(k) + B_clᵀ λ(k+1)
                                   + 2 R_du ( 2u_FF_opt(k) - u_FF_opt(k-1) - u_FF_opt(k+1) )
            末尾の括弧は Dirichlet境界条件 u_FF_opt(-1)=u_FF_opt(N_ff)=0 を課した2階差分
            （3重対角行列 L=tridiag(-1,2,-1)）で、(L u_FF_opt)(k) と書ける。
            端点 k=0 は u_FF_opt(0)=0 のハード制約なので、勾配の第0成分を0にして更新しない（射影）。
            k=N_ff は決定変数の外なので u_FF_opt(N_ff)=0 は構造的に満たされる。

        解法（未知変数は [0,T] の u_FF_opt(k)、k≥N_ff では u_FF_opt(k)=0）：
          1. u_FF_opt(k) を初期化する（零入力から開始。u_FF_opt(0)=0 は以後ずっと保たれる）
          2. ADRCを含む閉ループから ξ(k) を順方向計算し、誤差 e(k)=Sξ(k)-x_tgt(k) を得る（5秒全体）
          3. λ(N) = 0
          4. 随伴方程式を逆方向計算（5秒全体）
          5. ∂H/∂u_FF_opt を計算（k=0..N_ff-1）し、端点条件のため第0成分を0にする
          6. Σ||∂H/∂u_FF_opt||² < ε なら終了
          7. そうでなければ u_FF_opt ← clip(u_FF_opt - α ∂H/∂u_FF_opt, -255, 255) として 2 へ戻る

        最後に、最適な u_FF_opt を与えたときに閉ループ内のADRCが実際に出していた入力を
        u_ADRC(k) = u_row·ξ(k) + u_c として取り出し、総最適入力 u_opt = (u_ADRC - u_hold) + u_FF_opt
        を作る。5次多項式で近似するのは総入力ではなくFF分 u_FF_opt であり、
        u_FF_opt(0)=u_FF_opt(T)=0 は5次関数の端点条件 f(0)=f(T)=0 とそのまま整合する。
        """
        T1, wn = target_params                                                                          # 目標モデルのパラメータ取得
        a2_tgt, a1_tgt, a0_tgt = self._target_coeffs(T1, wn)                                            # 目標モデルの3次遅れ系の係数を計算

        dt = self.dt                                                                                    # シミュレーションのサンプル刻み幅
        M = len(self.t_eval)                                                                            # 評価ホライズン（5秒全体）のステップ数
        N_ff = int(round(self.T / dt))                                                                  # 入力ホライズン [0,T] のステップ数
        n_x = 3                                                                                         # システムモデルの状態次元 (可制御正準形: x0=位置, x1=速度, x2=加速度)
        n_xi = 6                                                                                        # 拡大状態の次元（システムモデル3状態 ＋ ADRCオブザーバ3状態）

        # -----------------------------------------------------------
        # 同定結果から離散時間状態方程式を構築（cont2discrete によるZOH厳密離散化）
        #   閉ループ : ξ(k+1) = A_cl ξ(k) + B_cl u_FF_opt(k) + c_cl（システムモデル＋ADRC）
        #   目標     : A_tgt は誤差系（y→0）の自由応答行列。同定はこのまま変更しない
        # -----------------------------------------------------------
        step = -y0                                                                                      # 目標変位 step = Pf - Pi（z形式での最終値。ADRCの目標偏差にも使う）
        A_cl, B_cl, c_cl, xi0, u_row, u_c = self._adrc_closed_loop(sys_params, adrc_params, step, u_hold)   # ADRCを内部に含む閉ループのZOH離散状態方程式
        A_tgt, _ = self._discretize(a2_tgt, a1_tgt, a0_tgt, 0.0)                                        # 目標モデルのZOH離散状態方程式（u=0）

        # 閉ループの安定性を記録する（同定が外れると内部ADRCが発散し、勾配が意味を失うため）
        with np.errstate(all='ignore'):                                                                 # 固有値計算が破綻しても最適化を止めない
            try:
                self.last_cl_radius = float(np.max(np.abs(np.linalg.eigvals(A_cl))))                    # 閉ループのスペクトル半径（1以下なら安定）
            except np.linalg.LinAlgError:                                                               # 固有値が求まらない場合
                self.last_cl_radius = float('nan')

        # 目標軌道 x_tgt(k) を z 形式（初期位置を原点とする系）で生成する
        #   A_tgt の同定は従来どおり誤差系（y = P - Pf, y0 → 0 へ収束する自由応答）で行うが、
        #   最適制御ではシステムモデル同定と同じ z = P - Pi の系（z(0)=0 → step へ立ち上がる）で扱う。
        #   両者は定数オフセットの関係にあり、z_tgt(k) = y_tgt(k) - y0 が厳密に成り立つ。
        #   （3次系は相対次数3なので、step応答 z は自由応答 y の1-補数 z = step(1 - y/y0), step = -y0。
        #     位置成分だけがオフセットされ、速度・加速度成分は自由応答のまま変わらない。）
        #   この座標系では x_sys(0)=x_tgt(0)=0 となり、入力規約も同定と同じ u = u_total - u_hold で
        #   そろう。姿勢保持のための定常入力はADRCが担うので、最適化する u_FF_opt には混入しない。
        #   e(M) までコストに使えるよう、M+1 点分を生成する。
        x_tgt = np.zeros((M + 1, n_x))                                                                  # 目標状態列 x_tgt(0..M) (M+1, 3)
        x_tgt[0] = np.array([y0, 0.0, 0.0])                                                             # 誤差系での初期状態（自由応答の初期値）
        for k in range(M):                                                                              # 目標モデルの自由応答を逐次計算
            x_tgt[k + 1] = A_tgt @ x_tgt[k]                                                             # x_tgt(k+1) = A_tgt x_tgt(k)
        y_tgt = x_tgt[:M, 0].copy()                                                                     # 目標出力（誤差系 y0→0）。呼び出し側の規約は従来どおり維持する
        x_tgt[:, 0] += step                                                                             # 位置成分だけを z 形式へオフセット（x_tgt(0)の位置=0, 最終値=step）

        Q = self.Q                                                                                      # 状態誤差の重み行列 (3x3)
        R = float(self.R[0, 0])                                                                         # 入力の重みスカラー
        R_du = float(self.R_du)                                                                         # 入力レート（Δu）の重みスカラー

        # ---- 手順2: ADRCを含む閉ループを順方向に計算する関数 ----
        #   ξ(k+1) = A_cl ξ(k) + B_cl u_FF_opt(k) + c_cl。A_cl の中で毎刻み
        #   「システムモデルへ入力 → 出力をESOへ渡す → 状態推定を更新 → u_ADRC を計算」が回り、
        #   ADRCの状態 ζ は ξ の一部として次の時刻へそのまま引き継がれる。
        #   u_FF_opt は [0,T](k<N_ff) のみで、それ以降は0（＝ADRCだけで姿勢を保持する区間）。
        def forward(u_seq):                                                                             # 引数(最適化変数 u_FF_opt の列)
            xi = np.zeros((M + 1, n_xi))                                                                # 拡大状態列 ξ(0..M)
            xi[0] = xi0                                                                                 # 初期状態（初期姿勢で静止しADRCが u_hold を出していた定常状態）
            for k in range(M):                                                                          # k=0..M-1 を順方向に更新
                uff_k = u_seq[k] if k < N_ff else 0.0                                                   # 最適化変数 u_FF_opt(k)（FF区間の外は0）
                xi[k + 1] = A_cl @ xi[k] + B_cl * uff_k + c_cl                                          # ξ(k+1)=A_cl ξ(k)+B_cl u_FF_opt(k)+c_cl
            return xi

        # ---- 拡大状態から目標軌道との誤差 e(k) = x_sys(k) - x_tgt(k) を取り出す関数 ----
        def error_traj(xi):                                                                             # 引数(拡大状態列)
            return xi[:, :n_x] - x_tgt                                                                  # 位置・速度・加速度の誤差 (M+1, 3)

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
        #   λ(M)=0（手順3, 終端コストなし）, λ(k)=2 Sᵀ Q e(k)+A_clᵀ λ(k+1)（手順4）
        #   λ は拡大状態と同じ6次元。状態コストは x_sys 成分だけに掛かるので、
        #   2 Q e(k) を先頭3成分に置き（＝Sᵀ を掛ける）、残りのオブザーバ成分は0にする。
        #   入力レート罰則は u だけの関数（∂/∂ξ = 0）なので、随伴方程式の形は従来のまま変わらない。
        def backward(e_traj):
            lam = np.zeros((M + 1, n_xi))                                                               # 随伴変数列 λ(0..M)（λ(M)=0）
            qe = np.zeros(n_xi)                                                                         # 2 Sᵀ Q e(k) を入れる作業ベクトル
            for k in range(M - 1, 0, -1):                                                               # k=M-1..1 を逆方向に更新
                qe[:n_x] = 2.0 * (Q @ e_traj[k])                                                        # 状態コストの寄与（x_sys 成分のみ）
                lam[k] = qe + A_cl.T @ lam[k + 1]                                                       # λ(k)=2 Sᵀ Q e(k)+A_clᵀ λ(k+1)
            return lam

        # ---- 手順5: 勾配（入力ホライズン k=0..N_ff-1） ----
        #   ∂J/∂u(k) = 2 R u(k) + B_clᵀ λ(k+1) + 2 R_du ( 2u(k)-u(k-1)-u(k+1) )
        #   第3項がレート罰則による寄与。u(k) が段 k と段 k+1 の両方の段コストに現れるため、
        #   H(k) からの +2R_du(u(k)-u(k-1)) と L(k+1) からの -2R_du(u(k+1)-u(k)) の和になる。
        def gradient(u_seq, lam):
            g = (2.0 * R * u_seq                                                                        # 入力そのものの罰則 2 R u_FF_opt(k)
                 + lam[1:N_ff + 1] @ B_cl                                                               # 随伴からの寄与 B_clᵀ λ(k+1)（ADRCの反応は A_cl 経由で λ に入っている）
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
            xi_traj = forward(u)                                                                        # 手順2: ADRCを含む閉ループを順方向計算
            e_traj = error_traj(xi_traj)                                                                # 目標軌道との誤差 e(k)=x_sys(k)-x_tgt(k)
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
            Js = cost(error_traj(forward(u - s * grad)), u - s * grad)                                  # 方向 -grad 上の1点でコストを評価
            gHg = 2.0 * (Js - J0 + s * gnorm2) / (s * s)                                                # 2次曲率 gᵀH g（Jはuの2次形式）
            alpha = gnorm2 / gHg if gHg > 1e-30 else s                                                  # 厳密最小ステップ α*=||g||²/(gᵀH g)

            accepted = False                                                                            # コストを減少させる α が見つかったか
            for _ls in range(EL_LS_MAX):                                                                # バックトラッキング
                u_cand = np.clip(u - alpha * grad, -255.0, 255.0)                                       # PWM制約 -255≤u≤255 を満たすようclip
                J_cand = cost(error_traj(forward(u_cand)), u_cand)                                      # 候補入力のコスト
                if J_cand < J0:                                                                         # コストが減少したら採用
                    accepted = True
                    break
                alpha *= 0.5                                                                            # 減少しなければステップ幅を半分にして再試行
            if not accepted:                                                                            # どのステップ幅でも改善しなければ収束
                break
            u = u_cand                                                                                  # 入力列を更新
            if J0 - J_cand < 1e-9 * (abs(J0) + 1.0):                                                    # 改善が微小なら収束
                break

        # 最適な u_FF_opt を与えたときの閉ループ軌道を作り直し、内部ADRCが出していた入力を取り出す
        #   u_ADRC(k) = u_row·ξ(k) + u_c（絶対PWM。実機の outputADRC に相当する量）。
        #   z形式のシステムモデルへ実際に入る量はここから保持分を引いた u_ADRC(k) - u_hold。
        xi_traj = forward(u)                                                                            # 最適入力での閉ループ軌道 ξ(0..M)
        u_adrc = xi_traj[:M] @ u_row + u_c                                                              # 内部ADRCが各時刻で出した入力 u_ADRC(0..M-1)
        self.last_u_adrc = u_adrc                                                                       # 呼び出し側の確認用に保存

        # FF分 u_FF_opt をシミュレーション時間全体の配列へ格納
        #   決定変数は [0,T] のみで、それ以降は0（＝ADRCだけで姿勢を保持する区間）。
        #   端点は u_FF_opt(0)=0（ハード制約）, u_FF_opt(N_ff)=0（決定変数の外）で、
        #   5次関数の端点条件 f(0)=f(T)=0 とそのまま整合する。
        u_ff_opt = np.zeros(M)                                                                          # ELで最適化したFF入力 u_FF_opt
        u_ff_opt[:N_ff] = u                                                                             # FF入力区間 (0..N_ff-1) に最適化したFF分を格納

        # 総最適入力 u_opt = ( u_ADRC - u_hold ) + u_FF_opt（z形式でシステムモデルへ入る入力）
        #   k ≥ N_ff では u_opt = u_ADRC - u_hold となり、ADRCが姿勢を保持し続ける。
        u_opt = (u_adrc - u_hold) + u_ff_opt                                                            # 総最適入力

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
        self.prev_u_ff_opt = [None] * 24                                                                                                         # 前回ループでELが計算したFF入力 u_FF_opt（今回の実測データを生成した入力）
        self.prev_u_opt = [None] * 24                                                                                                            # 前回ループでELが計算した総最適入力 u_opt =（内部ADRC － 保持分）＋ u_FF_opt
        self.prev_tgt_params = [None] * 24                                                                                                       # 前回の目標モデル同定結果[T1, wn]（次回同定の初期値に使う）
        self.prev_sys_params = [None] * 24                                                                                                       # 前回のシステムモデル同定結果[T1, zeta, wn, b0]（次回同定の初期値に使う）
        self.prev_fit_restart = [0] * 24                                                                                                         # 前回ループのCMA-ES再探索回数（ビューア用）
        self.param_history = []                                                                                                                  # 内側ループごとのパラメータ・評価値の履歴（ビューア用、外側ループごとにリセット）

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
            def build_24_dof(buffer, label, grid, quiet=False):                                                         # 引数(受信バッファ, ログ表示名, 揃える時間軸, 警告を出さないか)
                out, gaps = [], []                                                                                      # 24自由度分の値と欠測マスクの空リスト
                for b_id in range(1, 6):                                                                                # board1からboard5の順に処理
                    samples = list(buffer[f'board{b_id}'])                                                  # 各boardに対応する受信データ（受信時刻, 値）を取りだす
                    n_dof = 3 if b_id in [4, 5] else 6                                                      # board4と5は3自由度、それ以外は6自由度
                    if not samples:                                                                         # 1点も受信できなかった場合
                        out.extend([None] * n_dof); gaps.extend([None] * n_dof)                             # 未受信として格納
                        if not quiet:
                            self.get_logger().warn(f"{label}-board{b_id}: 1点も受信できませんでした")
                        continue
                    stamps = np.array([s[0] for s in samples], dtype=float)                                 # 受信時刻の配列
                    arr = np.array([s[1] for s in samples], dtype=float)                                    # 受信値の配列
                    gap = self.gap_mask(stamps, grid)                                                       # 欠測マスク（受信時刻はboard内の全自由度で共通）
                    if not quiet:
                        self.report_gaps(label, b_id, stamps, gap, grid)                                    # 欠測があれば警告ログを出す
                    for j in range(n_dof):                                                                  # そのboardの自由度の順に処理
                        out.append(self.resample_by_time(stamps, arr[:, j], grid))                          # 受信時刻で補間して格納
                        gaps.append(gap)
                return out, gaps

            # 1段目: まず素の時間軸で揃えて、目標値がロボットへ届くまでの通信遅れ δ を測る
            #   シミュレーションの t=0 は「Pythonが目標値をpublishし終えた瞬間」だが、ロボットが
            #   実際に目標値を受け取ってFFタイマを開始するのは DDS→micro-ROS→シリアル の分だけ
            #   後になる。この遅れはどこにも計測されていないため、実測PWMの比例キック
            #       Δu_pwm = kp·step/input_coef
            #   が立ち上がるサンプル番号から直接読み取る（詳細は detect_adrc_kick 参照）。
            pot_raw, _ = build_24_dof(self.buffer_pot, "POT", t_grid, quiet=True)                                       # 遅れ推定用のPOT（警告は2段目で出す）
            pwm_raw, _ = build_24_dof(self.buffer_pwm, "PWM", t_grid, quiet=True)                                       # 遅れ推定用のPWM
            kick_ratio = [float('nan')] * 24                                                                            # 自由度ごとの実測キック/理論キック
            kick_delay = [float('nan')] * 24                                                                            # 自由度ごとの通信遅れ[サンプル]
            for d_i in range(24):                                                                                       # 24自由度分のキック検査
                u_d = pwm_raw[d_i]                                                                                      # その自由度の実測PWM
                hold_d = self.hold_pwm_24[d_i] if self.hold_pwm_24 is not None else None                                # 目標値切替直前の保持PWM
                if u_d is None or hold_d is None:                                                                       # PWMか保持PWMが取れなかった場合
                    continue                                                                                            # 判定不能のまま次の自由度へ
                step_d = self.target_pot[dof_map[d_i]] - self.initial_pot[dof_map[d_i]]                                 # 目標変位 Pf - Pi
                kick_delay[d_i], kick_ratio[d_i] = solver.detect_adrc_kick(                                             # 比例キックから遅れとゲイン整合を測る
                    u_d, hold_d, ADRC_KP[d_i], ADRC_INPUT_COEF[d_i], step_d)

            det = [d for d in kick_delay if np.isfinite(d)]                                                             # 判定できた自由度の遅れ
            lag = int(round(float(np.median(det)))) if det else 0                                                       # 全体の遅れは中央値を採用（通信路の性質なのでboard間で共通のはず）
            if det:                                                                                                     # 1つでも判定できた場合
                ratios = [r for r in kick_ratio if np.isfinite(r)]                                                      # 判定できた自由度のキック比
                self.get_logger().info(                                                                                 # ログ出力
                    f"通信遅れ δ = {lag} サンプル ({lag * self.dt * 1000:.0f} ms)  "
                    f"[判定できた自由度 {len(det)}/24, δ={sorted(set(int(d) for d in det))}]  "
                    f"比例キック 実測/理論 = {np.mean(ratios):.3f} (平均)"
                )
                if lag > 5:                                                                                             # 遅れが大きい場合
                    self.get_logger().warn(f"目標値がロボットへ届くまで {lag} サンプル遅れています。通信経路を確認してください")
            else:                                                                                                       # 1つも判定できなかった場合
                self.get_logger().info("比例キックを判定できる自由度がありません（目標値の変化が小さい）。通信遅れ補正なしで進みます")

            # 2段目: 実測を δ ぶん前詰めして、t=0 が本当の目標値切替時刻になるよう揃え直す
            #   ロボット時間はグリッド時間より δ だけ遅れているので、グリッド時刻 t+δ·dt の実測を
            #   時刻 t の値として使う。resample_by_time / gap_mask をそのまま使えるので、
            #   時間軸をずらすだけで済む（末尾は既存どおり端の値でクランプされる）。
            t_shift = t_grid + lag * self.dt                                                                            # 前詰めした時間軸
            data_24_dof, gap_24_dof = build_24_dof(self.buffer_pot, "POT", t_shift)                                     # 実測POT値を24自由度分へ変換
            pwm_24_dof, _ = build_24_dof(self.buffer_pwm, "PWM", t_shift)                                               # 実制御入力（PWM）を24自由度分へ変換

            J_array = []                                                                                                # 24自由度それぞれの評価関数Jの空リスト
            debug_info = []                                                                                             # デバッグExcelへ保存する情報の空リスト
            extrema_list = []                                                                                           # 保存する極大値と極小値の空リスト
            used_extrema_list = []                                                                                      # 極値のリスト
            snap_dof = []                                                                                               # ビューア用データ（自由度ごと）の空リスト

            # 今回ロボットへ送信したFF（更新前）を保存
            ff_used_this_iteration = [row[:] for row in self.current_ff_matrix]                                         # パラメータの保存
            used_extrema_list =  [solver.calc_extrema_from_ff(ff) for ff in ff_used_this_iteration]                     # 極値の保存

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

                # ビューア表示用に「今回印加したFFを作るのに使った値」を、同定で上書きされる前に控えておく
                #   prev_u_ff_opt が None のときは初回同定（最適制御前）なので、モデルパラメータもu_FF_optも無い
                if self.prev_u_ff_opt[dof_idx] is None:                                                      # 初回同定（印加したのは初期励振FF）
                    u_ff_opt_used = np.full(N_samples, np.nan)                                               # 表示なし（NaNはグラフに描かれない）
                    u_opt_used = np.full(N_samples, np.nan)                                                  # 表示なし
                    tgt_used = [np.nan, np.nan]                                                              # 表示なし
                    sys_used = [np.nan] * 4                                                                  # 表示なし
                    restart_used = -2                                                                        # 表示なしを示す値
                else:                                                                                        # 2回目以降（前回の最適制御結果を印加している）
                    u_ff_opt_used = self.prev_u_ff_opt[dof_idx]                                              # 今回のFFの元になったu_FF_opt
                    u_opt_used = self.prev_u_opt[dof_idx]                                                    # そのときの総最適入力（内部ADRC分を含む）
                    tgt_used = list(self.prev_tgt_params[dof_idx])                                           # そのu_FF_optを計算するのに使った目標モデル
                    sys_used = list(self.prev_sys_params[dof_idx])                                           # そのu_FF_optを計算するのに使ったシステムモデル
                    restart_used = self.prev_fit_restart[dof_idx]                                            # そのFFを作ったときのCMA-ES再探索回数

                # 1. Target Model ID
                tgt_params = solver.fit_target_model(y_shifted, y0, self.prev_tgt_params[dof_idx])          # 目標モデルの同定をして、パラメータを取得（前回の同定結果を初期値に使う）
                self.prev_tgt_params[dof_idx] = list(tgt_params)                                            # 次回同定の初期値として保存

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

                # この自由度のADRCパラメータ（実機と同じ固定値。最適化しない）
                #   u_ADRC は実測から与えず、最適制御の中でこのパラメータのADRCを逐次計算して得る。
                adrc_params = (ADRC_KP[dof_idx], ADRC_KD[dof_idx],                                          # PDゲイン kp, kd
                               ADRC_INPUT_COEF[dof_idx], ADRC_OBS_POLE[dof_idx])                            # 制御入力係数 input_coef, オブザーバの極 λ₀

                # 実測PWMから、実機ADRCが実際に出していた入力を復元する
                #   実機は pub2 = outputADRC + outputAdd を配信しているので、こちらが送った5次関数FF
                #   u_ff を引けばADRC分だけが残る。FFはESOの入力項に入らない（実機 ESO() 参照）ので、
                #   この引き算だけで u_ADRC が厳密に取り出せる。最適制御の内部で計算される u_ADRC は
                #   この時系列と一致していなければならない。
                #   PWMを受信できなかった自由度では u_pwm がFF入力で代用されており実測ではないので、
                #   照合できる量が無いことを示すNaNにする（0を入れると平坦な線として描かれ誤解を招く）。
                u_adrc_meas = (u_pwm - u_ff) if pwm_24_dof[dof_idx] is not None else np.full(N_samples, np.nan)   # 実測ADRC出力（絶対PWM）

                sys_params = solver.fit_system_model(z_data, u_ident, 0.0, self.prev_sys_params[dof_idx])   # システムモデルの同定をして、パラメータを取得（初期値0、前回の同定結果を初期値に使う）

                # 閉ループ同定の仕上げ（開ループ同定の結果を初期値に、実測POTと実測PWMの両方へ合わせる）
                #   開ループ同定は実測PWMを入力として与えるだけでADRCを見ていないため、結果をADRCで
                #   閉じ直すと発散することがある。ここで閉ループのまま再フィットして、内部ADRCの出力が
                #   実測PWMに一致し、かつ発散しないモデルへ寄せる。
                #   PWMが無い・欠測が多い自由度では判断材料が無いので仕上げをスキップする。
                cl_skip = (pwm_24_dof[dof_idx] is None) or (y_gap.mean() > GAP_WARN_RATIO)                   # 仕上げを行えない自由度か判定
                if cl_skip:                                                                                 # 実測PWMが無い、または欠測が多い場合
                    cl_ok, adrc_res_open, adrc_res = False, float('nan'), float('nan')                      # 仕上げなし。残差も判定不能
                else:                                                                                       # 仕上げを行える場合
                    sys_params, cl_ok, adrc_res_open, adrc_res = solver.refine_system_model_closed_loop(     # 閉ループで再フィット
                        sys_params, adrc_params, -y0, u_hold, z_data, u_adrc_meas, u_ff)
                self.prev_sys_params[dof_idx] = list(sys_params)                                            # 仕上げ後の値を次回同定の初期値として保存

                # 3. Calculate squared error J
                #   仕上げを採用できた自由度は、モデル応答も「合わせた対象」と同じ閉ループ応答で評価する。
                #   仕上げできなかった自由度は従来どおり、実測PWMを入力として与えた開ループ応答で評価する。
                if cl_ok:                                                                                   # 閉ループ同定の仕上げを採用した場合
                    z_sys_sim, u_adrc_cl, _ = solver.simulate_closed_loop(                                  # 印加したFFで閉ループを回す
                        sys_params, adrc_params, -y0, u_hold, u_ff)
                else:                                                                                       # 仕上げできなかった場合
                    T1_sys, zeta_sys, wn_sys, b0_sys = sys_params                                           # システムモデルのパラメータ[T1, zeta, wn, b0]を取り出す
                    a2_sys, a1_sys, a0_sys = solver.system_coeffs(T1_sys, zeta_sys, wn_sys)                 # 3次遅れ系の係数へ展開
                    z_sys_sim, _, _ = solver.simulate_forced(solver.t_eval, a2_sys, a1_sys, a0_sys, b0_sys, u_ident, 0.0)   # 同定したシステムモデルの応答を取り出す（z形式：初期値0）
                    u_adrc_cl = np.full(N_samples, np.nan)                                                  # 比較できる内部ADRC出力は無い
                y_sys_sim = z_sys_sim + Pi - Pf                                                             # 表示・残差評価用に偏差系（Pf基準）へ戻す

                # 4. 離散時間オイラー・ラグランジュ最適制御入力計算 + 5次多項式フィット
                #   総入力 u = ( u_ADRC - u_hold ) + u_FF_opt のうち u_FF_opt だけを最適化し、
                #   その u_FF_opt を5次関数で近似する。u_ADRC は実測から与えるのではなく、
                #   同定済みシステムモデルとADRCを結合した閉ループの中で逐次計算される。
                new_ff, extrema, u_pred_full, y_tgt, u_opt, u_ff_opt = solver.calculate_el_ff(tgt_params, sys_params, y0, adrc_params, u_hold)   # FFパラメータ、極値、5次関数FF入力、目標モデルの応答、総最適入力、ELのFF入力を格納
                self.current_ff_matrix[dof_idx] = new_ff                                                    # FFパラメータの更新
                extrema_list.append(extrema)                                                                # 極値を保存

                # 内部ADRCが発散していないか確認する（同定が外れると閉ループが不安定になりうる）
                cl_radius = solver.last_cl_radius                                                           # 閉ループのスペクトル半径（calculate_el_ff が計算した値）
                if not (cl_radius <= 1.0):                                                                  # スペクトル半径が1を超える（NaNもここに入る）場合
                    self.get_logger().warn(                                                                 # 警告ログを出力
                        f"DOF {dof_idx + 1:02d}: 内部ADRC閉ループが不安定です "
                        f"(スペクトル半径 = {cl_radius:.4g})。"
                        f"システムモデルの同定結果を確認してください"
                    )

                # 内部ADRCの出力が実測PWMと合っているか確認する
                #   合わない＝同定モデルが実機を再現できていないということなので、u_FF_opt の最適化も
                #   その分だけ的外れになる。仕上げを採用できなかった自由度も併せて知らせる。
                if not cl_skip and not cl_ok:                                                               # 仕上げを試したが採用できなかった場合
                    self.get_logger().warn(                                                                 # 警告ログを出力
                        f"DOF {dof_idx + 1:02d}: 閉ループ同定の仕上げを採用できませんでした "
                        f"(内部ADRCと実測PWMの残差 = {adrc_res_open:.4g} PWM)。"
                        f"目標値の変化量が小さく励振が足りない可能性があります"
                    )
                if np.isfinite(kick_ratio[dof_idx]) and abs(kick_ratio[dof_idx] - 1.0) > ADRC_KICK_WARN:     # 比例キックが理論値から外れている場合
                    self.get_logger().warn(                                                                 # 警告ログを出力
                        f"DOF {dof_idx + 1:02d}: 目標値切替時の比例キックが理論値の "
                        f"{kick_ratio[dof_idx]:.2f} 倍です。ADRC_KP / ADRC_INPUT_COEF の表が"
                        f"実機のゲインと食い違っている可能性があります"
                    )

                # 内側ループの判定用評価関数J（実測データと目標モデルとの差）
                J = np.sum((y_tgt - y_shifted) ** 2)                                                        # 二乗和誤差を計算
                J_array.append(J)                                                                           # 24自由度それぞれの評価関数Jの空リストに追加

                # 今回の実測データ（＝今回のJ）を生成した入力は、前回ループで計算した u_pred_full / u_FF_opt
                # 初回ループは前回値が無いため、実際に印加したFF入力(u_ff)で代用する
                u_pred_full_for_J = self.prev_u_pred_full[dof_idx] if self.prev_u_pred_full[dof_idx] is not None else u_ff.copy()   # 今回のJを生成した5次関数FF入力
                u_ff_opt_for_J = self.prev_u_ff_opt[dof_idx] if self.prev_u_ff_opt[dof_idx] is not None else u_ff.copy()            # 今回のJを生成したELのFF入力

                # デバック用データを保存
                debug_info.append({
                    't': solver.t_eval,             # 実測時間
                    'y_data': raw_y,                # 実測POT値
                    'y_sys': y_sys_sim + Pf,        # システムモデル応答
                    'y_tgt': y_tgt + Pf,            # 目標モデル応答
                    'J': J,
                    'u_pred_full': u_pred_full_for_J,   # 今回のJを生成した5次関数FF入力（最小J時の値）
                    'u_ff_opt': u_ff_opt_for_J,         # 今回のJを生成したELのFF入力 u_FF_opt（最小J時の値）
                })

                # ビューア用データを保存
                #   波形と極値・モデルパラメータは「この内側ループで実際に使った値」に揃える。
                #   ・応答(Time-POT) は今回の実測と今回同定したモデル
                #   ・入力(Time-PWM) は今回印加したFF(5次関数)と、その元になった前回のu_FF_opt
                #   ・モデルパラメータ履歴は、そのu_FF_optを計算するのに使った前回の同定値
                if ENABLE_SNAPSHOT:
                    snap_dof.append({
                        'y_data': raw_y,                                                    # 実測POT値
                        'y_gap': y_gap,                                                     # 実測が無く補間の直線になっている区間（ビューアはここを実測として描かない）
                        'y_sys': y_sys_sim + Pf,                                            # 今回同定したシステムモデルの応答
                        'y_tgt': y_tgt + Pf,                                                # 今回同定した目標モデルの応答
                        'u_ff_opt_used': u_ff_opt_used,                                     # 今回印加したFFの元になったELのFF入力 u_FF_opt（初回同定時はNaN）
                        'u_opt_used': u_opt_used,                                           # そのときELが想定した総最適入力（内部ADRC分＋u_FF_opt。初回同定時はNaN）
                        'u_ff_applied': u_ff,                                               # 今回ロボットへ送信した5次関数FF入力（この実測データを生成した入力）
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
                        'fit_res': float(np.sum((u_ff - u_ff_opt_used) ** 2)),               # 印加したFFの近似残差 Σ(FF-u_FF_opt)²（初回同定時はNaN）
                        'fit_restart': restart_used,                                        # そのFFを作ったときのCMA-ES再探索回数（-1は退避, -2は該当なし）
                        'id_res_sys': float(np.sum((y_sys_sim - y_shifted) ** 2)),           # 今回のシステムモデル同定の残差
                        # --- 内部ADRCと実機の照合 ---
                        'u_adrc_meas': u_adrc_meas,                                         # 実測ADRC出力 u_pwm - u_ff（絶対PWM, δ前詰め済み）
                        'u_adrc_cl': u_adrc_cl,                                             # 仕上げ後の閉ループが出した内部ADRC出力（絶対PWM）。上と一致すべき量
                        'adrc_res': float(adrc_res),                                        # 内部ADRCと実測PWMの残差rms [PWM]（一致しているかの主指標）
                        'adrc_res_open': float(adrc_res_open),                              # 仕上げ前（開ループ同定のまま）の同じ残差。改善量が見える
                        'adrc_kick': float(kick_ratio[dof_idx]),                            # 目標値切替時の比例キック 実測/理論（ゲイン表の整合）
                        'adrc_delay': float(kick_delay[dof_idx]),                           # この自由度で検出した通信遅れ[サンプル]（未検出はNaN）
                        'cl_radius': float(cl_radius),                                      # 内部ADRC閉ループのスペクトル半径（1を超えると発散）
                        'cl_ok': float(cl_ok),                                              # 閉ループ同定の仕上げを採用したか（1/0）
                    })

                # 次回ループで印加する（＝次回の実測データを生成する）入力を保存
                self.prev_u_pred_full[dof_idx] = u_pred_full                                                 # 次回ループ用の5次関数FF入力を保存
                self.prev_u_ff_opt[dof_idx] = u_ff_opt                                                       # 次回ループ用のELのFF入力 u_FF_opt を保存
                self.prev_u_opt[dof_idx] = u_opt                                                             # 次回ループ用の総最適入力（内部ADRC分を含む）を保存
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
                self.prev_u_ff_opt = [None] * 24                                                                        # FFリセットに伴い前回のu_FF_optもリセット（初回はu_ffで代用）
                self.prev_u_opt = [None] * 24                                                                           # FFリセットに伴い前回の総最適入力もリセット
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
            'adrc_res': stack('adrc_res'), 'cl_radius': stack('cl_radius'),                                             # 内部ADRCと実測PWMの残差・閉ループのスペクトル半径
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
            'y_gap': np.array([np.asarray(d['y_gap'], dtype=bool) for d in snap_dof], dtype=bool),                      # 実測が無く補間で作った区間
            'u_ff_opt_used': stack('u_ff_opt_used'), 'u_ff_applied': stack('u_ff_applied'),                             # ELのFF入力u_FF_opt・印加した5次関数FF
            'u_opt_used': stack('u_opt_used'),                                                                          # ELの総最適入力（内部ADRC分＋u_FF_opt）
            'u_adrc_meas': stack('u_adrc_meas'), 'u_adrc_cl': stack('u_adrc_cl'),                                       # 実測ADRC出力・仕上げ後の閉ループが出した内部ADRC出力
            # --- 最新のパラメータ（24自由度分） ---
            'tgt_params_id': stack('tgt_params_id'), 'sys_params_id': stack('sys_params_id'),                           # 今回同定したモデル
            'tgt_params_used': stack('tgt_params_used'), 'sys_params_used': stack('sys_params_used'),                   # 今回のFFを作るのに使ったモデル
            'ff': stack('ff'), 'extrema': stack('extrema'),                                                             # 印加したFFのパラメータ・極値
            'J': stack('J'), 'Pi': stack('Pi'), 'Pf': stack('Pf'),                                                      # 評価関数値・初期位置・目標位置
            'pot_idx': np.array([d['pot_idx'] for d in snap_dof], dtype=np.int32),                                      # 26要素配列でのインデックス
            'fit_res': stack('fit_res'), 'fit_restart': np.array([d['fit_restart'] for d in snap_dof], dtype=np.int32),  # 近似残差・CMA-ES再探索回数
            'id_res_sys': stack('id_res_sys'),                                                                          # システムモデル同定の残差
            'adrc_res': stack('adrc_res'), 'adrc_res_open': stack('adrc_res_open'),                                     # 内部ADRCと実測PWMの残差（仕上げ後・仕上げ前）
            'adrc_kick': stack('adrc_kick'), 'adrc_delay': stack('adrc_delay'),                                         # 比例キック 実測/理論・通信遅れ[サンプル]
            'cl_radius': stack('cl_radius'), 'cl_ok': stack('cl_ok'),                                                   # 閉ループのスペクトル半径・仕上げを採用したか
            # --- 履歴（内側ループ × 24自由度） ---
            'hist_tgt': np.array([r['tgt'] for r in hist]), 'hist_sys': np.array([r['sys'] for r in hist]),             # 使用した目標モデル・システムモデルの推移
            'hist_ext': np.array([r['ext'] for r in hist]),                                                             # 印加したFFの極値の推移
            'hist_J': np.array([r['J'] for r in hist]),                                                                 # 評価関数値の推移
            'hist_adrc_res': np.array([r['adrc_res'] for r in hist]),                                                   # 内部ADRCと実測PWMの残差の推移
            'hist_cl_radius': np.array([r['cl_radius'] for r in hist]),                                                 # 閉ループのスペクトル半径の推移
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
            plt.plot(data['t'], data['u_ff_opt'], label='EL FF input (u_FF_opt)')
            plt.plot(data['t'], data['u_pred_full'], '--', label='5th-order FF (u_FF)')
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
