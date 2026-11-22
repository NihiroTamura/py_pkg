#!/usr/bin/env python3
"""
FF比較GUI（ADRC＋5次関数FF）: FFを設定・送信して記録し、2つの記録を比較する

単体で動く。既存の GUI_android_tk.py・GUI_android_tk_FFsetting.py・optimize_optuna_ff_ROS2.py・optuna_ff_common.py は
import せず、必要な処理をこのファイルに移植した（移植した部分には「# 移植元: <ファイル>:<行> @105a28e」を付け、
変えた箇所には「# 変更:」で理由を書いた）。このGUIは目標値を送らない。目標値は GUI_android_tk.py などから送る。

<記録タブ>
・FF（/board{1..5}_FFparam_float/sub, Float32MultiArray, 36要素 = 6スロット × [a,b,c,d,e,T]）
    f(t) = a t⁵ + b t⁴ + c t³ + d t² + e t（0 ≤ t < T、それ以外は 0）。極値 (t1,y1),(t2,y2) と T から一意に決める。
    FFが掛かるのは24自由度（board4・5 はスロット0〜2 = DOF 18〜20, 22〜24）。DOF 21・25 とスロット3〜5は [0,0,0,0,0,1.5]。
    送信・更新のルールは GUI_android_tk_FFsetting.py と同じ: 送るのはボタンを押したときだけ（自動では送らない）、
    条件を満たさない自由度があれば全自由度とも送らない、記録中は送らない、受信側がいないボードは確認してから送る、
    「FFを全て0にして送信」は確認してから送る。起動時は FF=0 を前提値（未確認）とし、ボードの配信が途切れて再開したら
    実機が起動したとみなして FF=0 として扱う。他のプログラムがFFを送ったら警告する（その値は使わない）。
・Optuna の結果（optimize_optuna_ff_ROS2.py の <フォルダ>/<フォルダ名>_trials.csv）から極値を読み込む（自由度ごとに選ぶ）
    ① J が最小の試行（COMPLETE かつ source≠past。optuna_ff_common.FFStudy.best() と同じ）
    ② その自由度の J_i が最小の試行（①と同じ集合の中で。自由度ごとに試行番号が違うことがある）
    ③ 試行番号を指定
    ④ その自由度が入っているグループの best（2026-11-20 以降の Optuna の「自由度ごと・グループを指定」。グループ g の評価値
       J_g = グループ内の J_i の平均 が最小の試行。optuna_ff_common.FFStudy.group_bests() と同じ。1グループなら①と同じ）
       「④ グループごとの best を送信」で全自由度に読み込んで送る（最適化していない自由度は FFなし。確認してから送る）。
    ①②④ の対象は COMPLETE で、過去の試行（source='past'）と最後の組み合わせの測定（source='final'）を除く。
    極値は送った float32 の係数から求めた d{DOF}_t1_sent, y1_sent, t2_sent, y2_sent（_results.csv と同じ基準）、
    T は d{DOF}_T（FF 入力時間 T_FF）。列は DOF番号で引く（FF番号ではない）。読み込むと設定欄に入るだけで、送信はしない。
    T・極値を読み込んだままにしている間は、極値から作り直した係数ではなく、その試行で Optuna が実際に送った float32 の係数
    d{DOF}_a..e をそのまま送る（極値から作り直すと、t2 が T に近い試行で波形が最大 1 PWM ほど変わるため）。
・記録: 「● 記録開始」で待機 → 目標値トピックの値が1自由度でも直前と変わった時点を 0 秒として（GUI_android_tk.py と同じ判定）、
    記録時間のあいだ全自由度の 実測POT・ボードの目標値（POTdesired）・PWM・z3・FF（/board{k}_tk_FF_float/pub、
    ファームの outputAdd）・目標値メッセージ を記録する。比較タブの評価で静止位置 P0 と配信番号を Optuna と同じに求めるため、
    目標値が変わる前の HISTORY_S 秒も受信の履歴から取り出して負の時刻で入れる（グラフは 0 秒から描く）。
    ADRC入力 = PWM − FF の FF は、記録開始の時点で送信済みだった設定から内部で作った値。時刻はロボットの時計に合わせて
    配信番号で数える（optimize_optuna_ff_ROS2.py の記録CSV と同じ t = (j − j_r)×0.010 − 0.010。j は PWM の配信番号、
    j_r はその自由度のボードの目標値（POTdesired）が変わった配信。ファームと同じく、目標値が変わるたびに t=0 から掛け直す）。
    PC の受信時刻は WSL2 ではロボットの時計より3〜5%ずれ、実機の試行で内部FFが最大 30 PWM ずれたため使わない。
    FFの有無は記録ごとに、記録中に受信したFFで判定する（1自由度でも 0 でなければ w/、どの自由度も 0 なら w/o）。
・保存: <保存先>/FF(<w|wo>_YYYYMMDD_HHMMSS)/
      FF(<w|wo>_YYYYMMDD_HHMMSS).csv   … 記録（形式は「記録CSV」の説明を参照）
      DOF/DOF<d>/POTと目標値のグラフ.png, 外乱推定値のグラフ.png, PWMとADRC入力のグラフ.png, FF入力のグラフ.png
      （d は DOF番号 0〜25。GUI_android_tk.py の DOF/DOF<n> と同じ番号）
<比較タブ>
・2つの記録CSVを、同じDOFどうしで比較する。グラフは記録と同じ4種類（1つ目 青・水色、2つ目 赤・桃色、目標値は黒の破線）。
  POT のグラフには、評価に使った目標軌道（3次遅れ系）を緑の一点鎖線で重ねる。
  比較グラフの横軸は評価と同じ配信番号の時間軸 t = (j − j0) × 0.010 s（j0 = ボードの目標値が変わった配信。
  PWM・z3・FF も optuna_ff_common.align_ticks と同じ方法で配信番号にそろえる）。PC の受信時刻は WSL2 では
  ロボットの時計より3〜5%ずれるため、比較グラフには使わない（記録タブのグラフは PC の受信時刻のまま）。
・比較する自由度はチェックで選ぶ（既定は全部。DOF 21・25 は評価しない）。選ばなかった自由度は J_i も全体の J も計算しない
  （比較グラフは全自由度を保存する）。
・変化率 = (J_i(2) − J_i(1)) / J_i(1) × 100 [%]（負なら改善 = 2つ目のほうが誤差が小さい）。全体の J も同じ。
  J に占める割合 = J_i / Σ J_i × 100 [%]（J = 平均(J_i) なので J_i / (n·J)）。画面に表と棒グラフで出す。
・評価値の定義は比較タブで選ぶ（既定は ΔP で割る = Optuna と同じ）:
    ΔP で割る   J_i = mean((POT − 目標軌道)/ΔP)²   … 無次元。ステップの大きさが違う自由度を同じ重みで比べる
    ΔP で割らない J_i = mean((POT − 目標軌道)²)     … 単位 count²。誤差の大きさそのもの（ΔP の大きい自由度が J で重くなる）
  目標軌道の当てはめはどちらでも同じ（自由度ごとに当てはめるので、ΔP で割っても割らなくても最適な軌道は変わらない）。
・評価（optimize_optuna_ff_ROS2.py / optuna_ff_common.py と同じ）:
    基準（FF=0）の記録に、ζ=1 の3次遅れ系 (T1 s+1)(s+ωn)² の自由応答を「T までに |y| ≤ ε|ΔP| に入り以後も維持」の拘束付きで
    当てはめて目標軌道を作る。2つのデータには同じ目標軌道を使う（FFありのデータで作り直さない）。
    J_i = mean((POT − 目標軌道)/ΔP)²（評価区間 j − j0 = 0..N_e、N_e = ⌊max(1.5T, E)/0.010⌋、時間軸は 100 Hz 配信の番号）。
・保存: <保存先>/cf-<1つ目のフォルダ名>-<2つ目のフォルダ名>/
      DOF/DOF<d>/POTと目標値の比較.png, 外乱推定値の比較.png, PWMとADRC入力の比較.png, FF入力の比較.png
      analysis/評価値.csv  … 自由度ごとの J_i(1)・J_i(2)・差・小さい方と条件、全体の評価値 J(1)・J(2)・差・小さい方、警告
      analysis/Jに占める割合と変化率.png … J に占める各自由度の割合（2つのデータ）と変化率の棒グラフ（画面と同じ）
      目標軌道.csv         … 配信番号ごとの目標軌道（3次遅れ系）と、評価に使った2つのデータの POT
<対応 Arduino プログラム> ADRC_ROS2_tk_FF_ADRCparamChange-board1〜5（/board{k}_tk_FF_float/pub がないファームでは
  FF のグラフが「no data」になり、FFの有無は送信した設定で判定して警告する）
"""
import csv
import itertools
import json
import math
import multiprocessing
import os
import queue
import re
import subprocess
import sys
import threading
import time
import traceback
import warnings
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime

import numpy as np
import scipy.optimize
from scipy.linalg import expm
from scipy.signal import cont2discrete

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from tkinter import font as tkfont

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray, UInt16MultiArray
    ROS_IMPORT_ERROR = None
except Exception as _exc:       # ROS 2 がない環境でも比較タブは使えるようにする
    rclpy = None
    Node = object
    Float32MultiArray = UInt16MultiArray = None
    ROS_IMPORT_ERROR = str(_exc)


# ==============================================================================
# 自由度・トピック
# ==============================================================================
# 移植元: GUI_android_tk.py:173-183 @105a28e、GUI_android_tk_FFsetting.py:79-107 @105a28e
N_DOF = 26
BOARD_LAYOUT = [            # (board名, 先頭DOF, DOF数)。ボード番号 k は並び順（board1 = 1）
    ('board1', 0, 6),
    ('board2', 6, 6),
    ('board3', 12, 6),
    ('board4', 18, 4),
    ('board5', 22, 4),
]
FF_SLOTS = 6                            # 1ボードのFFスロット数
# ボードごとの、FFが掛かる自由度の数（スロット0から順に先頭DOFから対応する）
FF_BOARD_DOFS = {'board1': 6, 'board2': 6, 'board3': 6, 'board4': 3, 'board5': 3}


def build_ff_dof_map():
    """FF番号 n（0〜23）→ DOF番号。board1-3 → 0..17 / board4 → 18,19,20 / board5 → 22,23,24"""
    dof_map = []
    for board, offset, _count in BOARD_LAYOUT:
        dof_map.extend(range(offset, offset + FF_BOARD_DOFS[board]))
    return dof_map


FF_DOF_MAP = build_ff_dof_map()                             # FF番号 → DOF
FF_INDEX = {d: n for n, d in enumerate(FF_DOF_MAP)}         # DOF → FF番号（FFのないDOFは含まない）
N_FF = len(FF_DOF_MAP)                                      # 24
NO_FF_DOFS = (21, 25)                                       # FFの枠がなく、現在は制御もしていないDOF（評価しない）
DOF_BOARD = {}                                              # DOF → (ボード番号 1〜5, board名, ボード内index, ボードのDOF数)
for _k, (_board, _offset, _count) in enumerate(BOARD_LAYOUT, start=1):
    for _local in range(_count):
        DOF_BOARD[_offset + _local] = (_k, _board, _local, _count)
BOARD_COUNT = {b: c for b, _o, c in BOARD_LAYOUT}           # board名 → DOF数

TOPIC_TARGET = '/board_android_float/sub'       # 目標値（Float32MultiArray, 26要素）。受信だけする
TOPIC_POT = '/board{}_tk/pub'                   # 実測（UInt16MultiArray）: [0..5] = 実測POT, [6..11] = ボードが使っている目標値
TOPIC_PWM = '/board{}_tk_PWM_float/pub'         # PWM = ADRC + FF（Float32MultiArray, 6要素。DOF対応は実測POTと同じ）
TOPIC_Z3 = '/board{}_tk_z3_float/pub'           # ESO外乱推定値 z3（Float32MultiArray, 6要素）
TOPIC_FF_OUT = '/board{}_tk_FF_float/pub'       # 実機が掛けているFF outputAdd（Float32MultiArray, 6要素。並びはPWMと同じ。
                                                #   board4・5 の要素3〜5は0固定。ADRC_ROS2_tk_FF_ADRCparamChange の .ino:69, 693-697）
TOPIC_FF = '/board{}_FFparam_float/sub'         # FF係数（Float32MultiArray, 36要素 = 6スロット × [a,b,c,d,e,T]）。送信する
TOPIC_PARAM = '/board{}_tk_ADRCparam_float/pub' # ADRCパラメータの現在値（50要素。[2+4i+p] = 自由度 i の kp, kd, input_coef, λ0）
SENSOR_QOS_DEPTH = 100

# ------------------------------
# FF波形の設定（GUI_android_tk_FFsetting.py と同じ値）
# ------------------------------
# 移植元: GUI_android_tk_FFsetting.py:121-130 @105a28e
PWM_LIMIT = 255.0           # 実機PWMの絶対上限
FF_T_DEFAULT = 1.5          # FF終了時刻 T の初期値 [s]（使わないスロットにも送る値）
FF_T_RANGE = (0.1, 10.0)    # T の範囲 [s]
Y_LIMIT_DEFAULT = (-50.0, 50.0)     # 極値 y の下限・上限の初期値 [PWM]
DEFAULT_FF_PEAK = 10.0      # 初めてFFを与えるときの極値の大きさ [PWM]
EXTREMA_IMAG_TOL = 1e-6     # f'(t)=0 の根を実根とみなす虚部の許容値（τ = t/T の単位）
EXTREMA_LEAD_TOL = 1e-9     # f'(τ) の最高次の係数がこれ未満（最大係数との比）なら次数を下げる
EXTREMA_SIMPLE_TOL = 1e-7   # |f''(τi)| / max(|y1|,|y2|) がこれ以下の極値は重根とみなす

# ------------------------------
# 記録
# ------------------------------
RECORD_DURATION_DEFAULT = 10.0  # 記録時間の初期値 [s]
RECORD_DURATION_MIN = 5.0       # 記録時間の範囲 [s]（GUI_android_tk_FFsetting.py と同じ。評価区間は max(1.5T, E) なので
RECORD_DURATION_MAX = 600.0     #   比較で使うなら、それより長くする）
HISTORY_S = 1.2                 # 記録に含める、目標値が変わる前の受信 [s]（optuna_ff_common.HISTORY_S と同じ）
RING_SEC = 3.0                  # 変化前の履歴として常に覚えておく受信の長さ [s]（optuna_ff_common.RING_SEC と同じ）
RING_LEN = int(RING_SEC * 150)  # 1トピックの履歴の数（各100Hz。余裕をみて 150Hz 分）
BOARD_RESTART_GAP = 3.0         # ボードの配信がこの時間途切れてから再開したら、実機が起動したとみなす [s]
STALL_WARN_S = 0.5              # 記録中にボードの受信がこの時間途切れたら警告する [s]
GAP_LINE_S = 0.05               # 受信間隔がこれを超えたところはグラフの線をつながない（欠測）[s]
FF_ON_EPS = 1e-6                # 受信したFFの絶対値がこれを超えたら「FFあり」とみなす [PWM]
POSTURE_TOL = 0.5               # 目標値の差がこれ以下なら同じ目標値とみなす（GUI_android_tk_FFsetting.POSTURE_TOL と同じ）

# ------------------------------
# 評価（optuna_ff_common.py と同じ値）
# ------------------------------
# 移植元: optuna_ff_common.py:95-108, 463-489 @105a28e
TICK_DT = 0.010             # 時間軸の刻み [s]（ROS2 の配信 100 Hz。制御周期は使わない）
EVAL_FACTOR = 1.5           # 測定・評価区間 = max(EVAL_FACTOR × T, E)
EVAL_MIN_S = 5.0            # 評価時間の固定値 E の既定 [s]
EPS_DEFAULT = 5.0           # 完了の許容 ε の既定 [% of ΔP]
REST_WINDOW_S = 0.5         # 静止位置 P0 を求める、切り替え直前の区間 [s]
MARGIN_TICKS = 15           # 評価区間のあとに余分に切り出す配信数
STEP_WARN = 10.0            # |ΔP| がこれ未満なら同定が不安定になりうると警告する [count]
XI_CAP = math.log(10.0)     # 目標軌道の完了を T/10 より早くはしない（ξ ≤ ln 10）
WN_MAX = 100.0              # 目標軌道の ωn の上限 [rad/s]
DEFAULT_ID_BOUNDS = {'tgt_T1': (1e-3, 10.0), 'tgt_wn': (0.1, 100.0)}

# ------------------------------
# ファイル・グラフ
# ------------------------------
SETTINGS_FILE = os.path.expanduser('~/.armrobot_gui_cfFF.json')
RECORD_FORMAT = 'GUI_android_tk_cfFF record v1'
CSV_ENCODING = 'utf-8-sig'      # BOM 付き UTF-8（Excel で開いても日本語が文字化けしないように）
GRAPH_KINDS = ('pot', 'z3', 'pwm', 'ff')
RECORD_GRAPH_FILES = {'pot': 'POTと目標値のグラフ.png', 'z3': '外乱推定値のグラフ.png',
                      'pwm': 'PWMとADRC入力のグラフ.png', 'ff': 'FF入力のグラフ.png'}
COMPARE_GRAPH_FILES = {'pot': 'POTと目標値の比較.png', 'z3': '外乱推定値の比較.png',
                       'pwm': 'PWMとADRC入力の比較.png', 'ff': 'FF入力の比較.png'}
ANALYSIS_DIR = 'analysis'
ANALYSIS_CSV = '評価値.csv'
TARGET_CSV = '目標軌道.csv'
SHARE_PNG = 'Jに占める割合と変化率.png'
# グラフの文字は英語（matplotlib に日本語フォントがない環境でも文字化けしないように。GUI_android_tk.py と同じ）
GRAPH_TITLES = {'pot': 'POT and reference', 'z3': 'Disturbance estimate z_3', 'pwm': 'PWM and ADRC input',
                'ff': 'FF input (received from the robot)'}
GRAPH_YLABELS = {'pot': 'POT', 'z3': 'z_3', 'pwm': 'PWM', 'ff': 'FF [PWM]'}
DATA_COLORS = (('blue', 'deepskyblue'), ('red', 'hotpink'))     # (POT・Disturbance・PWM・FF, ADRC)。1つ目, 2つ目
REF_COLOR = 'black'
TARGET_COLOR, TARGET_STYLE, TARGET_LABEL = 'green', '-.', "Target (3rd-order lag)"     # 目標軌道（緑の一点鎖線）
RECORD_XLABEL = "Time [s]"
COMPARE_XLABEL = "Time [s]   (100 Hz publish count x 0.010 s;  t = 0: board target changed)"


# ==============================================================================
# 日本語フォント・ウィンドウの大きさ
# ==============================================================================
# 移植元: GUI_android_tk.py:71-129 @105a28e
JP_FONT_CANDIDATES = [
    "Noto Sans CJK JP", "Noto Sans JP", "Source Han Sans JP",
    "IPAexGothic", "IPAPGothic", "IPAGothic",
    "TakaoPGothic", "TakaoGothic", "VL PGothic",
    "Yu Gothic UI", "Meiryo UI", "Meiryo", "MS UI Gothic", "MS Gothic",
    "BIZ UDPゴシック", "BIZ UDゴシック",
]
UI_FONT = "TkDefaultFont"   # setup_japanese_font() で実際のフォント名に置き換わる

# ※ WSLg ではタイトルバー（ウィンドウ名・messagebox/filedialog の title）は日本語が文字化けするので英字にする。
WINDOW_TITLE = "FF record / compare GUI (ADRC + quintic FF)"
TITLE_SEND = "Send FF parameters"
TITLE_SEND_ZERO = "Send zero FF"
TITLE_RECORD = "Record"
TITLE_OPTUNA = "Load Optuna result"
TITLE_FF = "FF setting"
TITLE_COMPARE = "Compare"
TITLE_SETTINGS = "Settings"


def setup_japanese_font(root):
    """日本語が豆腐(□)にならないよう、日本語グリフを持つフォントを既定にする"""
    global UI_FONT
    try:
        available = set(tkfont.families(root))
    except tk.TclError:
        return
    family = next((name for name in JP_FONT_CANDIDATES if name in available), None)
    if family is None:
        print("警告: 日本語フォントが見つかりませんでした。GUIの日本語が□で表示されます。\n"
              "      Ubuntu/WSL の場合は sudo apt install fonts-noto-cjk などで入れてください", file=sys.stderr)
        return
    UI_FONT = family
    for named in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont",
                  "TkCaptionFont", "TkSmallCaptionFont", "TkIconFont", "TkTooltipFont"):
        try:
            tkfont.nametofont(named, root=root).configure(family=family)
        except tk.TclError:
            pass
    try:
        style = ttk.Style(root)
        style.configure(".", font=(family, 10))
        style.configure("TNotebook.Tab", font=(family, 10))
        style.configure("Treeview.Heading", font=(family, 9))
    except tk.TclError:
        pass


# 移植元: GUI_android_tk.py:696-756 @105a28e
WINDOW_SCREEN_RATIO = 0.85      # 起動時のウィンドウの大きさ（表示するモニターの幅・高さに対する割合）
FALLBACK_MONITOR = (1920, 1080) # モニターの情報が取れないときに想定するモニターの大きさの上限


def parse_xrandr_monitors(text):
    """`xrandr --listmonitors` の出力 → [(x, y, 幅, 高さ, primaryか), ...]"""
    monitors = []
    for line in text.splitlines():
        m = re.match(r"\s*\d+:\s+\+?(\*?)\S+\s+(\d+)/\d+x(\d+)/\d+\+(-?\d+)\+(-?\d+)", line)
        if m:
            primary, w, h, x, y = m.groups()
            monitors.append((int(x), int(y), int(w), int(h), primary == "*"))
    return monitors


def list_monitors():
    try:
        out = subprocess.run(["xrandr", "--listmonitors"], capture_output=True, text=True, timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return parse_xrandr_monitors(out)


def initial_monitor(root, monitors):
    """ウィンドウを出すモニター: マウスポインタのあるモニター → primary → 最初のモニター"""
    if monitors:
        px, py = root.winfo_pointerxy()
        for mon in monitors:
            mx, my, mw, mh, _ = mon
            if mx <= px < mx + mw and my <= py < my + mh:
                return mon
        for mon in monitors:
            if mon[4]:
                return mon
        return monitors[0]
    return (0, 0, min(root.winfo_screenwidth(), FALLBACK_MONITOR[0]),
            min(root.winfo_screenheight(), FALLBACK_MONITOR[1]), True)


def window_geometry_for(monitor):
    mx, my, mw, mh, _ = monitor
    w, h = int(mw * WINDOW_SCREEN_RATIO), int(mh * WINDOW_SCREEN_RATIO)
    return f"{w}x{h}+{mx + (mw - w) // 2}+{my + (mh - h) // 2}"


def to_float32_tuple(values):
    """Float32MultiArray で送受信したときと同じ値（float32 に丸めた値）のタプル"""
    return tuple(np.asarray(values, dtype=np.float32).tolist())


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


# ==============================================================================
# FF波形（5次関数）
# ==============================================================================
# 移植元: GUI_android_tk_FFsetting.py:162-320 @105a28e
def ff_poly(coeffs, t):
    """f(t) = a t⁵ + b t⁴ + c t³ + d t² + e t（区間の切り出しはしない）"""
    a, b, c, d, e = [float(v) for v in coeffs[:5]]
    t = np.asarray(t, dtype=float)
    return ((((a * t + b) * t + c) * t + d) * t + e) * t


def _value_rows(tau):
    return np.stack([tau ** 5, tau ** 4, tau ** 3, tau ** 2, tau], axis=-1)


def _slope_rows(tau):
    return np.stack([5 * tau ** 4, 4 * tau ** 3, 3 * tau ** 2, 2 * tau, np.ones_like(tau)], axis=-1)


def quintic_from_extrema(T, t1, y1, t2, y2):
    """極値 (t1,y1),(t2,y2) と T から f の係数 [a,b,c,d,e] を求める（配列で渡せばまとめて解く）

    f(T)=0, f'(t1)=f'(t2)=0, f(t1)=y1, f(t2)=y2 の5式で一意に決まる。桁をそろえるため τ = t/T の空間で解いて戻す。
    戻り値: (係数 (n,5), τ空間の係数 (n,5), 解けたか (n,))。0 < t1 < t2 < T でない行は NaN
    """
    T, t1, y1, t2, y2 = np.broadcast_arrays(*[np.atleast_1d(np.asarray(v, dtype=float)) for v in (T, t1, y1, t2, y2)])
    with np.errstate(divide='ignore', invalid='ignore'):
        tau1, tau2 = t1 / T, t2 / T
    ok = (np.isfinite(T) & (T > 0) & np.isfinite(tau1) & np.isfinite(tau2) & np.isfinite(y1) & np.isfinite(y2)
          & (tau1 > 0) & (tau2 > tau1) & (tau2 < 1))
    ta = np.where(ok, tau1, 1.0 / 3.0)
    tb = np.where(ok, tau2, 2.0 / 3.0)
    A = np.stack([_value_rows(np.ones_like(ta)), _slope_rows(ta), _slope_rows(tb), _value_rows(ta), _value_rows(tb)],
                 axis=-2)
    zero = np.zeros_like(ta)
    rhs = np.stack([zero, zero, zero, np.where(ok, y1, 0.0), np.where(ok, y2, 0.0)], axis=-1)
    G = np.linalg.solve(A, rhs[..., None])[..., 0]
    G[~ok] = np.nan
    with np.errstate(invalid='ignore', over='ignore'):
        coeffs = G / (T[:, None] ** np.array([5, 4, 3, 2, 1]))
    return coeffs, G, ok


def slope_roots(G):
    """τ空間の係数 G (n,5) について、g'(τ)（4次以下）の根を (n,4) の複素数で返す（足りない分は NaN）"""
    D = np.asarray(G, dtype=float) * np.array([5.0, 4.0, 3.0, 2.0, 1.0])
    n = len(D)
    roots = np.full((n, 4), np.nan + 0j)
    scale = np.abs(D).max(axis=1) if n else np.zeros(0)
    good = np.isfinite(D).all(axis=1) & (scale > 0)
    Dn = np.zeros_like(D)
    Dn[good] = D[good] / scale[good, None]
    big = np.abs(Dn) > EXTREMA_LEAD_TOL
    lead = np.where(big.any(axis=1), big.argmax(axis=1), 5)
    for m in (4, 3, 2, 1):
        rows = np.flatnonzero(good & (lead == 4 - m))
        if rows.size == 0:
            continue
        c = Dn[rows, 4 - m:]
        monic = c[:, 1:] / c[:, :1]
        C = np.zeros((rows.size, m, m))
        C[:, 0, :] = -monic
        for i in range(1, m):
            C[:, i, i - 1] = 1.0
        roots[rows, :m] = np.linalg.eigvals(C)
    return roots


def _simple_at(G, tau, amp):
    """τ で |g''(τ)| が十分大きい（重根でない）か"""
    g5, g4, g3, g2 = G[:, 0], G[:, 1], G[:, 2], G[:, 3]
    ddg = 20 * g5 * tau ** 3 + 12 * g4 * tau ** 2 + 6 * g3 * tau + 2 * g2
    return np.abs(ddg) > EXTREMA_SIMPLE_TOL * amp


class FFCheck:
    """1自由度のFF設定の判定結果"""

    def __init__(self):
        self.ok = False
        self.coeffs = None          # [a,b,c,d,e]（計算できなければ None）
        self.errors = []            # 満たしていない条件（表示用の文）
        self.extrema_t = []         # (0,T) 内にある f'(t)=0 の実根 [s]


def check_ff(T, t1, y1, t2, y2, ylo, yhi):
    """FF設定 (T, t1, y1, t2, y2) と y の上下限を判定し、満たしていない条件を文で返す

    条件: 0 < t1 < t2 < T、y1·y2 < 0、下限 ≤ y1,y2 ≤ 上限、(0,T) 内の f'(t)=0 の実根がちょうど2個（= t1, t2）で単純根
    """
    res = FFCheck()
    vals = (T, t1, y1, t2, y2, ylo, yhi)
    if not all(_num(v) for v in vals):
        res.errors.append("数値でない値があります")
        return res
    if not (FF_T_RANGE[0] <= T <= FF_T_RANGE[1]):
        res.errors.append(f"FF終了時刻 T は {FF_T_RANGE[0]:g}〜{FF_T_RANGE[1]:g} s にしてください")
        return res
    if not (ylo < 0 < yhi):
        res.errors.append("y の上下限は 下限 < 0 < 上限 にしてください（極値は山と谷で符号が逆になるため）")
    if not (0 < t1 < t2 < T):
        res.errors.append("0 < t1 < t2 < T にしてください（t1 が第1極値、t2 が第2極値の時刻）")
    if not (y1 * y2 < 0):
        res.errors.append("y1 と y2 は符号を逆にしてください（山→谷 または 谷→山。0 は不可）")
    for name, y in (("y1", y1), ("y2", y2)):
        if y > yhi:
            res.errors.append(f"{name} = {y:g} が上限 {yhi:g} を超えています")
        elif y < ylo:
            res.errors.append(f"{name} = {y:g} が下限 {ylo:g} を下回っています")

    coeffs, G, solved = quintic_from_extrema(T, t1, y1, t2, y2)
    if solved[0] and np.isfinite(coeffs[0]).all():
        res.coeffs = coeffs[0]
        r = slope_roots(G)[0]
        real = np.sort(r.real[(np.abs(r.imag) <= EXTREMA_IMAG_TOL) & (r.real > 1e-9) & (r.real < 1 - 1e-9)])
        res.extrema_t = [float(v) * T for v in real]
        if not res.errors:
            amp = max(abs(y1), abs(y2))
            n_ext = len(real)
            simple = bool(_simple_at(G, np.array([t1 / T]), amp)[0] and _simple_at(G, np.array([t2 / T]), amp)[0])
            if n_ext != 2 or not simple:
                listing = ", ".join(f"{v:.3f}" for v in res.extrema_t)
                res.errors.append(
                    f"(0,T) 内の極値が{'重根を含めて ' if not simple else ''}{n_ext} 個になります（{listing} s）。"
                    "ちょうど2個（t1, t2）である必要があります")
    res.ok = not res.errors
    return res


def zero_ff_table():
    """FFなし（全係数0）の24自由度分の [a,b,c,d,e,T]"""
    table = np.zeros((N_FF, 6))
    table[:, 5] = FF_T_DEFAULT
    return table


def format_formula(coeffs, T):
    """f(t) の数式（表示用）"""
    if coeffs is None:
        return "f(t) = （計算できません）"
    if not np.any(np.asarray(coeffs) != 0):
        return "f(t) = 0（FFなし）"
    text = "f(t) ="
    for i, (v, power) in enumerate(zip(coeffs, ("t^5", "t^4", "t^3", "t^2", "t"))):
        sign = "−" if v < 0 else ("" if i == 0 else "+")
        text += f" {sign}{'' if i == 0 else ' '}{abs(v):.5g} {power}" if sign else f" {abs(v):.5g} {power}"
    return text + f"    (0 ≤ t < T={T:g} s、それ以外は 0)"


def ff_internal(ff6, restarts, t):
    """記録の時刻 t のFF入力（設定した係数から内部で作る値。PC の受信時刻で数える。/tk がなく配信番号を振れないときだけ使う）

    restarts = その自由度の目標値が変わった時刻（0 秒以降）。最後に変わってから T 秒以上たっていれば 0。
    # 変更: GUI_android_tk_FFsetting.ff_series（目標値メッセージを受信するたびに t=0）を、ファームと同じく
    #       「その自由度の目標値が変わったときだけ t=0」にした（.ino の POT_desired_change :1268-1283）。
    #       記録開始より前（t < 0）は掛かっているFFが分からないので NaN。
    """
    t = np.asarray(t, dtype=float)
    out = np.where(t < 0, np.nan, 0.0)
    coeffs = np.asarray(ff6[:5], dtype=float)
    T = float(ff6[5])
    restarts = np.asarray(restarts, dtype=float)
    if restarts.size == 0 or not np.any(coeffs != 0):
        return out
    idx = np.searchsorted(restarts, t, side='right') - 1
    has = (idx >= 0) & (t >= 0)
    tau = np.where(has, t - restarts[np.clip(idx, 0, None)], -1.0)
    on = has & (tau >= 0) & (tau < T)
    out[on] = ff_poly(coeffs, tau[on])
    return out


def ff_internal_ticks(ff6, restart_ticks, j, t_pc):
    """PWM の配信番号 j の FF 入力（設定した係数から内部で作る値。時刻は配信番号で数える）

    FF の時刻 = (j − j_r) × 0.010 − 0.010（optuna_ff_common.ff_times_for_csv と同じ。PWM は POT より1周回古いので
    10 ms 前の FF）。j_r は j 以前で最後にその自由度の目標値が変わった配信。0 ≤ 時刻 < T で5次関数、それ以外は 0
    （ファームの AddInput、optuna_ff_common.ff_eval と同じ）。記録開始より前（PC の時刻 t_pc < 0）は分からないので NaN。
    """
    j = np.asarray(j, dtype=float)
    t_pc = np.asarray(t_pc, dtype=float)
    out = np.where(t_pc < 0, np.nan, 0.0)
    coeffs = np.asarray(ff6[:5], dtype=float)
    T = float(ff6[5])
    R = np.asarray(restart_ticks, dtype=float)
    if R.size == 0 or not np.any(coeffs != 0):
        return out
    idx = np.searchsorted(R, j, side='right') - 1
    has = (idx >= 0) & (t_pc >= 0)
    tau = np.where(has, (j - R[np.clip(idx, 0, None)]) * TICK_DT - TICK_DT, -1.0)
    on = has & (tau >= 0) & (tau < T)
    out[on] = ff_poly(coeffs, tau[on])
    return out


def echo_restart_ticks(t_tk, j_tk, echo):
    """その自由度のボードの目標値（POTdesired）が前の受信と変わった配信番号（目標値を受信する直前の 0.05 s 以後）。
    ファームは目標値が変わった周回で FF を t=0 から掛け直し、その周回の配信からボードの目標値が新しい値になる
    （optuna_ff_common.extract_series の j0 と同じ考え方）"""
    e = np.asarray(echo, dtype=float)
    ok = np.isfinite(e)
    ch = np.flatnonzero(ok[1:] & ok[:-1] & (e[1:] != e[:-1])) + 1
    ch = ch[np.asarray(t_tk)[ch] >= -0.05]
    return np.asarray(j_tk)[ch]


def target_restarts(cmd_t, cmd_v, init_target, dof):
    """その自由度の目標値が変わった時刻（0 秒以降の目標値メッセージのうち、直前の値と違うもの）"""
    prev = init_target[dof] if init_target is not None else None
    out = []
    for t, v in zip(cmd_t, cmd_v):
        if t < 0:
            continue
        val = float(v[dof]) if dof < len(v) else float('nan')
        if not math.isfinite(val):
            continue
        if prev is None or not _num(prev) or val != float(prev):
            out.append(float(t))
        prev = val
    return out


# ==============================================================================
# 目標軌道（ζ=1 の3次遅れ系）と累積誤差 J_i（optuna_ff_common.py から移植）
# ==============================================================================
# 移植元: optuna_ff_common.py:137-166 @105a28e
def system_coeffs(T1, zeta, wn):
    """(T1 s + 1)(s² + 2ζωn s + ωn²) を s³ + a2 s² + a1 s + a0 に展開して返す"""
    a2 = (2.0 * zeta * wn * T1 + 1.0) / T1
    a1 = (wn ** 2 * T1 + 2.0 * zeta * wn) / T1
    a0 = wn ** 2 / T1
    return a2, a1, a0


def discretize(a2, a1, a0, b0, L):
    """可制御正準形 ẋ = A x + B u（x = [位置, 速度, 加速度]）を間隔 L で ZOH 離散化する"""
    A = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [-a0, -a1, -a2]])
    B = np.array([[0.0], [0.0], [float(b0)]])
    Ad, Bd, _, _, _ = cont2discrete((A, B, np.zeros((1, 3)), np.zeros((1, 1))), float(L), method='zoh')
    return Ad, Bd.ravel()


def target_coeffs(T1, wn):
    """(T1 s + 1)(s + ωn)² の係数（system_coeffs の ζ=1）"""
    return system_coeffs(T1, 1.0, wn)


def target_states(T1, wn, y0, L, n):
    """y 系（目標値が原点）での自由応答の状態列 (n+1, 3) = [位置, 速度, 加速度]。初期状態 [y0, 0, 0]"""
    Ad, _ = discretize(*target_coeffs(T1, wn), 0.0, L)
    X = np.zeros((n + 1, 3))
    X[0] = [y0, 0.0, 0.0]
    for k in range(n):
        X[k + 1] = Ad @ X[k]
    return X


# 移植元: optuna_ff_common.py:169-197 @105a28e
def fit_target(y_meas, valid, y0, L, bounds, prev=None):
    """実測 y = P − Pf（k = 0..N）に ζ=1 の3次遅れ系の自由応答を最小二乗で当てはめる。戻り値 ((T1, wn), rms)"""
    y_meas = np.asarray(y_meas, dtype=float)
    n = len(y_meas) - 1
    m = np.asarray(valid, dtype=bool) & np.isfinite(y_meas)
    lo = [np.log(bounds['tgt_T1'][0]), np.log(bounds['tgt_wn'][0])]
    hi = [np.log(bounds['tgt_T1'][1]), np.log(bounds['tgt_wn'][1])]

    def res(q):
        with np.errstate(all='ignore'):
            X = target_states(np.exp(q[0]), np.exp(q[1]), y0, L, n)
            r = (X[:, 0] - y_meas)[m]
        return np.nan_to_num(r, nan=1e6, posinf=1e6, neginf=-1e6)

    starts = [[np.log(a), np.log(b)] for a in (0.03, 0.15, 0.5) for b in (3.0, 8.0, 20.0)]
    if prev is not None and np.all(np.isfinite(prev)) and min(prev) > 0:
        starts.insert(0, [np.log(prev[0]), np.log(prev[1])])
    best = None
    for q0 in starts:
        try:
            r = scipy.optimize.least_squares(res, np.clip(q0, lo, hi), bounds=(lo, hi), method='trf', x_scale='jac')
        except Exception:
            continue
        if best is None or r.cost < best.cost:
            best = r
    T1, wn = float(np.exp(best.x[0])), float(np.exp(best.x[1]))
    rms = float(np.sqrt(np.mean(res(best.x) ** 2))) if m.any() else float('nan')
    return (T1, wn), rms


# 移植元: optuna_ff_common.py:200-213 @105a28e
def normalized_error(y_meas, y_tgt, dP, valid):
    """J_i = (1/N_valid) Σ_k (POT − Target)² / ΔP²（T・周回間隔・欠測点数に依存しない無次元量）"""
    if dP == 0:
        return float('nan')
    m = np.asarray(valid, dtype=bool) & np.isfinite(y_meas) & np.isfinite(y_tgt)
    if not m.any():
        return float('nan')
    d = (np.asarray(y_meas)[m] - np.asarray(y_tgt)[m]) / float(dP)
    return float(np.mean(d * d))


def squared_error(y_meas, y_tgt, valid):
    """ΔP で割らない評価値 J_i = (1/N_valid) Σ_k (POT − Target)² [count²]（normalized_error に ΔP² を掛けたもの）"""
    m = np.asarray(valid, dtype=bool) & np.isfinite(y_meas) & np.isfinite(y_tgt)
    if not m.any():
        return float('nan')
    d = np.asarray(y_meas)[m] - np.asarray(y_tgt)[m]
    return float(np.mean(d * d))


NORM_TEXT = {True: "ΔPで割る: J_i = mean((POT − 目標軌道)/ΔP)²（無次元。optimize_optuna_ff_ROS2.py と同じ）",
             False: "ΔPで割らない: J_i = mean((POT − 目標軌道)²) [count²]"}
NORM_TEXT_EN = {True: "J_i = mean(((POT - target)/dP)^2)", False: "J_i = mean((POT - target)^2) [count^2]"}


def error_metric(y_meas, y_tgt, dP, valid, normalize=True):
    """評価値 J_i。normalize=True は ΔP で割る（Optuna と同じ）、False は割らない"""
    return normalized_error(y_meas, y_tgt, dP, valid) if normalize else squared_error(y_meas, y_tgt, valid)


def rms_percent(J_i):
    """J_i を ΔP に対する RMS 比 [%] にした表示用の値"""
    return float(np.sqrt(J_i) * 100.0) if np.isfinite(J_i) and J_i >= 0 else float('nan')


# 移植元: optuna_ff_common.py:482-489 @105a28e
def eval_time(T, t_min=EVAL_MIN_S):
    """測定・評価区間の長さ [s] = max(1.5T, t_min)。T は動作完了時間、t_min は評価時間の固定値（既定 5 s）"""
    return max(EVAL_FACTOR * float(T), float(t_min))


def n_eval(T, t_min=EVAL_MIN_S):
    """評価区間の最終の配信番号 N_e = ⌊max(1.5T, t_min)/0.010⌋（j − j0 = 0..N_e の N_e+1 点）"""
    return int(np.floor(eval_time(T, t_min) / TICK_DT + 1e-9))


# 移植元: optuna_ff_common.py:870-917 @105a28e（phi_at は使わないので移植しない）
def _companion(T1, wn):
    a2, a1, a0 = system_coeffs(T1, 1.0, wn)
    return np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [-a0, -a1, -a2]])


def completion_theta(kappa, eps_eff):
    """無次元の完了時刻 θ(κ)：φ(θ; T1=κ, ωn=1) = ε_eff の唯一の根（φ は単調減少）。ε_eff ≥ 1 なら 0"""
    if eps_eff >= 1.0:
        return 0.0
    if eps_eff <= 0.0:
        raise ValueError("ε_eff は正の値にしてください")
    A = _companion(float(kappa), 1.0)

    def f(lt):
        return float((expm(A * math.exp(lt)) @ np.array([1.0, 0.0, 0.0]))[0]) - eps_eff

    lo, hi = 1e-6, 10.0
    while f(math.log(hi)) >= 0.0:
        hi *= 2.0
        if hi > 1e7:
            raise ValueError(f"κ = {kappa:g}: 完了時刻が 1e7 を超えます")
    return math.exp(scipy.optimize.brentq(f, math.log(lo), math.log(hi), xtol=1e-13, rtol=1e-12))


def completion_time(T1, wn, eps_eff):
    return completion_theta(T1 * wn, eps_eff) / wn


def measured_completion(y, valid, band, dt=TICK_DT):
    """実測の完了時刻：それ以後の有効な点がすべて |y| ≤ band に入る最初の時刻（評価区間の最後まで入らなければ nan）"""
    y = np.asarray(y, dtype=float)
    m = np.asarray(valid, dtype=bool) & np.isfinite(y)
    idx = np.flatnonzero(m)
    if len(idx) == 0:
        return float('nan')
    out = idx[np.abs(y[idx]) > band]
    if len(out) == 0:
        return 0.0
    if out[-1] == idx[-1]:
        return float('nan')
    nxt = idx[idx > out[-1]][0]
    return float(nxt * dt)


# 移植元: optuna_ff_common.py:919-964 @105a28e
@dataclass
class TargetSpec:
    """1自由度の目標軌道（固定）。seq() が評価に使う数列（POT の値）を返す"""
    dof: int
    Pi: float
    Pf: float
    T: float
    eps: float
    N_e: int
    dt: float
    y0_ref: float
    P0_ref: float
    eps_eff: float
    T1: float
    wn: float
    kappa: float
    xi: float
    theta: float
    active: bool
    method: str
    cost: float
    free_T1: float
    free_wn: float
    free_t_eps: float
    t_eps_meas: list
    n_runs: int
    warnings: list = field(default_factory=list)

    @property
    def dP(self):
        return self.Pf - self.Pi

    def seq(self):
        """目標軌道の POT の値（j − j0 = 0..N_e）"""
        return target_states(self.T1, self.wn, self.y0_ref, self.dt, self.N_e)[:, 0] + self.Pf


# 移植元: optuna_ff_common.py:967-988 @105a28e
def _fit_free_multi(ys, ms, y0, dt, bounds):
    """複数の基準試行への自由フィット（fit_target と同じ開始点・上下限・手法で、残差を縦に並べる）"""
    n = len(ys[0]) - 1
    lo = [np.log(bounds['tgt_T1'][0]), np.log(bounds['tgt_wn'][0])]
    hi = [np.log(bounds['tgt_T1'][1]), np.log(bounds['tgt_wn'][1])]

    def res(q):
        with np.errstate(all='ignore'):
            X = target_states(np.exp(q[0]), np.exp(q[1]), y0, dt, n)
            r = np.concatenate([(X[:, 0] - y)[m] for y, m in zip(ys, ms)])
        return np.nan_to_num(r, nan=1e6, posinf=1e6, neginf=-1e6)

    starts = [[np.log(a), np.log(b)] for a in (0.03, 0.15, 0.5) for b in (3.0, 8.0, 20.0)]
    best = None
    for q0 in starts:
        try:
            r = scipy.optimize.least_squares(res, np.clip(q0, lo, hi), bounds=(lo, hi), method='trf', x_scale='jac')
        except Exception:
            continue
        if best is None or r.cost < best.cost:
            best = r
    return (float(np.exp(best.x[0])), float(np.exp(best.x[1])))


# 移植元: optuna_ff_common.py:991-1098 @105a28e
def fit_target_constrained(dof, ys, valids, P0s, Pi, Pf, T, eps, bounds=None, dt=TICK_DT, log=None):
    """基準試行（y = P − Pf, j − j0 = 0..N_e）に、完了拘束 φ(T) ≤ ε_eff 付きで ζ=1 の3次遅れ系を当てはめる"""
    bounds = bounds or DEFAULT_ID_BOUNDS
    ys = [np.asarray(y, dtype=float) for y in ys]
    ms = [np.asarray(v, dtype=bool) & np.isfinite(y) for y, v in zip(ys, valids)]
    N = len(ys[0]) - 1
    dP = float(Pf - Pi)
    P0_ref = float(np.mean(P0s))
    y0 = P0_ref - float(Pf)
    if dP == 0.0 or y0 == 0.0:
        raise ValueError(f"DOF {dof}: ΔP または y0 が 0 なので目標軌道を作れません")
    eps_eff = float(eps) * abs(dP) / abs(y0)
    n_res = int(sum(m.sum() for m in ms))
    warnings_ = []

    def resid(T1, wn):
        with np.errstate(all='ignore'):
            phi = target_states(T1, wn, 1.0, dt, N)[:, 0]
            r = np.concatenate([(y0 * phi - y)[m] for y, m in zip(ys, ms)]) / abs(dP)
        return np.nan_to_num(r, nan=1e3, posinf=1e3, neginf=-1e3)

    def cost(T1, wn):
        r = resid(T1, wn)
        return float(np.mean(r * r)) if len(r) else float('nan')

    def in_bounds(T1, wn):
        return (bounds['tgt_T1'][0] <= T1 <= bounds['tgt_T1'][1]) and (bounds['tgt_wn'][0] <= wn <= WN_MAX)

    # 1. 自由フィット（基準試行が1回なら fit_target そのもの。前回値は使わない）
    if len(ys) == 1:
        (T1f, wnf), _rms = fit_target(ys[0], ms[0], y0, dt, bounds, prev=None)
    else:
        T1f, wnf = _fit_free_multi(ys, ms, y0, dt, bounds)
    t_meas = [measured_completion(y, m, eps * abs(dP), dt) for y, m in zip(ys, ms)]
    common = dict(dof=dof, Pi=float(Pi), Pf=float(Pf), T=float(T), eps=float(eps), N_e=N, dt=dt, y0_ref=y0,
                  P0_ref=P0_ref, eps_eff=eps_eff, free_T1=T1f, free_wn=wnf, t_eps_meas=t_meas, n_runs=len(ys))
    if eps_eff >= 1.0:
        warnings_.append(f"始点がすでに許容帯の中（ε_eff = {eps_eff:.3g} ≥ 1）なので拘束なし（自由フィット）にしました")
        return TargetSpec(T1=T1f, wn=wnf, kappa=T1f * wnf, xi=float('nan'), theta=0.0, active=False, method='free',
                          cost=cost(T1f, wnf), free_t_eps=0.0, warnings=warnings_, **common)
    t_free = completion_time(T1f, wnf, eps_eff)
    if t_free <= T:
        warnings_.append(f"基準の応答（自由フィット）がすでに T 以内に完了しています（{t_free:.3f} s ≤ T = {T:g} s）。"
                         f"拘束は効かず、FF で改善できる余地は小さいと考えられます")
        th = completion_theta(T1f * wnf, eps_eff)
        return TargetSpec(T1=T1f, wn=wnf, kappa=T1f * wnf, xi=float(np.log(wnf * T / th)) if th > 0 else float('nan'),
                          theta=th, active=False, method='free', cost=cost(T1f, wnf), free_t_eps=t_free,
                          warnings=warnings_, **common)

    cands = []

    # 3. 境界 ξ = 0（ωn = θ(κ)/T）上の1次元探索：log κ の121点格子 → 有界 Brent 法
    def bnd_cost(lk):
        kappa = math.exp(lk)
        th = completion_theta(kappa, eps_eff)
        wn = th / T
        T1 = kappa / wn
        return cost(T1, wn) if in_bounds(T1, wn) else float('inf')

    grid = np.linspace(math.log(1e-3), math.log(1e3), 121)
    gc = np.array([bnd_cost(lk) for lk in grid])
    if np.isfinite(gc).any():
        i = int(np.nanargmin(gc))
        a, b = grid[max(i - 1, 0)], grid[min(i + 1, len(grid) - 1)]
        r = scipy.optimize.minimize_scalar(bnd_cost, bounds=(a, b), method='bounded', options={'xatol': 1e-7})
        lk = float(r.x) if np.isfinite(r.fun) and r.fun <= gc[i] else float(grid[i])
        kappa = math.exp(lk)
        th = completion_theta(kappa, eps_eff)
        cands.append((bnd_cost(lk), kappa, 0.0, th, 'boundary'))

    # 4. (log κ, σ) の箱付き least_squares（ξ = σ·ξ_max(κ)、ξ_max = min(ln 10, ln(ωn_max T/θ))）を多点から
    def unpack(q):
        kappa = math.exp(q[0])
        th = completion_theta(kappa, eps_eff)
        xmax = min(XI_CAP, math.log(WN_MAX * T / th))
        if xmax < 0:
            return None
        xi = float(q[1]) * xmax
        wn = th / T * math.exp(xi)
        T1 = kappa / wn
        return (kappa, xi, th, T1, wn) if in_bounds(T1, wn) else None

    def resid2(q):
        u = unpack(q)
        return resid(u[3], u[4]) if u is not None else np.full(n_res, 1e3)

    lo_b, hi_b = [math.log(1e-3), 0.0], [math.log(1e3), 1.0]
    for k0 in (0.03, 0.1, 0.3, 1.0, 3.0, 10.0, 30.0):
        for s0 in (0.0, 0.3, 0.7):
            try:
                r = scipy.optimize.least_squares(resid2, [math.log(k0), s0], bounds=(lo_b, hi_b), method='trf',
                                                 x_scale='jac')
            except Exception:
                continue
            u = unpack(r.x)
            if u is not None:
                cands.append((cost(u[3], u[4]), u[0], u[1], u[2], 'interior'))
    cands = [c for c in cands if np.isfinite(c[0])]
    if not cands:
        raise ValueError(f"DOF {dof}: 拘束を満たす目標軌道が見つかりません（T・ε・上下限を見直してください）")
    c_best = min(cands, key=lambda c: c[0])
    _cst, kappa, xi, th, how = c_best
    wn = th / T * math.exp(xi)
    T1 = kappa / wn
    if abs(kappa - 1.0) < 0.02:
        warnings_.append("拘束付きフィットは κ ≈ 1（3重極）になりました。基準が T より遅いとき、T に完了する族の中で"
                         "3重極がすべての時刻で最も緩やかなので、実測の形によらずこの解になります（仕様どおり）")
    return TargetSpec(T1=T1, wn=wn, kappa=kappa, xi=xi, theta=th, active=True, method=how, cost=cost(T1, wn),
                      free_t_eps=t_free, warnings=warnings_, **common)


# ==============================================================================
# 100 Hz の時間軸（配信番号）。optuna_ff_common.py から移植
# ==============================================================================
# 移植元: optuna_ff_common.py:1122-1237 @105a28e
TICK_GAP_FACTOR = 1.5   # 受信間隔が局所の中央値のこの倍を超えたところだけを欠損の候補にする
TICK_WIN = 5            # 候補の前後で「遅れのない到着」を見る受信数
TICK_HALF = 50          # 局所の受信間隔の中央値を取る範囲（前後それぞれの間隔数）
TICK_COMPRESSED = 0.7   # 局所の中央値のこの倍より短い間隔は「遅れを取り戻している途中」
TICK_EARLY = 0.3        # 欠損の位置を決めるとき、この配信数より早い到着を強く罰する
TICK_ENUM_MAX = 20000   # 欠損の置き方の組み合わせがこれより多いクラスタは、丸めで位置を決める


def local_intervals(d, half=TICK_HALF):
    """各受信間隔 d_n の前後 half 個の中央値（0.5〜1.5 倍の外を除いてもう一度中央値）"""
    d = np.asarray(d, dtype=float)
    if len(d) == 0:
        return np.empty(0)
    pad = np.concatenate([np.full(half, np.nan), d, np.full(half, np.nan)])
    W = np.lib.stride_tricks.sliding_window_view(pad, 2 * half + 1)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        med = np.nanmedian(W, axis=1)
        W2 = np.where((W > 0.5 * med[:, None]) & (W < 1.5 * med[:, None]), W, np.nan)
        med2 = np.nanmedian(W2, axis=1)
    return np.where(np.isfinite(med2), med2, med)


def _place_drops(est, m):
    """クラスタの中で m 個の欠損を置き、受信 s+1..e−1 の s からの番号の差を返す"""
    L = len(est) - 1
    base = np.arange(1, L)
    if L <= 1 or m == 0:
        return base

    def cost(jk):
        lam = est[1:L] - jk
        return float(np.sum(np.where(lam >= -TICK_EARLY, np.abs(lam), 10.0 * -lam)))

    if math.comb(L + m - 1, m) > TICK_ENUM_MAX:
        out = np.empty(L - 1, dtype=int)
        p = 0
        for k in range(1, L):
            p = max(p + 1, min(int(round(est[k] - TICK_EARLY)), m + k))
            out[k - 1] = p
        return out
    best, best_cost = base, None
    for pos in itertools.combinations_with_replacement(range(L), m):
        jk = base + np.searchsorted(np.asarray(pos), base, side='left')
        cst = cost(jk)
        if best_cost is None or cst < best_cost:
            best, best_cost = jk, cst
    return best


def assign_ticks(t, u_hint=TICK_DT):
    """受信時刻 t（PC の時計、昇順）に配信番号 j を振る（受信順に数え、追いつかなかった分だけ欠損として番号を飛ばす）。
    戻り値 (j（最初を 0）, u（局所中央値の中央値）, 欠損数)。詳しい説明は optuna_ff_common.assign_ticks を参照。
    """
    t = np.asarray(t, dtype=float)
    n = len(t)
    if n < 3:
        return np.arange(n), u_hint, 0
    d = np.diff(t)
    U = local_intervals(d)
    U = np.where(np.isfinite(U) & (U > 0), U, u_hint)
    clusters = []
    for g in np.flatnonzero(d > TICK_GAP_FACTOR * U):
        g = int(g)
        if clusters and g <= clusters[-1][1] + TICK_WIN - 1:
            clusters[-1][0].append(g)
        else:
            clusters.append([[g], 0])
        k = g + 1
        while True:
            w = np.arange(k, min(n - 1, k + TICK_WIN))
            q = w[d[w] < TICK_COMPRESSED * U[w]]
            if len(q) == 0:
                break
            k = int(q[-1]) + 1
        clusters[-1][1] = k
    j = np.arange(n)
    add = 0
    done = 0
    for cl, be in clusters:
        s, e = cl[0], cl[-1] + 1
        u = float(U[s])
        j[done:s + 1] = np.arange(done, s + 1) + add
        kb = np.arange(max(0, s - TICK_WIN + 1), s + 1)
        ka = np.arange(e, min(n, be + TICK_WIN))
        Ts = float(np.min(t[kb] + (s - kb) * u))
        Te = float(np.min(t[ka] - (ka - e) * u))
        m_hi = int(sum(max(0, int(round(d[g] / u)) - 1) for g in cl))
        m = min(max(int(round((Te - Ts) / u - (e - s))), 0), m_hi)
        j[s + 1:e] = s + add + _place_drops((t[s:e + 1] - Ts) / u, m)
        add += m
        done = e
    j[done:] = np.arange(done, n) + add
    return j, float(np.median(U)), int(add)


# 移植元: optuna_ff_common.py:1273-1287 @105a28e
def align_ticks(t_kind, t_ref, j_ref, u_hint=TICK_DT):
    """他の種類（PWM など）のメッセージにも配信番号を振り、tk の番号に合わせる。

    種類ごとに assign_ticks で番号を振ってから、各メッセージに受信時刻が最も近い tk の番号との差の最頻値をずらし量にする
    （同じ配信のメッセージは続けて届くので、個々の遅れに左右されない）。戻り値 j（tk の番号系）
    """
    t_kind = np.asarray(t_kind, dtype=float)
    if len(t_kind) == 0 or len(t_ref) == 0:
        return np.zeros(len(t_kind), dtype=int)
    jk, _u, _nd = assign_ticks(t_kind, u_hint)
    pos = np.clip(np.searchsorted(t_ref, t_kind), 1, len(t_ref) - 1)
    near = np.where(np.abs(t_ref[pos - 1] - t_kind) <= np.abs(t_ref[pos] - t_kind), pos - 1, pos)
    off = j_ref[near] - jk
    vals, counts = np.unique(off, return_counts=True)
    return jk + int(vals[np.argmax(counts)])


class TrialIssue(Exception):
    """測定・解析で起きた問題。kind: 'comm'（受信の問題）/ 'eval'（解析できない）"""

    def __init__(self, kind, reason):
        super().__init__(reason)
        self.kind = kind
        self.reason = reason


# 移植元: optuna_ff_common.py:1300-1334 @105a28e
@dataclass
class DofSeries:
    """1自由度の、配信の時間軸にそろえた測定（j − j0 = -H..N_e+余裕）
    # 変更: 評価に使う POT・ボードの目標値だけを持つ（PWM・z1・z2・z3・derivatives は比較グラフ側で記録の時刻のまま使う）
    """
    dof: int
    board: str
    local: int
    j0: int
    jrel: np.ndarray
    pot: np.ndarray
    echo: np.ndarray
    Pi: float
    Pf: float
    P0: float
    H: int
    N_e: int
    echo_delay: float

    @property
    def t(self):
        return self.jrel * TICK_DT

    @property
    def dP(self):
        return self.Pf - self.Pi

    def win(self):
        return slice(self.H, self.H + self.N_e + 1)

    def missing(self):
        """評価区間 j − j0 = 0..N_e の中で受信できなかった配信の数"""
        return int(np.sum(~np.isfinite(self.pot[self.win()])))


# 移植元: optuna_ff_common.py:1337-1416 @105a28e
def extract_series(raw, initial, target, dofs, t_send, T, history_s=HISTORY_S, eval_min=EVAL_MIN_S):
    """受信データ {board: {'pot': (t, V)}} を、配信の時間軸で自由度ごとに切り出す。

    評価区間は j − j0 = 0..N_e、N_e = ⌊max(1.5T, eval_min)/0.010⌋。
    j0 はその自由度のボードの目標値（[6+i]）が round(新しい目標値) になった最初の配信。同じボードの自由度は同じ j0。
    目標値を受け取ってから 1 s 以内にボードの目標値が変わらなければ、そのボードの自由度は解析できない。
    # 変更: 1つのボードで問題があっても他のボードは切り出す（Optuna は試行全体を TrialIssue にする）。
    #       問題のあったボードの自由度は errors に理由を入れる。PWM などは切り出さない。
    戻り値 ({dof: DofSeries}, {board: 情報}, {dof: 解析できない理由})
    """
    N_e = n_eval(T, eval_min)
    H = int(round(history_s / TICK_DT))
    out, info, errors = {}, {}, {}
    by_board = {}
    for d in dofs:
        by_board.setdefault(DOF_BOARD[d][1], []).append(d)
    for board, bdofs in by_board.items():
        try:
            count = next(c for b, _o, c in BOARD_LAYOUT if b == board)
            kinds = raw.get(board, {})
            if 'pot' not in kinds or len(kinds['pot'][0]) < 3:
                raise TrialIssue('comm', f"{board}: POT（/tk）を受信できていません")
            t_tk, V = kinds['pot']
            j_tk, u, n_drop = assign_ticks(t_tk)
            j0s = []
            for d in bdofs:
                local = DOF_BOARD[d][2]
                new = int(round(float(target[d])))
                if new == int(round(float(initial[d]))):
                    continue
                hit = np.flatnonzero((V[:, count + local] == new) & (t_tk >= t_send - 0.05))
                if len(hit) == 0:
                    raise TrialIssue('comm', f"DOF {d}: ボードの目標値（POTdesired）が {new} になりませんでした")
                delay = float(t_tk[hit[0]] - t_send)
                if delay > 1.0:
                    raise TrialIssue('comm', f"DOF {d}: ボードの目標値が変わるまで {delay:.2f} s かかりました（> 1 s）")
                j0s.append((int(j_tk[hit[0]]), delay))
            if not j0s:
                raise TrialIssue('eval', f"{board}: 目標値が変わる自由度がないので t=0 を決められません")
            if max(j for j, _ in j0s) - min(j for j, _ in j0s) > 1:
                raise TrialIssue('eval', f"{board}: 自由度ごとのボードの目標値の変化がそろいません {j0s}")
            j0, delay = min(j0s)
            jrel = np.arange(-H, N_e + MARGIN_TICKS + 1)
            A = np.full((len(jrel), V.shape[1]), np.nan)
            pos = j_tk - j0 + H
            ok = (pos >= 0) & (pos < len(jrel))
            first = np.ones(len(pos), dtype=bool)
            seen = set()
            for i in np.flatnonzero(ok):
                if pos[i] in seen:
                    first[i] = False
                else:
                    seen.add(pos[i])
            sel = ok & first
            A[pos[sel]] = V[sel]
            for d in bdofs:
                local = DOF_BOARD[d][2]
                pot = A[:, local]
                pre = pot[max(0, H - int(round(REST_WINDOW_S / TICK_DT))):H]
                pre = pre[np.isfinite(pre)]
                P0 = float(np.mean(pre)) if len(pre) else float('nan')
                out[d] = DofSeries(dof=d, board=board, local=local, j0=j0, jrel=jrel, pot=pot,
                                   echo=A[:, count + local], Pi=float(initial[d]), Pf=float(target[d]),
                                   P0=P0, H=H, N_e=N_e, echo_delay=delay)
            info[board] = {'j0': j0, 'u_pc': u, 'n_drop': n_drop, 'n_msgs': len(t_tk), 'echo_delay': delay}
        except TrialIssue as exc:
            for d in bdofs:
                errors[d] = exc.reason
    return out, info, errors


# ==============================================================================
# 記録データ
# ==============================================================================
# 記録CSV（GUI_android_tk_FFsetting.py の時系列CSV と同じく、時刻を縦に並べ、トピックごとの組を横に並べる形式）
#   ボード k（1〜5）ごとに次の組（d はそのボードのDOF番号 0〜25）
#     Time{k},        POT{d}…, POTdesired{d}…        ← /board{k}_tk/pub の受信時刻と、実測POT・ボードが使っている目標値
#     Time{k}_PWM,    PWM{d}…, FF{d}…, ADRC{d}…      ← PWM と、その配信での FF（設定から内部で作った値。compute_internal_ff）・
#                                                       ADRC = PWM − FF。目標値が変わる前（負の時刻）の FF・ADRC は空欄
#     Time{k}_z3,     z3_{d}…                         ← 外乱推定値 z3
#     Time{k}_FFrecv, FFrecv_{d}…                     ← /board{k}_tk_FF_float/pub で受信した FF（実機の outputAdd）
#   Time_cmd, cmd0..cmd25        … 受信した目標値メッセージ（0 秒が記録開始のメッセージ）
#   cond_*（26行。行 = DOF 0〜25） … 変化前・変化後の目標値、記録開始の時点で送信済みだったFF（有無・T・極値・係数・出どころ）、
#                                   記録中に受信したFFの最大絶対値、記録開始時のADRCパラメータ
#   meta_key, meta_value          … 形式・名前・開始日時・記録時間・FFの有無（w/wo）とその判定・警告
#   時刻は記録開始（目標値が変わったメッセージを受信した時刻）からの秒数で、全トピック共通の PC の時計で測る。
#   負の時刻は目標値が変わる前の受信（HISTORY_S 秒）。トピックごとに別々に届くので、i 行目は「各トピックの i 回目の受信」。
#   値は必ず同じ組の時刻列と組にして使うこと。受信回数が少ない組の残りの行は空欄。
KINDS = ('pot', 'pwm', 'z3', 'ffrecv')
COND_KEYS = ('DOF', 'board', 'FF_number', 'init_target', 'target', 'FF_set', 'FF_T', 't1', 'y1', 't2', 'y2',
             'a', 'b', 'c', 'd', 'e', 'FF_source', 'FF_recv_maxabs', 'kp', 'kd', 'ic', 'lambda')


def kind_width(kind, count):
    return 2 * count if kind == 'pot' else count


def _fmt_int(v):
    return "" if v != v else str(int(round(v)))


def _fmt_float(v):
    return "" if v != v else "%.7g" % v


def _fmt_any(v):
    if v is None:
        return ""
    if isinstance(v, (bool, np.bool_)):
        return "1" if v else "0"
    if isinstance(v, (int, np.integer)):
        return str(int(v))
    if isinstance(v, (float, np.floating)):
        return "" if not np.isfinite(v) else "%.9g" % v
    return str(v)


def _parse_float(s):
    try:
        return float(s) if s not in ('', None) else float('nan')
    except ValueError:
        return float('nan')


class Record:
    """1回の記録（GUIで作る、またはCSVから読む）

    meta: name, start (ISO), stamp, duration, history_s, init_target[26], target[26], ff_table[24][6],
          ff_setting[24]（{T,t1,y1,t2,y2} or None）, ff_source[24], ff_status {k: [状態, 時刻]}, adrc {DOF: [kp,kd,ic,λ]},
          ff_label 'w' / 'wo', ff_judge（判定の根拠の文）, ff_recv_boards, ff_recv_max[26], warnings
    series: board名 → {'pot': (t, V[n, 2c]), 'pwm': (t, V), 'z3': (t, V), 'ffrecv': (t, V), 'ff': (t, V)}
            'ff' は 'pwm' と同じ時刻の、設定から内部で作った FF
    """

    def __init__(self, meta, series, cmd_t, cmd_v):
        self.meta = meta
        self.series = series
        self.cmd_t = np.asarray(cmd_t, dtype=float)
        self.cmd_v = (np.asarray(cmd_v, dtype=float).reshape(len(self.cmd_t), N_DOF) if len(self.cmd_t)
                      else np.zeros((0, N_DOF)))

    @property
    def name(self):
        return self.meta.get('name', '')

    @property
    def duration(self):
        return float(self.meta.get('duration', 0.0))

    def legend(self):
        """凡例の表記（FFあり 'w/'、FFなし 'w/o'）"""
        return 'w/' if self.meta.get('ff_label') == 'w' else 'w/o'

    def ff6(self, dof):
        n = FF_INDEX.get(dof)
        if n is None:
            return [0.0, 0.0, 0.0, 0.0, 0.0, FF_T_DEFAULT]
        return [float(v) for v in self.meta['ff_table'][n]]

    def kind_data(self, board, kind):
        s = self.series.get(board, {})
        if kind in s:
            return s[kind]
        return np.zeros(0), np.zeros((0, kind_width(kind if kind != 'ff' else 'pwm', BOARD_COUNT[board])))

    def dof_series(self, dof, kind):
        """DOF の (時刻, 値)。kind = 'pot' / 'desired' / 'pwm' / 'z3' / 'ffrecv' / 'ff'（内部で作ったFF）/ 'adrc'"""
        _k, board, local, count = DOF_BOARD[dof]
        if kind in ('pot', 'desired'):
            t, V = self.kind_data(board, 'pot')
            col = local if kind == 'pot' else count + local
        elif kind == 'adrc':
            t, P = self.kind_data(board, 'pwm')
            _tf, F = self.kind_data(board, 'ff')
            if len(t) == 0:
                return np.zeros(0), np.zeros(0)
            if F.shape != P.shape:
                F = np.full(P.shape, np.nan)
            return t, P[:, local] - F[:, local]
        else:
            t, V = self.kind_data(board, kind)
            col = local
        if len(t) == 0:
            return np.zeros(0), np.zeros(0)
        return t, V[:, col]

    def received(self, board, kind):
        return len(self.kind_data(board, kind)[0]) > 0

    def pot_raw(self):
        """評価（extract_series）に渡す {board: {'pot': (t, V)}}"""
        return {board: {'pot': self.kind_data(board, 'pot')} for board, _o, _c in BOARD_LAYOUT}

    def changed_dofs(self):
        out = []
        for d in range(N_DOF):
            pi, pf = self.meta['init_target'][d], self.meta['target'][d]
            if _num(pi) and _num(pf) and abs(pf - pi) > POSTURE_TOL:
                out.append(d)
        return out

    def payload(self):
        """別プロセス（保存処理）へ渡す形"""
        return {'meta': self.meta, 'series': self.series, 'cmd_t': self.cmd_t, 'cmd_v': self.cmd_v}

    @classmethod
    def from_payload(cls, p):
        return cls(p['meta'], p['series'], p['cmd_t'], p['cmd_v'])


def compute_internal_ff(record):
    """series[board]['ff'] に、各 PWM の配信での内部FF（設定した係数から作る。ADRC = PWM − FF に使う）を入れる

    POT（/tk）に assign_ticks で配信番号を振り、PWM は align_ticks でその番号にそろえる（評価・比較グラフと同じ番号）。
    FF の時刻は ff_internal_ticks（Optuna の記録CSV と同じ）。/tk を受信できなかったボードは配信番号を振れないので、
    PC の受信時刻で計算し（ff_internal）、警告を残す。
    """
    meta = record.meta
    for board, offset, count in BOARD_LAYOUT:
        t, _P = record.kind_data(board, 'pwm')
        t_tk, V = record.kind_data(board, 'pot')
        F = np.zeros((len(t), count))
        ticks = len(t_tk) >= 3 and len(t) > 0
        if ticks:
            j_tk, _u, _nd = assign_ticks(t_tk)
            j_pw = align_ticks(t, t_tk, j_tk)
        elif len(t) and any(np.any(np.asarray(record.ff6(offset + j)[:5]) != 0) for j in range(count)):
            meta.setdefault('warnings', []).append(
                f"{board}: 実測POT（/tk）が足りず配信番号を振れないため、ADRC 用の FF を PC の受信時刻で計算しました"
                "（PC の時計のずれの分だけ不正確です）")
        for j in range(count):
            d = offset + j
            if d not in FF_INDEX:
                F[:, j] = np.where(t < 0, np.nan, 0.0)
            elif ticks:
                F[:, j] = ff_internal_ticks(record.ff6(d), echo_restart_ticks(t_tk, j_tk, V[:, count + j]), j_pw, t)
            else:
                F[:, j] = ff_internal(record.ff6(d), target_restarts(record.cmd_t, record.cmd_v,
                                                                     meta['init_target'], d), t)
        record.series.setdefault(board, {})['ff'] = (t, F)


def judge_ff_label(record):
    """FFの有無（記録ごと）を、記録中（0 ≤ t ≤ 記録時間）に受信したFFで判定する。meta の ff_label・ff_judge・ff_recv_* を書く

    1自由度でも |FF| > FF_ON_EPS なら 'w'、どの自由度も 0 なら 'wo'。
    FFを受信できなかったボードは、送信した設定（係数が0でない自由度があるか）で判定し、警告を残す。
    """
    meta = record.meta
    warns = meta.setdefault('warnings', [])
    recv_max = [float('nan')] * N_DOF
    recv_boards, missing = [], []
    on_recv = False
    for k, (board, offset, count) in enumerate(BOARD_LAYOUT, start=1):
        t, V = record.kind_data(board, 'ffrecv')
        sel = (t >= 0) & (t <= record.duration)
        if not sel.any():
            missing.append(k)
            continue
        recv_boards.append(k)
        for j in range(count):
            col = V[sel, j]
            col = col[np.isfinite(col)]
            recv_max[offset + j] = float(np.max(np.abs(col))) if len(col) else float('nan')
            if len(col) and recv_max[offset + j] > FF_ON_EPS:
                on_recv = True
    sent_on = [d for d in FF_DOF_MAP if np.any(np.asarray(record.ff6(d)[:5]) != 0)]
    sent_on_missing = [d for d in sent_on if DOF_BOARD[d][0] in missing]
    if on_recv:
        label, judge = 'w', "記録中に受信したFF（0 でない自由度あり）"
    elif sent_on_missing:
        label = 'w'
        judge = (f"board{', board'.join(map(str, missing))} のFFを受信できなかったため、送信した設定で判定"
                 f"（DOF {', '.join(map(str, sent_on_missing))} にFFを送信済み）")
    else:
        label = 'wo'
        judge = "記録中に受信したFF（全自由度 0）" if not missing else \
            f"受信したFF（全自由度 0）＋受信できなかった board{', board'.join(map(str, missing))} は送信した設定（FF=0）"
    if missing:
        warns.append(f"board{', board'.join(map(str, missing))} の実機のFF（/board{{k}}_tk_FF_float/pub）を受信できませんでした"
                     "（ファームに FF の配信がない可能性）。FFのグラフは no data になります")
    changed = set(record.changed_dofs())
    not_applied = [d for d in sent_on if d in changed and DOF_BOARD[d][0] in recv_boards
                   and not (recv_max[d] > FF_ON_EPS)]
    if not_applied:
        warns.append(f"DOF {', '.join(map(str, not_applied))}: FFを送信済みで目標値も変わりましたが、実機のFFは 0 のままでした"
                     "（送信が届いていない可能性）")
    foreign = [d for d in range(N_DOF) if recv_max[d] > FF_ON_EPS and d not in sent_on]
    if foreign:
        warns.append(f"DOF {', '.join(map(str, foreign))}: このGUIはFFを送信していませんが、実機のFFが 0 ではありませんでした"
                     "（他のプログラムが送信した可能性）")
    meta['ff_label'] = label
    meta['ff_judge'] = judge
    meta['ff_recv_boards'] = recv_boards
    meta['ff_recv_max'] = recv_max
    return label


def record_folder_name(record):
    return f"FF({record.meta['ff_label']}_{record.meta['stamp']})"


def make_unique_folder(base_dir, name):
    """<base_dir>/<name> を作る（同じ名前があれば _2, _3 … を付け、既存のフォルダには書かない）"""
    path = os.path.join(base_dir, name)
    n = 2
    while os.path.exists(path):
        path = os.path.join(base_dir, f"{name}_{n}")
        n += 1
    os.makedirs(path)
    return path


def record_header(k, kind, offset, count):
    dofs = range(offset, offset + count)
    if kind == 'pot':
        return [f"Time{k}"] + [f"POT{d}" for d in dofs] + [f"POTdesired{d}" for d in dofs]
    if kind == 'pwm':
        return ([f"Time{k}_PWM"] + [f"PWM{d}" for d in dofs] + [f"FF{d}" for d in dofs]
                + [f"ADRC{d}" for d in dofs])
    if kind == 'z3':
        return [f"Time{k}_z3"] + [f"z3_{d}" for d in dofs]
    return [f"Time{k}_FFrecv"] + [f"FFrecv_{d}" for d in dofs]


def cond_rows(record):
    meta = record.meta
    rows = []
    for d in range(N_DOF):
        k, board, local, _c = DOF_BOARD[d]
        n = FF_INDEX.get(d)
        ff6 = record.ff6(d)
        st = meta['ff_setting'][n] if n is not None else None
        on = n is not None and np.any(np.asarray(ff6[:5]) != 0)
        adrc = (meta.get('adrc') or {}).get(str(d)) or [None] * 4
        rows.append({
            'DOF': d, 'board': board, 'FF_number': (n + 1) if n is not None else '',
            'init_target': meta['init_target'][d], 'target': meta['target'][d],
            'FF_set': int(bool(on)), 'FF_T': ff6[5] if n is not None else None,
            't1': st['t1'] if st else None, 'y1': st['y1'] if st else None,
            't2': st['t2'] if st else None, 'y2': st['y2'] if st else None,
            'a': ff6[0], 'b': ff6[1], 'c': ff6[2], 'd': ff6[3], 'e': ff6[4],
            'FF_source': (meta['ff_source'][n] if n is not None else 'FFの枠なし'),
            'FF_recv_maxabs': meta.get('ff_recv_max', [None] * N_DOF)[d],
            'kp': adrc[0], 'kd': adrc[1], 'ic': adrc[2], 'lambda': adrc[3]})
    return rows


def meta_rows(record):
    meta = record.meta
    rows = [('format', RECORD_FORMAT), ('name', meta.get('name', '')), ('start', meta.get('start', '')),
            ('stamp', meta.get('stamp', '')), ('duration_s', _fmt_any(meta.get('duration'))),
            ('history_s', _fmt_any(meta.get('history_s'))), ('ff_label', meta.get('ff_label', '')),
            ('ff_judge', meta.get('ff_judge', '')),
            ('ff_recv_boards', ' '.join(map(str, meta.get('ff_recv_boards', []))))]
    for k, (status, stamp) in sorted((meta.get('ff_status') or {}).items()):
        rows.append((f'ff_status_board{k}', f"{status} {stamp or ''}".strip()))
    for i, w in enumerate(meta.get('warnings', []), start=1):
        rows.append((f'warning_{i}', w))
    rows.append(('program', 'GUI_android_tk_cfFF.py'))
    return rows


def write_record_csv(record, path):
    """記録CSV（形式は上の説明）を書く"""
    blocks = []
    for k, (board, offset, count) in enumerate(BOARD_LAYOUT, start=1):
        t, V = record.kind_data(board, 'pot')
        blocks.append((record_header(k, 'pot', offset, count),
                       [[f"{t[i]:.6f}"] + [_fmt_int(x) for x in V[i]] for i in range(len(t))]))
        t, V = record.kind_data(board, 'pwm')
        _tf, F = record.kind_data(board, 'ff')
        if F.shape != V.shape:
            F = np.full(V.shape, np.nan)
        A = V - F
        blocks.append((record_header(k, 'pwm', offset, count),
                       [[f"{t[i]:.6f}"] + [_fmt_float(x) for x in V[i]] + [_fmt_float(x) for x in F[i]]
                        + [_fmt_float(x) for x in A[i]] for i in range(len(t))]))
        for kind in ('z3', 'ffrecv'):
            t, V = record.kind_data(board, kind)
            blocks.append((record_header(k, kind, offset, count),
                           [[f"{t[i]:.6f}"] + [_fmt_float(x) for x in V[i]] for i in range(len(t))]))
    blocks.append((["Time_cmd"] + [f"cmd{d}" for d in range(N_DOF)],
                   [[f"{record.cmd_t[i]:.6f}"] + [_fmt_float(x) for x in record.cmd_v[i]]
                    for i in range(len(record.cmd_t))]))
    blocks.append(([f"cond_{key}" for key in COND_KEYS],
                   [[_fmt_any(r[key]) for key in COND_KEYS] for r in cond_rows(record)]))
    blocks.append((["meta_key", "meta_value"], [[a, b] for a, b in meta_rows(record)]))
    n_rows = max((len(rows) for _, rows in blocks), default=0)
    with open(path, 'w', newline='', encoding=CSV_ENCODING) as f:
        w = csv.writer(f)
        w.writerow([name for header, _ in blocks for name in header])
        for i in range(n_rows):
            row = []
            for header, rows in blocks:
                row.extend(rows[i] if i < len(rows) else [""] * len(header))
            w.writerow(row)


def _column_block(cols, tcol, vcols):
    """CSV の列 → (時刻[n], 値[n, len(vcols)])。時刻が空の行は除く"""
    if tcol not in cols:
        return np.zeros(0), np.zeros((0, len(vcols)))
    t = np.array([_parse_float(s) for s in cols[tcol]])
    ok = np.isfinite(t)
    V = np.full((len(t), len(vcols)), np.nan)
    for j, c in enumerate(vcols):
        if c in cols:
            V[:, j] = [_parse_float(s) for s in cols[c]]
    return t[ok], V[ok]


def read_record_csv(path):
    """記録CSV（write_record_csv の形式）を Record にする。形式が違えば ValueError"""
    with open(path, newline='', encoding='utf-8-sig') as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if not header:
            raise ValueError("空のファイルです")
        cols = {name: [] for name in header}
        for row in reader:
            for name, cell in zip(header, row):
                cols[name].append(cell)
    if 'meta_key' not in cols:
        raise ValueError("このGUI（GUI_android_tk_cfFF.py）の記録CSVではありません（meta_key 列がありません）")
    meta_kv = {k: v for k, v in zip(cols['meta_key'], cols['meta_value']) if k}
    if meta_kv.get('format') != RECORD_FORMAT:
        raise ValueError(f"記録CSVの形式が違います（format = {meta_kv.get('format')!r}）")
    series = {}
    for k, (board, offset, count) in enumerate(BOARD_LAYOUT, start=1):
        dofs = list(range(offset, offset + count))
        s = {}
        s['pot'] = _column_block(cols, f"Time{k}", [f"POT{d}" for d in dofs] + [f"POTdesired{d}" for d in dofs])
        t, PFA = _column_block(cols, f"Time{k}_PWM", [f"PWM{d}" for d in dofs] + [f"FF{d}" for d in dofs])
        s['pwm'] = (t, PFA[:, :count])
        s['ff'] = (t, PFA[:, count:])
        s['z3'] = _column_block(cols, f"Time{k}_z3", [f"z3_{d}" for d in dofs])
        s['ffrecv'] = _column_block(cols, f"Time{k}_FFrecv", [f"FFrecv_{d}" for d in dofs])
        series[board] = s
    cmd_t, cmd_v = _column_block(cols, "Time_cmd", [f"cmd{d}" for d in range(N_DOF)])
    cond = {}
    for i, dtext in enumerate(cols.get('cond_DOF', [])):
        if dtext == '':
            continue
        cond[int(dtext)] = {key: cols[f'cond_{key}'][i] for key in COND_KEYS if f'cond_{key}' in cols}
    if sorted(cond) != list(range(N_DOF)):
        raise ValueError("記録CSVの cond_* の行が26自由度分ありません")

    def num_or_none(s):
        v = _parse_float(s)
        return v if math.isfinite(v) else None

    ff_table = zero_ff_table()
    ff_setting, ff_source = [None] * N_FF, [''] * N_FF
    adrc = {}
    for d, c in cond.items():
        n = FF_INDEX.get(d)
        if n is not None:
            ff_table[n] = [_parse_float(c[key]) if c.get(key, '') != '' else 0.0 for key in ('a', 'b', 'c', 'd', 'e')] \
                + [_parse_float(c['FF_T']) if c.get('FF_T', '') != '' else FF_T_DEFAULT]
            if c.get('t1', '') != '':
                ff_setting[n] = {k2: _parse_float(c[k2]) for k2 in ('t1', 'y1', 't2', 'y2')}
                ff_setting[n]['T'] = float(ff_table[n][5])
            ff_source[n] = c.get('FF_source', '')
        vals = [num_or_none(c.get(key, '')) for key in ('kp', 'kd', 'ic', 'lambda')]
        if any(v is not None for v in vals):
            adrc[str(d)] = vals
    status = {}
    for key, val in meta_kv.items():
        m = re.match(r'ff_status_board(\d+)$', key)
        if m:
            parts = val.split(' ', 1)
            status[m.group(1)] = [parts[0], parts[1] if len(parts) > 1 else None]
    meta = {
        'name': meta_kv.get('name') or os.path.splitext(os.path.basename(path))[0],
        'start': meta_kv.get('start', ''), 'stamp': meta_kv.get('stamp', ''),
        'duration': _parse_float(meta_kv.get('duration_s', 'nan')),
        'history_s': _parse_float(meta_kv.get('history_s', 'nan')),
        'init_target': [num_or_none(cond[d]['init_target']) for d in range(N_DOF)],
        'target': [num_or_none(cond[d]['target']) for d in range(N_DOF)],
        'ff_table': ff_table.tolist(), 'ff_setting': ff_setting, 'ff_source': ff_source, 'ff_status': status,
        'adrc': adrc, 'ff_label': meta_kv.get('ff_label', ''), 'ff_judge': meta_kv.get('ff_judge', ''),
        'ff_recv_boards': [int(x) for x in meta_kv.get('ff_recv_boards', '').split()],
        'ff_recv_max': [num_or_none(cond[d].get('FF_recv_maxabs', '')) for d in range(N_DOF)],
        'warnings': [v for k, v in sorted(((k, v) for k, v in meta_kv.items() if re.match(r'warning_\d+$', k)),
                                          key=lambda kv: int(kv[0].split('_')[1]))],
        'path': os.path.abspath(path),
    }
    if meta['ff_label'] not in ('w', 'wo'):
        raise ValueError(f"記録CSVの ff_label が不正です（{meta['ff_label']!r}）")
    if not math.isfinite(meta['duration']):
        raise ValueError("記録CSVの duration_s が不正です")
    return Record(meta, series, cmd_t, cmd_v)


# ==============================================================================
# グラフ
# ==============================================================================
def _gapped(t, v, max_gap=GAP_LINE_S):
    """受信間隔が max_gap を超えたところに NaN を入れて、欠測の前後を線でつながないようにする"""
    t = np.asarray(t, dtype=float)
    v = np.asarray(v, dtype=float)
    ok = np.isfinite(t) & np.isfinite(v)
    t, v = t[ok], v[ok]
    if len(t) < 2:
        return t, v
    gaps = np.flatnonzero(np.diff(t) > max_gap)
    if len(gaps) == 0:
        return t, v
    return np.insert(t, gaps + 1, np.nan), np.insert(v, gaps + 1, np.nan)


def plot_dof_kind(ax, entries, dof, kind, xmax, title=None, target=None, xlabel=RECORD_XLABEL):
    """1枚のグラフを描く。entries = [(Record, (主の色, ADRCの色)), ...]（比較では2つ）

    凡例: POT (w/o|w/) / Reference / Disturbance z_3 (…) / PWM = ADRC + FF (…) / ADRC (…) / FF (…)
    線種: Reference だけ破線、ほかは実線。Reference は黒。FF は実機から受信した値（設定から作った値ではない）
    target = (t, POT) を渡すと、POT のグラフに目標軌道（3次遅れ系）を緑の一点鎖線で重ねる（比較のとき）
    """
    k = DOF_BOARD[dof][0]
    missing = []
    for rec, (main, sub) in entries:
        wl = rec.legend()
        if kind == 'pot':
            series = [('pot', f"POT ({wl})", main, '-'), ('desired', "Reference", REF_COLOR, '--')]
        elif kind == 'z3':
            series = [('z3', f"Disturbance z_3 ({wl})", main, '-')]
        elif kind == 'pwm':
            series = [('pwm', f"PWM = ADRC + FF ({wl})", main, '-'), ('adrc', f"ADRC ({wl})", sub, '-')]
        else:
            series = [('ffrecv', f"FF ({wl})", main, '-')]
        for skind, label, color, style in series:
            t, v = rec.dof_series(dof, skind)
            if not np.isfinite(v).any():
                topic = {'pot': TOPIC_POT, 'desired': TOPIC_POT, 'z3': TOPIC_Z3, 'pwm': TOPIC_PWM, 'adrc': TOPIC_PWM,
                         'ffrecv': TOPIC_FF_OUT}[skind].format(k)
                missing.append(f"{rec.name}: {topic} not received")
                continue
            x, y = _gapped(t, v)
            # FF=0 では ADRC = PWM なので、PWM を ADRC より上に描く（凡例の順は変えない）
            ax.plot(x, y, color=color, linestyle=style, linewidth=1.2, label=label, zorder=2.5 if skind == 'pwm' else 2)
    if kind == 'pot' and target is not None:
        ax.plot(target[0], target[1], color=TARGET_COLOR, linestyle=TARGET_STYLE, linewidth=1.4, label=TARGET_LABEL)
    handles, labels = ax.get_legend_handles_labels()
    seen, h2, l2 = set(), [], []
    for h, lab in zip(handles, labels):       # 同じ凡例（Reference が2本など）は1つにまとめる
        key = (lab, h.get_color(), h.get_linestyle())
        if key not in seen:
            seen.add(key)
            h2.append(h)
            l2.append(lab)
    if h2:
        ax.legend(h2, l2, loc='best', fontsize=8)
    if missing:
        ax.text(0.5, 0.5, "no data\n" + "\n".join(sorted(set(missing))), ha='center', va='center',
                transform=ax.transAxes, color='gray', fontsize=8)
    ax.set_xlim(0.0, max(float(xmax), 1e-3))
    ax.set_xlabel(xlabel)
    ax.set_ylabel(GRAPH_YLABELS[kind])
    ax.grid(True, alpha=0.4)
    if title is not None:
        ax.set_title(title, fontsize=9)


def _visible_ylim(ax, xmax):
    """表示範囲（0〜xmax）の値だけで縦軸を決める（変化前の負の時刻の値で縦軸が広がらないように）"""
    lo, hi = np.inf, -np.inf
    for line in ax.get_lines():
        x = np.asarray(line.get_xdata(), dtype=float)
        y = np.asarray(line.get_ydata(), dtype=float)
        sel = np.isfinite(x) & np.isfinite(y) & (x >= 0) & (x <= xmax)
        if sel.any():
            lo, hi = min(lo, y[sel].min()), max(hi, y[sel].max())
    if np.isfinite(lo) and np.isfinite(hi):
        pad = 0.06 * (hi - lo) if hi > lo else max(1.0, abs(hi) * 0.05)
        ax.set_ylim(lo - pad, hi + pad)


def save_dof_graph(path, entries, dof, kind, xmax, title, target=None, xlabel=RECORD_XLABEL):
    """PNG を1枚書く（pyplot は使わない。別プロセスでも安全に描ける）"""
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    fig = Figure(figsize=(8, 4.5), dpi=100)
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(1, 1, 1)
    plot_dof_kind(ax, entries, dof, kind, xmax, title, target, xlabel)
    _visible_ylim(ax, xmax)
    fig.tight_layout()
    fig.savefig(path)


def record_graph_title(rec, dof, kind):
    return f"DOF {dof}  {GRAPH_TITLES[kind]}\n{rec.name}"


def compare_graph_title(rec1, rec2, dof, kind):
    return (f"DOF {dof}  {GRAPH_TITLES[kind]} (comparison)\n"
            f"1 (blue): {rec1.name}    2 (red): {rec2.name}")


def write_record_graphs(rec, folder, progress=None):
    total = N_DOF * len(GRAPH_KINDS)
    made = 0
    for dof in range(N_DOF):
        out_dir = os.path.join(folder, 'DOF', f'DOF{dof}')
        os.makedirs(out_dir, exist_ok=True)
        for kind in GRAPH_KINDS:
            save_dof_graph(os.path.join(out_dir, RECORD_GRAPH_FILES[kind]), [(rec, DATA_COLORS[0])], dof, kind,
                           rec.duration, record_graph_title(rec, dof, kind))
            made += 1
            if progress is not None:
                progress.put(('graph', made, total))


def write_compare_graphs(tr1, tr2, folder, progress=None, xmax=None, targets=None):
    """比較グラフ。tr1, tr2 は配信番号の時間軸にした記録（tick_record）、targets = {DOF: (t, 目標軌道の POT)}"""
    total = N_DOF * len(GRAPH_KINDS)
    made = 0
    if xmax is None:
        xmax = max(tick_end(tr1), tick_end(tr2))
    for dof in range(N_DOF):
        out_dir = os.path.join(folder, 'DOF', f'DOF{dof}')
        os.makedirs(out_dir, exist_ok=True)
        for kind in GRAPH_KINDS:
            save_dof_graph(os.path.join(out_dir, COMPARE_GRAPH_FILES[kind]),
                           [(tr1, DATA_COLORS[0]), (tr2, DATA_COLORS[1])], dof, kind, xmax,
                           compare_graph_title(tr1, tr2, dof, kind), (targets or {}).get(dof), COMPARE_XLABEL)
            made += 1
            if progress is not None:
                progress.put(('graph', made, total))


def save_record_outputs(payload, folder, progress):
    """記録の保存（GUIとは別プロセスで動かす）: CSV を書き、そのCSVを読み直してグラフを作る（CSVとグラフの値をそろえる）"""
    try:
        rec = Record.from_payload(payload)
        path = os.path.join(folder, os.path.basename(folder) + '.csv')
        write_record_csv(rec, path)
        progress.put(('csv', path))
        rec2 = read_record_csv(path)
        write_record_graphs(rec2, folder, progress)
        progress.put(('done', path))
    except Exception:
        progress.put(('error', traceback.format_exc()))


# ==============================================================================
# Optuna の最適化結果（optimize_optuna_ff_ROS2.py の <フォルダ名>_trials.csv）
# ==============================================================================
OPTUNA_NOT_COUNTED = ('past', 'final')     # best に数えない試行（optuna_ff_common.NOT_COUNTED_SOURCES と同じ）
OPTUNA_DOF_KEYS = ('T', 't1_sent', 'y1_sent', 't2_sent', 'y2_sent', 'a', 'b', 'c', 'd', 'e', 'J_i')


class OptunaResult:
    """trials.csv（optuna_ff_common.trial_columns の列）を読む。列は DOF番号 d{DOF}_… で引く（FF番号ではない）"""

    def __init__(self, folder):
        self.folder = os.path.abspath(folder)
        self.name = os.path.basename(self.folder.rstrip(os.sep))
        path = os.path.join(self.folder, f"{self.name}_trials.csv")
        if not os.path.exists(path):
            cands = sorted(f for f in os.listdir(self.folder) if f.endswith('_trials.csv'))
            if not cands:
                raise ValueError(f"{self.folder} に *_trials.csv がありません（optimize_optuna_ff_ROS2.py の結果フォルダを"
                                 "選んでください）")
            path = os.path.join(self.folder, cands[0])
        self.path = path
        with open(path, newline='', encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            cols = reader.fieldnames or []
            rows = list(reader)
        need = ('trial', 'state', 'source', 'J')
        if any(c not in cols for c in need):
            raise ValueError(f"{os.path.basename(path)} は Optuna の結果ではありません（列 {need} がありません）")
        self.dofs = sorted({int(m.group(1)) for c in cols for m in [re.match(r'd(\d+)_t1_sent$', c)] if m})
        if not self.dofs:
            raise ValueError(f"{os.path.basename(path)} に自由度の列（d<DOF>_t1_sent など）がありません")
        self.trials = {}
        for r in rows:
            try:
                n = int(r['trial'])
            except (TypeError, ValueError):
                continue
            tr = {'n': n, 'state': r.get('state', ''), 'source': r.get('source', ''), 'J': _parse_float(r.get('J')),
                  'dof': {}}
            for d in self.dofs:
                tr['dof'][d] = {k: _parse_float(r.get(f'd{d}_{k}')) for k in OPTUNA_DOF_KEYS}
            self.trials[n] = tr         # 同じ番号の行が複数あれば後の行（追記された最新）を使う
        self.config = {}
        try:
            with open(os.path.join(self.folder, 'config.json'), encoding='utf-8') as f:
                self.config = json.load(f)
        except (OSError, ValueError):
            pass
        # グループ（config.json の groups。ない・不正なら最適化した自由度全体で1グループ＝従来どおり）
        groups = self.config.get('groups') if isinstance(self.config, dict) else None
        ok = (isinstance(groups, list) and groups
              and all(isinstance(g, list) and g and all(isinstance(d, int) and d in self.dofs for d in g) for g in groups))
        self.groups = [list(g) for g in groups] if ok else [list(self.dofs)]

    def candidates(self):
        """①②④ の対象: COMPLETE で J が有限、過去の試行（past）と最後の組み合わせの測定（final）を除く
        （optuna_ff_common.FFStudy.best()・group_bests() と同じ集合。確認試行・再投入を含む）"""
        return [t for t in self.trials.values()
                if t['state'] == 'COMPLETE' and t['source'] not in OPTUNA_NOT_COUNTED and np.isfinite(t['J'])]

    def group_of(self, dof):
        """自由度が入っているグループ (番号, 自由度のリスト)。最適化していない自由度は (None, None)"""
        for k, g in enumerate(self.groups):
            if dof in g:
                return k, g
        return None, None

    def group_best(self, k):
        """グループ k の best: J_g = グループ内の J_i の平均 が最小の試行と J_g（optuna_ff_common.FFStudy.group_bests と同じ。
        J_i が1つでも欠ける試行は使わない。同じ値なら番号の小さい試行）。戻り値 (試行 or None, J_g)"""
        g = self.groups[k]
        best, bv = None, float('inf')
        for t in sorted(self.candidates(), key=lambda t: t['n']):
            vals = [t['dof'][d]['J_i'] for d in g]
            if all(np.isfinite(vals)):
                v = float(np.mean(vals))
                if v < bv:
                    best, bv = t, v
        return best, (bv if best is not None else float('nan'))

    def best_overall(self):
        c = self.candidates()
        return min(c, key=lambda t: (t['J'], t['n'])) if c else None

    def best_for_dof(self, dof):
        if dof not in self.dofs:
            return None
        c = [t for t in self.candidates() if np.isfinite(t['dof'][dof]['J_i'])]
        return min(c, key=lambda t: (t['dof'][dof]['J_i'], t['n'])) if c else None

    def trial(self, n):
        return self.trials.get(int(n))

    def extrema(self, tr, dof):
        """試行 tr の自由度 dof の (T, t1, y1, t2, y2, 送った係数[a..e])。極値がなければ None"""
        if tr is None or dof not in self.dofs:
            return None
        v = tr['dof'][dof]
        vals = [v[k] for k in ('T', 't1_sent', 'y1_sent', 't2_sent', 'y2_sent')]
        if not all(np.isfinite(vals)):
            return None
        return tuple(float(x) for x in vals) + ([v[k] for k in ('a', 'b', 'c', 'd', 'e')],)

    def summary(self):
        cfg = self.config
        best = self.best_overall()
        text = (f"{self.name}: 試行 {len(self.trials)} 個（①②の対象 {len(self.candidates())} 個）、"
                f"最適化した自由度 {', '.join(map(str, self.dofs))}")
        if best is not None:
            text += f"、J 最小 = 試行 {best['n']}（J = {best['J']:.4g}）"
        if len(self.groups) >= 2:
            parts = []
            for k in range(len(self.groups)):
                t, v = self.group_best(k)
                parts.append(f"g{k}（DOF {' '.join(map(str, self.groups[k]))}）: 試行 {t['n']}" if t is not None
                             else f"g{k}: なし")
            text += f"\nグループ {len(self.groups)} 個の best: " + "、".join(parts)
        norm = cfg.get('err_norm', 'dP') if cfg else 'dP'
        text += "、評価値は ΔP で割る" if norm == 'dP' else "、評価値は ΔP で割らない [count²]"
        if cfg:
            text += (f"\n動作完了時間 T = {cfg.get('T')} s、FF入力時間 T_FF = {cfg.get('T_ff', cfg.get('T'))} s、"
                     f"ε = {float(cfg.get('eps', float('nan'))) * 100:g}%、評価時間の固定値 E = {cfg.get('eval_min', EVAL_MIN_S)} s")
            init, tgt = cfg.get('initial'), cfg.get('target')
            if isinstance(init, list) and isinstance(tgt, list) and len(init) == N_DOF and len(tgt) == N_DOF:
                moved = [f"DOF{d} {init[d]:g}→{tgt[d]:g}" for d in self.dofs if init[d] != tgt[d]]
                text += "\nOptuna の初期姿勢→目標姿勢: " + "、".join(moved)
        return text


# ==============================================================================
# 比較（評価値）
# ==============================================================================
ANALYSIS_COLUMNS = (
    'DOF', 'J_i(1)', 'J_i(2)', '差 J_i(2)-J_i(1)', '評価値が小さい方', '変化率[%]', '改善/悪化',
    'Jに占める割合(1)[%]', 'Jに占める割合(2)[%]', 'RMS%(1)', 'RMS%(2)', 'RMS[count](1)', 'RMS[count](2)', '評価値の定義',
    '動作完了時間T[s]', '完了の許容ε[%]', '評価時間の固定値E[s]', '評価区間[s]', '評価点数',
    '変化前の目標値Pi(1)', '変化後の目標値Pf(1)', '変化前の目標値Pi(2)', '変化後の目標値Pf(2)',
    '基準の変化前の目標値Pi', '基準の変化後の目標値Pf', 'ΔP',
    '目標軌道の始点P0_ref', '目標軌道T1[s]', '目標軌道ωn[rad/s]', 'κ=T1·ωn', '完了拘束', '当てはめ方法',
    '基準の記録', '基準のJ_i（参考）', 'P0(1)', 'P0(2)', '欠測した配信(1)', '欠測した配信(2)',
    'FF(1)', 'FF(2)', 'データ1', 'データ2', '備考')


def _close(a, b, tol=POSTURE_TOL):
    return _num(a) and _num(b) and abs(float(a) - float(b)) <= tol


def compare_records(rec1, rec2, refs, T, eps, E, progress=None, selected=None, normalize=True):
    """2つの記録の評価値を、基準（FF=0）の記録から作った目標軌道で求める（optimize_optuna_ff_ROS2.py と同じ）

    rec1, rec2: Record、refs: 基準の Record のリスト、T: 動作完了時間 [s]、eps: 完了の許容（比。5% なら 0.05）、
    E: 評価時間の固定値 [s]、selected: 比較する自由度（None なら全部）、normalize: J_i を ΔP で割るか（True = Optuna と同じ）。
    戻り値 dict（rows: 自由度ごとの ANALYSIS_COLUMNS の辞書、overall、warnings、dofs、selected）
    """
    warns = []
    t_eval = eval_time(T, E)
    N_e = n_eval(T, E)
    datas = [rec1, rec2]
    # 評価する自由度: 基準で目標値が変わった自由度（DOF 21・25 は除く）
    ref0 = refs[0]
    selected = sorted(set(range(N_DOF)) - set(NO_FF_DOFS) if selected is None else {int(d) for d in selected})
    cand = [d for d in ref0.changed_dofs() if d not in NO_FF_DOFS and d in selected]
    for d in ref0.changed_dofs():
        if d in NO_FF_DOFS:
            warns.append(f"DOF {d}: FFの枠がなく制御していない自由度なので評価しません")
    unsel = [d for d in ref0.changed_dofs() if d not in NO_FF_DOFS and d not in selected]
    if unsel:
        warns.append(f"比較する自由度に選ばなかった DOF {', '.join(map(str, unsel))} は評価していません（全体の J にも入れていません）")
    ref_bad = {}
    for r in refs[1:]:
        for d in cand:
            if not (_close(r.meta['init_target'][d], ref0.meta['init_target'][d])
                    and _close(r.meta['target'][d], ref0.meta['target'][d])):
                ref_bad[d] = f"基準の記録どうしで目標値が違います（{ref0.name} と {r.name}）"
    for d, why in ref_bad.items():
        warns.append(f"DOF {d}: {why}")
    for r in refs:
        if r.meta.get('ff_label') != 'wo':
            raise ValueError(f"基準の記録 {r.name} は FF=0（wo）ではありません")
    Pi_ref = {d: float(ref0.meta['init_target'][d]) for d in cand}
    Pf_ref = {d: float(ref0.meta['target'][d]) for d in cand}

    def matches(rec, d):
        return _close(rec.meta['init_target'][d], Pi_ref[d]) and _close(rec.meta['target'][d], Pf_ref[d])

    # 配信の時間軸で切り出す（各記録の 0 秒 = 目標値の変化を受信した時刻を、Optuna の t_send とする）
    extracted = {}
    for rec in refs + datas:
        if id(rec) in extracted:
            continue
        dofs = [d for d in cand if matches(rec, d)]
        extracted[id(rec)] = extract_series(rec.pot_raw(), rec.meta['init_target'], rec.meta['target'], dofs, 0.0, T,
                                            history_s=HISTORY_S, eval_min=E)
    # 目標軌道（基準だけから作る。FFありのデータで作り直さない）
    specs, spec_err = {}, {}
    for i, d in enumerate(cand):
        if progress is not None:
            progress.put(('fit', i, len(cand), d))
        if d in ref_bad:
            spec_err[d] = ref_bad[d]
            continue
        ser = []
        for r in refs:
            s_all, _info, errs = extracted[id(r)]
            if d in s_all:
                ser.append(s_all[d])
            else:
                spec_err[d] = f"基準 {r.name}: {errs.get(d, '切り出せません')}"
        if d in spec_err:
            continue
        ys = [s.pot[s.win()] - s.Pf for s in ser]
        vs = [np.isfinite(y) for y in ys]
        if not any(v.any() for v in vs):
            spec_err[d] = "基準の評価区間に POT がありません"
            continue
        try:
            specs[d] = fit_target_constrained(d, ys, vs, [s.P0 for s in ser], Pi_ref[d], Pf_ref[d], T, eps)
        except Exception as exc:
            spec_err[d] = f"目標軌道を作れません: {exc}"
    if progress is not None:
        progress.put(('fit', len(cand), len(cand), None))
    rows = []
    J_both = {0: [], 1: []}
    J_dofs, J_only = [], {1: [], 2: []}     # J に入れた自由度（両方で評価できた）、片方だけで評価できた自由度
    ref_names = " / ".join(r.name for r in refs)
    for d in cand:
        notes = []
        row = {c: None for c in ANALYSIS_COLUMNS}
        row.update({'DOF': d, '動作完了時間T[s]': T, '完了の許容ε[%]': eps * 100.0, '評価時間の固定値E[s]': E,
                    '評価区間[s]': t_eval, '評価点数': N_e + 1, '基準の変化前の目標値Pi': Pi_ref[d],
                    '基準の変化後の目標値Pf': Pf_ref[d], 'ΔP': Pf_ref[d] - Pi_ref[d], '基準の記録': ref_names,
                    '評価値の定義': NORM_TEXT[normalize],
                    'データ1': rec1.name, 'データ2': rec2.name, 'FF(1)': rec1.legend(), 'FF(2)': rec2.legend()})
        for i, rec in enumerate(datas, start=1):
            row[f'変化前の目標値Pi({i})'] = rec.meta['init_target'][d]
            row[f'変化後の目標値Pf({i})'] = rec.meta['target'][d]
        if abs(Pf_ref[d] - Pi_ref[d]) < STEP_WARN:
            notes.append(f"|ΔP| = {abs(Pf_ref[d] - Pi_ref[d]):g} < {STEP_WARN:g} のため評価値がばらつきやすい")
        sp = specs.get(d)
        if sp is None:
            notes.append(spec_err.get(d, "目標軌道がありません"))
        else:
            row.update({'目標軌道の始点P0_ref': sp.P0_ref, '目標軌道T1[s]': sp.T1, '目標軌道ωn[rad/s]': sp.wn,
                        'κ=T1·ωn': sp.kappa, '完了拘束': 'あり' if sp.active else 'なし', '当てはめ方法': sp.method})
            notes += sp.warnings
            refJ = []
            for r in refs:
                s = extracted[id(r)][0].get(d)
                if s is not None:
                    refJ.append(error_metric(s.pot[s.win()], sp.seq(), s.dP, np.isfinite(s.pot[s.win()]), normalize))
            row['基準のJ_i（参考）'] = " / ".join(f"{v:.6g}" for v in refJ)
        Js = [float('nan'), float('nan')]
        for i, rec in enumerate(datas):
            tag = f"({i + 1})"
            if not matches(rec, d):
                notes.append(f"データ{i + 1} の目標値（{rec.meta['init_target'][d]}→{rec.meta['target'][d]}）が基準"
                             f"（{Pi_ref[d]:g}→{Pf_ref[d]:g}）と違うため評価しません")
                continue
            s_all, _info, errs = extracted[id(rec)]
            s = s_all.get(d)
            if s is None:
                notes.append(f"データ{i + 1}: {errs.get(d, '切り出せません')}")
                continue
            row['P0' + tag] = s.P0
            miss = s.missing()
            row['欠測した配信' + tag] = miss
            if miss:
                short = rec.duration < t_eval
                notes.append(f"データ{i + 1}: 評価区間 {N_e + 1} 配信のうち {miss} 配信を受信できていません"
                             + (f"（記録時間 {rec.duration:g} s が評価区間 {t_eval:g} s より短い）" if short else "")
                             + "。受信できた配信だけで評価しました")
            if sp is None:
                continue
            pot = s.pot[s.win()]
            Js[i] = error_metric(pot, sp.seq(), s.dP, np.isfinite(pot), normalize)
            row['J_i' + tag] = Js[i]
            sq = squared_error(pot, sp.seq(), np.isfinite(pot))
            row['RMS[count]' + tag] = math.sqrt(sq) if np.isfinite(sq) else float('nan')
            row['RMS%' + tag] = row['RMS[count]' + tag] / abs(s.dP) * 100.0 if s.dP else float('nan')
        if np.isfinite(Js[0]) and np.isfinite(Js[1]):
            row['差 J_i(2)-J_i(1)'] = Js[1] - Js[0]
            row['評価値が小さい方'] = ('1' if Js[0] < Js[1] else '2' if Js[1] < Js[0] else '同じ')
            row['変化率[%]'], row['改善/悪化'] = change_percent(Js[0], Js[1])
            J_both[0].append(Js[0])
            J_both[1].append(Js[1])
            J_dofs.append(d)
        elif np.isfinite(Js[0]) or np.isfinite(Js[1]):
            J_only[1 if np.isfinite(Js[0]) else 2].append(d)
        row['備考'] = " / ".join(notes)
        rows.append(row)
        for n_ in notes:
            warns.append(f"DOF {d}: {n_}")
    # データどうしの条件の違い（目標値・評価区間中の別の目標値・ADRCパラメータ）
    for d in range(N_DOF):
        a = (rec1.meta['init_target'][d], rec1.meta['target'][d])
        b = (rec2.meta['init_target'][d], rec2.meta['target'][d])
        if not (_close(a[0], b[0]) and _close(a[1], b[1])) and (_num(a[0]) or _num(b[0])):
            warns.append(f"DOF {d}: データ1とデータ2で目標値が違います（1: {a[0]}→{a[1]}, 2: {b[0]}→{b[1]}）。"
                         "単純には比較できません")
    for i, rec in enumerate(datas, start=1):
        for t, v in zip(rec.cmd_t, rec.cmd_v):
            if 0 < t <= t_eval:
                diff = [d for d in range(N_DOF) if _num(rec.meta['target'][d]) and np.isfinite(v[d])
                        and abs(v[d] - rec.meta['target'][d]) > 1e-6]
                if diff:
                    warns.append(f"データ{i}: 評価区間中（{t:.2f} s）に別の目標値を受信しています（DOF "
                                 f"{', '.join(map(str, diff[:8]))}{' …' if len(diff) > 8 else ''}）")
                    break
        if rec.meta.get('ff_label') not in ('w', 'wo'):
            warns.append(f"データ{i}: FFの有無が分かりません")
    p1, p2 = rec1.meta.get('adrc') or {}, rec2.meta.get('adrc') or {}
    adrc_diff = []
    for key in sorted(set(p1) & set(p2), key=int):
        a, b = p1[key], p2[key]
        if any(_num(x) and _num(y) and abs(x - y) > 1e-6 * max(1.0, abs(x)) for x, y in zip(a, b)):
            adrc_diff.append(key)
    if adrc_diff:
        warns.append(f"DOF {', '.join(adrc_diff)}: データ1とデータ2で ADRC パラメータが違います。単純には比較できません")
    for i, rec in enumerate(datas, start=1):
        for w in rec.meta.get('warnings', []):
            warns.append(f"データ{i}（記録時の警告）: {w}")
    # 全体の評価値 J = 平均(J_i)（optimize_optuna_ff_ROS2.py の J と同じ定義）。2つのデータで同じ自由度の組を比べるため、
    # 両方で評価できた自由度だけで平均する
    J1 = float(np.mean(J_both[0])) if J_both[0] else float('nan')
    J2 = float(np.mean(J_both[1])) if J_both[1] else float('nan')
    pct, judge = change_percent(J1, J2)
    overall = {'n': len(J_dofs), 'dofs': J_dofs, 'only': J_only, 'J1': J1, 'J2': J2,
               'diff': J2 - J1 if np.isfinite(J1) and np.isfinite(J2) else float('nan'),
               'smaller': ('1' if J1 < J2 else '2' if J2 < J1 else '同じ') if np.isfinite(J1) and np.isfinite(J2) else '',
               'pct': pct, 'judge': judge}
    # J に占める各自由度の割合 J_i / Σ J_i（J に入れた自由度だけ。J = 平均(J_i) なので J_i / (n·J) と同じ）
    sums = [float(np.sum(J_both[0])), float(np.sum(J_both[1]))]
    for row in rows:
        if row['DOF'] in J_dofs:
            for i in (1, 2):
                if sums[i - 1] > 0:
                    row[f'Jに占める割合({i})[%]'] = row[f'J_i({i})'] / sums[i - 1] * 100.0
    for i in (1, 2):
        if J_only[i]:
            warns.append(f"データ{i} だけで評価できた自由度（DOF {', '.join(map(str, J_only[i]))}）は、全体の J に入れていません"
                         "（2つのデータで同じ自由度の組の J を比べるため）")
    targets = {d: {'T1': sp.T1, 'wn': sp.wn, 'y0_ref': sp.y0_ref, 'Pf': sp.Pf, 'N_e': sp.N_e} for d, sp in specs.items()}
    j0s = [{b: inf['j0'] for b, inf in extracted[id(rec)][1].items()} for rec in datas]
    return {'rows': rows, 'overall': overall, 'warnings': merge_dof_warnings(warns), 'dofs': cand, 'T': T, 'eps': eps,
            'E': E, 't_eval': t_eval, 'N_e': N_e, 'targets': targets, 'j0': j0s, 'selected': selected,
            'normalize': normalize}


# ------------------------------------------------------------------------------
# 比較グラフの時間軸（配信番号）・目標軌道
# ------------------------------------------------------------------------------
def board_tick_j0(rec, board, t_tk, V, j_tk):
    """ボードの配信番号の t=0（j0）。extract_series と同じく、目標値が変わった自由度のボードの目標値（POTdesired）が
    round(新しい目標値) になった最初の配信（目標値の受信から 1 s 以内）。そういう自由度がないボードは、
    記録開始（0 秒）以後の最初の配信。戻り値 (j0, 'echo' | 'first')"""
    count = BOARD_COUNT[board]
    offset = next(o for b, o, _c in BOARD_LAYOUT if b == board)
    hits = []
    for local in range(count):
        d = offset + local
        pi, pf = rec.meta['init_target'][d], rec.meta['target'][d]
        if not (_num(pi) and _num(pf)) or int(round(pf)) == int(round(pi)):
            continue
        hit = np.flatnonzero((V[:, count + local] == int(round(pf))) & (t_tk >= -0.05))
        if len(hit) and t_tk[hit[0]] <= 1.0:
            hits.append(int(j_tk[hit[0]]))
    if hits:
        return min(hits), 'echo'
    after = np.flatnonzero(t_tk >= 0)
    return (int(j_tk[after[0]]) if len(after) else int(j_tk[-1]) + 1), 'first'


def tick_record(rec, j0_override=None):
    """比較グラフ用に、各トピックの時刻を配信番号の時間軸 (j − j0) × 0.010 s に置き換えた Record を返す。

    POT（/tk）の受信に assign_ticks で配信番号を振り（評価の extract_series と同じ番号）、PWM・z3・FF は align_ticks で
    tk の番号にそろえる（optimize_optuna_ff_ROS2.py の best_trial_grid.csv と同じ方法）。j0 は j0_override（評価で
    求めた {board: j0}）があればそれを使い、なければ board_tick_j0 で決める。
    """
    series, notes = {}, []
    for board, _offset, count in BOARD_LAYOUT:
        t_tk, V = rec.kind_data(board, 'pot')
        if len(t_tk) < 2:
            series[board] = {}
            continue
        j_tk, _u, _nd = assign_ticks(t_tk)
        if j0_override and board in j0_override:
            j0 = int(j0_override[board])
        else:
            j0, how = board_tick_j0(rec, board, t_tk, V, j_tk)
            if how != 'echo':
                notes.append(f"{board}: 目標値が変わった自由度がないため、記録開始後の最初の配信を t=0 にしました")
        s = {'pot': ((j_tk - j0) * TICK_DT, V)}
        for kind in ('pwm', 'z3', 'ffrecv'):
            t, W = rec.kind_data(board, kind)
            if len(t) == 0:
                continue
            jk = align_ticks(t, t_tk, j_tk)
            s[kind] = ((jk - j0) * TICK_DT, W)
            if kind == 'pwm':
                tf, F = rec.kind_data(board, 'ff')
                if len(tf) == len(t):
                    s['ff'] = ((jk - j0) * TICK_DT, F)
        series[board] = s
    meta = dict(rec.meta)
    meta['tick_notes'] = notes
    return Record(meta, series, rec.cmd_t, rec.cmd_v)


def tick_end(rec):
    """記録の最後の配信の時刻（配信番号の時間軸）"""
    ends = [float(np.max(rec.series[b]['pot'][0])) for b, _o, _c in BOARD_LAYOUT
            if 'pot' in rec.series.get(b, {}) and len(rec.series[b]['pot'][0])]
    return max(ends) if ends else rec.duration


def target_curve(tp, K):
    """目標軌道（3次遅れ系の自由応答）の POT を j = 0..K で返す (t, POT)。j ≤ N_e は評価に使った数列と同じ値"""
    K = max(int(K), int(tp['N_e']))
    y = target_states(tp['T1'], tp['wn'], tp['y0_ref'], TICK_DT, K)[:, 0] + tp['Pf']
    return np.arange(K + 1) * TICK_DT, y


def pot_on_ticks(rec, dof, K):
    """配信番号の時間軸にした記録の POT を j = 0..K に並べる（受信できなかった配信は NaN）"""
    out = np.full(K + 1, np.nan)
    t, v = rec.dof_series(dof, 'pot')
    if len(t):
        k = np.round(np.asarray(t) / TICK_DT).astype(int)
        sel = (k >= 0) & (k <= K)
        out[k[sel]] = v[sel]
    return out


def write_target_csv(path, result, tr1, tr2, xmax):
    """目標軌道.csv: 配信番号 j（= j − j0）ごとの目標軌道（3次遅れ系）の POT と、評価に使った2つのデータの POT。
    評価区間（j ≤ N_e）の行だけで J_i = mean((POT − Target)/ΔP)² を計算すると評価値.csv の J_i になる"""
    K = int(np.floor(xmax / TICK_DT + 1e-9))
    dofs = [d for d in result['dofs'] if d in result['targets']]
    header = ['j（= j − j0）', 't [s]', '評価区間']
    cols = []
    for d in dofs:
        header += [f'Target{d}', f'POT{d}(1)', f'POT{d}(2)']
        cols += [target_curve(result['targets'][d], K)[1][:K + 1], pot_on_ticks(tr1, d, K), pot_on_ticks(tr2, d, K)]
    with open(path, 'w', newline='', encoding=CSV_ENCODING) as f:
        w = csv.writer(f)
        w.writerow(header)
        for j in range(K + 1):
            w.writerow([j, f"{j * TICK_DT:.2f}", 1 if j <= result['N_e'] else 0]
                       + [_fmt_any(float(c[j])) for c in cols])


def merge_dof_warnings(warns):
    """「DOF d: 文」で文が同じ警告を「DOF 0, 1, 2: 文」に1つにまとめる（順序は最初に出た順）"""
    groups, order = {}, []
    for w in warns:
        m = re.match(r'DOF (\d+): (.*)$', w, re.S)
        key = m.group(2) if m else w
        if key not in groups:
            groups[key] = []
            order.append(key)
        if m:
            groups[key].append(m.group(1))
    out = []
    for key in order:
        dofs = list(dict.fromkeys(groups[key]))
        out.append(f"DOF {', '.join(dofs)}: {key}" if dofs else key)
    return out


J_COLUMNS = ('項目', 'J(1)', 'J(2)', '差 J(2)-J(1)', 'Jが小さい方', '変化率[%]', '改善/悪化', '評価値の定義',
             '平均した自由度数', '平均した自由度', '比較に選んだ自由度', '動作完了時間T[s]', '完了の許容ε[%]',
             '評価時間の固定値E[s]', 'データ1', 'データ2', '備考')


def change_percent(a, b):
    """変化率 (b − a)/a × 100 [%] と、その判定（負なら改善）"""
    if not (np.isfinite(a) and np.isfinite(b)) or a <= 0:
        return float('nan'), ''
    pct = (b - a) / a * 100.0
    return pct, ('改善' if pct < 0 else '悪化' if pct > 0 else '同じ')


def j_summary_row(result):
    """全体の評価値 J の行（J_COLUMNS の辞書）"""
    ov = result['overall']
    rows = result['rows']
    note = ("J = mean(J_i)（optimize_optuna_ff_ROS2.py の J と同じく J_i の平均）。2つのデータの両方で評価できた自由度で平均"
            + ("" if result.get('normalize', True) else "。J_i は ΔP で割らない値なので、ΔP の大きい自由度ほど J で重くなる"))
    for i in (1, 2):
        if ov['only'][i]:
            note += f"。データ{i} だけで評価できた DOF {', '.join(map(str, ov['only'][i]))} は含めない"
    return {'項目': 'J', 'J(1)': ov['J1'], 'J(2)': ov['J2'], '差 J(2)-J(1)': ov['diff'], 'Jが小さい方': ov['smaller'],
            '変化率[%]': ov['pct'], '改善/悪化': ov['judge'],
            '評価値の定義': NORM_TEXT[result.get('normalize', True)], '平均した自由度数': ov['n'],
            '平均した自由度': ' '.join(map(str, ov['dofs'])),
            '比較に選んだ自由度': ' '.join(map(str, result.get('selected', []))), '動作完了時間T[s]': result['T'],
            '完了の許容ε[%]': result['eps'] * 100.0, '評価時間の固定値E[s]': result['E'],
            'データ1': rows[0]['データ1'] if rows else '', 'データ2': rows[0]['データ2'] if rows else '', '備考': note}


def write_analysis_csv(path, result):
    """評価値.csv: 自由度ごとの J_i の表 → 全体の評価値 J の表 → 警告"""
    with open(path, 'w', newline='', encoding=CSV_ENCODING) as f:
        w = csv.writer(f)
        w.writerow(ANALYSIS_COLUMNS)
        for r in result['rows']:
            w.writerow([_fmt_any(r.get(c)) for c in ANALYSIS_COLUMNS])
        w.writerow([])
        w.writerow(['全体の評価値 J（小さいほど目標軌道への追従誤差が小さい）'])
        w.writerow(J_COLUMNS)
        js = j_summary_row(result)
        w.writerow([_fmt_any(js.get(c)) for c in J_COLUMNS])
        w.writerow([])
        w.writerow(['警告'])
        for text in result['warnings']:
            w.writerow([text])


def compare_folder_name(rec1, rec2):
    return f"cf-{rec1.name}-{rec2.name}"


def run_comparison_job(params, progress):
    """比較の計算と保存（GUIとは別プロセスで動かす）。params = {csv1, csv2, refs, T, eps, E, folder}"""
    try:
        rec1 = read_record_csv(params['csv1'])
        rec2 = read_record_csv(params['csv2'])
        refs = [read_record_csv(p) for p in params['refs']]
        progress.put(('stage', "目標軌道を当てはめています"))
        result = compare_records(rec1, rec2, refs, params['T'], params['eps'], params['E'], progress,
                                 params.get('dofs'), params.get('normalize', True))
        folder = params['folder']
        os.makedirs(os.path.join(folder, ANALYSIS_DIR), exist_ok=True)
        write_analysis_csv(os.path.join(folder, ANALYSIS_DIR, ANALYSIS_CSV), result)
        save_share_chart(os.path.join(folder, ANALYSIS_DIR, SHARE_PNG), result)
        # 比較グラフ・目標軌道.csv は評価と同じ配信番号の時間軸（j0 も評価で求めた値）
        tr1, tr2 = tick_record(rec1, result['j0'][0]), tick_record(rec2, result['j0'][1])
        xmax = max(tick_end(tr1), tick_end(tr2))
        write_target_csv(os.path.join(folder, TARGET_CSV), result, tr1, tr2, xmax)
        progress.put(('result', result))
        progress.put(('stage', "比較グラフを作っています"))
        K = int(np.floor(xmax / TICK_DT + 1e-9))
        targets = {d: target_curve(tp, K) for d, tp in result['targets'].items()}
        write_compare_graphs(tr1, tr2, folder, progress, xmax, targets)
        progress.put(('done', folder))
    except Exception:
        progress.put(('error', traceback.format_exc()))


# ==============================================================================
# 記録（1回分）
# ==============================================================================
# 移植元: GUI_android_tk_FFsetting.py:1480-1575 @105a28e（変更: 変化前の履歴・FFの受信・ADRCパラメータ・欠測の検出を足した）
class Recorder:
    """待機 → 記録中 → 終了（または取消）。受信コールバック（ROSスレッド）がデータを渡す"""
    ARMED, RECORDING, DONE, CANCELLED = range(4)

    def __init__(self, duration):
        self.duration = float(duration)     # 記録時間 [s]。記録中もGUIから変更される
        self.state = self.ARMED
        self.lock = threading.Lock()
        self.t0 = None                      # 記録開始の時刻（time.monotonic()）
        self.t_end = None                   # 記録を終えた時刻（記録開始からの秒）
        self.start = None                   # 記録開始の日時
        self.init_target = None
        self.target = None
        self.ff = None                      # 記録開始の時点でGUIが送信済みだったFF
        self.adrc = None                    # 記録開始の時点のADRCパラメータ {DOF文字列: [kp,kd,ic,λ]}
        self.data = {board: {kind: [] for kind in KINDS} for board, _, _ in BOARD_LAYOUT}
        self.cmd = []                       # 受信した目標値メッセージ (記録開始からの秒, 値)
        self.rows = 0
        self.warnings = []                  # 記録中に気付いた問題（受信の途切れなど）

    def trigger(self, t, init_target, target, values, ff, history, cmd_history, adrc):
        """記録を開始する。t = 目標値が変わったメッセージの受信時刻（0 秒）、history = 変化前の受信 {(board, kind): [(時刻, 値)]}"""
        with self.lock:
            if self.state != self.ARMED:
                return
            self.t0 = t
            self.start = datetime.now()
            self.init_target = list(init_target)
            self.target = list(target)
            self.ff = ff
            self.adrc = adrc
            for (board, kind), items in history.items():
                self.data[board][kind].extend((tt - t, v) for tt, v in items if t - HISTORY_S <= tt < t)
            self.cmd.extend((tt - t, v) for tt, v in cmd_history if t - HISTORY_S <= tt < t)
            self.cmd.append((0.0, tuple(values)))
            self.state = self.RECORDING

    def elapsed(self):
        return 0.0 if self.t0 is None else time.monotonic() - self.t0

    def add(self, kind, board, t, values):
        with self.lock:
            if self.state != self.RECORDING:
                return
            tr = t - self.t0
            if tr > self.duration:
                self._finish_locked(self.duration)
                return
            if tr >= 0:
                self.data[board][kind].append((tr, values))
                self.rows += 1

    def add_cmd(self, t, values):
        with self.lock:
            if self.state == self.RECORDING and 0 <= t - self.t0 <= self.duration:
                self.cmd.append((t - self.t0, tuple(values)))

    def add_warning(self, text):
        with self.lock:
            self.warnings.append(text)

    def _finish_locked(self, t_end):
        if self.state == self.RECORDING:
            self.t_end = float(min(t_end, self.duration))
            self.state = self.DONE

    def finish(self):
        """記録を終える（待機中なら取り消す）"""
        with self.lock:
            if self.state == self.ARMED:
                self.state = self.CANCELLED
            else:
                self._finish_locked(self.elapsed())

    def to_record(self):
        """Record にする（内部FF・FFの有無の判定はまだ入れない）"""
        series = {}
        for board, _offset, count in BOARD_LAYOUT:
            out = {}
            for kind in KINDS:
                lst = sorted(self.data[board][kind], key=lambda x: x[0])
                width = kind_width(kind, count)
                out[kind] = (np.array([x[0] for x in lst], dtype=float),
                             np.array([x[1] for x in lst], dtype=float).reshape(len(lst), width) if lst
                             else np.zeros((0, width)))
            series[board] = out
        pad = lambda v: [float(v[i]) if i < len(v) else float('nan') for i in range(N_DOF)]
        cmd = sorted(self.cmd, key=lambda x: x[0])
        duration = float(self.t_end if self.t_end is not None else self.duration)
        warns = list(self.warnings)
        for board, _offset, _count in BOARD_LAYOUT:       # 記録中の受信の欠測（PC の時計で 0.1 s を超えた間隔）
            t = series[board]['pot'][0]
            t = t[(t >= 0) & (t <= duration)]
            if len(t) == 0:
                warns.append(f"{board}: 記録中に実測POT（/tk）を1回も受信できませんでした")
                continue
            d = np.diff(np.concatenate([[0.0], t]))
            big = np.flatnonzero(d > 0.1)
            if len(big):
                warns.append(f"{board}: 受信が 0.1 s を超えて途切れたところが {len(big)} か所あります"
                             f"（最大 {d[big].max():.2f} s、{t[big[0]]:.2f} s 付近など）。その区間は欠測です")
            if duration - t[-1] > 0.5:
                warns.append(f"{board}: 記録の終わり（{duration:g} s）の {duration - t[-1]:.1f} s 前から受信していません")
        meta = {
            'name': '',
            'start': self.start.isoformat(timespec='seconds'),
            'stamp': self.start.strftime('%Y%m%d_%H%M%S'),
            'duration': duration,
            'history_s': HISTORY_S,
            'init_target': list(self.init_target),
            'target': list(self.target),
            'ff_table': [list(map(float, row)) for row in self.ff['table']],
            'ff_setting': self.ff['setting'],
            'ff_source': self.ff['source'],
            'ff_status': self.ff['status'],
            'adrc': self.adrc or {},
            'warnings': warns,
        }
        return Record(meta, series, [c[0] for c in cmd], [pad(c[1]) for c in cmd])


# ==============================================================================
# ROS2 ノード
# ==============================================================================
# GUIが最後に送信したFFの状態（ボードごと）。どれも実機の値を確かめたものではない
# 移植元: GUI_android_tk_FFsetting.py:1582-1592 @105a28e
SENT_STATUS_TEXT = {
    'startup':   "起動時の前提値 FF=0（まだ送信していません・未確認）",
    'sent':      "GUIが送信（受信確認なし）",
    'nosub':     "送信時に受信側が見つかりませんでした（届いていない可能性）",
    'restart':   "実機の起動を検出したため FF=0 とみなしています（未確認）",
    'foreign':   "他のプログラムがFFを送信しました（このGUIの値と違う可能性）",
}
SENT_STATUS_SHORT = {'startup': "未送信(0前提)", 'sent': "送信済", 'nosub': "受信側なし",
                     'restart': "実機起動→0", 'foreign': "他プログラム"}


# 移植元: GUI_android_tk_FFsetting.py:1595-1755 @105a28e（変更: 変化前の履歴・FFの受信・ADRCパラメータの受信を足した）
class CfNode(Node):
    def __init__(self):
        super().__init__('ff_compare_gui_node')
        self.lock = threading.Lock()
        self.events = queue.SimpleQueue()       # GUIへの通知 ('board_first'|'board_restart'|'foreign_ff', board番号, ...)

        self.board_desired = [None] * N_DOF     # ボードが使っている目標値（実測トピックの [6..11]）
        self.last_cmd_values = None             # 目標値トピックで最後に受信した値
        self.last_rx = {}                       # board名 → 実測トピックを最後に受信した時刻
        self.rx_kind = {}                       # (board名, 種類) → 最後に受信した時刻
        self.session = None                     # Recorder（GUIスレッドが設定する）
        self.rings = {(board, kind): deque(maxlen=RING_LEN) for board, _o, _c in BOARD_LAYOUT for kind in KINDS}
        self.cmd_ring = deque(maxlen=256)
        self.adrc = {}                          # DOF → [kp, kd, ic, λ]（Teensy が返す現在値）

        # GUIが最後に送信したFF（実機の値ではない）
        self.sent_table = zero_ff_table()
        self.sent_setting = [None] * N_FF
        self.sent_source = ['FFなし（起動時の前提値）'] * N_FF
        self.sent_status = {k: ['startup', None] for k in range(1, len(BOARD_LAYOUT) + 1)}
        self.own_ff = deque(maxlen=64)          # 自分の送信 (時刻, ボード番号, float32に丸めた値)

        self.ff_pubs = {}
        for k, (board, offset, count) in enumerate(BOARD_LAYOUT, start=1):
            self.ff_pubs[k] = self.create_publisher(Float32MultiArray, TOPIC_FF.format(k), 10)
            self.create_subscription(
                UInt16MultiArray, TOPIC_POT.format(k),
                lambda msg, b=board, o=offset, c=count, k=k: self.pot_cb(msg, k, b, o, c), SENSOR_QOS_DEPTH)
            for kind, topic in (('pwm', TOPIC_PWM), ('z3', TOPIC_Z3), ('ffrecv', TOPIC_FF_OUT)):
                self.create_subscription(
                    Float32MultiArray, topic.format(k),
                    lambda msg, kd=kind, b=board, c=count: self.float_cb(msg, kd, b, c), SENSOR_QOS_DEPTH)
            self.create_subscription(Float32MultiArray, TOPIC_PARAM.format(k),
                                     lambda msg, o=offset, c=count: self.param_cb(msg, o, c), 10)
            # 他のプログラムがFFを送信したことに気付くためだけに受信する（値は使わない）
            self.create_subscription(Float32MultiArray, TOPIC_FF.format(k), lambda msg, k=k: self.ff_cb(msg, k), 10)
        self.create_subscription(Float32MultiArray, TOPIC_TARGET, self.target_cb, 10)

    # ---- 受信 ----
    def pot_cb(self, msg, k, board, offset, count):
        t = time.monotonic()
        data = msg.data
        row = [float('nan')] * (2 * count)
        for i in range(count):
            if i < len(data):
                row[i] = float(data[i])
            if 6 + i < len(data):
                v = float(data[6 + i])
                self.board_desired[offset + i] = v
                row[count + i] = v
        prev = self.last_rx.get(board)
        self.last_rx[board] = t
        if prev is None:
            self.events.put(('board_first', k))
        elif t - prev > BOARD_RESTART_GAP:
            # 配信が途切れて再開した → 実機が起動したとみなす（実機のFFは起動時 0）
            self.reset_board_ff(k, 'restart')
            self.events.put(('board_restart', k, t - prev))
        self._store('pot', board, t, row)

    def float_cb(self, msg, kind, board, count):
        t = time.monotonic()
        data = msg.data
        self._store(kind, board, t, [float(data[i]) if i < len(data) else float('nan') for i in range(count)])

    def _store(self, kind, board, t, row):
        self.rx_kind[(board, kind)] = t
        self.rings[(board, kind)].append((t, row))
        s = self.session
        if s is not None and s.state == Recorder.RECORDING:
            s.add(kind, board, t, row)

    def param_cb(self, msg, offset, count):
        """ADRCパラメータの現在値（[2+4i+p] = 自由度 i の kp, kd, input_coef, λ0。変更できない自由度は NaN）"""
        data = msg.data
        for i in range(count):
            vals = [float(data[2 + 4 * i + p]) if 2 + 4 * i + p < len(data) else float('nan') for p in range(4)]
            if all(math.isfinite(v) for v in vals):
                self.adrc[offset + i] = vals

    def target_cb(self, msg):
        t = time.monotonic()
        values = tuple(msg.data)
        with self.lock:
            s = self.session
            if s is not None:
                if s.state == Recorder.ARMED and self.target_changed(values):
                    prev = self.last_cmd_values
                    init = []
                    for i in range(N_DOF):
                        if prev is not None and i < len(prev):
                            init.append(float(prev[i]))
                        else:
                            init.append(self.board_desired[i])
                    target = [float(values[i]) if i < len(values) else None for i in range(N_DOF)]
                    history = {key: list(ring) for key, ring in self.rings.items()}
                    adrc = {str(d): list(v) for d, v in self.adrc.items()}
                    s.trigger(t, init, target, values, self._sent_snapshot_locked(), history, list(self.cmd_ring), adrc)
                elif s.state == Recorder.RECORDING:
                    s.add_cmd(t, values)
            self.cmd_ring.append((t, values))
            self.last_cmd_values = values

    # 移植元: GUI_android_tk.py:872-884 @105a28e
    def target_changed(self, values):
        """受信した目標値が、直前の目標値と1つの自由度でも違うか"""
        prev = self.last_cmd_values
        for i in range(min(N_DOF, len(values))):
            if prev is not None:
                if i >= len(prev) or values[i] != prev[i]:
                    return True
            else:
                # 起動後まだ目標値トピックを受信していない → ボードが使っている目標値（整数）と比べる
                bd = self.board_desired[i]
                if bd is None or abs(values[i] - bd) >= 0.5:
                    return True
        return False

    def ff_cb(self, msg, k):
        vals = to_float32_tuple(msg.data)
        now = time.monotonic()
        with self.lock:
            own = any(b == k and v == vals and now - t0 < 10.0 for t0, b, v in self.own_ff)
            if not own:
                self.sent_status[k] = ['foreign', datetime.now().strftime('%H:%M:%S')]
        if not own:
            self.events.put(('foreign_ff', k))

    # ---- FFの送信 ----
    def board_data(self, k, table):
        """ボード k へ送る36要素（6スロット × [a,b,c,d,e,T]）"""
        board, offset, _count = BOARD_LAYOUT[k - 1]
        data = []
        for s in range(FF_SLOTS):
            if s < FF_BOARD_DOFS[board]:
                data.extend(float(x) for x in table[FF_INDEX[offset + s]])
            else:
                data.extend([0.0, 0.0, 0.0, 0.0, 0.0, FF_T_DEFAULT])
        return data

    def publish_ff(self, table, settings, sources, statuses):
        """statuses = {ボード番号: 状態}。そのボードの6スロットをまとめて送る"""
        stamp = datetime.now().strftime('%H:%M:%S')
        with self.lock:
            for k, status in statuses.items():
                msg = Float32MultiArray()
                msg.data = self.board_data(k, table)
                self.own_ff.append((time.monotonic(), k, to_float32_tuple(msg.data)))
                self.ff_pubs[k].publish(msg)
                board, offset, _ = BOARD_LAYOUT[k - 1]
                for s in range(FF_BOARD_DOFS[board]):
                    n = FF_INDEX[offset + s]
                    self.sent_table[n] = table[n]
                    self.sent_setting[n] = None if settings[n] is None else dict(settings[n])
                    self.sent_source[n] = sources[n]
                self.sent_status[k] = [status, stamp]

    def reset_board_ff(self, k, status):
        board, offset, _ = BOARD_LAYOUT[k - 1]
        with self.lock:
            for s in range(FF_BOARD_DOFS[board]):
                n = FF_INDEX[offset + s]
                self.sent_table[n][:5] = 0.0
                self.sent_setting[n] = None
                self.sent_source[n] = 'FFなし（実機の起動を検出したため 0 とみなす）'
            self.sent_status[k] = [status, datetime.now().strftime('%H:%M:%S')]

    def _sent_snapshot_locked(self):
        return {'table': self.sent_table.copy().tolist(),
                'setting': [None if s is None else dict(s) for s in self.sent_setting],
                'source': list(self.sent_source),
                'status': {str(k): list(v) for k, v in self.sent_status.items()}}

    def sent_snapshot(self):
        with self.lock:
            return self._sent_snapshot_locked()

    def ff_subscriber_present(self, k):
        """ボード k のFFトピックに、このノード以外の受信側（実機）がいるか"""
        try:
            infos = self.get_subscriptions_info_by_topic(TOPIC_FF.format(k))
        except Exception:
            return False
        me = (self.get_name(), self.get_namespace())
        return any((i.node_name, i.node_namespace) != me for i in infos)


# ==============================================================================
# 設定ファイル
# ==============================================================================
# JSON: {"record_dir", "record_duration", "optuna_dir", "dof": {DOF: {T, ylo, yhi, saved}}, "cmp": {...}}
#   FFあり/なしは保存しない（GUI_android_tk_FFsetting.py と同じく、起動時は必ず FFなし から。極値は saved に覚えておく）
def default_dof_state():
    return {'T': FF_T_DEFAULT, 'ylo': Y_LIMIT_DEFAULT[0], 'yhi': Y_LIMIT_DEFAULT[1], 'saved': None, 'exact': None,
            'source': '手動'}


def default_settings():
    return {'record_dir': None, 'record_duration': RECORD_DURATION_DEFAULT, 'optuna_dir': '',
            'dof': {str(d): default_dof_state() for d in FF_DOF_MAP},
            'cmp': {'csv1': '', 'csv2': '', 'refs': [], 'T': '1.0', 'eps': f"{EPS_DEFAULT:g}", 'E': f"{EVAL_MIN_S:g}",
                    'dir': ''}}


def load_settings(path=None):
    """設定ファイルを読む。戻り値 (settings, 警告文 or None)。読めない項目は既定値のまま"""
    path = path or SETTINGS_FILE
    settings = default_settings()
    if not os.path.exists(path):
        return settings, None
    try:
        with open(path, encoding='utf-8') as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise ValueError("JSONの最上位がオブジェクトではありません")
    except (OSError, ValueError) as e:
        return settings, f"{path} を読めないため、既定値を使います。\n{e}"
    bad = []
    if isinstance(data.get('record_dir'), str) and data['record_dir']:
        settings['record_dir'] = data['record_dir']
    v = data.get('record_duration')
    if _num(v) and RECORD_DURATION_MIN <= v <= RECORD_DURATION_MAX:
        settings['record_duration'] = float(v)
    elif v is not None:
        bad.append('record_duration')
    if isinstance(data.get('optuna_dir'), str):
        settings['optuna_dir'] = data['optuna_dir']
    for d in FF_DOF_MAP:
        st = (data.get('dof') or {}).get(str(d))
        if not isinstance(st, dict):
            continue
        out = settings['dof'][str(d)]
        if _num(st.get('T')) and FF_T_RANGE[0] <= st['T'] <= FF_T_RANGE[1]:
            out['T'] = float(st['T'])
        if _num(st.get('ylo')) and _num(st.get('yhi')) and -PWM_LIMIT <= st['ylo'] < 0 < st['yhi'] <= PWM_LIMIT:
            out['ylo'], out['yhi'] = float(st['ylo']), float(st['yhi'])
        sv = st.get('saved')
        if isinstance(sv, dict) and all(_num(sv.get(k)) for k in ('t1', 'y1', 't2', 'y2')):
            out['saved'] = {k: float(sv[k]) for k in ('t1', 'y1', 't2', 'y2')}
        ex = st.get('exact')
        if (isinstance(ex, dict) and all(_num(ex.get(k)) for k in ('T', 't1', 'y1', 't2', 'y2'))
                and isinstance(ex.get('coeffs'), list) and len(ex['coeffs']) == 5 and all(_num(v) for v in ex['coeffs'])):
            out['exact'] = {k: float(ex[k]) for k in ('T', 't1', 'y1', 't2', 'y2')}
            out['exact']['coeffs'] = [float(v) for v in ex['coeffs']]
        if isinstance(st.get('source'), str) and st['source']:
            out['source'] = st['source']
    cmp_ = data.get('cmp')
    if isinstance(cmp_, dict):
        for key in ('csv1', 'csv2', 'T', 'eps', 'E', 'dir'):
            if isinstance(cmp_.get(key), str):
                settings['cmp'][key] = cmp_[key]
        dofs = cmp_.get('dofs')
        if isinstance(cmp_.get('normalize'), bool):
            settings['cmp']['normalize'] = cmp_['normalize']
        if isinstance(dofs, list) and all(isinstance(d, int) and not isinstance(d, bool) and 0 <= d < N_DOF for d in dofs):
            settings['cmp']['dofs'] = sorted(set(dofs) - set(NO_FF_DOFS))
        if isinstance(cmp_.get('refs'), list) and all(isinstance(p, str) for p in cmp_['refs']):
            settings['cmp']['refs'] = list(cmp_['refs'])
    if bad:
        return settings, f"{path} の次の項目が不正なため既定値にしました: {', '.join(bad)}"
    return settings, None


def save_settings(settings, path=None):
    """設定ファイルへ書く（一時ファイルに書いてから置き換える）"""
    path = path or SETTINGS_FILE
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(settings, f, indent=2, ensure_ascii=False)
    os.replace(tmp, path)


# ==============================================================================
# 1自由度のFF設定
# ==============================================================================
# 移植元: GUI_android_tk_FFsetting.py:1939-1991 @105a28e（変更: 出どころ source と、Optuna が送った係数 exact を持たせ、
#         First・手動の x の扱いは外した）
class DofFF:
    """1自由度のFF設定。exact は Optuna から読み込んだ試行で実際に送られた float32 の係数
    {'T','t1','y1','t2','y2','coeffs'}。T・極値がその試行のままの間は、極値から作り直した係数ではなくこれを送る
    （t2 が T に近く y2 が小さい試行では、極値から作り直すと波形が最大 1 PWM ほど変わるため。試行を正確に再現する）。
    """

    def __init__(self, dof, T=FF_T_DEFAULT, ylo=Y_LIMIT_DEFAULT[0], yhi=Y_LIMIT_DEFAULT[1], saved=None, exact=None,
                 source='手動'):
        self.dof = dof
        self.enabled = False            # 起動時は必ずFFなし（極値・係数すべて0）
        self.T = float(T)
        self.t1 = self.y1 = self.t2 = self.y2 = 0.0
        self.ylo, self.yhi = float(ylo), float(yhi)
        self.saved = None if saved is None else dict(saved)     # 最後に設定した極値（FFなしにしても覚えておく）
        self.source = str(source)       # 極値の出どころ（手動 / Optuna の試行）
        self.exact = None if exact is None else dict(exact)
        self.listeners = []
        self._check = None

    def values(self):
        return {'t1': self.t1, 'y1': self.y1, 't2': self.t2, 'y2': self.y2}

    def setting(self):
        return {'T': self.T, 't1': self.t1, 'y1': self.y1, 't2': self.t2, 'y2': self.y2}

    def state(self):
        return {'T': self.T, 'ylo': self.ylo, 'yhi': self.yhi,
                'saved': None if self.saved is None else dict(self.saved),
                'exact': None if self.exact is None else dict(self.exact), 'source': self.source}

    def exact_valid(self):
        """Optuna が送った係数をそのまま使えるか（FFあり・T と極値が読み込んだときのまま）"""
        e = self.exact
        return (self.enabled and e is not None
                and all(e[k] == getattr(self, k) for k in ('T', 't1', 'y1', 't2', 'y2')))

    def update(self, **kw):
        for key, v in kw.items():
            if key == 'enabled':
                self.enabled = bool(v)
            elif key == 'source':
                self.source = str(v)
            elif key == 'exact':
                self.exact = None if v is None else dict(v)
            else:
                setattr(self, key, float(v))
        if self.enabled:
            vals = self.values()
            if any(v != 0 for v in vals.values()):
                self.saved = vals
        else:
            self.t1 = self.y1 = self.t2 = self.y2 = 0.0
        self._check = None
        for fn in list(self.listeners):
            fn(self)

    def check(self):
        if self._check is None:
            if self.enabled:
                self._check = check_ff(self.T, self.t1, self.y1, self.t2, self.y2, self.ylo, self.yhi)
                if self._check.ok and self.exact_valid():
                    self._check.coeffs = np.asarray(self.exact['coeffs'], dtype=float)
            else:
                c = FFCheck()
                c.ok = FF_T_RANGE[0] <= self.T <= FF_T_RANGE[1]
                c.coeffs = np.zeros(5)
                if not c.ok:
                    c.errors.append(f"FF終了時刻 T は {FF_T_RANGE[0]:g}〜{FF_T_RANGE[1]:g} s にしてください")
                self._check = c
        return self._check

    def coeffs6(self):
        """送信する [a,b,c,d,e,T]。条件を満たさなければ None"""
        c = self.check()
        return [float(v) for v in c.coeffs] + [self.T] if c.ok else None


def write_entry(entry, text, state=None):
    cur = str(entry.cget('state'))
    if cur == tk.DISABLED:
        entry.config(state=tk.NORMAL)
    entry.delete(0, tk.END)
    entry.insert(0, text)
    entry.config(state=state or cur)
    entry.written = text


def nice_ticks(lo, hi, max_n):
    span = hi - lo
    if not span > 0:
        return [lo]
    raw = span / max(1, max_n)
    mag = 10 ** math.floor(math.log10(raw))
    step = next(m * mag for m in (1, 2, 5, 10) if m * mag >= raw)
    start = math.ceil(lo / step - 1e-9) * step
    ticks = []
    v = start
    while v <= hi + 1e-9 * span:
        ticks.append(round(v, 10))
        v += step
    return ticks


def _fmt_cell(v, digits=6):
    if v is None:
        return ""
    if isinstance(v, (float, np.floating)):
        return "" if not np.isfinite(v) else f"{v:.{digits}g}"
    return str(v)


# ==============================================================================
# GUI本体
# ==============================================================================
COORD_KEYS = ('t1', 'y1', 't2', 'y2')
ENTRY_KEYS = ('T', 'ylo', 'yhi') + COORD_KEYS
TABLE_HEAD = ("DOF", "ボード", "FF", "T [s]", "y下限", "y上限", "t1 [s]", "y1", "t2 [s]", "y2", "判定",
              "極値の出どころ", "①J最小", "②J_i最小", "③試行", "", "④群best", "GUIが最後に送信した値")
MODE_TEXT = {'1': "①J最小", '2': "②J_i最小", '3': "③指定", '4': "④グループの best"}


class CfApp:
    STATE_STYLE = {
        'idle':      ("停止中", "#909090"),
        'armed':     ("待機中", "#e08000"),
        'recording': ("● 記録中", "#d00000"),
        'saving':    ("保存中", "#7030a0"),
        'done':      ("記録終了", "#208020"),
        'error':     ("エラー", "#700000"),
        'noros':     ("ROS 2 なし", "#700000"),
    }

    def __init__(self, node, settings, settings_path=None, settings_warning=None, ros_error=None):
        global UI_FONT
        self.node = node
        self.settings = settings
        self.settings_path = settings_path or SETTINGS_FILE
        self.ros_error = ros_error
        self.models = {d: DofFF(d, **settings['dof'][str(d)]) for d in FF_DOF_MAP}
        self.rows = {}
        self.sel_dof = FF_DOF_MAP[0]
        self.optuna = None
        self.saves = []                 # 記録の保存の別プロセス
        self.cmp_job = None             # 比較の別プロセス
        self.cmp_records = {}           # 'csv1' / 'csv2' → Record（読み込んだ記録）
        self.cmp_ticks = {}             # 'csv1' / 'csv2' → 配信番号の時間軸にした Record（比較グラフの表示用）
        self.cmp_targets = {}           # DOF → 目標軌道のパラメータ（比較が終わると入る）
        self.cmp_jtext = ''             # 全体の評価値 J の表示
        self.last_record = None
        self.stalled = set()
        self.prefs_job = None
        self.refresh_jobs = {}
        self.warn_lines = deque(maxlen=6)
        self.scroll_canvases = []
        self.sent_snap = node.sent_snapshot() if node is not None else {
            'table': zero_ff_table().tolist(), 'setting': [None] * N_FF, 'source': ['FFなし'] * N_FF,
            'status': {str(k): ['startup', None] for k in range(1, len(BOARD_LAYOUT) + 1)}}

        self.root = tk.Tk()
        setup_japanese_font(self.root)
        self.root.title(WINDOW_TITLE)
        self.root.geometry(window_geometry_for(initial_monitor(self.root, list_monitors())))
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        self.tab_rec = tk.Frame(self.notebook)
        self.tab_cmp = tk.Frame(self.notebook)
        self.notebook.add(self.tab_rec, text="記録")
        self.notebook.add(self.tab_cmp, text="比較")
        self.build_record_tab()
        self.build_compare_tab()
        for d in FF_DOF_MAP:
            self.models[d].listeners.append(self.on_model_change)
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            self.root.bind_all(seq, self.on_mousewheel)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        for d in FF_DOF_MAP:
            self.refresh_row(d)
        self.select_dof(self.sel_dof)
        self.refresh_sent_views()
        if node is None:
            self.show_record_state('noros', f"ROS 2 に接続できないため記録・FF送信はできません（比較タブは使えます）: {ros_error}")
            for b in (self.record_button, self.send_button, self.zero_button, self.group_send_button):
                b.config(state=tk.DISABLED)
        else:
            self.show_record_state('idle', "「● 記録開始」を押してから、GUI_android_tk.py などで目標値を送ってください")
        if settings.get('optuna_dir'):
            self.root.after(200, lambda: self.load_optuna(quiet=True))
        if settings['cmp'].get('csv1') or settings['cmp'].get('csv2'):
            self.root.after(300, lambda: self.load_compare(quiet=True))
        self.poll()
        if settings_warning:
            self.root.after(400, lambda: messagebox.showwarning(TITLE_SETTINGS, settings_warning))

    # =========================================================
    # 共通
    # =========================================================
    def make_scrollable(self, parent):
        """縦横スクロールできる領域（中の Frame を返す）"""
        canvas = tk.Canvas(parent, highlightthickness=0)
        vs = ttk.Scrollbar(parent, orient=tk.VERTICAL, command=canvas.yview)
        hs = ttk.Scrollbar(parent, orient=tk.HORIZONTAL, command=canvas.xview)
        inner = tk.Frame(canvas)
        canvas.create_window((0, 0), window=inner, anchor='nw')
        inner.bind('<Configure>', lambda e: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.configure(yscrollcommand=vs.set, xscrollcommand=hs.set)
        canvas.grid(row=0, column=0, sticky='nsew')
        vs.grid(row=0, column=1, sticky='ns')
        hs.grid(row=1, column=0, sticky='ew')
        parent.rowconfigure(0, weight=1)
        parent.columnconfigure(0, weight=1)
        self.scroll_canvases.append(canvas)
        return inner

    def on_mousewheel(self, event):
        try:
            w = self.root.winfo_containing(event.x_root, event.y_root)
        except (KeyError, tk.TclError):
            return
        while w is not None:
            if w in self.scroll_canvases:
                if event.num == 4 or getattr(event, 'delta', 0) > 0:
                    w.yview_scroll(-2, 'units')
                else:
                    w.yview_scroll(2, 'units')
                return
            w = getattr(w, 'master', None)

    def add_warning(self, text):
        self.warn_lines.append(f"{datetime.now().strftime('%H:%M:%S')}  {text}")
        self.warn_label.config(text="\n".join(self.warn_lines))

    def save_prefs_later(self, delay_ms=1000):
        if self.prefs_job is not None:
            self.root.after_cancel(self.prefs_job)
        self.prefs_job = self.root.after(delay_ms, self.save_prefs)

    def save_prefs(self):
        self.prefs_job = None
        self.settings['dof'] = {str(d): m.state() for d, m in self.models.items()}
        self.settings['optuna_dir'] = self.optuna_dir_var.get().strip()
        c = self.settings['cmp']
        c['csv1'], c['csv2'] = self.csv_vars['csv1'].get().strip(), self.csv_vars['csv2'].get().strip()
        c['refs'] = list(self.ref_list.get(0, tk.END))
        for key in ('T', 'eps', 'E'):
            c[key] = self.cmp_vars[key].get().strip()
        c['dir'] = self.cmp_dir_var.get().strip()
        c['dofs'] = self.selected_cmp_dofs()
        c['normalize'] = bool(self.cmp_norm_var.get())
        try:
            save_settings(self.settings, self.settings_path)
        except OSError as e:
            print(f"設定ファイルを保存できません: {e}", file=sys.stderr)

    def busy(self):
        s = self.node.session if self.node is not None else None
        return s is not None or bool(self.saves) or self.cmp_job is not None

    # =========================================================
    # 記録タブ
    # =========================================================
    def build_record_tab(self):
        tab = self.tab_rec
        bar = tk.Frame(tab, bd=1, relief=tk.GROOVE)
        bar.pack(fill=tk.X, padx=4, pady=(4, 2))
        r1 = tk.Frame(bar)
        r1.pack(fill=tk.X, padx=4, pady=2)
        self.record_button = tk.Button(r1, text="● 記録開始", width=16, command=self.on_record_button,
                                       font=(UI_FONT, 10, 'bold'))
        self.record_button.pack(side=tk.LEFT)
        tk.Label(r1, text="  記録時間 [s]").pack(side=tk.LEFT)
        self.duration_var = tk.StringVar(value=f"{self.settings['record_duration']:g}")
        self.duration_spin = tk.Spinbox(r1, from_=RECORD_DURATION_MIN, to=RECORD_DURATION_MAX, increment=1, width=6,
                                        textvariable=self.duration_var)
        self.duration_spin.pack(side=tk.LEFT)
        self.duration_var.trace_add("write", lambda *a: self.on_duration_change())
        tk.Label(r1, text=f"（{RECORD_DURATION_MIN:g}〜{RECORD_DURATION_MAX:g} s。比較で評価するなら評価区間 max(1.5T, E) より長く）"
                          "   保存先").pack(side=tk.LEFT)
        self.record_dir_var = tk.StringVar(value=self.settings['record_dir'] or os.getcwd())
        tk.Entry(r1, textvariable=self.record_dir_var, width=40).pack(side=tk.LEFT)
        tk.Button(r1, text="参照", command=self.choose_record_dir).pack(side=tk.LEFT, padx=2)

        r2 = tk.Frame(bar)
        r2.pack(fill=tk.X, padx=4, pady=2)
        self.record_state = tk.Label(r2, text="", width=14, fg='white', font=(UI_FONT, 12, 'bold'))
        self.record_state.pack(side=tk.LEFT)
        self.progress = ttk.Progressbar(r2, length=220, maximum=1000)
        self.progress.pack(side=tk.LEFT, padx=6)
        self.record_detail = tk.Label(r2, text="", font=(UI_FONT, 10, 'bold'), anchor='w', justify='left')
        self.record_detail.pack(side=tk.LEFT, fill=tk.X, expand=True)

        r3 = tk.Frame(bar)
        r3.pack(fill=tk.X, padx=4, pady=2)
        self.send_button = tk.Button(r3, text="FFパラメータ送信（全自由度）", command=self.send_ff,
                                     bg='#d8f0d8', font=(UI_FONT, 10, 'bold'))
        self.send_button.pack(side=tk.LEFT)
        self.zero_button = tk.Button(r3, text="FFを全て0にして送信", command=self.send_zero, bg='#f4e4c4')
        self.zero_button.pack(side=tk.LEFT, padx=6)
        self.sent_summary = tk.Label(r3, text="", font=(UI_FONT, 9), anchor='w', justify='left')
        self.sent_summary.pack(side=tk.LEFT, fill=tk.X, expand=True)

        r4 = tk.Frame(bar)
        r4.pack(fill=tk.X, padx=4, pady=2)
        tk.Label(r4, text="Optuna の結果フォルダ").pack(side=tk.LEFT)
        self.optuna_dir_var = tk.StringVar(value=self.settings.get('optuna_dir', ''))
        tk.Entry(r4, textvariable=self.optuna_dir_var, width=60).pack(side=tk.LEFT)
        tk.Button(r4, text="参照", command=self.choose_optuna_dir).pack(side=tk.LEFT, padx=2)
        tk.Button(r4, text="読み込む", command=self.load_optuna).pack(side=tk.LEFT, padx=2)
        tk.Label(r4, text="   全自由度に一括で:").pack(side=tk.LEFT)
        tk.Button(r4, text="① J 最小の試行", command=lambda: self.apply_optuna_all('1')).pack(side=tk.LEFT, padx=1)
        tk.Button(r4, text="② 各自由度の J_i 最小の試行", command=lambda: self.apply_optuna_all('2')).pack(side=tk.LEFT, padx=1)
        tk.Label(r4, text="  ③ 試行番号").pack(side=tk.LEFT)
        self.bulk_trial_var = tk.StringVar()
        tk.Entry(r4, textvariable=self.bulk_trial_var, width=6).pack(side=tk.LEFT)
        tk.Button(r4, text="③ 読み込む", command=lambda: self.apply_optuna_all('3')).pack(side=tk.LEFT, padx=1)
        tk.Button(r4, text="④ グループごとの best", command=lambda: self.apply_optuna_all('4')).pack(side=tk.LEFT,
                                                                                               padx=(6, 1))
        self.group_send_button = tk.Button(r4, text="④ グループごとの best を送信", command=self.send_group_best,
                                           bg='#d8f0d8', font=(UI_FONT, 9, 'bold'))
        self.group_send_button.pack(side=tk.LEFT, padx=(6, 1))
        self.optuna_info = tk.Label(bar, text="Optuna の結果: 未読み込み（読み込んでも送信はしません。送るときは「FFパラメータ送信」）",
                                    font=(UI_FONT, 9), anchor='w', justify='left', fg='#304080')
        self.optuna_info.pack(fill=tk.X, padx=8)
        self.warn_label = tk.Label(bar, text="", font=(UI_FONT, 9), anchor='w', justify='left', fg='#b04000')
        self.warn_label.pack(fill=tk.X, padx=8)

        paned = ttk.PanedWindow(tab, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=4, pady=2)
        left = tk.Frame(paned)
        right = tk.Frame(paned)
        paned.add(left, weight=3)
        paned.add(right, weight=2)
        self.build_table(self.make_scrollable(left))
        self.build_detail(right)

    def build_table(self, table):
        for j, text in enumerate(TABLE_HEAD):
            tk.Label(table, text=text, font=(UI_FONT, 9, 'bold'), bg='#e8e8e8', relief=tk.RIDGE
                     ).grid(row=0, column=j, sticky='nsew')
        for d in range(N_DOF):
            r = d + 1
            k, board, local, _c = DOF_BOARD[d]
            lab = tk.Label(table, text=f"DOF {d}", font=(UI_FONT, 9, 'bold'), cursor='hand2')
            lab.grid(row=r, column=0, sticky='w', padx=2)
            tk.Label(table, text=f"{board}-{local}", font=(UI_FONT, 8)).grid(row=r, column=1, sticky='w')
            if d not in FF_INDEX:
                tk.Label(table, text="FFの枠なし（board4・5 の index3。FFは送らない）", fg='#808080',
                         font=(UI_FONT, 8)).grid(row=r, column=2, columnspan=10, sticky='w')
                continue
            lab.bind("<Button-1>", lambda e, d=d: self.select_dof(d))
            w = {}
            w['enabled'] = tk.BooleanVar(value=False)
            tk.Checkbutton(table, variable=w['enabled'], command=lambda d=d: self.on_enable(d)
                           ).grid(row=r, column=2)
            for j, key in enumerate(ENTRY_KEYS, start=3):
                e = tk.Entry(table, width=8, justify='right')
                e.grid(row=r, column=j, padx=1)
                e.bind("<Return>", lambda ev, d=d, key=key: self.on_row_entry(d, key))
                e.bind("<FocusOut>", lambda ev, d=d, key=key: self.on_row_entry(d, key))
                e.bind("<FocusIn>", lambda ev, d=d: self.select_dof(d))
                w[key] = e
            w['status'] = tk.Label(table, text="", width=10, anchor='w', font=(UI_FONT, 9))
            w['status'].grid(row=r, column=10, sticky='w')
            w['source'] = tk.Label(table, text="", width=34, anchor='w', font=(UI_FONT, 8), justify='left',
                                   wraplength=260)
            w['source'].grid(row=r, column=11, sticky='w')
            tk.Button(table, text="①", width=2, command=lambda d=d: self.apply_optuna_one(d, '1')
                      ).grid(row=r, column=12)
            tk.Button(table, text="②", width=2, command=lambda d=d: self.apply_optuna_one(d, '2')
                      ).grid(row=r, column=13)
            w['trial'] = tk.Entry(table, width=5, justify='right')
            w['trial'].grid(row=r, column=14)
            tk.Button(table, text="③", width=2, command=lambda d=d: self.apply_optuna_one(d, '3')
                      ).grid(row=r, column=15)
            tk.Button(table, text="④", width=2, command=lambda d=d: self.apply_optuna_one(d, '4')
                      ).grid(row=r, column=16)
            w['sent'] = tk.Label(table, text="", anchor='w', font=(UI_FONT, 8), justify='left')
            w['sent'].grid(row=r, column=17, sticky='w', padx=4)
            self.rows[d] = w

    def build_detail(self, parent):
        top = tk.LabelFrame(parent, text="選択中の自由度のFF", font=(UI_FONT, 10, 'bold'), padx=4, pady=2)
        top.pack(fill=tk.X)
        self.detail_title = tk.Label(top, text="", font=(UI_FONT, 10, 'bold'), anchor='w')
        self.detail_title.pack(fill=tk.X)
        self.wave = tk.Canvas(top, height=170, bg='white', highlightthickness=1, highlightbackground='#c8c8c8')
        self.wave.pack(fill=tk.X)
        self.wave.bind("<Configure>", lambda e: self.draw_wave())
        self.detail_text = tk.Label(top, text="", anchor='w', justify='left', font=(UI_FONT, 9), wraplength=600)
        self.detail_text.pack(fill=tk.X)
        prev = tk.LabelFrame(parent, text="最後の記録（保存したグラフと同じ内容）", font=(UI_FONT, 10, 'bold'), padx=4, pady=2)
        prev.pack(fill=tk.BOTH, expand=True, pady=(4, 0))
        row = tk.Frame(prev)
        row.pack(fill=tk.X)
        tk.Label(row, text="表示する自由度").pack(side=tk.LEFT)
        self.rec_prev_dof = ttk.Combobox(row, values=[f"DOF {d}" for d in range(N_DOF)], width=8, state='readonly')
        self.rec_prev_dof.current(0)
        self.rec_prev_dof.pack(side=tk.LEFT)
        self.rec_prev_dof.bind("<<ComboboxSelected>>", lambda e: self.draw_record_preview())
        tk.Button(row, text="比較の1つ目に入れる", command=lambda: self.last_to_compare('csv1')).pack(side=tk.LEFT, padx=4)
        tk.Button(row, text="比較の2つ目に入れる", command=lambda: self.last_to_compare('csv2')).pack(side=tk.LEFT)
        self.rec_prev_label = tk.Label(prev, text="（まだ記録していません）", anchor='w', font=(UI_FONT, 9))
        self.rec_prev_label.pack(fill=tk.X)
        self.rec_prev_frame = tk.Frame(prev)
        self.rec_prev_frame.pack(fill=tk.BOTH, expand=True)
        self.rec_fig = None

    # ---------- 表の表示 ----------
    def on_model_change(self, model):
        d = model.dof
        if self.refresh_jobs.get(d) is None:
            self.refresh_jobs[d] = self.root.after(20, lambda d=d: self.refresh_row(d))
        self.save_prefs_later()

    def is_unsent(self, d):
        n = FF_INDEX[d]
        c = self.models[d].coeffs6()
        sent = np.asarray(self.sent_snap['table'][n], dtype=float)
        if c is None:
            return True
        c = np.asarray(c, dtype=float)
        if not np.any(c[:5] != 0) and not np.any(sent[:5] != 0):
            return False
        return not np.allclose(c, sent, rtol=0.0, atol=1e-12)

    def unsent_dofs(self):
        return [d for d in FF_DOF_MAP if self.is_unsent(d)]

    def refresh_row(self, d):
        self.refresh_jobs[d] = None
        m, w = self.models[d], self.rows[d]
        w['enabled'].set(m.enabled)
        focus = self.focused()
        vals = {'T': f"{m.T:g}", 'ylo': f"{m.ylo:g}", 'yhi': f"{m.yhi:g}"}
        for key in COORD_KEYS:
            v = getattr(m, key)
            vals[key] = (f"{v:.4f}" if key in ('t1', 't2') else f"{v:.3f}") if m.enabled else "0"
        for key in ENTRY_KEYS:
            e = w[key]
            state = tk.NORMAL if (m.enabled or key in ('T', 'ylo', 'yhi')) else tk.DISABLED
            if e is not focus:
                write_entry(e, vals[key], state)
                e.config(bg='white')
            else:
                e.config(state=state)
        chk = m.check()
        if not m.enabled:
            w['status'].config(text="FFなし" if chk.ok else "× T範囲外", fg='#404040' if chk.ok else '#c00000')
        else:
            w['status'].config(text="○ 送信可" if chk.ok else "× 条件違反", fg='#006000' if chk.ok else '#c00000')
        w['source'].config(text=m.source if m.enabled else "")
        self.refresh_sent_row(d)
        if d == self.sel_dof:
            self.refresh_detail()

    def refresh_sent_row(self, d):
        n = FF_INDEX[d]
        s = self.sent_snap['setting'][n]
        sent = self.sent_snap['table'][n]
        if s is None or not np.any(np.asarray(sent[:5]) != 0):
            text = "FFなし"
        else:
            text = f"t1={s['t1']:.3f}, y1={s['y1']:.2f}, t2={s['t2']:.3f}, y2={s['y2']:.2f}, T={s['T']:g}"
        unsent = self.is_unsent(d)
        self.rows[d]['sent'].config(text=text + ("   ※未送信の変更あり" if unsent else ""),
                                    fg='#a05000' if unsent else '#303030')

    def refresh_sent_views(self):
        snap = self.sent_snap
        parts = []
        for k in range(1, len(BOARD_LAYOUT) + 1):
            status, stamp = snap['status'][str(k)]
            parts.append(f"board{k}: {SENT_STATUS_SHORT[status]}{' ' + stamp if stamp else ''}")
        self.sent_summary.config(text="GUIの送信状態（受信確認なし）  " + "  |  ".join(parts))
        for d in FF_DOF_MAP:
            self.refresh_sent_row(d)
        self.refresh_detail()

    def focused(self):
        try:
            return self.root.focus_get()
        except (KeyError, tk.TclError):
            return None

    def select_dof(self, d):
        if d not in FF_INDEX:
            return
        self.sel_dof = d
        for dd, w in self.rows.items():
            w['status'].config(bg='#fff4c0' if dd == d else self.root.cget('bg'))
        self.refresh_detail()

    def refresh_detail(self):
        if not hasattr(self, 'detail_title'):
            return
        d = self.sel_dof
        m = self.models[d]
        k, board, local, _c = DOF_BOARD[d]
        self.detail_title.config(text=f"DOF {d}（{board} スロット{local}、FF番号 {FF_INDEX[d] + 1}）")
        chk = m.check()
        n = FF_INDEX[d]
        status, stamp = self.sent_snap['status'][str(k)]
        lines = [format_formula(chk.coeffs if m.enabled else np.zeros(5), m.T)]
        if m.enabled:
            lines.append("○ 条件を満たしています（送信できます）" if chk.ok else "× エラー: " + " / ".join(chk.errors))
            lines.append(f"極値の出どころ: {m.source}"
                         + ("（係数は Optuna が送った float32 の値をそのまま送ります）" if m.exact_valid() else ""))
        else:
            lines.append("FFなし（極値・係数すべて0）。送信すると [0,0,0,0,0,T] を送ります")
        lines.append(f"GUIが最後に送信した値の出どころ: {self.sent_snap['source'][n]}   "
                     f"[{SENT_STATUS_TEXT[status]}{' ' + stamp if stamp else ''}]")
        lines.append("条件: 0 < t1 < t2 < T、y1·y2 < 0、下限 ≤ y1, y2 ≤ 上限、(0,T) 内の極値がちょうど2個（t1, t2）で重根でない")
        self.detail_text.config(text="\n".join(lines))
        self.draw_wave()

    def draw_wave(self):
        """選択中の自由度のFF（設定中・最後に送信した値）"""
        c = self.wave
        c.delete('all')
        w, h = c.winfo_width(), int(c.cget('height'))
        if w < 50:
            return
        d = self.sel_dof
        m = self.models[d]
        chk = m.check()
        left, right, top, bottom = 52, 10, 8, 20
        pw, ph = w - left - right, h - top - bottom
        sent = self.sent_snap['table'][FF_INDEX[d]]
        x1 = max(m.T, float(sent[5]) if np.any(np.asarray(sent[:5]) != 0) else 0.0) * 1.08
        tt = np.linspace(0, x1, max(200, pw))
        curves = []
        if m.enabled and chk.coeffs is not None:
            curves.append((np.where(tt < m.T, ff_poly(chk.coeffs, tt), 0.0), '#1050d0' if chk.ok else '#d02020', None,
                           "設定中のFF"))
        if np.any(np.asarray(sent[:5]) != 0):
            curves.append((np.where(tt < sent[5], ff_poly(sent[:5], tt), 0.0), '#909090', (4, 3), "GUIが最後に送信したFF"))
        lo, hi = m.ylo, m.yhi
        for f, *_ in curves:
            fin = f[np.isfinite(f)]
            if fin.size:
                lo, hi = min(lo, fin.min()), max(hi, fin.max())
        pad = 0.08 * (hi - lo)
        lo, hi = lo - pad, hi + pad

        def X(v):
            return left + pw * np.asarray(v, dtype=float) / x1

        def Y(v):
            return top + ph * (1 - (np.clip(np.asarray(v, dtype=float), lo, hi) - lo) / (hi - lo))

        font = (UI_FONT, 8)
        for v in nice_ticks(lo, hi, max(2, ph // 30)):
            c.create_line(left, float(Y(v)), left + pw, float(Y(v)), fill='#e8e8e8')
            c.create_text(left - 4, float(Y(v)), anchor='e', text=f"{v:.6g}", font=font)
        for v in nice_ticks(0, x1, max(2, pw // 60)):
            c.create_line(float(X(v)), top, float(X(v)), top + ph, fill='#e8e8e8')
            c.create_text(float(X(v)), top + ph + 3, anchor='n', text=f"{v:.6g}", font=font)
        c.create_rectangle(left, top, left + pw, top + ph, outline='#808080')
        c.create_line(left, float(Y(0)), left + pw, float(Y(0)), fill='#a0a0a0')
        c.create_line(float(X(m.T)), top, float(X(m.T)), top + ph, fill='#808080', dash=(2, 3))
        c.create_text(float(X(m.T)) + 2, top + ph - 2, anchor='sw', text=f"T={m.T:g}s", font=font, fill='#606060')
        for v, name in ((m.yhi, "上限"), (m.ylo, "下限")):
            c.create_line(left, float(Y(v)), left + pw, float(Y(v)), fill='#d07000', dash=(3, 3))
        ly = top + 4
        for f, color, dash, label in curves:
            pts = np.column_stack((X(tt), Y(f))).ravel().tolist()
            c.create_line(*pts, fill=color, width=2, dash=dash)
            c.create_text(left + pw - 4, ly, anchor='ne', text=label, font=font, fill=color)
            ly += 13
        if m.enabled:
            for i, (tx, ty) in enumerate(((m.t1, m.y1), (m.t2, m.y2)), start=1):
                xx, yy = float(X(tx)), float(Y(ty))
                c.create_oval(xx - 4, yy - 4, xx + 4, yy + 4, fill='#ffd040', outline='black')
                c.create_text(xx + 6, yy, anchor='w', text=f"極値{i} ({tx:.3f}, {ty:.2f})", font=font)

    # ---------- 表の入力 ----------
    def on_enable(self, d):
        m = self.models[d]
        w = self.rows[d]
        self.select_dof(d)
        if not w['enabled'].get():
            m.update(enabled=False)
            return
        if m.saved is not None:
            v = m.saved
            m.update(enabled=True, t1=v['t1'], y1=v['y1'], t2=v['t2'], y2=v['y2'])
        else:
            # 前に設定した極値がなければ t(t−T/2)(t−T) 形の極値の時刻と ±DEFAULT_FF_PEAK から始める（FFsetting と同じ既定）
            T = m.T
            m.update(enabled=True, t1=T * (3 - math.sqrt(3)) / 6, y1=DEFAULT_FF_PEAK,
                     t2=T * (3 + math.sqrt(3)) / 6, y2=-DEFAULT_FF_PEAK, source='手動')

    def on_row_entry(self, d, key):
        m = self.models[d]
        e = self.rows[d][key]
        text = e.get().strip()
        if text == getattr(e, 'written', None):
            return
        if key in COORD_KEYS and not m.enabled:
            return
        try:
            v = float(text)
            if not math.isfinite(v):
                raise ValueError
        except ValueError:
            e.config(bg='#ffc0c0')
            return
        if key == 'T' and not FF_T_RANGE[0] <= v <= FF_T_RANGE[1]:
            e.config(bg='#ffc0c0')
            self.add_warning(f"DOF {d}: T は {FF_T_RANGE[0]:g}〜{FF_T_RANGE[1]:g} s にしてください")
            return
        if key == 'ylo' and not (-PWM_LIMIT <= v < 0):
            e.config(bg='#ffc0c0')
            self.add_warning(f"DOF {d}: y の下限は −{PWM_LIMIT:g} 以上 0 未満にしてください")
            return
        if key == 'yhi' and not (0 < v <= PWM_LIMIT):
            e.config(bg='#ffc0c0')
            self.add_warning(f"DOF {d}: y の上限は 0 より大きく {PWM_LIMIT:g} 以下にしてください")
            return
        e.config(bg='white')
        e.written = text
        if abs(getattr(m, key) - v) <= 1e-12:
            return
        kw = {key: v}
        if key not in ('ylo', 'yhi') and m.enabled:
            kw['source'] = '手動'
        m.update(**kw)

    # ---------- 記録 ----------
    def show_record_state(self, state, detail):
        text, color = self.STATE_STYLE[state]
        self.record_state.config(text=text, bg=color)
        self.record_detail.config(text=detail, fg=color if state in ('recording', 'error') else 'black')
        if self.node is None:
            return
        if state == 'armed':
            self.record_button.config(text="待機を取消", state=tk.NORMAL)
        elif state == 'recording':
            self.record_button.config(text="■ ここで記録終了", state=tk.NORMAL)
        else:
            self.record_button.config(text="● 記録開始", state=tk.NORMAL)
        s = self.node.session
        recording = s is not None and s.state == Recorder.RECORDING
        self.send_button.config(state=tk.DISABLED if recording else tk.NORMAL)
        self.group_send_button.config(state=tk.DISABLED if recording else tk.NORMAL)
        self.zero_button.config(state=tk.DISABLED if recording else tk.NORMAL)

    def read_duration(self):
        try:
            v = float(self.duration_var.get())
        except ValueError:
            return None
        return v if RECORD_DURATION_MIN <= v <= RECORD_DURATION_MAX else None

    def on_duration_change(self):
        v = self.read_duration()
        self.duration_spin.config(bg='white' if v is not None else '#ffc0c0')
        if v is None:
            return
        s = self.node.session if self.node is not None else None
        if s is not None:
            s.duration = v
        if v != self.settings['record_duration']:
            self.settings['record_duration'] = v
            self.save_prefs_later()

    def choose_record_dir(self):
        path = filedialog.askdirectory(initialdir=self.record_dir_var.get() or os.getcwd(), title="Select save folder")
        if path:
            self.record_dir_var.set(path)
            self.settings['record_dir'] = path
            self.save_prefs_later()

    def base_dir(self, var, key=None):
        base = os.path.abspath(os.path.expanduser(var.get().strip() or os.getcwd()))
        os.makedirs(base, exist_ok=True)
        if not os.access(base, os.W_OK):
            raise OSError(f"{base} に書き込み権限がありません")
        if key is not None and base != self.settings.get(key):
            self.settings[key] = base
            self.save_prefs_later()
        return base

    def on_record_button(self):
        if self.node is None:
            return
        s = self.node.session
        if s is None:
            duration = self.read_duration()
            if duration is None:
                messagebox.showerror(TITLE_RECORD, f"記録時間は {RECORD_DURATION_MIN:g}〜{RECORD_DURATION_MAX:g} s で入力してください")
                return
            try:
                self.base_dir(self.record_dir_var, 'record_dir')
            except OSError as e:
                messagebox.showerror(TITLE_RECORD, f"保存先を使えません:\n{e}")
                return
            self.stalled = set()
            self.node.session = Recorder(duration)
            unsent = self.unsent_dofs()
            note = (f"   ※DOF {', '.join(map(str, unsent))} のFF設定は未送信です（この記録には送信済みの値が使われます）"
                    if unsent else "")
            if not self.node.last_rx:
                note += "   ※ボードからまだ受信していません"
            self.show_record_state('armed', "目標値を送ってください。目標値が1自由度でも変わった時点を 0 秒として記録します" + note)
        elif s.state == Recorder.ARMED:
            s.finish()
        elif s.state == Recorder.RECORDING:
            s.finish()

    def poll_record(self):
        if self.node is None:
            return
        s = self.node.session
        if s is None:
            if not self.saves:
                self.progress['value'] = 0
            return
        if s.state == Recorder.RECORDING:
            el = s.elapsed()
            if el >= s.duration:
                s.finish()
            now = time.monotonic()
            for board, _o, _c in BOARD_LAYOUT:          # 受信の途切れ（ROS 2 の切断・ボードの停止）
                last = self.node.last_rx.get(board)
                gap = now - last if last is not None else float('inf')
                if gap > STALL_WARN_S and board not in self.stalled:
                    self.stalled.add(board)
                    text = (f"{board}: 記録中に受信が途切れています（記録 {el:.1f} s 時点で "
                            f"{'一度も受信していません' if last is None else f'{gap:.1f} s 受信なし'}）。その区間は欠測になります")
                    s.add_warning(text)
                    self.add_warning(text)
                elif gap <= STALL_WARN_S:
                    self.stalled.discard(board)
            self.progress['value'] = int(1000 * min(1.0, el / max(s.duration, 1e-6)))
            self.show_record_state('recording', f"記録 {min(el, s.duration):.1f} / {s.duration:g} s（受信 {s.rows} 件）"
                                                "   記録中はFFを送信できません")
        elif s.state == Recorder.CANCELLED:
            self.node.session = None
            self.show_record_state('idle', "記録の待機を取り消しました")
        elif s.state == Recorder.DONE:
            self.node.session = None
            self.finish_record(s)

    def finish_record(self, s):
        try:
            rec = s.to_record()
            compute_internal_ff(rec)
            judge_ff_label(rec)
            base = self.base_dir(self.record_dir_var, 'record_dir')
            folder = make_unique_folder(base, record_folder_name(rec))
        except Exception as e:
            traceback.print_exc()
            self.show_record_state('error', f"記録を保存できません: {e}")
            return
        rec.meta['name'] = os.path.basename(folder)
        self.last_record = rec
        self.last_record_csv = os.path.join(folder, rec.meta['name'] + '.csv')
        ctx = multiprocessing.get_context('spawn')
        q = ctx.Queue()
        proc = ctx.Process(target=save_record_outputs, args=(rec.payload(), folder, q), name="cfff_record_save")
        proc.start()
        self.saves.append({'proc': proc, 'queue': q, 'folder': folder, 'dead': 0, 'text': "開始"})
        label = rec.legend()
        self.show_record_state('saving', f"記録終了: {rec.meta['name']}（FF {label}: {rec.meta['ff_judge']}）。保存中…")
        for w in rec.meta.get('warnings', []):
            if w not in s.warnings:
                self.add_warning(w)
        self.rec_prev_label.config(text=f"{rec.meta['name']}（{rec.meta['start'].replace('T', ' ')}、"
                                        f"記録 {rec.duration:g} s、FF {label}）")
        changed = rec.changed_dofs()
        if changed:
            self.rec_prev_dof.current(changed[0])
        self.draw_record_preview()

    def poll_saves(self):
        for job in list(self.saves):
            while True:
                try:
                    msg = job['queue'].get_nowait()
                except queue.Empty:
                    break
                if msg[0] == 'csv':
                    job['text'] = "CSVを書きました。グラフを作成中"
                elif msg[0] == 'graph':
                    job['text'] = f"グラフ {msg[1]}/{msg[2]}"
                    self.progress['value'] = int(1000 * msg[1] / max(1, msg[2]))
                elif msg[0] == 'done':
                    job['text'] = "完了"
                    job['finished'] = True
                elif msg[0] == 'error':
                    print(msg[1], file=sys.stderr)
                    job['text'] = "エラー（詳細は端末に表示）"
                    job['finished'] = True
                    job['error'] = True
            if job.get('finished'):
                job['proc'].join(timeout=1)
                self.saves.remove(job)
                state = 'error' if job.get('error') else 'done'
                self.show_record_state(state, f"保存{job['text']}: {job['folder']}")
                continue
            if not job['proc'].is_alive():
                job['dead'] += 1
                if job['dead'] >= 5:
                    self.saves.remove(job)
                    self.show_record_state('error', f"保存の処理が異常終了しました（exitcode={job['proc'].exitcode}）: "
                                                    f"{job['folder']}")
                    continue
            if self.node is None or self.node.session is None:
                self.show_record_state('saving', f"保存中（{job['text']}）: {job['folder']}")

    def draw_record_preview(self):
        rec = self.last_record
        if rec is None:
            return
        if self.rec_fig is None:
            self.rec_fig, self.rec_canvas = self.make_preview_figure(self.rec_prev_frame)
        dof = self.rec_prev_dof.current()
        draw_preview(self.rec_fig, [(rec, DATA_COLORS[0])], dof, rec.duration)
        self.rec_canvas.draw_idle()

    def make_preview_figure(self, parent):
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        fig = Figure(figsize=(8, 5), dpi=90)
        canvas = FigureCanvasTkAgg(fig, master=parent)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        return fig, canvas

    def last_to_compare(self, key):
        if self.last_record is None or self.saves:
            messagebox.showinfo(TITLE_COMPARE, "保存が終わった記録がありません（保存中なら終わってから押してください）")
            return
        self.csv_vars[key].set(self.last_record_csv)
        self.notebook.select(self.tab_cmp)
        self.load_compare()

    # ---------- FFの送信 ----------
    # 移植元: GUI_android_tk_FFsetting.py:2939-3008 @105a28e（変更: 出どころを一緒に覚える。First は使わない）
    def send_ff(self):
        self.publish_all_current(TITLE_SEND)

    def send_zero(self):
        """全自由度を「FFなし」にして送る。確認でOKを押すまで、設定も変えず何も送らない"""
        if not messagebox.askokcancel(TITLE_SEND_ZERO,
                                      "全自由度のFF設定を「FFなし」（極値・係数すべて0）にして送信します。よろしいですか？\n"
                                      "（今の極値は覚えておき、FFを与え直すと戻ります）"):
            return
        table = zero_ff_table()
        for n, d in enumerate(FF_DOF_MAP):
            table[n, 5] = self.models[d].T
        if not self.publish_all(table, [None] * N_FF, ['FFなし（FF=0 を送信）'] * N_FF, TITLE_SEND_ZERO):
            return
        for d in FF_DOF_MAP:
            self.models[d].update(enabled=False)

    def publish_all(self, table, settings, sources, title=None):
        """全ボードへ送る。戻り値: 送ったか（記録中・確認でキャンセルなら False で、何も送らない）"""
        title = title or TITLE_SEND
        if self.node is None:
            return False
        s = self.node.session
        if s is not None and s.state == Recorder.RECORDING:
            messagebox.showerror(title, "記録中はFFを送信できません（記録のFFが分からなくなるため）")
            return False
        boards = range(1, len(BOARD_LAYOUT) + 1)
        missing = [k for k in boards if not self.node.ff_subscriber_present(k)]
        if missing and not messagebox.askokcancel(
                title, "次のボードのFFの受信側が見つかりません。送信しても届かない可能性があります:\n  "
                + ", ".join(f"board{k}" for k in missing) + "\n送信しますか？"):
            return False
        self.node.publish_ff(table, settings, sources, {k: ('nosub' if k in missing else 'sent') for k in boards})
        self.sent_snap = self.node.sent_snapshot()
        self.refresh_sent_views()
        return True

    # ---------- Optuna ----------
    def choose_optuna_dir(self):
        path = filedialog.askdirectory(initialdir=self.optuna_dir_var.get() or os.getcwd(),
                                       title="Select Optuna result folder")
        if path:
            self.optuna_dir_var.set(path)
            self.load_optuna()

    def load_optuna(self, quiet=False):
        folder = self.optuna_dir_var.get().strip()
        if not folder:
            if not quiet:
                messagebox.showerror(TITLE_OPTUNA, "Optuna の結果フォルダを選んでください")
            return
        try:
            res = OptunaResult(os.path.expanduser(folder))
        except Exception as e:
            self.optuna = None
            self.optuna_info.config(text=f"Optuna の結果を読めません: {e}", fg='#a00000')
            if not quiet:
                messagebox.showerror(TITLE_OPTUNA, f"読み込めません:\n{e}")
            return
        self.optuna = res
        self.optuna_info.config(text="Optuna の結果: " + res.summary() + "\n（読み込んでも送信はしません。"
                                     "送るときは「FFパラメータ送信」）", fg='#304080')
        self.save_prefs_later()

    def optuna_pick(self, dof, mode, n_text=None):
        """自由度 dof に読み込む試行。戻り値 (試行, 説明) または (None, 理由)"""
        res = self.optuna
        if dof not in res.dofs:
            return None, "Optuna で最適化していない自由度です"
        if mode == '1':
            tr = res.best_overall()
            return (tr, f"①J最小 J={tr['J']:.4g}") if tr else (None, "COMPLETE の試行がありません")
        if mode == '2':
            tr = res.best_for_dof(dof)
            return (tr, f"②J_i最小 J_i={tr['dof'][dof]['J_i']:.4g}") if tr else (None, "J_i のある試行がありません")
        if mode == '4':
            k, g = res.group_of(dof)
            tr, v = res.group_best(k)
            return ((tr, f"④グループ g{k}（DOF {' '.join(map(str, g))}）の best J_g={v:.4g}") if tr
                    else (None, f"グループ g{k} に J_i のそろった試行がありません"))
        try:
            n = int(str(n_text).strip())
        except ValueError:
            return None, "試行番号を整数で入力してください"
        tr = res.trial(n)
        if tr is None:
            return None, f"試行 {n} がありません"
        jtext = f"J={tr['J']:.4g}" if np.isfinite(tr['J']) else tr['state']
        return tr, f"③指定 {jtext}"

    def optuna_apply(self, dof, tr, how):
        """試行 tr の極値を dof の設定に入れる。戻り値 (成功したか, 文)"""
        res = self.optuna
        ext = res.extrema(tr, dof)
        if ext is None:
            return False, f"試行 {tr['n']} にこの自由度の極値がありません"
        T, t1, y1, t2, y2, sent = ext
        m = self.models[dof]
        if max(abs(y1), abs(y2)) > PWM_LIMIT:
            return False, f"極値 {y1:g}, {y2:g} が PWM の上限 ±{PWM_LIMIT:g} を超えます"
        ylo = min(m.ylo, math.floor(min(y1, y2)) - 1.0) if min(y1, y2) < m.ylo else m.ylo
        yhi = max(m.yhi, math.ceil(max(y1, y2)) + 1.0) if max(y1, y2) > m.yhi else m.yhi
        ylo, yhi = max(ylo, -PWM_LIMIT), min(yhi, PWM_LIMIT)
        chk = check_ff(T, t1, y1, t2, y2, ylo, yhi)
        if not chk.ok:
            return False, "条件を満たしません: " + " / ".join(chk.errors)
        note = ""
        sent = np.asarray(sent, dtype=float)
        exact = None
        if np.all(np.isfinite(sent)):
            exact = {'T': T, 't1': t1, 'y1': y1, 't2': t2, 'y2': y2, 'coeffs': [float(v) for v in sent]}
            note = "（係数は Optuna が送った値をそのまま使います）"
        else:
            note = "（trials.csv に係数がないため、極値から係数を作ります）"
        widened = (ylo, yhi) != (m.ylo, m.yhi)
        m.update(enabled=True, T=T, ylo=ylo, yhi=yhi, t1=t1, y1=y1, t2=t2, y2=y2, exact=exact,
                 source=f"Optuna {res.name} 試行{tr['n']}（{how}）")
        text = f"試行 {tr['n']}（{how}）: T={T:g}, t1={t1:.4f}, y1={y1:.3f}, t2={t2:.4f}, y2={y2:.3f}{note}"
        if widened:
            text += f"  y の上下限を [{ylo:g}, {yhi:g}] に広げました"
        return True, text

    def confirm_state(self, tr):
        if tr['state'] == 'COMPLETE':
            return True
        return messagebox.askokcancel(TITLE_OPTUNA, f"試行 {tr['n']} は {tr['state']} です（COMPLETE ではありません）。"
                                                    "この試行の極値を読み込みますか？")

    def apply_optuna_one(self, dof, mode):
        if self.optuna is None:
            messagebox.showerror(TITLE_OPTUNA, "先に Optuna の結果フォルダを読み込んでください")
            return
        self.select_dof(dof)
        tr, how = self.optuna_pick(dof, mode, self.rows[dof]['trial'].get())
        if tr is None:
            messagebox.showerror(TITLE_OPTUNA, f"DOF {dof}: {how}")
            return
        if not self.confirm_state(tr):
            return
        ok, text = self.optuna_apply(dof, tr, how)
        if ok:
            self.add_warning(f"DOF {dof} に読み込みました: {text}（未送信）")
        else:
            messagebox.showerror(TITLE_OPTUNA, f"DOF {dof}: {text}")

    def apply_optuna_all(self, mode):
        if self.optuna is None:
            messagebox.showerror(TITLE_OPTUNA, "先に Optuna の結果フォルダを読み込んでください")
            return
        if mode == '3':
            tr, how = self.optuna_pick(self.optuna.dofs[0], '3', self.bulk_trial_var.get())
            if tr is None:
                messagebox.showerror(TITLE_OPTUNA, how)
                return
            if not self.confirm_state(tr):
                return
        done, skipped = [], []
        for d in FF_DOF_MAP:
            tr, how = self.optuna_pick(d, mode, self.bulk_trial_var.get())
            if tr is None:
                skipped.append(f"DOF {d}: {how}（設定は変えていません）")
                continue
            ok, text = self.optuna_apply(d, tr, how)
            (done if ok else skipped).append(f"DOF {d}: {text}" + ("" if ok else "（設定は変えていません）"))
        msg = f"{MODE_TEXT[mode]} の極値を {len(done)} 自由度に読み込みました（まだ送信していません）。"
        if skipped:
            msg += "\n\n読み込まなかった自由度:\n  " + "\n  ".join(skipped)
        if done:
            msg += "\n\n読み込んだ自由度:\n  " + "\n  ".join(done)
        messagebox.showinfo(TITLE_OPTUNA, msg)

    def send_group_best(self):
        """④ グループごとの best を全自由度に読み込んで送信する（確認してから）。Optuna で最適化していない自由度は FFなし
        （Optuna の「グループの best を組み合わせた FF」と同じ）"""
        res = self.optuna
        if res is None:
            messagebox.showerror(TITLE_OPTUNA, "先に Optuna の結果フォルダを読み込んでください")
            return
        picks, missing = [], []
        for k, g in enumerate(res.groups):
            t, v = res.group_best(k)
            if t is None:
                missing.append(f"g{k}（DOF {' '.join(map(str, g))}）")
            else:
                picks.append(f"g{k}（DOF {' '.join(map(str, g))}）: 試行 {t['n']}、J_g = {v:.4g}")
        if missing:
            messagebox.showerror(TITLE_OPTUNA, "best のないグループがあるため送信しません: " + "、".join(missing))
            return
        others = [d for d in FF_DOF_MAP if d not in res.dofs]
        text = (f"{res.name} のグループごとの best（{len(res.groups)} グループ）を全自由度に読み込んで送信します。\n\n  "
                + "\n  ".join(picks[:30]) + (f"\n  …ほか {len(picks) - 30} グループ" if len(picks) > 30 else ""))
        if others:
            text += f"\n\nOptuna で最適化していない DOF {', '.join(map(str, others))} は FFなし にして送ります。"
        text += "\n\nよろしいですか？（キャンセルすると何も変えません）"
        if not messagebox.askokcancel(TITLE_SEND, text):
            return
        failed = []
        for d in FF_DOF_MAP:
            if d in res.dofs:
                tr, how = self.optuna_pick(d, '4')
                ok, msg = self.optuna_apply(d, tr, how) if tr is not None else (False, how)
                if not ok:
                    failed.append(f"DOF {d}: {msg}")
            elif self.models[d].enabled:
                self.models[d].update(enabled=False)
        if failed:
            messagebox.showerror(TITLE_OPTUNA, "次の自由度を読み込めなかったため送信しません（読み込めた自由度の設定は変わっています）:\n  "
                                 + "\n  ".join(failed))
            return
        if self.publish_all_current(TITLE_SEND):
            self.add_warning(f"{res.name} のグループごとの best（{len(res.groups)} グループ）を送信しました")

    def publish_all_current(self, title=None):
        """今の設定を全自由度送る（send_ff と同じ判定。戻り値: 送ったか）"""
        bad = [d for d in FF_DOF_MAP if self.models[d].coeffs6() is None]
        if bad:
            messagebox.showerror(title or TITLE_SEND, "次の自由度のFF設定が条件を満たしていないため、送信しません:\n"
                                 + "\n".join(f"  DOF {d}: {' / '.join(self.models[d].check().errors)}" for d in bad))
            return False
        table = np.array([self.models[d].coeffs6() for d in FF_DOF_MAP])
        settings = [self.models[d].setting() if self.models[d].enabled else None for d in FF_DOF_MAP]
        sources = [self.models[d].source if self.models[d].enabled else 'FFなし' for d in FF_DOF_MAP]
        return self.publish_all(table, settings, sources, title)

    # =========================================================
    # 比較タブ
    # =========================================================
    def build_compare_tab(self):
        tab = self.tab_cmp
        paned = ttk.PanedWindow(tab, orient=tk.VERTICAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        upper = tk.Frame(paned)
        lower = tk.Frame(paned)
        paned.add(upper, weight=3)
        paned.add(lower, weight=2)
        f = tk.Frame(upper)
        f.pack(fill=tk.X)
        c = self.settings['cmp']
        self.csv_vars = {}
        for i, (key, text) in enumerate((('csv1', "1つ目の記録CSV（青・水色）"), ('csv2', "2つ目の記録CSV（赤・桃色）"))):
            tk.Label(f, text=text).grid(row=i, column=0, sticky='w')
            self.csv_vars[key] = tk.StringVar(value=c.get(key, ''))
            tk.Entry(f, textvariable=self.csv_vars[key], width=90).grid(row=i, column=1, sticky='ew')
            tk.Button(f, text="参照", command=lambda k=key: self.choose_csv(k)).grid(row=i, column=2, padx=2)
        tk.Button(f, text="読み込む（グラフを表示）", command=self.load_compare).grid(row=0, column=3, rowspan=2, padx=6,
                                                                              sticky='ns')
        tk.Label(f, text="基準（FF=0）の記録CSV\n（目標軌道を作る。複数可）").grid(row=2, column=0, sticky='nw')
        lf = tk.Frame(f)
        lf.grid(row=2, column=1, sticky='ew')
        self.ref_list = tk.Listbox(lf, height=3, width=90)
        self.ref_list.pack(side=tk.LEFT, fill=tk.X, expand=True)
        for p in c.get('refs', []):
            self.ref_list.insert(tk.END, p)
        bf = tk.Frame(f)
        bf.grid(row=2, column=2, columnspan=2, sticky='nw')
        tk.Button(bf, text="追加", command=self.add_ref).pack(side=tk.LEFT)
        tk.Button(bf, text="削除", command=self.remove_ref).pack(side=tk.LEFT, padx=2)
        tk.Button(bf, text="1つ目・2つ目の FF=0 を入れる", command=lambda: self.auto_refs(replace=True)).pack(side=tk.LEFT)
        f.columnconfigure(1, weight=1)

        sel = tk.Frame(upper)
        sel.pack(fill=tk.X, pady=(4, 0))
        tk.Label(sel, text="比較する自由度").pack(side=tk.LEFT)
        chosen = set(c.get('dofs', [d for d in range(N_DOF) if d not in NO_FF_DOFS]))
        self.cmp_dof_vars = {}
        for d in range(N_DOF):
            v = tk.BooleanVar(value=(d in chosen and d not in NO_FF_DOFS))
            cb = tk.Checkbutton(sel, text=str(d), variable=v, command=self.save_prefs_later, padx=0)
            if d in NO_FF_DOFS:
                cb.config(state=tk.DISABLED)        # FFの枠がなく制御していない自由度は評価しない
            cb.pack(side=tk.LEFT)
            self.cmp_dof_vars[d] = v
        tk.Button(sel, text="全部", command=lambda: self.set_cmp_dofs(True)).pack(side=tk.LEFT, padx=(6, 1))
        tk.Button(sel, text="なし", command=lambda: self.set_cmp_dofs(False)).pack(side=tk.LEFT)
        tk.Label(sel, text="（DOF 21・25 は評価しない）", font=(UI_FONT, 8)).pack(side=tk.LEFT, padx=4)
        nf = tk.Frame(upper)
        nf.pack(fill=tk.X, pady=(2, 0))
        tk.Label(nf, text="評価値の定義").pack(side=tk.LEFT)
        self.cmp_norm_var = tk.BooleanVar(value=bool(c.get('normalize', True)))
        for val in (True, False):
            tk.Radiobutton(nf, text=NORM_TEXT[val], variable=self.cmp_norm_var, value=val,
                           command=self.save_prefs_later).pack(side=tk.LEFT, padx=(4, 8))

        p = tk.Frame(upper)
        p.pack(fill=tk.X, pady=(4, 0))
        self.cmp_vars = {}
        for key, text in (('T', "動作完了時間 T [s]"), ('eps', "完了の許容 ε [% of ΔP]"), ('E', "評価時間の固定値 E [s]")):
            tk.Label(p, text=text).pack(side=tk.LEFT, padx=(6, 1))
            self.cmp_vars[key] = tk.StringVar(value=c.get(key, ''))
            tk.Entry(p, textvariable=self.cmp_vars[key], width=7, justify='right').pack(side=tk.LEFT)
        tk.Button(p, text="Optuna の T・ε・E・評価値の定義を入れる", command=self.optuna_to_compare).pack(side=tk.LEFT, padx=6)
        tk.Label(p, text="   保存先").pack(side=tk.LEFT)
        self.cmp_dir_var = tk.StringVar(value=c.get('dir') or self.settings.get('record_dir') or os.getcwd())
        tk.Entry(p, textvariable=self.cmp_dir_var, width=40).pack(side=tk.LEFT)
        tk.Button(p, text="参照", command=self.choose_cmp_dir).pack(side=tk.LEFT, padx=2)
        q = tk.Frame(upper)
        q.pack(fill=tk.X, pady=(4, 0))
        self.cmp_button = tk.Button(q, text="比較して保存", command=self.run_compare, bg='#d8e4f8',
                                    font=(UI_FONT, 10, 'bold'))
        self.cmp_button.pack(side=tk.LEFT)
        self.cmp_progress = ttk.Progressbar(q, length=220, maximum=1000)
        self.cmp_progress.pack(side=tk.LEFT, padx=6)
        self.cmp_status = tk.Label(q, text="評価値 J_i は小さいほど目標応答への追従誤差が小さい", anchor='w', font=(UI_FONT, 9))
        self.cmp_status.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.cmp_info = tk.Label(upper, text="", anchor='w', justify='left', font=(UI_FONT, 9))
        self.cmp_info.pack(fill=tk.X)

        res = tk.Frame(upper)
        res.pack(fill=tk.BOTH, expand=True)
        cols = ('dof', 'j1', 'j2', 'pct', 'judge', 'share1', 'share2', 'diff', 'small', 'note')
        self.tree = ttk.Treeview(res, columns=cols, show='headings', height=8)
        for key, text, width in (('dof', "DOF", 60), ('j1', "J_i(1)", 85), ('j2', "J_i(2)", 85),
                                 ('pct', "変化率 [%]", 80), ('judge', "改善/悪化", 70),
                                 ('share1', "Jに占める割合(1) [%]", 120), ('share2', "Jに占める割合(2) [%]", 120),
                                 ('diff', "J_i(2)−J_i(1)", 95), ('small', "小さい方", 65), ('note', "備考", 500)):
            self.tree.heading(key, text=text)
            self.tree.column(key, width=width, anchor='w' if key == 'note' else 'e', stretch=(key == 'note'))
        ts = ttk.Scrollbar(res, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=ts.set)
        self.tree.grid(row=0, column=0, sticky='nsew')
        ts.grid(row=0, column=1, sticky='ns')
        self.cmp_warn = tk.Text(res, height=8, width=60, wrap='word', font=(UI_FONT, 9), fg='#a04000')
        ws = ttk.Scrollbar(res, orient=tk.VERTICAL, command=self.cmp_warn.yview)
        self.cmp_warn.configure(yscrollcommand=ws.set)
        self.cmp_warn.grid(row=0, column=2, sticky='nsew', padx=(6, 0))
        ws.grid(row=0, column=3, sticky='ns')
        res.columnconfigure(0, weight=3)
        res.columnconfigure(2, weight=2)
        res.rowconfigure(0, weight=1)
        self.tree.bind("<<TreeviewSelect>>", self.on_tree_select)

        lower_nb = ttk.Notebook(lower)
        lower_nb.pack(fill=tk.BOTH, expand=True)
        graphs = tk.Frame(lower_nb)
        self.share_frame = tk.Frame(lower_nb)
        lower_nb.add(graphs, text="自由度ごとのグラフ")
        lower_nb.add(self.share_frame, text="J に占める割合・変化率（棒グラフ）")
        self.share_fig = None
        lower = graphs
        row = tk.Frame(lower)
        row.pack(fill=tk.X)
        tk.Label(row, text="表示する自由度").pack(side=tk.LEFT)
        self.cmp_prev_dof = ttk.Combobox(row, values=[f"DOF {d}" for d in range(N_DOF)], width=8, state='readonly')
        self.cmp_prev_dof.current(0)
        self.cmp_prev_dof.pack(side=tk.LEFT)
        self.cmp_prev_dof.bind("<<ComboboxSelected>>", lambda e: self.draw_compare_preview())
        tk.Label(row, text="（保存する比較グラフと同じ内容。1つ目 = 青・水色、2つ目 = 赤・桃色、Reference = 黒の破線、"
                           "目標軌道 = 緑の一点鎖線（比較の後）。横軸は配信番号 × 0.010 s、t=0 はボードの目標値が変わった配信）",
                 font=(UI_FONT, 9)).pack(side=tk.LEFT)
        self.cmp_prev_frame = tk.Frame(lower)
        self.cmp_prev_frame.pack(fill=tk.BOTH, expand=True)
        self.cmp_fig = None

    def set_cmp_dofs(self, on):
        for d, v in self.cmp_dof_vars.items():
            v.set(on and d not in NO_FF_DOFS)
        self.save_prefs_later()

    def selected_cmp_dofs(self):
        return [d for d, v in self.cmp_dof_vars.items() if v.get() and d not in NO_FF_DOFS]

    def choose_csv(self, key):
        cur = self.csv_vars[key].get().strip()
        path = filedialog.askopenfilename(initialdir=os.path.dirname(cur) if cur else self.record_dir_var.get(),
                                          title="Select record CSV", filetypes=[("CSV", "*.csv"), ("all", "*")])
        if path:
            self.csv_vars[key].set(path)
            self.load_compare()

    def add_ref(self):
        paths = filedialog.askopenfilenames(initialdir=self.record_dir_var.get(), title="Select reference (FF=0) CSV",
                                            filetypes=[("CSV", "*.csv"), ("all", "*")])
        for p in paths:
            if p not in self.ref_list.get(0, tk.END):
                self.ref_list.insert(tk.END, p)
        self.save_prefs_later()

    def remove_ref(self):
        for i in reversed(self.ref_list.curselection()):
            self.ref_list.delete(i)
        self.save_prefs_later()

    def auto_refs(self, replace=False):
        """1つ目・2つ目の記録のうち FF=0（w/o）のものを基準にする。戻り値: 入れた CSV"""
        paths = [self.csv_vars[k].get().strip() for k in ('csv1', 'csv2')
                 if k in self.cmp_records and self.cmp_records[k].meta.get('ff_label') == 'wo']
        paths = list(dict.fromkeys(paths))
        if replace:
            self.ref_list.delete(0, tk.END)
        for pth in paths:
            if pth not in self.ref_list.get(0, tk.END):
                self.ref_list.insert(tk.END, pth)
        if replace and not paths:
            messagebox.showinfo(TITLE_COMPARE, "読み込んだ2つの記録に FF=0（w/o）の記録がありません。"
                                               "「追加」で FF=0 の記録CSVを選んでください")
        self.save_prefs_later()
        return paths

    def choose_cmp_dir(self):
        path = filedialog.askdirectory(initialdir=self.cmp_dir_var.get() or os.getcwd(), title="Select save folder")
        if path:
            self.cmp_dir_var.set(path)
            self.save_prefs_later()

    def optuna_to_compare(self):
        if self.optuna is None or not self.optuna.config:
            messagebox.showinfo(TITLE_COMPARE, "Optuna の結果（config.json）を記録タブで読み込んでください")
            return
        cfg = self.optuna.config
        try:
            self.cmp_vars['T'].set(f"{float(cfg['T']):g}")
            self.cmp_vars['eps'].set(f"{float(cfg['eps']) * 100:g}")
            self.cmp_vars['E'].set(f"{float(cfg.get('eval_min', EVAL_MIN_S)):g}")
            self.cmp_norm_var.set(cfg.get('err_norm', 'dP') == 'dP')     # Optuna の評価値（ΔP で割るか）に合わせる
        except (KeyError, TypeError, ValueError) as e:
            messagebox.showerror(TITLE_COMPARE, f"config.json の T・ε・E を読めません: {e}")
            return
        self.save_prefs_later()

    def load_compare(self, quiet=False):
        """2つの記録CSVを読み、情報とグラフを表示する（評価はしない）"""
        self.cmp_records = {}
        self.cmp_ticks = {}
        self.cmp_targets = {}
        lines, errors = [], []
        for i, key in enumerate(('csv1', 'csv2'), start=1):
            path = self.csv_vars[key].get().strip()
            if not path:
                continue
            try:
                rec = read_record_csv(path)
            except Exception as e:
                errors.append(f"{i}つ目: {e}")
                continue
            self.cmp_records[key] = rec
            self.cmp_ticks[key] = tick_record(rec)
            changed = rec.changed_dofs()
            moved = "、".join(f"DOF{d} {rec.meta['init_target'][d]:g}→{rec.meta['target'][d]:g}" for d in changed[:12])
            lines.append(f"{i}つ目: {rec.name}  FF {rec.legend()}（{rec.meta.get('ff_judge', '')}）  "
                         f"記録 {rec.duration:g} s  {rec.meta.get('start', '').replace('T', ' ')}\n"
                         f"      目標値が変わった自由度: {moved}{' …' if len(changed) > 12 else ''}")
        if errors:
            lines += errors
            if not quiet:
                messagebox.showerror(TITLE_COMPARE, "\n".join(errors))
        if self.ref_list.size() == 0:
            added = self.auto_refs()
            if added:
                lines.append("基準（FF=0）に入れました: " + "、".join(os.path.basename(pth) for pth in added))
        self.cmp_info.config(text="\n".join(lines))
        if self.cmp_records:
            first = next(iter(self.cmp_records.values()))
            changed = first.changed_dofs()
            if changed:
                self.cmp_prev_dof.current(changed[0])
        self.draw_compare_preview()
        self.save_prefs_later()

    def draw_compare_preview(self):
        recs = [(self.cmp_ticks[k], DATA_COLORS[i]) for i, k in enumerate(('csv1', 'csv2')) if k in self.cmp_ticks]
        if not recs:
            return
        if self.cmp_fig is None:
            self.cmp_fig, self.cmp_canvas = self.make_preview_figure(self.cmp_prev_frame)
        dof = self.cmp_prev_dof.current()
        xmax = max(tick_end(r) for r, _ in recs)
        tp = self.cmp_targets.get(dof)
        target = target_curve(tp, int(np.floor(xmax / TICK_DT + 1e-9))) if tp is not None else None
        draw_preview(self.cmp_fig, recs, dof, xmax, target, COMPARE_XLABEL)
        self.cmp_canvas.draw_idle()

    def on_tree_select(self, _event=None):
        sel = self.tree.selection()
        if not sel:
            return
        dof = self.tree.set(sel[0], 'dof')
        if dof.isdigit():
            self.cmp_prev_dof.current(int(dof))
            self.draw_compare_preview()

    def run_compare(self):
        if self.cmp_job is not None:
            return
        self.load_compare(quiet=True)
        if 'csv1' not in self.cmp_records or 'csv2' not in self.cmp_records:
            messagebox.showerror(TITLE_COMPARE, "1つ目と2つ目の記録CSVを読み込めていません")
            return
        try:
            T = float(self.cmp_vars['T'].get())
            eps = float(self.cmp_vars['eps'].get())
            E = float(self.cmp_vars['E'].get())
            if not (T > 0 and 0 < eps <= 50 and E > 0):
                raise ValueError
        except ValueError:
            messagebox.showerror(TITLE_COMPARE, "T > 0、0 < ε ≤ 50 [%]、E > 0 の数値を入力してください")
            return
        refs = list(self.ref_list.get(0, tk.END))
        if not refs:
            messagebox.showerror(TITLE_COMPARE, "基準（FF=0）の記録CSVを選んでください（目標軌道は FF=0 の記録だけから作ります）")
            return
        bad = []
        for pth in refs:
            try:
                r = read_record_csv(pth)
                if r.meta.get('ff_label') != 'wo':
                    bad.append(f"{os.path.basename(pth)}: FF {r.legend()} の記録です（FF=0 の記録だけを基準にできます）")
            except Exception as e:
                bad.append(f"{os.path.basename(pth)}: {e}")
        if bad:
            messagebox.showerror(TITLE_COMPARE, "基準に使えない記録があります:\n" + "\n".join(bad))
            return
        rec1, rec2 = self.cmp_records['csv1'], self.cmp_records['csv2']
        dofs = self.selected_cmp_dofs()
        if not dofs:
            messagebox.showerror(TITLE_COMPARE, "比較する自由度を1つ以上選んでください")
            return
        t_ev = eval_time(T, E)
        short = [f"{i}つ目（{r.duration:g} s）" for i, r in enumerate((rec1, rec2), start=1) if r.duration < t_ev]
        if short and not messagebox.askokcancel(
                TITLE_COMPARE, f"評価区間 max(1.5T, E) = {t_ev:g} s より記録時間が短い記録があります: {'、'.join(short)}\n"
                               "受信できた配信だけで評価します（欠測として警告に書きます）。続けますか？"):
            return
        try:
            base = self.base_dir(self.cmp_dir_var)
            folder = make_unique_folder(base, compare_folder_name(rec1, rec2))
        except OSError as e:
            messagebox.showerror(TITLE_COMPARE, f"保存先にフォルダを作れません:\n{e}")
            return
        params = {'csv1': self.csv_vars['csv1'].get().strip(), 'csv2': self.csv_vars['csv2'].get().strip(),
                  'refs': refs, 'T': T, 'eps': eps / 100.0, 'E': E, 'folder': folder, 'dofs': dofs,
                  'normalize': bool(self.cmp_norm_var.get())}
        ctx = multiprocessing.get_context('spawn')
        q = ctx.Queue()
        proc = ctx.Process(target=run_comparison_job, args=(params, q), name="cfff_compare")
        proc.start()
        self.cmp_job = {'proc': proc, 'queue': q, 'folder': folder, 'dead': 0}
        self.cmp_button.config(state=tk.DISABLED)
        self.cmp_status.config(text=f"比較中… → {folder}", fg='#204080')
        for item in self.tree.get_children():
            self.tree.delete(item)
        self.cmp_warn.delete('1.0', tk.END)
        self.save_prefs_later()

    def poll_compare(self):
        job = self.cmp_job
        if job is None:
            return
        while True:
            try:
                msg = job['queue'].get_nowait()
            except queue.Empty:
                break
            kind = msg[0]
            if kind == 'stage':
                self.cmp_status.config(text=f"{msg[1]}… → {job['folder']}")
            elif kind == 'fit':
                self.cmp_progress['value'] = int(500 * msg[1] / max(1, msg[2]))
                if msg[3] is not None:
                    self.cmp_status.config(text=f"目標軌道を当てはめています（DOF {msg[3]}、{msg[1] + 1}/{msg[2]}）…")
            elif kind == 'graph':
                self.cmp_progress['value'] = 500 + int(500 * msg[1] / max(1, msg[2]))
                self.cmp_status.config(text=f"比較グラフ {msg[1]}/{msg[2]} … → {job['folder']}")
            elif kind == 'result':
                self.show_compare_result(msg[1])
            elif kind == 'done':
                job['finished'] = True
                self.cmp_status.config(text=f"{self.cmp_jtext}   保存しました: {msg[1]}", fg='#006000')
            elif kind == 'error':
                print(msg[1], file=sys.stderr)
                job['finished'] = True
                last = msg[1].strip().splitlines()[-1] if msg[1].strip() else ''
                self.cmp_status.config(text=f"エラー: {last}（詳細は端末に表示）", fg='#a00000')
        if job.get('finished'):
            job['proc'].join(timeout=1)
            self.cmp_job = None
            self.cmp_button.config(state=tk.NORMAL)
            self.cmp_progress['value'] = 0
            return
        if not job['proc'].is_alive():
            job['dead'] += 1
            if job['dead'] >= 5:
                self.cmp_job = None
                self.cmp_button.config(state=tk.NORMAL)
                self.cmp_status.config(text=f"比較の処理が異常終了しました（exitcode={job['proc'].exitcode}）", fg='#a00000')

    def show_compare_result(self, result):
        for item in self.tree.get_children():
            self.tree.delete(item)
        def pct_text(v):
            return f"{v:+.1f}" if v is not None and np.isfinite(v) else ""

        def share_text(v):
            return f"{v:.1f}" if v is not None and np.isfinite(v) else ""

        for r in result['rows']:
            small = r.get('評価値が小さい方') or ''
            self.tree.insert('', tk.END, values=(r['DOF'], _fmt_cell(r.get('J_i(1)')), _fmt_cell(r.get('J_i(2)')),
                                                 pct_text(r.get('変化率[%]')), r.get('改善/悪化') or '',
                                                 share_text(r.get('Jに占める割合(1)[%]')),
                                                 share_text(r.get('Jに占める割合(2)[%]')),
                                                 _fmt_cell(r.get('差 J_i(2)-J_i(1)')),
                                                 {'1': '1つ目', '2': '2つ目'}.get(small, small), r.get('備考') or ''))
        ov = result['overall']
        small = {'1': '1つ目', '2': '2つ目'}.get(ov['smaller'], ov['smaller'])
        self.tree.insert('', tk.END, values=("全体 J", _fmt_cell(ov['J1']), _fmt_cell(ov['J2']), pct_text(ov['pct']),
                                             ov['judge'], "100" if ov['n'] else "", "100" if ov['n'] else "",
                                             _fmt_cell(ov['diff']), small, j_summary_row(result)['備考']))
        self.cmp_jtext = (f"全体の評価値 J: 1つ目 {_fmt_cell(ov['J1'], 5)} / 2つ目 {_fmt_cell(ov['J2'], 5)} → "
                          f"{pct_text(ov['pct'])}% {ov['judge']}"
                          f"（{ov['n']} 自由度の平均。変化率は1つ目に対する2つ目。"
                          f"{'ΔPで割る' if result.get('normalize', True) else 'ΔPで割らない'}）")
        if self.share_fig is None:
            self.share_fig, self.share_canvas = self.make_preview_figure(self.share_frame)
        draw_share_chart(self.share_fig, result)
        self.share_canvas.draw_idle()
        self.cmp_status.config(text=self.cmp_jtext + "   比較グラフを作っています…")
        self.cmp_warn.delete('1.0', tk.END)
        self.cmp_warn.insert(tk.END, "\n".join(result['warnings']) if result['warnings'] else "警告はありません")
        # 画面のグラフも、評価と同じ j0 の配信番号の時間軸と目標軌道で描き直す
        self.cmp_targets = dict(result['targets'])
        for i, key in enumerate(('csv1', 'csv2')):
            if key in self.cmp_records:
                self.cmp_ticks[key] = tick_record(self.cmp_records[key], result['j0'][i])
        self.draw_compare_preview()

    # =========================================================
    # 定期処理・終了
    # =========================================================
    def poll(self):
        try:
            self.poll_events()
            self.poll_record()
            self.poll_saves()
            self.poll_compare()
        except Exception:
            traceback.print_exc()
        self.root.after(100, self.poll)

    def poll_events(self):
        if self.node is None:
            return
        changed = False
        while True:
            try:
                ev = self.node.events.get_nowait()
            except queue.Empty:
                break
            kind, k = ev[0], ev[1]
            if kind == 'board_restart':
                self.add_warning(f"board{k} の配信が {ev[2]:.1f} s 途切れて再開しました。実機が起動したとみなし、"
                                 f"board{k} のFFを 0 として扱います（未確認）。必要ならFFを送信し直してください")
                changed = True
            elif kind == 'foreign_ff':
                self.add_warning(f"他のプログラムが board{k} のFFを送信しました。実機のFFはこのGUIの送信値と違う可能性があります"
                                 "（その値は使いません）")
                changed = True
        snap = self.node.sent_snapshot()
        if changed or snap != self.sent_snap:
            self.sent_snap = snap
            self.refresh_sent_views()

    def on_close(self):
        if self.busy() and not messagebox.askokcancel(
                WINDOW_TITLE, "記録・保存・比較の途中です。終了すると途中のデータは失われます。終了しますか？"):
            return
        if self.prefs_job is not None:
            self.root.after_cancel(self.prefs_job)
        self.save_prefs()
        for job in self.saves + ([self.cmp_job] if self.cmp_job else []):
            if job['proc'].is_alive():
                job['proc'].terminate()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def draw_share_chart(fig, result):
    """J に占める各自由度の割合 J_i / Σ J_i [%] を、2つのデータで並べた棒グラフ（上）と、自由度ごとの変化率 [%]（下）"""
    fig.clear()
    rows = [r for r in result['rows'] if r.get('Jに占める割合(1)[%]') is not None]
    ov = result['overall']
    if not rows:
        ax = fig.add_subplot(1, 1, 1)
        ax.text(0.5, 0.5, "no DOF evaluated in both data", ha='center', va='center', transform=ax.transAxes)
        return
    ax1, ax2 = fig.subplots(2, 1, sharex=True)
    x = np.arange(len(rows))
    w = 0.4
    s1 = [r['Jに占める割合(1)[%]'] for r in rows]
    s2 = [r['Jに占める割合(2)[%]'] for r in rows]
    ax1.bar(x - w / 2, s1, w, color=DATA_COLORS[0][0], label=f"1: {rows[0]['データ1']} (J = {ov['J1']:.4g})")
    ax1.bar(x + w / 2, s2, w, color=DATA_COLORS[1][0], label=f"2: {rows[0]['データ2']} (J = {ov['J2']:.4g})")
    for xi, (a, b) in enumerate(zip(s1, s2)):
        ax1.text(xi - w / 2, a, f"{a:.0f}", ha='center', va='bottom', fontsize=7)
        ax1.text(xi + w / 2, b, f"{b:.0f}", ha='center', va='bottom', fontsize=7)
    ax1.set_ylabel("share of J [%]\n(J_i / sum J_i)")
    ax1.set_title(f"Share of each DOF in J  ({NORM_TEXT_EN[result.get('normalize', True)]}, J = mean J_i over {ov['n']} DOFs;  "
                  f"J change {ov['pct']:+.1f}% = {ov['judge'] and {'改善': 'improved', '悪化': 'worse', '同じ': 'same'}[ov['judge']]})",
                  fontsize=9)
    ax1.legend(fontsize=8, loc='upper right')
    ax1.grid(True, axis='y', alpha=0.4)
    pct = np.array([r['変化率[%]'] for r in rows], dtype=float)
    shown = np.clip(pct, -100, 300)
    ax2.bar(x, shown, 0.6, color=np.where(pct < 0, '#2e8b57', '#d2691e'))
    for xi, (v, sv) in enumerate(zip(pct, shown)):
        ax2.text(xi, sv, f"{v:+.0f}", ha='center', va='bottom' if sv >= 0 else 'top', fontsize=7)
    ax2.axhline(0, color='black', linewidth=0.8)
    ax2.set_ylabel("change of J_i [%]\n(2 vs 1; < 0: improved)")
    ax2.set_xticks(x)
    ax2.set_xticklabels([str(r['DOF']) for r in rows])
    ax2.set_xlabel("DOF  (bars beyond +300% are cut at 300; the label shows the value)")
    ax2.grid(True, axis='y', alpha=0.4)
    fig.tight_layout()


def save_share_chart(path, result):
    """draw_share_chart と同じ棒グラフを PNG に保存する（pyplot は使わない）"""
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    fig = Figure(figsize=(12, 7), dpi=100)
    FigureCanvasAgg(fig)
    draw_share_chart(fig, result)
    fig.savefig(path)


def draw_preview(fig, entries, dof, xmax, target=None, xlabel=RECORD_XLABEL):
    """GUIに表示する4つのグラフ（保存するPNGと同じ描き方を2×2に並べる）"""
    fig.clear()
    axes = fig.subplots(2, 2)
    for ax, kind in zip(axes.ravel(), GRAPH_KINDS):
        plot_dof_kind(ax, entries, dof, kind, xmax, title=f"DOF {dof}  {GRAPH_TITLES[kind]}", target=target,
                      xlabel=xlabel)
        _visible_ylim(ax, xmax)
    fig.tight_layout()


# ==============================================================================
# 起動
# ==============================================================================
def main(args=None):
    node = executor = None
    ros_error = ROS_IMPORT_ERROR
    if rclpy is not None:
        try:
            from rclpy.executors import SingleThreadedExecutor
            rclpy.init(args=args)
            node = CfNode()
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            threading.Thread(target=executor.spin, daemon=True).start()
        except Exception as exc:
            ros_error = str(exc)
            node = None
    settings, warning = load_settings()
    try:
        app = CfApp(node, settings, SETTINGS_FILE, warning, ros_error)
        app.run()
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
