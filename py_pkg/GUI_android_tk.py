#!/usr/bin/env python3
"""
POT目標値GUI（/board_android_float/sub へ26自由度の目標値を送る）

・起動時には目標値を送信しない。ボードが実際に使っている目標値（/boardN_tk/pub の [6..11]）を
  読み込んで表示に反映するので、最初の操作で他の自由度を古い値で上書きすることもない。
・初期姿勢（Reset pose）とプリセット1〜5は SETTINGS_FILE に保存し、再起動後も引き継ぐ。
    プリセット1〜4 …「初期姿勢・プリセット」タブで編集
    プリセット5   … メイン画面の「現在の目標値をプリセット5に登録」で登録
・全自由度の実測POTと目標値を常時受信して保持するので、表示DOFを切り替えてもグラフが途切れない。
・グラフは表示DOFを複数選択でき、選択したDOFごとに実測POTと目標値を表示する。
  グラフの大きさ・並びは表示領域（モニター・ウィンドウの大きさ、上下の境界の位置）に合わせて自動で変わり、
  収まらないときは縦スクロールで見られる。上（グラフ）と下（操作部）の境界はドラッグで動かせる。
  「グラフ停止」で表示を止めて見られる（受信・記録は止まらない。停止中も表示DOF・表示時間幅は変えられる）。
・操作部は 左 = Random Target / 中央 = 目標値の入力 / 右 = 記録 の3列（ウィンドウが狭いときは左右を中央の下へ回す）。
・他プログラムが /board_android_float/sub に送った目標値もGUIの表示・内部状態に反映する。
・記録: 「記録開始」で待機状態になり、どれか1つの自由度でも目標値が直前と変わった時点を 0 秒として、
  記録時間（記録中も変更可）のあいだ全自由度の 実測POT・目標値・z3・PWM を記録する。
    <保存先>/data_YYYYMMDD_HHMMSS/data_YYYYMMDD_HHMMSS.csv     … 記録データ（形式は RECORD CSV の説明を参照）
    <保存先>/data_YYYYMMDD_HHMMSS/DOF/DOF<n>/time-pot.png       … 実測POTと目標値
                                            /time-z3.png        … 外乱推定値 z3
                                            /time-PWM.png       … PWM
  グラフは保存したCSVを読み直して作る（別プロセスで作るのでGUIは止まらない）。
"""
import csv
import json
import math
import multiprocessing
import os
import queue
import re
import shutil
import subprocess
import sys
import threading
import traceback
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt16MultiArray

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from tkinter import font as tkfont
import time

import random

# ------------------------------
# 日本語表示用フォント
# ------------------------------
# 日本語グリフを持たないフォントが既定になっていると、日本語がすべて豆腐(□)になる。
# 起動時に使えるフォントを探して既定フォントに設定する。
JP_FONT_CANDIDATES = [
    "Noto Sans CJK JP", "Noto Sans JP", "Source Han Sans JP",
    "IPAexGothic", "IPAPGothic", "IPAGothic",
    "TakaoPGothic", "TakaoGothic", "VL PGothic",
    "Yu Gothic UI", "Meiryo UI", "Meiryo", "MS UI Gothic", "MS Gothic",
    "BIZ UDPゴシック", "BIZ UDゴシック",
]

# setup_japanese_font() で実際のフォント名に置き換わる
UI_FONT = "TkDefaultFont"

# ※ WSLg ではタイトルバー（ウィンドウ名・messagebox/filedialog の title）はWSLg側が描くため日本語が文字化けする。
#   タイトルは英字にし、本文は日本語でよい（本文は Tk の日本語フォントで描かれる）。


def setup_japanese_font(root):
    """日本語が豆腐(□)にならないよう、日本語グリフを持つフォントを既定にする"""
    global UI_FONT

    try:
        available = set(tkfont.families(root))
    except tk.TclError:
        return

    family = None
    for name in JP_FONT_CANDIDATES:
        if name in available:
            family = name
            break

    if family is None:
        print(
            "警告: 日本語フォントが見つかりませんでした。GUIの日本語が□で表示されます。\n"
            "      Ubuntu/WSL の場合は次のどちらかで解決できます:\n"
            "        1) sudo apt install fonts-noto-cjk\n"
            "        2) WSLなら ~/.config/fontconfig/fonts.conf に\n"
            "           <dir>/mnt/c/Windows/Fonts</dir> を書いて fc-cache -f",
            file=sys.stderr,
        )
        return

    UI_FONT = family

    # tk ウィジェット（tk.Label / tk.Button / messagebox など）の既定フォント
    # ※ TkFixedFont は等幅を保ちたいので変更しない
    for named in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont",
                  "TkCaptionFont", "TkSmallCaptionFont", "TkIconFont", "TkTooltipFont"):
        try:
            tkfont.nametofont(named, root=root).configure(family=family)
        except tk.TclError:
            pass

    # ttk ウィジェット（Notebook のタブ・Combobox など）のフォント
    try:
        style = ttk.Style(root)
        style.configure(".", font=(family, 10))
        style.configure("TNotebook.Tab", font=(family, 10))
    except tk.TclError:
        pass

# POT ranges（スライダ・Step Input の手入力・初期姿勢/プリセット・グラフ縦軸の範囲）
POT_RANGE = [
    (185, 700), (135, 550), (130, 680), (10, 734), (66, 259), (192, 389),
    (70, 600), (60, 465), (115, 619), (22, 794), (239, 430), (205, 395),
    (30, 660), (30, 690), (110, 830), (3, 630), (3, 700),(9, 660),
    (275, 360), (115, 785), (192, 440), (284, 557),
    (323, 580), (188, 630), (375, 500), (300, 490),
]

# Initial desired values（設定ファイルが無いときの初期姿勢）
INITIAL_DESIRED = [
    500, 200, 500, 300, 170, 300,
    160, 410, 200, 500, 350, 220,
    300, 200, 400, 350, 420, 400,
    325, 370, 280, 420,
    360, 390, 420, 390
]

# Random target ranges（Send Random Target で生成する目標値の範囲。POT_RANGE の内側にすること）
#   ※以前は `RANDOM_RANGE = POT_RANGE = [...]` と書かれていて、POT_RANGE がこの値で上書きされていた。
#     スライダ・手入力は POT_RANGE、ランダム目標値は RANDOM_RANGE を使うよう分けている。
RANDOM_RANGE = [
    (450, 700), (135, 550), (500, 680), (250, 700), (66, 259), (192, 389),
    (70, 200), (60, 465), (115, 200), (100, 550), (239, 430), (205, 395),
    (30, 660), (30, 690), (110, 830), (3, 630), (3, 700), (9, 660),
    (275, 360), (115, 785), (192, 440), (284, 557),
    (323, 580), (188, 630), (375, 500), (300, 490),
]

RANDOM_ENABLE = [True]*26

# ------------------------------
# 自由度(DOF)とボードの対応・トピック
# ------------------------------
# (board名, 先頭DOF, DOF数)。ボード番号 k は並び順（board1 = 1）
BOARD_LAYOUT = [
    ('board1',  0, 6),
    ('board2',  6, 6),
    ('board3', 12, 6),
    ('board4', 18, 4),
    ('board5', 22, 4),
]
TOPIC_TARGET = '/board_android_float/sub'   # 目標値（Float32MultiArray, 26要素）。GUIが送信し、他プログラムの送信も受信する
TOPIC_POT = '/{}_tk/pub'                    # 実測（UInt16MultiArray）: [0..5] = 実測POT, [6..11] = ボードが使っている目標値
TOPIC_PWM = '/{}_tk_PWM_float/pub'          # PWM（Float32MultiArray, 6要素。DOF対応は実測POTと同じ）
TOPIC_Z3 = '/{}_tk_z3_float/pub'            # ESO外乱推定値 z3（Float32MultiArray, 6要素。DOF対応は実測POTと同じ）

# ボードからのトピックの受信キューの深さ（各100Hz。GUI描画で受信が一瞬遅れても取りこぼさないよう大きめ）
SENSOR_QOS_DEPTH = 100

# ------------------------------
# 設定ファイル（初期姿勢・プリセット・グラフ表示DOF・記録の設定）
# ------------------------------
# JSON: {"version": 1,
#        "initial_pose": [26個], "presets": [[26個] または null（未登録）, ×5],
#        "graph_dofs": [グラフを表示するDOF番号, ...],
#        "record_dir": 記録の保存先フォルダ（null なら起動したフォルダ）, "record_duration": 記録時間 [s]}
SETTINGS_FILE = os.path.expanduser('~/.armrobot_gui_android_tk.json')
NUM_PRESETS = 5
# 「現在の目標値をプリセットに登録」ボタンの登録先（プリセット5）。これ以外はタブで編集する
REGISTER_PRESET_INDEX = 4

# ------------------------------
# 時系列の保持・グラフ表示
# ------------------------------
HISTORY_SEC = 120          # 保持する時系列の長さ [s]（表示時間幅の上限）
HISTORY_MAX_RATE = 150     # 想定する最大受信レート [Hz]（各トピックは100Hz）。保持数 = HISTORY_SEC × これ
PLOT_WINDOW_SEC = 5        # グラフの表示時間幅の初期値 [s]（従来: 100点×50ms ≒ 5秒）
PLOT_MIN_W = 300           # グラフ1つの最小の幅 [px]。全部をこれ以上の大きさで並べられないときは縦スクロールにする
PLOT_MIN_H = 160           # グラフ1つの最小の高さ [px]
PLOT_ASPECT = 2.5          # 見やすいグラフの 横:縦
PLOT_PAD = 2               # グラフどうしの間隔 [px]
PLOT_BORDER = 2            # グラフの枠の太さ [px]（スライダで操作中のDOFは枠を色付けする）

# ------------------------------
# 記録
# ------------------------------
RECORD_DURATION_DEFAULT = 10.0  # 記録時間の初期値 [s]
RECORD_DURATION_MAX = 3600.0    # 記録時間の上限 [s]
RECORD_KINDS = ('pot', 'pwm', 'z3')
RECORD_PARTS_DIR = '.parts'     # 記録中にトピックごとに書く一時CSVの置き場（後処理で1つのCSVにまとめてから消す）

# RECORD CSV（write_limitdata_ADRC_android_csv.py と同じく、時刻を縦に並べた形式）
#   ボード k（1〜5）ごとに次の列を横に並べる（d はそのボードのDOF番号 0〜25）
#     Time{k},     POT{d}..., POTdesired{d}...   ← /board{k}_tk/pub の受信時刻と、実測POT・ボードが使っている目標値
#     Time{k}_PWM, PWM{d}...                     ← /board{k}_tk_PWM_float/pub の受信時刻と値
#     Time{k}_z3,  z3_{d}...                     ← /board{k}_tk_z3_float/pub の受信時刻と値
#   時刻は記録開始（目標値が変わったメッセージを受信した時刻）からの秒数で、全トピック共通の時計で測る。
#   トピックごとに別々に届くので、i 行目は「各トピックの i 回目の受信」であり、同じ行でも時刻列ごとに時刻が違う。
#   値は必ず同じグループの時刻列と組にして使うこと。受信回数が少ないトピックの残りの行は空欄。

# ------------------------------
# 他プログラムからの目標値変更への追従
# ------------------------------
OWN_ECHO_KEEP_SEC = 5.0     # GUIが送った値をこの時間は「自分の送信の折り返し」として無視する
OWN_SENT_KEEP = 256         # 上の判定のために覚えておく送信の最大数（スライダのドラッグ中は連続送信になる）
ECHO_SYNC_QUIET_SEC = 1.0   # 目標値トピックの送受信がこの時間なければ、ボードが使っている目標値に表示を合わせる
ECHO_SYNC_STABLE_SEC = 0.5  # ただしボードの目標値がこの時間変化していない（落ち着いている）ときだけ


# ------------------------------
# 設定ファイルの読み書き
# ------------------------------
def default_settings():
    return {
        'initial_pose': [float(v) for v in INITIAL_DESIRED],
        'presets': [None]*NUM_PRESETS,
        'graph_dofs': [0],
        'record_dir': None,
        'record_duration': RECORD_DURATION_DEFAULT,
    }


def _is_pose(values):
    """26自由度ぶんの有限な数値のリストか"""
    return (isinstance(values, list) and len(values) == 26
            and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                    for v in values))


def load_settings(path=None):
    """設定ファイルを読む。戻り値 (settings, 警告文 or None)。読めない項目は既定値のまま"""
    path = path or SETTINGS_FILE
    settings = default_settings()
    if not os.path.exists(path):
        return settings, None

    try:
        with open(path) as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise ValueError("JSONの最上位がオブジェクトではありません")
    except (OSError, ValueError) as e:
        return settings, f"{path} を読めないため、初期姿勢・プリセットは既定値を使います。\n{e}"

    bad = []
    init = data.get('initial_pose')
    if _is_pose(init):
        settings['initial_pose'] = [float(v) for v in init]
    elif init is not None:
        bad.append("initial_pose")

    presets = data.get('presets')
    presets = presets if isinstance(presets, list) else []
    for k in range(min(NUM_PRESETS, len(presets))):
        p = presets[k]
        if p is None:
            continue
        if _is_pose(p):
            settings['presets'][k] = [float(v) for v in p]
        else:
            bad.append(f"presets[{k}]")

    dofs = data.get('graph_dofs')
    if isinstance(dofs, list) and all(isinstance(d, int) and not isinstance(d, bool) and 0 <= d < 26 for d in dofs):
        settings['graph_dofs'] = sorted(set(dofs))
    elif dofs is not None:
        bad.append("graph_dofs")

    record_dir = data.get('record_dir')
    if isinstance(record_dir, str) and record_dir:
        settings['record_dir'] = record_dir
    elif record_dir is not None:
        bad.append("record_dir")

    duration = data.get('record_duration')
    if (isinstance(duration, (int, float)) and not isinstance(duration, bool)
            and 0 < duration <= RECORD_DURATION_MAX):
        settings['record_duration'] = float(duration)
    elif duration is not None:
        bad.append("record_duration")

    if bad:
        return settings, (f"{path} の次の項目が不正なため既定値にしました:\n  " + ", ".join(bad)
                          + "\n（初期姿勢・プリセットは26個の数値のリストである必要があります）")
    return settings, None


def save_settings(settings, path=None):
    """設定ファイルへ書く（途中で落ちても壊れないよう、一時ファイルに書いてから置き換える）"""
    path = path or SETTINGS_FILE
    if os.path.exists(path):
        try:
            with open(path) as f:
                json.load(f)
        except (OSError, ValueError):
            # 読めない設定ファイルは消さずに退避しておく（手で直したい場合のため）
            os.replace(path, path + '.bak')

    data = {
        'version': 1,
        'initial_pose': [float(v) for v in settings['initial_pose']],
        'presets': [None if p is None else [float(v) for v in p] for p in settings['presets']],
        'graph_dofs': [int(d) for d in settings['graph_dofs']],
        'record_dir': settings['record_dir'],
        'record_duration': float(settings['record_duration']),
    }
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)


def format_value(v):
    """入力欄に表示する文字列（整数なら小数点なし）。None は空欄"""
    if v is None:
        return ""
    v = float(v)
    if v == int(v):
        return str(int(v))
    return str(v)


def to_float32_tuple(values):
    """Float32MultiArray で送受信したときと同じ値（float32 に丸めた値）のタプル"""
    return tuple(np.asarray(values, dtype=np.float32).tolist())


# ------------------------------
# 時系列のリングバッファ
# ------------------------------
class TimeRing:
    """受信時刻つき時系列の固定長リングバッファ（受信スレッドが書き、GUIスレッドが読む）

    各サンプルを [i] と [i + capacity] の2か所に書いておくことで、
    直近 capacity 個がいつも連続した領域に並び、時間窓をスライスで切り出せる。
    """

    def __init__(self, capacity, width):
        self.capacity = int(capacity)
        self.t = np.zeros(2 * self.capacity)
        self.v = np.full((2 * self.capacity, width), np.nan)
        self.count = 0      # これまでに書いたサンプル数
        self.lock = threading.Lock()

    def append(self, t, values):
        with self.lock:
            i = self.count % self.capacity
            self.t[i] = self.t[i + self.capacity] = t
            self.v[i] = self.v[i + self.capacity] = values
            self.count += 1

    def window(self, t_from, cols):
        """時刻 t_from 以降のサンプルを (時刻[n], 値[n, len(cols)]) のコピーで返す"""
        with self.lock:
            n = min(self.count, self.capacity)
            start = self.count % self.capacity if self.count > self.capacity else 0
            seg_t = self.t[start:start + n]
            k = int(np.searchsorted(seg_t, t_from, side='left'))
            return seg_t[k:].copy(), self.v[start + k:start + n][:, cols]


# ------------------------------
# 記録
# ------------------------------
def record_header(k, kind, offset, count):
    """記録CSVのうち、ボード k の1トピック分の列名"""
    dofs = range(offset, offset + count)
    if kind == 'pot':
        return [f"Time{k}"] + [f"POT{d}" for d in dofs] + [f"POTdesired{d}" for d in dofs]
    if kind == 'pwm':
        return [f"Time{k}_PWM"] + [f"PWM{d}" for d in dofs]
    return [f"Time{k}_z3"] + [f"z3_{d}" for d in dofs]


def make_record_folder(base_dir, when):
    """<保存先>/data_YYYYMMDD_HHMMSS を作る（同じ秒に2回記録したときは _2, _3 … を付ける）"""
    name = when.strftime("data_%Y%m%d_%H%M%S")
    path = os.path.join(base_dir, name)
    n = 2
    while os.path.exists(path):
        path = os.path.join(base_dir, f"{name}_{n}")
        n += 1
    os.makedirs(path)
    return path


def _fmt_int(v):
    return "" if v != v else str(int(v))       # NaN（未受信）は空欄


def _fmt_float(v):
    return "" if v != v else "%.7g" % v         # float32 の有効桁（約7桁）で書く


class RecordSession:
    """1回分の記録: 待機 → 記録中 → 書き出し完了

    受信コールバック（ROSスレッド）が add() で渡したデータを、書き出しスレッドがトピックごとの一時CSVへ書く。
    記録時間が長くてもメモリに溜めないので、GUIの応答性もメモリ使用量も変わらない。
    """
    ARMED, RECORDING, FINISHING, DONE, CANCELLED = range(5)

    def __init__(self, base_dir, duration):
        self.base_dir = base_dir
        self.duration = float(duration)     # 記録時間 [s]。記録中もGUIから変更される
        self.state = self.ARMED
        self.t0 = None                      # 記録開始の時刻（time.monotonic()）
        self.folder = None
        self.rows = 0                       # 書いた行数（受信件数）
        self.error = None
        self.lock = threading.Lock()
        self.queue = queue.SimpleQueue()
        self.thread = threading.Thread(target=self._writer, daemon=True)
        self.thread.start()

    def trigger(self, t):
        """記録を開始する（目標値が変わったメッセージの受信時刻 t を 0 秒とする）"""
        with self.lock:
            if self.state != self.ARMED:
                return
            self.t0 = t
            self.queue.put(('start', datetime.now()))
            self.state = self.RECORDING

    def add(self, key, t, values):
        """受信したデータを記録する。key = (種類, board名)。記録時間を過ぎたら記録を終える"""
        if self.state != self.RECORDING:
            return
        tr = t - self.t0
        if tr > self.duration:
            self.finish()
            return
        self.queue.put(('row', key, tr, values))

    def finish(self):
        """記録を終える（待機中なら取り消す）"""
        with self.lock:
            if self.state == self.ARMED:
                self.state = self.CANCELLED
            elif self.state == self.RECORDING:
                self.state = self.FINISHING
            else:
                return
            self.queue.put(('end',))

    def _writer(self):
        files = []
        writers = {}
        try:
            while True:
                item = self.queue.get()
                if item[0] == 'row':
                    _, key, tr, values = item
                    fmt = _fmt_int if key[0] == 'pot' else _fmt_float
                    writers[key].writerow([f"{tr:.6f}"] + [fmt(v) for v in values])
                    self.rows += 1
                elif item[0] == 'start':
                    self.folder = make_record_folder(self.base_dir, item[1])
                    parts = os.path.join(self.folder, RECORD_PARTS_DIR)
                    os.makedirs(parts)
                    for k, (board, offset, count) in enumerate(BOARD_LAYOUT, start=1):
                        for kind in RECORD_KINDS:
                            f = open(os.path.join(parts, f"{board}_{kind}.csv"), 'w', newline='')
                            files.append(f)
                            writers[(kind, board)] = csv.writer(f)
                            writers[(kind, board)].writerow(record_header(k, kind, offset, count))
                else:   # 'end'
                    break
        except Exception:
            self.error = traceback.format_exc()
        finally:
            for f in files:
                f.close()
            with self.lock:
                if self.state != self.CANCELLED:
                    self.state = self.DONE


def combine_record_parts(folder):
    """トピックごとの一時CSVを、write_limitdata_ADRC_android_csv.py と同じ形の1つのCSVにまとめる"""
    parts = os.path.join(folder, RECORD_PARTS_DIR)
    blocks = []
    for k, (board, offset, count) in enumerate(BOARD_LAYOUT, start=1):
        for kind in RECORD_KINDS:
            header = record_header(k, kind, offset, count)
            path = os.path.join(parts, f"{board}_{kind}.csv")
            rows = []
            if os.path.exists(path):
                with open(path, newline='') as f:
                    rows = list(csv.reader(f))[1:]     # ヘッダ除外
            blocks.append((header, rows))

    max_rows = max((len(rows) for _, rows in blocks), default=0)
    out = os.path.join(folder, os.path.basename(folder) + '.csv')
    with open(out, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([name for header, _ in blocks for name in header])
        for i in range(max_rows):
            row = []
            for header, rows in blocks:
                row.extend(rows[i] if i < len(rows) else [""] * len(header))
            writer.writerow(row)
    return out


def read_record_csv(path):
    """記録CSVを {列名: np.array} で読む（空欄は NaN）"""
    with open(path, newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        cols = [[] for _ in header]
        for row in reader:
            for j, cell in enumerate(row):
                cols[j].append(float(cell) if cell else np.nan)
    return {name: np.array(c, dtype=float) for name, c in zip(header, cols)}


def make_record_graphs(csv_path, folder, progress=None):
    """記録CSVから DOF/DOF<n>/time-pot.png, time-z3.png, time-PWM.png を作る"""
    # pyplot は使わない（GUIのバックエンドに触れず、別プロセスでも安全に描ける）
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg

    data = read_record_csv(csv_path)
    total = 3 * sum(count for _, _, count in BOARD_LAYOUT)
    made = 0
    for k, (board, offset, count) in enumerate(BOARD_LAYOUT, start=1):
        for d in range(offset, offset + count):
            out_dir = os.path.join(folder, 'DOF', f'DOF{d}')
            os.makedirs(out_dir, exist_ok=True)
            graphs = [
                # (ファイル名, 時刻列, [(値の列, 凡例, 色, 線種)], 縦軸, 元のトピック)
                ('time-pot', f"Time{k}",
                 [(f"POT{d}", "POT (measured)", "red", "-"), (f"POTdesired{d}", "POT desired", "blue", "--")],
                 "POT", TOPIC_POT.format(board)),
                ('time-z3', f"Time{k}_z3", [(f"z3_{d}", "z3", "green", "-")], "z3", TOPIC_Z3.format(board)),
                ('time-PWM', f"Time{k}_PWM", [(f"PWM{d}", "PWM", "purple", "-")], "PWM", TOPIC_PWM.format(board)),
            ]
            for name, tcol, series, ylabel, topic in graphs:
                fig = Figure(figsize=(8, 4.5), dpi=100)
                FigureCanvasAgg(fig)
                ax = fig.add_subplot(1, 1, 1)
                t = data[tcol]
                plotted = False
                for col, label, color, style in series:
                    v = data[col]
                    ok = np.isfinite(t) & np.isfinite(v)
                    if ok.any():
                        ax.plot(t[ok], v[ok], color=color, linestyle=style, linewidth=1.2, label=label)
                        plotted = True
                if plotted:
                    ax.legend(loc="best")
                    ax.set_xlim(left=0)
                else:
                    ax.text(0.5, 0.5, f"no data ({topic} was not received)", ha="center", va="center",
                            transform=ax.transAxes, color="gray")
                ax.set_title(f"DOF {d}  {name}")
                ax.set_xlabel("Time [s]")
                ax.set_ylabel(ylabel)
                ax.grid(True, alpha=0.4)
                fig.tight_layout()
                fig.savefig(os.path.join(out_dir, f"{name}.png"))
                made += 1
                if progress is not None:
                    progress.put(('graph', made, total))


def postprocess_record(folder, progress):
    """記録の後処理（GUIとは別プロセスで動かす）: 一時CSVを1つのCSVにまとめ、そのCSVからグラフを作る"""
    try:
        csv_path = combine_record_parts(folder)
        progress.put(('csv', csv_path))
        make_record_graphs(csv_path, folder, progress)
        shutil.rmtree(os.path.join(folder, RECORD_PARTS_DIR), ignore_errors=True)
        progress.put(('done', csv_path))
    except Exception:
        progress.put(('error', traceback.format_exc()))


# ------------------------------
# グラフの並べ方
# ------------------------------
def grid_layout(n, width, height):
    """n 個のグラフを width×height の表示領域に並べるときの (列数, 1つの幅, 1つの高さ)

    全部を最小サイズ以上で表示領域に収められるなら、見やすい縦横比でいちばん大きくなる列数を選び、
    表示領域いっぱいに並べる（領域が大きいほどグラフも大きくなる）。
    収められないなら、最小幅以上で入るだけの列数にして高さは縦横比から決め、縦スクロールで見る。
    """
    best = None
    for cols in range(1, n + 1):
        rows = math.ceil(n / cols)
        w, h = width // cols, height // rows
        if w < PLOT_MIN_W or h < PLOT_MIN_H:
            continue
        score = min(w / PLOT_ASPECT, h)
        if best is None or score > best[0]:
            best = (score, cols, w, h)
    if best is not None:
        _, cols, w, h = best
        return cols, w, int(min(h, w * 0.75))    # 縦長になりすぎないようにする

    cols = max(1, min(n, width // PLOT_MIN_W))
    w = width // cols
    h = int(min(max(PLOT_MIN_H, w / PLOT_ASPECT), max(PLOT_MIN_H, height)))
    return cols, w, h


# ------------------------------
# 起動時のウィンドウの大きさ
# ------------------------------
WINDOW_SCREEN_RATIO = 0.85      # 起動時のウィンドウの大きさ（表示するモニターの幅・高さに対する割合）
FALLBACK_MONITOR = (1920, 1080) # モニターの情報が取れないときに想定するモニターの大きさの上限


def parse_xrandr_monitors(text):
    """`xrandr --listmonitors` の出力 → [(x, y, 幅, 高さ, primaryか), ...]

    例: " 0: +*DP-1 2560/597x1440/336+0+0  DP-1"（* が primary）
    """
    monitors = []
    for line in text.splitlines():
        m = re.match(r"\s*\d+:\s+\+?(\*?)\S+\s+(\d+)/\d+x(\d+)/\d+\+(-?\d+)\+(-?\d+)", line)
        if m:
            primary, w, h, x, y = m.groups()
            monitors.append((int(x), int(y), int(w), int(h), primary == "*"))
    return monitors


def list_monitors():
    """接続されているモニターの [(x, y, 幅, 高さ, primaryか), ...]。取れなければ空

    複数モニターの環境（WSLg など）では Tk の winfo_screenwidth() / winfo_screenheight() は
    全モニターを囲む大きさ（例: 7680x2712）を返すので、それを基準にするとウィンドウがモニターからはみ出す。
    """
    try:
        out = subprocess.run(["xrandr", "--listmonitors"], capture_output=True, text=True, timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return parse_xrandr_monitors(out)


def monitor_at(monitors, x, y):
    for mon in monitors:
        mx, my, mw, mh, _ = mon
        if mx <= x < mx + mw and my <= y < my + mh:
            return mon
    return None


def window_geometry_for(monitor):
    """モニターの WINDOW_SCREEN_RATIO の大きさで、モニターの中央に置く geometry 文字列"""
    mx, my, mw, mh, _ = monitor
    w, h = int(mw * WINDOW_SCREEN_RATIO), int(mh * WINDOW_SCREEN_RATIO)
    return f"{w}x{h}+{mx + (mw - w) // 2}+{my + (mh - h) // 2}"


def initial_monitor(root, monitors):
    """ウィンドウを出すモニター: マウスポインタのあるモニター → primary → 最初のモニター"""
    if monitors:
        px, py = root.winfo_pointerxy()
        mon = monitor_at(monitors, px, py)
        if mon is not None:
            return mon
        for mon in monitors:
            if mon[4]:
                return mon
        return monitors[0]
    # モニターの情報が取れないとき: 画面全体を1台とみなす。ただし複数モニターを囲んだ大きさかもしれないので
    # 一般的なモニターの大きさを上限にする
    return (0, 0, min(root.winfo_screenwidth(), FALLBACK_MONITOR[0]),
            min(root.winfo_screenheight(), FALLBACK_MONITOR[1]), True)


# ------------------------------
# ROS2 Node
# ------------------------------
class PotGuiNode(Node):
    def __init__(self, initial_desired=INITIAL_DESIRED):
        super().__init__('pot_gui_node')
        self.publisher = self.create_publisher(Float32MultiArray, TOPIC_TARGET, 10)

        self.desired = [float(v) for v in initial_desired]
        self.real_raw = [0.0]*26

        # ボードが実際に使っている目標値（実測トピックの [6..11]）。未受信の自由度は None
        self.board_desired = [None]*26
        # board_desired が最後に変化した時刻（time.monotonic()）
        self.board_desired_changed = [0.0]*26

        # 目標値トピックの送受信の記録（他プログラムによる変更かどうかの判定に使う）
        self.last_publish_time = 0.0        # GUIが最後に送信した時刻
        self.last_cmd_recv_time = 0.0       # 目標値トピックを最後に受信した時刻（自分の送信の折り返しも含む）
        self.own_sent = deque(maxlen=OWN_SENT_KEEP)     # GUIが送った目標値 (送信時刻, float32に丸めた値)
        # 受信した目標値 (受信時刻, 値)。node.desired はGUIスレッドだけが書き換えるので、ここで受け渡す
        self.cmd_queue = queue.SimpleQueue()
        # 目標値トピックで最後に受信した値（記録開始の判定に使う。自分の送信の折り返しも含む）
        self.last_cmd_values = None

        # 記録（RecordSession）。GUIスレッドが設定し、受信コールバックがデータを渡す
        self.session = None

        # 全自由度の時系列（ボードごと。列 = [実測POT×n, 目標値×n]）
        self.history = {}
        # DOF -> (board名, ボード内index, ボードのDOF数)
        self.dof_slot = [None]*26

        for board, offset, count in BOARD_LAYOUT:
            self.history[board] = TimeRing(HISTORY_SEC * HISTORY_MAX_RATE, 2 * count)
            for local in range(count):
                self.dof_slot[offset + local] = (board, local, count)
            self.create_subscription(
                UInt16MultiArray, TOPIC_POT.format(board),
                lambda msg, o=offset, b=board, c=count: self.board_cb(msg, o, b, c),
                SENSOR_QOS_DEPTH)
            # PWM・z3 は記録のためだけに受信する
            for kind, topic in (('pwm', TOPIC_PWM), ('z3', TOPIC_Z3)):
                self.create_subscription(
                    Float32MultiArray, topic.format(board),
                    lambda msg, kd=kind, b=board, c=count: self.record_float_cb(msg, kd, b, c),
                    SENSOR_QOS_DEPTH)

        # 他のプログラムが目標値トピックへ publish した場合も追従する
        self.create_subscription(Float32MultiArray, TOPIC_TARGET, self.desired_cb, 10)

    def board_cb(self, msg, offset, board_name, num_pot):
        t = time.monotonic()
        data = msg.data
        row = [float('nan')] * (2 * num_pot)
        for i in range(num_pot):
            idx = offset + i
            if i < len(data):
                self.real_raw[idx] = float(data[i])
                row[i] = float(data[i])

            # .ino は pub[6+i] に POT_desired[i] を格納している（実際にボードが使っている目標値）
            if (6 + i) < len(data):
                v = float(data[6 + i])
                if self.board_desired[idx] != v:
                    self.board_desired_changed[idx] = t
                self.board_desired[idx] = v
                row[num_pot + i] = v

        self.history[board_name].append(t, row)

        session = self.session
        if session is not None:
            session.add(('pot', board_name), t, row)

    def record_float_cb(self, msg, kind, board_name, count):
        """PWM・z3 の受信（記録中だけ使う）"""
        t = time.monotonic()
        session = self.session
        if session is None or session.state != RecordSession.RECORDING:
            return
        data = msg.data
        session.add((kind, board_name), t,
                    [float(data[i]) if i < len(data) else float('nan') for i in range(count)])

    def dof_history(self, dof, t_from, snapshot=None):
        """DOF の時刻 t_from 以降の (時刻, 実測POT, 目標値) を返す（snapshot を渡すとその時点の時系列から）"""
        board, local, count = self.dof_slot[dof]
        if snapshot is None:
            t, v = self.history[board].window(t_from, [local, count + local])
        else:
            t_all, v_all = snapshot[board]
            k = int(np.searchsorted(t_all, t_from, side='left'))
            t, v = t_all[k:], v_all[k:][:, [local, count + local]]
        return t, v[:, 0], v[:, 1]

    def snapshot_history(self, t_from):
        """全ボードの時刻 t_from 以降の時系列のコピー（グラフ停止中の表示に使う）"""
        return {board: ring.window(t_from, list(range(ring.v.shape[1]))) for board, ring in self.history.items()}

    def desired_cb(self, msg):
        t = time.monotonic()
        values = tuple(msg.data)
        session = self.session
        if session is not None and session.state == RecordSession.ARMED and self.target_changed(values):
            session.trigger(t)
        self.last_cmd_values = values
        self.last_cmd_recv_time = t
        self.cmd_queue.put((t, values))

    def target_changed(self, values):
        """受信した目標値が、直前の目標値と1つの自由度でも違うか"""
        prev = self.last_cmd_values
        for i in range(min(26, len(values))):
            if prev is not None:
                if i >= len(prev) or values[i] != prev[i]:
                    return True
            else:
                # 起動後まだ目標値トピックを受信していない → ボードが使っている目標値（整数）と比べる
                bd = self.board_desired[i]
                if bd is None or abs(values[i] - bd) >= 0.5:
                    return True
        return False

    def is_own_payload(self, values):
        """受信した目標値が、GUI自身が最近送ったものと同じか"""
        now = time.monotonic()
        while self.own_sent and now - self.own_sent[0][0] > OWN_ECHO_KEEP_SEC:
            self.own_sent.popleft()
        return any(values == sent for _, sent in self.own_sent)

    def publish(self):
        msg = Float32MultiArray()
        # ROS2 は各要素が Python の float 型であることを要求する（int は不可）
        msg.data = [float(v) for v in self.desired]
        # 自分の送信がトピック経由で戻ってきたときに「他プログラムによる変更」と誤認しないよう記録する
        now = time.monotonic()
        self.own_sent.append((now, to_float32_tuple(msg.data)))
        self.last_publish_time = now
        self.publisher.publish(msg)

    def publish_initial(self, initial=INITIAL_DESIRED):
        self.desired = [float(v) for v in initial]
        self.publish()


# ------------------------------
# 時系列グラフ（tk.Canvas）
# ------------------------------
class TimePlot:
    """1自由度の実測POT（赤）と目標値（青破線）の時系列を描く。横軸は現在時刻を 0 とした相対時間。

    線は毎回作り直さず coords() で更新し、目盛り・補助線はサイズや縦軸範囲が変わったときだけ描き直す。
    """
    # 余白（左, 右, 上, 下）[px]。上にはタイトル、左と下には目盛りを書く
    MARGINS = (46, 12, 22, 22)

    def __init__(self, canvas, dof):
        self.canvas = canvas
        self.dof = dof
        self.axes_key = None
        self.line_real = canvas.create_line(0, 0, 0, 0, fill="red", width=2, state="hidden")
        self.line_des = canvas.create_line(0, 0, 0, 0, fill="blue", width=2, dash=(4, 2), state="hidden")
        self.title = canvas.create_text(self.MARGINS[0], 4, anchor="nw", text=f"DOF {dof}",
                                        font=(UI_FONT, 9, "bold"))
        self.title_text = None

    def size(self):
        return int(self.canvas.cget('width')), int(self.canvas.cget('height'))

    def plot_area(self, w, h):
        left, right, top, bottom = self.MARGINS
        return left, top, max(1, w - left - right), max(1, h - top - bottom)

    @staticmethod
    def y_limits(pot_range, *series):
        """縦軸はPOT範囲。範囲外のデータがあるときだけ 50 刻みで広げる"""
        lo, hi = pot_range
        for s in series:
            s = s[np.isfinite(s)]
            if s.size:
                if s.min() < lo:
                    lo = math.floor(s.min() / 50) * 50
                if s.max() > hi:
                    hi = math.ceil(s.max() / 50) * 50
        return lo, hi

    def draw_axes(self, w, h, lo, hi, window):
        c = self.canvas
        c.delete("axes")
        x0, y0, plot_width, plot_height = self.plot_area(w, h)

        ## Y軸目盛り（グラフの高さに応じて 2〜5分割）
        y_div = max(2, min(5, plot_height // 30))
        for i in range(y_div + 1):
            y_val = lo + i*(hi - lo)/y_div
            y = y0 + plot_height * (1 - (y_val - lo)/(hi - lo))
            c.create_line(x0, y, x0 + plot_width, y, fill="#cccccc", dash=(2, 2), tags="axes")
            c.create_text(x0 - 6, y, text=str(int(y_val)), anchor="e", font=(UI_FONT, 8), tags="axes")

        # X軸補助線（表示時間幅とグラフの幅に応じて 0.5秒〜 の間隔）
        max_lines = max(2, min(12, plot_width // 45))
        step = next((s for s in (0.5, 1, 2, 5, 10, 20, 30, 60) if window / s <= max_lines), 120)
        k = 0
        while k*step <= window + 1e-9:
            x = x0 + plot_width * (1 - k*step/window)
            c.create_line(x, y0, x, y0 + plot_height, fill="#cccccc", dash=(2, 2), tags="axes")
            c.create_text(x, y0 + plot_height + 10, text=(f"-{k*step:g}" if k else "0 s"),
                          font=(UI_FONT, 8), tags="axes")
            k += 1

        c.tag_lower("axes")

    def set_line(self, item, x, y, max_points):
        ok = np.isfinite(y)
        x, y = x[ok], y[ok]
        if len(x) < 2:
            self.canvas.itemconfigure(item, state="hidden")
            return
        if len(x) > max_points:     # 1ピクセルに何点も描いても見えないので間引く
            idx = np.linspace(0, len(x) - 1, int(max_points)).astype(int)
            x, y = x[idx], y[idx]
        self.canvas.coords(item, np.column_stack((x, y)).ravel().tolist())
        self.canvas.itemconfigure(item, state="normal")

    def draw(self, t, real, desired, now, window, pot_range, title):
        w, h = self.size()
        lo, hi = self.y_limits(pot_range, real, desired)
        key = (w, h, lo, hi, window)
        if key != self.axes_key:
            self.draw_axes(w, h, lo, hi, window)
            self.axes_key = key

        if title != self.title_text:
            self.canvas.itemconfigure(self.title, text=title)
            self.title_text = title

        x0, y0, plot_width, plot_height = self.plot_area(w, h)
        x = x0 + plot_width * (1 - (now - t) / window)
        self.set_line(self.line_real, x, y0 + plot_height * (1 - (real - lo)/(hi - lo)), plot_width)
        self.set_line(self.line_des, x, y0 + plot_height * (1 - (desired - lo)/(hi - lo)), plot_width)


# ------------------------------
# Tkinter GUI
# ------------------------------
class PotGuiTk:
    def __init__(self, node: PotGuiNode, settings, settings_warning=None):
        self.node = node
        self.current_dof = 0

        # 初期姿勢・プリセット（SETTINGS_FILE の内容）
        self.settings = settings

        # プログラム側からスライダを動かしたときに on_slider が publish しないためのガード。
        # tk.Scale の command は set()/config() で値が変わったときにも（アイドル時に）呼ばれるので、
        # 時間ではなく「ユーザがスライダを操作したか」でガードを解除する。
        # （このガードが無いと、GUI起動時やDOF切替時に勝手に目標値が送信されてしまう）
        self.slider_programmatic = True

        # スライダをドラッグ中かどうか
        self.slider_active = False

        # 目標値が分かっている自由度（GUIが送信した / 目標値トピックを受信した / ボードの目標値を読み込んだ）。
        # 分かっていない自由度は、ボードが使っている目標値を受信した時点でそれを採用する。
        # これにより起動直後の最初の送信で、他の自由度を初期姿勢などの古い値で上書きしない。
        self.target_known = [False]*26

        # make_scrollable() で作ったスクロール領域の Canvas
        self.scroll_canvases = set()

        # グラフを表示中のDOF -> TimePlot
        self.plots = {}
        self.layout_job = None
        self.prefs_job = None

        # 記録の後処理（CSV作成・グラフ作成）の進み具合
        self.post = None

        # グラフ停止中: 停止した時刻（time.monotonic()）と、その時点の全DOFの時系列。動作中は None
        #   受信・記録は止めない（再開すると、停止中のデータも含めて最新の時系列を表示する）
        self.paused_at = None
        self.paused_history = None

        self.root = tk.Tk()

        # ウィジェットを作る前にフォントを決める（日本語の豆腐化対策）
        setup_japanese_font(self.root)

        self.root.title("POT GUI (time plot)")
        # 表示するモニターの大きさに合わせる（グラフの大きさ・並びはウィンドウの大きさに追従する。
        # 中身が収まらない部分はスクロールで見る）
        self.monitors = list_monitors()
        self.root.geometry(window_geometry_for(initial_monitor(self.root, self.monitors)))
        # ウィンドウマネージャが指定した位置を無視して別のモニターに出した場合に備え、表示後にも確かめる
        self.root.after(500, self.fit_window_to_monitor)

        # --------------------------
        # 画面上部のタブ
        # --------------------------
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        self.tab_pot = tk.Frame(self.notebook)
        self.tab_pose = tk.Frame(self.notebook)
        self.notebook.add(self.tab_pot, text="目標値 (POT)")
        self.notebook.add(self.tab_pose, text="初期姿勢・プリセット")

        self.build_pot_tab(self.tab_pot)
        self.build_pose_tab(self.tab_pose)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # マウスホイールでスクロールする領域（make_scrollable で登録）
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            self.root.bind_all(seq, self.on_mousewheel)

        self.update_ui()
        self.set_graph_dofs(self.settings['graph_dofs'])
        self.update_plots()
        self.poll_targets()
        self.poll_record()

        if settings_warning:
            self.root.after(300, lambda: messagebox.showwarning("Settings file", settings_warning))

    # =========================================================
    # 目標値タブ
    # =========================================================
    def build_pot_tab(self, parent):
        # 上: グラフ領域 / 下: 操作部。境界はドラッグで動かせる（ウィンドウを広げた分はグラフ領域が広がる）
        self.paned = ttk.PanedWindow(parent, orient=tk.VERTICAL)
        self.paned.pack(fill=tk.BOTH, expand=True)

        graph_frame = tk.Frame(self.paned)
        ctrl_frame = tk.Frame(self.paned)
        self.paned.add(graph_frame, weight=1)
        self.paned.add(ctrl_frame, weight=0)

        self.build_graph_area(graph_frame)
        # 操作部は境界を上げて狭くしてもスクロールで全部操作できるようにする
        _, self.ctrl_inner = self.make_scrollable(ctrl_frame, fit_width=True)
        self.build_controls(self.ctrl_inner)

        self.root.after(50, self.init_sash)

    def init_sash(self):
        """起動時の上下の境界: 操作部が全部見える高さを残し、残りをグラフ領域にする（グラフ領域は最低4割）"""
        total = self.paned.winfo_height()
        if total <= 1:      # まだ表示されていない
            self.root.after(50, self.init_sash)
            return
        need = self.ctrl_inner.winfo_reqheight() + 12
        self.paned.sashpos(0, max(int(total * 0.4), total - need))

    def build_graph_area(self, parent):
        bar = tk.Frame(parent)
        bar.pack(fill=tk.X, padx=6, pady=(4, 0))

        title = tk.Label(bar, text="グラフ表示DOF", font=(UI_FONT, 9, "bold"))
        title.grid(row=0, column=0, padx=(0, 6), sticky="w")
        boxes = tk.Frame(bar)
        boxes.grid(row=0, column=1, sticky="w")
        self.graph_dof_vars = []
        for i in range(26):
            v = tk.BooleanVar(value=False)
            tk.Checkbutton(boxes, text=str(i), variable=v,
                           command=self.on_graph_dof_toggle).grid(row=i//13, column=i%13, sticky="w")
            self.graph_dof_vars.append(v)

        sel = tk.Frame(bar)
        sel.grid(row=0, column=2, padx=8, sticky="w")
        tk.Button(sel, text="全選択", command=lambda: self.set_graph_dofs(range(26))).grid(row=0, column=0, sticky="ew")
        tk.Button(sel, text="全解除", command=lambda: self.set_graph_dofs([])).grid(row=1, column=0, sticky="ew")

        opt = tk.Frame(bar)
        opt.grid(row=0, column=3, padx=8, sticky="w")
        tk.Label(opt, text="表示時間幅 [s]").grid(row=0, column=0, sticky="e")
        self.window_var = tk.StringVar(value=str(PLOT_WINDOW_SEC))
        tk.Spinbox(opt, from_=1, to=HISTORY_SEC, increment=1, width=5,
                   textvariable=self.window_var).grid(row=0, column=1, sticky="w")
        self.pause_button = tk.Button(opt, text="‖ グラフ停止", width=12, command=self.toggle_graph_pause)
        self.pause_button.grid(row=0, column=2, padx=(12, 0), sticky="w")
        self.pause_label = tk.Label(opt, text="", fg="#c00000", font=(UI_FONT, 9, "bold"))
        self.pause_label.grid(row=1, column=2, padx=(12, 0), sticky="w")
        legend = tk.Canvas(opt, width=330, height=18, highlightthickness=0)
        legend.grid(row=1, column=0, columnspan=2, sticky="w")
        legend.create_line(2, 9, 24, 9, fill="red", width=2)
        legend.create_text(28, 9, text="実測POT", anchor="w", font=(UI_FONT, 9))
        legend.create_line(92, 9, 114, 9, fill="blue", width=2, dash=(4, 2))
        legend.create_text(118, 9, text="目標値（ボード）", anchor="w", font=(UI_FONT, 9))
        legend.create_rectangle(226, 3, 238, 15, outline="#ff8c00", width=2)
        legend.create_text(242, 9, text="スライダのDOF", anchor="w", font=(UI_FONT, 9))

        # ウィンドウが狭いときは「表示時間幅・凡例」を2段目に回す
        wrapped = [False]

        def wrap(_event=None):
            one_line = sum(w.winfo_reqwidth() for w in (title, boxes, sel, opt)) + 40
            want = bar.winfo_width() < one_line
            if want != wrapped[0]:
                wrapped[0] = want
                if want:
                    opt.grid(row=1, column=1, columnspan=2, padx=0, pady=(2, 0), sticky="w")
                else:
                    opt.grid(row=0, column=3, columnspan=1, padx=8, pady=0, sticky="w")

        bar.bind("<Configure>", wrap)

        self.graph_canvas, self.graph_inner = self.make_scrollable(parent, fit_width=True, keep_min_width=False)
        self.graph_canvas.bind("<Configure>", lambda e: self.schedule_layout(), add="+")
        self.graph_empty = tk.Label(self.graph_inner, text="グラフを表示するDOFを上で選択してください",
                                    fg="#808080")

    def build_controls(self, parent):
        # 横に3列: 左 = Random Target / 中央 = 目標値の入力（スライダ・Reset pose・プリセット・Step Input）/ 右 = 記録。
        # 操作部を横長・縦短にして、上のグラフ領域を縦に広く取る。
        self.ctrl_panels = tk.Frame(parent)
        self.ctrl_panels.pack(fill=tk.X, padx=4, pady=2)

        left = tk.LabelFrame(self.ctrl_panels, text="Random Target")
        middle = tk.LabelFrame(self.ctrl_panels, text="目標値の入力")
        right = tk.LabelFrame(self.ctrl_panels, text="記録（全DOFの 実測POT・目標値・z3・PWM）")
        self.build_random_area(left)
        self.build_target_area(middle)
        self.build_record_area(right)

        self.ctrl_columns = (left, middle, right)
        self.ctrl_layout_mode = None
        self.layout_controls()
        # ウィンドウ幅が変わったら並べ方を見直す
        self.ctrl_inner.master.bind("<Configure>", lambda e: self.layout_controls(), add="+")

    def layout_controls(self):
        """操作部の3列の並べ方: 横に並びきるなら1段、狭ければ中央を上段・左右を下段にする"""
        left, middle, right = self.ctrl_columns
        view_width = self.ctrl_inner.master.winfo_width()
        one_row = sum(c.winfo_reqwidth() for c in self.ctrl_columns) + 30
        mode = 'row' if view_width <= 1 or view_width >= one_row else 'stack'
        if mode == self.ctrl_layout_mode:
            return
        self.ctrl_layout_mode = mode

        panels = self.ctrl_panels
        if mode == 'row':
            left.grid(row=0, column=0, columnspan=1, sticky="nsew", padx=3, pady=2)
            middle.grid(row=0, column=1, columnspan=1, sticky="nsew", padx=3, pady=2)
            right.grid(row=0, column=2, columnspan=1, sticky="nsew", padx=3, pady=2)
            for c in range(3):      # 余った幅は3列で分け合う（左右に隙間を作らない）
                panels.columnconfigure(c, weight=1)
        else:
            middle.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=3, pady=2)
            left.grid(row=1, column=0, columnspan=1, sticky="nsew", padx=3, pady=2)
            right.grid(row=1, column=1, columnspan=1, sticky="nsew", padx=3, pady=2)
            panels.columnconfigure(0, weight=1)
            panels.columnconfigure(1, weight=1)
            panels.columnconfigure(2, weight=0)

    def build_random_area(self, parent):
        tk.Label(parent, text="Random DOF", font=(UI_FONT, 9)).pack(pady=(2, 0))
        # ボードごとに1行（board1: DOF0-5, …, board5: DOF22-25）
        boxes = tk.Frame(parent)
        boxes.pack(padx=4)
        self.random_enable_vars = [None]*26
        for r, (board, offset, count) in enumerate(BOARD_LAYOUT):
            tk.Label(boxes, text=board, font=(UI_FONT, 8), fg="#606060").grid(row=r, column=0, sticky="w", padx=(0, 4))
            for local in range(count):
                i = offset + local
                v = tk.BooleanVar(value=RANDOM_ENABLE[i])
                tk.Checkbutton(boxes, text=str(i), variable=v).grid(row=r, column=local + 1, sticky="w")
                self.random_enable_vars[i] = v
        self.random_button = tk.Button(parent, text="Send Random Target", command=self.send_random)
        self.random_button.pack(pady=4)

    def build_target_area(self, parent):
        top = tk.Frame(parent)
        top.pack(padx=4, pady=(2, 0))

        # DOF selector（スライダで操作するDOF）
        tk.Label(top, text="スライダのDOF").pack(side=tk.LEFT, padx=(0, 4))
        self.dof_box = ttk.Combobox(top, values=[f"DOF {i}" for i in range(26)], state="readonly", width=8)
        self.dof_box.current(0)
        self.dof_box.bind("<<ComboboxSelected>>", self.change_dof)
        self.dof_box.pack(side=tk.LEFT)

        self.info_label = tk.Label(top, text="", font=(UI_FONT, 10), width=30, anchor="w")
        self.info_label.pack(side=tk.LEFT, padx=(12, 0))

        self.real_label = tk.Label(top, text="", font=(UI_FONT, 10), width=36, anchor="w")
        self.real_label.pack(side=tk.LEFT)

        # Slider
        self.slider = tk.Scale(parent, orient=tk.HORIZONTAL, length=800, command=self.on_slider)
        # ユーザがスライダに触れたらガードを解除する（これ以降の command は本人の操作）
        self.slider.bind("<Button-1>", self.on_slider_press)
        self.slider.bind("<ButtonRelease-1>", self.on_slider_release)
        self.slider.bind("<Key>", self.on_slider_key)
        self.slider.pack(padx=4)

        # Reset・プリセット
        pose_row = tk.Frame(parent)
        pose_row.pack(pady=(2, 0))

        self.init_button = tk.Button(pose_row, text="Reset pose", command=self.send_initial)
        self.init_button.pack(side=tk.LEFT, padx=(0, 16))

        self.preset_buttons = []
        for k in range(NUM_PRESETS):
            b = tk.Button(pose_row, text=f"プリセット{k+1}", command=lambda k=k: self.send_preset(k))
            b.pack(side=tk.LEFT, padx=2)
            self.preset_buttons.append(b)

        tk.Button(pose_row, text=f"現在の目標値をプリセット{REGISTER_PRESET_INDEX+1}に登録",
                  command=self.register_current_preset).pack(side=tk.LEFT, padx=(16, 0))
        self.refresh_preset_buttons()

        self.pot_status = tk.Label(parent, text="", font=(UI_FONT, 9), wraplength=800)
        self.pot_status.pack()

        # --------------------------
        # ★ Step入力エリア（送信ボタンは入力欄の右）
        # --------------------------
        step = tk.Frame(parent)
        step.pack(padx=4, pady=(0, 4))
        tk.Label(step, text="Step Input (All DOF)", font=(UI_FONT, 10, "bold")).grid(row=0, column=0, sticky="w")

        self.step_entries = []
        # GUIが各欄に最後に書いた文字列。これと違う欄は「手入力して未送信」とみなし、自動更新で上書きしない
        self.step_written = [""]*26
        frame = tk.Frame(step)
        frame.grid(row=1, column=0)

        for i in range(26):
            sub = tk.Frame(frame)
            sub.grid(row=i//13, column=i%13, padx=3, pady=1)

            tk.Label(sub, text=f"{i}").pack()
            entry = tk.Entry(sub, width=5)
            entry.pack()
            entry.bind("<KeyRelease>", lambda e, idx=i: self.color_step_entry(idx))

            self.step_entries.append(entry)
            self.write_step_entry(i, force=True)

        self.step_button = tk.Button(step, text="Send Step Input", command=self.send_step)
        self.step_button.grid(row=1, column=1, padx=(10, 0))

    def build_record_area(self, parent):
        row1 = tk.Frame(parent)
        row1.pack(fill=tk.X, padx=4, pady=(4, 2))
        self.record_button = tk.Button(row1, text="● 記録開始", width=12, command=self.on_record_button)
        self.record_button.pack(side=tk.LEFT)
        self.record_state = tk.Label(row1, text="", width=10, fg="white", font=(UI_FONT, 11, "bold"))
        self.record_state.pack(side=tk.LEFT, padx=8)

        row2 = tk.Frame(parent)
        row2.pack(fill=tk.X, padx=4, pady=2)
        tk.Label(row2, text="記録時間 [s]").pack(side=tk.LEFT)
        self.duration_var = tk.StringVar()
        self.duration_spin = tk.Spinbox(row2, from_=1, to=RECORD_DURATION_MAX, increment=1, width=7,
                                        textvariable=self.duration_var)
        self.duration_spin.pack(side=tk.LEFT, padx=(4, 0))
        # Spinbox は作成時に値を increment の桁に丸めてしまうので、作成後に設定する（12.5 → 12 にしない）
        self.duration_var.set(format_value(self.settings['record_duration']))
        # 記録中に変えてもすぐ反映する
        self.duration_var.trace_add("write", lambda *a: self.on_duration_change())

        row3 = tk.Frame(parent)
        row3.pack(fill=tk.X, padx=4, pady=2)
        tk.Label(row3, text="保存先").pack(side=tk.LEFT)
        self.record_dir_var = tk.StringVar(value=self.settings['record_dir'] or os.getcwd())
        tk.Entry(row3, textvariable=self.record_dir_var, width=34).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        tk.Button(row3, text="参照", command=self.choose_record_dir).pack(side=tk.LEFT)

        # 状態の説明（3行ぶん確保して、文の長さで操作部の高さが変わらないようにする）
        self.record_detail = tk.Label(parent, text="", font=(UI_FONT, 9), anchor="nw", justify="left",
                                      wraplength=380, height=3)
        self.record_detail.pack(fill=tk.X, padx=4, pady=(2, 4))
        self.show_record_state('idle', "「● 記録開始」を押すと待機状態になり、目標値が変わった時点から記録します")

    # =========================================================
    # 初期姿勢・プリセットタブ
    # =========================================================
    def build_pose_tab(self, parent):
        ctrl = tk.Frame(parent)
        ctrl.pack(fill=tk.X, pady=5)

        tk.Button(ctrl, text="保存", width=8, command=self.save_pose_tab).pack(side=tk.LEFT, padx=4)
        tk.Button(ctrl, text="編集を破棄", command=self.revert_pose_tab).pack(side=tk.LEFT, padx=4)
        self.pose_status = tk.Label(ctrl, text="", font=(UI_FONT, 9))
        self.pose_status.pack(side=tk.LEFT, padx=12)

        tk.Label(parent,
                 text="初期姿勢 … 目標値タブの「Reset pose」で送信する値（起動時には送信しません）。\n"
                      f"プリセット1〜{REGISTER_PRESET_INDEX} … ここで編集し「保存」で確定します。"
                      "列をすべて空欄にして保存すると未登録になります。\n"
                      f"プリセット{REGISTER_PRESET_INDEX+1} … 目標値タブの"
                      f"「現在の目標値をプリセット{REGISTER_PRESET_INDEX+1}に登録」で登録します（ここでは表示のみ）。\n"
                      f"値は「範囲」内で入力してください。保存先: {SETTINGS_FILE}",
                 font=(UI_FONT, 9), fg="#505050", justify="left").pack(anchor="w", padx=6)

        _, inner = self.make_scrollable(parent)

        # 列: 'init' = 初期姿勢, 0..4 = プリセット1..5
        self.pose_columns = ['init'] + list(range(NUM_PRESETS))
        self.pose_entries = {}

        tk.Label(inner, text="DOF", width=5, font=(UI_FONT, 9, "bold")).grid(row=0, column=0, padx=2)
        tk.Label(inner, text="範囲", width=10, font=(UI_FONT, 9, "bold")).grid(row=0, column=1, padx=2)

        for c, col in enumerate(self.pose_columns):
            gc = c + 2
            editable = (col != REGISTER_PRESET_INDEX)
            title = "初期姿勢" if col == 'init' else f"プリセット{col+1}"
            tk.Label(inner, text=title, font=(UI_FONT, 9, "bold")).grid(row=0, column=gc, padx=4)

            if editable:
                tk.Button(inner, text="現在の目標値", font=(UI_FONT, 8),
                          command=lambda col=col: self.fill_pose_column(col, self.node.desired)
                          ).grid(row=1, column=gc, padx=4, pady=1, sticky="ew")
            if editable and col != 'init':
                tk.Button(inner, text="クリア", font=(UI_FONT, 8),
                          command=lambda col=col: self.fill_pose_column(col, None)
                          ).grid(row=2, column=gc, padx=4, pady=1, sticky="ew")

            entries = []
            for dof in range(26):
                e = tk.Entry(inner, width=9, justify="right")
                e.grid(row=dof + 3, column=gc, padx=4, pady=1)
                if editable:
                    e.bind("<KeyRelease>", lambda ev: self.update_pose_dirty())
                entries.append(e)
            self.pose_entries[col] = entries

        for dof in range(26):
            mn, mx = POT_RANGE[dof]
            tk.Label(inner, text=str(dof), width=5).grid(row=dof + 3, column=0, padx=2)
            tk.Label(inner, text=f"[{mn}, {mx}]", width=10).grid(row=dof + 3, column=1, padx=2)

        self.revert_pose_tab()
        self.pose_status.config(text="")

    def make_scrollable(self, parent, fit_width=False, keep_min_width=True):
        """スクロールできる領域を作り、(Canvas, 中身を置く Frame) を返す

        fit_width: 中身の幅を表示領域の幅に合わせる（中央寄せや、幅に合わせたグラフの配置のため）。
        keep_min_width: 中身の必要幅より狭くはせず、表示領域のほうが狭いときは横スクロールバーを出す。
        """
        outer = tk.Frame(parent)
        outer.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)
        outer.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)

        canvas = tk.Canvas(outer, highlightthickness=0)
        vbar = tk.Scrollbar(outer, orient=tk.VERTICAL, command=canvas.yview)
        hbar = tk.Scrollbar(outer, orient=tk.HORIZONTAL, command=canvas.xview)
        inner = tk.Frame(canvas)

        window = canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=vbar.set, xscrollcommand=hbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        vbar.grid(row=0, column=1, sticky="ns")

        def update(_event=None):
            view_width = canvas.winfo_width()
            if fit_width:
                canvas.itemconfigure(window, width=(max(view_width, inner.winfo_reqwidth())
                                                    if keep_min_width else view_width))
            # 中身が表示領域より広いときだけ横スクロールバーを出す
            if keep_min_width and inner.winfo_reqwidth() > view_width > 1:
                hbar.grid(row=1, column=0, sticky="ew")
            else:
                hbar.grid_remove()
                canvas.xview_moveto(0)
            canvas.configure(scrollregion=canvas.bbox("all"))

        inner.bind("<Configure>", update)
        canvas.bind("<Configure>", update)

        # マウスホイールでのスクロールは on_mousewheel がポインタ位置から対象を決める
        self.scroll_canvases.add(canvas)
        return canvas, inner

    def fit_window_to_monitor(self):
        """表示されたモニターからウィンドウがはみ出していたら、そのモニターに収まる大きさ・位置にする"""
        if not self.monitors:
            return
        x, y = self.root.winfo_rootx(), self.root.winfo_rooty()
        w, h = self.root.winfo_width(), self.root.winfo_height()
        mon = monitor_at(self.monitors, x + w // 2, y + h // 2) or monitor_at(self.monitors, x, y)
        if mon is None:
            mon = initial_monitor(self.root, self.monitors)
        mx, my, mw, mh, _ = mon
        if x < mx or y < my or x + w > mx + mw or y + h > my + mh:
            self.root.geometry(window_geometry_for(mon))

    def on_mousewheel(self, event):
        """ポインタの下にあるスクロール領域をスクロールする（子ウィジェットの上でも効くように親をたどる）"""
        try:
            w = self.root.winfo_containing(event.x_root, event.y_root)
        except (KeyError, tk.TclError):
            return
        while w is not None:
            if w in self.scroll_canvases:
                delta = -1 if (event.num == 5 or event.delta < 0) else 1
                if event.state & 0x0001:    # Shift + ホイールは横スクロール
                    w.xview_scroll(-delta, "units")
                else:
                    w.yview_scroll(-delta, "units")
                return
            w = w.master

    @staticmethod
    def set_entry_text(entry, text):
        readonly = str(entry.cget("state")) == "readonly"
        if readonly:
            entry.config(state="normal")
        entry.delete(0, tk.END)
        entry.insert(0, text)
        if readonly:
            entry.config(state="readonly")

    def fill_pose_column(self, col, values):
        """表の1列に値を書く（values=None なら空欄）。保存はしない"""
        for dof, e in enumerate(self.pose_entries[col]):
            self.set_entry_text(e, "" if values is None else format_value(values[dof]))
            e.config(bg="white")
        self.update_pose_dirty()

    def committed_pose(self, col):
        if col == 'init':
            return self.settings['initial_pose']
        return self.settings['presets'][col]

    def revert_pose_tab(self):
        for col in self.pose_columns:
            self.fill_pose_column(col, self.committed_pose(col))
            if col == REGISTER_PRESET_INDEX:
                for e in self.pose_entries[col]:
                    e.config(state="readonly")
        self.update_pose_dirty()

    def pose_tab_dirty(self):
        for col in self.pose_columns:
            if col == REGISTER_PRESET_INDEX:
                continue
            committed = self.committed_pose(col)
            for dof, e in enumerate(self.pose_entries[col]):
                want = "" if committed is None else format_value(committed[dof])
                if e.get().strip() != want:
                    return True
        return False

    def update_pose_dirty(self):
        if self.pose_tab_dirty():
            self.pose_status.config(text="未保存の変更があります", fg="#c06000")
        elif self.pose_status.cget("text") == "未保存の変更があります":
            self.pose_status.config(text="")

    def read_pose_column(self, col):
        """表の1列を読む。戻り値 (値のリスト or None(空欄=未登録), [(dof, 理由), ...])"""
        texts = [e.get().strip() for e in self.pose_entries[col]]
        if col != 'init' and all(t == "" for t in texts):
            return None, []

        values, errors = [], []
        for dof, text in enumerate(texts):
            try:
                v = float(text)
                if not math.isfinite(v):
                    raise ValueError
            except ValueError:
                errors.append((dof, "数値ではありません" if text else "空欄です"))
                values.append(None)
                continue
            mn, mx = POT_RANGE[dof]
            if not (mn <= v <= mx):
                errors.append((dof, f"範囲 [{mn}, {mx}] の外です"))
            values.append(v)
        return values, errors

    def save_pose_tab(self):
        new_values = {}
        problems = []
        for col in self.pose_columns:
            if col == REGISTER_PRESET_INDEX:
                continue
            values, errors = self.read_pose_column(col)
            title = "初期姿勢" if col == 'init' else f"プリセット{col+1}"
            for e in self.pose_entries[col]:
                e.config(bg="white")
            for dof, reason in errors:
                self.pose_entries[col][dof].config(bg="#ffc0c0")
                problems.append(f"{title} DOF{dof}: {reason}")
            new_values[col] = values

        if problems:
            more = f"\n…ほか {len(problems) - 15} 件" if len(problems) > 15 else ""
            messagebox.showerror("Input error", "保存できません（赤い欄を直してください）:\n"
                                 + "\n".join(problems[:15]) + more)
            return False

        old = (list(self.settings['initial_pose']), list(self.settings['presets']))
        for col, values in new_values.items():
            if col == 'init':
                self.settings['initial_pose'] = values
            else:
                self.settings['presets'][col] = values
        if not self.write_settings():
            self.settings['initial_pose'], self.settings['presets'] = old
            return False

        self.revert_pose_tab()      # 表示を保存した値（整形済み）に揃える
        self.refresh_preset_buttons()
        self.pose_status.config(text=f"[{time.strftime('%H:%M:%S')}] 保存しました", fg="#006000")
        return True

    def write_settings(self):
        try:
            save_settings(self.settings)
        except OSError as e:
            messagebox.showerror("Save error", f"{SETTINGS_FILE} に保存できません:\n{e}")
            return False
        return True

    # =========================================================
    # 目標値タブの処理
    # =========================================================
    def set_pot_status(self, text, color="#006000"):
        self.pot_status.config(text=f"[{time.strftime('%H:%M:%S')}] {text}", fg=color)

    def refresh_preset_buttons(self):
        for k, b in enumerate(self.preset_buttons):
            b.config(state=(tk.NORMAL if self.settings['presets'][k] is not None else tk.DISABLED))

    def step_entry_dirty(self, i):
        return self.step_entries[i].get().strip() != self.step_written[i]

    def color_step_entry(self, i):
        # 手入力して未送信の欄は黄色（他プログラムの目標値で上書きされない）
        self.step_entries[i].config(bg=("#fff3b0" if self.step_entry_dirty(i) else "white"))

    def write_step_entry(self, i, force=False):
        """Step Input 欄に現在の目標値を書く。手入力して未送信の欄は force でない限り触らない"""
        if not force and self.step_entry_dirty(i):
            return
        want = str(int(self.node.desired[i]))
        e = self.step_entries[i]
        if e.get() != want:
            e.delete(0, tk.END)
            e.insert(0, want)
        self.step_written[i] = want
        self.color_step_entry(i)

    def change_dof(self, event):
        # 時系列は全自由度ぶん常に保持しているので、切り替えてもグラフは途切れない
        self.current_dof = self.dof_box.current()
        self.update_ui()
        # 従来どおり選んだDOFのグラフが見えるよう、表示DOFに加える（他の表示DOFはそのまま）
        if not self.graph_dof_vars[self.current_dof].get():
            self.graph_dof_vars[self.current_dof].set(True)
            self.on_graph_dof_toggle()
        else:
            self.highlight_slider_plot()

    def update_ui(self):
        mn, mx = POT_RANGE[self.current_dof]
        desired = int(self.node.desired[self.current_dof])

        # config()/set() はどちらも command=on_slider を呼び得るのでガードしてから触る
        self.slider_programmatic = True
        self.slider.config(from_=mn, to=mx)
        self.slider.set(desired)

        self.update_info_label()

    def update_info_label(self):
        mn, mx = POT_RANGE[self.current_dof]
        desired = int(self.node.desired[self.current_dof])
        self.info_label.config(text=f"Desired: {desired} Range: [{mn},{mx}]")

    # -------------------------
    def set_slider_silently(self, value):
        """on_slider から publish させずにスライダの表示だけ更新する"""
        self.slider_programmatic = True
        self.slider.set(value)

    def on_slider_press(self, event):
        # ユーザ操作の開始。以降の command は本人の操作によるものなので送信してよい
        self.slider_programmatic = False
        self.slider_active = True

    def on_slider_release(self, event):
        self.slider_active = False

    def on_slider_key(self, event):
        self.slider_programmatic = False

    def on_slider(self, value):
        # プログラムからスライダを動かした場合（起動時・DOF切替・外部目標値の反映）は送信しない
        if self.slider_programmatic:
            return

        val = float(value)
        self.node.desired[self.current_dof] = float(val)

        # Entryも同期
        self.write_step_entry(self.current_dof, force=True)
        self.update_info_label()

        self.node.publish()
        self.target_known = [True]*26

    def after_send(self, text, overwrite=range(26)):
        """全自由度の目標値を送信した後の表示更新。overwrite の自由度は手入力中の欄も送信値で書き換える"""
        self.target_known = [True]*26
        for i in range(26):
            self.write_step_entry(i, force=(i in overwrite))
        self.update_ui()
        self.set_pot_status(text)

    def send_initial(self):
        self.node.publish_initial(self.settings['initial_pose'])
        self.after_send("初期姿勢 (Reset pose) を送信しました")

    def send_preset(self, k):
        values = self.settings['presets'][k]
        if values is None:
            return
        self.node.desired = [float(v) for v in values]
        self.node.publish()
        self.after_send(f"プリセット{k+1} を送信しました")

    def register_current_preset(self):
        k = REGISTER_PRESET_INDEX
        old = self.settings['presets'][k]
        self.settings['presets'][k] = [float(v) for v in self.node.desired]
        if not self.write_settings():
            self.settings['presets'][k] = old
            return
        self.fill_pose_column(k, self.settings['presets'][k])
        self.refresh_preset_buttons()
        self.set_pot_status(f"現在の目標値をプリセット{k+1}に登録しました")

    # -------------------------
    # ★ Step送信
    # -------------------------
    def send_step(self):
        clamped = []
        for i in range(26):
            try:
                val = float(self.step_entries[i].get().strip())
                mn, mx = POT_RANGE[i]
                limited = float(max(mn, min(mx, val)))  # 範囲制限（POT_RANGE、float で保持）
                if limited != val:
                    clamped.append(i)
                self.node.desired[i] = limited
            except (ValueError, IndexError):
                pass

        self.node.publish()
        self.after_send("Step Input を送信しました")
        if clamped:
            self.set_pot_status("Step Input を送信しました（POT_RANGE の外だったため範囲内に丸めた DOF: "
                                f"{', '.join(map(str, clamped))}）", "#c06000")


    def send_random(self):
        changed=[]
        for i in range(26):
            if not self.random_enable_vars[i].get():
                continue
            mn,mx=RANDOM_RANGE[i]
            val=random.randint(int(mn),int(mx))
            self.node.desired[i]=float(val)
            changed.append(i)
        self.node.publish()
        # 従来どおり、ランダム対象外の欄に手入力した値はそのまま残す
        self.after_send("Random Target を送信しました", overwrite=changed)

    # -------------------------
    # 他プログラムが変更した目標値・ボードが使っている目標値を画面に反映する
    # -------------------------
    def poll_targets(self):
        external = None
        while True:
            try:
                t_recv, values = self.node.cmd_queue.get_nowait()
            except queue.Empty:
                break
            if self.node.is_own_payload(values):
                continue    # 自分の送信がトピック経由で戻ってきたもの
            if t_recv < self.node.last_publish_time:
                continue    # 受信後にGUIが全自由度を送り直しているので、ボードにはGUIの値が届いている
            external = values

        if external is not None:
            for i in range(min(26, len(external))):
                self.node.desired[i] = float(external[i])
                self.target_known[i] = True
            self.set_pot_status("他プログラムから目標値を受信して反映しました", "#0050a0")

        # ボードが実際に使っている目標値に合わせる
        #   ・起動直後など、まだ目標値が分かっていない自由度 → すぐに採用
        #   ・それ以外 → 目標値トピックの送受信がしばらく無く、ボードの値が落ち着いていて、
        #     GUIの値と1以上違うとき（ボード再起動・受信漏れなど）
        now = time.monotonic()
        quiet = now - max(self.node.last_publish_time, self.node.last_cmd_recv_time) > ECHO_SYNC_QUIET_SEC
        adopted = []
        for i in range(26):
            bd = self.node.board_desired[i]
            if bd is None:
                continue
            if not self.target_known[i]:
                adopted.append(i)
            elif (quiet and abs(bd - self.node.desired[i]) >= 1.0
                    and now - self.node.board_desired_changed[i] > ECHO_SYNC_STABLE_SEC):
                adopted.append(i)
        for i in adopted:
            self.node.desired[i] = self.node.board_desired[i]
            self.target_known[i] = True
        if adopted and external is None:
            dofs = ", ".join(map(str, adopted)) if len(adopted) <= 8 else f"{len(adopted)}自由度"
            self.set_pot_status(f"ボードが使っている目標値を表示に反映しました (DOF {dofs})", "#0050a0")

        if external is not None or adopted:
            for i in range(26):
                self.write_step_entry(i)
            # スライダの表示を更新（ドラッグ中は触らない）
            if not self.slider_active:
                mn, mx = POT_RANGE[self.current_dof]
                cur = int(max(mn, min(mx, self.node.desired[self.current_dof])))
                if int(self.slider.get()) != cur:
                    self.set_slider_silently(cur)
            self.update_info_label()

        self.root.after(100, self.poll_targets)

    # -------------------------
    def plot_window(self):
        try:
            w = float(self.window_var.get())
        except (ValueError, tk.TclError):
            w = PLOT_WINDOW_SEC
        return min(max(w, 0.5), HISTORY_SEC)

    def toggle_graph_pause(self):
        """グラフの停止・再開。停止中も受信・記録は続き、表示DOF・表示時間幅・ウィンドウの大きさは変えられる"""
        if self.paused_at is None:
            now = time.monotonic()
            self.paused_history = self.node.snapshot_history(now - HISTORY_SEC)
            self.paused_at = now
            self.pause_button.config(text="▶ グラフ再開", bg="#ffd27f", activebackground="#ffc04d")
            self.pause_label.config(text=f"停止中（{time.strftime('%H:%M:%S')} の時点）")
        else:
            self.paused_at = None
            self.paused_history = None
            default_bg = self.root.cget("bg")
            self.pause_button.config(text="‖ グラフ停止", bg=default_bg, activebackground=default_bg)
            self.pause_label.config(text="")

    def update_plots(self):
        start = time.perf_counter()
        window = self.plot_window()
        if self.paused_at is None:
            now, snapshot = time.monotonic(), None
        else:
            now, snapshot = self.paused_at, self.paused_history

        # スクロール領域のうち見えている範囲のグラフだけ描く
        top = self.graph_canvas.canvasy(0)
        bottom = top + self.graph_canvas.winfo_height()
        for dof, plot in self.plots.items():
            c = plot.canvas
            if not c.winfo_ismapped() or c.winfo_y() + c.winfo_height() < top or c.winfo_y() > bottom:
                continue
            t, real, desired_hist = self.node.dof_history(dof, now - window, snapshot)
            if snapshot is None:
                pot_now, des_now = self.node.real_raw[dof], self.node.board_desired[dof]
            else:   # 停止中は停止した時点の値を表示する
                pot_now = real[-1] if len(real) else float('nan')
                des_now = desired_hist[-1] if len(desired_hist) else float('nan')
            title = (f"DOF {dof}    POT {'-' if pot_now is None or pot_now != pot_now else int(pot_now)}"
                     f"    目標 {'-' if des_now is None or des_now != des_now else int(des_now)}")
            plot.draw(t, real, desired_hist, now, window, POT_RANGE[dof], title)

        dof = self.current_dof
        raw = self.node.real_raw[dof]
        desired = self.node.desired[dof]
        bd = self.node.board_desired[dof]
        board_text = "-" if bd is None else str(int(bd))
        self.real_label.config(text=f"Real: {int(raw)}  Desired: {int(desired)}  (board: {board_text})")

        # グラフが多い・表示時間幅が長いなどで描画に時間がかかるときは、更新間隔を延ばしてGUIの応答性を保つ
        cost = time.perf_counter() - start
        interval = min(500, max(50, cost * 4000))
        if self.paused_at is not None:      # 停止中は表示DOF・表示時間幅・大きさの変更に追従できれば十分
            interval = max(interval, 250)
        self.root.after(int(interval), self.update_plots)

    # -------------------------
    # グラフの表示DOF・配置
    # -------------------------
    def set_graph_dofs(self, dofs):
        dofs = set(dofs)
        for i, v in enumerate(self.graph_dof_vars):
            v.set(i in dofs)
        self.on_graph_dof_toggle()

    def on_graph_dof_toggle(self):
        selected = [i for i, v in enumerate(self.graph_dof_vars) if v.get()]
        for dof in list(self.plots):
            if dof not in selected:
                self.plots.pop(dof).canvas.destroy()
        for dof in selected:
            if dof not in self.plots:
                canvas = tk.Canvas(self.graph_inner, width=PLOT_MIN_W, height=PLOT_MIN_H, bg="#eeeeee",
                                   highlightthickness=PLOT_BORDER)
                self.plots[dof] = TimePlot(canvas, dof)
        self.highlight_slider_plot()
        self.schedule_layout()

        if selected != self.settings['graph_dofs']:
            self.settings['graph_dofs'] = selected
            self.save_prefs_later()

    def highlight_slider_plot(self):
        for dof, plot in self.plots.items():
            color = "#ff8c00" if dof == self.current_dof else "#c8c8c8"
            plot.canvas.config(highlightbackground=color, highlightcolor=color)

    def schedule_layout(self):
        # ウィンドウのリサイズ中は何度も呼ばれるので、まとめて1回だけ並べ直す
        if self.layout_job is not None:
            self.root.after_cancel(self.layout_job)
        self.layout_job = self.root.after(50, self.layout_plots)

    def layout_plots(self):
        """表示領域の大きさに合わせてグラフの列数と大きさを決めて並べる"""
        self.layout_job = None
        dofs = sorted(self.plots)
        if not dofs:
            self.graph_empty.grid(row=0, column=0, padx=20, pady=20)
            return
        self.graph_empty.grid_remove()

        width, height = self.graph_canvas.winfo_width(), self.graph_canvas.winfo_height()
        if width < 50 or height < 20:   # まだ表示されていない・境界で畳まれている
            return
        cols, cell_w, cell_h = grid_layout(len(dofs), width, height)
        inset = PLOT_PAD + PLOT_BORDER
        for i, dof in enumerate(dofs):
            canvas = self.plots[dof].canvas
            canvas.config(width=max(10, cell_w - 2*inset), height=max(10, cell_h - 2*inset))
            canvas.grid(row=i // cols, column=i % cols, padx=PLOT_PAD, pady=PLOT_PAD)

    # -------------------------
    # 記録
    # -------------------------
    RECORD_STATE_STYLE = {
        # 状態: (表示, 背景色)
        'idle':      ("停止中", "#909090"),
        'armed':     ("待機中", "#e08000"),
        'recording': ("● 記録中", "#d00000"),
        'saving':    ("保存中", "#2060c0"),
        'done':      ("保存完了", "#208020"),
        'error':     ("エラー", "#700000"),
    }

    def show_record_state(self, state, detail):
        text, color = self.RECORD_STATE_STYLE[state]
        self.record_state.config(text=text, bg=color)
        self.record_detail.config(text=detail)
        if state == 'armed':
            self.record_button.config(text="待機を取消", state=tk.NORMAL)
        elif state == 'recording':
            self.record_button.config(text="■ 記録終了", state=tk.NORMAL)
        elif state == 'saving':
            self.record_button.config(text="● 記録開始", state=tk.DISABLED)
        else:
            self.record_button.config(text="● 記録開始", state=tk.NORMAL)

    def read_duration(self):
        try:
            v = float(self.duration_var.get())
        except ValueError:
            return None
        return v if 0 < v <= RECORD_DURATION_MAX else None

    def on_duration_change(self):
        v = self.read_duration()
        self.duration_spin.config(bg=("white" if v is not None else "#ffc0c0"))
        if v is None:
            return
        session = self.node.session
        if session is not None:
            session.duration = v
        if v != self.settings['record_duration']:
            self.settings['record_duration'] = v
            self.save_prefs_later()

    def choose_record_dir(self):
        path = filedialog.askdirectory(initialdir=self.record_dir_var.get() or os.getcwd(),
                                       title="Folder for recordings")
        if path:
            self.record_dir_var.set(path)

    def on_record_button(self):
        session = self.node.session
        if session is None:
            self.arm_record()
        else:
            session.finish()    # 待機中なら取消、記録中なら終了

    def arm_record(self):
        duration = self.read_duration()
        if duration is None:
            messagebox.showerror("Record duration", f"記録時間は 0 より大きく {RECORD_DURATION_MAX:g} 以下の秒数で入力してください")
            return
        base_dir = os.path.abspath(os.path.expanduser(self.record_dir_var.get().strip() or os.getcwd()))
        try:
            os.makedirs(base_dir, exist_ok=True)
            if not os.access(base_dir, os.W_OK):
                raise OSError("書き込み権限がありません")
        except OSError as e:
            messagebox.showerror("Save folder", f"{base_dir} に保存できません:\n{e}")
            return
        self.record_dir_var.set(base_dir)
        if base_dir != self.settings['record_dir']:
            self.settings['record_dir'] = base_dir
            self.save_prefs_later()

        self.node.session = RecordSession(base_dir, duration)
        self.show_record_state('armed', "目標値がどれか1つの自由度でも変わった時点から記録します")

    def poll_record(self):
        session = self.node.session
        if session is not None:     # 待機中（ARMED）は arm_record() で表示済み
            if session.state == RecordSession.RECORDING:
                elapsed = time.monotonic() - session.t0
                if elapsed > session.duration:     # データが届かなくなっても時間で終える
                    session.finish()
                self.show_record_state('recording', f"{min(elapsed, session.duration):.1f} / {session.duration:g} s"
                                                    f"   受信 {session.rows} 件   → {session.folder or ''}")
            elif session.state == RecordSession.FINISHING:
                self.show_record_state('saving', "記録データを書き込んでいます…")
            elif session.state == RecordSession.CANCELLED:
                self.node.session = None
                self.show_record_state('idle', "待機を取り消しました")
            elif session.state == RecordSession.DONE:
                self.node.session = None
                if session.error:
                    print(session.error, file=sys.stderr)
                    self.show_record_state('error', "記録データを書き込めませんでした（詳細は端末に表示）")
                else:
                    self.start_postprocess(session.folder)

        if self.post is not None:
            self.poll_postprocess()
        self.root.after(100, self.poll_record)

    def start_postprocess(self, folder):
        """CSVへのまとめとグラフ作成を別プロセスで行う（GUIと受信を止めないため）"""
        ctx = multiprocessing.get_context('spawn')
        progress = ctx.Queue()
        proc = ctx.Process(target=postprocess_record, args=(folder, progress), name="record_postprocess")
        proc.start()
        self.post = {'proc': proc, 'queue': progress, 'folder': folder, 'dead_polls': 0}
        self.show_record_state('saving', f"CSVを作成しています…   {folder}")

    def poll_postprocess(self):
        post = self.post
        while True:
            try:
                msg = post['queue'].get_nowait()
            except queue.Empty:
                break
            if msg[0] == 'csv':
                self.show_record_state('saving', f"グラフを作成しています…   {post['folder']}")
            elif msg[0] == 'graph':
                self.show_record_state('saving', f"グラフを作成しています… {msg[1]}/{msg[2]}   {post['folder']}")
            elif msg[0] == 'done':
                self.post = None
                post['proc'].join(timeout=1)
                self.show_record_state('done', f"{post['folder']}")
                return
            elif msg[0] == 'error':
                self.post = None
                print(msg[1], file=sys.stderr)
                self.show_record_state('error', f"CSV・グラフを作成できませんでした（詳細は端末に表示）  {post['folder']}")
                return
        if not post['proc'].is_alive():
            post['dead_polls'] += 1
            if post['dead_polls'] >= 5:
                self.post = None
                self.show_record_state('error', f"後処理が異常終了しました（exitcode={post['proc'].exitcode}）"
                                                f"  {post['folder']}")

    def wait_postprocess(self):
        """終了時: 後処理が終わるまで待つ（ウィンドウを閉じた後に呼ぶ）"""
        post = self.post
        if post is None:
            return
        print(f"記録のCSV・グラフを作成しています。終わるまでお待ちください: {post['folder']}", file=sys.stderr)
        while True:
            try:
                msg = post['queue'].get(timeout=0.5)
            except queue.Empty:
                if not post['proc'].is_alive():
                    break
                continue
            if msg[0] in ('done', 'error'):
                print("完了しました" if msg[0] == 'done' else msg[1], file=sys.stderr)
                break
        post['proc'].join()
        self.post = None

    def save_prefs_later(self, delay_ms=1000):
        """グラフ表示DOF・記録の設定を設定ファイルへ保存する（続けて変更されたらまとめて1回）"""
        if self.prefs_job is not None:
            self.root.after_cancel(self.prefs_job)
        self.prefs_job = self.root.after(delay_ms, self.save_prefs)

    def save_prefs(self):
        self.prefs_job = None
        try:
            save_settings(self.settings)
        except OSError as e:
            print(f"{SETTINGS_FILE} に保存できません: {e}", file=sys.stderr)

    def on_close(self):
        if self.pose_tab_dirty():
            ans = messagebox.askyesnocancel(
                "Unsaved changes", "初期姿勢・プリセットの変更が保存されていません。保存して終了しますか？")
            if ans is None:
                return
            if ans and not self.save_pose_tab():
                return

        session = self.node.session
        if session is not None:
            if session.state == RecordSession.RECORDING:
                if not messagebox.askokcancel("Recording", "記録中です。ここまでのデータを保存してから終了します。"):
                    return
            session.finish()
            session.thread.join(timeout=10)
            self.node.session = None
            if session.state == RecordSession.DONE and not session.error and session.folder:
                self.start_postprocess(session.folder)

        if self.prefs_job is not None:
            self.root.after_cancel(self.prefs_job)
            self.save_prefs()
        self.root.destroy()

    def run(self):
        self.root.mainloop()
        # 記録の後処理が残っていれば、終わるまで待つ
        self.wait_postprocess()

# ------------------------------
def check_ranges():
    """RANDOM_RANGE が POT_RANGE の内側か（外側だとランダム目標値がスライダ・手入力の範囲を超える）"""
    bad = [i for i, ((rmin, rmax), (pmin, pmax)) in enumerate(zip(RANDOM_RANGE, POT_RANGE))
           if rmin < pmin or rmax > pmax]
    if len(POT_RANGE) != 26 or len(RANDOM_RANGE) != 26:
        print("警告: POT_RANGE / RANDOM_RANGE は26自由度ぶん必要です", file=sys.stderr)
    if bad:
        print(f"警告: RANDOM_RANGE が POT_RANGE の外に出ている自由度があります: DOF {bad}", file=sys.stderr)


def spin_ros(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


def main():
    check_ranges()
    rclpy.init()
    settings, warning = load_settings()
    node = PotGuiNode(settings['initial_pose'])
    spin_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    spin_thread.start()
    gui = PotGuiTk(node, settings, warning)
    gui.run()
    # 受信を止めてから終了する（受信コールバックの途中でPythonが終了すると異常終了することがあるため）
    rclpy.try_shutdown()
    spin_thread.join(timeout=2)
    node.destroy_node()

if __name__ == '__main__':
    main()
