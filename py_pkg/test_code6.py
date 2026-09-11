#!/usr/bin/env python3
import json
import os
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt16MultiArray

import tkinter as tk
from tkinter import ttk, messagebox
from tkinter import font as tkfont
import sys
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

# POT ranges
POT_RANGE = [
    (185, 700), (135, 550), (130, 680), (10, 734), (66, 259), (192, 389),
    (70, 600), (60, 465), (115, 619), (22, 794), (239, 430), (205, 395),
    (30, 660), (30, 690), (110, 830), (3, 630), (3, 700),(9, 660),
    (275, 360), (115, 785), (192, 440), (284, 557),
    (323, 580), (188, 630), (375, 500), (300, 490),
]

# Initial desired values
INITIAL_DESIRED = [
    500, 200, 500, 300, 170, 300,
    160, 410, 200, 500, 350, 220,
    300, 250, 400, 350, 420, 400,
    325, 370, 280, 420,
    360, 390, 420, 390
]

# Random target ranges
RANDOM_RANGE = [
    (450, 700), (135, 550), (500, 680), (250, 700), (66, 259), (192, 389),
    (70, 200), (60, 465), (115, 200), (100, 550), (239, 430), (205, 395),
    (30, 660), (30, 690), (110, 830), (3, 630), (3, 700), (9, 660),
    (275, 360), (115, 785), (192, 440), (284, 557),
    (323, 580), (188, 630), (375, 500), (300, 490),
]

# ※既存挙動の維持:
#   元のコードは `RANDOM_RANGE = POT_RANGE = [...]` と書かれており、上で定義した POT_RANGE が
#   RANDOM_RANGE の値で上書きされていた（スライダ範囲・クランプにも RANDOM_RANGE が使われていた）。
#   挙動を変えないため明示的に代入している。上の POT_RANGE を使いたい場合は次の1行を消すこと。
POT_RANGE = RANDOM_RANGE

RANDOM_ENABLE = [True]*26

# ------------------------------
# ADRC パラメータ設定
# ------------------------------
# 自由度(DOF) -> (board名, board内のローカルindex)
# .ino 側の POT_desired[i] = sub[i + offset] に対応する
BOARD_LAYOUT = [
    # (board名, パラメータ用トピック, 先頭DOF, DOF数)
    ('board1', '/board1_ADRCparam_float/sub',  0, 6),
    ('board2', '/board2_ADRCparam_float/sub',  6, 6),
    ('board3', '/board3_ADRCparam_float/sub', 12, 6),
    ('board4', '/board4_ADRCparam_float/sub', 18, 4),
    ('board5', '/board5_ADRCparam_float/sub', 22, 4),
]

# .ino 1台あたりのパラメータ配列長（kp6 + kd6 + input_coef6 + lamda_0 6）
PARAM_MSG_LEN = 24

# 「変更しない」を意味する値（.ino 側は負の値を無視する）
PARAM_KEEP = -1.0

PARAM_KEYS = ['kp', 'kd', 'input_coef', 'lamda_0']
PARAM_LABELS = {
    'kp': 'kp',
    'kd': 'kd',
    'input_coef': 'input_coef',
    'lamda_0': 'lamda_0',
}

# 各 .ino の初期値（GUI起動時の表示用。ADRC_ROS2_tk_FFoptimize-board1~5 の設定値）
DEFAULT_PARAMS = {
    'kp': [
        2200.0, 12000.0, 8000.0, 9000.0, 6000.0, 6000.0,      # board1 (DOF0-5)
        2000.0, 14000.0, 9000.0, 9000.0, 6000.0, 6000.0,      # board2 (DOF6-11)
        10000.0, 10000.0, 8000.0, 10000.0, 8000.0, 8500.0,    # board3 (DOF12-17)
        7000.0, 6000.0, 6000.0, 0.0,                          # board4 (DOF18-21)
        4000.0, 19000.0, 8000.0, 4000.0,                      # board5 (DOF22-25)
    ],
    'kd': [
        115.0, 650.0, 225.0, 500.0, 420.0, 520.0,
        100.0, 600.0, 300.0, 500.0, 300.0, 450.0,
        230.0, 230.0, 400.0, 230.0, 250.0, 230.0,
        300.0, 280.0, 230.0, 0.0,
        150.0, 240.0, 200.0, 50.0,
    ],
    'input_coef': [
        40000.0, 20000.0, 30000.0, 200000.0, 30000.0, 30000.0,
        40000.0, 20000.0, 30000.0, 200000.0, 30000.0, 30000.0,
        40000.0, 54000.0, 30000.0, 40000.0, 40000.0, 50000.0,
        30000.0, 50000.0, 20000.0, 0.0,
        60000.0, 30000.0, 60000.0, 60000.0,
    ],
    'lamda_0': [
        800.0, 300.0, 300.0, 700.0, 300.0, 300.0,
        800.0, 300.0, 300.0, 700.0, 300.0, 300.0,
        300.0, 300.0, 300.0, 300.0, 300.0, 500.0,
        300.0, 500.0, 500.0, 0.0,
        800.0, 300.0, 800.0, 800.0,
    ],
}

# パラメータの保存先
PARAM_FILE = os.path.expanduser('~/.armrobot_adrc_params.json')

# パラメータ自動再送の周期[ms]（Teensy再起動時に自動で設定値を上書きし直すため）
PARAM_RESEND_PERIOD_MS = 3000


# ------------------------------
# ROS2 Node
# ------------------------------
class PotGuiNode(Node):
    def __init__(self):
        super().__init__('pot_gui_node')
        self.publisher = self.create_publisher(Float32MultiArray, '/board_android_float/sub', 10)

        self.desired = [float(v) for v in INITIAL_DESIRED]
        self.real_raw = [0.0]*26

        # 他プログラムが送った目標値をボードのpublishから読み戻すための配列
        self.board_desired = [None]*26

        # GUIが最後にpublishした時刻（読み戻しとの競合を避けるため）
        self.last_publish_time = 0.0

        # ADRCパラメータ用パブリッシャ（ボードごと）
        self.param_publishers = {}
        for board, topic, _offset, _count in BOARD_LAYOUT:
            self.param_publishers[board] = self.create_publisher(Float32MultiArray, topic, 10)

        self.POT_COUNT = {
            'board1': 6,
            'board2': 6,
            'board3': 6,
            'board4': 4,
            'board5': 4,
        }

        self.create_subscription(UInt16MultiArray, '/board1_tk/pub', lambda msg: self.board_cb(msg, 0, 'board1'), 10)
        self.create_subscription(UInt16MultiArray, '/board2_tk/pub', lambda msg: self.board_cb(msg, 6, 'board2'), 10)
        self.create_subscription(UInt16MultiArray, '/board3_tk/pub', lambda msg: self.board_cb(msg, 12, 'board3'), 10)
        self.create_subscription(UInt16MultiArray, '/board4_tk/pub', lambda msg: self.board_cb(msg, 18, 'board4'), 10)
        self.create_subscription(UInt16MultiArray, '/board5_tk/pub', lambda msg: self.board_cb(msg, 22, 'board5'), 10)

        # 他のプログラムが目標値トピックへ publish した場合も追従する
        self.create_subscription(Float32MultiArray, '/board_android_float/sub', self.desired_cb, 10)

    def board_cb(self, msg, offset, board_name):
        num_pot = self.POT_COUNT[board_name]
        for i in range(num_pot):
            idx = offset + i
            if idx < 26 and i < len(msg.data):
                self.real_raw[idx] = float(msg.data[i])

            # .ino は pub[6+i] に POT_desired[i] を格納している（実際にボードが使っている目標値）
            if idx < 26 and (6 + i) < len(msg.data):
                self.board_desired[idx] = float(msg.data[6 + i])

    def desired_cb(self, msg):
        # 自分自身がpublishした直後は無視（往復遅延で値が揺れるのを防ぐ）
        if time.time() - self.last_publish_time < 0.5:
            return
        for i in range(min(26, len(msg.data))):
            self.desired[i] = float(msg.data[i])

    def publish(self):
        msg = Float32MultiArray()
        # ROS2 は各要素が Python の float 型であることを要求する（int は不可）
        msg.data = [float(v) for v in self.desired]
        self.last_publish_time = time.time()
        self.publisher.publish(msg)

    def publish_initial(self):
        self.desired = [float(v) for v in INITIAL_DESIRED]
        self.publish()

    def publish_params(self, params, boards=None):
        """ADRCパラメータをボードごとのトピックへ送信する。

        params: {'kp': [26個], 'kd': [...], 'input_coef': [...], 'lamda_0': [...]}
                値が None の要素は PARAM_KEEP（= .ino の設定値を維持）として送る
        boards: 送信するボード名のリスト。None なら全ボード
        """
        for board, _topic, offset, count in BOARD_LAYOUT:
            if boards is not None and board not in boards:
                continue

            data = [PARAM_KEEP] * PARAM_MSG_LEN
            for local in range(count):
                dof = offset + local
                for k, key in enumerate(PARAM_KEYS):
                    val = params[key][dof]
                    if val is None:
                        continue
                    data[k * 6 + local] = float(val)

            msg = Float32MultiArray()
            msg.data = data
            self.param_publishers[board].publish(msg)


# ------------------------------
# Tkinter GUI
# ------------------------------
class PotGuiTk:
    def __init__(self, node: PotGuiNode):
        self.node = node
        self.current_dof = 0

        # プログラム側からスライダを動かしたときに on_slider が publish しないためのガード。
        # tk.Scale の command はアイドル時（= set() から戻った後）に呼ばれるので、
        # 時間ではなく「ユーザがスライダを操作したか」でガードを解除する。
        # （このガードが無いと、GUI起動時や外部目標値の反映時に勝手に目標値が送信されてしまう）
        self.slider_programmatic = True

        # スライダをドラッグ中かどうか
        self.slider_active = False

        # ユーザが手入力して未送信の Step Input 欄（自動更新で消さないため）
        self.step_entry_dirty = [False]*26

        self.root = tk.Tk()

        # ウィジェットを作る前にフォントを決める（日本語の豆腐化対策）
        setup_japanese_font(self.root)

        self.root.title("POT GUI (time plot)")
        self.root.geometry("1200x800")

        # --------------------------
        # 画面上部のタブ
        # --------------------------
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        self.tab_pot = tk.Frame(self.notebook)
        self.tab_param = tk.Frame(self.notebook)
        self.notebook.add(self.tab_pot, text="目標値 (POT)")
        self.notebook.add(self.tab_param, text="ADRCパラメータ")

        self.build_pot_tab(self.tab_pot)
        self.build_param_tab(self.tab_param)

        # --------------------------
        # Plot buffer
        # --------------------------
        self.buffer_len = 100
        self.time_buffer = [0]*self.buffer_len
        self.real_buffer = [0]*self.buffer_len
        self.desired_buffer = [0]*self.buffer_len
        self.start_time = time.time()

        self.update_ui()
        self.update_plot()
        self.sync_from_boards()

    # =========================================================
    # 目標値タブ
    # =========================================================
    def build_pot_tab(self, parent):
        # DOF selector
        self.dof_box = ttk.Combobox(parent, values=[f"DOF {i}" for i in range(26)], state="readonly")
        self.dof_box.current(0)
        self.dof_box.bind("<<ComboboxSelected>>", self.change_dof)
        self.dof_box.pack(pady=5)

        self.info_label = tk.Label(parent, text="", font=(UI_FONT, 10))
        self.info_label.pack()

        self.real_label = tk.Label(parent, text="", font=(UI_FONT, 10))
        self.real_label.pack()

        # Canvas
        self.canvas = tk.Canvas(parent, width=800, height=300, bg="#eeeeee")
        self.canvas.pack(pady=10)

        # Slider
        self.slider = tk.Scale(parent, orient=tk.HORIZONTAL, length=800, command=self.on_slider)
        # ユーザがスライダに触れたらガードを解除する（これ以降の command は本人の操作）
        self.slider.bind("<Button-1>", self.on_slider_press)
        self.slider.bind("<ButtonRelease-1>", self.on_slider_release)
        self.slider.bind("<Key>", self.on_slider_key)
        self.slider.pack()

        # Reset
        self.init_button = tk.Button(parent, text="Reset pose", command=self.send_initial)
        self.init_button.pack(pady=5)

        # --------------------------
        # ★ Step入力エリア
        # --------------------------
        tk.Label(parent, text="Step Input (All DOF)", font=(UI_FONT, 10, "bold")).pack()

        self.step_entries = []
        frame = tk.Frame(parent)
        frame.pack(pady=5)

        for i in range(26):
            sub = tk.Frame(frame)
            sub.grid(row=i//13, column=i%13, padx=3, pady=3)

            tk.Label(sub, text=f"{i}").pack()
            entry = tk.Entry(sub, width=5)
            entry.insert(0, str(int(self.node.desired[i])))
            entry.pack()

            # 手入力した欄は「未送信」として印を付け、自動更新で上書きしないようにする
            entry.bind("<Key>", lambda e, idx=i: self.mark_step_dirty(idx))

            self.step_entries.append(entry)

        self.step_button = tk.Button(parent, text="Send Step Input", command=self.send_step)
        self.step_button.pack(pady=5)

        rand_frame=tk.LabelFrame(parent,text="Random DOF")
        rand_frame.pack(pady=5)
        self.random_enable_vars=[]
        for i in range(26):
            v=tk.BooleanVar(value=RANDOM_ENABLE[i])
            tk.Checkbutton(rand_frame,text=str(i),variable=v).grid(row=i//13,column=i%13,sticky="w")
            self.random_enable_vars.append(v)
        self.random_button=tk.Button(parent,text="Send Random Target",command=self.send_random)
        self.random_button.pack(pady=5)

    # =========================================================
    # ADRCパラメータタブ
    # =========================================================
    def build_param_tab(self, parent):
        # --- 操作ボタン ---
        ctrl = tk.Frame(parent)
        ctrl.pack(fill=tk.X, pady=5)

        tk.Button(ctrl, text="全DOF送信", command=self.send_params_all).pack(side=tk.LEFT, padx=4)
        tk.Button(ctrl, text="ファイルへ保存", command=self.save_params).pack(side=tk.LEFT, padx=4)
        tk.Button(ctrl, text="ファイルから読込", command=self.load_params_button).pack(side=tk.LEFT, padx=4)
        tk.Button(ctrl, text=".inoの初期値に戻す", command=self.reset_params_to_default).pack(side=tk.LEFT, padx=4)

        self.param_autoresend_var = tk.BooleanVar(value=True)
        tk.Checkbutton(ctrl, text=f"自動再送 ({PARAM_RESEND_PERIOD_MS/1000:.0f}秒ごと)",
                       variable=self.param_autoresend_var).pack(side=tk.LEFT, padx=12)

        self.param_status = tk.Label(parent, text="", font=(UI_FONT, 9), fg="#006000")
        self.param_status.pack(anchor="w", padx=6)

        tk.Label(parent,
                 text="空欄にすると、その項目は .ino 側の設定値のまま変更されません。"
                      " input_coef は 0 以下だと無視されます（ADRC計算式の分母のため）。",
                 font=(UI_FONT, 9), fg="#505050", justify="left").pack(anchor="w", padx=6)

        # --- スクロール可能なエリア ---
        outer = tk.Frame(parent)
        outer.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)

        pcanvas = tk.Canvas(outer, highlightthickness=0)
        vbar = tk.Scrollbar(outer, orient=tk.VERTICAL, command=pcanvas.yview)
        inner = tk.Frame(pcanvas)

        inner.bind("<Configure>", lambda e: pcanvas.configure(scrollregion=pcanvas.bbox("all")))
        pcanvas.create_window((0, 0), window=inner, anchor="nw")
        pcanvas.configure(yscrollcommand=vbar.set)

        pcanvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        vbar.pack(side=tk.RIGHT, fill=tk.Y)

        # マウスホイールでのスクロール
        def _on_wheel(event):
            delta = -1 if (event.num == 5 or event.delta < 0) else 1
            pcanvas.yview_scroll(-delta, "units")
        pcanvas.bind_all("<MouseWheel>", _on_wheel)
        pcanvas.bind_all("<Button-4>", _on_wheel)
        pcanvas.bind_all("<Button-5>", _on_wheel)

        # --- ボードごとのパラメータ入力欄 ---
        # self.param_entries[key][dof] -> tk.Entry
        self.param_entries = {key: [None]*26 for key in PARAM_KEYS}

        for board, _topic, offset, count in BOARD_LAYOUT:
            box = tk.LabelFrame(inner, text=f"{board}  (DOF {offset}~{offset+count-1})")
            box.pack(fill=tk.X, pady=4, padx=2)

            tk.Label(box, text="DOF", width=6, font=(UI_FONT, 9, "bold")).grid(row=0, column=0, padx=2, pady=2)
            for k, key in enumerate(PARAM_KEYS):
                tk.Label(box, text=PARAM_LABELS[key], width=12,
                         font=(UI_FONT, 9, "bold")).grid(row=0, column=k+1, padx=2, pady=2)

            for local in range(count):
                dof = offset + local
                r = local + 1
                tk.Label(box, text=str(dof), width=6).grid(row=r, column=0, padx=2, pady=1)

                for k, key in enumerate(PARAM_KEYS):
                    e = tk.Entry(box, width=12, justify="right")
                    e.insert(0, self.format_param(DEFAULT_PARAMS[key][dof]))
                    e.grid(row=r, column=k+1, padx=2, pady=1)
                    self.param_entries[key][dof] = e

            tk.Button(box, text=f"{board} のみ送信",
                      command=lambda b=board: self.send_params_board(b)).grid(
                          row=0, column=len(PARAM_KEYS)+1, rowspan=1, padx=8)

        # 保存済みパラメータがあれば読み込む
        self.load_params(quiet=True)

        # 自動再送を開始
        self.auto_resend_params()

    # -------------------------
    @staticmethod
    def format_param(v):
        if v is None:
            return ""
        if float(v) == int(float(v)):
            return str(int(float(v)))
        return str(float(v))

    def collect_params(self):
        """入力欄から {key: [26個]} を作る。空欄・不正値は None（= 変更しない）"""
        params = {key: [None]*26 for key in PARAM_KEYS}
        bad = []
        for key in PARAM_KEYS:
            for dof in range(26):
                e = self.param_entries[key][dof]
                if e is None:
                    continue
                text = e.get().strip()
                if text == "":
                    continue
                try:
                    params[key][dof] = float(text)
                except ValueError:
                    bad.append(f"{PARAM_LABELS[key]}[{dof}]='{text}'")
        return params, bad

    def send_params_all(self):
        self.send_params_board(None)

    def send_params_board(self, board):
        params, bad = self.collect_params()
        if bad:
            messagebox.showerror("入力エラー", "数値として読めない項目があります:\n" + "\n".join(bad))
            return

        boards = None if board is None else [board]
        self.node.publish_params(params, boards)

        target = "全ボード" if board is None else board
        self.param_status.config(
            text=f"[{time.strftime('%H:%M:%S')}] {target} へパラメータを送信しました", fg="#006000")

    def auto_resend_params(self):
        """Teensyを再起動しても設定値が.inoの初期値に戻らないよう、定期的に送り直す"""
        try:
            if self.param_autoresend_var.get():
                params, bad = self.collect_params()
                if not bad:
                    self.node.publish_params(params)
        except Exception:
            pass
        self.root.after(PARAM_RESEND_PERIOD_MS, self.auto_resend_params)

    def save_params(self):
        params, bad = self.collect_params()
        if bad:
            messagebox.showerror("入力エラー", "数値として読めない項目があります:\n" + "\n".join(bad))
            return
        try:
            with open(PARAM_FILE, 'w') as f:
                json.dump(params, f, indent=2)
        except OSError as e:
            messagebox.showerror("保存エラー", f"{PARAM_FILE} に保存できません:\n{e}")
            return
        self.param_status.config(text=f"[{time.strftime('%H:%M:%S')}] {PARAM_FILE} に保存しました", fg="#006000")

    def load_params_button(self):
        self.load_params(quiet=False)

    def load_params(self, quiet=False):
        if not os.path.exists(PARAM_FILE):
            if not quiet:
                messagebox.showinfo("読込", f"{PARAM_FILE} が見つかりません")
            return
        try:
            with open(PARAM_FILE) as f:
                params = json.load(f)
        except (OSError, ValueError) as e:
            if not quiet:
                messagebox.showerror("読込エラー", f"{PARAM_FILE} を読めません:\n{e}")
            return

        for key in PARAM_KEYS:
            values = params.get(key)
            if not isinstance(values, list):
                continue
            for dof in range(min(26, len(values))):
                e = self.param_entries[key][dof]
                if e is None:
                    continue
                e.delete(0, tk.END)
                e.insert(0, self.format_param(values[dof]))

        self.param_status.config(text=f"[{time.strftime('%H:%M:%S')}] {PARAM_FILE} を読み込みました", fg="#006000")

    def reset_params_to_default(self):
        for key in PARAM_KEYS:
            for dof in range(26):
                e = self.param_entries[key][dof]
                if e is None:
                    continue
                e.delete(0, tk.END)
                e.insert(0, self.format_param(DEFAULT_PARAMS[key][dof]))
        self.param_status.config(
            text=f"[{time.strftime('%H:%M:%S')}] 表示を .ino の初期値に戻しました（送信は別途ボタンで）",
            fg="#804000")

    # =========================================================
    # 目標値タブの処理
    # =========================================================
    def mark_step_dirty(self, idx):
        self.step_entry_dirty[idx] = True

    def clear_step_dirty(self):
        self.step_entry_dirty = [False]*26

    def change_dof(self, event):
        self.current_dof = self.dof_box.current()
        self.real_buffer = [0]*self.buffer_len
        self.desired_buffer = [0]*self.buffer_len
        self.time_buffer = [0]*self.buffer_len
        self.start_time = time.time()
        self.update_ui()

    def update_ui(self):
        mn, mx = POT_RANGE[self.current_dof]
        desired = int(self.node.desired[self.current_dof])

        # config()/set() はどちらも command=on_slider を呼び得るのでガードしてから触る
        self.slider_programmatic = True
        self.slider.config(from_=mn, to=mx)
        self.slider.set(desired)

        self.info_label.config(text=f"Desired: {desired} Range: [{mn},{mx}]")

        raw = self.node.real_raw[self.current_dof]
        self.real_label.config(text=f"Real (raw): {int(raw)}")

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
        # プログラムからスライダを動かした場合（初期化・外部目標値の反映）は送信しない
        if self.slider_programmatic:
            return

        val = float(value)
        self.node.desired[self.current_dof] = float(val)

        # Entryも同期
        self.step_entries[self.current_dof].delete(0, tk.END)
        self.step_entries[self.current_dof].insert(0, str(int(val)))
        self.step_entry_dirty[self.current_dof] = False

        self.node.publish()

    def send_initial(self):
        self.node.publish_initial()
        for i in range(26):
            self.step_entries[i].delete(0, tk.END)
            self.step_entries[i].insert(0, str(int(self.node.desired[i])))
        self.clear_step_dirty()
        self.update_ui()

    # -------------------------
    # ★ Step送信
    # -------------------------
    def send_step(self):
        for i in range(26):
            try:
                val = float(self.step_entries[i].get().strip())
                mn, mx = POT_RANGE[i]
                val = float(max(mn, min(mx, val)))  # 範囲制限（float で保持）
                self.node.desired[i] = val
            except (ValueError, IndexError):
                pass

        self.node.publish()
        self.clear_step_dirty()
        self.update_ui()


    def send_random(self):
        for i in range(26):
            if not self.random_enable_vars[i].get():
                continue
            mn,mx=RANDOM_RANGE[i]
            val=random.randint(int(mn),int(mx))
            self.node.desired[i]=float(val)
            self.step_entries[i].delete(0,tk.END)
            self.step_entries[i].insert(0,str(val))
        self.node.publish()
        self.clear_step_dirty()
        self.update_ui()

    # -------------------------
    # 他プログラムが変更した目標値を画面に反映する
    # -------------------------
    def sync_from_boards(self):
        # ボードが publish している「実際に使っている目標値」を GUI の内部状態に取り込む。
        # これにより、他プログラムが目標値を変えた後に GUI から送信しても
        # 古い値で上書きしてしまうことがなくなる。
        now = time.time()
        if now - self.node.last_publish_time > 0.5:
            for i in range(26):
                bd = self.node.board_desired[i]
                if bd is None:
                    continue
                if abs(bd - self.node.desired[i]) >= 1.0:
                    self.node.desired[i] = bd

        # Entry の表示を更新（手入力して未送信の欄は触らない）
        for i in range(26):
            if self.step_entry_dirty[i]:
                continue
            e = self.step_entries[i]
            want = str(int(self.node.desired[i]))
            if e.get().strip() != want:
                e.delete(0, tk.END)
                e.insert(0, want)

        # スライダの表示を更新（ドラッグ中は触らない）
        if not self.slider_active:
            # スライダは範囲外の値を保持できないので、比較もクランプ後の値で行う
            mn, mx = POT_RANGE[self.current_dof]
            cur = int(max(mn, min(mx, self.node.desired[self.current_dof])))
            if int(self.slider.get()) != cur:
                self.set_slider_silently(cur)

        self.root.after(200, self.sync_from_boards)

    # -------------------------
    def update_plot(self):
        dof = self.current_dof
        raw = self.node.real_raw[dof]
        desired = self.node.desired[dof]
        t = time.time() - self.start_time

        self.time_buffer.append(t)
        self.time_buffer = self.time_buffer[-self.buffer_len:]
        self.real_buffer.append(raw)
        self.real_buffer = self.real_buffer[-self.buffer_len:]
        self.desired_buffer.append(desired)
        self.desired_buffer = self.desired_buffer[-self.buffer_len:]

        self.canvas.delete("all")
        mn, mx = POT_RANGE[dof]
        width = int(self.canvas['width'])
        height = int(self.canvas['height'])
        margin = 50

        plot_width = width - 2*margin
        plot_height = height - 2*margin

        ## Y軸目盛り（5分割）
        y_div = 5
        for i in range(y_div + 1):
            y_val = mn + i*(mx - mn)/y_div
            y = margin + plot_height * (1 - (y_val - mn)/(mx - mn))
            self.canvas.create_line(margin, y, width - margin, y, fill="#cccccc", dash=(2,2))
            self.canvas.create_text(margin-10, y, text=str(int(y_val)), anchor="e", font=(UI_FONT, 8))

        # X軸補助線（0.5秒ごと）
        t_start = self.time_buffer[0]
        t_end = self.time_buffer[-1]
        if t_end - t_start < 0.001:
            t_end = t_start + 1.0
        time_range = t_end - t_start
        x_interval = 0.5
        x = (t_start // x_interval) * x_interval
        while x <= t_end:
            x_pos = margin + (x - t_start)/time_range * plot_width
            self.canvas.create_line(x_pos, margin, x_pos, margin + plot_height, fill="#cccccc", dash=(2,2))
            x += x_interval

        # グラフ
        if len(self.real_buffer) > 1:
            for i in range(1, len(self.real_buffer)):
                x1 = margin + (i-1)/self.buffer_len * plot_width
                x2 = margin + i/self.buffer_len * plot_width

                y1 = margin + plot_height * (1 - (self.real_buffer[i-1]-mn)/(mx-mn))
                y2 = margin + plot_height * (1 - (self.real_buffer[i]-mn)/(mx-mn))
                self.canvas.create_line(x1, y1, x2, y2, fill="red", width=2)

                y1d = margin + plot_height * (1 - (self.desired_buffer[i-1]-mn)/(mx-mn))
                y2d = margin + plot_height * (1 - (self.desired_buffer[i]-mn)/(mx-mn))
                self.canvas.create_line(x1, y1d, x2, y2d, fill="blue", width=2, dash=(4,2))

        self.real_label.config(text=f"Real: {int(raw)}  Desired: {int(desired)}")
        self.root.after(50, self.update_plot)

    def run(self):
        self.root.mainloop()

# ------------------------------
def main():
    rclpy.init()
    node = PotGuiNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    gui = PotGuiTk(node)
    gui.run()

if __name__ == '__main__':
    main()
