#!/usr/bin/env python3
"""
ADRCパラメータ調整GUI（GUI_android_tk.py を基にしたもの。目標値の送信・記録・初期姿勢/プリセットは GUI_android_tk.py と同じ）
<対応Arduinoプログラム> ADRC_ROS2_tk_FF_ADRCparamChange-board1〜5

・グラフ: 表示DOF（複数選択）ごとに次の5つを横1段に並べ（時間軸の長さをそろえる）、自由度を増やすと段を下に追加する
  （収まらなければ縦スクロール。ウィンドウが狭くグラフが GRAPH_W_MIN より細くなるときは横スクロール）
    1. 実測POT・目標値（/board{k}_tk/pub）        2. 実測POT・z1（/board{k}_tk_z1_float/pub）
    3. derivatives・z2（/board{k}_tk_derivatives_float/pub, /board{k}_tk_z2_float/pub）
    4. z3（/board{k}_tk_z3_float/pub）              5. 入力PWM（/board{k}_tk_PWM_float/pub）
  グラフをクリックすると、その自由度を「選択中の自由度」（スライダとパラメータ調整の対象。オレンジの枠）にする。
・パラメータ調整: 選択中の1自由度の kp, kd, input_coef, lambda_0 を手入力・上下ボタンで変え、「DOF n へ送信」で送る。
    現在設定値 … Teensy から受信した値（/board{k}_tk_ADRCparam_float/pub）。送信した値をこれとして扱うことはしない。
    送信予定値 … GUI上で変更し、次に送信する値。
  送信後は Teensy が返す処理結果と現在設定値で「設定完了・拒否・応答なし」を表示する。
・エラー（発散）: z1・z2・z3・PWM に NaN・inf・極端に大きな値（DIVERGE_LIMITS）が届いた自由度は、
  その自由度のグラフの見出し・パラメータ欄・一覧を「エラー」にする（他の自由度の表示・描画は続ける）。
  エラーの自由度を選んでいるときだけ「一つ前のパラメータに戻す」が押せ、最後に送信する前の現在設定値を送り直す。
・「ADRCパラメータ一覧」タブ: ボードごとに全自由度の送信予定値・現在設定値・.ino初期値。保存・読み込み・初期値への復元など。
    保存先 adrc_param_common.PARAM_FILE は GUI_android_tk.py が起動時に読み込み、送信ボタンで使う。
・「制御対象DOF」タブ: 制御する自由度を選ぶ（既定は DOF21・DOF25 以外）。制御対象の自由度だけ調整・送信できる。
・送信予定値・制御対象に未保存の変更があるまま終了しようとすると「保存して終了」「保存せずに終了」を選ぶ。
・記録: GUI_android_tk.py と同じ保存先・ファイル名・形式。CSVの右端に z1・derivatives・z2 の列
  （Time{k}_z1, z1_{d}, Time{k}_derivatives, derivatives_{d}, Time{k}_z2, z2_{d}）を追加し（従来の列の名前・位置は不変）、
  グラフに DOF/DOF<n>/time-z1.png（実測POT・z1）・time-z2.png（derivatives・z2）を追加する。
  <フォルダ名>_ADRCparam.csv に記録開始時・終了時の全自由度のADRCパラメータ（Teensy の現在設定値）を書く。
・表示DOF・記録の設定などは SETTINGS_FILE に保存する（初回は GUI_android_tk.py の初期姿勢・プリセット・記録の設定を引き継ぐ）。
"""
import math
import os
import shutil
import threading
import time
import traceback

import numpy as np
import rclpy
from std_msgs.msg import Float32MultiArray

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

try:
    from . import GUI_android_tk as base
    from . import adrc_param_common as adrc
except ImportError:     # スクリプトとして直接実行したとき
    import GUI_android_tk as base
    import adrc_param_common as adrc

# 表示DOF・記録の設定など（形式は GUI_android_tk.py の設定ファイルと同じ）
SETTINGS_FILE = os.path.expanduser('~/.armrobot_gui_adrc_param_chg.json')

# 記録する種類: GUI_android_tk.py の種類（pot, pwm, z3）の後ろに追加する
EXTRA_KINDS = ('z1', 'derivatives', 'z2')
RECORD_BLOCKS = base.record_blocks((base.RECORD_KINDS, EXTRA_KINDS))

# 時系列を保持する Float32MultiArray の種類とトピック
FLOAT_KINDS = ('pwm', 'z3') + EXTRA_KINDS
FLOAT_TOPICS = {
    'pwm': base.TOPIC_PWM, 'z3': base.TOPIC_Z3,
    'z1': adrc.TOPIC_Z1, 'derivatives': adrc.TOPIC_DERIVATIVES, 'z2': adrc.TOPIC_Z2,
}

# 1自由度あたりのグラフ: (名前, 縦軸, [(データ, 凡例, 色, 破線)])
#   縦軸: 'pot' = POT範囲（範囲外のデータがあれば広げる）, 'auto' = データに合わせる
#   データ: 'real' = 実測POT, 'desired' = 目標値（ボードが使っている値）, それ以外は FLOAT_KINDS
GRAPHS = [
    ("POT・目標値", 'pot', [('real', "POT", "red", None), ('desired', "目標値", "blue", (4, 2))]),
    ("POT・z1", 'pot', [('real', "POT", "red", None), ('z1', "z1", "#00a000", None)]),
    ("derivatives・z2", 'auto', [('derivatives', "derivatives", "#e07000", None), ('z2', "z2", "purple", None)]),
    ("z3", 'auto', [('z3', "z3", "#00a000", None)]),
    ("入力PWM", 'auto', [('pwm', "PWM", "purple", None)]),
]
GRAPH_W_MIN = 220       # グラフ1つの最小の幅 [px]（5つ並べてこれより細くなるときは横スクロール）
GRAPH_H_MIN = 120       # グラフ1つの最小の高さ [px]
GRAPH_ASPECT = 1.8      # グラフの 横:縦（時系列が見やすい横長。表示領域の高さを超えない範囲で）
GRAPH_GAP = 2           # 横に並べたグラフどうしの間隔 [px]
BLOCK_BORDER = 3        # 自由度ごとの枠の太さ [px]（選択中の自由度はオレンジ）
VALUE_CLAMP = 1e30      # 縦軸の計算で扱う値の上限（発散した値でも計算が破綻しないように、これより外は切り詰める）

# 発散の検出: 直近 DIVERGE_CHECK_SEC 秒に受信した値に NaN・inf、または絶対値が DIVERGE_LIMITS を超える値があれば、
# その自由度を「エラー」にする（POT は 0〜1023、z3 は通常 1e8 程度までなので、十分大きくとっている）
DIVERGE_LIMITS = {'z1': 1e5, 'z2': 1e7, 'z3': 1e12, 'pwm': 1e6}
DIVERGE_CHECK_SEC = 0.5
ERROR_HOLD_SEC = 2.0        # 発散が見えなくなってからも、この時間はエラーの表示を続ける
PARAM_HISTORY_MAX = 20      # 「一つ前のパラメータに戻す」ために自由度ごとに覚えておく、送信前の現在設定値の数

# パラメータの上下ボタンの刻み（選択肢と初期値）
PARAM_STEPS = {
    'kp': ('1', '10', '100', '1000'),
    'kd': ('1', '5', '10', '50', '100'),
    'input_coef': ('100', '1000', '5000', '10000'),
    'lambda_0': ('1', '5', '10', '50', '100'),
}
PARAM_STEP_DEFAULT = {'kp': '100', 'kd': '10', 'input_coef': '1000', 'lambda_0': '10'}


# ------------------------------
# 記録の後処理（別プロセスで動かす）
# ------------------------------
def extra_record_graphs(k, board, d):
    """GUI_android_tk.py のグラフに追加するグラフ（系列の5番目の要素はその系列の時刻列）"""
    return [
        ('time-z1', f"Time{k}",
         [(f"POT{d}", "POT (measured)", "red", "-"), (f"z1_{d}", "z1", "green", "-", f"Time{k}_z1")],
         "POT", f"{base.TOPIC_POT.format(board)}, {adrc.TOPIC_Z1.format(board)}"),
        ('time-z2', f"Time{k}_derivatives",
         [(f"derivatives_{d}", "derivatives", "orange", "-"), (f"z2_{d}", "z2", "purple", "-", f"Time{k}_z2")],
         "angular velocity [POT/s]", f"{adrc.TOPIC_DERIVATIVES.format(board)}, {adrc.TOPIC_Z2.format(board)}"),
    ]


def postprocess_record_adrc(folder, progress):
    """一時CSVを1つのCSVにまとめ（z1・derivatives・z2 の列を右端に追加）、グラフを作る"""
    try:
        csv_path = base.combine_record_parts(folder, RECORD_BLOCKS)
        progress.put(('csv', csv_path))
        base.make_record_graphs(csv_path, folder, progress, extra_graphs=extra_record_graphs)
        shutil.rmtree(os.path.join(folder, base.RECORD_PARTS_DIR), ignore_errors=True)
        progress.put(('done', csv_path))
    except Exception:
        progress.put(('error', traceback.format_exc()))


def load_tune_settings():
    """設定を読む。初回は GUI_android_tk.py の初期姿勢・プリセット・記録の設定を引き継ぐ"""
    if os.path.exists(SETTINGS_FILE):
        return base.load_settings(SETTINGS_FILE)
    settings, warning = base.load_settings(base.SETTINGS_FILE)
    settings['graph_dofs'] = [0]
    return settings, warning


# ------------------------------
# ROS2 Node
# ------------------------------
class AdrcTuneNode(base.PotGuiNode):
    """GUI_android_tk.py のノードに、PWM・z3・z1・derivatives・z2 の時系列の保持を加えたもの"""

    def __init__(self, initial_desired):
        super().__init__(initial_desired, node_name='adrc_param_gui_node')
        capacity = base.HISTORY_SEC * base.HISTORY_MAX_RATE
        self.float_history = {}
        for board, _offset, count in base.BOARD_LAYOUT:
            for kind in FLOAT_KINDS:
                self.float_history[(kind, board)] = base.TimeRing(capacity, count)
            # PWM・z3 は基のノードが購読している（コールバックは下の record_float_cb）
            for kind in EXTRA_KINDS:
                self.create_subscription(
                    Float32MultiArray, FLOAT_TOPICS[kind].format(board),
                    lambda msg, kd=kind, b=board, c=count: self.record_float_cb(msg, kd, b, c),
                    base.SENSOR_QOS_DEPTH)

    def record_float_cb(self, msg, kind, board_name, count):
        """PWM・z3・z1・derivatives・z2 の受信（常に時系列を保持し、記録中は記録もする）"""
        t = time.monotonic()
        data = msg.data
        row = [float(data[i]) if i < len(data) else float('nan') for i in range(count)]
        self.float_history[(kind, board_name)].append(t, row)
        session = self.session
        if session is not None and session.state == base.RecordSession.RECORDING:
            session.add((kind, board_name), t, row)

    def snapshot_history(self, t_from):
        snap = super().snapshot_history(t_from)
        for key, ring in self.float_history.items():
            snap[key] = ring.window(t_from, list(range(ring.v.shape[1])))
        return snap

    def float_window(self, kind, dof, t_from, snapshot=None):
        """DOF の kind の時刻 t_from 以降の (時刻, 値)"""
        board, local, _count = self.dof_slot[dof]
        if snapshot is None:
            t, v = self.float_history[(kind, board)].window(t_from, [local])
        else:
            t_all, v_all = snapshot[(kind, board)]
            k = int(np.searchsorted(t_all, t_from, side='left'))
            t, v = t_all[k:], v_all[k:][:, [local]]
        return t, v[:, 0]


# ------------------------------
# グラフ
# ------------------------------
def clamp_value(v):
    return min(max(float(v), -VALUE_CLAMP), VALUE_CLAMP)


def nice_limits(vmin, vmax):
    """データの範囲を少し広げ、きりのよい値に丸めた縦軸の範囲（float。±VALUE_CLAMP に切り詰める）

    ※ 整数のまま計算すると、発散して 9.2e18 を超えた値で int64 に入らない巨大な整数になり、
      numpy の配列が object 型になって描画が止まる（np.isfinite の TypeError）。必ず float で計算する。
    """
    vmin, vmax = clamp_value(vmin), clamp_value(vmax)
    span = vmax - vmin
    if span <= 0:
        pad = max(1.0, abs(vmin) * 0.1)
        vmin, vmax = vmin - pad, vmax + pad
        span = vmax - vmin
    vmin -= span * 0.05
    vmax += span * 0.05
    raw = (vmax - vmin) / 4
    mag = 10.0 ** math.floor(math.log10(raw))
    step = next(m * mag for m in (1, 2, 2.5, 5, 10) if m * mag >= raw)
    return float(math.floor(vmin / step) * step), float(math.ceil(vmax / step) * step)


def fmt_tick(v):
    if v == 0:
        return "0"
    if abs(v) >= 1e6:
        return f"{v:.3g}"
    if abs(v) >= 100:
        return f"{v:.0f}"
    return f"{v:.4g}"


def fmt_latest(data, integer=False):
    """見出しに出す最新値（未受信・NaN は "-"、inf はそのまま、発散した大きな値は指数表記）"""
    v = float(data[-1]) if len(data) else float('nan')
    if v != v:
        return "-"
    if not math.isfinite(v):
        return "inf" if v > 0 else "-inf"
    if integer and abs(v) < 1e6:
        return str(int(round(v)))
    return f"{v:.4g}"


class SeriesPlot(base.TimePlot):
    """複数の系列（系列ごとに時刻が違ってよい）を描く時系列グラフ。目盛り・補助線の描き方は TimePlot と同じ"""
    MARGINS = (58, 10, 32, 20)     # 上の余白は2行（1行目: 見出し, 2行目: 凡例）

    def __init__(self, canvas, dof, spec):
        self.canvas = canvas
        self.dof = dof
        self.name, self.ymode, self.series = spec
        self.axes_key = None
        self.ylim = None
        self.lines = []
        for _, _, color, dash in self.series:
            opts = {'dash': dash} if dash else {}
            self.lines.append(canvas.create_line(0, 0, 0, 0, fill=color, width=2, state="hidden", **opts))
        self.title = canvas.create_text(self.MARGINS[0], 2, anchor="nw", text="", font=(base.UI_FONT, 9, "bold"))
        self.title_text = None

    def draw_axes(self, w, h, lo, hi, window):
        c = self.canvas
        c.delete("axes")
        x0, y0, plot_width, plot_height = self.plot_area(w, h)

        y_div = max(2, min(5, plot_height // 25))
        for i in range(y_div + 1):
            y_val = lo + i*(hi - lo)/y_div
            y = y0 + plot_height * (1 - (y_val - lo)/(hi - lo))
            c.create_line(x0, y, x0 + plot_width, y, fill="#cccccc", dash=(2, 2), tags="axes")
            c.create_text(x0 - 4, y, text=fmt_tick(y_val), anchor="e", font=(base.UI_FONT, 8), tags="axes")
        if self.ymode == 'auto' and lo < 0 < hi:
            y = y0 + plot_height * (1 - (0 - lo)/(hi - lo))
            c.create_line(x0, y, x0 + plot_width, y, fill="#909090", tags="axes")

        max_lines = max(2, min(12, plot_width // 45))
        step = next((s for s in (0.5, 1, 2, 5, 10, 20, 30, 60) if window / s <= max_lines), 120)
        k = 0
        while k*step <= window + 1e-9:
            x = x0 + plot_width * (1 - k*step/window)
            c.create_line(x, y0, x, y0 + plot_height, fill="#cccccc", dash=(2, 2), tags="axes")
            c.create_text(x, y0 + plot_height + 10, text=(f"-{k*step:g}" if k else "0 s"),
                          font=(base.UI_FONT, 8), tags="axes")
            k += 1

        # 凡例（見出しの下の行、右寄せ）
        x = x0 + plot_width
        for _, label, color, _ in reversed(self.series):
            item = c.create_text(x, 17, anchor="ne", text=f"━ {label}", fill=color,
                                 font=(base.UI_FONT, 8, "bold"), tags="axes")
            bbox = c.bbox(item)
            x = (bbox[0] if bbox else x - 80) - 8

        c.tag_lower("axes")

    @staticmethod
    def y_limits(pot_range, *series):
        """縦軸はPOT範囲。範囲外のデータがあるときだけ 50 刻みで広げる（TimePlot.y_limits と同じ。float で計算する）"""
        lo, hi = float(pot_range[0]), float(pot_range[1])
        for s in series:
            s = s[np.isfinite(s)]
            if s.size:
                smin, smax = clamp_value(s.min()), clamp_value(s.max())
                if smin < lo:
                    lo = float(math.floor(smin / 50) * 50)
                if smax > hi:
                    hi = float(math.ceil(smax / 50) * 50)
        return lo, hi

    def auto_limits(self, series):
        vals = [s[np.isfinite(s)] for s in series]
        vals = [v for v in vals if v.size]
        if not vals:
            return self.ylim or (-1.0, 1.0)
        vmin = float(min(v.min() for v in vals))
        vmax = float(max(v.max() for v in vals))
        if self.ylim is not None:
            lo, hi = self.ylim
            # データが範囲内で、範囲の3割以上を使っているうちは変えない（縦軸がちらつかないように）
            if lo <= vmin and vmax <= hi and (vmax - vmin) >= 0.3 * (hi - lo):
                return self.ylim
        self.ylim = nice_limits(vmin, vmax)
        return self.ylim

    def draw(self, data, now, window, pot_range, title):
        """data: 系列ごとの (時刻, 値)"""
        w, h = self.size()
        values = [v for _, v in data]
        if self.ymode == 'pot':
            lo, hi = self.y_limits(pot_range, *values)
        else:
            lo, hi = self.auto_limits(values)
        key = (w, h, lo, hi, window)
        if key != self.axes_key:
            self.draw_axes(w, h, lo, hi, window)
            self.axes_key = key

        if title != self.title_text:
            self.canvas.itemconfigure(self.title, text=title)
            self.title_text = title

        x0, y0, plot_width, plot_height = self.plot_area(w, h)
        for item, (t, v) in zip(self.lines, data):
            x = x0 + plot_width * (1 - (now - t) / window)
            y = y0 + plot_height * (1 - (np.asarray(v, dtype=float) - lo)/(hi - lo))
            # VALUE_CLAMP より外の値で、描画の座標が極端な値にならないようにする（NaN はそのまま＝描かない）
            y = np.clip(y, y0 - 2 * plot_height, y0 + 3 * plot_height)
            self.set_line(item, x, y, plot_width)


class DofBlock:
    """1自由度ぶんのグラフ（見出しの下に GRAPHS の5つを横1段に並べる）"""

    def __init__(self, gui, parent, dof):
        self.dof = dof
        board, local = adrc.dof_slot(dof)
        self.label = f"DOF {dof}  ({board} #{local})"
        self.frame = tk.Frame(parent, highlightthickness=BLOCK_BORDER, highlightbackground="#c8c8c8", bd=0)
        self.header = tk.Label(self.frame, text=self.label, anchor="w", font=(base.UI_FONT, 10, "bold"))
        self.header.grid(row=0, column=0, columnspan=len(GRAPHS), sticky="w", padx=4)
        self.plots = []
        for col, spec in enumerate(GRAPHS):
            canvas = tk.Canvas(self.frame, width=GRAPH_W_MIN, height=GRAPH_H_MIN, bg="#eeeeee",
                               highlightthickness=0)
            canvas.grid(row=1, column=col, padx=(0 if col == 0 else GRAPH_GAP, 0), pady=(0, 1))
            self.plots.append(SeriesPlot(canvas, dof, spec))
        # クリックでこの自由度を選択中にする
        for w in [self.frame, self.header] + [p.canvas for p in self.plots]:
            w.bind("<Button-1>", lambda e, d=dof: gui.select_dof(d))

    def update_header(self, selected, error=None):
        """選択中はオレンジの枠。error（発散・描画できない理由）があれば見出しを赤くして表示する"""
        color = "#ff8c00" if selected else "#c8c8c8"
        self.frame.config(highlightbackground=color, highlightcolor=color)
        text = self.label + ("    ← 選択中（スライダ・パラメータ調整の対象）" if selected else "")
        if error:
            text += f"    ⚠ エラー: {error}"
            opts = {'fg': "white", 'bg': "#c00000"}
        else:
            opts = {'fg': ("#c06000" if selected else "black"), 'bg': self.frame.cget('bg')}
        if self.header.cget('text') != text or self.header.cget('fg') != opts['fg']:
            self.header.config(text=text, **opts)


# ------------------------------
# Tkinter GUI
# ------------------------------
class AdrcTuneTk(base.PotGuiTk):
    SETTINGS_PATH = SETTINGS_FILE
    RECORD_BLOCKS = RECORD_BLOCKS
    POSTPROCESS = staticmethod(postprocess_record_adrc)

    def __init__(self, node, settings, settings_warning=None):
        super().__init__(node, settings, settings_warning)
        self.root.title("ADRC parameter tuning")
        if self.plan_load_warning:
            self.root.after(400, lambda: messagebox.showwarning("ADRC parameters", self.plan_load_warning))

    # =========================================================
    # 送信予定値（plan）と保存値（saved）
    # =========================================================
    def init_adrc_state(self):
        self.saved, err = adrc.load_param_file()
        self.plan = adrc.copy_param_set(self.saved) if self.saved is not None else adrc.default_param_set()
        self.plan_invalid = set()           # 選択中の自由度で、送信予定値の欄が不正なパラメータ
        self.plan_programmatic = False      # 送信予定値の欄をプログラムから書き換え中
        self.plan_load_warning = None
        # 自由度ごとのエラー: 発散 {dof: (理由, 最後に発散を見た時刻)}、描画できなかった理由 {dof: 理由}
        self.diverged = {}
        self.draw_errors = {}
        # 「一つ前のパラメータに戻す」用: 自由度ごとに、送信する直前の Teensy の現在設定値（新しいものが後ろ）
        self.param_history = {dof: [] for dof in range(26)}
        if self.saved is None:
            if os.path.exists(adrc.PARAM_FILE):
                self.plan_load_warning = err + "\n\n.ino の初期値を送信予定値にしています。"
            self.list_message_initial = (f"保存ファイルを読み込めないため、.ino の初期値を送信予定値にしています（{adrc.PARAM_FILE}）"
                                         if os.path.exists(adrc.PARAM_FILE) else
                                         f"保存ファイルがまだ無いので、.ino の初期値を送信予定値にしています（{adrc.PARAM_FILE}）")
        else:
            self.list_message_initial = f"{adrc.PARAM_FILE} を読み込みました（保存 {self.saved.get('saved_at') or '-'}）"

    def plan_dirty(self):
        return not adrc.param_sets_equal(self.plan, self.saved)

    def effective_default(self, dof):
        """.ino の初期値 ({key: 値}, 出所)。Teensy が送ってきた初期値があればそれ、無ければ GUI の表"""
        d = self.node.adrc.teensy_default(dof)
        if d is not None and all(v is not None for v in d.values()):
            return d, 'Teensy'
        return {key: adrc.INO_DEFAULTS[key][dof] for key in adrc.PARAM_KEYS}, 'GUIの表'

    def dof_error(self, dof, now=None):
        """自由度のエラーの理由（発散している・描画できない）。無ければ None"""
        now = time.monotonic() if now is None else now
        item = self.diverged.get(dof)
        if item is not None and now - item[1] < ERROR_HOLD_SEC:
            return item[0]
        return self.draw_errors.get(dof)

    def check_divergence(self, now):
        """直近 DIVERGE_CHECK_SEC 秒の z1・z2・z3・PWM に NaN・inf・極端に大きい値があれば、その自由度を発散とする"""
        for dof in range(26):
            for kind, limit in DIVERGE_LIMITS.items():
                _, v = self.node.float_window(kind, dof, now - DIVERGE_CHECK_SEC)
                if not v.size:
                    continue
                bad_nan = ~np.isfinite(v)
                if bad_nan.any():
                    self.diverged[dof] = (f"{kind} が発散（NaN/inf）", now)
                    break
                peak = float(np.max(np.abs(v)))
                if peak > limit:
                    self.diverged[dof] = (f"{kind} が発散（|{kind}| = {peak:.3g}）", now)
                    break

    def plan_dof_state(self, dof, now=None):
        error = self.dof_error(dof, now)
        if error:
            return 'error', f"エラー（{error}）"
        if self.plan['targets'][dof] and any(adrc.check_value(k, self.plan['values'][k][dof]) for k in adrc.PARAM_KEYS):
            return 'invalid', "送信予定値が範囲外"
        return adrc.dof_state(self.node.adrc, dof, adrc.dof_values(self.plan, dof), self.plan['targets'][dof], now)

    def replace_plan(self, ps, message):
        """送信予定値・制御対象をまとめて置き換える（読み込み・初期値への復元など）"""
        self.plan = adrc.copy_param_set(ps)
        self.load_param_panel()
        self.sync_target_vars()
        self.set_list_message(message)

    # =========================================================
    # グラフ（表示DOFごとに5つ）
    # =========================================================
    def build_graph_area(self, parent):
        bar = tk.Frame(parent)
        bar.pack(fill=tk.X, padx=6, pady=(4, 0))

        tk.Label(bar, text="グラフ表示DOF", font=(base.UI_FONT, 9, "bold")).grid(row=0, column=0, padx=(0, 6), sticky="w")
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
        self.window_var = tk.StringVar(value=str(base.PLOT_WINDOW_SEC))
        tk.Spinbox(opt, from_=1, to=base.HISTORY_SEC, increment=1, width=5,
                   textvariable=self.window_var).grid(row=0, column=1, sticky="w")
        self.pause_button = tk.Button(opt, text="‖ グラフ停止", width=12, command=self.toggle_graph_pause)
        self.pause_button.grid(row=0, column=2, padx=(12, 0), sticky="w")
        self.pause_label = tk.Label(opt, text="", fg="#c00000", font=(base.UI_FONT, 9, "bold"))
        self.pause_label.grid(row=1, column=2, padx=(12, 0), sticky="w")
        tk.Label(opt, text="グラフをクリックすると、その自由度を選択します", font=(base.UI_FONT, 8),
                 fg="#606060").grid(row=1, column=0, columnspan=2, sticky="w")

        # グラフが GRAPH_W_MIN より細くなる狭いウィンドウでは横スクロールにする
        self.graph_canvas, self.graph_inner = self.make_scrollable(parent, fit_width=True, keep_min_width=True)
        self.graph_canvas.bind("<Configure>", lambda e: self.schedule_layout(), add="+")
        self.graph_empty = tk.Label(self.graph_inner, text="グラフを表示するDOFを上で選択してください",
                                    fg="#808080")

    def on_graph_dof_toggle(self):
        selected = [i for i, v in enumerate(self.graph_dof_vars) if v.get()]
        for dof in list(self.plots):
            if dof not in selected:
                self.plots.pop(dof).frame.destroy()
        for dof in selected:
            if dof not in self.plots:
                self.plots[dof] = DofBlock(self, self.graph_inner, dof)
        self.highlight_slider_plot()
        self.schedule_layout()

        if selected != self.settings['graph_dofs']:
            self.settings['graph_dofs'] = selected
            self.save_prefs_later()

    def highlight_slider_plot(self):
        now = time.monotonic()
        for dof, block in self.plots.items():
            block.update_header(dof == self.current_dof, self.dof_error(dof, now))

    def layout_plots(self):
        """自由度ごとに5つのグラフを横1段に並べ、自由度の段を縦に積む

        幅は表示領域の幅を5等分（GRAPH_W_MIN 未満にはしない＝横スクロール）。
        高さは横:縦 = GRAPH_ASPECT にし、1自由度の段が表示領域の高さに収まるよう抑える（GRAPH_H_MIN 未満にはしない）。
        """
        self.layout_job = None
        dofs = sorted(self.plots)
        if not dofs:
            self.graph_empty.grid(row=0, column=0, padx=20, pady=20)
            return
        self.graph_empty.grid_remove()

        width, height = self.graph_canvas.winfo_width(), self.graph_canvas.winfo_height()
        if width < 50 or height < 20:   # まだ表示されていない・境界で畳まれている
            return
        n = len(GRAPHS)
        header_h = self.plots[dofs[0]].header.winfo_reqheight()
        inset = base.PLOT_PAD + BLOCK_BORDER
        graph_w = max(GRAPH_W_MIN, (width - 2*inset - (n - 1)*GRAPH_GAP) // n)
        fit_h = height - header_h - 2*inset - 1         # 1自由度の段が表示領域に収まる高さ
        graph_h = int(max(GRAPH_H_MIN, min(graph_w / GRAPH_ASPECT, fit_h)))
        for row, dof in enumerate(dofs):
            block = self.plots[dof]
            for plot in block.plots:
                plot.canvas.config(width=graph_w, height=graph_h)
            block.frame.grid(row=row, column=0, padx=base.PLOT_PAD, pady=base.PLOT_PAD, sticky="w")

    def scroll_to_dof(self, dof):
        """選択した自由度のグラフが見えるようにスクロールする"""
        block = self.plots.get(dof)
        total = self.graph_inner.winfo_height()
        if block is not None and total > 1:
            self.graph_canvas.yview_moveto(max(0.0, block.frame.winfo_y() / total))

    def update_plots(self):
        start = time.perf_counter()
        try:
            self.draw_visible_plots()
        finally:
            # 描画で例外が出ても次の更新は必ず予約する（予約しないとグラフが止まったままになる）
            cost = time.perf_counter() - start
            interval = min(500, max(50, cost * 4000))
            if self.paused_at is not None:
                interval = max(interval, 250)
            self.root.after(int(interval), self.update_plots)

    def draw_visible_plots(self):
        window = self.plot_window()
        if self.paused_at is None:
            now, snapshot = time.monotonic(), None
        else:
            now, snapshot = self.paused_at, self.paused_history

        # スクロール領域のうち見えている範囲のグラフだけ描く
        top = self.graph_canvas.canvasy(0)
        bottom = top + self.graph_canvas.winfo_height()
        for dof, block in self.plots.items():
            frame = block.frame
            if not frame.winfo_ismapped():
                continue
            fy = frame.winfo_y()
            visible = [p for p in block.plots
                       if not (fy + p.canvas.winfo_y() + p.canvas.winfo_height() < top
                               or fy + p.canvas.winfo_y() > bottom)]
            if not visible:
                continue
            # 1つの自由度が描けなくても、他の自由度は描き続ける（描けない自由度はその見出しにエラーを出す）
            try:
                t, real, desired = self.node.dof_history(dof, now - window, snapshot)
                series = {'real': (t, real), 'desired': (t, desired)}
                for kind in FLOAT_KINDS:
                    series[kind] = self.node.float_window(kind, dof, now - window, snapshot)
                for plot in visible:
                    data = [series[name] for name, _, _, _ in plot.series]
                    title = "   ".join(f"{label} {fmt_latest(series[name][1], plot.ymode == 'pot')}"
                                       for name, label, _, _ in plot.series)
                    if plot is block.plots[0]:
                        title = f"DOF {dof}   " + title
                    plot.draw(data, now, window, base.POT_RANGE[dof], title)
                self.draw_errors.pop(dof, None)
            except Exception as e:
                if dof not in self.draw_errors:
                    traceback.print_exc()
                self.draw_errors[dof] = f"グラフを描けません（{type(e).__name__}）"

        dof = self.current_dof
        raw = self.node.real_raw[dof]
        desired = self.node.desired[dof]
        bd = self.node.board_desired[dof]
        board_text = "-" if bd is None else str(int(bd))
        self.real_label.config(text=f"Real: {int(raw)}  Desired: {int(desired)}  (board: {board_text})")

    # =========================================================
    # 操作部: Random Target / 目標値の入力 / ADRCパラメータの調整 / 記録
    # =========================================================
    def build_controls(self, parent):
        self.ctrl_panels = tk.Frame(parent)
        self.ctrl_panels.pack(fill=tk.X, padx=4, pady=2)

        left = tk.LabelFrame(self.ctrl_panels, text="Random Target")
        middle = tk.LabelFrame(self.ctrl_panels, text="目標値の入力")
        param = tk.LabelFrame(self.ctrl_panels, text="ADRCパラメータの調整（選択中の1自由度）")
        right = tk.LabelFrame(self.ctrl_panels, text="記録（全DOFの全グラフの値・ADRCパラメータ）")
        self.build_random_area(left)
        self.build_target_area(middle)
        self.build_param_panel(param)
        self.build_record_area(right)

        self.ctrl_columns = (left, middle, param, right)
        self.ctrl_layout_mode = None
        self.layout_controls()
        self.ctrl_inner.master.bind("<Configure>", lambda e: self.layout_controls(), add="+")

    def layout_controls(self):
        """横に並びきるなら1段。狭ければ 上段 = 目標値の入力・パラメータ調整、下段 = Random Target・記録"""
        left, middle, param, right = self.ctrl_columns
        view_width = self.ctrl_inner.master.winfo_width()
        one_row = sum(c.winfo_reqwidth() for c in self.ctrl_columns) + 40
        mode = 'row' if view_width <= 1 or view_width >= one_row else 'stack'
        if mode == self.ctrl_layout_mode:
            return
        self.ctrl_layout_mode = mode

        panels = self.ctrl_panels
        if mode == 'row':
            for c, panel in enumerate((left, middle, param, right)):
                panel.grid(row=0, column=c, sticky="nsew", padx=3, pady=2)
                panels.columnconfigure(c, weight=1)
        else:
            middle.grid(row=0, column=0, sticky="nsew", padx=3, pady=2)
            param.grid(row=0, column=1, sticky="nsew", padx=3, pady=2)
            left.grid(row=1, column=0, sticky="nsew", padx=3, pady=2)
            right.grid(row=1, column=1, sticky="nsew", padx=3, pady=2)
            for c, weight in enumerate((1, 1, 0, 0)):
                panels.columnconfigure(c, weight=weight)

    def build_param_panel(self, parent):
        # 入力欄を作るときに Spinbox が値を書いても送信予定値にしない（最後の load_param_panel で書く）
        self.plan_programmatic = True
        top = tk.Frame(parent)
        top.pack(fill=tk.X, padx=4, pady=(2, 0))
        tk.Label(top, text="選択中のDOF").pack(side=tk.LEFT)
        self.param_dof_box = ttk.Combobox(top, values=[f"DOF {i}" for i in range(26)], state="readonly", width=8)
        self.param_dof_box.current(self.current_dof)
        self.param_dof_box.bind("<<ComboboxSelected>>",
                                lambda e: self.select_dof(self.param_dof_box.current(), scroll=True))
        self.param_dof_box.pack(side=tk.LEFT, padx=4)
        self.param_dof_info = tk.Label(top, text="", font=(base.UI_FONT, 9, "bold"))
        self.param_dof_info.pack(side=tk.LEFT, padx=(6, 0))

        grid = tk.Frame(parent)
        grid.pack(padx=4, pady=2)
        for c, text in enumerate(("", "現在設定値\n(Teensyから受信)", "送信予定値\n(手入力・▲▼)", "▲▼の刻み", ".ino初期値")):
            tk.Label(grid, text=text, font=(base.UI_FONT, 8), fg="#505050").grid(row=0, column=c, padx=3)

        self.param_vars, self.param_spins, self.param_cur_labels, self.param_def_labels = {}, {}, {}, {}
        for r, key in enumerate(adrc.PARAM_KEYS, start=1):
            lo, hi = adrc.PARAM_LIMITS[key]
            tk.Label(grid, text=key, font=(base.UI_FONT, 10, "bold")).grid(row=r, column=0, sticky="w", padx=3)

            cur = tk.Label(grid, text="", width=10, anchor="e", font=(base.UI_FONT, 10, "bold"),
                           relief=tk.SUNKEN, bg="#f4f4f4")
            cur.grid(row=r, column=1, padx=3, pady=1)

            var = tk.StringVar()
            spin = tk.Spinbox(grid, from_=int(lo), to=int(hi), increment=int(PARAM_STEP_DEFAULT[key]), width=10,
                              justify="right", textvariable=var, font=(base.UI_FONT, 10))
            spin.grid(row=r, column=2, padx=3, pady=1)
            var.trace_add("write", lambda *a, k=key: self.on_plan_edit(k))

            step = ttk.Combobox(grid, values=PARAM_STEPS[key], width=6)
            step.set(PARAM_STEP_DEFAULT[key])
            step.grid(row=r, column=3, padx=3, pady=1)
            for seq in ("<<ComboboxSelected>>", "<KeyRelease>"):
                step.bind(seq, lambda e, k=key, s=step: self.on_step_change(k, s))

            dlab = tk.Label(grid, text="", width=9, anchor="e", fg="#606060")
            dlab.grid(row=r, column=4, padx=3, pady=1)

            self.param_vars[key] = var
            self.param_spins[key] = spin
            self.param_cur_labels[key] = cur
            self.param_def_labels[key] = dlab

        buttons = tk.Frame(parent)
        buttons.pack(padx=4, pady=(2, 0))
        self.param_send_button = tk.Button(buttons, text="", width=14, font=(base.UI_FONT, 10, "bold"),
                                           command=self.send_selected_dof)
        self.param_send_button.pack(side=tk.LEFT)
        tk.Button(buttons, text="現在設定値に戻す", command=self.plan_from_current_selected).pack(side=tk.LEFT, padx=(8, 0))
        tk.Button(buttons, text=".ino初期値に戻す", command=self.plan_from_default_selected).pack(side=tk.LEFT, padx=(4, 0))

        # 状態 + 「一つ前のパラメータに戻す」（選択中の自由度がエラー（発散など）のときだけ押せる）
        state_row = tk.Frame(parent)
        state_row.pack(fill=tk.X, padx=4, pady=(2, 0))
        self.param_revert_button = tk.Button(state_row, text="一つ前のパラメータに戻す", fg="#a00000",
                                             font=(base.UI_FONT, 9, "bold"), command=self.revert_previous_params,
                                             state=tk.DISABLED)
        self.param_revert_button.pack(side=tk.RIGHT, anchor="n")
        self.param_state = tk.Label(state_row, text="", font=(base.UI_FONT, 10, "bold"), anchor="w", padx=4,
                                    justify="left", wraplength=250)
        self.param_state.pack(side=tk.LEFT, fill=tk.X, expand=True)
        state_row.bind("<Configure>", lambda e: self.param_state.config(
            wraplength=max(120, e.width - self.param_revert_button.winfo_reqwidth() - 2 * 4 - 8)))
        # 注記は行数を固定せず、欄の幅で折り返す（文字が切れないように）
        self.param_note = tk.Label(parent, text="", font=(base.UI_FONT, 8), fg="#505050", anchor="nw",
                                   justify="left", wraplength=430)
        self.param_note.pack(fill=tk.X, padx=4, pady=(0, 4))
        self.param_note.bind("<Configure>", lambda e: self.param_note.config(wraplength=max(150, e.width - 4)))

        self.load_param_panel()

    def on_step_change(self, key, box):
        try:
            step = float(box.get())
        except ValueError:
            return
        if step > 0:
            self.param_spins[key].config(increment=step)

    def load_param_panel(self):
        """選択中の自由度の送信予定値を入力欄に書く"""
        dof = self.current_dof
        self.plan_programmatic = True
        try:
            for key in adrc.PARAM_KEYS:
                self.param_vars[key].set(adrc.fmt_param(self.plan['values'][key][dof]))
        finally:
            self.plan_programmatic = False
        self.plan_invalid.clear()
        self.refresh_param_panel()

    def on_plan_edit(self, key):
        """送信予定値の欄の変更（範囲内の数値になったときだけ送信予定値にする）"""
        if self.plan_programmatic:
            return
        try:
            v = float(self.param_vars[key].get().strip())
        except ValueError:
            v = None
        if adrc.check_value(key, v) is None:
            self.plan['values'][key][self.current_dof] = v
            self.plan_invalid.discard(key)
        else:
            self.plan_invalid.add(key)
        self.refresh_param_panel()

    def refresh_param_panel(self):
        dof = self.current_dof
        board, local = adrc.dof_slot(dof)
        target = self.plan['targets'][dof]
        now = time.monotonic()
        cur = self.node.adrc.current(dof, now)
        default, source = self.effective_default(dof)

        self.param_dof_info.config(text=f"{board} #{local}   " + ("制御対象" if target else "制御対象外"),
                                   fg=("#006000" if target else "#a00000"))
        for key in adrc.PARAM_KEYS:
            self.param_cur_labels[key].config(text=("未受信" if cur is None else adrc.fmt_param(cur[key])))
            self.param_def_labels[key].config(text=adrc.fmt_param(default[key]))
            planned = self.plan['values'][key][dof]
            if key in self.plan_invalid or (target and adrc.check_value(key, planned)):
                bg = "#ffc0c0"
            elif cur is not None and not adrc.same_value(planned, cur[key]):
                bg = "#fff3b0"
            else:
                bg = "white"
            self.param_spins[key].config(state=(tk.NORMAL if target else tk.DISABLED), bg=bg)

        error = self.dof_error(dof, now)
        sendable = target and not self.plan_invalid and not any(
            adrc.check_value(k, self.plan['values'][k][dof]) for k in adrc.PARAM_KEYS)
        if error:
            code, text = 'error', f"エラー（{error}）"
        elif not target:
            code, text = 'not_target', "制御対象外のため送信できません"
        elif self.plan_invalid:
            code, text = 'invalid', "送信予定値の欄が不正です（" + ", ".join(sorted(self.plan_invalid)) + "）"
        else:
            code, text = self.plan_dof_state(dof, now)
        bg, fg = adrc.STATE_TAG_STYLE.get(code, ("white", "black"))
        self.param_state.config(text=f"状態: {text}", bg=bg, fg=fg)
        self.param_send_button.config(text=f"DOF {dof} へ送信", state=(tk.NORMAL if sendable else tk.DISABLED))
        # 「一つ前のパラメータに戻す」はエラーのときだけ
        history = self.param_history[dof]
        self.param_revert_button.config(state=(tk.NORMAL if error and target and history else tk.DISABLED))

        notes = []
        if error:
            if history:
                prev = ", ".join(f"{k} {adrc.fmt_param(history[-1][k])}" for k in adrc.PARAM_KEYS)
                notes.append(f"一つ前のパラメータ（最後に送信する前の現在設定値）: {prev}")
            else:
                notes.append("一つ前のパラメータがありません（起動後このGUIからこの自由度へ送信していません）。"
                             "「.ino初期値に戻す」などで送信予定値を直して送信してください。")
        if not target:
            notes.append("「制御対象DOF」タブで制御対象にすると、調整・送信できます。")
        else:
            lo_hi = ", ".join(f"{k} {adrc.fmt_param(adrc.PARAM_LIMITS[k][0])}〜{adrc.fmt_param(adrc.PARAM_LIMITS[k][1])}"
                              for k in adrc.PARAM_KEYS)
            notes.append(f"範囲: {lo_hi}")
        notes.append(("未保存の変更があります（「ADRCパラメータ一覧」タブの「保存」で保存）" if self.plan_dirty()
                      else "送信予定値・制御対象は保存済みです") + f"   .ino初期値の出所: {source}")
        note = "\n".join(notes)
        if self.param_note.cget("text") != note:
            self.param_note.config(text=note)

    def select_dof(self, dof, scroll=False):
        """選択中の自由度（スライダ・パラメータ調整の対象）を変える"""
        if dof != self.current_dof:
            self.dof_box.current(dof)
            self.change_dof(None)
        if scroll:
            self.root.after(150, lambda: self.scroll_to_dof(dof))

    def change_dof(self, event):
        super().change_dof(event)
        self.param_dof_box.current(self.current_dof)
        self.load_param_panel()

    def send_selected_dof(self):
        """選択中の自由度の送信予定値（4つ）だけを送る（同じボードの他の自由度は「変更しない」）"""
        dof = self.current_dof
        if not self.plan['targets'][dof] or self.plan_invalid:
            return
        planned = adrc.dof_values(self.plan, dof)
        errors = [f"{k}: {r}" for k in adrc.PARAM_KEYS for r in [adrc.check_value(k, planned[k])] if r]
        if errors:
            messagebox.showerror("Send error", f"DOF {dof} の送信予定値を送信できません:\n" + "\n".join(errors))
            return
        board, local = adrc.dof_slot(dof)
        self.remember_previous(dof, planned)
        self.node.adrc.send(board, {local: planned})
        self.refresh_param_panel()

    def remember_previous(self, dof, new_values):
        """送信する直前の Teensy の現在設定値を「一つ前のパラメータ」として覚える（送る値と同じなら覚えない）"""
        cur = self.node.adrc.current(dof)
        if cur is None or any(v is None for v in cur.values()):
            return
        if all(adrc.same_value(cur[k], new_values[k]) for k in adrc.PARAM_KEYS):
            return
        history = self.param_history[dof]
        history.append(dict(cur))
        del history[:-PARAM_HISTORY_MAX]

    def revert_previous_params(self):
        """エラー（発散など）の自由度を、一つ前のパラメータ（最後に送信する前の現在設定値）に戻して送信する"""
        dof = self.current_dof
        history = self.param_history[dof]
        if not (history and self.plan['targets'][dof] and self.dof_error(dof)):
            return
        prev = history[-1]
        errors = [f"{k}: {r}" for k in adrc.PARAM_KEYS for r in [adrc.check_value(k, prev[k])] if r]
        if errors:
            messagebox.showerror("Revert error", f"DOF {dof} の一つ前のパラメータは送信できる範囲の外です:\n"
                                 + "\n".join(errors))
            return
        history.pop()
        for key in adrc.PARAM_KEYS:
            self.plan['values'][key][dof] = prev[key]
        self.load_param_panel()
        board, local = adrc.dof_slot(dof)
        self.node.adrc.send(board, {local: dict(prev)})
        text = ", ".join(f"{k} {adrc.fmt_param(prev[k])}" for k in adrc.PARAM_KEYS)
        self.set_list_message(f"DOF {dof} を一つ前のパラメータに戻して送信しました（{text}）", "#a00000")
        self.refresh_param_panel()

    def plan_from_current_selected(self):
        dof = self.current_dof
        cur = self.node.adrc.current(dof)
        if cur is None or any(v is None for v in cur.values()):
            messagebox.showinfo("ADRC parameters", f"DOF {dof} の現在設定値を Teensy から受信していません")
            return
        for key in adrc.PARAM_KEYS:
            self.plan['values'][key][dof] = cur[key]
        self.load_param_panel()

    def plan_from_default_selected(self):
        dof = self.current_dof
        default, _source = self.effective_default(dof)
        for key in adrc.PARAM_KEYS:
            self.plan['values'][key][dof] = default[key]
        self.load_param_panel()

    # =========================================================
    # 「ADRCパラメータ一覧」タブ・「制御対象DOF」タブ
    # =========================================================
    def build_extra_tabs(self):
        self.notebook.tab(self.tab_pot, text="調整")
        self.tab_list = tk.Frame(self.notebook)
        self.tab_targets = tk.Frame(self.notebook)
        self.notebook.insert(self.tab_pose, self.tab_list, text="ADRCパラメータ一覧")
        self.notebook.insert(self.tab_pose, self.tab_targets, text="制御対象DOF")
        self.build_list_tab(self.tab_list)
        self.build_targets_tab(self.tab_targets)

    def build_list_tab(self, parent):
        row1 = tk.Frame(parent)
        row1.pack(fill=tk.X, padx=6, pady=(6, 2))
        tk.Button(row1, text="保存", width=8, font=(base.UI_FONT, 10, "bold"),
                  command=self.save_plan).pack(side=tk.LEFT)
        tk.Button(row1, text="名前を付けて保存…", command=self.save_plan_as).pack(side=tk.LEFT, padx=(4, 0))
        tk.Button(row1, text="ファイルを開く…", command=self.open_plan_file).pack(side=tk.LEFT, padx=(4, 0))
        tk.Button(row1, text="保存した値に戻す", command=self.revert_plan).pack(side=tk.LEFT, padx=(4, 0))
        self.list_dirty_label = tk.Label(row1, text="", font=(base.UI_FONT, 10, "bold"))
        self.list_dirty_label.pack(side=tk.LEFT, padx=12)

        row2 = tk.Frame(parent)
        row2.pack(fill=tk.X, padx=6, pady=2)
        tk.Button(row2, text="全DOFを .ino初期値に戻す", command=self.plan_all_defaults).pack(side=tk.LEFT)
        tk.Button(row2, text="Teensyの現在設定値を送信予定値に取り込む",
                  command=self.plan_all_from_current).pack(side=tk.LEFT, padx=(4, 0))
        tk.Button(row2, text="制御対象DOFをまとめて送信…", command=self.send_all_targets).pack(side=tk.LEFT, padx=(16, 0))

        self.list_message = tk.Label(parent, text="", font=(base.UI_FONT, 9), anchor="w", justify="left",
                                     wraplength=1200)
        self.list_message.pack(fill=tk.X, padx=6)
        tk.Label(parent,
                 text="予定 … 送信予定値（調整タブで変更）。「保存」で保存し、GUI_android_tk.py が起動時に読み込みます"
                      f"（{adrc.PARAM_FILE}）。\n"
                      "現在 … Teensy から受信した現在設定値（送信した値ではありません）。"
                      "初期 … .ino の初期値（Teensy から受信できればその値、できなければ GUI の表の値）。\n"
                      ".ino初期値に戻す・取り込む・開くは送信予定値を変えるだけで、送信はしません。",
                 font=(base.UI_FONT, 9), fg="#505050", justify="left").pack(anchor="w", padx=6, pady=(2, 0))

        self.list_table = adrc.ParamTable(parent, [
            ("予定", lambda dof, key: self.plan['values'][key][dof]),
            ("現在", self.current_value),
            ("初期", lambda dof, key: self.effective_default(dof)[0][key]),
        ], font=(base.UI_FONT, 9, "bold"), height=20)
        self.set_list_message(self.list_message_initial)

    def current_value(self, dof, key):
        cur = self.node.adrc.current(dof)
        return None if cur is None else cur[key]

    def set_list_message(self, text, color="#006000"):
        self.list_message.config(text=f"[{time.strftime('%H:%M:%S')}] {text}", fg=color)

    def board_text(self, board):
        """一覧の board 行に出す、Teensy との通信と .ino 初期値の照合の状態"""
        link = self.node.adrc
        st = link.board_status(board)
        if link.bad_status[board] is not None:
            return f"状態トピックの要素数が {link.bad_status[board]}（{adrc.STATUS_LEN} のはず）"
        if st is None:
            return "Teensy 未受信" if link.board_status(board, allow_stale=True) is None else "Teensy 受信が途切れています"
        mismatch = [d for d in adrc.board_dofs(board)
                    if any(not adrc.same_value(st['default'][d - adrc.board_dofs(board)[0]][k], adrc.INO_DEFAULTS[k][d])
                           for k in adrc.PARAM_KEYS)]
        if mismatch:
            return f"受信中  ⚠ .ino初期値がGUIの表と違う（DOF {', '.join(map(str, mismatch))}）"
        return f"受信中  最後の処理 seq {st['ack_seq']}: {adrc.RESULT_TEXT.get(st['result'], st['result'])}"

    def build_targets_tab(self, parent):
        tk.Label(parent,
                 text="制御対象の自由度だけ、パラメータの調整・送信ができます（GUI_android_tk.py の送信も制御対象の自由度だけ）。\n"
                      "制御対象の選択は送信予定値と一緒に「ADRCパラメータ一覧」タブの「保存」で保存します。\n"
                      "※ 制御対象にしても、Teensy の .ino がその自由度を出力していなければ実際には動きません。"
                      "現在の .ino では board4・board5 の要素3（DOF21・DOF25）の PWM・z3・z1・derivatives・z2 は0固定で送信されます。",
                 font=(base.UI_FONT, 9), fg="#505050", justify="left").pack(anchor="w", padx=6, pady=(6, 4))
        bar = tk.Frame(parent)
        bar.pack(fill=tk.X, padx=6, pady=2)
        tk.Button(bar, text="既定に戻す（DOF " + "・".join(map(str, adrc.DEFAULT_NON_TARGETS)) + " 以外）",
                  command=lambda: self.set_targets([d not in adrc.DEFAULT_NON_TARGETS for d in range(26)])
                  ).pack(side=tk.LEFT)
        tk.Button(bar, text="全選択", command=lambda: self.set_targets([True] * 26)).pack(side=tk.LEFT, padx=(4, 0))
        tk.Button(bar, text="全解除", command=lambda: self.set_targets([False] * 26)).pack(side=tk.LEFT, padx=(4, 0))
        self.targets_message = tk.Label(parent, text="", font=(base.UI_FONT, 9), anchor="w", justify="left",
                                        fg="#a04000", wraplength=1000)
        self.targets_message.pack(fill=tk.X, padx=6, pady=(2, 4))

        _, inner = self.make_scrollable(parent)
        self.target_vars = []
        for board, offset, count in base.BOARD_LAYOUT:
            box = tk.LabelFrame(inner, text=f"{board}  (DOF {offset}〜{offset + count - 1})")
            box.pack(fill=tk.X, anchor="w", pady=3)
            for local in range(count):
                dof = offset + local
                v = tk.BooleanVar(value=self.plan['targets'][dof])
                tk.Checkbutton(box, text=f"DOF {dof}  (#{local})", variable=v,
                               command=lambda d=dof: self.on_target_toggle(d)).grid(row=local, column=0, sticky="w", padx=4)
                if adrc.INO_DEFAULTS['input_coef'][dof] == 0:
                    tk.Label(box, text=".ino の初期値は未設定（kp・kd・input_coef・lambda_0 = 0）。"
                                       "制御対象にしたら送信予定値を入力してから送信してください",
                             font=(base.UI_FONT, 8), fg="#808080").grid(row=local, column=1, sticky="w", padx=8)
                self.target_vars.append(v)

    def set_targets(self, targets):
        self.plan['targets'] = list(targets)
        self.sync_target_vars()
        self.after_targets_change()

    def sync_target_vars(self):
        for dof, v in enumerate(self.target_vars):
            v.set(self.plan['targets'][dof])

    def on_target_toggle(self, dof):
        self.plan['targets'][dof] = bool(self.target_vars[dof].get())
        self.after_targets_change()

    def after_targets_change(self):
        bad = [d for d, _k, _r in adrc.validate_param_set(self.plan)]
        if bad:
            dofs = ", ".join(map(str, sorted(set(bad))))
            self.targets_message.config(text=f"DOF {dofs} の送信予定値に範囲外の値があります。"
                                             "調整タブで値を入力してから送信・保存してください。")
        else:
            self.targets_message.config(text="")
        self.load_param_panel()

    # -------------------------
    # 一覧タブの操作
    # -------------------------
    def save_plan(self, path=None):
        """送信予定値・制御対象を保存する（path=None なら GUI_android_tk.py が読むファイル）"""
        errors = adrc.validate_param_set(self.plan)
        if errors:
            more = f"\n…ほか {len(errors) - 15} 件" if len(errors) > 15 else ""
            messagebox.showerror("Save error", "制御対象の自由度に範囲外の値があるため保存できません:\n"
                                 + "\n".join(f"DOF {d} {k}: {r}" for d, k, r in errors[:15]) + more)
            return False
        try:
            saved_at = adrc.save_param_file(self.plan, path)
        except OSError as e:
            messagebox.showerror("Save error", f"{path or adrc.PARAM_FILE} に保存できません:\n{e}")
            return False
        if path is None:
            self.plan['saved_at'] = saved_at
            self.saved = adrc.copy_param_set(self.plan)
            self.set_list_message(f"{adrc.PARAM_FILE} に保存しました（GUI_android_tk.py は起動時・再読込時にこの値を読みます）")
        else:
            self.set_list_message(f"{path} に保存しました（GUI_android_tk.py が読むファイルは変わっていません）")
        return True

    def save_plan_as(self):
        path = filedialog.asksaveasfilename(title="Save ADRC parameters as", defaultextension=".json",
                                            filetypes=[("JSON", "*.json"), ("All files", "*")],
                                            initialdir=os.path.expanduser('~'))
        if path:
            self.save_plan(path)

    def open_plan_file(self):
        path = filedialog.askopenfilename(title="Open ADRC parameters",
                                          filetypes=[("JSON", "*.json"), ("All files", "*")],
                                          initialdir=os.path.expanduser('~'))
        if not path:
            return
        ps, err = adrc.load_param_file(path)
        if ps is None:
            messagebox.showerror("Open error", err)
            return
        self.replace_plan(ps, f"{path} を読み込みました（送信予定値・制御対象。「保存」で GUI_android_tk.py 用のファイルに保存します）")

    def revert_plan(self):
        if self.saved is None:
            messagebox.showinfo("ADRC parameters", f"{adrc.PARAM_FILE} がまだありません")
            return
        self.replace_plan(self.saved, "送信予定値・制御対象を保存した値に戻しました")

    def plan_all_defaults(self):
        ps = adrc.copy_param_set(self.plan)
        sources = {}
        for dof in range(26):
            default, source = self.effective_default(dof)
            sources.setdefault(source, set()).add(adrc.dof_slot(dof)[0])
            for key in adrc.PARAM_KEYS:
                ps['values'][key][dof] = default[key]
        where = "、".join(f"{src}: {', '.join(sorted(boards))}" for src, boards in sources.items())
        self.replace_plan(ps, f"全DOFの送信予定値を .ino の初期値に戻しました（{where}。送信はしていません）")

    def plan_all_from_current(self):
        ps = adrc.copy_param_set(self.plan)
        got = []
        for dof in range(26):
            cur = self.node.adrc.current(dof)
            if cur is None or any(v is None for v in cur.values()):
                continue
            for key in adrc.PARAM_KEYS:
                ps['values'][key][dof] = cur[key]
            got.append(dof)
        if not got:
            messagebox.showinfo("ADRC parameters", "Teensy から現在設定値を受信していません")
            return
        missing = [d for d in range(26) if d not in got]
        self.replace_plan(ps, f"{len(got)} 自由度の送信予定値を Teensy の現在設定値にしました"
                              + (f"（未受信のため変えていない DOF: {', '.join(map(str, missing))}）" if missing else ""))

    def send_all_targets(self):
        """制御対象の全自由度の送信予定値を送る（読み込み・初期値への復元のあとにまとめて反映するため）"""
        errors = adrc.validate_param_set(self.plan)
        if errors:
            messagebox.showerror("Send error", "制御対象の自由度に範囲外の値があるため送信できません:\n"
                                 + "\n".join(f"DOF {d} {k}: {r}" for d, k, r in errors[:15]))
            return
        link = self.node.adrc
        changes = []
        for dof in range(26):
            if not self.plan['targets'][dof]:
                continue
            cur = link.current(dof)
            for key in adrc.PARAM_KEYS:
                new = self.plan['values'][key][dof]
                if cur is None:
                    changes.append(f"DOF {dof} {key}: (未受信) → {adrc.fmt_param(new)}")
                elif not adrc.same_value(new, cur[key]):
                    changes.append(f"DOF {dof} {key}: {adrc.fmt_param(cur[key])} → {adrc.fmt_param(new)}")
        more = f"\n…ほか {len(changes) - 20} 件" if len(changes) > 20 else ""
        text = (f"制御対象の {sum(self.plan['targets'])} 自由度の送信予定値を Teensy へ送信します。\n\n"
                + (f"現在設定値から変わる値: {len(changes)} 件\n" + "\n".join(changes[:20]) + more
                   if changes else "現在設定値から変わる値はありません。"))
        if not messagebox.askokcancel("Send ADRC parameters", text):
            return
        for board, offset, count in base.BOARD_LAYOUT:
            local_values = {d - offset: adrc.dof_values(self.plan, d)
                            for d in range(offset, offset + count) if self.plan['targets'][d]}
            for local, values in local_values.items():
                self.remember_previous(offset + local, values)
            if local_values:
                link.send(board, local_values)
        self.set_list_message("制御対象の全自由度の送信予定値を送信しました（状態欄で設定完了を確認してください）")

    # -------------------------
    # 周期的な表示の更新
    # -------------------------
    def poll_adrc(self):
        now = time.monotonic()
        # 発散の検出と、エラーの自由度の見出しの表示（グラフを止めていても検出は続ける）
        self.check_divergence(now)
        self.highlight_slider_plot()
        self.refresh_param_panel()
        states = {dof: self.plan_dof_state(dof, now) for dof in range(26)}
        self.list_table.refresh(lambda d: self.plan['targets'][d], lambda d: states[d], self.board_text)
        dirty = self.plan_dirty()
        text, color = (("未保存の変更があります", "#c06000") if dirty else
                       (f"保存済み（{(self.saved or {}).get('saved_at') or '-'}）", "#006000"))
        if self.list_dirty_label.cget("text") != text:
            self.list_dirty_label.config(text=text, fg=color)
        self.root.after(300, self.poll_adrc)

    def adrc_record_rows(self):
        """記録用: 全自由度の Teensy の現在設定値（制御対象は送信予定値側の選択）"""
        return self.node.adrc.snapshot(list(self.plan['targets']))

    # -------------------------
    # 終了
    # -------------------------
    def ask_save_on_exit(self):
        """「保存して終了」「保存せずに終了」の2択（どちらかを選ぶまで閉じない）。戻り値 'save' / 'discard'"""
        dlg = tk.Toplevel(self.root)
        dlg.title("Unsaved ADRC parameters")
        dlg.transient(self.root)
        dlg.resizable(False, False)
        tk.Label(dlg, text="ADRCパラメータ（送信予定値・制御対象）に未保存の変更があります。",
                 font=(base.UI_FONT, 10), padx=16, pady=12).pack()
        tk.Label(dlg, text=f"保存先: {adrc.PARAM_FILE}（GUI_android_tk.py が起動時に読み込みます）",
                 font=(base.UI_FONT, 9), fg="#505050", padx=16).pack()
        choice = {'value': None}

        def choose(value):
            choice['value'] = value
            dlg.destroy()

        buttons = tk.Frame(dlg)
        buttons.pack(pady=12)
        tk.Button(buttons, text="保存して終了", width=14, font=(base.UI_FONT, 10, "bold"),
                  command=lambda: choose('save')).pack(side=tk.LEFT, padx=8)
        tk.Button(buttons, text="保存せずに終了", width=14, command=lambda: choose('discard')).pack(side=tk.LEFT, padx=8)
        dlg.protocol("WM_DELETE_WINDOW", lambda: None)
        dlg.update_idletasks()
        x = self.root.winfo_rootx() + (self.root.winfo_width() - dlg.winfo_reqwidth()) // 2
        y = self.root.winfo_rooty() + (self.root.winfo_height() - dlg.winfo_reqheight()) // 3
        dlg.geometry(f"+{max(0, x)}+{max(0, y)}")
        dlg.grab_set()
        self.root.wait_window(dlg)
        return choice['value']

    def on_close(self):
        if self.plan_dirty():
            if self.ask_save_on_exit() == 'save' and not self.save_plan():
                return      # 保存できなかったので終了しない（もう一度閉じると選び直せる）
        super().on_close()


# ------------------------------
def main():
    base.check_ranges()
    rclpy.init()
    settings, warning = load_tune_settings()
    node = AdrcTuneNode(settings['initial_pose'])
    spin_thread = threading.Thread(target=base.spin_ros, args=(node,), daemon=True)
    spin_thread.start()
    gui = AdrcTuneTk(node, settings, warning)
    gui.run()
    # 受信を止めてから終了する（受信コールバックの途中でPythonが終了すると異常終了することがあるため）
    rclpy.try_shutdown()
    spin_thread.join(timeout=2)
    node.destroy_node()


if __name__ == '__main__':
    main()
