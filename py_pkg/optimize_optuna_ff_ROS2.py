#!/usr/bin/env python3
"""
Optuna で5次関数FF（a t⁵ + b t⁴ + c t³ + d t² + e t, 0 ≤ t < T）を実機試行により探索するプログラム（GUI）

<処理の流れ>（計画書 第4版, 2026-10-23）
  1. 基準試行：FF=0（ADRC のみ）で目標値を送り、評価区間 max(1.5T, 5 秒) のあいだ記録する（回数は GUI で指定、既定 2）。
     終わるたびに FF を2段で止め（T=0 → 係数0）、静止を待ってから初期姿勢へ戻し、待機時間だけ待つ。
  2. 目標軌道：基準試行に ζ=1 の3次遅れ系を「T までに |y| ≤ ε|ΔP| に入り以後も維持」の拘束付きで当てはめ、固定する。
  3. Optuna：u = (u1,u2,u3,A)（自由度ごと）を選ぶ → 写像で FF を作る → float32 でファームの計算を再現して検証 →
     近傍棄却 → 実機で max(1.5T, 5 秒) 測定 → J = mean_i mean((POT − 目標軌道)/ΔP)² を tell（log10 J）。
     J ≤ 閾値なら同じ FF で確認試行をし、2回の平均も閾値以下なら終了する。
     評価に失敗した試行は TrialState.FAIL（原因・データ・ログを残す）。
<時間軸> 100 Hz 配信の番号 × 10 ms（制御周期は使わない）。<計算・通信・保存> optuna_ff_common.py
<対応 Arduino プログラム> ADRC_ROS2_tk_FF_ADRCparamChange-board1〜5（変更しない）

<保存>  <保存先>/optimize_YYYYMMDD_HHMM/
    *_results.csv           … best（COMPLETE で J 最小）の、送った float32 の係数から求めた極値（既存と同じ168列）
    *_trials.csv            … 全試行（FAIL も含む）。試行ごとに追記
    *_conditions.csv        … 再現に必要な条件（写像・目標軌道・Sampler と引数・seed・閾値・安全の設定・ADRC パラメータ …）
    *_best_trial.csv        … best の受信データ（GUI_android_tk.py の記録CSV と同じ列 ＋ FF・ADRC・z1・z2・derivatives）
    *_best_trial_grid.csv   … best を配信の時間軸にそろえた解析用データ
    study.journal           … Optuna の履歴（再開に使う）      target.json … 目標軌道      config.json … 設定
    ffmap_table.npz         … FF の可行域の表        trials/      … 全測定の生データ（npz）と例外の traceback
    DOF/DOF<d>/             … グラフの画像            run.log      … ログ      status.json … 状態
"""
import base64
import json
import math
import os
import queue
import signal
import sys
import threading
import time
import traceback

import numpy as np

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from tkinter import font as tkfont

if __package__:
    from . import optuna_ff_common as c
else:
    import optuna_ff_common as c

APP_SETTINGS_FILE = os.path.expanduser('~/.armrobot_optuna_ff_settings.json')
gat = sys.modules[__name__]     # 変更: 移植した GUI 部品が参照する gat（GUI_android_tk）を、このモジュール自身にする
POT_RANGE = c.POT_RANGE         # 変更: 移植した TargetTable が参照する gat.POT_RANGE


# ==============================================================================
# 移植：日本語フォント（GUI_android_tk.py）
# ==============================================================================
# 移植元: GUI_android_tk.py:71-77 @f8fba60
JP_FONT_CANDIDATES = [
    "Noto Sans CJK JP", "Noto Sans JP", "Source Han Sans JP",
    "IPAexGothic", "IPAPGothic", "IPAGothic",
    "TakaoPGothic", "TakaoGothic", "VL PGothic",
    "Yu Gothic UI", "Meiryo UI", "Meiryo", "MS UI Gothic", "MS Gothic",
    "BIZ UDPゴシック", "BIZ UDゴシック",
]


# 移植元: GUI_android_tk.py:80-80 @f8fba60
UI_FONT = "TkDefaultFont"


# 移植元: GUI_android_tk.py:86-129 @f8fba60
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


# ==============================================================================
# 移植：GUI 部品（optimize_EL2_gui.py）
# ==============================================================================
# 移植元: optimize_EL2_gui.py:27-50 @f8fba60
def load_json(path, default):
    try:
        with open(path, encoding='utf-8') as f:
            data = json.load(f)
        return data if isinstance(data, dict) else default
    except (OSError, ValueError):
        return default


def save_json(path, data):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    os.replace(tmp, path)


def parse_float(text, name, lo=-math.inf, hi=math.inf, positive=False):
    try:
        v = float(str(text).strip())
    except ValueError:
        raise ValueError(f"{name} が数値ではありません: {text!r}")
    if not math.isfinite(v) or v < lo or v > hi or (positive and v <= 0):
        raise ValueError(f"{name} = {v} は範囲外です")
    return v


# 移植元: optimize_EL2_gui.py:65-142 @f8fba60
class Logger:
    def __init__(self):
        self.q = queue.Queue()
        self.file = None

    def open_file(self, path):
        self.close_file()
        self.file = open(path, 'a', encoding='utf-8')

    def close_file(self):
        if self.file is not None:
            try:
                self.file.close()
            except OSError:
                pass
        self.file = None

    def __call__(self, msg):
        line = f"[{time.strftime('%H:%M:%S')}] {msg}"
        print(line, flush=True)
        self.q.put(line)
        if self.file is not None:
            try:
                self.file.write(line + '\n')
                self.file.flush()
            except OSError:
                pass


class LogPane:
    """ログ表示（選択してコピーできる。全体のコピーと保存のボタン付き）"""

    def __init__(self, parent, logger, height=10):
        self.logger = logger
        frame = ttk.LabelFrame(parent, text="ログ（ターミナルと同じ内容）")
        frame.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)
        bar = ttk.Frame(frame)
        bar.pack(fill=tk.X)
        ttk.Button(bar, text="全体をコピー", command=self.copy_all).pack(side=tk.LEFT, padx=2, pady=2)
        ttk.Button(bar, text="ファイルに保存…", command=self.save).pack(side=tk.LEFT, padx=2, pady=2)
        ttk.Button(bar, text="表示を消去", command=lambda: self.text.delete('1.0', tk.END)).pack(side=tk.LEFT, padx=2)
        body = ttk.Frame(frame)
        body.pack(fill=tk.BOTH, expand=True)
        self.text = tk.Text(body, height=height, wrap='none', font=('TkFixedFont', 9))
        vs = ttk.Scrollbar(body, orient=tk.VERTICAL, command=self.text.yview)
        hs = ttk.Scrollbar(body, orient=tk.HORIZONTAL, command=self.text.xview)
        self.text.configure(yscrollcommand=vs.set, xscrollcommand=hs.set)
        self.text.grid(row=0, column=0, sticky='nsew')
        vs.grid(row=0, column=1, sticky='ns')
        hs.grid(row=1, column=0, sticky='ew')
        body.rowconfigure(0, weight=1)
        body.columnconfigure(0, weight=1)
        # 読み取り専用にするが、選択とコピー（Ctrl+C）はできるようにする
        self.text.bind('<Key>', lambda e: None if (e.state & 0x4 and e.keysym.lower() in ('c', 'a')) else 'break')

    def poll(self):
        n = 0
        while n < 500:
            try:
                line = self.logger.q.get_nowait()
            except queue.Empty:
                break
            at_end = self.text.yview()[1] > 0.999
            self.text.insert(tk.END, line + '\n')
            if at_end:
                self.text.see(tk.END)
            n += 1

    def copy_all(self):
        root = self.text.winfo_toplevel()
        root.clipboard_clear()
        root.clipboard_append(self.text.get('1.0', tk.END))

    def save(self):
        path = filedialog.asksaveasfilename(title="Save log", defaultextension='.txt', filetypes=[('Text', '*.txt')])
        if path:
            with open(path, 'w', encoding='utf-8') as f:
                f.write(self.text.get('1.0', tk.END))


# 移植元: optimize_EL2_gui.py:148-273 @f8fba60
class Fields:
    """ラベル付きの Entry を並べる（name → StringVar）"""

    def __init__(self, parent, title=None):
        self.frame = ttk.LabelFrame(parent, text=title) if title else ttk.Frame(parent)
        self.frame.pack(fill=tk.X, padx=6, pady=4)
        self.vars = {}
        self.row = 0

    def add(self, name, label, default, width=12, note=''):
        var = tk.StringVar(value=str(default))
        ttk.Label(self.frame, text=label).grid(row=self.row, column=0, sticky='w', padx=4, pady=2)
        e = ttk.Entry(self.frame, textvariable=var, width=width)
        e.grid(row=self.row, column=1, sticky='w', padx=4, pady=2)
        if note:
            ttk.Label(self.frame, text=note, foreground='#555').grid(row=self.row, column=2, sticky='w', padx=4)
        self.vars[name] = var
        self.row += 1
        return e

    def get(self, name):
        return self.vars[name].get()

    def set(self, name, value):
        if name in self.vars:
            self.vars[name].set(str(value))


# ==============================================================================
# 目標値の表（26自由度）
# ==============================================================================
class TargetTable:
    """DOF ごとに 初期値（Pi）・目標値（Pf）を入れる表。|ΔP| ≠ 0 の自由度が最適化対象になる（DOF 21・25 は固定）"""

    def __init__(self, parent, initial, target, on_change=None):
        self.on_change = on_change
        outer = ttk.Frame(parent)
        outer.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)
        canvas = tk.Canvas(outer, highlightthickness=0, height=300)
        vs = ttk.Scrollbar(outer, orient=tk.VERTICAL, command=canvas.yview)
        inner = ttk.Frame(canvas)
        inner.bind('<Configure>', lambda e: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.create_window((0, 0), window=inner, anchor='nw')
        canvas.configure(yscrollcommand=vs.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        vs.pack(side=tk.RIGHT, fill=tk.Y)
        heads = ["DOF", "board #", "FF番号", "初期値 Pi", "目標値 Pf", "ΔP", "最適化", "POT範囲"]
        for j, h in enumerate(heads):
            ttk.Label(inner, text=h, font=(gat.UI_FONT, 9, 'bold')).grid(row=0, column=j, padx=4, pady=2)
        self.ini, self.tgt, self.dp, self.flag = [], [], [], []
        for d in range(c.N_DOF):
            k, board, local = c.DOF_BOARD[d]
            r = d + 1
            ttk.Label(inner, text=f"{d}").grid(row=r, column=0)
            ttk.Label(inner, text=f"{board} #{local}").grid(row=r, column=1)
            ttk.Label(inner, text=str(c.FF_INDEX[d] + 1) if d in c.FF_INDEX else "－").grid(row=r, column=2)
            vi = tk.StringVar(value=f"{float(initial[d]):g}")
            vt = tk.StringVar(value=f"{float(target[d]):g}")
            ei = ttk.Entry(inner, textvariable=vi, width=8)
            et = ttk.Entry(inner, textvariable=vt, width=8)
            ei.grid(row=r, column=3, padx=2)
            et.grid(row=r, column=4, padx=2)
            if d in c.NO_FF_DOFS:
                et.state(['disabled'])                                  # FF の枠がなく制御もしていない自由度は動かさない
            lab_dp = ttk.Label(inner, text="", width=7)
            lab_dp.grid(row=r, column=5)
            lab_f = ttk.Label(inner, text="", width=6)
            lab_f.grid(row=r, column=6)
            lo, hi = gat.POT_RANGE[d]
            ttk.Label(inner, text=f"{lo}〜{hi}", foreground='#666').grid(row=r, column=7)
            for v in (vi, vt):
                v.trace_add('write', lambda *_: self.refresh())
            self.ini.append(vi)
            self.tgt.append(vt)
            self.dp.append(lab_dp)
            self.flag.append(lab_f)
        self.refresh()

    def _val(self, var):
        try:
            return float(var.get())
        except ValueError:
            return math.nan

    def refresh(self):
        for d in range(c.N_DOF):
            if d in c.NO_FF_DOFS and self.tgt[d].get() != self.ini[d].get():
                self.tgt[d].set(self.ini[d].get())
            a, b = self._val(self.ini[d]), self._val(self.tgt[d])
            dp = b - a
            self.dp[d].configure(text="" if not math.isfinite(dp) else f"{dp:+g}")
            self.flag[d].configure(text="○" if (math.isfinite(dp) and dp != 0 and d not in c.NO_FF_DOFS) else "")
        if self.on_change:
            self.on_change()

    def get(self):
        """(初期値26個, 目標値26個)。不正な値があれば ValueError"""
        ini, tgt = [], []
        for d in range(c.N_DOF):
            a = parse_float(self.ini[d].get(), f"DOF {d} の初期値")
            b = parse_float(self.tgt[d].get(), f"DOF {d} の目標値")
            lo, hi = gat.POT_RANGE[d]
            for v, name in ((a, '初期値'), (b, '目標値')):
                if not (lo <= v <= hi):
                    raise ValueError(f"DOF {d} の{name} {v:g} が POT 範囲 {lo}〜{hi} の外です")
            ini.append(a)
            tgt.append(b if d not in c.NO_FF_DOFS else a)
        return ini, tgt

    def set_initial(self, vals):
        for d, v in enumerate(vals):
            if v is not None and math.isfinite(float(v)):
                self.ini[d].set(f"{float(v):g}")

    def set_target(self, vals):
        for d, v in enumerate(vals):
            if v is not None and math.isfinite(float(v)):
                self.tgt[d].set(f"{float(v):g}")

    def opt_dofs(self):
        out = []
        for d in range(c.N_DOF):
            a, b = self._val(self.ini[d]), self._val(self.tgt[d])
            if d not in c.NO_FF_DOFS and math.isfinite(a) and math.isfinite(b) and a != b:
                out.append(d)
        return out


# 移植元: optimize_EL2_gui.py:394-404 @f8fba60
def setup_root(title):
    root = tk.Tk()
    root.title(title)
    gat.setup_japanese_font(root)
    return root


def ask_folder(var):
    path = filedialog.askdirectory(title="Select folder")
    if path:
        var.set(path)


# ==============================================================================
# フォルダの選択（Tk 標準の画面では、ダブルクリックしてもフォルダの中へ移動しないことがあるため、自前の画面にする）
# ==============================================================================
DOUBLE_CLICK_S = 0.5            # 同じ項目をこの時間内に2回押したらダブルクリックとみなす


def choose_folder(parent, title="Select folder", initial=None):
    """フォルダを選ぶ画面。一覧のフォルダをダブルクリック（または選んで Enter・「開く」）すると、そのフォルダの中へ移動する。
    「上へ」で1つ上の階層へ。「選択」で、一覧で選んでいるフォルダ（選んでいなければ今いるフォルダ）を選んで閉じる。
    ダブルクリックは Tk の判定に頼らず、同じ項目を DOUBLE_CLICK_S 秒以内に2回押したことで判定する（WSLg でも動くように）。
    戻り値は選んだフォルダの絶対パス（キャンセルなら None）"""
    import time as _time
    cur = os.path.abspath(os.path.expanduser(initial)) if initial else os.path.expanduser('~')
    while not os.path.isdir(cur) and os.path.dirname(cur) != cur:
        cur = os.path.dirname(cur)
    result = {'path': None}
    last = {'idx': None, 't': 0.0}
    win = tk.Toplevel(parent)
    win.title(title)
    win.transient(parent)
    win.geometry('560x440')
    path_var = tk.StringVar(value=cur)
    will_var = tk.StringVar()
    top = ttk.Frame(win)
    top.pack(fill=tk.X, padx=6, pady=4)
    ttk.Label(top, text="場所").pack(side=tk.LEFT)
    ent = ttk.Entry(top, textvariable=path_var)
    ent.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
    ttk.Label(win, text="フォルダをダブルクリックすると中へ移動します。選び終わったら「選択」を押してください",
              foreground='#555').pack(anchor='w', padx=6)
    mid = ttk.Frame(win)
    mid.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)
    lb = tk.Listbox(mid, activestyle='dotbox', exportselection=False)
    vs = ttk.Scrollbar(mid, orient=tk.VERTICAL, command=lb.yview)
    lb.configure(yscrollcommand=vs.set)
    lb.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    vs.pack(side=tk.RIGHT, fill=tk.Y)
    ttk.Label(win, textvariable=will_var, foreground='#004').pack(anchor='w', padx=6)

    def selected_path():
        sel = lb.curselection()
        return os.path.join(path_var.get(), lb.get(sel[0])) if sel else None

    def update_will(_e=None):
        will_var.set(f"選ぶフォルダ: {selected_path() or path_var.get()}")

    def listing(path):
        try:
            names = sorted((n for n in os.listdir(path) if os.path.isdir(os.path.join(path, n)) and not n.startswith('.')),
                           key=str.lower)
        except OSError as exc:
            messagebox.showerror("Folder", f"開けません: {exc}", parent=win)
            return False
        lb.delete(0, tk.END)
        for n in names:
            lb.insert(tk.END, n)
        path_var.set(path)
        last['idx'] = None
        update_will()
        return True

    def go(path):
        path = os.path.abspath(os.path.expanduser(path))
        if os.path.isdir(path):
            listing(path)
        else:
            messagebox.showerror("Folder", f"フォルダがありません: {path}", parent=win)

    def open_sel(_e=None):
        p = selected_path()
        go(p if p else path_var.get())

    def on_click(e):
        idx = lb.nearest(e.y)
        if idx < 0 or idx >= lb.size():
            return
        now = _time.monotonic()
        if last['idx'] == idx and now - last['t'] <= DOUBLE_CLICK_S:
            last['idx'] = None
            go(os.path.join(path_var.get(), lb.get(idx)))      # ダブルクリック → 中へ移動
            return
        last['idx'], last['t'] = idx, now
        update_will()

    def finish():
        path = selected_path() or path_var.get()
        if path and os.path.isdir(path):
            result['path'] = os.path.abspath(path)
            win.destroy()

    def new_folder():
        from tkinter import simpledialog
        name = simpledialog.askstring("New folder", "新しいフォルダの名前", parent=win)
        if name:
            p = os.path.join(path_var.get(), name)
            try:
                os.makedirs(p, exist_ok=True)
            except OSError as exc:
                messagebox.showerror("Folder", f"作れません: {exc}", parent=win)
                return
            listing(path_var.get())

    lb.bind('<ButtonRelease-1>', on_click)
    lb.bind('<<ListboxSelect>>', update_will)
    lb.bind('<Return>', open_sel)
    ent.bind('<Return>', lambda e: go(path_var.get()))
    bot = ttk.Frame(win)
    bot.pack(fill=tk.X, padx=6, pady=6)
    ttk.Button(bot, text="上へ", command=lambda: go(os.path.dirname(path_var.get()))).pack(side=tk.LEFT, padx=2)
    ttk.Button(bot, text="開く", command=open_sel).pack(side=tk.LEFT, padx=2)
    ttk.Button(bot, text="新しいフォルダ…", command=new_folder).pack(side=tk.LEFT, padx=2)
    ttk.Button(bot, text="キャンセル", command=win.destroy).pack(side=tk.RIGHT, padx=2)
    ttk.Button(bot, text="選択", command=finish).pack(side=tk.RIGHT, padx=2)
    listing(cur)
    win.grab_set()
    lb.focus_set()
    parent.wait_window(win)
    return result['path']


# ==============================================================================
# ログ（試行ごとに分けてコピーできるように、行も覚えておく）
# ==============================================================================
class RunLogger(Logger):
    """移植した Logger に「全行の保持」と「試行の区切り」を足したもの"""

    MARKERS = ('=== 試行', '=== 基準試行', '=== 確認試行')

    def __init__(self):
        super().__init__()
        self.lock = threading.Lock()
        self.lines = []
        self.sections = []          # (見出し, 開始行)

    def __call__(self, msg):
        super().__call__(msg)
        line = f"[{time.strftime('%H:%M:%S')}] {msg}"
        with self.lock:
            if str(msg).startswith(self.MARKERS):
                self.sections.append((str(msg).strip('= ').split('===')[0].strip(), len(self.lines)))
            self.lines.append(line)

    def section_text(self, k):
        with self.lock:
            if not (0 <= k < len(self.sections)):
                return ''
            a = self.sections[k][1]
            b = self.sections[k + 1][1] if k + 1 < len(self.sections) else len(self.lines)
            return '\n'.join(self.lines[a:b])


# ==============================================================================
# グラフ（matplotlib。GUI と画像で共通）
# ==============================================================================
JP_FONT_FILES = [
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', '/usr/share/fonts/truetype/fonts-japanese-gothic.ttf',
    '/mnt/c/Windows/Fonts/BIZ-UDGothicR.ttc', '/mnt/c/Windows/Fonts/meiryo.ttc', '/mnt/c/Windows/Fonts/YuGothR.ttc',
    '/mnt/c/Windows/Fonts/msgothic.ttc',
]
JP_MPL_NAMES = ('Noto Sans CJK JP', 'IPAexGothic', 'IPAGothic', 'TakaoGothic', 'BIZ UDGothic', 'Meiryo', 'Yu Gothic',
                'MS Gothic')


def setup_mpl_japanese():
    """matplotlib に日本語フォントを設定する。見つからなければ英字のラベルにする（戻り値 'ja' / 'en'）"""
    import matplotlib
    from matplotlib import font_manager as fm
    names = {f.name for f in fm.fontManager.ttflist}
    family = next((n for n in JP_MPL_NAMES if n in names), None)
    if family is None:
        for p in JP_FONT_FILES:
            if os.path.exists(p):
                try:
                    fm.fontManager.addfont(p)
                    family = fm.FontProperties(fname=p).get_name()
                    break
                except Exception:
                    continue
    matplotlib.rcParams['axes.unicode_minus'] = False
    if family is None:
        return 'en'
    matplotlib.rcParams['font.family'] = [family, 'DejaVu Sans']
    return 'ja'


TXT = {
    'ja': {'t': '時刻 t [s]（目標値のエコーが変わった配信 = 0）', 'pot': 'POT [count]', 'pwm': '入力 [PWM]',
           'trial': '試行番号', 'J': '累積誤差 J（対数）', 'ext_t': '極値の時刻 [s]', 'ext_y': '極値の値 [PWM]',
           'meas': '実測 POT', 'tgt': '目標軌道（固定）', 'ref': '基準試行（FF=0）', 'best': 'best',
           'adrc': 'ADRC = PWM − FF', 'ff': 'FF（送った値、ファームの計算）', 'thr': '閾値', 'fail': '失敗',
           'g1': 'POT と目標軌道', 'g2': '累積誤差の推移', 'g3': 'FF の極値の推移', 'g4': 'PWM と ADRC', 'g5': 'FF と極値'},
    'en': {'t': 't [s] (echo change = 0)', 'pot': 'POT [count]', 'pwm': 'input [PWM]', 'trial': 'trial',
           'J': 'J (log)', 'ext_t': 'extremum time [s]', 'ext_y': 'extremum value [PWM]', 'meas': 'measured POT',
           'tgt': 'target (fixed)', 'ref': 'reference (FF=0)', 'best': 'best', 'adrc': 'ADRC = PWM - FF',
           'ff': 'FF (sent, firmware calc.)', 'thr': 'threshold', 'fail': 'FAIL',
           'g1': 'POT and target', 'g2': 'cost history', 'g3': 'FF extrema', 'g4': 'PWM and ADRC', 'g5': 'FF'},
}
FAIL_COLORS = {'presend': '#999999', 'tabu': '#bbbb00', 'unsafe': '#d62728', 'comm': '#9467bd', 'sensor': '#8c564b',
               'eval': '#e377c2', 'stopped': '#7f7f7f', 'orphan': '#17becf', 'error': '#000000'}


def _title(ax, L, key, d, n=None):
    s = f"DOF {d}  {TXT[L][key]}" + (f"  （試行 {n}）" if (n is not None and L == 'ja') else
                                      (f"  (trial {n})" if n is not None else ''))
    ax.set_title(s, fontsize=9)


def plot_g1(ax, L, d, p, n):
    ax.clear()
    if p is None:
        _title(ax, L, 'g1', d, n)
        return
    ax.plot(p['t'], p['pot'], color='#1f77b4', lw=1.2, label=TXT[L]['meas'])
    ax.plot(p['t_win'], p['target'], color='#d62728', lw=1.2, ls='--', label=TXT[L]['tgt'])
    if p.get('ref') is not None:
        ax.plot(p['t'], p['ref'], color='#7f7f7f', lw=0.8, ls=':', label=TXT[L]['ref'])
    if p.get('best') is not None:
        ax.plot(p['t'], p['best'], color='#2ca02c', lw=0.8, alpha=0.8, label=TXT[L]['best'])
    ax.axvline(0.0, color='#aaa', lw=0.6)
    ax.axvline(p['T'], color='#aaa', lw=0.6, ls='--')
    ax.set_xlabel(TXT[L]['t'], fontsize=8)
    ax.set_ylabel(TXT[L]['pot'], fontsize=8)
    ax.legend(fontsize=7, loc='best')
    ax.grid(alpha=0.3)
    _title(ax, L, 'g1', d, n)


def plot_g2(ax, L, d, hist, J_thr):
    ax.clear()
    done = [h for h in hist if h['state'] == 'COMPLETE' and h.get('J') is not None]
    if done:
        ns = [h['n'] for h in done]
        ax.semilogy(ns, [h['J'] for h in done], 'o-', color='#1f77b4', ms=3, lw=1, label='J')
        ji = [h['J_i'].get(str(d), h['J_i'].get(d)) if h.get('J_i') else None for h in done]
        if any(v is not None for v in ji):
            ax.semilogy(ns, [v if v is not None else np.nan for v in ji], 's', color='#ff7f0e', ms=2,
                        label=f'J_{d}')
        best = np.minimum.accumulate([h['J'] for h in done])
        ax.semilogy(ns, best, color='#2ca02c', lw=1, label=TXT[L]['best'])
    if J_thr and J_thr > 0:
        ax.axhline(J_thr, color='#d62728', ls='--', lw=0.8, label=TXT[L]['thr'])
    import matplotlib.transforms as mtrans
    tr = mtrans.blended_transform_factory(ax.transData, ax.transAxes)
    for h in hist:
        if h['state'] == 'FAIL':
            ax.plot([h['n']], [0.03], marker='x', color=FAIL_COLORS.get(h.get('fail_class'), '#000'), ms=5,
                    transform=tr, clip_on=False)
    ax.set_xlabel(TXT[L]['trial'], fontsize=8)
    ax.set_ylabel(TXT[L]['J'], fontsize=8)
    if done or J_thr:
        ax.legend(fontsize=7, loc='upper right')
    ax.grid(alpha=0.3, which='both')
    _title(ax, L, 'g2', d)


def plot_g3(ax, ax2, L, d, hist):
    ax.clear()
    ax2.clear()
    pts = [(h['n'], h['ext'].get(str(d)) or h['ext'].get(d)) for h in hist
           if h['state'] == 'COMPLETE' and h.get('ext') and (h['ext'].get(str(d)) or h['ext'].get(d))]
    if pts:
        ns = [n for n, _ in pts]
        e = np.array([v for _, v in pts], dtype=float)
        ax.plot(ns, e[:, 0], 'o-', color='#1f77b4', ms=3, lw=0.8, label='t1')
        ax.plot(ns, e[:, 2], 's-', color='#17becf', ms=3, lw=0.8, label='t2')
        ax2.plot(ns, e[:, 1], 'o--', color='#d62728', ms=3, lw=0.8, label='y1')
        ax2.plot(ns, e[:, 3], 's--', color='#ff7f0e', ms=3, lw=0.8, label='y2')
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, fontsize=7, loc='best')
    ax.set_xlabel(TXT[L]['trial'], fontsize=8)
    ax.set_ylabel(TXT[L]['ext_t'], fontsize=8)
    ax2.set_ylabel(TXT[L]['ext_y'], fontsize=8)
    ax.grid(alpha=0.3)
    _title(ax, L, 'g3', d)


def plot_g4(ax, L, d, p, n):
    ax.clear()
    if p is not None:
        ax.plot(p['t'], p['pwm'], color='#1f77b4', lw=1.0, label='PWM')
        ax.plot(p['t'], p['adrc'], color='#2ca02c', lw=1.0, label=TXT[L]['adrc'])
        ax.axvline(0.0, color='#aaa', lw=0.6)
        ax.axvline(p['T'], color='#aaa', lw=0.6, ls='--')
        ax.legend(fontsize=7, loc='best')
    ax.set_xlabel(TXT[L]['t'], fontsize=8)
    ax.set_ylabel(TXT[L]['pwm'], fontsize=8)
    ax.grid(alpha=0.3)
    _title(ax, L, 'g4', d, n)


def plot_g5(ax, L, d, p, n):
    ax.clear()
    if p is not None:
        ax.plot(p['ff_t'], p['ff'], color='#9467bd', lw=1.2, label=TXT[L]['ff'])
        t1, y1, t2, y2 = p['ext']
        ax.plot([t1, t2], [y1, y2], 'o', color='#d62728', ms=5)
        for tt, yy, nm in ((t1, y1, '1'), (t2, y2, '2')):
            ax.annotate(f"({tt:.3f} s, {yy:+.1f})", (tt, yy), fontsize=7, textcoords='offset points', xytext=(4, 4))
        ax.axhline(0.0, color='#aaa', lw=0.6)
        ax.set_xlim(-0.02 * p['T'], p.get('t_eval', 1.5 * p['T']))
        ax.legend(fontsize=7, loc='best')
    ax.set_xlabel(TXT[L]['t'], fontsize=8)
    ax.set_ylabel('FF [PWM]', fontsize=8)
    ax.grid(alpha=0.3)
    _title(ax, L, 'g5', d, n)


def save_png(path, draw, twin=False):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    fig = Figure(figsize=(7.0, 4.2), dpi=110)
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(111)
    if twin:
        draw(ax, ax.twinx())
    else:
        draw(ax)
    fig.tight_layout()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path)


# ==============================================================================
# 別プロセスでの描画・計算
#   matplotlib の描画（特に日本語フォントの文字の大きさの計算）と目標軌道のフィットは重く、GUI と同じプロセスで行うと
#   Tk のメインスレッドが数秒止まり、実行中にタブを移動できなくなる。描画用・計算用のプロセスをそれぞれ1つ使う。
# ==============================================================================
_RENDER_LANG = 'en'


def _render_init():
    import matplotlib
    matplotlib.use('Agg')
    global _RENDER_LANG
    _RENDER_LANG = setup_mpl_japanese()


def render_preview(payload, hist, dofs, J_thr, width_px):
    """GUI のグラフタブ用の画像（PNG のバイト列）。1 DOF 分の5つを横1段、複数の DOF は縦に積む"""
    import io
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    L = _RENDER_LANG
    n = max(1, len(dofs))
    dpi = 80
    fig = Figure(figsize=(max(width_px, 600) / dpi, 3.0 * n), dpi=dpi)
    FigureCanvasAgg(fig)
    for i, d in enumerate(dofs):
        axs = [fig.add_subplot(n, 5, 5 * i + k + 1) for k in range(5)]
        pd = payload['dofs'].get(d) if payload else None
        nn = payload['n'] if payload else None
        plot_g1(axs[0], L, d, pd, nn)
        plot_g2(axs[1], L, d, hist, J_thr)
        plot_g3(axs[2], axs[2].twinx(), L, d, hist)
        plot_g4(axs[3], L, d, pd, nn)
        plot_g5(axs[4], L, d, pd, nn)
    fig.subplots_adjust(left=0.04, right=0.97, bottom=0.12 / n + 0.02, top=1.0 - 0.08 / n, wspace=0.45, hspace=0.55)
    buf = io.BytesIO()
    fig.savefig(buf, format='png')
    return buf.getvalue()


def render_trial_pngs(folder, n, payload):
    """試行ごとの画像（g1・g4・g5）を保存する"""
    L = _RENDER_LANG
    for d, p in payload['dofs'].items():
        base = os.path.join(folder, 'DOF', f'DOF{d}')
        save_png(os.path.join(base, f'D{d}_g1_pot_trial{n:03d}.png'), lambda ax: plot_g1(ax, L, d, p, n))
        save_png(os.path.join(base, f'D{d}_g4_pwm_adrc_trial{n:03d}.png'), lambda ax: plot_g4(ax, L, d, p, n))
        save_png(os.path.join(base, f'D{d}_g5_ff_trial{n:03d}.png'), lambda ax: plot_g5(ax, L, d, p, n))
    return n


def render_end_pngs(folder, dofs, hist, J_thr):
    """終了時の画像（g2・g3）を保存する"""
    L = _RENDER_LANG
    for d in dofs:
        base = os.path.join(folder, 'DOF', f'DOF{d}')
        save_png(os.path.join(base, f'D{d}_g2_cost.png'), lambda ax: plot_g2(ax, L, d, hist, J_thr))
        save_png(os.path.join(base, f'D{d}_g3_extrema.png'), lambda ax, ax2: plot_g3(ax, ax2, L, d, hist), twin=True)
    return len(dofs)


class Offload:
    """描画用（render）と計算用（compute）のプロセス。spawn で作る（ROS のスレッドを fork しないため）"""

    def __init__(self):
        import multiprocessing as mp
        from concurrent.futures import ProcessPoolExecutor
        ctx = mp.get_context('spawn')
        self.render = ProcessPoolExecutor(max_workers=1, mp_context=ctx, initializer=_render_init)
        self.compute = ProcessPoolExecutor(max_workers=1, mp_context=ctx)

    def close(self):
        for pool in (self.render, self.compute):
            try:
                pool.shutdown(wait=False, cancel_futures=True)
            except Exception:
                pass


# ==============================================================================
# 最適化の手順（ワーカースレッド）
# ==============================================================================
class OptimizationRun:
    def __init__(self, link, cfg, log, stop_event, events, lang='ja', resume_folder=None, offload=None):
        self.offload = offload                          # 別プロセスでの描画・計算（None ならこのスレッドで行う）
        self.render_futures = []
        self.link = link
        self.cfg = cfg
        self.log = log
        self.stop = stop_event
        self.events = events
        self.lang = lang
        self.resume_folder = resume_folder
        self.folder = None
        self.status = ('running', '')
        self.targets = {}
        self.study = None
        self.ffmap = None
        self.trials_csv = None
        self.hist = []
        self.best_mem = None
        self.ref_pot = {}
        self.best_pot = {}
        self.run_meta = {}
        self.safety = None
        self.dofs = list(cfg.get('dofs', []))

    # ---------------- 補助 ----------------
    def path(self, name):
        return os.path.join(self.folder, f"{os.path.basename(self.folder)}_{name}")

    def tpath(self, name):
        return os.path.join(self.folder, c.TRIALS_DIR, name)

    def emit(self, kind, data):
        self.events.put((kind, data))

    @staticmethod
    def cond_dict(cfg, params, target_hash):
        return {'map_version': c.MAP_VERSION, 'm': cfg['m'], 'sep': cfg['sep'], 'r_margin': 1.01,
                'r_range': [c.R_MIN, c.R_MAX], 'shrink': 1e-4, 'A_max': cfg['A_max'], 'A_min': c.A_MIN,
                'A_sign': 'signed A (|A| < 1 -> +-1)', 'rho': cfg['rho'], 'tabu_on': cfg['tabu_on'],
                'dofs': cfg['dofs'], 'T': cfg['T'], 'time_axis': '100 Hz publish (0.010 s)', 'eps': cfg['eps'],
                'eval_time': c.eval_time(cfg['T']), 'eval_rule': 'max(1.5T, 5 s)',
                'xi_cap': c.XI_CAP, 'wn_max': c.WN_MAX, 'initial': cfg['initial'], 'target': cfg['target'],
                'adrc': {str(d): (params[d].as_list() if params.get(d) is not None else None) for d in cfg['dofs']},
                'target_hash': target_hash}

    @staticmethod
    def check_resume(folder, link):
        """再開できるか（GUI で開始前に呼ぶ）。戻り値 (ok, メッセージ, cfg, ロックファイルの path or None)"""
        need = ['config.json', 'run.json', 'target.json', 'study.journal', 'ffmap_table.npz']
        miss = [n for n in need if not os.path.exists(os.path.join(folder, n))]
        if miss:
            return False, f"再開に必要なファイルがありません: {miss}", None, None
        cfg = load_json(os.path.join(folder, 'config.json'), {})
        meta = load_json(os.path.join(folder, 'run.json'), {})
        pid_path = os.path.join(folder, 'run.pid')
        if os.path.exists(pid_path):
            try:
                pid = int(open(pid_path).read().strip())
                if pid != os.getpid():
                    os.kill(pid, 0)
                    return False, f"別のプロセス（pid {pid}）がこのフォルダで実行中です", None, None
            except (ValueError, ProcessLookupError, PermissionError, OSError):
                pass
        if link is None:
            return False, "ROS2 に接続していません", None, None
        params = link.adrc_params()
        with open(os.path.join(folder, 'target.json'), encoding='utf-8') as f:
            tj = json.load(f)
        now = OptimizationRun.cond_dict(cfg, params, tj.get('target_hash'))
        if c.condition_hash(now) != meta.get('cond_hash'):
            old = meta.get('cond', {})
            diffs = [f"  {k}: 保存 {old.get(k)!r} / 現在 {now.get(k)!r}" for k in sorted(set(old) | set(now))
                     if old.get(k) != now.get(k)]
            return False, "条件が違うため再開できません（条件 hash が一致しません）:\n" + "\n".join(diffs[:20]), None, None
        lock = os.path.join(folder, 'study.journal.lock')
        return True, '', cfg, (lock if os.path.lexists(lock) else None)

    def _compute(self, fn, *args):
        """重い計算を計算用のプロセスで行う（GUI の応答を保つため）。プロセスがなければこのスレッドで行う"""
        if self.offload is None:
            return fn(*args)
        return self.offload.compute.submit(fn, *args).result()

    # ---------------- 本体 ----------------
    def run(self):
        log = self.log
        try:
            if self.lang == 'ja' and self.offload is None:
                self.lang = setup_mpl_japanese()            # 画像の日本語が □ にならないように（GUI なしで動かすときも）
            if self.resume_folder:
                self._setup_resume()
            else:
                self._setup_new()
            if self.study is not None and not self.stop.is_set():
                self._loop()
            elif self.stop.is_set() and self.status[0] == 'running':
                self.status = ('stopped', '停止ボタン')
        except c.StopRequested:
            self.status = ('stopped', '停止ボタン')
            log("停止しました")
        except Exception as exc:
            log(f"エラー: {exc}\n{traceback.format_exc()}")
            self.status = ('error', str(exc))
            try:
                c.safe_return(self.link, self.cfg['initial'], self.dofs, self.safety or c.SafetyConfig(), log)
            except Exception:
                pass
        finally:
            self._save_all()
            if self.folder:
                try:
                    c.write_status(self.folder, self.status[0], self.status[1],
                                   n_complete=(self.study.n_complete() if self.study else 0))
                    pid_path = os.path.join(self.folder, 'run.pid')
                    if os.path.exists(pid_path):
                        os.remove(pid_path)
                except Exception:
                    pass
            log(f"=== 終了（{self.status[0]}{'：' + self.status[1] if self.status[1] else ''}）。"
                f"保存先 {self.folder}。次の探索を開始・再開できます")
            self.emit('done', self.status)
            log.close_file()

    def _open_folder(self, folder):
        self.folder = folder
        os.makedirs(os.path.join(folder, c.TRIALS_DIR), exist_ok=True)
        self.log.open_file(os.path.join(folder, 'run.log'))
        with open(os.path.join(folder, 'run.pid'), 'w') as f:
            f.write(str(os.getpid()))
        c.write_status(folder, 'running')

    def _setup_new(self):
        cfg, log = self.cfg, self.log
        self.safety = c.SafetyConfig(**cfg['safety'])
        self._open_folder(c.make_run_folder(cfg['base'], 'optimize'))
        c.write_json_atomic(os.path.join(self.folder, 'config.json'), cfg)
        T = cfg['T']
        log(f"=== 開始: 保存先 {self.folder}")
        log(f"最適化対象 DOF {self.dofs}  T = {T} s, 評価区間 max(1.5T, 5 s) = {c.eval_time(T):.3f} s（{c.n_eval(T) + 1} 配信, 100 Hz）, "
            f"Sampler {cfg['sampler']}（seed {cfg['seed']}）, 最大試行数 {cfg['max_trials']}, "
            f"閾値 J ≤ {cfg['J_thr']:.3g}（RMS {cfg['rms_thr']:g}%）")
        log("FF の可行域の表を用意しています（初回だけ十数秒かかります）…")
        table = self._compute(c.ffmap_table_cached, cfg['m'], cfg['sep'])
        table.save(os.path.join(self.folder, 'ffmap_table.npz'))
        self.ffmap = c.FFMap(table, cfg['A_max'])
        log(f"FF の写像: s1 ∈ [{table.s1min:.4f}, {table.s1max:.4f}], 位置の余裕 m = {cfg['m']}, 最小間隔 sep = {cfg['sep']}")
        # 1. 基準試行（FF=0）
        refs = []
        ff0 = np.zeros((c.N_FF, 5))
        for r in range(cfg['n_ref']):
            if self.stop.is_set():
                raise c.StopRequested()
            log(f"=== 基準試行 {r + 1}/{cfg['n_ref']}（FF=0）===")
            try:
                m = c.measure_with_retries(
                    self.link, cfg['initial'], cfg['target'], self.dofs, ff0, T, cfg['wait'], self.safety, self.stop,
                    log, rest_ref=None,
                    on_attempt=lambda a, raw, ts, why, prm, prm_after, r=r: self._save_attempt(
                        f'ref_{r:02d}_a{a}.npz', raw, ff0, ts, why, prm, prm_after))
            except (c.UnsafeAbort, c.SensorFault) as exc:
                self._save_attempt(f'ref_{r:02d}_abort.npz', getattr(exc, 'raw', None), ff0, getattr(exc, 't_send', 0),
                                   str(exc), *self._exc_params(exc))
                log(f"基準試行（FF=0）で中断しました: {exc}。目標軌道を作れないので study を作らずに止めます")
                self.status = ('stopped', f'基準試行で中断: {exc}')
                return
            except c.CommFail as exc:
                log(f"基準試行の測定ができません: {exc}")
                self.status = ('stopped', f'基準試行の通信: {exc}')
                return
            except c.TrialIssue as exc:
                self._save_attempt(f'ref_{r:02d}_eval.npz', getattr(exc, 'raw', None), ff0, getattr(exc, 't_send', 0),
                                   exc.reason, *self._exc_params(exc))
                log(f"基準試行を解析できません: {exc.reason}")
                self.status = ('error', f'基準試行の解析: {exc.reason}')
                return
            c.save_trial_npz_ext(self.tpath(f'ref_{r:02d}.npz'), m.rec, self._npz_extra(m, ff0))
            self._log_ticks(m)
            refs.append(m)
            for d, s in m.series.items():
                log(f"  DOF {d}: P0 = {s.P0:.1f}, 欠損 {s.missing()}, エコーまで {s.echo_delay * 1000:.0f} ms")
        # 2. 目標軌道
        ys_by, vs_by = {}, {}
        for d in self.dofs:
            ys = [m.series[d].pot[m.series[d].win()] - m.series[d].Pf for m in refs]
            vs = [np.isfinite(y) for y in ys]
            ys_by[d], vs_by[d] = ys, vs
            log(f"  DOF {d}: 目標軌道を当てはめています…")
            sp = self._compute(c.fit_target_constrained, d, ys, vs, [m.series[d].P0 for m in refs], cfg['initial'][d],
                               cfg['target'][d], T, cfg['eps'])
            self.targets[d] = sp
            log(f"  DOF {d}: 目標軌道 T1 = {sp.T1:.4f} s, ωn = {sp.wn:.3f} rad/s, κ = {sp.kappa:.3f}, "
                f"拘束 {'あり' if sp.active else 'なし'}（{sp.method}）, フィット誤差 {sp.cost:.3g}, "
                f"自由フィットの完了 {sp.free_t_eps:.3f} s, 実測の完了 {sp.t_eps_meas}")
            for w in sp.warnings:
                log(f"  DOF {d}: 警告: {w}")
        J_refs = []
        for m in refs:
            J, J_i = c.evaluate(m.series, self.targets)
            J_refs.append(J)
            for d in self.dofs:
                self.targets[d].J_ref.append(J_i[d])
        J_noise, J_quant = c.noise_floor(list(self.targets.values()), ys_by, vs_by)
        sigma = float(np.std(J_refs, ddof=1)) if len(J_refs) >= 2 else float('nan')
        for sp in self.targets.values():
            sp.J_noise, sp.J_quant = J_noise, J_quant
        log(f"  基準試行の J = {[round(v, 6) for v in J_refs]}, ばらつき σ_J = {sigma:.3g}, "
            f"雑音の下限 J_noise = {J_noise:.3g}, 量子化の下限 = {J_quant:.3g}")
        floor = np.nanmax([J_noise, J_quant])
        if np.isfinite(floor) and cfg['J_thr'] < floor:
            log(f"  警告: 終了閾値 {cfg['J_thr']:.3g} が雑音の下限 {floor:.3g} より小さいので、閾値に届かない可能性があります")
        tdata = {str(d): sp.to_dict() for d, sp in self.targets.items()}
        target_hash = c.condition_hash(tdata)
        c.write_json_atomic(os.path.join(self.folder, 'target.json'), {'targets': tdata, 'target_hash': target_hash,
                                                                       'J_ref': J_refs, 'sigma_J': sigma})
        for d in self.dofs:
            self.ref_pot[d] = refs[0].series[d]
        # 3. study
        params = refs[-1].rec.params
        cond = self.cond_dict(cfg, params, target_hash)
        seeds = self._seed_candidates()
        past = self._past_candidates(refs[0])
        D = 4 * len(self.dofs)
        n_prior = len(seeds) + len(past)
        n_startup = n_prior + max(10, 2 * D)
        x0 = self.study_params_x0(seeds)
        self.study = c.FFStudy(self.folder, self.dofs, cfg['A_max'], cfg['sampler'], cfg['seed'], n_startup, self.ffmap,
                               cfg['rho'], cfg['tabu_on'], resume=False, x0=x0, log=log)
        self.run_meta = {'cond': cond, 'cond_hash': c.condition_hash(cond), 'target_hash': target_hash,
                         'sampler': cfg['sampler'], 'seed': cfg['seed'], 'seeds_used': [cfg['seed']],
                         'n_prior': n_prior, 'n_startup': n_startup, 'sampler_args': c._jsonable(self.study.sampler_args),
                         'J_ref': J_refs, 'sigma_J': sigma, 'J_noise': J_noise, 'J_quant': J_quant}
        c.write_json_atomic(os.path.join(self.folder, 'run.json'), self.run_meta)
        self.study.study.set_user_attr('cond_hash', self.run_meta['cond_hash'])
        self.study.study.set_user_attr('target_hash', target_hash)
        self.trials_csv = c.TrialsCsv(self.path('trials.csv'), c.trial_columns(self.dofs))
        for u, attrs in seeds:
            self.study.enqueue(u, attrs)
        for u, J, attrs, row in past:
            self.study.add_past(u, J, attrs)
            ft = self.study.trials()[-1]
            row.update({'trial': ft.number, 'state': 'COMPLETE', 'source': 'past'})
            self.trials_csv.append(row)
            self.hist.append({'n': ft.number, 'state': 'COMPLETE', 'J': J, 'J_i': attrs.get('J_i', {}),
                              'ext': attrs.get('ext_sent', {}), 'source': 'past'})
        log(f"study を作りました: 事前の候補 enqueue {len(seeds)} 個・過去の試行 add_trial {len(past)} 個, "
            f"n_startup = {n_startup}（= {n_prior} + max(10, 2×{D})）")
        c.write_kv_csv(self.path('conditions.csv'), self._conditions_rows())

    def study_params_x0(self, seeds):
        if seeds:
            u = seeds[0][0]
        else:
            u = {d: {'u1': 0.5, 'u2': 0.5, 'u3': 0.5, 'A': 0.25 * self.cfg['A_max']} for d in self.dofs}
        return {f'd{d}_{k}': float(u[d][k]) for d in self.dofs for k in ('u1', 'u2', 'u3', 'A')}

    def _setup_resume(self):
        log = self.log
        folder = self.resume_folder
        self.cfg = load_json(os.path.join(folder, 'config.json'), {})
        cfg = self.cfg
        self.dofs = list(cfg['dofs'])
        self.safety = c.SafetyConfig(**cfg['safety'])
        self._open_folder(folder)
        log(f"=== 再開: {folder}")
        self.run_meta = load_json(os.path.join(folder, 'run.json'), {})
        with open(os.path.join(folder, 'target.json'), encoding='utf-8') as f:
            tj = json.load(f)
        self.targets = {int(d): c.TargetSpec.from_dict(v) for d, v in tj['targets'].items()}
        now = self.cond_dict(cfg, self.link.adrc_params(), tj.get('target_hash'))
        if c.condition_hash(now) != self.run_meta.get('cond_hash'):
            raise RuntimeError("条件 hash が一致しないため再開できません")
        table = c.FFMapTable.load(os.path.join(folder, 'ffmap_table.npz'))
        self.ffmap = c.FFMap(table, cfg['A_max'])
        self.study = c.FFStudy(folder, self.dofs, cfg['A_max'], cfg['sampler'], cfg['seed'], self.run_meta['n_startup'],
                               self.ffmap, cfg['rho'], cfg['tabu_on'], resume=True,
                               x0=self.study_params_x0([]), log=log)
        self.run_meta.setdefault('seeds_used', []).append(self.study.seed)
        c.write_json_atomic(os.path.join(folder, 'run.json'), self.run_meta)
        n_orphan = self.study.handle_orphans()
        log(f"Sampler {cfg['sampler']} を seed {self.study.seed} で作り直しました。孤立した試行 {n_orphan} 個を FAIL にしました")
        self.trials_csv = c.TrialsCsv(self.path('trials.csv'), c.trial_columns(self.dofs))
        for t in self.study.trials():
            if t.state.name in ('COMPLETE', 'FAIL'):
                self.hist.append({'n': t.number, 'state': t.state.name, 'J': t.user_attrs.get('J'),
                                  'J_i': t.user_attrs.get('J_i', {}), 'ext': t.user_attrs.get('ext_sent', {}),
                                  'fail_class': t.user_attrs.get('fail_class')})
        try:
            rec = c.load_trial_npz(self.tpath('ref_00.npz'))
            ser, _info = c.extract_series(rec.raw, cfg['initial'], cfg['target'], self.dofs, rec.t_target, cfg['T'])
            self.ref_pot = ser
        except Exception:
            self.ref_pot = {}
        c.write_kv_csv(self.path('conditions.csv'), self._conditions_rows())

    # ---------------- 事前の候補（§6.5） ----------------
    def _seed_candidates(self):
        """(a)(b) 結果 CSV（既存と同じ168列）の最後の行の極値を enqueue する。可行域の外なら retract で戻してログに残す"""
        import csv as _csv
        out = []
        for path in self.cfg.get('seed_csv', []):
            try:
                with open(path, newline='', encoding='utf-8-sig') as f:
                    rows = list(_csv.DictReader(f))
            except OSError as exc:
                self.log(f"  初期候補 {path} を読めません: {exc}")
                continue
            if not rows:
                continue
            row = rows[-1]
            u = {}
            for d in self.dofs:
                n = c.FF_INDEX[d] + 1
                try:
                    Tn = float(row[f'T_{n}'])
                    t1, y1, t2, y2 = (float(row[f'{k}_{n}']) for k in ('t1', 'y1', 't2', 'y2'))
                except (KeyError, ValueError):
                    u = None
                    break
                p = np.array([t1 / Tn, t2 / Tn, y1, y2])
                if y1 == 0.0 or y2 == 0.0:
                    u = None
                    self.log(f"  初期候補 {os.path.basename(path)}: DOF {d} の FF がありません（使いません）")
                    break
                q = c.retract(1.0, p)
                if q is None:
                    u = None
                    self.log(f"  初期候補 {os.path.basename(path)}: DOF {d} を可行域へ戻せません（使いません）")
                    break
                if not np.allclose(q, p):
                    self.log(f"  初期候補 {os.path.basename(path)}: DOF {d} を retract で可行域へ戻しました "
                             f"{np.round(p, 4).tolist()} → {np.round(q, 4).tolist()}")
                s1, s2, y1, y2 = q
                amp = max(abs(y1), abs(y2))
                if amp > self.cfg['A_max']:
                    y1, y2 = y1 * self.cfg['A_max'] / amp, y2 * self.cfg['A_max'] / amp
                    self.log(f"  初期候補: DOF {d} の振幅を A_max = {self.cfg['A_max']:g} に縮めました")
                ud, why = self.ffmap.inverse(s1, y1, s2, y2, 1.0)
                if ud is None:
                    ud = self._inverse_clamped(s1, y1, s2, y2)
                    self.log(f"  初期候補: DOF {d} は写像の範囲の外（{why}）なので、範囲の端へ寄せました")
                u[d] = ud
            if u:
                out.append((u, {'source': 'seed_csv', 'seed_file': os.path.basename(path)}))
                self.log(f"  初期候補を enqueue します: {os.path.basename(path)}")
        return out

    def _inverse_clamped(self, s1, y1, s2, y2):
        tb = self.ffmap.table
        u1 = min(max((s1 - tb.s1min) / (tb.s1max - tb.s1min), 0.0), 1.0)
        lo, hi = tb.bounds(tb.s1min + u1 * (tb.s1max - tb.s1min))
        u2 = min(max((s2 - lo) / (hi - lo), 0.0), 1.0)
        s1c = tb.s1min + u1 * (tb.s1max - tb.s1min)
        s2c = lo + u2 * (hi - lo)
        rl, rh = c.r_interval_margin(np.array([s1c]), np.array([s2c]))
        r = -y2 / y1 if y1 != 0 else 1.0
        u3 = min(max((math.log(max(r, 1e-12)) - math.log(rl[0])) / (math.log(rh[0]) - math.log(rl[0])), 0.0), 1.0)
        A = math.copysign(min(max(abs(y1), abs(y2)), self.cfg['A_max']), y1 if y1 != 0 else 1.0)
        return {'u1': u1, 'u2': u2, 'u3': u3, 'A': A}

    def _past_candidates(self, ref):
        """(c) 過去の npz を add_trial する（往復誤差が許容内・retract 不要・対象の全 DOF がそろう試行だけ。J は計算し直す）"""
        out = []
        if not self.cfg.get('use_past'):
            return out
        cfg = self.cfg
        T = cfg['T']
        for path in cfg.get('past_npz', []):
            name = os.path.basename(path)
            try:
                rec = c.load_trial_npz(path)
            except Exception as exc:
                self.log(f"  過去の試行 {name} を読めません: {exc}")
                continue
            if abs(rec.T - T) > 1e-9 or not np.allclose(rec.initial, cfg['initial']) or \
                    not np.allclose(rec.target, cfg['target']):
                self.log(f"  過去の試行 {name}: T・初期姿勢・目標値が違うので使いません")
                continue
            u, ext, ok = {}, {}, True
            for d in self.dofs:
                co = np.asarray(rec.ff[c.FF_INDEX[d]], dtype=float)
                if c.extrema_ok(co, T) is None:
                    ok = False
                    break
                t1, y1, t2, y2 = c.extrema_of(co, T)
                ud, why = self.ffmap.inverse(t1, y1, t2, y2, T)
                if ud is None:
                    ok = False
                    break
                back = self.ffmap.design(ud['u1'], ud['u2'], ud['u3'], ud['A'], T)['coeffs']
                tt = np.arange(0, int(T * 1000)) / 1000.0
                if np.max(np.abs(c.ff_eval(back, T, tt) - c.ff_eval(co, T, tt))) > 0.01:   # 往復誤差の許容（計画書 §6.5）
                    ok = False
                    break
                u[d], ext[d] = ud, (t1, y1, t2, y2)
            if not ok:
                self.log(f"  過去の試行 {name}: FF が写像で表せない（往復誤差・可行域）ので使いません")
                continue
            try:
                ser, _info = c.extract_series(rec.raw, cfg['initial'], cfg['target'], self.dofs, rec.t_target, T)
                J, J_i = c.evaluate(ser, self.targets)
            except Exception as exc:
                self.log(f"  過去の試行 {name}: 解析できません（{exc}）")
                continue
            short = {d: s_.missing() for d, s_ in ser.items() if s_.missing() > self.safety.missing_tol}
            if short:
                self.log(f"  過去の試行 {name}: 評価区間 {c.eval_time(T):.2f} 秒の配信がそろっていない（欠け {short}。"
                         f"評価区間が短かったころの記録など）ので使いません")
                continue
            if not np.isfinite(J):
                continue
            attrs = {'past_file': name, 'J_i': {str(d): v for d, v in J_i.items()},
                     'ext_sent': {str(d): list(v) for d, v in ext.items()}}
            row = {'J': J, 'log10J': math.log10(max(J, 1e-12)), 'RMS_percent': c.rms_percent(J)}
            out.append((u, J, attrs, row))
            self.log(f"  過去の試行 {name} を学習データに入れます（J = {J:.4g}）")
        return out

    # ---------------- 探索のループ ----------------
    def _loop(self):
        cfg, log, st = self.cfg, self.log, self.study
        lim = cfg['limits']
        consec = {'norobot': 0, 'unsafe': 0, 'eval': 0}
        rest_ref = {d: self.targets[d].P0_ref for d in self.dofs}
        while True:
            if self.stop.is_set():
                self.status = ('stopped', '停止ボタン')
                break
            if st.n_complete() >= cfg['max_trials']:
                self.status = ('completed', f"最大試行数（COMPLETE {cfg['max_trials']} 回）")
                log(f"=== 最大試行数 {cfg['max_trials']} に達しました")
                break
            trial = st.ask()
            try:
                res = self._one_trial(trial, rest_ref)
            except Exception as exc:
                tb = traceback.format_exc()
                log(f"エラー（試行 {trial.number}）: {exc}\n{tb}")
                err = self.tpath(f'trial_{trial.number:03d}_error.txt')
                try:
                    with open(err, 'w', encoding='utf-8') as f:
                        f.write(tb)
                except OSError:
                    pass
                try:
                    st.tell_fail(trial, 'error', repr(exc), {'files': [os.path.basename(err)]})
                    self._row_fail(trial.number, 'error', repr(exc), trial.user_attrs.get('stage', ''), {})
                except Exception:
                    pass
                try:
                    c.safe_return(self.link, cfg['initial'], self.dofs, self.safety, log)
                except Exception:
                    pass
                self.status = ('error', str(exc))
                break
            kind = res['kind']
            if kind == 'complete':
                consec = {'norobot': 0, 'unsafe': 0, 'eval': 0}
                if res['J'] <= cfg['J_thr']:
                    conv, stop_reason = self._confirm(trial, res)
                    if conv:
                        self.status = ('completed', '収束（確認試行も閾値以下）')
                        break
                    if stop_reason:
                        self.status = stop_reason
                        break
                continue
            if kind in c.NO_ROBOT_CLASSES:
                consec['norobot'] += 1
                if consec['norobot'] >= lim['norobot']:
                    self.status = ('stopped', f"実機を動かさない失敗が {lim['norobot']} 回続きました")
                    log("=== " + self.status[1])
                    break
                continue
            consec['norobot'] = 0
            if kind == 'unsafe':
                consec['unsafe'] += 1
                if consec['unsafe'] >= lim['unsafe']:
                    self.status = ('stopped', f"FF が原因の安全中断が {lim['unsafe']} 回続きました")
                    log("=== " + self.status[1])
                    break
            elif kind == 'eval':
                consec['eval'] += 1
                if consec['eval'] >= lim['eval']:
                    self.status = ('stopped', f"評価の失敗が {lim['eval']} 回続きました")
                    log("=== " + self.status[1])
                    break
            elif kind == 'stopped':
                self.status = ('stopped', '停止ボタン')
                break
            elif kind in ('comm', 'sensor'):
                self.status = ('stopped', res.get('reason', kind))
                break

    def _log_ticks(self, m):
        for b, inf in m.info.items():
            if inf['n_drop'] > 0:
                self.log(f"  {b}: 配信の欠損 {inf['n_drop']} 個（大きな受信間隔 {inf['n_gap']} か所のうち、後の受信が"
                         f"追いつかなかった分）")

    def _npz_extra(self, m, ff_rows):
        ex = {'ff_sent_rows': np.asarray(ff_rows, dtype=np.float32).astype(float), 'time_axis': '100Hz'}
        for b, inf in m.info.items():
            ex[f'{b}_j0'] = inf['j0']
            ex[f'{b}_n_drop'] = inf['n_drop']
            ex[f'{b}_u_pc'] = inf['u_pc']
            ex[f'{b}_n_gap'] = inf['n_gap']
            ex[f'{b}_n_drop_phase'] = inf['n_drop_phase']
        ex['remeasure'] = json.dumps(m.attempts, ensure_ascii=False)
        return ex

    @staticmethod
    def _exc_params(exc):
        """中断の例外に付いている、試行開始時・終了時の ADRC パラメータ（measure_once が付ける）"""
        return getattr(exc, 'params', None), getattr(exc, 'params_after', None)

    def _save_attempt(self, name, raw, ff_rows, t_send, note, params=None, params_after=None):
        if raw is None:
            return None
        try:
            c.save_raw_attempt(self.tpath(name), raw, self.cfg['initial'], self.cfg['target'], ff_rows, self.cfg['T'],
                               t_send, note, params, params_after)
            return name
        except Exception as exc:
            self.log(f"  生データの保存に失敗しました（{name}）: {exc}")
            return None

    def _design(self, u):
        T = self.cfg['T']
        designs, checks = {}, {}
        ff_rows = np.zeros((c.N_FF, 5))
        for d in self.dofs:
            designs[d] = self.ffmap.design(u[d]['u1'], u[d]['u2'], u[d]['u3'], u[d]['A'], T)
            ff_rows[c.FF_INDEX[d]] = designs[d]['coeffs']
        for d in self.dofs:
            checks[d] = c.presend_check(ff_rows[c.FF_INDEX[d]], T, self.cfg['A_max'])
        sent = ff_rows.astype(np.float32).astype(float)
        return designs, checks, sent

    def _flat(self, u, designs, checks, sent):
        T = self.cfg['T']
        row, design_a, sent_a, ext_sent = {}, {}, {}, {}
        for d in self.dofs:
            p = f'd{d}_'
            ds, ch = designs[d], checks[d]
            co = sent[c.FF_INDEX[d]]
            ex = c.extrema_of(co, float(np.float32(T)))
            row.update({p + 'u1': u[d]['u1'], p + 'u2': u[d]['u2'], p + 'u3': u[d]['u3'], p + 'A': u[d]['A'],
                        p + 's1': ds['s1'], p + 's2': ds['s2'], p + 'r': ds['r'], p + 't1': ds['t1'], p + 'y1': ds['y1'],
                        p + 't2': ds['t2'], p + 'y2': ds['y2'], p + 'a': co[0], p + 'b': co[1], p + 'c': co[2],
                        p + 'd': co[3], p + 'e': co[4], p + 'T': float(np.float32(T)), p + 't1_sent': ex[0],
                        p + 'y1_sent': ex[1], p + 't2_sent': ex[2], p + 'y2_sent': ex[3], p + 'ff_max': ch['max_abs'],
                        p + 'ff_end': ch['end_value'], p + 'ff_step': ch['step_at_T']})
            design_a[str(d)] = {k: ds[k] for k in ('s1', 's2', 'r', 'y1', 'y2', 'A_eff', 't1', 't2')}
            sent_a[str(d)] = [float(v) for v in co] + [float(np.float32(T))]
            ext_sent[str(d)] = [float(v) for v in ex]
        return row, {'design': design_a, 'sent': sent_a, 'ext_sent': ext_sent}

    def _row_fail(self, n, fail_class, reason, stage, row, extra=None):
        r = dict(row)
        r.update({'trial': n, 'state': 'FAIL', 'fail_class': fail_class, 'fail_reason': reason, 'fail_stage': stage,
                  'finished_at': time.strftime('%Y-%m-%d %H:%M:%S')})
        r.update(extra or {})
        self._add_best_cols(r)
        self.trials_csv.append(r)
        self.hist.append({'n': n, 'state': 'FAIL', 'fail_class': fail_class})
        self.emit('hist', list(self.hist))

    def _add_best_cols(self, r):
        b = self.study.best()
        r['best_trial_so_far'] = b.number if b else None
        r['best_J_so_far'] = b.user_attrs['J'] if b else None

    def _fail(self, trial, fail_class, reason, row, attrs, files=(), extra_row=None):
        stage = trial.user_attrs.get('stage', '')
        self.study.tell_fail(trial, fail_class, reason, dict(attrs, files=list(files)))
        self.log(f"  試行 {trial.number}: 失敗（{fail_class}）: {reason}")
        self._row_fail(trial.number, fail_class, reason, stage, row, extra_row)
        if fail_class == 'eval':
            self.study.retry(self.study.trials()[trial.number], '評価の失敗')
        return {'kind': fail_class, 'reason': reason}

    def _one_trial(self, trial, rest_ref):
        cfg, log, st = self.cfg, self.log, self.study
        n = trial.number
        T = cfg['T']
        u = st.u_of(trial.params)
        src = trial.user_attrs.get('source', 'sampler')
        retry_of = trial.user_attrs.get('retry_of')
        log(f"=== 試行 {n}（{src}{'' if retry_of is None else f'、試行 {retry_of} の再投入'}）===")
        base_row = {'source': src, 'retry_of': retry_of, 'started_at': time.strftime('%Y-%m-%d %H:%M:%S')}
        try:
            designs, checks, sent = self._design(u)
        except c.MapError as exc:
            return self._fail(trial, 'presend', f"写像できません: {exc}", base_row, {})
        row, attrs = self._flat(u, designs, checks, sent)
        row.update(base_row)
        bad = {d: ch['reasons'] for d, ch in checks.items() if not ch['ok']}
        if bad:
            return self._fail(trial, 'presend', f"送信前の検証に不合格: {bad}", row, attrs)
        st.stage(trial, 'tabu_check')
        dist = st.tabu_hit(u)
        if dist is not None:
            return self._fail(trial, 'tabu', f"安全中断した点から距離 {dist:.3f} ≤ ρ = {st.rho:g}", row, attrs)
        for d in self.dofs:
            ds = designs[d]
            log(f"  DOF {d}: 極値 ({ds['t1']:.3f} s, {ds['y1']:+.1f}) / ({ds['t2']:.3f} s, {ds['y2']:+.1f}), "
                f"r = {ds['r']:.3g}, max|FF| = {checks[d]['max_abs']:.1f}, T 直前の段差 {checks[d]['step_at_T']:.2f}")
        st.stage(trial, 'measuring')
        files = []

        def on_attempt(a, raw, ts, why, prm, prm_after):
            nm = self._save_attempt(f'trial_{n:03d}_a{a}.npz', raw, sent, ts, why, prm, prm_after)
            if nm:
                files.append(nm)
            st.stage(trial, f'remeasure_{a + 1}')

        try:
            m = c.measure_with_retries(self.link, cfg['initial'], cfg['target'], self.dofs, sent, T, cfg['wait'],
                                       self.safety, self.stop, log, rest_ref, on_attempt)
        except c.StopRequested as exc:
            nm = self._save_attempt(f'trial_{n:03d}_stop.npz', getattr(exc, 'raw', None), sent,
                                    getattr(exc, 't_send', 0), '停止', *self._exc_params(exc))
            files += [nm] if nm else []
            return self._fail(trial, 'stopped', '停止ボタン', row, attrs, files)
        except c.UnsafeAbort as exc:
            nm = self._save_attempt(f'trial_{n:03d}_unsafe.npz', getattr(exc, 'raw', None), sent,
                                    getattr(exc, 't_send', 0), str(exc), *self._exc_params(exc))
            files += [nm] if nm else []
            return self._fail(trial, 'unsafe', str(exc), row, attrs, files)
        except c.SensorFault as exc:
            nm = self._save_attempt(f'trial_{n:03d}_sensor.npz', getattr(exc, 'raw', None), sent,
                                    getattr(exc, 't_send', 0), str(exc), *self._exc_params(exc))
            files += [nm] if nm else []
            return self._fail(trial, 'sensor', str(exc), row, attrs, files)
        except c.CommFail as exc:
            return self._fail(trial, 'comm', str(exc), row, dict(attrs, remeasure=getattr(exc, 'attempts', [])),
                              files, {'remeasure': len(getattr(exc, 'attempts', []))})
        except c.TrialIssue as exc:
            nm = self._save_attempt(f'trial_{n:03d}_eval.npz', getattr(exc, 'raw', None), sent,
                                    getattr(exc, 't_send', 0), exc.reason, *self._exc_params(exc))
            files += [nm] if nm else []
            return self._fail(trial, 'eval', exc.reason, row, attrs, files)
        st.stage(trial, 'evaluating')
        self._log_ticks(m)
        J, J_i = c.evaluate(m.series, self.targets)
        name = f'trial_{n:03d}.npz'
        extra = self._npz_extra(m, sent)
        extra.update({'J': J if np.isfinite(J) else np.nan})
        c.save_trial_npz_ext(self.tpath(name), m.rec, extra)
        files.append(name)
        if not np.isfinite(J):
            return self._fail(trial, 'eval', f"J を計算できません（J_i = {J_i}）", row, attrs, files)
        for d, s in m.series.items():
            row.update({f'd{d}_J_i': J_i[d], f'd{d}_P0': s.P0, f'd{d}_delta_P0': s.P0 - self.targets[d].P0_ref,
                        f'd{d}_missing': s.missing()})
        attrs.update({'J_i': {str(d): v for d, v in J_i.items()}, 'files': files, 'remeasure': m.attempts,
                      'missing': {str(d): s.missing() for d, s in m.series.items()}, 'rest_ok': m.rest_ok,
                      'P0': {str(d): s.P0 for d, s in m.series.items()}})
        st.tell_complete(trial, J, attrs)
        row.update({'trial': n, 'state': 'COMPLETE', 'J': J, 'log10J': math.log10(max(J, 1e-12)),
                    'RMS_percent': c.rms_percent(J), 'remeasure': len(m.attempts),
                    'missing_max': max(s.missing() for s in m.series.values()),
                    'finished_at': time.strftime('%Y-%m-%d %H:%M:%S')})
        self._after_complete(n, J, J_i, m, sent, designs, row, attrs)
        log(f"  試行 {n}: J = {J:.4g}（RMS {c.rms_percent(J):.2f}% of ΔP）  "
            + "  ".join(f"DOF{d}: {J_i[d]:.4g}" for d in self.dofs)
            + (f"  測り直し {len(m.attempts)} 回" if m.attempts else ''))
        return {'kind': 'complete', 'J': J, 'sent': sent, 'params': dict(trial.params)}

    def _after_complete(self, n, J, J_i, m, sent, designs, row, attrs):
        b = self.study.best()
        if b is not None and b.number == n:
            self.best_mem = {'n': n, 'J': J, 'rec': m.rec, 'series': m.series, 'info': m.info, 'sent': sent}
            self.best_pot = {d: s for d, s in m.series.items()}
            self.log(f"  best を更新しました（試行 {n}）")
        self._add_best_cols(row)
        self.trials_csv.append(row)
        self.hist.append({'n': n, 'state': 'COMPLETE', 'J': J, 'J_i': {str(d): v for d, v in J_i.items()},
                          'ext': attrs.get('ext_sent', {}), 'confirm_of': row.get('confirm_of')})
        payload = self._payload(n, m, sent, designs)
        self.emit('trial', payload)
        self.emit('hist', list(self.hist))
        self._save_trial_pngs(n, payload)

    def _confirm(self, trial, res):
        """確認試行（§5.5）。ask を通さずに同じ FF で直ちに測り、add_trial で記録する。戻り値 (収束, 止める理由)"""
        cfg, log, st = self.cfg, self.log, self.study
        n0 = trial.number
        log(f"=== 確認試行（試行 {n0} と同じ FF。J = {res['J']:.4g} ≤ 閾値 {cfg['J_thr']:.3g}）===")
        sent = res['sent']
        u = st.u_of(res['params'])
        designs, checks, _s = self._design(u)
        row, attrs = self._flat(u, designs, checks, sent)
        row.update({'source': 'confirm', 'confirm_of': n0, 'started_at': time.strftime('%Y-%m-%d %H:%M:%S')})
        rest_ref = {d: self.targets[d].P0_ref for d in self.dofs}
        fail = None
        try:
            m = c.measure_with_retries(self.link, cfg['initial'], cfg['target'], self.dofs, sent, cfg['T'], cfg['wait'],
                                       self.safety, self.stop, log, rest_ref)
            J2, J_i2 = c.evaluate(m.series, self.targets)
            if not np.isfinite(J2):
                fail = ('eval', 'J を計算できません', None)
        except c.StopRequested:
            fail = ('stopped', '停止ボタン', ('stopped', '停止ボタン'))
        except c.UnsafeAbort as exc:
            fail = ('unsafe', str(exc), None)
        except c.SensorFault as exc:
            fail = ('sensor', str(exc), ('stopped', str(exc)))
        except c.CommFail as exc:
            fail = ('comm', str(exc), ('stopped', str(exc)))
        except c.TrialIssue as exc:
            fail = ('eval', exc.reason, None)
        if fail is not None:
            ft = st.add_confirm(dict(res['params']), attrs=dict(attrs, confirm_of=n0), fail_class=fail[0],
                                reason=fail[1])
            if fail[0] == 'unsafe':
                st.add_tabu(u)
            log(f"  確認試行（試行 {ft.number}）は失敗しました（{fail[0]}）: {fail[1]}。確認は不成立として探索を続けます")
            self._row_fail(ft.number, fail[0], fail[1], 'confirm', row, {'confirm_of': n0})
            return False, fail[2]
        ft = st.add_confirm(dict(res['params']), J2, attrs=dict(attrs, confirm_of=n0,
                                                                J_i={str(d): v for d, v in J_i2.items()}))
        name = f'trial_{ft.number:03d}.npz'
        c.save_trial_npz_ext(self.tpath(name), m.rec, dict(self._npz_extra(m, sent), J=J2, confirm_of=n0))
        for d, s in m.series.items():
            row.update({f'd{d}_J_i': J_i2[d], f'd{d}_P0': s.P0, f'd{d}_delta_P0': s.P0 - self.targets[d].P0_ref,
                        f'd{d}_missing': s.missing()})
        row.update({'trial': ft.number, 'state': 'COMPLETE', 'J': J2, 'log10J': math.log10(max(J2, 1e-12)),
                    'RMS_percent': c.rms_percent(J2), 'remeasure': len(m.attempts),
                    'missing_max': max(s.missing() for s in m.series.values()),
                    'finished_at': time.strftime('%Y-%m-%d %H:%M:%S')})
        self._after_complete(ft.number, J2, J_i2, m, sent, designs, row, attrs)
        mean = 0.5 * (res['J'] + J2)
        log(f"  確認試行（試行 {ft.number}）: J = {J2:.4g}。2回の平均 {mean:.4g}"
            f"{' ≤ ' if mean <= cfg['J_thr'] else ' > '}閾値 {cfg['J_thr']:.3g}")
        return mean <= cfg['J_thr'], None

    # ---------------- グラフ ----------------
    def _payload(self, n, m, sent, designs):
        T = self.cfg['T']
        t_ms = np.arange(0, int(math.ceil(1000.0 * c.eval_time(T))) + 1)
        out = {'n': n, 'T': T, 'dofs': {}}
        t_eval = c.eval_time(T)
        for d, s in m.series.items():
            co = sent[c.FF_INDEX[d]]
            ffv = c.ff_eval(co, T, s.t - c.TICK_DT)
            ref = self.ref_pot.get(d)
            best = self.best_pot.get(d)
            ex = c.extrema_of(co, float(np.float32(T)))
            out['dofs'][d] = {
                'T': T, 't_eval': t_eval, 't': s.t, 'pot': s.pot, 't_win': s.t[s.win()], 'target': self.targets[d].seq(),
                'ref': ref.pot if ref is not None and len(ref.pot) == len(s.pot) else None,
                'best': best.pot if best is not None and len(best.pot) == len(s.pot) else None,
                'pwm': s.pwm, 'adrc': s.pwm - ffv, 'ff_t': t_ms / 1000.0,
                'ff': c.firmware_ff(co, T, t_ms).astype(float), 'ext': ex}
        return out

    def _check_render(self, wait=0.0):
        """終わった描画の失敗をログに出す。wait > 0 なら残りが終わるまで待つ"""
        from concurrent.futures import wait as fwait
        if wait > 0 and self.render_futures:
            fwait(self.render_futures, timeout=wait)
        keep = []
        for f in self.render_futures:
            if not f.done():
                keep.append(f)
            elif f.exception() is not None:
                self.log(f"  グラフの保存に失敗しました: {f.exception()}")
        self.render_futures = keep

    def _save_trial_pngs(self, n, payload):
        if self.offload is not None:
            self._check_render()
            self.render_futures.append(self.offload.render.submit(render_trial_pngs, self.folder, n, payload))
            return
        L = self.lang
        try:
            for d, p in payload['dofs'].items():
                base = os.path.join(self.folder, 'DOF', f'DOF{d}')
                save_png(os.path.join(base, f'D{d}_g1_pot_trial{n:03d}.png'), lambda ax: plot_g1(ax, L, d, p, n))
                save_png(os.path.join(base, f'D{d}_g4_pwm_adrc_trial{n:03d}.png'), lambda ax: plot_g4(ax, L, d, p, n))
                save_png(os.path.join(base, f'D{d}_g5_ff_trial{n:03d}.png'), lambda ax: plot_g5(ax, L, d, p, n))
        except Exception as exc:
            self.log(f"  グラフの保存に失敗しました: {exc}")

    # ---------------- 保存 ----------------
    def _conditions_rows(self):
        cfg = self.cfg
        import scipy
        import optuna
        try:
            import cmaes
            cm = cmaes.__version__
        except Exception:
            cm = ''
        import matplotlib
        meta = self.run_meta
        rows = [
            ('実行', '状態', f"{self.status[0]} {self.status[1]}", ''),
            ('実行', 'プログラム', 'optimize_optuna_ff_ROS2.py', '計算は optuna_ff_common.py'),
            ('実行', 'ライブラリ', f"python {sys.version.split()[0]}, numpy {np.__version__}, scipy {scipy.__version__}, "
                               f"optuna {optuna.__version__}, cmaes {cm}, matplotlib {matplotlib.__version__}", ''),
            ('時間', 'T', cfg['T'], '動作完了時間 [s]（FF は 0 ≤ t < T）'),
            ('時間', '評価区間', c.eval_time(cfg['T']), 'max(1.5T, 5 s) [s]'),
            ('時間', '時間軸', '100 Hz 配信（0.010 s）', '配信番号 × 10 ms。制御周期は使わない'),
            ('時間', '評価区間の配信数 N_e+1', c.n_eval(cfg['T']) + 1, 'j − j0 = 0..N_e'),
            ('時間', '待機時間', cfg['wait'], '[s]（実際は max(待機時間, T+0.5)）'),
            ('写像', '版', c.MAP_VERSION, '案4（s1, s2 は正確な境界、r は閉形式、符号付き A）'),
            ('写像', '位置の余裕 m / 最小間隔 sep', f"{cfg['m']} / {cfg['sep']}", 's = t/T'),
            ('写像', 'r の余裕 / 範囲', f"1.01 / [{c.R_MIN:g}, {c.R_MAX:g}]", 'r = −y2/y1'),
            ('写像', 'A_max / |A| の下限', f"{cfg['A_max']} / {c.A_MIN}", '[PWM]'),
            ('目標軌道', 'ε', cfg['eps'], '完了: t ≥ T で |y| ≤ ε|ΔP|'),
            ('目標軌道', 'ξ の上限 / ωn の上限', f"{c.XI_CAP:.6g} / {c.WN_MAX:g}", 'ξ ≤ ln 10'),
            ('評価', '累積誤差', 'J_i = mean((POT − Target)/ΔP)²', 'J = 対象自由度の平均'),
            ('評価', '終了閾値 J', cfg['J_thr'], f"RMS {cfg['rms_thr']}%"),
            ('評価', '基準試行の回数', cfg['n_ref'], ''),
            ('評価', '基準試行の J / σ_J', f"{meta.get('J_ref')} / {meta.get('sigma_J')}", ''),
            ('評価', 'J_noise / 量子化の下限', f"{meta.get('J_noise')} / {meta.get('J_quant')}", ''),
            ('Optuna', 'Sampler', cfg['sampler'], json.dumps(meta.get('sampler_args'), ensure_ascii=False)),
            ('Optuna', 'seed（開始時・再開時）', cfg['seed'], str(meta.get('seeds_used'))),
            ('Optuna', 'n_prior / n_startup', f"{meta.get('n_prior')} / {meta.get('n_startup')}",
             'n_startup = n_prior + max(10, 2D)'),
            ('Optuna', '最大試行数', cfg['max_trials'], 'COMPLETE の数'),
            ('Optuna', 'study', f"{c.STUDY_NAME} / study.journal", 'JournalStorage(JournalFileBackend)'),
            ('Optuna', '近傍棄却', f"{'有効' if cfg['tabu_on'] else '無効'} ρ = {cfg['rho']}", ''),
            ('Optuna', '初期候補（結果 CSV）', '; '.join(cfg.get('seed_csv', [])), ''),
            ('Optuna', '過去の npz', '; '.join(cfg.get('past_npz', [])) if cfg.get('use_past') else '使わない', ''),
            ('Optuna', '条件 hash', meta.get('cond_hash'), ''),
            ('Optuna', 'target hash', meta.get('target_hash'), ''),
        ]
        for k, v in cfg['safety'].items():
            if k != 'joint':
                rows.append(('安全', k, v, ''))
        for k, v in cfg['limits'].items():
            rows.append(('安全', f'連続失敗の上限 {k}', v, ''))
        for d in self.dofs:
            sp = self.targets.get(d)
            jr = cfg['safety']['joint'][d] if cfg['safety'].get('joint') else POT_RANGE[d]
            rows.append(('目標値', f'DOF {d}', f"{cfg['initial'][d]:g} -> {cfg['target'][d]:g}", f'可動範囲 {jr}'))
            if sp is not None:
                rows.append(('目標軌道', f'DOF {d}', f"T1={sp.T1:.6g} wn={sp.wn:.6g} kappa={sp.kappa:.6g} xi={sp.xi:.6g} "
                                                     f"active={sp.active} method={sp.method} cost={sp.cost:.6g}",
                             f"y0_ref={sp.y0_ref:.6g} P0_ref={sp.P0_ref:.6g} eps_eff={sp.eps_eff:.6g} "
                             f"t_eps_meas={sp.t_eps_meas}"))
            adrc = (meta.get('cond') or {}).get('adrc', {}).get(str(d))
            if adrc:
                rows.append(('ADRC', f'DOF {d}', f"kp={adrc[0]:g} kd={adrc[1]:g} ic={adrc[2]:g} lambda0={adrc[3]:g}",
                             '実機から受信'))
        return rows

    def _save_all(self):
        if self.folder is None:
            return
        log = self.log
        try:
            if self.cfg.get('dofs') and self.run_meta:
                c.write_kv_csv(self.path('conditions.csv'), self._conditions_rows())
            if self.study is None:
                return
            b = self.study.best()
            if b is not None:
                if self.best_mem is not None and self.best_mem['n'] == b.number:
                    rec, series, info, sent = (self.best_mem[k] for k in ('rec', 'series', 'info', 'sent'))
                else:
                    path = self.tpath(f'trial_{b.number:03d}.npz')
                    rec = c.load_trial_npz(path)
                    with np.load(path, allow_pickle=False) as z:
                        sent = z['x_ff_sent_rows'].copy() if 'x_ff_sent_rows' in z.files else rec.ff
                    series, info = c.extract_series(rec.raw, self.cfg['initial'], self.cfg['target'], self.dofs,
                                                    rec.t_target, self.cfg['T'])
                c.write_results_csv(self.path('results.csv'), self.cfg['initial'], self.cfg['target'], self.cfg['T'], sent)
                c.write_trial_timeseries(self.path('best_trial.csv'), rec, c.ff_times_for_csv(rec, info))
                c.write_grid_csv(self.path('best_trial_grid.csv'), series, self.targets, sent, self.cfg['T'])
                log(f"保存: best は試行 {b.number}（J = {b.user_attrs['J']:.4g}）")
            else:
                log("COMPLETE の試行がないので、結果 CSV は書きません")
            if self.offload is not None:
                self.render_futures.append(self.offload.render.submit(render_end_pngs, self.folder, list(self.dofs),
                                                                      list(self.hist), self.cfg['J_thr']))
                self._check_render(wait=300.0)
            else:
                L = self.lang
                for d in self.dofs:
                    base = os.path.join(self.folder, 'DOF', f'DOF{d}')
                    save_png(os.path.join(base, f'D{d}_g2_cost.png'),
                             lambda ax: plot_g2(ax, L, d, self.hist, self.cfg['J_thr']))
                    save_png(os.path.join(base, f'D{d}_g3_extrema.png'),
                             lambda ax, ax2: plot_g3(ax, ax2, L, d, self.hist), twin=True)
            log(f"保存先: {self.folder}")
        except Exception as exc:
            log(f"保存に失敗しました: {exc}\n{traceback.format_exc()}")


# ==============================================================================
# GUI
# ==============================================================================
def _fmt(v):
    """数値なら %g、文字列ならそのまま（設定ファイルの値を欄に戻すとき用）"""
    try:
        return f"{float(v):g}"
    except (TypeError, ValueError):
        return str(v)


class JointTable:
    """関節の可動範囲（DOF ごと。既定は POT_RANGE）"""

    def __init__(self, parent, values):
        outer = ttk.LabelFrame(parent, text="関節の可動範囲 [count]（外に出たら FF が原因の安全中断。既定は GUI_android_tk の POT_RANGE）")
        outer.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)
        canvas = tk.Canvas(outer, highlightthickness=0, height=220)
        vs = ttk.Scrollbar(outer, orient=tk.VERTICAL, command=canvas.yview)
        inner = ttk.Frame(canvas)
        inner.bind('<Configure>', lambda e: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.create_window((0, 0), window=inner, anchor='nw')
        canvas.configure(yscrollcommand=vs.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        vs.pack(side=tk.RIGHT, fill=tk.Y)
        self.vars = []
        for d in range(c.N_DOF):
            lo, hi = values[d]
            ttk.Label(inner, text=f"DOF {d}").grid(row=d, column=0, padx=4)
            a, b = tk.StringVar(value=_fmt(lo)), tk.StringVar(value=_fmt(hi))
            ttk.Entry(inner, textvariable=a, width=7).grid(row=d, column=1, padx=2)
            ttk.Entry(inner, textvariable=b, width=7).grid(row=d, column=2, padx=2)
            ttk.Label(inner, text=f"（既定 {POT_RANGE[d][0]}〜{POT_RANGE[d][1]}）", foreground='#666').grid(row=d, column=3)
            self.vars.append((a, b))

    def get(self):
        out = []
        for d, (a, b) in enumerate(self.vars):
            lo = parse_float(a.get(), f"DOF {d} の可動範囲の下限")
            hi = parse_float(b.get(), f"DOF {d} の可動範囲の上限")
            if not lo < hi:
                raise ValueError(f"DOF {d} の可動範囲の下限 {lo:g} が上限 {hi:g} 以上です")
            out.append([lo, hi])
        return out

    def set(self, values):
        for (a, b), (lo, hi) in zip(self.vars, values):
            a.set(f"{lo:g}")
            b.set(f"{hi:g}")


class FileList:
    """ファイルの一覧（追加・削除）"""

    def __init__(self, parent, title, items, patterns):
        self.patterns = patterns
        f = ttk.LabelFrame(parent, text=title)
        f.pack(fill=tk.X, padx=6, pady=4)
        self.lb = tk.Listbox(f, height=3)
        self.lb.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4, pady=2)
        bar = ttk.Frame(f)
        bar.pack(side=tk.LEFT)
        ttk.Button(bar, text="追加…", command=self.add).pack(fill=tk.X)
        ttk.Button(bar, text="削除", command=self.remove).pack(fill=tk.X)
        for it in items:
            self.lb.insert(tk.END, it)

    def add(self):
        for p in filedialog.askopenfilenames(title="Select files", filetypes=self.patterns):
            self.lb.insert(tk.END, p)

    def remove(self):
        for i in reversed(self.lb.curselection()):
            self.lb.delete(i)

    def get(self):
        return list(self.lb.get(0, tk.END))

    def set(self, items):
        self.lb.delete(0, tk.END)
        for it in items:
            self.lb.insert(tk.END, it)


class App:
    def __init__(self):
        self.root = setup_root("Optuna FF optimizer")
        self.lang = 'ja'
        self.offload = Offload()                        # 描画と重い計算は別プロセス（実行中もタブを移動できるように）
        self.graph_dirty = False
        self.graph_future = None
        self.g_image = None
        self.g_width = 0
        self.settings = self._load_settings()
        self._saved_snapshot = None
        self.log = RunLogger()
        self.events = queue.Queue()
        self.link = None
        self.worker = None
        self.run_obj = None
        self.stop_event = threading.Event()
        self.closing = False
        self.hist = []
        self.last_payload = None
        self.fig_dofs = []
        self.n_sections = 0
        try:
            self.link = c.OptunaLink('optuna_ff_optimizer')
        except Exception as exc:
            messagebox.showerror("ROS2", f"ROS2 に接続できません:\n{exc}")
        self.torch_ok, self.torch_why = c.torch_available()
        s = self.settings
        self.nb = ttk.Notebook(self.root)
        self.nb.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text="基本")
        self.basic = Fields(tab, "動作・終了条件・保存")
        self.basic.add('T', "動作完了時間 T [s]", s.get('T', 1.5), note="測定・評価は max(1.5T, 5 秒)（100 Hz 配信の時間軸）")
        self.basic.add('dofs', "最適化対象 DOF（カンマ区切り。空欄なら |ΔP| ≠ 0 の全自由度）", s.get('dofs', ''), width=20)
        self.basic.add('wait', "初期姿勢に戻った後の待機時間 [s]", s.get('wait', 5.0), note="実際は max(待機時間, T+0.5)")
        self.basic.add('n_ref', "基準試行（FF=0）の回数", s.get('n_ref', 2), note="2回以上で試行間のばらつきを推定する")
        self.basic.add('max_trials', "最大試行数（COMPLETE の数）", s.get('max_trials', 40))
        self.basic.add('rms_thr', "終了閾値 RMS [% of ΔP]", s.get('rms_thr', 1.0), note="J_thr = (RMS/100)²（1% → 1e−4）")
        self.basic.add('eps', "完了の許容 ε [% of ΔP]", s.get('eps', 2.0), note="目標軌道は t ≥ T で |y| ≤ ε|ΔP|")
        self.basic.add('A_max', "FF の振幅の上限 A_max [PWM]", s.get('A_max', 255.0), note="≤ 255。最初の実機確認では小さく")
        self.basic.add('base', "保存先フォルダ", s.get('base', os.path.expanduser('~/ArmRobot/optimize_data')), width=50)
        ttk.Button(self.basic.frame, text="参照…", command=self.ask_base).grid(
            row=self.basic.row - 1, column=2, sticky='w')

        tab_s = ttk.Frame(self.nb)
        self.nb.add(tab_s, text="探索")
        self.search = Fields(tab_s, "Optuna")
        fr = self.search.frame
        ttk.Label(fr, text="Sampler").grid(row=self.search.row, column=0, sticky='w', padx=4)
        names = [n for n in c.SAMPLERS if n != 'GP' or self.torch_ok]
        self.sampler_var = tk.StringVar(value=s.get('sampler') if s.get('sampler') in names else 'TPE')
        self.sampler_cb = ttk.Combobox(fr, textvariable=self.sampler_var, values=names, state='readonly', width=10)
        self.sampler_cb.grid(row=self.search.row, column=1, sticky='w', padx=4)
        ttk.Label(fr, text=("TPE（multivariate）が既定。" + ("" if self.torch_ok else f"GP は選べません: {self.torch_why}")),
                  foreground='#555').grid(row=self.search.row, column=2, sticky='w')
        self.search.row += 1
        self.search.add('seed', "乱数の seed", s.get('seed', 1))
        self.search.add('m', "極値の位置の余裕 m（s = t/T）", s.get('m', 0.02))
        self.search.add('sep', "極値の最小間隔 sep（s）", s.get('sep', 0.02))
        self.search.add('rho', "近傍棄却の距離 ρ（正規化した u の空間）", s.get('rho', 0.05))
        self.tabu_var = tk.BooleanVar(value=s.get('tabu_on', True))
        ttk.Checkbutton(fr, text="近傍棄却（FF が原因で安全中断した点の近くは実機に送らない）", variable=self.tabu_var).grid(
            row=self.search.row, column=0, columnspan=3, sticky='w', padx=4)
        self.search.row += 1
        self.seed_files = FileList(tab_s, "初期候補（(a)(b) 結果 CSV。最後の行の極値を enqueue する）",
                                   s.get('seed_csv', []), [('CSV', '*.csv')])
        self.use_past_var = tk.BooleanVar(value=s.get('use_past', False))
        ttk.Checkbutton(tab_s, text="(c) 過去の試行（npz）を学習データに入れる（T・初期姿勢・目標値が同じで、FF を写像で表せるものだけ）",
                        variable=self.use_past_var).pack(anchor='w', padx=8)
        self.past_files = FileList(tab_s, "過去の試行（npz）", s.get('past_npz', []), [('npz', '*.npz')])

        tab_a = ttk.Frame(self.nb)
        self.nb.add(tab_a, text="安全")
        sf = s.get('safety', {})
        self.safe = Fields(tab_a, "安全・測り直し")
        self.safe.add('sensor_lo', "センサ値の有効範囲 下限 [count]", sf.get('sensor_lo', 2.0), note="外に出たら故障とみなし study を止める")
        self.safe.add('sensor_hi', "センサ値の有効範囲 上限 [count]", sf.get('sensor_hi', 1021.0))
        self.safe.add('dev_const', "偏差の上限 D [count]", sf.get('dev_const', 50.0), note="|P − Pf| > 1.5|ΔP| + D で安全中断")
        self.safe.add('rest_tol', "静止判定の許容ずれ [count]", sf.get('rest_tol', 3.0), note="基準は基準試行の静止位置 P0_ref")
        self.safe.add('still_wait', "安全な戻しで静止を待つ上限 [s]", sf.get('still_wait', 5.0))
        self.safe.add('missing_tol', "評価区間で欠損してよい配信の数", sf.get('missing_tol', 0), note="超えたら測り直す")
        self.safe.add('remeasure_max', "測り直しの上限（1試行あたり）", sf.get('remeasure_max', 3))
        lim = s.get('limits', {})
        self.safe.add('lim_norobot', "連続失敗の上限：実機を動かさない失敗", lim.get('norobot', 20))
        self.safe.add('lim_unsafe', "連続失敗の上限：FF が原因の安全中断", lim.get('unsafe', 3))
        self.safe.add('lim_eval', "連続失敗の上限：評価の失敗", lim.get('eval', 3))
        self.joint = JointTable(tab_a, sf.get('joint') or POT_RANGE)

        tab_t = ttk.Frame(self.nb)
        self.nb.add(tab_t, text="目標値")
        init0, presets = c.load_gui_poses()
        bar = ttk.Frame(tab_t)
        bar.pack(fill=tk.X, padx=6, pady=4)
        self.table = TargetTable(tab_t, s.get('initial') or init0, s.get('target') or list(s.get('initial') or init0))
        ttk.Button(bar, text="実機の現在の目標値 → 初期値", command=self.from_robot).pack(side=tk.LEFT, padx=2)
        ttk.Button(bar, text="GUI_android_tk の初期姿勢 → 初期値",
                   command=lambda: self.table.set_initial(c.load_gui_poses()[0])).pack(side=tk.LEFT, padx=2)
        for k in range(c.NUM_PRESETS):
            ttk.Button(bar, text=f"プリセット{k + 1} → 目標値", command=lambda k=k: self.preset(k)).pack(side=tk.LEFT, padx=1)

        tab_l = ttk.Frame(self.nb)
        self.nb.add(tab_l, text="実行ログ")
        pan = ttk.Frame(tab_l)
        pan.pack(fill=tk.BOTH, expand=True)
        left = ttk.LabelFrame(pan, text="試行ごとのログ")
        left.pack(side=tk.LEFT, fill=tk.Y, padx=4, pady=4)
        self.sec_lb = tk.Listbox(left, width=34, height=14)
        self.sec_lb.pack(fill=tk.Y, expand=True)
        self.sec_lb.bind('<<ListboxSelect>>', lambda e: self.show_section())
        ttk.Button(left, text="この試行のログをコピー", command=self.copy_section).pack(fill=tk.X)
        right = ttk.Frame(pan)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.sec_text = tk.Text(right, height=12, wrap='none', font=('TkFixedFont', 9))
        self.sec_text.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        self.sec_text.bind('<Key>', lambda e: None if (e.state & 0x4 and e.keysym.lower() in ('c', 'a')) else 'break')
        self.logpane = LogPane(tab_l, self.log, height=12)

        self.tab_g = ttk.Frame(self.nb)
        self.nb.add(self.tab_g, text="グラフ")
        self.graph_note = ttk.Label(self.tab_g, text="開始すると、対象の自由度ごとに5つのグラフを横1段に並べます（複数の自由度は縦に積む）")
        self.graph_note.pack(anchor='w', padx=6)
        gouter = ttk.Frame(self.tab_g)
        gouter.pack(fill=tk.BOTH, expand=True)
        self.g_canvas = tk.Canvas(gouter, highlightthickness=0)
        gvs = ttk.Scrollbar(gouter, orient=tk.VERTICAL, command=self.g_canvas.yview)
        self.g_inner = ttk.Frame(self.g_canvas)
        self.g_inner.bind('<Configure>', lambda e: self.g_canvas.configure(scrollregion=self.g_canvas.bbox('all')))
        self.g_canvas.create_window((0, 0), window=self.g_inner, anchor='nw')
        self.g_canvas.configure(yscrollcommand=gvs.set)
        self.g_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.g_label = tk.Label(self.g_inner)
        self.g_label.pack(anchor='nw')
        self.g_canvas.bind('<Configure>', self._graph_resized)
        gvs.pack(side=tk.RIGHT, fill=tk.Y)

        ctrl = ttk.Frame(self.root)
        ctrl.pack(fill=tk.X, padx=6, pady=4)
        self.btn_start = ttk.Button(ctrl, text="開始", command=self.start)
        self.btn_start.pack(side=tk.LEFT, padx=4)
        self.btn_resume = ttk.Button(ctrl, text="前回の続きから再開…", command=self.resume)
        self.btn_resume.pack(side=tk.LEFT, padx=4)
        self.btn_stop = ttk.Button(ctrl, text="停止", command=self.request_stop, state='disabled')
        self.btn_stop.pack(side=tk.LEFT, padx=4)
        ttk.Button(ctrl, text="設定を保存", command=self.save_settings).pack(side=tk.LEFT, padx=4)
        self.status = ttk.Label(ctrl, text="待機中")
        self.status.pack(side=tk.LEFT, padx=12)
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)
        self._after = self.root.after(100, self.tick)

    def ask_base(self):
        path = choose_folder(self.root, "Select folder", self.basic.get('base'))
        if path:
            self.basic.set('base', path)

    # ---------------- 目標値タブのボタン ----------------
    def from_robot(self):
        if self.link is None:
            messagebox.showwarning("Robot", "ROS2 に接続していません")
            return
        cur = self.link.board_targets(max_age=2.0)
        missing = [d for d, v in enumerate(cur) if v is None]
        if missing:
            messagebox.showwarning("Robot", f"受信できていない自由度があります: {missing}")
        self.table.set_initial([v if v is not None else math.nan for v in cur])
        self.log("実機が今使っている目標値を初期値に読み込みました")

    def preset(self, k):
        p = c.load_gui_poses()[1][k]
        if p is None:
            messagebox.showinfo("Preset", f"プリセット{k + 1} は登録されていません")
            return
        self.table.set_target(p)

    # ---------------- 設定 ----------------
    def collect(self):
        b, sr, sa = self.basic, self.search, self.safe
        T = parse_float(b.get('T'), 'T', lo=0.05, hi=10.0)
        initial, target = self.table.get()
        moving = self.table.opt_dofs()
        txt = b.get('dofs').replace('、', ',').strip()
        if txt:
            want = sorted({int(float(x)) for x in txt.split(',') if x.strip()})
            zero = [d for d in want if d not in moving]
            if zero:
                messagebox.showwarning("DOF", f"ΔP = 0 または FF の枠がない自由度は対象外にします: DOF {zero}")
            dofs = [d for d in want if d in moving]
        else:
            dofs = list(moving)
        same = [d for d in dofs if int(round(initial[d])) == int(round(target[d]))]
        if same:
            raise ValueError(f"|ΔP| < 0.5 で目標値のエコーが変わらない自由度があります: DOF {same}")
        if not dofs:
            raise ValueError("最適化対象の自由度がありません（目標値が初期値と違う自由度を指定してください）")
        sampler = self.sampler_var.get()
        if sampler == 'GP' and not self.torch_ok:
            raise ValueError(f"GP は選べません: {self.torch_why}")
        rms = parse_float(b.get('rms_thr'), '終了閾値 RMS', positive=True)
        base = b.get('base').strip()
        if not base:
            raise ValueError("保存先フォルダを指定してください")
        safety = {'sensor_lo': parse_float(sa.get('sensor_lo'), 'センサ下限'),
                  'sensor_hi': parse_float(sa.get('sensor_hi'), 'センサ上限'),
                  'dev_const': parse_float(sa.get('dev_const'), '偏差の上限 D', lo=0.0),
                  'rest_tol': parse_float(sa.get('rest_tol'), '静止判定の許容ずれ', positive=True),
                  'still_wait': parse_float(sa.get('still_wait'), '静止待ちの上限', lo=0.0, hi=120.0),
                  'missing_tol': int(parse_float(sa.get('missing_tol'), '欠損の許容数', lo=0, hi=1000)),
                  'remeasure_max': int(parse_float(sa.get('remeasure_max'), '測り直しの上限', lo=0, hi=20)),
                  'joint': self.joint.get()}
        if not safety['sensor_lo'] < safety['sensor_hi']:
            raise ValueError("センサ値の有効範囲の下限が上限以上です")
        for d in dofs:
            lo, hi = safety['joint'][d]
            for v, nm in ((initial[d], '初期値'), (target[d], '目標値')):
                if not lo <= v <= hi:
                    raise ValueError(f"DOF {d} の{nm} {v:g} が関節の可動範囲 [{lo:g}, {hi:g}] の外です")
        A_max = parse_float(b.get('A_max'), 'A_max', lo=1.0, hi=255.0)
        return {
            'T': T, 'dofs': dofs, 'wait': parse_float(b.get('wait'), '待機時間', lo=0.0, hi=600.0),
            'n_ref': int(parse_float(b.get('n_ref'), '基準試行の回数', lo=1, hi=10)),
            'max_trials': int(parse_float(b.get('max_trials'), '最大試行数', lo=1, hi=100000)),
            'rms_thr': rms, 'J_thr': (rms / 100.0) ** 2,
            'eps': parse_float(b.get('eps'), 'ε', positive=True, hi=50.0) / 100.0, 'A_max': A_max, 'base': base,
            'sampler': sampler, 'seed': int(parse_float(sr.get('seed'), 'seed', lo=0, hi=2 ** 31 - 1)),
            'm': parse_float(sr.get('m'), 'm', lo=0.001, hi=0.2), 'sep': parse_float(sr.get('sep'), 'sep', lo=0.001, hi=0.3),
            'rho': parse_float(sr.get('rho'), 'ρ', lo=0.0, hi=2.0), 'tabu_on': bool(self.tabu_var.get()),
            'seed_csv': self.seed_files.get(), 'past_npz': self.past_files.get(), 'use_past': bool(self.use_past_var.get()),
            'safety': safety,
            'limits': {'norobot': int(parse_float(sa.get('lim_norobot'), '上限', lo=1, hi=10000)),
                       'unsafe': int(parse_float(sa.get('lim_unsafe'), '上限', lo=1, hi=10000)),
                       'eval': int(parse_float(sa.get('lim_eval'), '上限', lo=1, hi=10000))},
            'initial': initial, 'target': target,
        }

    # ---------------- 設定の保存と読み込み（再起動しても最後の内容を引き継ぐ） ----------------
    @staticmethod
    def _load_settings():
        """設定ファイルを読む。表の値が壊れていても起動できるように、数値にできない値は既定値に戻す"""
        s = load_json(APP_SETTINGS_FILE, {})
        init0, _presets = c.load_gui_poses()

        def pose(vals, default):
            if not isinstance(vals, list) or len(vals) != c.N_DOF:
                return list(default)
            out = []
            for v, d0 in zip(vals, default):
                try:
                    f = float(v)
                    out.append(f if math.isfinite(f) else float(d0))
                except (TypeError, ValueError):
                    out.append(float(d0))
            return out

        s['initial'] = pose(s.get('initial'), init0)
        s['target'] = pose(s.get('target'), s['initial'])
        sf = s.get('safety') if isinstance(s.get('safety'), dict) else {}
        joint = sf.get('joint')
        if isinstance(joint, list) and len(joint) == c.N_DOF:
            fixed = []
            for d, jr in enumerate(joint):
                try:
                    lo, hi = float(jr[0]), float(jr[1])
                    fixed.append([lo, hi] if lo < hi else list(POT_RANGE[d]))
                except (TypeError, ValueError, IndexError):
                    fixed.append(list(POT_RANGE[d]))
            sf['joint'] = fixed
        else:
            sf.pop('joint', None)
        s['safety'] = sf
        if not isinstance(s.get('limits'), dict):
            s['limits'] = {}
        for k in ('seed_csv', 'past_npz'):
            if not isinstance(s.get(k), list):
                s[k] = []
        return s

    def snapshot(self):
        """今の欄の内容（入力した文字列のまま。検査はしない）"""
        b, sr, sa = self.basic, self.search, self.safe
        return {
            'T': b.get('T'), 'dofs': b.get('dofs'), 'wait': b.get('wait'), 'n_ref': b.get('n_ref'),
            'max_trials': b.get('max_trials'), 'rms_thr': b.get('rms_thr'), 'eps': b.get('eps'), 'A_max': b.get('A_max'),
            'base': b.get('base'),
            'sampler': self.sampler_var.get(), 'seed': sr.get('seed'), 'm': sr.get('m'), 'sep': sr.get('sep'),
            'rho': sr.get('rho'), 'tabu_on': bool(self.tabu_var.get()), 'seed_csv': self.seed_files.get(),
            'use_past': bool(self.use_past_var.get()), 'past_npz': self.past_files.get(),
            'safety': {k: sa.get(k) for k in ('sensor_lo', 'sensor_hi', 'dev_const', 'rest_tol', 'still_wait',
                                              'missing_tol', 'remeasure_max')}
            | {'joint': [[a.get(), b_.get()] for a, b_ in self.joint.vars]},
            'limits': {'norobot': sa.get('lim_norobot'), 'unsafe': sa.get('lim_unsafe'), 'eval': sa.get('lim_eval')},
            'initial': [v.get() for v in self.table.ini], 'target': [v.get() for v in self.table.tgt],
        }

    def _autosave(self, force=False):
        """欄の内容が変わっていたら設定ファイルへ書く（tick から1秒ごと・終了時・開始時に呼ぶ）"""
        try:
            snap = self.snapshot()
            text = json.dumps(snap, ensure_ascii=False, sort_keys=True)
            if force or text != self._saved_snapshot:
                save_json(APP_SETTINGS_FILE, snap)
                self._saved_snapshot = text
        except Exception as exc:
            print(f"設定の保存に失敗しました: {exc}", file=sys.stderr)

    def save_settings(self, cfg=None):
        """「設定を保存」ボタン・開始時。欄の内容をそのまま保存し、ボタンのときは値が正しいかも確かめて知らせる"""
        self._autosave(force=True)
        if cfg is None:
            try:
                self.collect()
                messagebox.showinfo("Settings", f"設定を保存しました（次回の起動でも使います）:\n{APP_SETTINGS_FILE}")
            except ValueError as exc:
                messagebox.showwarning("Settings", f"設定は保存しましたが、正しくない値があります（開始できません）:\n{exc}")

    def show_cfg(self, cfg):
        """再開する study の設定を欄に表示する（再開中は変更できない）"""
        for k in ('T', 'wait', 'n_ref', 'max_trials', 'rms_thr', 'A_max', 'base'):
            self.basic.set(k, cfg[k])
        self.basic.set('eps', cfg['eps'] * 100.0)
        self.basic.set('dofs', ', '.join(str(d) for d in cfg['dofs']))
        self.sampler_var.set(cfg['sampler'])
        for k in ('seed', 'm', 'sep', 'rho'):
            self.search.set(k, cfg[k])
        self.tabu_var.set(cfg['tabu_on'])
        for k, v in cfg['safety'].items():
            if k != 'joint':
                self.safe.set(k, v)
        for k, v in cfg['limits'].items():
            self.safe.set(f'lim_{k}', v)
        if cfg['safety'].get('joint'):
            self.joint.set(cfg['safety']['joint'])
        self.table.set_initial(cfg['initial'])
        self.table.set_target(cfg['target'])

    def lock(self, on):
        """実行中は設定欄を変更できないようにする（DOF 21・25 の目標値の欄は常に無効のまま）"""
        fixed = {str(self.table.tgt[d]) for d in c.NO_FF_DOFS}

        def walk(w):
            for ch in w.winfo_children():
                try:
                    if isinstance(ch, ttk.Combobox):
                        ch.configure(state='disabled' if on else 'readonly')
                    elif isinstance(ch, ttk.Entry):
                        keep = str(ch.cget('textvariable')) in fixed
                        ch.state(['disabled'] if (on or keep) else ['!disabled'])
                    elif isinstance(ch, (ttk.Checkbutton, ttk.Button)):
                        ch.state(['disabled'] if on else ['!disabled'])
                    elif isinstance(ch, tk.Listbox):
                        ch.configure(state='disabled' if on else 'normal')
                except tk.TclError:
                    pass
                walk(ch)

        for tab in self.nb.tabs():
            if self.nb.tab(tab, 'text') in ('基本', '探索', '安全', '目標値'):
                walk(self.nb.nametowidget(tab))
        for b in (self.btn_start, self.btn_resume):
            b.configure(state='disabled' if on else 'normal')
        self.btn_stop.configure(state='normal' if on else 'disabled')

    # ---------------- 開始・再開・停止 ----------------
    def _prechecks(self, dofs):
        if self.link is None:
            messagebox.showerror("ROS2", "ROS2 に接続していないので開始できません")
            return False
        prm = self.link.adrc_params()
        missing = [d for d in dofs if prm.get(d) is None]
        if missing:
            messagebox.showerror("ADRC", f"実機から ADRC パラメータを受信できていない最適化対象があります: DOF {missing}\n"
                                         f"（ボードの受信状況: {self.link.receiving()}）")
            return False
        others = {k: v for k, v in self.link.other_publishers().items() if v > 0}
        if others:
            if not messagebox.askokcancel("Other publishers",
                                          "目標値・FF のトピックに、このプログラム以外の publisher があります。\n"
                                          + "\n".join(f"  {k}: {v}" for k, v in others.items())
                                          + "\n\n送っていない GUI でも数に入ります。試行中に他のプログラムが送ると、"
                                            "その測定は測り直しになります。続けますか？"):
                return False
        return True

    def _launch(self, cfg, resume_folder=None):
        self.stop_event.clear()
        self.hist = []
        self.last_payload = None
        self._make_figure(cfg['dofs'])
        self.run_obj = OptimizationRun(self.link, cfg, self.log, self.stop_event, self.events, self.lang, resume_folder,
                                       offload=self.offload)
        self.worker = threading.Thread(target=self.run_obj.run, daemon=True)
        self.worker.start()
        self.lock(True)
        self.status.configure(text="実行中")

    def start(self):
        if self.worker is not None and self.worker.is_alive():
            return
        try:
            cfg = self.collect()
        except ValueError as exc:
            messagebox.showerror("Settings", str(exc))
            return
        if not self._prechecks(cfg['dofs']):
            return
        moves = "\n".join(f"  DOF {d}: {cfg['initial'][d]:g} → {cfg['target'][d]:g}" for d in cfg['dofs'])
        if not messagebox.askokcancel("Start", f"ロボットが動きます。\n\n最適化対象:\n{moves}\n\n"
                                               f"T = {cfg['T']} s, 基準試行 {cfg['n_ref']} 回, 最大試行数 {cfg['max_trials']}, "
                                               f"A_max = {cfg['A_max']:g} PWM, Sampler {cfg['sampler']}\n\n開始しますか？"):
            return
        self.save_settings(cfg)
        self._launch(cfg)

    def resume(self):
        if self.worker is not None and self.worker.is_alive():
            return
        folder = choose_folder(self.root, "Select study folder", self.basic.get('base'))
        if not folder:
            return
        ok, msg, cfg, lock = OptimizationRun.check_resume(folder, self.link)
        if not ok:
            messagebox.showerror("Resume", msg)
            return
        if lock is not None:
            if not messagebox.askokcancel("Lock", f"Optuna のロックファイルが残っています（前回の異常終了）:\n{lock}\n\n"
                                                  "実行中のプロセスはありません。削除して再開しますか？"):
                return
            try:
                os.remove(lock)
            except OSError as exc:
                messagebox.showerror("Lock", f"削除できません: {exc}")
                return
        if not self._prechecks(cfg['dofs']):
            return
        self.show_cfg(cfg)
        if not messagebox.askokcancel("Resume", f"ロボットが動きます。\n\n{folder}\nの探索を続けます"
                                                f"（設定は保存されたものを使い、変更できません）。開始しますか？"):
            return
        self._launch(cfg, resume_folder=folder)

    def request_stop(self):
        self.stop_event.set()
        self.status.configure(text="停止中（FF を止める → 初期姿勢 → 保存）…")
        self.btn_stop.configure(state='disabled')

    def on_signal(self, *_):
        """SIGINT・SIGTERM：停止を要求し、ワーカーが安全に戻して保存したら終了する（§7.6）"""
        self.stop_event.set()
        self.closing = True

    # ---------------- 表示 ----------------
    def show_section(self):
        sel = self.sec_lb.curselection()
        if not sel:
            return
        self.sec_text.delete('1.0', tk.END)
        self.sec_text.insert(tk.END, self.log.section_text(sel[0]))

    def copy_section(self):
        sel = self.sec_lb.curselection()
        if not sel:
            return
        self.root.clipboard_clear()
        self.root.clipboard_append(self.log.section_text(sel[0]))

    def _make_figure(self, dofs):
        """グラフタブの対象の自由度を決める（描画は描画用のプロセスで行い、できた画像を表示するだけ）"""
        self.fig_dofs = list(dofs)
        self.graph_dirty = True

    def _redraw(self):
        self.graph_dirty = True

    def _graph_resized(self, event):
        if abs(event.width - self.g_width) > 40:
            self.g_width = event.width
            self.graph_dirty = True

    def _graph_update(self):
        """描画が終わっていれば表示し、グラフタブを開いていて更新が必要なら次の描画を頼む（同時に1つだけ）"""
        if self.graph_future is not None and self.graph_future.done():
            try:
                png = self.graph_future.result()
                self.g_image = tk.PhotoImage(data=base64.b64encode(png).decode('ascii'), format='png')
                self.g_label.configure(image=self.g_image)
            except Exception as exc:
                self.log(f"グラフの更新に失敗しました: {exc}")
            self.graph_future = None
        if not (self.graph_dirty and self.graph_future is None and self.fig_dofs):
            return
        if self.nb.select() != str(self.tab_g):
            return
        J_thr = self.run_obj.cfg.get('J_thr') if self.run_obj else None
        width = max(800, self.g_canvas.winfo_width() - 20)
        self.graph_future = self.offload.render.submit(render_preview, self.last_payload, list(self.hist),
                                                       list(self.fig_dofs), J_thr, width)
        self.graph_dirty = False

    def tick(self):
        self.logpane.poll()
        self._tick_n = getattr(self, '_tick_n', 0) + 1
        if self._tick_n % 10 == 0:
            self._autosave()
        with self.log.lock:
            secs = list(self.log.sections)
        if len(secs) != self.n_sections:
            for k in range(self.n_sections, len(secs)):
                self.sec_lb.insert(tk.END, secs[k][0])
            self.n_sections = len(secs)
        changed = False
        while True:
            try:
                kind, data = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == 'trial':
                self.last_payload = data
                changed = True
            elif kind == 'hist':
                self.hist = data
                changed = True
            elif kind == 'done':
                self.status.configure(text=f"終了（{data[0]}{'：' + data[1] if data[1] else ''}）。次の探索を開始・再開できます")
        if changed:
            self.graph_dirty = True
        self._graph_update()
        if self.worker is not None and not self.worker.is_alive():
            self.worker = None
            self.lock(False)
        if self.closing and (self.worker is None or not self.worker.is_alive()):
            self.finalize()
            return
        self._after = self.root.after(100, self.tick)

    def on_close(self):
        if self.worker is not None and self.worker.is_alive():
            if not messagebox.askokcancel("Quit", "実行中です。停止して（FF を止め、初期姿勢へ戻し、保存して）終了しますか？"):
                return
            self.request_stop()
            self.closing = True
            return
        self.finalize()

    def finalize(self):
        """終了の順序（§7.6）: ワーカーが FF 停止 → 初期姿勢 → エコー確認を済ませた後に、executor 停止 → node 破棄 → shutdown"""
        try:
            self.root.after_cancel(self._after)
        except Exception:
            pass
        self._autosave(force=True)
        self.offload.close()
        if self.link is not None:
            try:
                if self.link.sent_any:
                    self.link.stop_ff()
            except Exception:
                pass
            self.link.close()
            try:
                import rclpy
                rclpy.try_shutdown()
            except Exception:
                pass
            self.link = None
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def run(self):
        self.root.mainloop()


def main(args=None):
    app = App()
    signal.signal(signal.SIGINT, app.on_signal)
    signal.signal(signal.SIGTERM, app.on_signal)
    try:
        app.run()
    finally:
        if app.link is not None:
            try:
                app.link.close()
            except Exception:
                pass


if __name__ == '__main__':
    main()
