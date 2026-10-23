#!/usr/bin/env python3
"""ADRC＋FF最適制御（_2 系）の GUI 部品

optimize_EL_ff_ROS2_2.py と optimize_TuningEL_ff_ROS2_2.py で共有する設定欄・目標値の表・ログ欄。
ウィンドウのタイトル・ダイアログのタイトルは英字にする（WSLg ではタイトルバーの日本語が文字化けするため）。
"""
import json
import math
import os
import queue
import time

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

if __package__:
    from . import optimize_EL2_common as c
    from . import GUI_android_tk as gat
else:
    import optimize_EL2_common as c
    import GUI_android_tk as gat


# ==============================================================================
# 設定ファイル
# ==============================================================================
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


def parse_list(text, name):
    vals = []
    for tok in str(text).replace('、', ',').split(','):
        tok = tok.strip()
        if tok:
            vals.append(parse_float(tok, name, positive=True))
    return vals


# ==============================================================================
# ログ（ワーカースレッド → GUI。ターミナルにも同じ内容を出す）
# ==============================================================================
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


# ==============================================================================
# 小さな入力欄
# ==============================================================================
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


def build_target_tab(nb, settings, link_getter, logger):
    """目標値タブ（表と、初期姿勢・プリセット・実機の現在値を読み込むボタン）"""
    tab = ttk.Frame(nb)
    nb.add(tab, text="目標値")
    gs, _warn = gat.load_settings()
    initial = settings.get('initial') or gs['initial_pose']
    target = settings.get('target') or list(initial)
    info = ttk.Label(tab, text="", foreground='#004')
    bar = ttk.Frame(tab)
    bar.pack(fill=tk.X, padx=6, pady=4)
    table = TargetTable(tab, initial, target)

    def from_robot():
        link = link_getter()
        if link is None:
            messagebox.showwarning("Robot", "ROS2 に接続していません")
            return
        cur = link.board_targets(max_age=2.0)
        if any(v is None for v in cur):
            missing = [d for d, v in enumerate(cur) if v is None]
            messagebox.showwarning("Robot", f"受信できていない自由度があります: {missing}")
        table.set_initial([v if v is not None else math.nan for v in cur])
        logger("実機が今使っている目標値を初期値に読み込みました")

    def from_gui_initial():
        g, _ = gat.load_settings()
        table.set_initial(g['initial_pose'])
        logger("GUI_android_tk の初期姿勢を初期値に読み込みました")

    def preset_to_target(k):
        g, _ = gat.load_settings()
        p = g['presets'][k]
        if p is None:
            messagebox.showinfo("Preset", f"プリセット{k + 1} は登録されていません")
            return
        table.set_target(p)

    ttk.Button(bar, text="実機の現在の目標値 → 初期値", command=from_robot).pack(side=tk.LEFT, padx=2)
    ttk.Button(bar, text="GUI_android_tk の初期姿勢 → 初期値", command=from_gui_initial).pack(side=tk.LEFT, padx=2)
    for k in range(gat.NUM_PRESETS):
        ttk.Button(bar, text=f"プリセット{k + 1} → 目標値", command=lambda k=k: preset_to_target(k)).pack(side=tk.LEFT, padx=1)
    ttk.Button(bar, text="目標値 = 初期値", command=lambda: table.set_target([v.get() for v in table.ini])).pack(side=tk.LEFT, padx=2)
    info.pack(fill=tk.X, padx=8)
    ttk.Label(tab, text="|ΔP| ≠ 0 の自由度がすべて最適化対象になります。DOF 21・25 は FF の枠がないため目標値を初期値に固定します。",
              foreground='#555').pack(fill=tk.X, padx=8)
    table.on_change = lambda: info.configure(text=f"最適化対象（|ΔP| ≠ 0）: DOF {table.opt_dofs()}")
    table.refresh()
    return table


def build_weights(parent, settings):
    f = Fields(parent, "評価関数の重み  J = e_Nᵀ Sf e_N + Σ (eᵀ Q e + R u²)·L   （e = [位置, 速度, 加速度] の誤差, u = FF）")
    w = settings.get('weights', c.DEFAULT_WEIGHTS)
    for i, nm in enumerate(('位置', '速度', '加速度')):
        f.add(f'Q{i}', f"Q（{nm}）", w['Q'][i])
    for i, nm in enumerate(('位置', '速度', '加速度')):
        f.add(f'Sf{i}', f"Sf 終端（{nm}）", w['Sf'][i])
    f.add('R', "R（FF入力の大きさ）", w['R'])
    return f


def read_weights(f):
    Q = [parse_float(f.get(f'Q{i}'), f'Q{i}', lo=0.0) for i in range(3)]
    Sf = [parse_float(f.get(f'Sf{i}'), f'Sf{i}', lo=0.0) for i in range(3)]
    R = parse_float(f.get('R'), 'R', positive=True)
    return c.Weights(Q=Q, Sf=Sf, R=R)


def build_bounds(parent, settings):
    b = settings.get('bounds', {})
    f = Fields(parent, "システムモデル b0/((T1 s+1)(s²+2ζωn s+ωn²)) と目標軌道 1/((T1 s+1)(s+ωn)²) の同定の上下限")
    for key, label in (('sys_T1', 'システム T1 [s]'), ('sys_zeta', 'システム ζ'), ('sys_wn', 'システム ωn [rad/s]'),
                       ('sys_b0', 'システム b0'), ('tgt_T1', '目標軌道 T1 [s]'), ('tgt_wn', '目標軌道 ωn [rad/s]')):
        lo, hi = b.get(key, c.DEFAULT_ID_BOUNDS[key])
        f.add(key + '_lo', f"{label} 下限", lo)
        f.add(key + '_hi', f"{label} 上限", hi)
    w = settings.get('id_weights', c.DEFAULT_ID_WEIGHTS)
    f.add('w_pwm', "閉ループ同定の u_ADRC 残差の重み", w['w_pwm'], note="POT 残差に対する比（各チャネルを自分のスケールで割った後）")
    f.add('w_z3', "閉ループ同定の z3 残差の重み", w['w_z3'])
    return f


def read_bounds(f):
    out = {}
    for key in c.DEFAULT_ID_BOUNDS:
        lo = parse_float(f.get(key + '_lo'), key + ' 下限')
        hi = parse_float(f.get(key + '_hi'), key + ' 上限')
        if not lo < hi:
            raise ValueError(f"{key} の下限 {lo} が上限 {hi} 以上です")
        if key in ('sys_T1', 'sys_wn', 'tgt_T1', 'tgt_wn') and lo <= 0:
            raise ValueError(f"{key} の下限は正の値にしてください")
        out[key] = (lo, hi)
    idw = {'w_pwm': parse_float(f.get('w_pwm'), 'w_pwm', lo=0.0), 'w_z3': parse_float(f.get('w_z3'), 'w_z3', lo=0.0)}
    return out, idw


def build_timing(parent, settings):
    f = Fields(parent, "ファームウェアの時間（dt・h は式の中の係数で、周回間隔とは別）")
    f.add('L', "制御の周回間隔 L [s]", settings.get('L', c.LOOP_DT_ASSUMED),
          note="仮定値（実機から正確には測れない）。ESO と ADRC は1周に1回")
    ttk.Label(f.frame, text=f"ESO の dt = {c.ESO_DT} s、derivatives の h = {c.DIFF_H} s、RCローパス係数 = {c.OMEGA_LPF}"
                            f"（ファームの値。変更しない）", foreground='#555').grid(row=f.row, column=0, columnspan=3,
                                                                              sticky='w', padx=4)
    f.row += 1
    f.add('sens_L', "感度評価する周回間隔 [s]（カンマ区切り）", settings.get('sens_L', "0.0105, 0.0115"), width=20,
          note="同定したモデルのまま周回間隔だけ変えて POT・u_ADRC・z3 の再現誤差を比べる（空欄で省略）")
    f.add('jitter', "周回間隔のばらつき σ [ms]", settings.get('jitter', 0.0),
          note="0 より大きければ、周回ごとに間隔をばらつかせた再現（10回平均）も評価する")
    return f


def read_timing(f):
    L = parse_float(f.get('L'), 'L', lo=1e-4, hi=1.0)
    sens = parse_list(f.get('sens_L'), '感度評価の周回間隔') if f.get('sens_L').strip() else []
    jit = parse_float(f.get('jitter'), 'ばらつき σ', lo=0.0) * 1e-3
    return L, sens, jit


def setup_root(title):
    root = tk.Tk()
    root.title(title)
    gat.setup_japanese_font(root)
    return root


def ask_folder(var):
    path = filedialog.askdirectory(title="Select folder")
    if path:
        var.set(path)
