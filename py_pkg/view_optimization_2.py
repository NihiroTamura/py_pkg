#!/usr/bin/env python3
"""optimize_EL_ff_ROS2_2.py の最適化ビューア

最適化本体がループ（試行）ごとに書く <保存先>/loops/loop_NN.npz を監視し、自由度ごとに9種類のグラフを表示する。
保存先は /tmp/el2_optimization_current.json（本体が書く）から自動で知るか、引数でフォルダを指定する。

  起動: ros2 run py_pkg view_optimization_2            （実行中・最後に実行した最適化を表示）
        python3 view_optimization_2.py <optimize_… フォルダ>

グラフ（横軸は目標値を変えた周回を 0 とした時間 t = k·L [s]。L は周回間隔の仮定値）
  1. POT: 実測、ζ=1 の目標軌道（実測に当てはめたもの）、システムモデルの応答（同じ初期状態・同じFFで閉ループを回したもの）
  2. 累積誤差 J_i（この自由度）と J（全対象自由度の平均）のループごとの変化
  3. 送ったFFの極値 (t1, y1), (t2, y2) のループごとの変化
  4. 実測の PWM と u_ADRC（= PWM − FF）
  5. 送ったFFの波形と極値
  6. システムモデルの同定パラメータ（T1, ζ, ωn, b0）のループごとの変化
  7. 目標軌道の同定パラメータ（T1, ωn）のループごとの変化
  8. z3（ESO の外乱推定値）: 実測と、システムモデルで同じ条件を回した内部シミュレーション
  9. PWM: 実測と、システムモデルで同じ条件を回した内部シミュレーション（u_ADRC + FF）
  ループ 0 は ADRC のみ（FF=0）の試行。

終了時の保存（最適化が終わると自動、ウィンドウを閉じるときにも未保存なら保存）
  <保存先>/DOF/DOF<d>/D<d>_g1_pot_loopNN.png, _g4_pwm_adrc_loopNN.png, _g5_ff_loopNN.png, _g8_z3_loopNN.png,
                      _g9_pwm_model_loopNN.png（ループごと）
                      D<d>_g2_cost.png, _g3_extrema.png, _g6_system_params.png, _g7_target_params.png（最新）
  画面の文字は英語（matplotlib に日本語フォントがない環境のため）。
"""
import glob
import json
import os
import sys

import numpy as np

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

import tkinter as tk
from tkinter import ttk, messagebox

POINTER_FILE = '/tmp/el2_optimization_current.json'     # optimize_EL2_common.POINTER_FILE と同じ
LOOP_DIR = 'loops'
FINISHED_FILE = 'finished.json'
POLL_MS = 1000
PWM_LIMIT = 255.0

GRAPHS = [   # (番号, ファイル名の種別, ループごとに保存するか)
    (1, 'pot', True), (2, 'cost', False), (3, 'extrema', False),
    (4, 'pwm_adrc', True), (5, 'ff', True), (6, 'system_params', False),
    (7, 'target_params', False), (8, 'z3', True), (9, 'pwm_model', True),
]


def load_loops(folder):
    out = {}
    for path in sorted(glob.glob(os.path.join(folder, LOOP_DIR, 'loop_*.npz'))):
        try:
            with np.load(path, allow_pickle=False) as z:
                d = {k: z[k] for k in z.files}
            out[int(d['loop'])] = d
        except Exception:
            continue                                                    # 書き込み途中などは次の更新で読む
    return out


def dof_row(d, dof):
    idx = np.flatnonzero(np.asarray(d['dofs']) == dof)
    return int(idx[0]) if len(idx) else None


def history(loops, dof, key, n):
    xs, ys = [], []
    for k in sorted(loops):
        r = dof_row(loops[k], dof)
        if r is None:
            continue
        xs.append(k)
        ys.append(np.asarray(loops[k][key])[r] if n else float(np.asarray(loops[k][key])[r]))
    return np.array(xs), np.array(ys, dtype=float)


# ==============================================================================
# 描画（画面と保存で同じ関数を使う）
# ==============================================================================
def _title(ax, text):
    ax.set_title(text, fontsize=9)
    ax.grid(alpha=0.3)


def plot_pot(ax, loops, dof, n):
    d = loops[n]
    r = dof_row(d, dof)
    t = d['t']
    pot = d['pot'][r]
    ax.plot(t, pot, 'k-', lw=1.2, label='measured POT')
    ax.plot(t, d['tgt'][r], 'r--', lw=1.2, label='target (zeta=1 fit)')
    ax.plot(t, d['sys'][r], 'b-', lw=1.0, alpha=0.8, label='system model (closed loop)')
    ax.axhline(float(d['Pf'][r]), color='g', lw=0.7, ls=':')
    ax.axvline(float(d['T']), color='0.5', lw=0.8, ls=':')
    _title(ax, f"[1] DOF {dof} loop {n}: POT  J_i={float(d['J_i'][r]):.3g}")
    ax.set_xlabel('t = k*L [s]')
    ax.set_ylabel('POT [count]')
    ax.legend(fontsize=7)


def plot_cost(ax, loops, dof, n):
    x, Ji = history(loops, dof, 'J_i', 0)
    xs = sorted(loops)
    J = [float(loops[k]['J']) for k in xs]
    ax.semilogy(x, Ji, 'o-', label=f'J_i (DOF {dof})')
    ax.semilogy(xs, J, 's--', label='J (mean over DOFs)')
    best = [k for k in xs if bool(loops[k].get('is_best', False))]
    if best:
        ax.semilogy(best, [float(loops[k]['J']) for k in best], 'r*', ms=12, label='best FF trial')
    _title(ax, f"[2] DOF {dof}: cumulative error (loop 0 = ADRC only)")
    ax.set_xlabel('loop')
    ax.set_ylabel('J = mean((POT-target)/dP)^2')
    ax.legend(fontsize=7)


def plot_extrema(ax, loops, dof, n):
    x, ext = history(loops, dof, 'extrema', 4)
    _, co = history(loops, dof, 'coeffs', 5)
    if len(x):
        ext[~np.any(co != 0, axis=1)] = np.nan                         # FF を送っていないループ（ループ 0 など）は描かない
    if len(x):
        ax.plot(x, ext[:, 0], 'o-', label='t1')
        ax.plot(x, ext[:, 2], 's-', label='t2')
    ax.set_ylabel('time [s]')
    ax2 = ax.twinx()
    if len(x):
        ax2.plot(x, ext[:, 1], 'o--', color='tab:red', label='y1')
        ax2.plot(x, ext[:, 3], 's--', color='tab:purple', label='y2')
    ax2.set_ylabel('PWM')
    _title(ax, f"[3] DOF {dof}: FF extrema sent in each loop")
    ax.set_xlabel('loop')
    ax.xaxis.get_major_locator().set_params(integer=True)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=7)


def plot_pwm_adrc(ax, loops, dof, n):
    d = loops[n]
    r = dof_row(d, dof)
    t = d['t']
    ax.plot(t, d['pwm'][r], 'k-', lw=1.0, label='measured PWM (u_ADRC + FF)')
    ax.plot(t, d['uA'][r], 'tab:orange', lw=1.0, label='u_ADRC = PWM - FF')
    ax.axvline(float(d['T']), color='0.5', lw=0.8, ls=':')
    _title(ax, f"[4] DOF {dof} loop {n}: PWM and u_ADRC")
    ax.set_xlabel('t = k*L [s]')
    ax.set_ylabel('PWM')
    ax.legend(fontsize=7)


def plot_ff(ax, loops, dof, n):
    d = loops[n]
    r = dof_row(d, dof)
    t = d['t']
    ax.plot(t, d['ff'][r], 'r-', lw=1.3, label='FF sent (5th-order)')
    t1, y1, t2, y2 = [float(v) for v in d['extrema'][r]]
    if abs(y1) > 0 or abs(y2) > 0:
        ax.plot([t1, t2], [y1, y2], 'ko', ms=5)
        ax.annotate(f"({t1:.3f}, {y1:+.2f})", (t1, y1), fontsize=7)
        ax.annotate(f"({t2:.3f}, {y2:+.2f})", (t2, y2), fontsize=7)
    ax.axhline(0.0, color='k', lw=0.5)
    ax.axvline(float(d['T']), color='0.5', lw=0.8, ls=':')
    co = d['coeffs'][r]
    _title(ax, f"[5] DOF {dof} loop {n}: FF  a..e = " + ", ".join(f"{v:.3g}" for v in co))
    ax.set_xlabel('t = k*L [s]')
    ax.set_ylabel('PWM')
    ax.legend(fontsize=7)


def plot_system_params(ax, loops, dof, n):
    x, p = history(loops, dof, 'sys_params', 4)
    if len(x):
        ax.semilogy(x, np.abs(p[:, 0]), 'o-', label='T1 [s]')
        ax.semilogy(x, np.abs(p[:, 2]), 's-', label='wn [rad/s]')
        ax.semilogy(x, np.abs(p[:, 1]), '^-', label='|zeta|')
        ax.semilogy(x, np.abs(p[:, 3]), 'd-', label='|b0|')
    _title(ax, f"[6] DOF {dof}: system model b0/((T1 s+1)(s^2+2 zeta wn s+wn^2))")
    ax.set_xlabel('loop (identified from that loop)')
    ax.legend(fontsize=7)


def plot_target_params(ax, loops, dof, n):
    x, p = history(loops, dof, 'tgt_params', 2)
    if len(x):
        ax.plot(x, p[:, 0], 'o-', label='T1 [s]')
    ax.set_ylabel('T1 [s]')
    ax2 = ax.twinx()
    if len(x):
        ax2.plot(x, p[:, 1], 's--', color='tab:red', label='wn [rad/s]')
    ax2.set_ylabel('wn [rad/s]')
    _title(ax, f"[7] DOF {dof}: target trajectory 1/((T1 s+1)(s+wn)^2)")
    ax.set_xlabel('loop (fitted to that loop)')
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=7)


def plot_z3(ax, loops, dof, n):
    d = loops[n]
    r = dof_row(d, dof)
    t = d['t']
    ax.plot(t, d['z3'][r], 'k-', lw=1.0, label='measured z3')
    ax.plot(t, d['z3_sim'][r], 'b--', lw=1.0, label='internal simulation z3')
    e = d['z3_sim'][r] - d['z3'][r]
    rms = float(np.sqrt(np.nanmean(e ** 2))) if np.isfinite(e).any() else float('nan')
    ic = float(d['ic'][r])
    _title(ax, f"[8] DOF {dof} loop {n}: z3 (ESO)  rms err {rms:.3g} (= {rms / ic:.2f} PWM eq.)")
    ax.set_xlabel('t = k*L [s]')
    ax.set_ylabel('z3 (firmware units)')
    ax.legend(fontsize=7)


def plot_pwm_model(ax, loops, dof, n):
    d = loops[n]
    r = dof_row(d, dof)
    t = d['t']
    ax.plot(t, d['pwm'][r], 'k-', lw=1.0, label='measured PWM')
    ax.plot(t, d['pwm_sim'][r], 'b--', lw=1.0, label='internal simulation PWM')
    e = d['pwm_sim'][r] - d['pwm'][r]
    rms = float(np.sqrt(np.nanmean(e ** 2))) if np.isfinite(e).any() else float('nan')
    _title(ax, f"[9] DOF {dof} loop {n}: PWM  rms err {rms:.2f}")
    ax.set_xlabel('t = k*L [s]')
    ax.set_ylabel('PWM')
    ax.legend(fontsize=7)


PLOTTERS = {1: plot_pot, 2: plot_cost, 3: plot_extrema, 4: plot_pwm_adrc, 5: plot_ff, 6: plot_system_params,
            7: plot_target_params, 8: plot_z3, 9: plot_pwm_model}


def save_images(folder, loops, log=print):
    """<folder>/DOF/DOF<d>/ にグラフを保存する（1・4・5・8・9 はループごと、2・3・6・7 は最新の1枚）"""
    if not loops:
        return 0
    dofs = sorted({int(v) for d in loops.values() for v in np.asarray(d['dofs'])})
    last = max(loops)
    count = 0
    for dof in dofs:
        out = os.path.join(folder, 'DOF', f'DOF{dof}')
        os.makedirs(out, exist_ok=True)
        for num, kind, per_loop in GRAPHS:
            ns = [k for k in sorted(loops) if dof_row(loops[k], dof) is not None] if per_loop else [last]
            for n in ns:
                fig = Figure(figsize=(7.0, 4.2), dpi=110)
                FigureCanvasAgg(fig)
                ax = fig.add_subplot(111)
                try:
                    PLOTTERS[num](ax, loops, dof, n)
                except Exception as exc:
                    ax.text(0.5, 0.5, f"cannot plot: {exc}", ha='center', transform=ax.transAxes)
                fig.tight_layout()
                name = f"D{dof}_g{num}_{kind}" + (f"_loop{n:02d}" if per_loop else "") + ".png"
                fig.savefig(os.path.join(out, name))
                count += 1
    log(f"グラフを保存しました: {os.path.join(folder, 'DOF')}（{count} 枚）")
    return count


# ==============================================================================
# GUI
# ==============================================================================
class Viewer:
    def __init__(self, root, folder=None):
        self.root = root
        self.fixed_folder = folder
        self.folder = folder
        self.loops = {}
        self.sig = None
        self.saved_for = None
        root.title("EL2 optimization viewer")
        bar = ttk.Frame(root)
        bar.pack(fill=tk.X, padx=4, pady=2)
        ttk.Label(bar, text="DOF").pack(side=tk.LEFT)
        self.dof_var = tk.StringVar()
        self.dof_box = ttk.Combobox(bar, textvariable=self.dof_var, width=6, state='readonly')
        self.dof_box.pack(side=tk.LEFT, padx=2)
        self.dof_box.bind('<<ComboboxSelected>>', lambda e: self.redraw())
        ttk.Label(bar, text="loop").pack(side=tk.LEFT, padx=(10, 0))
        self.loop_var = tk.StringVar()
        self.loop_box = ttk.Combobox(bar, textvariable=self.loop_var, width=6, state='readonly')
        self.loop_box.pack(side=tk.LEFT, padx=2)
        self.loop_box.bind('<<ComboboxSelected>>', lambda e: (self.follow.set(False), self.redraw()))
        self.follow = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="最新のループを表示", variable=self.follow, command=self.redraw).pack(side=tk.LEFT, padx=6)
        ttk.Button(bar, text="画像を保存", command=self.save).pack(side=tk.LEFT, padx=6)
        self.info = ttk.Label(bar, text="")
        self.info.pack(side=tk.LEFT, padx=10)
        self.fig = Figure(figsize=(16, 10), dpi=80)
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        NavigationToolbar2Tk(self.canvas, root).update()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        root.protocol('WM_DELETE_WINDOW', self.on_close)
        self.poll()

    def resolve_folder(self):
        if self.fixed_folder:
            return self.fixed_folder
        try:
            with open(POINTER_FILE, encoding='utf-8') as f:
                return json.load(f).get('folder')
        except (OSError, ValueError):
            return None

    def poll(self):
        folder = self.resolve_folder()
        if folder and os.path.isdir(folder):
            files = sorted(glob.glob(os.path.join(folder, LOOP_DIR, 'loop_*.npz')))
            sig = (folder, tuple((p, os.path.getmtime(p)) for p in files))
            if sig != self.sig:
                self.sig = sig
                if folder != self.folder:
                    self.folder = folder
                    self.saved_for = None
                self.loops = load_loops(folder)
                self.update_choices()
                self.redraw()
            finished = os.path.exists(os.path.join(folder, FINISHED_FILE))
            if finished and self.loops and self.saved_for != (folder, len(self.loops)):
                self.save()
            self.info.configure(text=f"{folder}  loops: {len(self.loops)}{'  (finished)' if finished else ''}")
        else:
            self.info.configure(text="最適化の保存先が見つかりません（optimize_EL_ff_ROS2_2.py の開始を待っています）")
        self.root.after(POLL_MS, self.poll)

    def update_choices(self):
        dofs = sorted({int(v) for d in self.loops.values() for v in np.asarray(d['dofs'])})
        self.dof_box['values'] = [str(v) for v in dofs]
        if dofs and self.dof_var.get() not in [str(v) for v in dofs]:
            self.dof_var.set(str(dofs[0]))
        ls = sorted(self.loops)
        self.loop_box['values'] = [str(v) for v in ls]
        if ls and (self.follow.get() or self.loop_var.get() not in [str(v) for v in ls]):
            self.loop_var.set(str(ls[-1]))

    def redraw(self):
        self.fig.clear()
        if not self.loops or not self.dof_var.get():
            self.canvas.draw_idle()
            return
        dof = int(self.dof_var.get())
        ls = sorted(self.loops)
        n = ls[-1] if self.follow.get() else int(self.loop_var.get())
        if self.follow.get():
            self.loop_var.set(str(n))
        if dof_row(self.loops[n], dof) is None:
            n = max(k for k in ls if dof_row(self.loops[k], dof) is not None)
        for i in range(9):
            ax = self.fig.add_subplot(3, 3, i + 1)
            try:
                PLOTTERS[i + 1](ax, self.loops, dof, n)
            except Exception as exc:
                ax.text(0.5, 0.5, f"cannot plot: {exc}", ha='center', transform=ax.transAxes, fontsize=8)
        d = self.loops[n]
        kind = 'ADRC only (FF=0)' if n == 0 else 'ADRC + FF'
        self.fig.suptitle(f"DOF {dof}  loop {n} ({kind})  T={float(d['T']):g}s  "
                          f"L={float(d['L']) * 1000:.1f}ms (assumed)  J={float(d['J']):.3g}", fontsize=11)
        self.fig.tight_layout(rect=(0, 0, 1, 0.96))
        self.canvas.draw_idle()

    def save(self):
        if not self.folder or not self.loops:
            return
        save_images(self.folder, self.loops)
        self.saved_for = (self.folder, len(self.loops))

    def on_close(self):
        if self.folder and self.loops and self.saved_for != (self.folder, len(self.loops)):
            try:
                self.save()
            except Exception as exc:
                messagebox.showerror("Save", f"画像を保存できませんでした: {exc}")
        self.root.destroy()


def main(args=None):
    argv = sys.argv[1:] if args is None else args
    folder = next((a for a in argv if os.path.isdir(a)), None)
    root = tk.Tk()
    try:
        if __package__:
            from . import GUI_android_tk as gat
        else:
            import GUI_android_tk as gat
        gat.setup_japanese_font(root)
    except Exception:
        pass
    Viewer(root, folder)
    root.mainloop()


if __name__ == '__main__':
    main()
