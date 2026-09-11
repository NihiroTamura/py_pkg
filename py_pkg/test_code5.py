## 2026/08/07　19:30までのview_optimizationプログラム。EL方程式プログラムはtest_code2に対応。
#!/usr/bin/env python3
"""最適化ビューア（test_code2.py が書き出すスナップショットを監視して表示する別プロセス）

test_code2.py は内側ループが1回終わるごとに、24自由度分の波形とパラメータ履歴を
1つの .npz へ書き出す。本ビューアはそのファイルの更新時刻を監視し、更新されていれば
読み直して自動で再描画する。別プロセスなので、ビューアを閉じても最適化は影響を受けない。

自由度を切り替えると、その自由度のすべての情報が表示される。
    ・実測 / システムモデル / 目標モデル の応答フィット
    ・最適入力 u_opt / 5次関数FF近似 / 今回印加したFF入力 と極値
    ・システムモデル同定に実際に使った入力（実PWM基準 or FF入力）
    ・目標モデルパラメータ (T1, wn) の推移
    ・システムモデルパラメータ (T1, zeta, wn, b0) の推移
    ・FF極値 (t1, t2, y1, y2) の推移
    ・評価関数値 J・近似残差・同定残差 の推移（再測定を要した内側ループには印を付ける）
    ・現在の数値一覧（左パネル）

システムモデル同定の入力（test_code2.py の IDENTIFICATION_INPUT_MODE）
    'actual_pwm' … 実際に印加されたPWM − 保持PWM。座標系は z = P - Pi
    'ff_input'   … その測定で与えた5次関数FF入力そのもの。座標系は y = P - Pf
    どちらで同定されたスナップショットかは、状態表示・左パネル・入力グラフの凡例に出る。

最適制御の対象自由度（test_code2.py の OPT_DOF_IDS）
    test_code2.py は OPT_DOF_IDS に指定した自由度だけを最適制御の対象にし、それ以外の自由度は
    FF入力を0にしたまま実測POT値だけを記録する（同定・最適入力計算はいっさい行わない）。
    本ビューアはスナップショットの 'opt_dof' フラグを見て、対象外の自由度では
    画面のレイアウト（6枚のグラフと左パネル）と表示枠をそのまま保ちつつ、
    応答グラフに実測POT値だけを描き、他のグラフには "Not optimized" と表示する。
    このフラグを持たない古いスナップショットは、全自由度が対象だったものとして扱う。

再測定について
    受信が途切れた測定は test_code2.py 側で失敗と判定され、同じFF入力で測り直される。
    失敗した測定はスナップショットを書き出さないので、ここに表示されるのは同定に使われた
    データだけになる。内側ループ番号も進まないため、履歴グラフの横軸はずれない。

使い方
    ros2 run py_pkg test_code5
    python3 test_code5.py [スナップショットのパス]

注意: この環境には日本語フォントが入っていない（fc-list :lang=ja が0件）ため、Tkの部品も
      matplotlibのラベルも日本語は □ になる。したがって画面上の文字はすべて英語にしている。
      日本語表示にしたい場合は `sudo apt install fonts-noto-cjk` を入れてから文字列を戻すこと。
"""
import os                                                   # OSライブラリ
import select                                               # 端末入力の有無の確認
import signal                                               # Ctrl+C などのシグナル処理
import sys                                                  # Pythonを扱うライブラリ
import time                                                 # 時間

import matplotlib                                           # グラフライブラリ
matplotlib.use('TkAgg')                                     # Tkinterへ埋め込む描画バックエンド
from matplotlib.backends.backend_agg import FigureCanvasAgg  # 画像保存専用のキャンバス
from matplotlib.backends.backend_tkagg import (             # Tkinter用のキャンバスとツールバー
    FigureCanvasTkAgg, NavigationToolbar2Tk,
)
from matplotlib.figure import Figure                        # pyplotを使わないOO APIの図
from matplotlib.ticker import FuncFormatter                 # 目盛り表記の指定
import numpy as np                                          # 数学計算

import tkinter as tk                                        # GUIライブラリ
from tkinter import ttk                                     # GUI部品

# ==============================================================================
# ビューアの設定（チューニング要素）
# ==============================================================================
SNAPSHOT_PATH = "/tmp/el_optimization_snapshot.npz"         # test_code2.py の SNAPSHOT_PATH と同じにすること
POLL_INTERVAL_MS = 1000                                     # スナップショットの更新確認の間隔[ms]
QUIT_CHECK_MS = 200                                         # 端末でEnterが押されたかの確認間隔[ms]
N_DOF = 24                                                  # 自由度数
PWM_LIMIT = 255.0                                           # PWMの上下限（入力グラフの補助線）
LEGEND_FONTSIZE = 7                                         # 凡例の文字サイズ
SHOW_IDENT_INPUT = True                                     # システムモデル同定に使った入力を入力グラフに重ねて描くか（'actual_pwm'のときのみ描画対象）

# 同定入力モードの表示名と座標系（test_code2.py の IDENTIFICATION_INPUT_MODE に対応）
IDENT_MODE_LABEL = {
    'actual_pwm': ('actual PWM', 'z = P - Pi'),             # 実PWM − 保持PWM で同定（初期位置を0とする系）
    'ff_input':   ('FF input',   'y = P - Pf'),             # 印加したFF入力で同定（最終位置を0とする偏差系）
}

# ==============================================================================
# 終了時のグラフ保存の設定（チューニング要素）
# ==============================================================================
SAVE_FIGSIZE = (7.0, 4.5)                                   # 保存する画像1枚のサイズ[inch]
SAVE_DPI = 100                                              # 保存する画像の解像度


# ==============================================================================
# 最適化ビューア本体
# ==============================================================================
class OptimizationViewer:
    # コンストラクタ
    def __init__(self, root, path, save_dir=None):          # 引数(Tkのルートウィンドウ, スナップショットのパス, 終了時の保存先フォルダ)
        self.root = root                                    # ルートウィンドウを保存
        self.path = path                                    # 監視するスナップショットのパス
        self.save_dir = save_dir                            # 終了時にグラフを保存するフォルダ（Noneなら保存しない）
        self.data = None                                    # 読み込んだスナップショット
        self.mtime = None                                   # 読み込み済みスナップショットの更新時刻
        self.records = {}                                   # 保存用に蓄積したスナップショット {外側ループ番号: {...}}
        self.session = None                                 # 表示中のスナップショットの実行ID
        self.start_time = time.time()                       # ビューアの起動時刻（これより古いファイルは前回の実行の残りとみなす）
        self.quit_requested = False                         # 終了要求（Enter または Ctrl+C）が来たか
        self.watch_stdin = True                             # 端末入力を監視するか（EOFなら止める）
        self.dof = tk.IntVar(value=1)                       # 表示中の自由度番号（1始まり）
        self.auto = tk.BooleanVar(value=True)               # 自動更新のON/OFF

        self.root.title("Optimal Control Monitor")          # ウィンドウタイトル
        self.root.geometry("1600x950")                      # ウィンドウサイズ

        self._build_ui()                                    # 画面部品を作る
        self._poll()                                        # スナップショットの監視を開始する
        # 終了要求の監視は after で予約する（ここで直接呼ぶと、mainloop開始前に quit() が
        # 空振りしたうえで再予約されず、終了できなくなる）
        self.root.after(QUIT_CHECK_MS, self._check_quit)

    # 端末でEnterが押されたかを定期的に確認する関数
    def _check_quit(self):
        """端末の入力を待ち受け、Enterが押されていたら mainloop を抜ける。

        input() を別スレッドで待つとインタプリタ終了時に stdin のロックが解放されず
        異常終了するため、Tkのタイマーから select で「入力があるか」だけを見る。
        Ctrl+C（SIGINT）で立てられた終了フラグもここで拾う。
        """
        if self.watch_stdin and not self.quit_requested:
            try:
                if select.select([sys.stdin], [], [], 0)[0]:                                 # 入力が来ているか（待たない）
                    if sys.stdin.readline() == '':                                           # EOF（端末が無い起動）なら監視をやめるだけ
                        self.watch_stdin = False
                    else:                                                                    # Enterが押された
                        self.quit_requested = True
            except Exception:                                                                # stdinが使えない環境では監視しない
                self.watch_stdin = False
        try:
            if self.quit_requested:
                self.root.quit()                                                             # mainloopを抜ける（保存はmain側のfinallyで行う）
                return
            self.root.after(QUIT_CHECK_MS, self._check_quit)
        except tk.TclError:                                                                  # 既にウィンドウが閉じられている場合
            pass

    # ------------------------------------------------------------------
    # 画面の組み立て
    # ------------------------------------------------------------------
    def _build_ui(self):
        # 上段: 自由度の切り替えと更新設定
        bar = ttk.Frame(self.root, padding=6)                                               # 上段のフレーム
        bar.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(bar, text="DOF:").pack(side=tk.LEFT)
        ttk.Button(bar, text="<", width=3, command=lambda: self._step_dof(-1)).pack(side=tk.LEFT)    # 1つ前の自由度へ
        spin = ttk.Spinbox(                                                                 # 自由度を直接指定するスピンボックス
            bar, from_=1, to=N_DOF, width=5, textvariable=self.dof,
            command=self._redraw, justify=tk.CENTER,
        )
        spin.pack(side=tk.LEFT, padx=2)
        spin.bind("<Return>", lambda _e: self._redraw())                                    # Enterでも反映する
        ttk.Button(bar, text=">", width=3, command=lambda: self._step_dof(+1)).pack(side=tk.LEFT)    # 1つ次の自由度へ
        ttk.Label(bar, text="(arrow keys)").pack(side=tk.LEFT, padx=4)

        ttk.Separator(bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=12)
        ttk.Checkbutton(bar, text="Auto update", variable=self.auto).pack(side=tk.LEFT)      # 自動更新のON/OFF
        ttk.Button(bar, text="Reload now", command=lambda: self._reload(force=True)).pack(side=tk.LEFT, padx=6)

        self.status = ttk.Label(bar, text="Waiting for snapshot...")                        # 状態表示ラベル
        self.status.pack(side=tk.LEFT, padx=16)

        # 中段: 左に数値一覧、右にグラフ
        main = ttk.Frame(self.root)                                                         # 中段のフレーム
        main.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.text = tk.Text(main, width=46, font="TkFixedFont", wrap=tk.NONE, bg="#f5f5f5")  # 数値一覧のパネル（等幅・折り返しなし）
        self.text.pack(side=tk.LEFT, fill=tk.Y)
        self.text.configure(state=tk.DISABLED)                                              # 読み取り専用にする

        right = ttk.Frame(main)                                                             # グラフ側のフレーム
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(12, 8), dpi=100)                                          # 図（pyplotは使わない）
        self.axes = self.fig.subplots(2, 3).ravel()                                          # 2行3列の6枚のグラフ
        self.fig.subplots_adjust(left=0.055, right=0.90, top=0.94, bottom=0.07, hspace=0.42, wspace=0.62)   # 右軸のラベルが隣や図の外と重ならない間隔
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)                              # Tkinterへ埋め込む
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        NavigationToolbar2Tk(self.canvas, right)                                             # 拡大・保存などのツールバー

        self.root.bind("<Left>", lambda _e: self._step_dof(-1))                              # ←キーで自由度を戻す
        self.root.bind("<Right>", lambda _e: self._step_dof(+1))                             # →キーで自由度を進める

    # 表示する自由度を1つ動かす関数
    def _step_dof(self, delta):                                                              # 引数(移動量)
        self.dof.set(int(np.clip(self.dof.get() + delta, 1, N_DOF)))                         # 1～24の範囲に収める
        self._redraw()

    # ------------------------------------------------------------------
    # スナップショットの監視と読み込み
    # ------------------------------------------------------------------
    def _poll(self):
        """一定間隔でスナップショットの更新を確認し続ける（監視ループ本体）"""
        self._reload()                                                                       # 更新を確認する
        self.root.after(POLL_INTERVAL_MS, self._poll)                                        # 次回の監視を予約する（ここだけで予約すること）

    def _reload(self, force=False):                                                          # 引数(自動更新OFFでも読み込むか)
        """スナップショットが更新されていれば読み直して再描画する。

        スナップショットは test_code2.py が終了してもファイルとして残るため、ビューアの
        起動より前に書かれたファイルは「前回の実行の残り」とみなして読み込まない。
        これをしないと、起動直後に前回の実行のデータが表示され、そのまま保存対象にも
        入ってしまう（さらに内側ループ番号が衝突して今回のデータを弾いてしまう）。
        """
        try:
            if not ((self.auto.get() or force) and os.path.exists(self.path)):
                return
            mtime = os.path.getmtime(self.path)                                              # 更新時刻を取得
            if mtime < self.start_time:                                                      # ビューア起動より前＝前回の実行の残り
                if self.data is None:                                                        # まだ何も表示していないときだけ知らせる
                    old = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(mtime))
                    self.status.configure(
                        text=f"Waiting for the current run...   "
                             f"(the file on disk is from a previous run: {old} — ignored)")
                return
            if mtime == self.mtime and not force:                                            # 前回から更新されていなければ何もしない
                return
            with np.load(self.path, allow_pickle=False) as npz:
                self.data = {k: npz[k] for k in npz.files}                                    # メモリへ展開してからファイルを閉じる
            self.mtime = mtime
            self._record(self.data)                                                          # 終了時の保存用に蓄積する
            self._redraw()
        except Exception as exc:                                                             # 読み込み失敗時は次回の監視で再試行する
            self.status.configure(text=f"Load failed: {exc}")

    # 終了時のグラフ保存用に、読み込んだスナップショットを蓄積する関数
    def _record(self, d):                                                                    # 引数(読み込んだスナップショット)
        """外側ループごとに、各内側ループのスナップショットをそのまま貯める。

        スナップショットは最新の内側ループ1回分しか残らないため、内側ループごとの
        Time-POT/Time-PWM を保存するにはビューア側で貯めておく必要がある。
        自動更新をOFFにしている間や、ビューアを起動する前の内側ループは記録されない。
        """
        session = str(d['session']) if 'session' in d else ''                                # test_code2.py の実行ID
        if self.session is not None and session != self.session:                             # 別の実行が始まった（ビューアを開いたまま再実行した）
            print(f"新しい実行を検出しました（{self.session or '不明'} -> {session or '不明'}）。"
                  f"それまでに蓄積した保存対象を破棄します。")
            self.records.clear()                                                             # 実行をまたいだデータを混ぜない
        self.session = session

        outer, inner = int(d['outer']), int(d['inner'])
        rec = self.records.setdefault(outer, {'snaps': {}})
        rec['snaps'].setdefault(inner, d)                                                    # 同じ内側ループは1回だけ記録する

    # ------------------------------------------------------------------
    # 再描画
    # ------------------------------------------------------------------
    def _redraw(self):
        if self.data is None:                                                                # まだデータが無ければ何もしない
            return
        d = self.data                                                                        # スナップショット
        i = self.dof.get() - 1                                                               # 表示する自由度の添字（0始まり）

        mode_name, _coord = self._ident_mode(d)                                              # システムモデル同定に使った入力
        retry_used, retry_total = self._retry_counts(d)                                      # この測定に要した再測定回数・通算の再測定回数
        retry_txt = (f"   Re-measured {retry_used} (total {retry_total})"                    # 再測定があったときだけ状態表示に出す
                     if retry_used or retry_total else "")
        opt_txt = "" if self._is_opt(d, i) else f"   [DOF {i + 1}: not optimized]"           # 最適化対象外の自由度を表示しているときだけ出す
        self.status.configure(                                                               # 状態表示を更新（どの実行のデータかも表示する）
            text=(f"Run {self.session or '?'}   "
                  f"Outer {int(d['outer'])}/{int(d['max_outer'])}   "
                  f"Inner {int(d['inner'])}/{int(d['max_inner'])}   "
                  f"J = {float(d['total_J']):.4g}   Best J = {float(d['best_J']):.4g}   "
                  f"ID input: {mode_name}{retry_txt}{opt_txt}   "
                  f"Updated {str(d['time'])}")
        )

        for ax in list(self.fig.axes):                                                       # 前回 twinx() で作った右軸を取り除く
            if ax not in self.axes:                                                          # （ax.clear() では右軸が消えず、再描画のたびに積み重なるため）
                ax.remove()
        for ax in self.axes:                                                                 # 6枚のグラフを消してから描き直す
            ax.clear()
        self._plot_response(self.axes[0], d, i)                                              # 応答フィット
        self._plot_input(self.axes[1], d, i)                                                 # 入力
        self._plot_target_history(self.axes[2], d, i)                                        # 目標モデルパラメータの推移
        self._plot_system_history(self.axes[3], d, i)                                        # システムモデルパラメータの推移
        self._plot_extrema_history(self.axes[4], d, i)                                       # 極値の推移
        self._plot_cost_history(self.axes[5], d, i)                                          # 評価関数値・残差の推移
        self.canvas.draw_idle()                                                              # キャンバスを更新

        self._update_text(d, i)                                                              # 数値一覧を更新

    # システムモデル同定に使った入力の種類を取り出す関数
    @staticmethod
    def _ident_mode(d):                                                                      # 引数(スナップショット)
        """('actual PWM' などの表示名, 'z = P - Pi' などの座標系) を返す。

        同定入力を切り替えられるようになる前のスナップショットには 'ident_mode' が無い。
        そのころは実PWMでの同定しか無かったので 'actual_pwm' とみなす。
        """
        mode = str(d['ident_mode']) if 'ident_mode' in d else 'actual_pwm'
        return IDENT_MODE_LABEL.get(mode, (mode, '?'))                                       # 未知の値はそのまま表示する

    # その自由度が最適制御の対象かどうかを取り出す関数
    @staticmethod
    def _is_opt(d, i):                                                                       # 引数(スナップショット, 自由度の添字)
        """test_code2.py の OPT_DOF_IDS にその自由度が含まれていたかを返す。

        対象外の自由度は、実測POT値以外（目標モデル・システムモデル・FF・u_opt・J）が
        いっさい計算されておらずNaNで埋まっているため、グラフにも数値一覧にも出さない。
        対象自由度を指定できるようになる前のスナップショットには 'opt_dof' が無いので、
        そのころは全自由度が対象だったものとして True を返す。
        """
        if 'opt_dof' not in d:
            return True
        mask = np.asarray(d['opt_dof'], dtype=bool)                                          # 自由度ごとの最適化対象フラグ
        return bool(mask[i]) if i < mask.size else True

    # 最適制御の対象外の自由度に対して、表示枠だけを残したグラフを描く関数
    def _plot_not_optimized(self, ax, title, xlabel, ylabel):                                # 引数(グラフ, タイトル, 横軸ラベル, 縦軸ラベル)
        """レイアウト・体裁（タイトルと軸ラベル）は対象自由度と同じまま、中身を空にする。

        対象外の自由度では計算そのものを行っていないので、描くべき値が存在しない。
        グラフを消してしまうと自由度を切り替えるたびに画面構成が変わって見比べにくいため、
        枠と見出しは残したうえで「最適化していない」ことだけを本文に出す。
        """
        ax.text(0.5, 0.5, 'Not optimized\n(measured POT only)', transform=ax.transAxes,
                ha='center', va='center', fontsize=10, color='gray')
        ax.set_title(title, fontsize=10)
        ax.set_xlabel(xlabel); ax.set_ylabel(ylabel)
        ax.set_xticks([]); ax.set_yticks([])                                                 # 目盛りは意味を持たないので消す
        ax.grid(False)

    # 再測定の回数を取り出す関数
    @staticmethod
    def _retry_counts(d):                                                                  # 引数(スナップショット)
        """(この測定に要した再測定回数, この実行での通算の再測定回数) を返す。

        再測定を実装する前のスナップショットにはどちらのキーも無いので0として扱う。
        """
        used = int(d['retry_used']) if 'retry_used' in d else 0
        total = int(d['retry_total']) if 'retry_total' in d else 0
        return used, total

    # 右軸(twinx)を作る関数
    def _make_twin(self, ax, ylabel, color=None):                                            # 引数(左軸, 右軸のラベル, ラベルと目盛りの色)
        """右軸を作り、目盛りを各ラベルが桁を含む自己完結した表記にする。

        既定のフォーマッタは値が大きいと "1e8" のような倍率を軸の外側へ別に描くが、
        その文字が隣のグラフに隠れて読めなくなる（目盛りが 0.0〜1.0 に見えてしまう）ため。
        """
        ax2 = ax.twinx()
        if color is None:                                                                    # 色指定なし（set_ylabelはcolor=Noneを受け付けない）
            ax2.set_ylabel(ylabel)
        else:
            ax2.set_ylabel(ylabel, color=color)
            ax2.tick_params(axis='y', labelcolor=color)
        ax2.yaxis.set_major_formatter(FuncFormatter(lambda v, _pos: f"{v:.4g}"))              # 例: 8.298e+07
        return ax2

    # 実際にプロットしたデータの範囲に合わせて縦軸の目盛りを決める関数
    def _fit_ylim(self, ax, series, margin=0.08):                                            # 引数(グラフ, 範囲に含めるデータ列, 上下に足す余白の割合)
        """縦軸をプロットしたデータの範囲に合わせる。

        axhline で引いた補助線（PWMの±255、目標位置）は matplotlib の自動スケールに
        含まれてしまい、データが小さいときに波形が潰れて形が見えなくなる。そこで
        補助線を除いたデータだけから範囲を決め直す。NaN は無視する。
        """
        vals = np.concatenate([np.asarray(s, dtype=float).ravel() for s in series])
        vals = vals[np.isfinite(vals)]                                                       # NaN・infは範囲計算から除く
        if vals.size == 0:
            return
        lo, hi = float(vals.min()), float(vals.max())
        pad = (hi - lo) * margin if hi > lo else max(abs(hi) * 0.1, 1e-3)                     # データが一定値のときも潰れないようにする
        ax.set_ylim(lo - pad, hi + pad)

    # 描画済みの線から凡例を作る関数
    def _add_legend(self, ax, twin=None, ncol=1):                                            # 引数(グラフ, 右軸, 凡例の列数)
        lines = list(ax.get_lines()) + (list(twin.get_lines()) if twin is not None else [])   # 左右の軸の凡例を1つにまとめる
        lines += list(ax.patches)                                                             # 欠測を示す帯（axvspan）も凡例に載せる
        lines = [ln for ln in lines if not str(ln.get_label()).startswith('_')]               # 補助線（ラベル無し）は除く
        self._legend_from_handles(ax, lines, ncol=ncol, twin=twin)

    # 凡例がプロット線と重ならないよう、凡例の高さぶんだけ縦軸を広げる関数
    def _legend_from_handles(self, ax, handles, ncol=1, twin=None):                          # 引数(グラフ, 凡例に載せる線, 凡例の列数, 右軸)
        """凡例をグラフ上部に置き、その高さのぶんだけ縦軸の上端を広げる。

        凡例の実寸をレンダラから測って必要な分だけ広げるので、固定の余白率と違って
        データの表示領域を無駄に狭めない。実寸が取れない場合は既定値で広げる。
        対数軸のときは比で広げる。
        """
        if not handles:
            return
        leg = ax.legend(handles=handles, fontsize=LEGEND_FONTSIZE, ncol=ncol,
                        loc='upper center', framealpha=0.85)

        try:                                                                                 # 凡例の下端を軸座標(0～1)で実測する
            bbox = leg.get_window_extent(ax.get_figure().canvas.get_renderer())              # 画面用・保存用どちらの図でも動くよう ax から図を辿る
            frac = bbox.transformed(ax.transAxes.inverted()).y0 - 0.02                        # 凡例の下端よりわずかに下まででデータを収める
        except Exception:
            frac = 0.75                                                                       # 実測できないときの既定値
        frac = float(np.clip(frac, 0.45, 0.98))                                               # 広げすぎ・狭めすぎを防ぐ

        for a in (ax, twin):                                                                 # データの上端が frac の高さに来るよう縦軸を広げる
            if a is None:
                continue
            lo, hi = a.get_ylim()
            if a.get_yscale() == 'log' and lo > 0 and hi > lo:
                a.set_ylim(lo, lo * (hi / lo) ** (1.0 / frac))
            elif hi > lo:
                a.set_ylim(lo, lo + (hi - lo) / frac)

    # 実測・システムモデル・目標モデルの応答を描く関数
    def _plot_response(self, ax, d, i):
        """受信が途切れた区間（y_gap）は実測ではなく補間の直線なので、実測の線としては描かない。

        描いてしまうと「実機は振動しているのにグラフは水平で、途中から急に振動する」という
        誤解を招く形になる。線を切って背景に帯を出し、欠測だと一目で分かるようにする。

        最適制御の対象外の自由度（test_code2.py の OPT_DOF_IDS に無い自由度）では、
        目標モデル・システムモデルが同定されていないので実測POT値だけを描く。
        """
        is_opt = self._is_opt(d, i)                                                          # この自由度が最適制御の対象か
        t = d['t']
        y_act = np.array(d['y_data'][i], dtype=float)                                        # 実測POT値
        gap = np.asarray(d['y_gap'][i], dtype=bool) if 'y_gap' in d else None                # 欠測区間（古いスナップショットには無い）
        if gap is not None and gap.any():
            y_act[gap] = np.nan                                                              # 補間で作った区間は線を切る
            for k, (lo, hi) in enumerate(self._mask_spans(t, gap)):                          # 欠測区間を帯で示す
                ax.axvspan(lo, hi, color='tab:red', alpha=0.12, zorder=0,
                           label='No data (interpolated)' if k == 0 else '_nolegend_')       # 凡例には1つだけ載せる
        mode_name, _coord = self._ident_mode(d)                                              # このシステムモデルを同定した入力
        ax.plot(t, y_act, label='Actual', linewidth=1.2)                                     # 実測POT値
        series = [y_act]                                                                     # 縦軸の範囲に含めるデータ（対象外は実測のみ）
        if is_opt:                                                                           # 最適制御の対象のときだけモデル応答を描く
            ax.plot(t, d['y_tgt'][i], '--', label='Target model', linewidth=1.2)             # 目標モデル応答
            ax.plot(t, d['y_sys'][i], ':', label=f'System model (ID: {mode_name})',          # システムモデル応答（保存した図だけを見ても同定入力が分かるよう凡例に出す）
                    linewidth=1.5)
            series += [d['y_tgt'][i], d['y_sys'][i]]
        ax.axhline(float(d['Pf'][i]), color='gray', linewidth=0.7, alpha=0.6)                # 目標位置
        if is_opt:                                                                           # 最適制御の対象
            title = f"DOF {i + 1}  Response fit  (J = {float(d['J'][i]):.4g})"
        else:                                                                                # 最適化対象外（実測POT値のみ）
            title = f"DOF {i + 1}  Measured POT  (not optimized)"
        if gap is not None and gap.any():                                                    # 欠測があればタイトルにも出す
            title += f"   [no data: {gap.mean() * 100:.0f}%]"
        ax.set_title(title, fontsize=10)
        ax.set_xlabel('Time [s]'); ax.set_ylabel('POT')
        ax.grid(alpha=0.3)
        self._fit_ylim(ax, series)                                                           # 描いた応答に合わせて縦軸を決める
        self._add_legend(ax, ncol=3)

    # 真になっている区間を (開始時刻, 終了時刻) の一覧へ変換する関数
    @staticmethod
    def _mask_spans(t, mask):                                                                # 引数(時間軸, マスク)
        spans = []                                                                           # 空のリスト
        edges = np.flatnonzero(np.diff(np.concatenate(([0], mask.astype(np.int8), [0]))))    # 立ち上がり・立ち下がりの位置
        for lo, hi in zip(edges[0::2], edges[1::2]):                                         # 2つずつ取り出して1区間にする
            spans.append((float(t[lo]), float(t[min(hi, t.size - 1)])))
        return spans

    # この内側ループで使用したu_optと、実際に印加したFF入力を描く関数
    def _plot_input(self, ax, d, i):
        """u_optと印加FFはどちらも1つ前の内側ループで計算された値（＝今回ロボットを動かした入力）。

        システムモデル同定に使った入力（u_ident）も重ねて描く。'ff_input' モードでは
        u_ident は印加したFFそのもので線が完全に重なるため、線を増やさず凡例で示す。

        最適制御の対象外の自由度はFF入力を0にしたままu_optも計算していないので、
        表示枠だけを残して中身は描かない。
        """
        if not self._is_opt(d, i):                                                           # 最適制御の対象外の自由度
            self._plot_not_optimized(ax, 'Applied input', 'Time [s]', 'PWM')
            return
        t = d['t']
        mode = str(d['ident_mode']) if 'ident_mode' in d else 'actual_pwm'                   # システムモデル同定に使った入力
        mode_name, _coord = self._ident_mode(d)
        u_opt = d['u_opt_used'][i]                                                           # 今回のFFの元になったu_opt（初回同定時はNaN）
        has_u_opt = bool(np.any(np.isfinite(u_opt)))                                         # 初回同定かどうかの判定
        if has_u_opt:
            ax.plot(t, u_opt, label='u_opt (used for this FF)', linewidth=1.2)               # 最適制御に使用したu_opt
        ff_label = ('FF sent to robot  = system ID input' if mode == 'ff_input'              # FF入力で同定した場合は同じ線が同定入力でもある
                    else 'FF sent to robot')
        ax.plot(t, d['u_ff_applied'][i], '--', label=ff_label, linewidth=1.5)                # 実際にロボットへ送信したFF入力
        show_ident = (SHOW_IDENT_INPUT and mode != 'ff_input' and 'u_ident' in d)             # 実PWMで同定した場合だけ別の線として描く
        if show_ident:
            ax.plot(t, d['u_ident'][i], color='tab:green', alpha=0.7, linewidth=1.0,          # システムモデル同定に実際に使った入力
                    label=f'System ID input ({mode_name} - hold)')
        t1, y1, t2, y2 = d['extrema'][i]
        ax.plot([t1, t2], [y1, y2], 'o', color='red', markersize=6, label='FF extrema t1, t2')   # 印加したFFの極値
        ax.axhline(PWM_LIMIT, color='gray', linewidth=0.7, alpha=0.6)                        # PWM上限（範囲外なら画面に出ない）
        ax.axhline(-PWM_LIMIT, color='gray', linewidth=0.7, alpha=0.6)                       # PWM下限（範囲外なら画面に出ない）
        ax.axvline(float(d['T']), color='gray', linewidth=0.7, alpha=0.6)                    # FF入力時間T
        x_max = min(float(d['T']) * 1.6, float(t[-1]))                                        # FF区間まわりを拡大表示
        ax.set_xlim(0, x_max)
        shown = t <= x_max                                                                    # 表示している時間範囲だけで縦軸を決める
        series = [d['u_ff_applied'][i][shown], [y1, y2]]                                      # 印加FFと極値
        if has_u_opt:
            series.append(u_opt[shown])                                                       # u_optも範囲に含める
        if show_ident:
            series.append(np.asarray(d['u_ident'][i], dtype=float)[shown])                    # 同定入力も範囲に含める（枠外に消えないように）
        self._fit_ylim(ax, series)                                                            # PWMの±255線に引きずられないようにする
        if has_u_opt:                                                                        # 通常のループ
            title = (f"Applied input  (fit residual = {float(d['fit_res'][i]):.4g}, "
                     f"CMA restarts = {int(d['fit_restart'][i])})")
        else:                                                                                # 初回同定（最適制御前）
            title = "Applied input  (initial excitation FF, before optimal control)"
        ax.set_title(title, fontsize=10)
        ax.set_xlabel('Time [s]'); ax.set_ylabel('PWM')
        ax.grid(alpha=0.3)
        self._add_legend(ax, ncol=2)

    # 履歴グラフの共通設定を行う関数
    def _setup_history_axis(self, ax, n, title):                                             # 引数(グラフ, 履歴の点数, タイトル)
        ax.set_title(title, fontsize=10)
        ax.set_xlabel('Inner loop')
        ax.grid(alpha=0.3)
        if n <= 12:                                                                          # 点数が少ないときは全ループ番号を目盛りにする
            ax.set_xticks(np.arange(n))

    # 目標モデルパラメータ(T1, wn)の推移を描く関数
    def _plot_target_history(self, ax, d, i):
        if not self._is_opt(d, i):                                                           # 最適制御の対象外の自由度は同定していない
            self._plot_not_optimized(ax, 'Target model params used for u_opt', 'Inner loop', 'T1 [s]')
            return
        h = d['hist_tgt'][:, i, :]                                                         # (履歴数, 2)
        x = np.arange(len(h))
        ax.plot(x, h[:, 0], 'o-', color='tab:blue', markersize=3, label='T1')                # 1次遅れの時定数
        ax.set_ylabel('T1 [s]', color='tab:blue'); ax.tick_params(axis='y', labelcolor='tab:blue')
        ax2 = self._make_twin(ax, 'wn [rad/s]', color='tab:red')                             # 右軸
        ax2.plot(x, h[:, 1], 's-', color='tab:red', markersize=3, label='wn')                # 固有振動数
        self._setup_history_axis(ax, len(h), 'Target model params used for u_opt')
        self._add_legend(ax, twin=ax2, ncol=2)

    # システムモデルパラメータ(T1, zeta, wn, b0)の推移を描く関数
    def _plot_system_history(self, ax, d, i):
        if not self._is_opt(d, i):                                                           # 最適制御の対象外の自由度は同定していない
            self._plot_not_optimized(ax, 'System model params used for u_opt', 'Inner loop', 'T1, wn, b0 (symlog)')
            return
        h = d['hist_sys'][:, i, :]                                                         # (履歴数, 4)
        x = np.arange(len(h))
        for k, name in [(0, 'T1 [s]'), (2, 'wn [rad/s]'), (3, 'b0')]:                        # T1・wnは正、b0は符号自由なので symlog 軸に載せる
            ax.plot(x, h[:, k], 'o-', markersize=3, label=name)
        ax.set_yscale('symlog', linthresh=1e-2); ax.set_ylabel('T1, wn, b0 (symlog)')        # 桁が離れるうえ b0 は負にもなるため symlog
        ax2 = self._make_twin(ax, 'zeta', color='tab:purple')                                # 減衰比は範囲が狭いので線形の右軸
        ax2.plot(x, h[:, 1], 's-', color='tab:purple', markersize=3, label='zeta')
        ax2.axhline(0.0, color='tab:purple', lw=0.8, ls=':')                                 # zeta=0: 振動モードの安定限界（下回ると振幅が増大する）
        ax2.axhline(1.0, color='tab:purple', lw=0.8, ls='--')                                # zeta=1: 臨界減衰（上回ると3実極で振動しない）
        self._setup_history_axis(ax, len(h), 'System model params used for u_opt')
        self._add_legend(ax, twin=ax2, ncol=4)

    # FF極値(t1, t2, y1, y2)の推移を描く関数
    def _plot_extrema_history(self, ax, d, i):
        if not self._is_opt(d, i):                                                           # 最適制御の対象外の自由度はFFを与えていない
            self._plot_not_optimized(ax, 'FF extrema sent to robot', 'Inner loop', 't1, t2 [s]')
            return
        h = d['hist_ext'][:, i, :]                                                         # (履歴数, 4) = t1,y1,t2,y2
        x = np.arange(len(h))
        ax.plot(x, h[:, 0], 'o-', color='tab:blue', markersize=3, label='t1')                # 1つ目の極値時刻
        ax.plot(x, h[:, 2], 'o--', color='tab:cyan', markersize=3, label='t2')               # 2つ目の極値時刻
        ax.set_ylabel('t1, t2 [s]'); ax.set_ylim(0, float(d['T']))
        ax2 = self._make_twin(ax, 'y1, y2 [PWM]')                                            # 右軸に極値の大きさ
        ax2.plot(x, h[:, 1], 's-', color='tab:red', markersize=3, label='y1')
        ax2.plot(x, h[:, 3], 's--', color='tab:orange', markersize=3, label='y2')
        self._setup_history_axis(ax, len(h), 'FF extrema sent to robot')
        self._add_legend(ax, twin=ax2, ncol=4)

    # 評価関数値Jの推移を描く関数
    def _plot_cost_history(self, ax, d, i):
        """J は常に「実測データと同定した目標モデルとの差」。Inner loop=0 は初回同定時のJ

        受信途切れで測り直した内側ループには印を付ける。失敗した測定そのものは同定に
        使われず履歴にも入らないので、印が付いた点は「何回か測り直した末に採用された
        測定」を表す（点の数＝内側ループの回数であることは変わらない）。

        最適制御の対象外の自由度はJを計算していない（内側ループの評価値にも算入されない）ため、
        表示枠だけを残して中身は描かない。
        """
        if not self._is_opt(d, i):                                                           # 最適制御の対象外の自由度
            self._plot_not_optimized(ax, 'J = actual vs target model', 'Inner loop', 'Squared error (log)')
            return
        x = np.arange(len(d['hist_J']))
        ax.plot(x, d['hist_J'][:, i], 'o-', markersize=3, label='J (this DOF)')              # このDOFの評価関数値
        if 'hist_retry' in d:                                                                # 再測定の履歴があるスナップショットの場合
            retry = np.asarray(d['hist_retry'], dtype=int)[:len(x)]                          # 各内側ループの再測定回数
            k = np.flatnonzero(retry > 0)                                                    # 測り直した内側ループ
            if k.size:
                ax.plot(x[k], d['hist_J'][k, i], 'o', markersize=9, markerfacecolor='none',  # 印（中抜きの赤丸）を重ねる
                        markeredgecolor='tab:red', label='Re-measured (gap detected)')
        ax.set_yscale('log'); ax.set_ylabel('Squared error (log)')
        self._setup_history_axis(ax, len(x), 'J = actual vs target model')
        self._add_legend(ax, ncol=1)

    # ------------------------------------------------------------------
    # 終了時のグラフ保存
    # ------------------------------------------------------------------
    def save_all(self):
        """蓄積したスナップショットから、自由度ごと・外側ループごとにグラフを保存する。

        保存先は  <実行ディレクトリ>/YYYYMMDD_HHMMSS/DOF01/OuterLoop00/*.png
          ・time_pot_InnerLoopNN.png / time_pwm_InnerLoopNN.png … 内側ループごとに1枚ずつ
          ・inner_model / inner_system / inner_extrema / inner_error … その外側ループの
            最後のスナップショット（＝終了・中断時点の最新の履歴グラフ）から1枚ずつ
        画面表示と同じ描画関数を使うので、保存された図は画面で見えていた図と一致する。
        画面用とは別に保存専用の Agg の図を使うため、Tkが閉じた後でも動く。
        """
        if self.save_dir is None:
            return
        if not self.records:
            print("保存するデータがありません（スナップショットを1度も読み込んでいません）。")
            return

        fig = Figure(figsize=SAVE_FIGSIZE, dpi=SAVE_DPI)                                     # 保存専用の図（使い回して高速化）
        FigureCanvasAgg(fig)                                                                 # レンダラを持たせる
        total = sum((2 * len(r['snaps']) + 4) * N_DOF for r in self.records.values())        # 保存するファイル総数
        done = 0
        t0 = time.perf_counter()
        print(f"\nグラフを保存します: {self.save_dir}  （{total} ファイル）")

        for outer in sorted(self.records):
            snaps = self.records[outer]['snaps']                                             # {内側ループ番号: スナップショット}
            inners = sorted(snaps)                                                           # その外側ループで記録できた内側ループ
            last = snaps[inners[-1]]                                                         # 履歴グラフ用の最新スナップショット
            for i in range(N_DOF):
                folder = os.path.join(self.save_dir, f"DOF{i + 1:02d}", f"OuterLoop{outer - 1:02d}")
                os.makedirs(folder, exist_ok=True)

                for inner in inners:                                                         # 内側ループごとに Time-POT / Time-PWM を1枚ずつ
                    d = snaps[inner]
                    tag = f"InnerLoop{inner - 1:02d}"                                        # 履歴グラフの横軸と同じ0始まりの番号
                    self._save_fig(fig, folder, f"time_pot_{tag}.png",
                                   lambda ax, d=d, i=i, inner=inner: self._titled(
                                       ax, self._plot_response, d, i, inner))
                    self._save_fig(fig, folder, f"time_pwm_{tag}.png",
                                   lambda ax, d=d, i=i, inner=inner: self._titled(
                                       ax, self._plot_input, d, i, inner))
                    done += 2

                for fname, draw in (                                                         # 履歴グラフは外側ループごとに1枚ずつ
                    ('inner_model.png',   self._plot_target_history),
                    ('inner_system.png',  self._plot_system_history),
                    ('inner_extrema.png', self._plot_extrema_history),
                    ('inner_error.png',   self._plot_cost_history),
                ):
                    self._save_fig(fig, folder, fname, lambda ax, f=draw: f(ax, last, i))
                    done += 1

            print(f"  OuterLoop{outer - 1:02d}: DOF01-{N_DOF:02d} 完了  ({done}/{total})")

        print(f"保存完了: {self.save_dir}  （{done} ファイル / {time.perf_counter() - t0:.1f} 秒）")

    # 保存用: 図を作り直して1枚保存する関数
    @staticmethod
    def _save_fig(fig, folder, fname, draw):                                                 # 引数(使い回す図, 保存先, ファイル名, 描画関数)
        fig.clear()                                                                          # 前の図（右軸・凡例含む）を消す
        draw(fig.add_subplot(111))
        fig.tight_layout()
        fig.savefig(os.path.join(folder, fname))

    # 保存用: 画面用の描画関数を呼び、タイトルに内側ループ番号を足す関数
    @staticmethod
    def _titled(ax, plot_func, d, i, inner):                                                 # 引数(グラフ, 画面用の描画関数, スナップショット, 自由度の添字, 内側ループ番号)
        plot_func(ax, d, i)
        ax.set_title(f"{ax.get_title()}   [Inner loop {inner - 1}]", fontsize=10)

    # ------------------------------------------------------------------
    # 数値一覧パネル
    # ------------------------------------------------------------------
    def _update_text(self, d, i):
        if not self._is_opt(d, i):                                                           # 最適制御の対象外の自由度
            self._write_text(self._not_optimized_lines(d, i))                                # 同定・最適化していない項目は出さない
            return
        a, b, c, dd, e = d['ff'][i]
        t1, y1, t2, y2 = d['extrema'][i]
        T1_id, wn_id = d['tgt_params_id'][i]                                                 # 今回同定した目標モデル（Time-POTの破線）
        T1s_id, zeta_id, wns_id, b0_id = d['sys_params_id'][i]                               # 今回同定したシステムモデル（Time-POTの点線）
        T1_us, wn_us = d['tgt_params_used'][i]                                               # 今回のFFを作るのに使った目標モデル
        T1s_us, zeta_us, wns_us, b0_us = d['sys_params_used'][i]                             # 今回のFFを作るのに使ったシステムモデル
        Pi, Pf = float(d['Pi'][i]), float(d['Pf'][i])
        restart = int(d['fit_restart'][i])                                                   # -1は退避形を使用, -2は初回同定で該当なし, -3はu_opt≈0で探索不要
        restart_msg = {                                                                      # 負値は回数ではなく状態を表すのでメッセージへ置き換える
            -1: "  fallback used",
            -3: "     zero u_opt",
        }.get(restart, f"{restart:14d}" if restart >= 0 else "             --")

        mode_name, coord = self._ident_mode(d)                                               # システムモデル同定に使った入力と座標系
        retry_used, retry_total = self._retry_counts(d)                                      # この測定に要した再測定回数・通算の再測定回数
        max_retry = int(d['max_retry']) if 'max_retry' in d else 0                           # 再測定の上限回数

        def num(v, fmt):                                                                     # 値が無い（NaN）ときは -- と表示する関数
            return f"{v:{fmt}}" if np.isfinite(v) else "            --"

        lines = [
            f" DOF {i + 1}   (POT array index {int(d['pot_idx'][i])})",
            "=" * 44,
            f" Outer loop      {int(d['outer'])} / {int(d['max_outer'])}",
            f" Inner loop      {int(d['inner'])} / {int(d['max_inner'])}",
            f" Updated         {str(d['time'])}",
            "",
            " [System ID input]",
            f"   Mode          {mode_name:>14s}",
            f"   Coordinates   {coord:>14s}",
            f"   Re-measured   {retry_used:>10d} / {max_retry:<2d}",
            f"   Re-meas total {retry_total:14d}",
            "",
            " [Position]",
            f"   Initial   Pi  {Pi:14.2f}",
            f"   Target    Pf  {Pf:14.2f}",
            f"   Offset    y0  {Pi - Pf:14.2f}",
            "",
            " [Identified from this run]  -> Time-POT curves",
            "   Target model  1/((T1 s+1)(s^2+2 wn s+wn^2))",
            f"     T1          {T1_id:14.6f}",
            f"     wn          {wn_id:14.6f}",
            "   System model  b0/((T1 s+1)(s^2+2 zeta wn s+wn^2))",
            f"     T1          {T1s_id:14.6f}",
            f"     zeta        {zeta_id:14.6f}",
            f"     wn          {wns_id:14.6f}",
            f"     b0          {b0_id:14.4f}",
            f"     ID residual {float(d['id_res_sys'][i]):14.4e}",
            "",
            " [Used for the applied FF]  -> history plots",
            f"     T1          {num(T1_us, '14.6f')}",
            f"     wn          {num(wn_us, '14.6f')}",
            f"     T1(sys)     {num(T1s_us, '14.6f')}",
            f"     zeta        {num(zeta_us, '14.6f')}",
            f"     wn(sys)     {num(wns_us, '14.6f')}",
            f"     b0          {num(b0_us, '14.4f')}",
            "",
            " [FF sent to robot]  f(t)=a t^5+...+e t",
            f"   a             {a:14.4e}",
            f"   b             {b:14.4e}",
            f"   c             {c:14.4e}",
            f"   d             {dd:14.4e}",
            f"   e             {e:14.4e}",
            f"   T             {float(d['T']):14.4f}",
            "",
            " [Extrema]  0 < t1 < t2 < T",
            f"   t1            {t1:14.4f}",
            f"   y1            {y1:14.2f}",
            f"   t2            {t2:14.4f}",
            f"   y2            {y2:14.2f}",
            "",
            " [Fit quality]",
            f"   FF fit res.   {num(float(d['fit_res'][i]), '14.4e')}",
            f"   CMA restarts  {restart_msg}",
            "",
            " [Cost]  J = actual vs target model",
            f"   J (this DOF)  {float(d['J'][i]):14.4e}",
            f"   J (all DOF)   {float(d['total_J']):14.4e}",
            f"   Best J        {float(d['best_J']):14.4e}",
        ]

        self._write_text(lines)                                                              # 数値一覧を書き込む

    # 最適制御の対象外の自由度に表示する数値一覧を作る関数
    def _not_optimized_lines(self, d, i):                                                    # 引数(スナップショット, 自由度の添字)
        """対象外の自由度は実測POT値しか無いので、位置と実測の状況だけを同じ体裁で並べる。

        目標モデル・システムモデル・FF・極値・近似残差・評価関数Jはいずれも計算されて
        いない（スナップショットではNaN）ため、欄そのものを出さない。
        """
        Pi, Pf = float(d['Pi'][i]), float(d['Pf'][i])
        y_act = np.asarray(d['y_data'][i], dtype=float)                                      # 実測POT値
        gap = np.asarray(d['y_gap'][i], dtype=bool) if 'y_gap' in d else None                # 欠測区間
        finite = y_act[np.isfinite(y_act)]                                                   # 受信できた実測値だけ

        def num(v, fmt):                                                                     # 値が無い（NaN）ときは -- と表示する関数
            return f"{v:{fmt}}" if np.isfinite(v) else "            --"

        return [
            f" DOF {i + 1}   (POT array index {int(d['pot_idx'][i])})",
            "=" * 44,
            f" Outer loop      {int(d['outer'])} / {int(d['max_outer'])}",
            f" Inner loop      {int(d['inner'])} / {int(d['max_inner'])}",
            f" Updated         {str(d['time'])}",
            "",
            " [Optimal control]",
            "   This DOF is NOT in OPT_DOF_IDS.",
            "   FF input is kept at 0 and no model",
            "   identification / optimization is run.",
            "   Only the measured POT is recorded.",
            "",
            " [Position]",
            f"   Initial   Pi  {Pi:14.2f}",
            f"   Target    Pf  {Pf:14.2f}",
            f"   Offset    y0  {Pi - Pf:14.2f}",
            "",
            " [Measured POT]",
            f"   Start         {num(float(finite[0]) if finite.size else np.nan, '14.2f')}",
            f"   End           {num(float(finite[-1]) if finite.size else np.nan, '14.2f')}",
            f"   Min           {num(float(finite.min()) if finite.size else np.nan, '14.2f')}",
            f"   Max           {num(float(finite.max()) if finite.size else np.nan, '14.2f')}",
            f"   No data       {(gap.mean() * 100 if gap is not None else 0.0):13.1f}%",
        ]

    # 数値一覧パネルへ書き込む関数
    def _write_text(self, lines):                                                            # 引数(表示する行のリスト)
        self.text.configure(state=tk.NORMAL)                                                 # 一時的に書き込み可能にする
        self.text.delete("1.0", tk.END)                                                      # 内容を消す
        self.text.insert(tk.END, "\n".join(lines))                                           # 数値一覧を書き込む
        self.text.configure(state=tk.DISABLED)                                               # 読み取り専用へ戻す


# ==============================================================================
# エントリーポイント
# ==============================================================================
def main(args=None):
    path = sys.argv[1] if len(sys.argv) > 1 else SNAPSHOT_PATH                               # 引数でパスを上書きできる
    save_dir = os.path.join(os.getcwd(), time.strftime('%Y%m%d_%H%M%S'))                     # 実行ディレクトリ直下・実行開始日時のフォルダ

    print(f"【監視するスナップショット】 {path}")
    print("test_code2.py 側で ENABLE_SNAPSHOT = True になっていることを確認してください。")
    print(f"【終了時のグラフ保存先】 {save_dir}")
    print("終了するには、この端末で Enter を押してください（ウィンドウを閉じても保存されます）。")

    root = tk.Tk()                                                                           # Tkinterを起動
    viewer = OptimizationViewer(root, path, save_dir)                                        # ビューアを生成

    # Ctrl+C（SIGINT）・kill（SIGTERM）でも保存してから終わるようにする
    #   Tkのmainloopは KeyboardInterrupt を握りつぶしてループを続けてしまうため、
    #   例外に頼らずシグナルハンドラで終了フラグを立てる。保存中に再度Ctrl+Cが来ても
    #   フラグを立てるだけなので、保存処理が中断されることはない。
    def on_signal(_signum, _frame):
        print("\n終了要求を受け取りました。グラフを保存します…")
        viewer.quit_requested = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, on_signal)

    try:
        root.mainloop()                                                                      # イベントループ開始
    except KeyboardInterrupt:                                                                # 念のため（通常はシグナルハンドラ側で処理される）
        pass
    finally:
        viewer.save_all()                                                                    # 正常終了・中断のどちらでも必ず保存する
        try:
            root.destroy()                                                                   # ウィンドウを閉じる（既に閉じていれば何もしない）
        except tk.TclError:
            pass


if __name__ == '__main__':
    main()
