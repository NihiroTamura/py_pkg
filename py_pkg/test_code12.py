#!/usr/bin/env python3
"""test_code11.py（ADRC＋FF最適制御・n次遅れ系版）の最適化ビューア

test_code11.py が内側ループごとに書き出す .npz スナップショットの更新時刻を監視し、
更新されていれば読み直して再描画する別プロセスのプログラム。最適化本体を止めないよう、
描画はこちらのプロセスだけで行う。

  起動: python3 <このファイル>              （引数でスナップショットのパスを上書きできる）
        ros2 run py_pkg test_code12

表示する8種類のグラフ（画面は2行4列。time_adrc と time_simPWM は枠の中を上下2段に分ける）
  1. time_pot      応答（Time-POT）。Q_k の切替時刻 k_s と ±10% 帯も描く
  2. time_pwm      ELで計算した u_FF_opt と、それを近似した5次関数 u_FF（極値に印と値）
  3. time_adrc     同定判定に使ったモデルの u_ADRC replica・z3 replica を、ROS2実測値と上下2段で重ねる
  4. time_simPWM   ELの解 u_opt = u_FF_opt + u_ADRC と、その u_opt を与えたシミュレーションの z3 sim
  5. inner_cost    評価関数の推移
  6. inner_extrema 印加したFF極値の推移
  7. inner_model   u_opt の計算に使った目標モデル（T1, wn）の推移（test_code5.py と同じ表示）
  8. inner_system  u_opt の計算に使ったシステムモデル（T1, zeta, wn, tau, b0）の推移（test_code5.py と同じ表示）
  最適化対象外の自由度は表示枠を保ったまま実測POT値だけを描く。

test_code8.py（test_code7.py 用ビューア）からの変更点は、システムモデルの次数が可変になったことに
伴う2箇所だけである。
  1. inner_system（_plot_system_history）
       θ = [T1, zeta, wn, tau, b0] は次数によらず5要素なので、遅れ連鎖の時定数 tau も
       T1・wn・b0 と同じ symlog 軸に載せる。遅れ連鎖が無い（n=3）ときの tau は式に現れない
       ダミー値なので描かない。
  2. 数値一覧の [Identified models] 節（_sys_model_lines）
       システムモデルの次数が可変になったので、sys_struct = [次数 n, 遅れ連鎖の長さ n-3, 0, 0]
       と、次数に依らない派生量 [DCゲイン, 支配極の周波数, 支配極の減衰比, 等価むだ時間] を
       併せて表示する。目標モデルは test_code7.py と同じ [T1, wn] の2要素である。

監視するパスは test_code7.py 用（..._v7.npz）や test_code3.py 用（/tmp/el_optimization_snapshot.npz）
とは分けてある（同時に起動したときに互いのスナップショットを誤読しないようにするため）。

日本語フォントが入っていない環境では matplotlib のラベルが □ になるため、
画面上の文字はすべて英語にしている（コメントは日本語のまま）。
"""
import os                                                       # ファイルの存在確認・更新時刻の取得を行う標準ライブラリ
import sys                                                      # コマンドライン引数の取得を行う標準ライブラリ
import signal                                                   # Ctrl+C / kill を捕まえて終了処理を行うための標準ライブラリ
import time                                                     # 保存先フォルダ名に使う日時の取得

import numpy as np                                              # 数値計算ライブラリ

import matplotlib                                               # グラフライブラリ
matplotlib.use('TkAgg')                                         # Tkinterへ埋め込む描画バックエンド
from matplotlib.backends.backend_agg import FigureCanvasAgg     # 画像保存専用のキャンバス
from matplotlib.backends.backend_tkagg import (                 # Tkinter用のキャンバスとツールバー
    FigureCanvasTkAgg, NavigationToolbar2Tk,
)
from matplotlib.figure import Figure                            # pyplotを使わないOO APIの図

import tkinter as tk                                            # GUIライブラリ
from tkinter import ttk                                         # Tkinterのテーマ付きウィジェット

SNAPSHOT_PATH = "/tmp/el_optimization_snapshot_v11.npz"         # 監視するスナップショット（test_code11.py 側と同じパスにすること）
POLL_MS = 400                                                   # スナップショットの更新を確認する間隔 [ms]
N_DOF_ALL = 24                                                  # 表示する自由度の総数
PWM_LIMIT = 255.0                                               # 実機PWMの絶対上限 [PWM]（グラフの補助線に使う）
SETTLE_BAND = 0.10                                              # 整定判定の帯幅（test_code11.py の SETTLE_BAND と合わせること）

# ==============================================================================
# 終了時に保存するグラフの設定
#   スナップショットは最新の内側ループ1回分しか残らないため、内側ループごとのグラフを
#   保存するにはビューア側で読み込んだものを貯めておく必要がある（_record 参照）。
#   保存先は  <実行ディレクトリ>/YYYYMMDD_HHMMSS/DOF01/OuterLoop00/*.png
#     ・time_pot_InnerLoopNN.png / time_pwm_InnerLoopNN.png / time_adrc_InnerLoopNN.png /
#       time_simPWM_InnerLoopNN.png
#         … 内側ループごとに1枚ずつ
#     ・inner_cost / inner_extrema / inner_model / inner_system
#         … その外側ループの最後のスナップショット（＝終了・中断時点の最新の履歴グラフ）から1枚ずつ
# ==============================================================================
SAVE_FIGSIZE = (7.0, 4.5)                                       # 保存する画像1枚のサイズ [inch]
SAVE_DPI = 100                                                  # 保存する画像の解像度
SAVE_MARGIN = dict(left=0.13, right=0.87, top=0.90, bottom=0.13)  # 保存する図の余白（左右に軸ラベルぶんを確保する）
SAVE_HSPACE = 0.45                                              # 上下2段のグラフを保存するときの段の間隔（各段の見出しが重ならない幅）

# 5次フィットの経路を表す値と、その意味（test_code11.py の last_fit_mode と対応）
FIT_MODE_TEXT = {
    0: "L2 projection",                                         # 閉形式の最小二乗で決まった（正常）
    1: "extrema param (windowed LS)",                           # 極値(t1,t2,y1,y2)を座標にして、最初の2極値が入る窓の中で最小二乗を解いた解
    -1: "FAILED -> seed shape",                                 # 退避経路でも見つからず初期励振形へ退避した
    -2: "n/a",                                                  # 該当なし（初回同定など）
    -3: "f=0 (no motion)",                                      # 動かす必要がない自由度
}


# ==============================================================================
# ビューア本体
# ==============================================================================
class OptimizationViewer:
    # 表示するグラフの一覧（画面の並び順＝左上から右下へ。保存名・描画関数名・段数・保存単位）
    #   段数 2 のグラフは枠の中を上下2段に分け、描画関数には (上段, 下段) の2つの軸を渡す。
    #   保存単位 'inner' は内側ループごとに1枚、'outer' は外側ループごとに1枚保存する。
    PANELS = [
        ('time_pot',      '_plot_response',        1, 'inner'),                             # 1: 応答（Time-POT）
        ('time_pwm',      '_plot_input',           1, 'inner'),                             # 2: 入力（u_FF_opt と u_FF）
        ('time_adrc',     '_plot_adrc',            2, 'inner'),                             # 3: ADRC照合（u_ADRC と z3）
        ('time_simPWM',   '_plot_sim_pwm',         2, 'inner'),                             # 4: ELの解のシミュレーション（u_opt と z3 sim）
        ('inner_cost',    '_plot_cost_history',    1, 'outer'),                             # 5: 評価関数の推移
        ('inner_extrema', '_plot_extrema_history', 1, 'outer'),                             # 6: FF極値の推移
        ('inner_model',   '_plot_target_history',  1, 'outer'),                             # 7: 目標モデルの推移
        ('inner_system',  '_plot_system_history',  1, 'outer'),                             # 8: システムモデルの推移
    ]

    # コンストラクタ
    def __init__(self, root, path, save_dir=None):                                          # 引数(Tkのルートウィンドウ, スナップショットのパス, 終了時の保存先フォルダ)
        self.root = root                                                                    # Tkのルートウィンドウ
        self.path = path                                                                    # 監視するスナップショットのパス
        self.save_dir = save_dir                                                            # 終了時にPNGを保存するフォルダ
        self.data = None                                                                    # 読み込んだスナップショット（辞書）
        self.mtime = None                                                                   # 最後に読み込んだファイルの更新時刻
        self.dof = 0                                                                        # 表示している自由度の添字（0始まり）
        self.auto = tk.BooleanVar(value=True)                                               # 自動更新のON/OFF
        self.quit_requested = False                                                         # シグナルで終了を要求されたか
        self.records = {}                                                                   # 保存用に蓄積したスナップショット {外側ループ番号: {'snaps': {内側ループ番号: データ}}}
        self.session = None                                                                 # 蓄積中のスナップショットの実行ID（別の実行が始まったら捨てる）
        self.root.title("test_code11 optimization viewer")                                   # ウィンドウのタイトル
        self._build_ui()                                                                    # ウィジェットを作る
        self._reload(force=True)                                                            # 起動時に1度読み込む
        self.root.after(POLL_MS, self._poll)                                                # 定期監視を開始
        self.root.after(200, self._check_quit)                                              # シグナルによる終了要求の監視を開始

    # シグナルによる終了要求を監視する関数
    def _check_quit(self):                                                                  # 引数なし（Tkのafterから定期的に呼ばれる）
        """Tkのmainloopは KeyboardInterrupt を握りつぶすため、フラグを定期的に見て終了する"""
        if self.quit_requested:                                                             # 終了要求が来ている場合
            self.root.quit()                                                                # mainloopを抜ける（保存は main() の finally で行う）
            return
        self.root.after(200, self._check_quit)                                              # まだなら次回の確認を予約する

    # ウィジェットを作る関数
    def _build_ui(self):                                                                    # 引数なし（コンストラクタから1度だけ呼ばれる）
        bar = ttk.Frame(self.root)                                                          # 上部の操作バー
        bar.pack(side=tk.TOP, fill=tk.X, padx=4, pady=2)                                    # 操作バーを上端へ横いっぱいに配置する

        ttk.Button(bar, text="<", width=3, command=lambda: self._step_dof(-1)).pack(side=tk.LEFT)   # 1つ前の自由度へ
        self.dof_var = tk.StringVar(value="DOF 01")                                         # 表示中の自由度を示す文字列
        ttk.Label(bar, textvariable=self.dof_var, width=8, anchor=tk.CENTER).pack(side=tk.LEFT)   # 表示中の自由度番号
        ttk.Button(bar, text=">", width=3, command=lambda: self._step_dof(+1)).pack(side=tk.LEFT)   # 1つ次の自由度へ
        ttk.Checkbutton(bar, text="Auto reload", variable=self.auto).pack(side=tk.LEFT, padx=8)     # 自動更新のON/OFF
        ttk.Button(bar, text="Reload now", command=lambda: self._reload(force=True)).pack(side=tk.LEFT, padx=6)  # 手動で読み直す
        self.status = tk.StringVar(value="waiting for snapshot...")                         # 状態表示（更新時刻など）
        ttk.Label(bar, textvariable=self.status).pack(side=tk.LEFT, padx=12)                # ループ番号・J・更新時刻の表示欄

        body = ttk.Frame(self.root)                                                         # 本体（左：数値一覧 / 右：グラフ）
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)                                   # 本体を残り全体へ広げる

        self.text = tk.Text(body, width=46, font=("monospace", 9), state=tk.DISABLED)        # 数値一覧のテキスト欄
        self.text.pack(side=tk.LEFT, fill=tk.Y, padx=(4, 0), pady=4)                        # 数値一覧を左端へ縦いっぱいに配置する

        right = ttk.Frame(body)                                                             # グラフ側のフレーム
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)                                 # グラフを残り全体へ広げる
        self.fig = Figure(figsize=(15.5, 7.6), dpi=100)                                     # 図（pyplotを使わないOO API）
        gs = self.fig.add_gridspec(2, 4, left=0.045, right=0.985, top=0.92, bottom=0.07,    # 2行4列の8枠（右軸の目盛りが隣の枠のラベルと重ならない間隔）
                                   hspace=0.50, wspace=0.40)
        self.panel_axes = []                                                                # 枠ごとの軸のリスト（1段なら1個、2段なら上下2個）
        for k, (_, _, n_rows, _) in enumerate(self.PANELS):                                 # 並び順どおりに枠を作る
            cell = gs[k // 4, k % 4]                                                        # その枠の位置
            if n_rows == 1:                                                                 # 1段のグラフ
                self.panel_axes.append([self.fig.add_subplot(cell)])
            else:                                                                           # 上下2段のグラフ（時間軸を共有する）
                sub = cell.subgridspec(2, 1, hspace=0.35)
                ax0 = self.fig.add_subplot(sub[0])
                self.panel_axes.append([ax0, self.fig.add_subplot(sub[1], sharex=ax0)])
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)                             # Tkinterへ埋め込むキャンバス
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)            # 描画キャンバスを配置する
        NavigationToolbar2Tk(self.canvas, right)                                            # 拡大・保存などのツールバー

    # 表示する自由度を切り替える関数
    def _step_dof(self, delta):                                                             # 引数(移動量)
        self.dof = (self.dof + delta) % N_DOF_ALL                                           # 端まで行ったら巻き戻す
        self.dof_var.set(f"DOF {self.dof + 1:02d}")                                         # 表示を更新
        self._redraw()                                                                      # 再描画

    # スナップショットの更新を定期的に確認する関数
    def _poll(self):                                                                        # 引数なし（Tkのafterから定期的に呼ばれる）
        if self.auto.get():                                                                 # 自動更新がONの場合
            self._reload()                                                                  # 更新されていれば読み直す
        self.root.after(POLL_MS, self._poll)                                                # 次回の確認を予約する

    # スナップショットを読み直す関数
    def _reload(self, force=False):                                                         # 引数(自動更新OFFでも読み込むか)
        if not os.path.exists(self.path):                                                   # ファイルがまだ無い場合
            self.status.set(f"snapshot not found: {self.path}")
            return
        try:
            mtime = os.path.getmtime(self.path)                                             # ファイルの更新時刻
        except OSError:                                                                     # 差し替えの瞬間に読むと失敗しうる
            return
        if not force and self.mtime is not None and mtime <= self.mtime:                    # 更新されていない場合
            return
        try:
            with np.load(self.path, allow_pickle=False) as z:                               # スナップショットを読み込む
                self.data = {k: z[k] for k in z.files}                                      # 辞書へ展開する
        except Exception as exc:                                                            # 書き込み途中を掴んだ場合（次回のpollで読み直す）
            self.status.set(f"read failed (retry): {exc}")
            return
        self.mtime = mtime                                                                  # 読み込んだ更新時刻を記録
        self._record(self.data)                                                             # 内側ループごとの保存用に貯めておく
        d = self.data
        self.status.set(
            f"outer {int(d['outer'])}/{int(d['max_outer'])}  "
            f"inner {int(d['inner'])}/{int(d['max_inner'])}  "
            f"J={float(d['total_J']):.4g}  best={float(d['best_J']):.4g}  "
            f"updated {str(d['time'])}"
        )
        self._redraw()                                                                      # 再描画

    # 読み込んだスナップショットを、内側ループごとの保存用に貯めておく関数
    def _record(self, d):                                                                   # 引数(読み込んだスナップショット)
        """外側ループごとに、各内側ループのスナップショットをそのまま貯める。

        スナップショットのファイルは最新の内側ループ1回分しか残らないため、内側ループごとの
        time_pot / time_pwm / time_adrc / time_simPWM を保存するにはビューア側で貯めておく必要がある。
        自動更新をOFFにしている間や、ビューアを起動する前の内側ループは記録されない。
        """
        session = str(d['session']) if 'session' in d else ''                               # test_code11.py の実行ID
        if self.session is not None and session != self.session:                            # 別の実行が始まった（ビューアを開いたまま再実行した）
            print(f"新しい実行を検出しました（{self.session or '不明'} -> {session or '不明'}）。"
                  f"それまでに蓄積した保存対象を破棄します。")
            self.records.clear()                                                            # 実行をまたいだデータを混ぜない
        self.session = session                                                              # 現在の実行IDを覚える
        outer, inner = int(d['outer']), int(d['inner'])                                     # 外側・内側ループ番号
        rec = self.records.setdefault(outer, {'snaps': {}})                                 # その外側ループの記録枠
        rec['snaps'].setdefault(inner, d)                                                   # 同じ内側ループは1回だけ記録する

    # 描画関数に軸を渡す関数（1段なら軸そのもの、2段なら (上段, 下段) を渡す）
    def _draw_panel(self, func_name, axes, d, i):                                           # 引数(描画関数名, 軸のリスト, スナップショット, 自由度の添字)
        func = getattr(self, func_name)                                                     # 描画関数
        func(axes[0] if len(axes) == 1 else tuple(axes), d, i)                              # 段数に合わせて軸を渡す

    # 全パネルを描き直す関数
    def _redraw(self):                                                                      # 引数なし（表示中の自由度を描き直す）
        if self.data is None:                                                               # まだ読み込めていない場合
            return
        d, i = self.data, self.dof                                                          # スナップショットと表示中の自由度
        for axes in self.panel_axes:                                                        # 全パネルを消す
            for ax in axes:
                ax.clear()                                                                  # 前回の描画内容を消す
                for tw in getattr(ax, '_twins', []):                                        # 前回作った右軸も消す
                    tw.remove()                                                             # 前回作った右軸を消す（残すと重なって増え続ける）
                ax._twins = []                                                              # 右軸の記録を初期化する
        for (_, func_name, _, _), axes in zip(self.PANELS, self.panel_axes):                # 並び順どおりに描く
            self._draw_panel(func_name, axes, d, i)
        opt = self._is_opt(d, i)                                                            # この自由度が最適化対象か
        self.fig.suptitle(
            f"DOF {i + 1:02d}   Pi={float(d['Pi'][i]):.0f} -> Pf={float(d['Pf'][i]):.0f} "
            f"(step={float(d['Pf'][i]) - float(d['Pi'][i]):+.0f} count)   T={float(d['T']):.2f}s   "
            f"{'[optimized]' if opt else '[NOT optimized - measurement only]'}",
            fontsize=11,
        )
        self._update_text(d, i)                                                             # 左の数値一覧を更新
        self.canvas.draw_idle()                                                             # 画面へ反映する

    # 右軸を作る関数
    def _make_twin(self, ax, ylabel, color=None):                                           # 引数(左軸, 右軸のラベル, ラベルと目盛りの色)
        tw = ax.twinx()                                                                     # 右軸を作る
        tw.set_ylabel(ylabel, fontsize=8, color=color or 'k')                               # ラベル
        tw.tick_params(axis='y', labelsize=7, colors=color or 'k')                          # 目盛り
        getattr(ax, '_twins', []).append(tw)                                                # 次回消せるよう控えておく
        return tw                                                                           # 作った右軸を返す

    # 補助線を無視して表示範囲を決める関数
    @staticmethod
    def _fit_ylim(ax, series, margin=0.08):                                                 # 引数(グラフ, 範囲に含めるデータ列, 上下に足す余白の割合)
        """axhline で引いた補助線（PWMの±255、目標位置）は自動スケールの対象になるため、
        補助線だけが遠くにあると実際のデータが潰れて見えなくなる。データ列だけから範囲を決める。
        """
        vals = np.concatenate([np.asarray(s, dtype=float).ravel() for s in series if s is not None])  # データ列を1本にまとめる
        vals = vals[np.isfinite(vals)]                                                      # NaN/Infを除く
        if vals.size == 0:                                                                  # 有効な値が無い場合
            return
        lo, hi = float(vals.min()), float(vals.max())                                       # データの上下端
        pad = max((hi - lo) * margin, 1e-9)                                                 # 余白
        ax.set_ylim(lo - pad, hi + pad)                                                     # データが収まる範囲へ設定する

    # その自由度が最適制御の対象かどうかを取り出す関数
    @staticmethod
    def _is_opt(d, i):                                                                      # 引数(スナップショット, 自由度の添字)
        """test_code11.py の OPT_DOF_IDS にその自由度が含まれていたかを返す（'opt' が無い古い形式は対象扱い）"""
        return float(d['opt'][i]) > 0.5 if 'opt' in d else True

    # 最適制御の対象外の自由度に対して、表示枠だけを残したグラフを描く関数（test_code5.py と同じ）
    @staticmethod
    def _plot_not_optimized(ax, title, xlabel, ylabel):                                     # 引数(グラフ, タイトル, 横軸ラベル, 縦軸ラベル)
        """レイアウト・体裁（タイトルと軸ラベル）は対象自由度と同じまま、中身を空にする。

        対象外の自由度では計算そのものを行っていないので、描くべき値が存在しない。
        グラフを消してしまうと自由度を切り替えるたびに画面構成が変わって見比べにくいため、
        枠と見出しは残したうえで「最適化していない」ことだけを本文に出す。
        """
        ax.text(0.5, 0.5, 'Not optimized\n(measured POT only)', transform=ax.transAxes,
                ha='center', va='center', fontsize=9, color='gray')
        ax.set_title(title, fontsize=9)
        ax.set_xlabel(xlabel, fontsize=8); ax.set_ylabel(ylabel, fontsize=8)
        ax.set_xticks([]); ax.set_yticks([])                                                # 目盛りは意味を持たないので消す
        ax.grid(False)

    # 実測とモデルの一致度（誤差のrms と実測の振れ幅）を求める関数
    @staticmethod
    def _match_stats(meas, model):                                                          # 引数(実測の系列, モデルの系列)
        """両方が有限値の点だけで rms(実測 - モデル) と std(実測) を返す（比べられなければ NaN）"""
        meas = np.asarray(meas, dtype=float); model = np.asarray(model, dtype=float)
        ok = np.isfinite(meas) & np.isfinite(model)                                         # 両方そろっている点
        if not ok.any():
            return float('nan'), float('nan')
        err = meas[ok] - model[ok]                                                          # 実測とモデルの差
        return float(np.sqrt(np.mean(err ** 2))), float(np.std(meas[ok]))

    # z3 を見やすい桁へそろえる倍率を決める関数
    @staticmethod
    def _z3_scale(series):                                                                  # 引数(z3 の系列のリスト)
        """z3 は 1e6〜1e7 の大きさなので、そのまま描くと軸の上に出る指数表記が見出しと重なる。
        10 のべき乗で割って描き、桁は軸ラベルに書く。戻り値は (倍率, 指数)。
        """
        vals = np.concatenate([np.asarray(s, dtype=float).ravel() for s in series])         # 全系列を1本にまとめる
        vals = np.abs(vals[np.isfinite(vals)])                                              # 有限値の絶対値
        if vals.size == 0 or vals.max() <= 0.0:                                             # 描くものが無い場合
            return 1.0, 0
        exp = int(np.floor(np.log10(vals.max())))                                           # 最大値の桁
        return 10.0 ** exp, exp

    # データが無いことを枠の中央に書く関数
    @staticmethod
    def _note_empty(ax, series, text):                                                      # 引数(グラフ, 判定に使う系列のリスト, 表示する文)
        """全系列が NaN（初回の内側ループなど）のときだけ、枠の中央に理由を出す"""
        if not any(np.isfinite(np.asarray(s, dtype=float)).any() for s in series):
            ax.text(0.5, 0.5, text, transform=ax.transAxes, ha='center', va='center', fontsize=8, color='gray')

    # 応答（Time-POT）を描く関数
    def _plot_response(self, ax, d, i):                                                     # 引数(描画先のグラフ, スナップショット, 自由度の添字)
        t = d['t']                                                                          # 時間軸 [s]
        y_data, y_sys, y_tgt = d['y_data'][i], d['y_sys'][i], d['y_tgt'][i]                 # 実測・システムモデル・目標モデル
        gap = d['y_gap'][i]                                                                 # 実測が無く補間の直線になっている区間
        y_show = np.where(gap, np.nan, y_data)                                              # 補間の直線は実測として描かない
        Pf = float(d['Pf'][i]); Pi = float(d['Pi'][i]); step = Pf - Pi                      # 目標位置・初期位置・目標変位
        ax.plot(t, y_show, 'k-', lw=1.2, label='measured')                                  # 実測POT値
        ax.plot(t, y_sys, 'b--', lw=1.0, label='system model')                              # 同定したシステムモデルの応答
        ax.plot(t, y_tgt, 'r:', lw=1.4, label='target model')                               # 同定した目標モデルの応答
        ax.axhline(Pf, color='g', lw=0.8, ls='-.')                                          # 目標位置
        band = SETTLE_BAND * abs(step)                                                      # 整定判定の帯幅
        if np.isfinite(band) and band > 0:                                                  # 目標変位がある場合だけ帯を描く
            ax.axhspan(Pf - band, Pf + band, color='g', alpha=0.08)                         # ±10% 帯
        k_s = float(d['k_s'][i]) if 'k_s' in d else np.nan                                  # Q_k の切替インデックス
        if np.isfinite(k_s) and 0 <= k_s < len(t):                                          # 切替時刻が有効な場合
            ax.axvline(t[int(k_s)], color='m', lw=1.0, ls='--')                             # k_s（整定区間の開始）
            ax.text(t[int(k_s)], ax.get_ylim()[1], ' k_s', color='m', fontsize=7, va='top')
        ax.axvline(float(d['T']), color='0.5', lw=0.8, ls=':')                              # FF入力の終了時刻 T
        ax.set_title('Response  (Time-POT)', fontsize=9)
        ax.set_xlabel('time [s]', fontsize=8); ax.set_ylabel('POT [count]', fontsize=8)
        ax.tick_params(labelsize=7); ax.grid(alpha=0.3)                                     # 目盛りの大きさとグリッド
        self._fit_ylim(ax, [y_show, y_sys, y_tgt, np.array([Pf])])                          # 補助線を無視して範囲を決める
        ax.legend(fontsize=6, loc='best')                                                   # 凡例（重ならない位置へ自動配置）

    # 入力（Time-PWM）を描く関数
    def _plot_input(self, ax, d, i):                                                        # 引数(描画先のグラフ, スナップショット, 自由度の添字)
        """ELで計算した u_FF_opt と、それを近似して今回ロボットへ送った5次関数 u_FF を重ねる。
        u_FF の極値2点に印を付け、その (時刻, 値) を書き添える。
        """
        t = d['t']                                                                          # 時間軸 [s]
        u_ff_opt = d['u_ff_opt_used'][i]                                                    # 今回印加したFFの元になった u_FF_opt（ELの解）
        u_ff = d['u_ff_applied'][i]                                                         # 今回ロボットへ送信した5次関数FF入力（u_FF_opt の近似）
        ax.plot(t, u_ff_opt, 'c-', lw=1.0, label='u_FF_opt (EL)')                            # ELが出した最適FF入力
        ax.plot(t, u_ff, 'r-', lw=1.4, label='u_FF (5th-order, sent)')                       # 実機へ送った5次関数FF
        t1, y1, t2, y2 = [float(v) for v in d['extrema'][i]]                                 # 印加したFFの極値
        ax.plot([t1, t2], [y1, y2], 'ro', ms=5)                                              # 極値マーカー
        ax.annotate(f'({t1:.3f}, {y1:+.1f})', (t1, y1), fontsize=6, xytext=(3, 4), textcoords='offset points')
        ax.annotate(f'({t2:.3f}, {y2:+.1f})', (t2, y2), fontsize=6, xytext=(3, -9), textcoords='offset points')
        ax.axhline(0.0, color='k', lw=0.6)                                                   # 0線
        ax.axvline(float(d['T']), color='0.5', lw=0.8, ls=':')                               # FF入力の終了時刻 T
        ax.set_title('Input  (Time-PWM)', fontsize=9)
        ax.set_xlabel('time [s]', fontsize=8); ax.set_ylabel('PWM', fontsize=8)
        ax.tick_params(labelsize=7); ax.grid(alpha=0.3)                                     # 目盛りの大きさとグリッド
        ax.set_xlim(0, min(float(d['T']) * 1.6, float(t[-1])))                               # FF区間まわりを拡大して見る
        self._fit_ylim(ax, [u_ff_opt, u_ff])                                                 # 補助線を無視して範囲を決める
        ax.legend(fontsize=6, loc='best')                                                   # 凡例（重ならない位置へ自動配置）

    # ADRC照合（同定判定に使ったモデル vs ROS2実測）を上下2段で描く関数
    def _plot_adrc(self, axes, d, i):                                                       # 引数((上段, 下段) のグラフ, スナップショット, 自由度の添字)
        """上段: u_ADRC、下段: z3。どちらも黒実線が ROS2実測、色付き破線が同定判定に使ったモデル。

        モデル側は、同定したシステムモデルを実測と同じ条件（印加したFF・目標変位・保持PWM・
        z3(0⁻)）でADRC込みの閉ループで回した値である（test_code11.py の閉ループ同定が合わせる対象）。
        段ごとに単位の違う量を分けて描き、見出しに誤差の rms と実測の振れ幅を出すので、
        一致しているかどうかが1目で分かる。ここが合っていないと、u_FF の最適化もその分だけ的外れになる。
        """
        ax_u, ax_z = axes                                                                    # 上段・下段
        t = d['t']                                                                           # 時間軸 [s]
        T = float(d['T'])                                                                    # FF入力の終了時刻

        u_meas, u_rep = d['u_adrc_meas'][i], d['u_adrc_cl'][i]                               # 実測ADRC出力・モデルのADRC出力
        rms_u, std_u = self._match_stats(u_meas, u_rep)                                      # 一致度
        ax_u.plot(t, u_meas, 'k-', lw=1.0, label='u_ADRC measured')                          # 実測ADRC出力（u_pwm - u_ff）
        ax_u.plot(t, u_rep, '--', color='tab:blue', lw=1.1, label='u_ADRC replica')          # 同定モデルの内部複製が出したADRC出力
        ax_u.axvline(T, color='0.5', lw=0.8, ls=':')                                         # FF入力の終了時刻 T
        ax_u.set_title(f'u_ADRC: measured vs replica   rms err {rms_u:.1f} / meas std {std_u:.1f} PWM', fontsize=8)
        ax_u.set_ylabel('PWM', fontsize=8)
        ax_u.tick_params(labelsize=7); ax_u.tick_params(labelbottom=False); ax_u.grid(alpha=0.3)
        self._fit_ylim(ax_u, [u_meas, u_rep])                                                # データだけから範囲を決める
        ax_u.legend(fontsize=6, loc='best')

        z3_meas = d['z3_meas'][i] if 'z3_meas' in d else np.full(len(t), np.nan)             # 実測 z3（ESO外乱推定値）
        z3_rep = d['z3_replica'][i] if 'z3_replica' in d else np.full(len(t), np.nan)        # 同定モデルの内部複製の ζ3
        rms_z, std_z = self._match_stats(z3_meas, z3_rep)                                    # 一致度（元の単位）
        sc, ex = self._z3_scale([z3_meas, z3_rep])                                           # 表示の桁
        ax_z.plot(t, z3_meas / sc, 'k-', lw=1.0, label='z3 measured')                        # 実測 z3
        ax_z.plot(t, z3_rep / sc, '--', color='tab:red', lw=1.1, label='z3 replica')         # 同定モデルの内部複製の ζ3
        ax_z.axvline(T, color='0.5', lw=0.8, ls=':')                                         # FF入力の終了時刻 T
        ax_z.set_title(f'z3: measured vs replica   rms err {rms_z:.3g} / meas std {std_z:.3g}', fontsize=8)
        ax_z.set_xlabel('time [s]', fontsize=8); ax_z.set_ylabel(f'z3 [1e{ex} count/s^2]' if ex else 'z3 [count/s^2]', fontsize=8)
        ax_z.tick_params(labelsize=7); ax_z.grid(alpha=0.3)
        ax_z.set_xlim(float(t[0]), float(t[-1]))                                             # 評価ホライズン全体を見る（上段も共有）
        self._fit_ylim(ax_z, [z3_meas / sc, z3_rep / sc])                                    # データだけから範囲を決める
        ax_z.legend(fontsize=6, loc='best')

    # ELの解（振動のない解が得られた場合のシミュレーション値）を上下2段で描く関数
    def _plot_sim_pwm(self, axes, d, i):                                                    # 引数((上段, 下段) のグラフ, スナップショット, 自由度の添字)
        """上段: ELで計算した総最適入力 u_opt = u_FF_opt + u_ADRC（絶対PWM）、
        下段: その u_opt を与えたシミュレーションの z3 sim。

        time_pwm の u_FF_opt と同じ回のELの解（＝今回印加したFFの元になった解）である。
        time_adrc の実測と見比べると、ELが想定した理想の動きと実機の差が分かる。
        初回の内側ループは初期励振FFを印加しているので、対応するELの解は無い。
        """
        ax_u, ax_z = axes                                                                    # 上段・下段
        t = d['t']                                                                           # 時間軸 [s]
        T = float(d['T'])                                                                    # FF入力の終了時刻
        u_hold = float(d['u_hold'][i]) if 'u_hold' in d else 0.0                             # 保持PWM（z形式の u_opt を絶対PWMへ戻す）
        if not np.isfinite(u_hold):                                                          # 保持PWMが無い（対象外の自由度など）場合
            u_hold = 0.0
        u_opt = np.asarray(d['u_opt_used'][i], dtype=float) + u_hold                          # u_opt = u_FF_opt + u_ADRC（絶対PWM）
        ax_u.plot(t, u_opt, '-', color='tab:purple', lw=1.2, label='u_opt = u_FF_opt + u_ADRC (EL)')
        ax_u.axvline(T, color='0.5', lw=0.8, ls=':')                                         # FF入力の終了時刻 T
        ax_u.set_title('EL solution (simulation): u_opt', fontsize=8)
        ax_u.set_ylabel('PWM', fontsize=8)
        ax_u.tick_params(labelsize=7); ax_u.tick_params(labelbottom=False); ax_u.grid(alpha=0.3)
        self._fit_ylim(ax_u, [u_opt])                                                        # データだけから範囲を決める
        self._note_empty(ax_u, [u_opt], 'n/a (no EL solution for this loop)')
        ax_u.legend(fontsize=6, loc='best')

        z3_sim = d['z3_sim_used'][i] if 'z3_sim_used' in d else np.full(len(t), np.nan)      # その u_opt を与えたときの ζ3
        sc, ex = self._z3_scale([z3_sim])                                                    # 表示の桁
        ax_z.plot(t, z3_sim / sc, '-', color='tab:green', lw=1.2, label='z3 sim (EL)')
        ax_z.axvline(T, color='0.5', lw=0.8, ls=':')                                         # FF入力の終了時刻 T
        ax_z.set_title('EL solution (simulation): z3 sim', fontsize=8)
        ax_z.set_xlabel('time [s]', fontsize=8); ax_z.set_ylabel(f'z3 [1e{ex} count/s^2]' if ex else 'z3 [count/s^2]', fontsize=8)
        ax_z.tick_params(labelsize=7); ax_z.grid(alpha=0.3)
        ax_z.set_xlim(float(t[0]), float(t[-1]))                                             # 評価ホライズン全体を見る（上段も共有）
        self._fit_ylim(ax_z, [z3_sim / sc])                                                  # データだけから範囲を決める
        self._note_empty(ax_z, [z3_sim], 'n/a (no EL solution for this loop)')
        ax_z.legend(fontsize=6, loc='best')

    # 履歴パネルの共通設定を行う関数
    @staticmethod
    def _setup_history_axis(ax, n, title):                                                    # 引数(グラフ, 履歴の点数, タイトル)
        ax.set_title(title, fontsize=9)                                                     # パネルのタイトル
        ax.set_xlabel('inner loop', fontsize=8)                                             # 横軸は内側ループ番号
        ax.tick_params(labelsize=7); ax.grid(alpha=0.3)                                     # 目盛りの大きさとグリッド
        if n > 0:
            ax.set_xlim(-0.5, max(n - 0.5, 0.5))                                            # 点が端で切れないよう少し余白を取る

    # 目標モデルパラメータ(T1, wn)の推移を描く関数（test_code5.py と同じ表示）
    def _plot_target_history(self, ax, d, i):                                               # 引数(描画先のグラフ, スナップショット, 自由度の添字)
        title = 'Target model params used for u_opt'
        if not self._is_opt(d, i):                                                          # 最適制御の対象外の自由度は同定していない
            self._plot_not_optimized(ax, title, 'inner loop', 'T1 [s]')
            return
        h = d['hist_tgt'][:, i, :] if d['hist_tgt'].size else np.zeros((0, 2))              # [T1, wn] の推移
        x = np.arange(len(h))                                                               # 内側ループ番号
        ax.plot(x, h[:, 0], 'o-', color='tab:blue', ms=3, label='T1')                       # 1次遅れの時定数
        ax.set_ylabel('T1 [s]', color='tab:blue', fontsize=8)
        ax.tick_params(axis='y', labelcolor='tab:blue')
        tw = self._make_twin(ax, 'wn [rad/s]', color='tab:red')                             # 右軸
        tw.plot(x, h[:, 1], 's-', color='tab:red', ms=3, label='wn')                        # 固有振動数
        h1, l1 = ax.get_legend_handles_labels(); h2, l2 = tw.get_legend_handles_labels()   # 左軸・右軸の凡例要素を集める
        ax.legend(h1 + h2, l1 + l2, fontsize=6, loc='best', ncol=2)                         # 左右の凡例をまとめて出す
        self._setup_history_axis(ax, len(h), title)

    # システムモデルパラメータ(T1, zeta, wn, tau, b0)の推移を描く関数（test_code5.py と同じ表示）
    def _plot_system_history(self, ax, d, i):                                               # 引数(描画先のグラフ, スナップショット, 自由度の添字)
        """θ = [T1, zeta, wn, tau, b0] の推移。T1・wn・tau・b0 を symlog の左軸、zeta を線形の右軸に描く。

        遅れ連鎖の長さ（sys_struct の2要素目 = n-3）が0のとき、tau は式に現れないダミー値なので描かない。
        """
        title = 'System model params used for u_opt'
        if not self._is_opt(d, i):                                                          # 最適制御の対象外の自由度は同定していない
            self._plot_not_optimized(ax, title, 'inner loop', 'T1, wn, tau, b0 (symlog)')
            return
        h = d['hist_sys'][:, i, :] if d['hist_sys'].size else np.zeros((0, 5))              # [T1, zeta, wn, tau, b0] の推移
        x = np.arange(len(h))                                                               # 内側ループ番号
        st = np.asarray(d['sys_struct'][i]).ravel() if 'sys_struct' in d else np.full(4, np.nan)  # モデル構造 [次数, 連鎖長, 0, 0]
        uses_tau = bool(np.isfinite(st[1]) and st[1] > 0)                                   # 遅れ連鎖があるか（tau が式に現れるか）
        series = [(0, 'T1 [s]'), (2, 'wn [rad/s]')] + ([(3, 'tau [s]')] if uses_tau else []) + [(4, 'b0')]
        for k, name in series:                                                              # T1・wn・tauは正、b0は符号自由なので symlog 軸に載せる
            ax.plot(x, h[:, k], 'o-', ms=3, label=name)
        ax.set_yscale('symlog', linthresh=1e-2)                                             # 桁が離れるうえ b0 は負にもなるため symlog
        ax.set_ylabel('T1, wn, tau, b0 (symlog)' if uses_tau else 'T1, wn, b0 (symlog)', fontsize=8)
        tw = self._make_twin(ax, 'zeta', color='tab:purple')                                # 減衰比は範囲が狭いので線形の右軸
        tw.plot(x, h[:, 1], 's-', color='tab:purple', ms=3, label='zeta')
        tw.axhline(0.0, color='tab:purple', lw=0.8, ls=':')                                 # zeta=0: 振動モードの安定限界（下回ると振幅が増大する）
        tw.axhline(1.0, color='tab:purple', lw=0.8, ls='--')                                # zeta=1: 臨界減衰（上回ると3実極で振動しない）
        h1, l1 = ax.get_legend_handles_labels(); h2, l2 = tw.get_legend_handles_labels()   # 左軸・右軸の凡例要素を集める
        ax.legend(h1 + h2, l1 + l2, fontsize=6, loc='best', ncol=5)                         # 左右の凡例をまとめて出す
        self._setup_history_axis(ax, len(h), title)

    # FF極値の推移を描く関数
    def _plot_extrema_history(self, ax, d, i):                                              # 引数(描画先のグラフ, スナップショット, 自由度の添字)
        ext = d['hist_ext'][:, i, :] if d['hist_ext'].size else np.zeros((0, 4))                # [t1, y1, t2, y2] の推移
        n = len(ext); x = np.arange(n)                                                          # 内側ループ番号
        if n:                                                                               # 履歴が1点でもある場合だけ描く
            ax.plot(x, ext[:, 1], 'r.-', lw=1.0, ms=4, label='y1')                              # 1つ目の極値
            ax.plot(x, ext[:, 3], 'b.-', lw=1.0, ms=4, label='y2')                              # 2つ目の極値
            ax.axhline(0.0, color='k', lw=0.6)                                                  # 0線
            tw = self._make_twin(ax, 't [s]', color='0.4')                                      # 右軸に極値時刻
            tw.plot(x, ext[:, 0], '.--', color='0.4', lw=0.8, ms=3, label='t1')
            tw.plot(x, ext[:, 2], '.--', color='0.7', lw=0.8, ms=3, label='t2')
            h1, l1 = ax.get_legend_handles_labels(); h2, l2 = tw.get_legend_handles_labels()   # 左軸・右軸の凡例要素を集める
            ax.legend(h1 + h2, l1 + l2, fontsize=6, loc='best')                             # 左右の凡例をまとめて出す
        ax.set_ylabel('PWM', fontsize=8)
        self._setup_history_axis(ax, n, 'FF extrema')

    # 評価関数の推移を描く関数
    def _plot_cost_history(self, ax, d, i):                                                 # 引数(描画先のグラフ, スナップショット, 自由度の添字)
        hJ = d['hist_J'][:, i] if d['hist_J'].size else np.zeros(0)                             # このDOFの評価関数値の推移
        tot = d['hist_total_J']; best = d['hist_best_J']                                        # 全DOF合計J・ベストJ の推移
        n = len(hJ); x = np.arange(n)                                                           # 内側ループ番号
        if n:                                                                               # 履歴が1点でもある場合だけ描く
            ax.semilogy(x, np.maximum(hJ, 1e-12), 'k.-', lw=1.2, ms=4, label='J (this DOF)')    # このDOFのJ
            ax.semilogy(x, np.maximum(tot, 1e-12), 'b.--', lw=0.9, ms=3, label='J (all opt DOF)')  # 合計J
            ax.semilogy(x, np.maximum(best, 1e-12), 'r.:', lw=0.9, ms=3, label='best J')         # ベストJ
            if 'threshold_J' in d:                                                               # 収束判定閾値
                ax.axhline(float(d['threshold_J']), color='g', lw=0.8, ls='-.')
            ax.legend(fontsize=6, loc='best')                                                   # 凡例（重ならない位置へ自動配置）
        ax.set_ylabel('J = sum (meas - target)^2', fontsize=8)
        self._setup_history_axis(ax, n, 'Cost history')

    # 数値を安全に文字列へ整形する関数
    @staticmethod
    def _num(v, fmt='{:14.4g}'):                                                                  # 引数(値, 書式)
        try:
            f = float(v)
        except (TypeError, ValueError):
            return f"{'n/a':>14s}"
        return f"{'n/a':>14s}" if not np.isfinite(f) else fmt.format(f)

    # スナップショットに無い項目を NaN として取り出す関数
    @staticmethod
    def _get(d, key, i):                                                                          # 引数(スナップショット, 項目名, 自由度の添字)
        return d[key][i] if key in d else np.nan

    # システムモデルの生パラメータと派生量を並べる関数
    def _sys_model_lines(self, d, i):                                                       # 引数(スナップショット, 自由度の添字)
        """[Identified models] 節のうち、システムモデルの部分の行を作る。

        test_code11.py のシステムモデルは

            G(s) = b0 / [ (T1 s + 1)(s² + 2ζω s + ω²)(τ s + 1)^(n-3) ]

        で、θ = [T1, zeta, wn, tau, b0] は次数によらず常に5要素である。
        次数と遅れ連鎖の長さは 'sys_struct' = [次数 n, 連鎖長 n-3, 0, 0] から読む。
        派生量 'sys_derived' = [DCゲイン, 支配極の周波数[Hz], 支配極の減衰比, 等価むだ時間[s]]
        も併せて出す（こちらが「モデルが物理的に妥当か」の主な判断材料になる）。

        b0 は「分子定数」であって DCゲインそのものではない点に注意（分母がモニックなので
        DCゲイン = b0/a0）。次数を上げると a0 が急激に大きくなるため b0 も大きな値になる。
        物理的な意味を見たいときは下の DC gain を見ること。
        """
        num = self._num                                                                         # 整形関数の別名
        par = np.asarray(d['sys_params_id'][i]).ravel()                                         # 同定したシステムモデルのパラメータ（NaNパディング済み）
        st = np.asarray(d['sys_struct'][i]).ravel() if 'sys_struct' in d else np.full(4, np.nan)  # モデル構造
        der = np.asarray(d['sys_derived'][i]).ravel() if 'sys_derived' in d else np.full(4, np.nan)  # 派生量
        order = int(st[0]) if np.isfinite(st[0]) else 0                                         # モデル次数 n
        chain = int(st[1]) if np.isfinite(st[1]) else 0                                         # 遅れ連鎖の長さ n-3

        lines = [f"   order / chain {order:>6d} /{chain:>3d}"]                                  # 次数と遅れ連鎖の長さ
        if par.size >= 5:                                                                       # パラメータがそろっている場合
            lines += [
                f"   sys T1        {num(par[0])} s",                                            # 1次遅れの時定数
                f"   sys zeta      {num(par[1])}",                                              # 減衰比
                f"   sys wn        {num(par[2])} rad/s",                                        # 固有振動数
                f"   sys tau       {num(par[3] * 1e3 if np.isfinite(par[3]) else np.nan)} ms",   # 遅れ連鎖の時定数
                f"   sys b0        {num(par[4], '{:14.4e}')}",                                  # 分子定数（DCゲインではない）
            ]
        lines += [
            f"   DC gain       {num(der[0])} count/PWM",                                        # 定常入力1PWMあたりの変位
            f"   pole freq     {num(der[1])} Hz",                                               # 支配極の周波数
            f"   pole damping  {num(der[2])}",                                                  # 支配極の減衰比
            f"   dead time     {num(der[3] * 1e3 if np.isfinite(der[3]) else np.nan, '{:14.1f}')} ms",  # 等価むだ時間 (n-3)*tau
        ]
        return lines

    # 左の数値一覧を更新する関数
    def _update_text(self, d, i):                                                           # 引数(スナップショット, 自由度の添字)
        num = self._num                                                                           # 整形関数の別名
        opt = self._is_opt(d, i)                                                                  # この自由度が最適化対象か
        ff = d['ff'][i]; ext = d['extrema'][i]                                                     # 印加したFFのパラメータと極値
        lines = [
            f" DOF {i + 1:02d}   {'OPTIMIZED' if opt else 'measurement only'}",
            f" session {str(d['session'])}",
            f" outer {int(d['outer'])}/{int(d['max_outer'])}   inner {int(d['inner'])}/{int(d['max_inner'])}",
            f" opt DOF = {list(np.asarray(d['opt_dof']).ravel()) if 'opt_dof' in d else '?'}",
            "",
            " [Motion]",
            f"   Pi            {num(d['Pi'][i])} count",
            f"   Pf            {num(d['Pf'][i])} count",
            f"   step          {num(float(d['Pf'][i]) - float(d['Pi'][i]))} count",
            f"   T             {num(d['T'])} s",
            f"   dt (CTRL_DT)  {num(d['dt']) if 'dt' in d else '':>14s} s",
            "",
            " [Applied FF]  f(t)=a t^5+..+e t",
            f"   a             {num(ff[0], '{:14.4e}')}",
            f"   b             {num(ff[1], '{:14.4e}')}",
            f"   c             {num(ff[2], '{:14.4e}')}",
            f"   d             {num(ff[3], '{:14.4e}')}",
            f"   e             {num(ff[4], '{:14.4e}')}",
            f"   t1 / y1       {num(ext[0], '{:6.3f}')} /{num(ext[1], '{:7.2f}')}",
            f"   t2 / y2       {num(ext[2], '{:6.3f}')} /{num(ext[3], '{:7.2f}')}",
            f"   fit residual  {num(d['fit_res'][i])}",
            f"   fit mode      {FIT_MODE_TEXT.get(int(d['fit_mode'][i]), '?'):>14s}",
        ]
        if opt:
            lines += [
                "",
                " [Identified models]",
                f"   tgt T1 / wn   {num(d['tgt_params_id'][i][0], '{:6.3f}')} /{num(d['tgt_params_id'][i][1], '{:7.3f}')}",
            ] + self._sys_model_lines(d, i) + [
                f"   id residual   {num(d['id_res_sys'][i])}",
                "",
                " [ADRC replica]  vs measured (identification)",
                f"   POT rms       {num(self._get(d, 'id_res_pot', i))} count",
                f"   u_ADRC rms    {num(self._get(d, 'id_res_pwm', i))} PWM",
                f"   z3 rms        {num(self._get(d, 'id_res_z3', i))}",
                f"   u_ADRC before {num(d['adrc_res_open'][i])} PWM",
                f"   closed-loop   {'adopted' if float(d['cl_ok'][i]) > 0.5 else 'NO':>14s}",
                f"   kick meas/th  {num(d['adrc_kick'][i])}",
                f"   delay         {num(d['adrc_delay'][i], '{:14.0f}')} samples",
                f"   rho(A_cl)     {num(d['cl_radius'][i], '{:14.5f}')}",
                "",
                " [Cost weighting]",
                f"   k_s (settle)  {num(d['k_s'][i], '{:14.0f}')} samples",
                f"   k_s time      {num(float(d['k_s'][i]) * float(d['dt'])) if 'dt' in d else '':>14s} s",
                f"   FF update     {'accepted' if float(d['accept'][i]) > 0.5 else 'SKIPPED':>14s}",
            ]
        lines += [
            "",
            " [Cost]  J = sum (measured - target)^2",
            f"   J (this DOF)  {num(d['J'][i], '{:14.4e}')}",
            f"   J (all opt)   {num(d['total_J'], '{:14.4e}')}",
            f"   Best J        {num(d['best_J'], '{:14.4e}')}",
        ]

        self.text.configure(state=tk.NORMAL)                                                      # 一時的に書き込み可能にする
        self.text.delete("1.0", tk.END)                                                           # 内容を消す
        self.text.insert(tk.END, "\n".join(lines))                                                # 数値一覧を書き込む
        self.text.configure(state=tk.DISABLED)                                                    # 読み取り専用へ戻す

    # 蓄積したスナップショットから、自由度ごと・外側ループごとにグラフを保存する関数
    def save_all(self):                                                                     # 引数なし（終了時に main() から呼ばれる）
        """終了時に、貯めておいた全スナップショットからグラフをPNGとして保存する。

        保存先は  <実行ディレクトリ>/YYYYMMDD_HHMMSS/DOF01/OuterLoop00/*.png
          ・time_pot_InnerLoopNN.png / time_pwm_InnerLoopNN.png / time_adrc_InnerLoopNN.png /
            time_simPWM_InnerLoopNN.png
              … 内側ループごとに1枚ずつ（_record で貯めたスナップショットを1つずつ描く）
          ・inner_cost / inner_extrema / inner_model / inner_system
              … その外側ループの最後のスナップショット（＝終了・中断時点の最新の履歴グラフ）から1枚ずつ
        画面表示と同じ描画関数を使うので、保存された図は画面で見えていた図と一致する。
        画面用とは別に保存専用の Agg の図を使うため、Tkが閉じた後でも動く。
        """
        if not self.save_dir:                                                                     # 保存先が指定されていない場合
            return
        if not self.records:                                                                      # スナップショットを1度も読み込めなかった場合
            print("保存するデータがありません（スナップショットを1度も読み込んでいません）。")
            return

        per_inner = [p for p in self.PANELS if p[3] == 'inner']                                   # 内側ループごとに1枚ずつ保存するグラフ
        per_outer = [p for p in self.PANELS if p[3] == 'outer']                                   # 外側ループごとに1枚ずつ保存する履歴グラフ

        fig = Figure(figsize=SAVE_FIGSIZE, dpi=SAVE_DPI)                                          # 保存専用の図（使い回して高速化）
        FigureCanvasAgg(fig)                                                                      # 画像保存専用のキャンバスを結び付ける
        total = sum((len(per_inner) * len(r['snaps']) + len(per_outer)) * N_DOF_ALL                # 保存するファイル総数
                    for r in self.records.values())
        done, t0 = 0, time.perf_counter()                                                         # 保存した枚数と開始時刻
        print(f"\nグラフを保存します: {self.save_dir}  （{total} ファイル）")

        for outer in sorted(self.records):                                                        # 外側ループごとに処理する
            snaps = self.records[outer]['snaps']                                                  # {内側ループ番号: スナップショット}
            inners = sorted(snaps)                                                                # その外側ループで記録できた内側ループ
            last = snaps[inners[-1]]                                                              # 履歴グラフ用の最新スナップショット
            for i in range(N_DOF_ALL):                                                            # 全自由度ループ
                folder = os.path.join(self.save_dir, f"DOF{i + 1:02d}", f"OuterLoop{outer - 1:02d}")
                os.makedirs(folder, exist_ok=True)                                                # 無ければ作る

                for inner in inners:                                                              # 内側ループごとに1枚ずつ保存する
                    d = snaps[inner]                                                              # その内側ループのスナップショット
                    tag = f"InnerLoop{inner - 1:02d}"                                             # 履歴グラフの横軸と同じ0始まりの番号
                    for name, func_name, n_rows, _ in per_inner:                                  # グラフごとに保存する
                        self._save_fig(fig, folder, f"{name}_{tag}.png", func_name, n_rows, d, i,
                                       f"DOF {i + 1:02d}  {tag}")
                        done += 1

                for name, func_name, n_rows, _ in per_outer:                                      # 履歴グラフは外側ループごとに1枚ずつ
                    self._save_fig(fig, folder, f"{name}.png", func_name, n_rows, last, i,
                                   f"DOF {i + 1:02d}  OuterLoop{outer - 1:02d}")
                    done += 1

            print(f"  OuterLoop{outer - 1:02d}: DOF01-{N_DOF_ALL:02d} 完了  ({done}/{total})")

        print(f"保存完了: {self.save_dir}  （{done} ファイル / {time.perf_counter() - t0:.1f} 秒）")

    # 保存用: 図を作り直して1枚保存する関数
    def _save_fig(self, fig, folder, fname, func_name, n_rows, d, i, title):                      # 引数(使い回す図, 保存先, ファイル名, 描画関数名, 段数, スナップショット, 自由度の添字, タイトル)
        """画面用と同じ描画関数で1枚だけ描き、PNGへ保存する。

        上下2段のグラフ（time_adrc / time_simPWM）は、時間軸を共有した2つの軸を縦に並べて描く。
        余白は tight_layout ではなく subplots_adjust で明示的に指定する。右軸（twinx）を持つ
        パネルでは軸ラベルのぶんだけ左右の余白が必要で、tight_layout はそれを確保しきれずに
        「Tight layout not applied. The left and right margins cannot be made large enough to
        accommodate all axes decorations.」という警告を出すため。余白を固定すれば警告は出ず、
        グラフの内容そのものは一切変わらない。
        """
        fig.clear()                                                                               # 前の図（右軸・凡例含む）を消す
        if n_rows == 1:                                                                           # 1段のグラフ
            axes = [fig.add_subplot(111)]
        else:                                                                                     # 上下2段のグラフ（時間軸を共有する）
            ax0 = fig.add_subplot(211)
            axes = [ax0, fig.add_subplot(212, sharex=ax0)]
        for ax in axes:
            ax._twins = []                                                                        # 右軸の記録を初期化する
        try:
            self._draw_panel(func_name, axes, d, i)                                               # 画面用と同じ描画関数を使う
        except Exception:                                                                         # 描けないパネルは飛ばす（保存全体は続ける）
            return
        fig.suptitle(title, fontsize=9)                                                           # 図のタイトル
        fig.subplots_adjust(hspace=SAVE_HSPACE, **SAVE_MARGIN)                                    # 余白を明示指定（tight_layoutの警告を出さない）
        fig.savefig(os.path.join(folder, fname))                                                  # PNGへ保存


# ==============================================================================
# エントリーポイント
# ==============================================================================
def main(args=None):
    path = sys.argv[1] if len(sys.argv) > 1 else SNAPSHOT_PATH                                    # 引数でパスを上書きできる
    save_dir = os.path.join(os.getcwd(), time.strftime('%Y%m%d_%H%M%S'))                          # 実行ディレクトリ直下・実行開始日時のフォルダ

    print(f"【監視するスナップショット】 {path}")
    print("test_code11.py 側で ENABLE_SNAPSHOT = True になっていることを確認してください。")
    print(f"【終了時のグラフ保存先】 {save_dir}")
    print("終了するには Ctrl+C を押すか、ウィンドウを閉じてください（どちらでも保存されます）。")

    root = tk.Tk()                                                                                # Tkinterを起動
    viewer = OptimizationViewer(root, path, save_dir)                                              # ビューアを生成

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
        root.mainloop()                                                                           # イベントループ開始
    except KeyboardInterrupt:                                                                     # 念のため（通常はシグナルハンドラ側で処理される）
        pass
    finally:
        viewer.save_all()                                                                         # 正常終了・中断のどちらでも必ず保存する
        try:
            root.destroy()                                                                        # ウィンドウを閉じる（既に閉じていれば何もしない）
        except tk.TclError:
            pass


if __name__ == '__main__':
    main()
