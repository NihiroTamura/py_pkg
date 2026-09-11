#!/usr/bin/env python3
"""test_code7.py（ADRC＋FF最適制御）の最適化ビューア

test_code7.py が内側ループごとに書き出す .npz スナップショットの更新時刻を監視し、
更新されていれば読み直して再描画する別プロセスのプログラム。最適化本体を止めないよう、
描画はこちらのプロセスだけで行う。

  起動: python3 <このファイル>              （引数でスナップショットのパスを上書きできる）
        ros2 run py_pkg test_code8

view_optimization.py（test_code3.py 用）と同じ設計方針だが、新方式で増えた量を表示する。
  ・応答パネルに Q_k の切替時刻 k_s と ±10% 帯を明示する
  ・ADRC照合パネルに ζ3_sim vs 実測 z3 を重ねる（内部複製の妥当性が一目で分かる）
  ・履歴パネルに ρ(A_cl)・U_max・ρ_tr・r3 の rms を出す（モデル妥当性とトラストリージョンの監視）
  ・最適化対象外の自由度は表示枠を保ったまま実測POT値だけを描く

監視するパスは test_code3.py が使う /tmp/el_optimization_snapshot.npz とは分けてある
（同時に起動したときに互いのスナップショットを誤読しないようにするため）。

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

SNAPSHOT_PATH = "/tmp/el_optimization_snapshot_v7.npz"          # 監視するスナップショット（test_code7.py 側と同じパスにすること）
POLL_MS = 400                                                   # スナップショットの更新を確認する間隔 [ms]
N_DOF_ALL = 24                                                  # 表示する自由度の総数
PWM_LIMIT = 255.0                                               # 実機PWMの絶対上限 [PWM]（グラフの補助線に使う）
SETTLE_BAND = 0.10                                              # 整定判定の帯幅（test_code7.py の SETTLE_BAND と合わせること）

# ==============================================================================
# 終了時に保存するグラフの設定
#   スナップショットは最新の内側ループ1回分しか残らないため、内側ループごとのグラフを
#   保存するにはビューア側で読み込んだものを貯めておく必要がある（_record 参照）。
#   保存先は  <実行ディレクトリ>/YYYYMMDD_HHMMSS/DOF01/OuterLoop00/*.png
#     ・time_pot_InnerLoopNN.png / time_pwm_InnerLoopNN.png / time_adrc_InnerLoopNN.png
#         … 内側ループごとに1枚ずつ
#     ・inner_model / inner_extrema / inner_cost / inner_gate / inner_trust
#         … その外側ループの最後のスナップショット（＝終了・中断時点の最新の履歴グラフ）から1枚ずつ
# ==============================================================================
SAVE_FIGSIZE = (7.0, 4.5)                                       # 保存する画像1枚のサイズ [inch]
SAVE_DPI = 100                                                  # 保存する画像の解像度
SAVE_MARGIN = dict(left=0.13, right=0.87, top=0.90, bottom=0.13)  # 保存する図の余白（左右に軸ラベルぶんを確保する）

# 5次フィットの経路を表す値と、その意味（test_code7.py の last_fit_mode と対応）
FIT_MODE_TEXT = {
    0: "L2 projection",                                         # 閉形式の最小二乗で決まった（正常）
    1: "fallback (extrema param)",                              # 極値条件を満たさず退避経路で決まった
    -1: "FAILED -> seed shape",                                 # 退避経路でも見つからず初期励振形へ退避した
    -2: "n/a",                                                  # 該当なし（初回同定など）
    -3: "f=0 (no motion)",                                      # 動かす必要がない自由度
}


# ==============================================================================
# ビューア本体
# ==============================================================================
class OptimizationViewer:
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
        self.root.title("test_code7 optimization viewer")                                   # ウィンドウのタイトル
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
        self.axes = self.fig.subplots(2, 4).ravel()                                         # 2行4列の8枚
        self.fig.subplots_adjust(left=0.045, right=0.985, top=0.92, bottom=0.07,            # ラベルが隣や図の外と重ならない間隔
                                 hspace=0.50, wspace=0.30)
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
        Time-POT / Time-PWM / ADRC照合 を保存するにはビューア側で貯めておく必要がある。
        自動更新をOFFにしている間や、ビューアを起動する前の内側ループは記録されない。
        """
        session = str(d['session']) if 'session' in d else ''                               # test_code7.py の実行ID
        if self.session is not None and session != self.session:                            # 別の実行が始まった（ビューアを開いたまま再実行した）
            print(f"新しい実行を検出しました（{self.session or '不明'} -> {session or '不明'}）。"
                  f"それまでに蓄積した保存対象を破棄します。")
            self.records.clear()                                                            # 実行をまたいだデータを混ぜない
        self.session = session                                                              # 現在の実行IDを覚える
        outer, inner = int(d['outer']), int(d['inner'])                                     # 外側・内側ループ番号
        rec = self.records.setdefault(outer, {'snaps': {}})                                 # その外側ループの記録枠
        rec['snaps'].setdefault(inner, d)                                                   # 同じ内側ループは1回だけ記録する

    # 全パネルを描き直す関数
    def _redraw(self):                                                                      # 引数なし（表示中の自由度を描き直す）
        if self.data is None:                                                               # まだ読み込めていない場合
            return
        d, i = self.data, self.dof                                                          # スナップショットと表示中の自由度
        for ax in self.axes:                                                                # 全パネルを消す
            ax.clear()                                                                      # 前回の描画内容を消す
            for tw in getattr(ax, '_twins', []):                                            # 前回作った右軸も消す
                tw.remove()                                                                 # 前回作った右軸を消す（残すと重なって増え続ける）
            ax._twins = []                                                                  # 右軸の記録を初期化する
        self._plot_response(self.axes[0], d, i)                                             # 1: 応答（Time-POT）
        self._plot_input(self.axes[1], d, i)                                                # 2: 入力（Time-PWM）
        self._plot_adrc(self.axes[2], d, i)                                                 # 3: ADRC照合（u_ADRC と ζ3）
        self._plot_model_history(self.axes[3], d, i)                                        # 4: モデルパラメータの推移
        self._plot_extrema_history(self.axes[4], d, i)                                      # 5: FF極値の推移
        self._plot_cost_history(self.axes[5], d, i)                                         # 6: 評価関数の推移
        self._plot_gate_history(self.axes[6], d, i)                                         # 7: モデル妥当性の推移
        self._plot_trust_history(self.axes[7], d, i)                                        # 8: トラストリージョンの推移
        opt = float(d['opt'][i]) > 0.5 if 'opt' in d else True                               # この自由度が最適化対象か
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
        t = d['t']                                                                          # 時間軸 [s]
        u_ff_opt = d['u_ff_opt_used'][i]                                                    # 今回印加したFFの元になった u_FF_opt（ELの解）
        u_opt = d['u_opt_used'][i]                                                          # そのときELが想定した総最適入力（内部ADRC分＋u_FF_opt）
        u_ff = d['u_ff_applied'][i]                                                         # 今回ロボットへ送信した5次関数FF入力
        ax.plot(t, u_ff_opt, 'c-', lw=1.0, label='u_FF_opt (EL)')                            # ELが出した最適FF入力
        ax.plot(t, u_opt, '0.6', lw=0.8, ls='--', label='u_opt = u_ADRC + u_FF')             # 総最適入力
        ax.plot(t, u_ff, 'r-', lw=1.4, label='u_FF (5th-order, sent)')                       # 実機へ送った5次関数FF
        t1, y1, t2, y2 = [float(v) for v in d['extrema'][i]]                                 # 印加したFFの極値
        ax.plot([t1, t2], [y1, y2], 'ro', ms=5)                                              # 極値マーカー
        ax.annotate(f'({t1:.3f}, {y1:+.1f})', (t1, y1), fontsize=6, xytext=(3, 4), textcoords='offset points')
        ax.annotate(f'({t2:.3f}, {y2:+.1f})', (t2, y2), fontsize=6, xytext=(3, -9), textcoords='offset points')
        u_max = float(d['u_max'][i]) if 'u_max' in d else np.nan                             # トラストリージョンのFF振幅上限
        if np.isfinite(u_max):                                                               # 上限が有効な場合
            ax.axhline(+u_max, color='m', lw=0.8, ls=':')                                    # +U_max
            ax.axhline(-u_max, color='m', lw=0.8, ls=':')                                    # -U_max
            ax.text(t[-1], u_max, f' U_max={u_max:.0f}', color='m', fontsize=6, ha='right', va='bottom')
        ax.axhline(0.0, color='k', lw=0.6)                                                   # 0線
        ax.axvline(float(d['T']), color='0.5', lw=0.8, ls=':')                               # FF入力の終了時刻 T
        ax.set_title('Input  (Time-PWM)', fontsize=9)
        ax.set_xlabel('time [s]', fontsize=8); ax.set_ylabel('PWM', fontsize=8)
        ax.tick_params(labelsize=7); ax.grid(alpha=0.3)                                     # 目盛りの大きさとグリッド
        ax.set_xlim(0, min(float(d['T']) * 1.6, float(t[-1])))                               # FF区間まわりを拡大して見る
        self._fit_ylim(ax, [u_ff_opt, u_opt, u_ff])                                          # 補助線を無視して範囲を決める
        ax.legend(fontsize=6, loc='best')                                                   # 凡例（重ならない位置へ自動配置）

    # ADRC照合（内部複製 vs 実機）を描く関数
    def _plot_adrc(self, ax, d, i):                                                         # 引数(描画先のグラフ, スナップショット, 自由度の添字)
        """内部複製が出した u_ADRC・ζ3 が、実機の実測PWM・実測z3 と一致しているかを見るパネル。

        ここが合っていないと、同定モデルが実機を再現できていないということなので、
        u_FF の最適化もその分だけ的外れになる。
        """
        t = d['t']                                                                           # 時間軸 [s]
        ax.plot(t, d['u_adrc_meas'][i], 'k-', lw=1.0, label='u_ADRC measured')                # 実測ADRC出力（u_pwm - u_ff）
        ax.plot(t, d['u_adrc_cl'][i], 'b--', lw=1.0, label='u_ADRC replica')                  # 内部複製が出したADRC出力
        ax.set_ylabel('PWM', fontsize=8)
        if 'z3_meas' in d:                                                                    # z3 を購読できている場合
            tw = self._make_twin(ax, 'z3 [count/s^2]', color='g')                             # 右軸に z3 を重ねる
            tw.plot(t, d['z3_meas'][i], 'g-', lw=0.9, label='z3 measured')                    # 実測 z3（ESO外乱推定値）
            tw.plot(t, d['zeta3_sim'][i], 'm--', lw=0.9, label='zeta3 replica')               # 内部複製が予測した ζ3
            h1, l1 = ax.get_legend_handles_labels(); h2, l2 = tw.get_legend_handles_labels()   # 左軸・右軸の凡例要素を集める
            ax.legend(h1 + h2, l1 + l2, fontsize=6, loc='best')                               # 左右の凡例をまとめる
        else:
            ax.legend(fontsize=6, loc='best')                                                   # 凡例（重ならない位置へ自動配置）
        ax.set_title('ADRC replica vs measured', fontsize=9)
        ax.set_xlabel('time [s]', fontsize=8)
        ax.tick_params(labelsize=7); ax.grid(alpha=0.3)                                     # 目盛りの大きさとグリッド
        self._fit_ylim(ax, [d['u_adrc_meas'][i], d['u_adrc_cl'][i]])                          # 補助線を無視して範囲を決める

    # 履歴パネルの共通設定を行う関数
    @staticmethod
    def _setup_history_axis(ax, n, title):                                                    # 引数(グラフ, 履歴の点数, タイトル)
        ax.set_title(title, fontsize=9)                                                     # パネルのタイトル
        ax.set_xlabel('inner loop', fontsize=8)                                             # 横軸は内側ループ番号
        ax.tick_params(labelsize=7); ax.grid(alpha=0.3)                                     # 目盛りの大きさとグリッド
        if n > 0:
            ax.set_xlim(-0.5, max(n - 0.5, 0.5))                                            # 点が端で切れないよう少し余白を取る

    # モデルパラメータの推移を描く関数
    def _plot_model_history(self, ax, d, i):                                                # 引数(描画先のグラフ, スナップショット, 自由度の添字)
        sysh = d['hist_sys'][:, i, :] if d['hist_sys'].size else np.zeros((0, 4))              # システムモデル[T1, zeta, wn, b0]の推移
        tgth = d['hist_tgt'][:, i, :] if d['hist_tgt'].size else np.zeros((0, 2))              # 目標モデル[T1, wn]の推移
        n = len(sysh); x = np.arange(n)                                                        # 内側ループ番号
        if n:                                                                               # 履歴が1点でもある場合だけ描く
            ax.plot(x, sysh[:, 1], 'b.-', lw=1.0, ms=4, label='sys zeta')                      # 減衰比（負なら発散振動）
            ax.plot(x, sysh[:, 2], 'c.-', lw=1.0, ms=4, label='sys wn')                        # 固有振動数
            ax.plot(x, tgth[:, 1], 'r.-', lw=1.0, ms=4, label='tgt wn')                         # 目標モデルの固有振動数
            tw = self._make_twin(ax, 'b0', color='g')                                           # 右軸に b0（桁が違うため）
            tw.plot(x, sysh[:, 3], 'g.-', lw=1.0, ms=4, label='sys b0')
            h1, l1 = ax.get_legend_handles_labels(); h2, l2 = tw.get_legend_handles_labels()   # 左軸・右軸の凡例要素を集める
            ax.legend(h1 + h2, l1 + l2, fontsize=6, loc='best')                             # 左右の凡例をまとめて出す
            ax.axhline(0.0, color='k', lw=0.6)                                                  # 0線（zeta<0, b0<0 が一目で分かる）
        self._setup_history_axis(ax, n, 'Model parameters')

    # FF極値の推移を描く関数
    def _plot_extrema_history(self, ax, d, i):                                              # 引数(描画先のグラフ, スナップショット, 自由度の添字)
        ext = d['hist_ext'][:, i, :] if d['hist_ext'].size else np.zeros((0, 4))                # [t1, y1, t2, y2] の推移
        n = len(ext); x = np.arange(n)                                                          # 内側ループ番号
        if n:                                                                               # 履歴が1点でもある場合だけ描く
            ax.plot(x, ext[:, 1], 'r.-', lw=1.0, ms=4, label='y1 (peak)')                       # 正の極大値
            ax.plot(x, ext[:, 3], 'b.-', lw=1.0, ms=4, label='y2 (trough)')                     # 負の極小値
            ax.axhline(0.0, color='k', lw=0.6)                                                  # 0線（山と谷の符号が一目で分かる）
            tw = self._make_twin(ax, 't [s]', color='0.4')                                      # 右軸に極値時刻
            tw.plot(x, ext[:, 0], '.--', color='0.4', lw=0.8, ms=3, label='t1')
            tw.plot(x, ext[:, 2], '.--', color='0.7', lw=0.8, ms=3, label='t2')
            h1, l1 = ax.get_legend_handles_labels(); h2, l2 = tw.get_legend_handles_labels()   # 左軸・右軸の凡例要素を集める
            ax.legend(h1 + h2, l1 + l2, fontsize=6, loc='best')                             # 左右の凡例をまとめて出す
        ax.set_ylabel('PWM', fontsize=8)
        self._setup_history_axis(ax, n, 'FF extrema  (want y1 > 0 > y2)')

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

    # モデル妥当性（ρ(A_cl) と残差）の推移を描く関数
    def _plot_gate_history(self, ax, d, i):                                                 # 引数(描画先のグラフ, スナップショット, 自由度の添字)
        """ρ(A_cl) が 1 を超えた自由度は内部ADRC閉ループが発散しており、FF更新が見送られる。
        r3 = z3_meas - ζ3_sim の rms は、モデルで説明できない外乱の大きさ（純粋なモデル誤差指標）。
        """
        rho = d['hist_cl_radius'][:, i] if d['hist_cl_radius'].size else np.zeros(0)             # 閉ループのスペクトル半径の推移
        n = len(rho); x = np.arange(n)                                                           # 内側ループ番号
        if n:                                                                               # 履歴が1点でもある場合だけ描く
            ax.plot(x, rho, 'b.-', lw=1.2, ms=4, label='rho(A_cl)')                              # スペクトル半径
            ax.axhline(1.0, color='r', lw=1.0, ls='--')                                          # 安定限界（これを超えると発散）
            ax.text(0, 1.0, ' unstable above', color='r', fontsize=6, va='bottom')
            tw = self._make_twin(ax, 'residual', color='g')                                      # 右軸に残差
            if d['hist_adrc_res'].size:
                tw.semilogy(x, np.maximum(d['hist_adrc_res'][:, i], 1e-12), 'g.--', lw=0.9, ms=3, label='u_ADRC res [PWM]')
            if 'hist_r3_rms' in d and d['hist_r3_rms'].size:
                tw.semilogy(x, np.maximum(np.abs(d['hist_r3_rms'][:, i]), 1e-12), 'm.:', lw=0.9, ms=3, label='r3 rms')
            h1, l1 = ax.get_legend_handles_labels(); h2, l2 = tw.get_legend_handles_labels()   # 左軸・右軸の凡例要素を集める
            ax.legend(h1 + h2, l1 + l2, fontsize=6, loc='best')                             # 左右の凡例をまとめて出す
        ax.set_ylabel('rho(A_cl)', fontsize=8)
        self._setup_history_axis(ax, n, 'Model validity gate')

    # トラストリージョンの推移を描く関数
    def _plot_trust_history(self, ax, d, i):                                                # 引数(描画先のグラフ, スナップショット, 自由度の添字)
        """U_max は FF振幅の上限。ρ_tr = (実測の改善)/(モデルが予測した改善) が
        0.25〜0.75 の範囲に収まり、U_max が発散も収縮もせず落ち着いていれば健全である。
        """
        um = d['hist_u_max'][:, i] if 'hist_u_max' in d and d['hist_u_max'].size else np.zeros(0)  # FF振幅上限の推移
        n = len(um); x = np.arange(n)                                                             # 内側ループ番号
        if n:                                                                               # 履歴が1点でもある場合だけ描く
            ax.plot(x, um, 'b.-', lw=1.2, ms=4, label='U_max [PWM]')                              # FF振幅の上限
            tw = self._make_twin(ax, 'rho_tr', color='r')                                         # 右軸にトラストリージョン比
            if 'hist_rho_tr' in d and d['hist_rho_tr'].size:
                tw.plot(x, d['hist_rho_tr'][:, i], 'r.--', lw=0.9, ms=4, label='rho_tr')
            tw.axhline(0.75, color='0.6', lw=0.7, ls=':')                                         # これより上なら上限を広げる
            tw.axhline(0.25, color='0.6', lw=0.7, ls=':')                                         # これより下なら上限を縮める
            tw.set_ylim(-1.0, 2.0)                                                                # 判定域が見える範囲に固定する
            h1, l1 = ax.get_legend_handles_labels(); h2, l2 = tw.get_legend_handles_labels()   # 左軸・右軸の凡例要素を集める
            ax.legend(h1 + h2, l1 + l2, fontsize=6, loc='best')                             # 左右の凡例をまとめて出す
        ax.set_ylabel('U_max [PWM]', fontsize=8)
        self._setup_history_axis(ax, n, 'Trust region')

    # 数値を安全に文字列へ整形する関数
    @staticmethod
    def _num(v, fmt='{:14.4g}'):                                                                  # 引数(値, 書式)
        try:
            f = float(v)
        except (TypeError, ValueError):
            return f"{'n/a':>14s}"
        return f"{'n/a':>14s}" if not np.isfinite(f) else fmt.format(f)

    # 左の数値一覧を更新する関数
    def _update_text(self, d, i):                                                           # 引数(スナップショット, 自由度の添字)
        num = self._num                                                                           # 整形関数の別名
        opt = float(d['opt'][i]) > 0.5 if 'opt' in d else True                                     # この自由度が最適化対象か
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
                f"   sys T1        {num(d['sys_params_id'][i][0])}",
                f"   sys zeta      {num(d['sys_params_id'][i][1])}",
                f"   sys wn        {num(d['sys_params_id'][i][2])}",
                f"   sys b0        {num(d['sys_params_id'][i][3])}",
                f"   id residual   {num(d['id_res_sys'][i])}",
                "",
                " [ADRC replica]  vs measured",
                f"   u_ADRC res    {num(d['adrc_res'][i])} PWM",
                f"   before refine {num(d['adrc_res_open'][i])} PWM",
                f"   refined       {'yes' if float(d['cl_ok'][i]) > 0.5 else 'NO':>14s}",
                f"   kick meas/th  {num(d['adrc_kick'][i])}",
                f"   delay         {num(d['adrc_delay'][i], '{:14.0f}')} samples",
                f"   rho(A_cl)     {num(d['cl_radius'][i], '{:14.5f}')}",
                f"   r3 rms        {num(d['r3_rms'][i]) if 'r3_rms' in d else '':>14s}",
                "",
                " [Cost weighting]",
                f"   k_s (settle)  {num(d['k_s'][i], '{:14.0f}')} samples",
                f"   k_s time      {num(float(d['k_s'][i]) * float(d['dt'])) if 'dt' in d else '':>14s} s",
                "",
                " [Trust region]",
                f"   U_max         {num(d['u_max'][i])} PWM",
                f"   rho_tr        {num(d['rho_tr'][i])}",
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
          ・time_pot_InnerLoopNN.png / time_pwm_InnerLoopNN.png / time_adrc_InnerLoopNN.png
              … 内側ループごとに1枚ずつ（_record で貯めたスナップショットを1つずつ描く）
          ・inner_model / inner_extrema / inner_cost / inner_gate / inner_trust
              … その外側ループの最後のスナップショット（＝終了・中断時点の最新の履歴グラフ）から1枚ずつ
        画面表示と同じ描画関数を使うので、保存された図は画面で見えていた図と一致する。
        画面用とは別に保存専用の Agg の図を使うため、Tkが閉じた後でも動く。
        """
        if not self.save_dir:                                                                     # 保存先が指定されていない場合
            return
        if not self.records:                                                                      # スナップショットを1度も読み込めなかった場合
            print("保存するデータがありません（スナップショットを1度も読み込んでいません）。")
            return

        per_inner = [                                                                             # 内側ループごとに1枚ずつ保存するパネル
            ('time_pot', self._plot_response),                                                    # 応答（Time-POT）
            ('time_pwm', self._plot_input),                                                       # 入力（Time-PWM）
            ('time_adrc', self._plot_adrc),                                                       # ADRC照合（u_ADRC と ζ3）
        ]
        per_outer = [                                                                             # 外側ループごとに1枚ずつ保存する履歴パネル
            ('inner_model.png', self._plot_model_history),                                        # モデルパラメータの推移
            ('inner_extrema.png', self._plot_extrema_history),                                    # FF極値の推移
            ('inner_cost.png', self._plot_cost_history),                                          # 評価関数の推移
            ('inner_gate.png', self._plot_gate_history),                                          # モデル妥当性の推移
            ('inner_trust.png', self._plot_trust_history),                                        # トラストリージョンの推移
        ]

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

                for inner in inners:                                                              # 内側ループごとに3種類を1枚ずつ保存する
                    d = snaps[inner]                                                              # その内側ループのスナップショット
                    tag = f"InnerLoop{inner - 1:02d}"                                             # 履歴グラフの横軸と同じ0始まりの番号
                    for name, func in per_inner:                                                  # パネルごとに保存する
                        self._save_fig(fig, folder, f"{name}_{tag}.png", func, d, i,
                                       f"DOF {i + 1:02d}  {tag}")
                        done += 1

                for fname, func in per_outer:                                                     # 履歴グラフは外側ループごとに1枚ずつ
                    self._save_fig(fig, folder, fname, func, last, i,
                                   f"DOF {i + 1:02d}  OuterLoop{outer - 1:02d}")
                    done += 1

            print(f"  OuterLoop{outer - 1:02d}: DOF01-{N_DOF_ALL:02d} 完了  ({done}/{total})")

        print(f"保存完了: {self.save_dir}  （{done} ファイル / {time.perf_counter() - t0:.1f} 秒）")

    # 保存用: 図を作り直して1枚保存する関数
    @staticmethod
    def _save_fig(fig, folder, fname, func, d, i, title):                                         # 引数(使い回す図, 保存先, ファイル名, 描画関数, スナップショット, 自由度の添字, タイトル)
        """画面用と同じ描画関数で1枚だけ描き、PNGへ保存する。

        余白は tight_layout ではなく subplots_adjust で明示的に指定する。右軸（twinx）を持つ
        パネルでは軸ラベルのぶんだけ左右の余白が必要で、tight_layout はそれを確保しきれずに
        「Tight layout not applied. The left and right margins cannot be made large enough to
        accommodate all axes decorations.」という警告を出すため。余白を固定すれば警告は出ず、
        グラフの内容そのものは一切変わらない。
        """
        fig.clear()                                                                               # 前の図（右軸・凡例含む）を消す
        ax = fig.add_subplot(111)                                                                 # 1枚だけのグラフ
        ax._twins = []                                                                            # 右軸の記録を初期化する
        try:
            func(ax, d, i)                                                                        # 画面用と同じ描画関数を使う
        except Exception:                                                                         # 描けないパネルは飛ばす（保存全体は続ける）
            return
        fig.suptitle(title, fontsize=9)                                                           # 図のタイトル
        fig.subplots_adjust(**SAVE_MARGIN)                                                        # 余白を明示指定（tight_layoutの警告を出さない）
        fig.savefig(os.path.join(folder, fname))                                                  # PNGへ保存


# ==============================================================================
# エントリーポイント
# ==============================================================================
def main(args=None):
    path = sys.argv[1] if len(sys.argv) > 1 else SNAPSHOT_PATH                                    # 引数でパスを上書きできる
    save_dir = os.path.join(os.getcwd(), time.strftime('%Y%m%d_%H%M%S'))                          # 実行ディレクトリ直下・実行開始日時のフォルダ

    print(f"【監視するスナップショット】 {path}")
    print("test_code7.py 側で ENABLE_SNAPSHOT = True になっていることを確認してください。")
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
