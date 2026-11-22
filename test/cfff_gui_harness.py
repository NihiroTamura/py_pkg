"""GUI_android_tk_cfFF.py の結合テスト（同じプロセスの偽ボードを相手に、GUIを自動で操作する）

実機（ROS_DOMAIN_ID=16）に届かないよう、別の DOMAIN で動かすこと:
  ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 DISPLAY=:0 python3 test/cfff_gui_harness.py <作業フォルダ>
（test/test_gui_cfff_integration.py が CFFF_GUI_TEST=1 のときにこの形で起動する）
確かめること: FF の手動設定と送信内容（スロット・T・使わないスロット）、記録の開始（目標値の変化）・記録時間・
保存（FF(w_…)/FF(wo_…)、CSV、PNG 104 枚）、FF=0 の送信、記録中の受信の途切れの警告、比較（基準の自動選択・保存）、
Optuna の読み込み（①②③・最適化していない自由度・送信した係数）
"""
import glob
import json
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'py_pkg'))
import GUI_android_tk_cfFF as g  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from std_msgs.msg import Float32MultiArray, UInt16MultiArray  # noqa: E402

OPTUNA = os.path.expanduser('~/ArmRobot/optimize_data/verify_optuna_20261104/optimize_20261104_1504')
INIT = [500, 200, 500, 300, 170, 300, 160, 410, 200, 500, 350, 220, 300, 200, 400, 350, 420, 400, 325, 370, 280, 420,
        360, 390, 420, 390]


class FakeRobot(Node):
    """5ボードの偽ファーム。目標値が変わった自由度だけ FF を t=0 から掛け、FF係数は毎周期読み直す（.ino と同じ）"""

    def __init__(self):
        super().__init__('fake_robot')
        self.lock = threading.Lock()
        self.target = [float(v) for v in INIT]
        self.prev_target = list(self.target)
        self.pos = [float(v) for v in INIT]
        self.ff_params = {k: [0.0] * 36 for k in range(1, 6)}
        self.ff_msgs = {k: [] for k in range(1, 6)}
        self.t_start = [-1e9] * 26
        self.silent = set()         # 配信を止めているボード
        self.pubs = {}
        for k, (board, offset, count) in enumerate(g.BOARD_LAYOUT, start=1):
            self.pubs[k] = {
                'tk': self.create_publisher(UInt16MultiArray, g.TOPIC_POT.format(k), 10),
                'pwm': self.create_publisher(Float32MultiArray, g.TOPIC_PWM.format(k), 10),
                'z3': self.create_publisher(Float32MultiArray, g.TOPIC_Z3.format(k), 10),
                'ff': self.create_publisher(Float32MultiArray, g.TOPIC_FF_OUT.format(k), 10),
                'param': self.create_publisher(Float32MultiArray, g.TOPIC_PARAM.format(k), 10),
            }
            self.create_subscription(Float32MultiArray, g.TOPIC_FF.format(k),
                                     lambda msg, k=k: self.on_ff(k, msg), 10)
        self.create_subscription(Float32MultiArray, g.TOPIC_TARGET, self.on_target, 10)
        self.target_pub = self.create_publisher(Float32MultiArray, g.TOPIC_TARGET, 10)
        self.n = 0
        self.create_timer(0.010, self.step)

    def on_ff(self, k, msg):
        with self.lock:
            self.ff_params[k] = list(msg.data)
            self.ff_msgs[k].append(list(msg.data))

    def on_target(self, msg):
        with self.lock:
            self.target = [float(v) for v in msg.data]

    def send_target(self, values):
        msg = Float32MultiArray()
        msg.data = [float(v) for v in values]
        self.target_pub.publish(msg)

    def step(self):
        now = time.monotonic()
        dt = 0.010
        self.n += 1
        with self.lock:
            for k, (board, offset, count) in enumerate(g.BOARD_LAYOUT, start=1):
                pwm, z3, ff = [0.0] * 6, [0.0] * 6, [0.0] * 6
                for i in range(count):
                    d = offset + i
                    if self.target[d] != self.prev_target[d]:
                        self.t_start[d] = now
                    self.prev_target[d] = self.target[d]
                    p = self.ff_params[k][6 * i:6 * i + 6] if i < g.FF_BOARD_DOFS[board] else [0.0] * 6
                    tau = min(max(now - self.t_start[d], 0.0), p[5])
                    f = sum(c * tau ** e for c, e in zip(p[:5], (5, 4, 3, 2, 1))) if tau < p[5] else 0.0
                    if board in ('board4', 'board5') and i >= 3:
                        f = 0.0
                    err = self.target[d] - self.pos[d]
                    u = 2.0 * err + f
                    self.pos[d] += dt * (err / 0.35 + 0.08 * f)
                    pwm[i], z3[i], ff[i] = u, 0.05 * err, f
                if k in self.silent:
                    continue
                tk = [int(round(min(max(self.pos[offset + i], 0), 1023))) for i in range(count)] + [0] * (6 - count)
                tk += [int(round(self.target[offset + i])) for i in range(count)] + [0] * (6 - count)
                m = UInt16MultiArray()
                m.data = tk
                self.pubs[k]['tk'].publish(m)
                for key, vals in (('pwm', pwm), ('z3', z3), ('ff', ff)):
                    mm = Float32MultiArray()
                    mm.data = [float(v) for v in vals]
                    self.pubs[k][key].publish(mm)
                if self.n % 50 == 0:
                    pm = Float32MultiArray()
                    pm.data = [1.0, 1.0] + sum(([9000.0, 500.0, 2e5, 700.0] for _ in range(6)), []) + [float('nan')] * 24
                    self.pubs[k]['param'].publish(pm)


def main(work):
    rclpy.init()
    node = g.CfNode()
    fake = FakeRobot()
    exes = []
    for n in (node, fake):
        ex = SingleThreadedExecutor()
        ex.add_node(n)
        threading.Thread(target=ex.spin, daemon=True).start()
        exes.append(ex)
    dialogs = []
    g.messagebox.askokcancel = lambda title, text: dialogs.append(('ask', text)) or True
    g.messagebox.showinfo = lambda title, text: dialogs.append(('info', text))
    g.messagebox.showerror = lambda title, text: dialogs.append(('error', text))
    g.messagebox.showwarning = lambda title, text: dialogs.append(('warn', text))
    settings_path = os.path.join(work, 'settings.json')
    settings = g.default_settings()
    settings['record_dir'] = os.path.join(work, 'rec')
    settings['cmp']['dir'] = os.path.join(work, 'cmp')
    app = g.CfApp(node, settings, settings_path)
    report = {}
    errors = []

    def check(cond, text):
        print(("OK  " if cond else "NG  ") + text, flush=True)
        if not cond:
            errors.append(text)

    def wait(cond, timeout, step=0.1):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if cond():
                return True
            yield step
        return False

    def script():
        # 0. 受信の開始
        yield 1.5
        check(len(node.last_rx) == 5, f"5ボードから受信: {sorted(node.last_rx)}")
        # 1. 手動でFFを設定して送信（DOF 3 と DOF 20）
        m3 = app.models[3]
        m3.update(enabled=True, T=1.0, t1=0.22, y1=30.0, t2=0.78, y2=-25.0)
        app.models[20].update(enabled=True, T=1.2, t1=0.27, y1=-15.0, t2=0.94, y2=12.0)
        yield 0.2
        check(app.models[3].check().ok and app.models[20].check().ok, "手動設定が条件を満たす")
        app.send_ff()
        yield 0.5
        b1 = fake.ff_msgs[1][-1] if fake.ff_msgs[1] else None
        c3 = g.quintic_from_extrema(1.0, 0.22, 30.0, 0.78, -25.0)[0][0]
        check(b1 is not None and len(b1) == 36 and np.allclose(b1[18:23], c3.astype(np.float32), rtol=1e-6)
              and b1[23] == 1.0, "board1 スロット3 に DOF 3 の係数と T が届く")
        check(b1 is not None and all(v == 0 for i, v in enumerate(b1) if i % 6 != 5 and not 18 <= i < 23),
              "board1 の他のスロットは係数0")
        b4 = fake.ff_msgs[4][-1] if fake.ff_msgs[4] else [0.0] * 36
        check(np.allclose(b4[12:17], g.quintic_from_extrema(1.2, 0.27, -15.0, 0.94, 12.0)[0][0].astype(np.float32),
                          rtol=1e-6) and b4[17] == np.float32(1.2) and b4[18:36] == [0, 0, 0, 0, 0, 1.5] * 3,
              "board4 スロット2 に DOF 20、スロット3〜5 は [0,0,0,0,0,1.5]")
        # 2. 記録（FFあり）
        app.duration_var.set("6")
        app.on_record_button()
        yield 0.2
        check(node.session is not None and node.session.state == g.Recorder.ARMED, "記録開始ボタンで待機")
        yield 1.0
        check(node.session.state == g.Recorder.ARMED, "目標値が変わるまでは待機のまま")
        tgt = list(INIT)
        tgt[3], tgt[20], tgt[0] = 400, 330, 520
        fake.send_target(INIT)          # 同じ値（変化なし）→ 記録は始まらない
        yield 0.3
        check(node.session.state == g.Recorder.ARMED, "同じ目標値の受信では記録が始まらない")
        t_send = time.monotonic()
        fake.send_target(tgt)
        ok = yield from wait(lambda: node.session is None or node.session.state != g.Recorder.ARMED, 2)
        s = node.session
        check(s is not None and s.state == g.Recorder.RECORDING and abs(s.t0 - t_send) < 0.2,
              "目標値が変わった時点から記録開始")
        ok = yield from wait(lambda: node.session is None, 10)
        el = time.monotonic() - t_send
        check(ok and 5.9 < el < 7.5, f"記録時間 6 s で記録終了（{el:.2f} s）")
        ok = yield from wait(lambda: not app.saves, 120, 0.5)
        folders = sorted(glob.glob(os.path.join(work, 'rec', 'FF(w_*')))
        check(ok and len(folders) == 1, f"FF(w_…) フォルダ: {folders}")
        report['w'] = folders[0] if folders else None
        if folders:     # ADRC 用の内部FF（配信番号で数える）と、実機（偽ボード）から受信した FF を同じ配信で比べる
            tr = g.tick_record(g.read_record_csv(os.path.join(folders[0], os.path.basename(folders[0]) + '.csv')))
            for d in (3, 20):
                tf, fi = tr.dof_series(d, 'ff')
                trx, fr = tr.dof_series(d, 'ffrecv')
                sel = (tf >= 0) & np.isfinite(fi)
                diff = np.abs(fi[sel] - np.interp(tf[sel], trx, fr))
                check(np.nanmax(np.abs(fr)) > 10 and diff.max() < 4.0,
                      f"DOF {d}: 内部FFと受信FFの差 最大 {diff.max():.2f} PWM（偽ボードは PWM と FF を同じ周回で出すので1配信分ずれる）")
        # 3. FF=0 を送って初期姿勢へ戻し、記録（FFなし）
        app.send_zero()
        yield 0.5
        check(all(v == 0 for k in range(1, 6) for i, v in enumerate(fake.ff_msgs[k][-1]) if i % 6 != 5)
              and not any(m.enabled for m in app.models.values()), "FF=0 送信で全ボード係数0・設定もFFなし")
        fake.send_target(INIT)
        yield 3.0
        app.on_record_button()
        yield 0.5
        fake.send_target(tgt)
        yield 1.0
        fake.silent.add(2)          # 記録中に board2 の配信を 1 s 止める（通信の途切れ）
        yield 1.0
        fake.silent.discard(2)
        ok = yield from wait(lambda: node.session is None, 10)
        ok = yield from wait(lambda: not app.saves, 120, 0.5)
        folders = sorted(glob.glob(os.path.join(work, 'rec', 'FF(wo_*')))
        check(ok and len(folders) == 1, f"FF(wo_…) フォルダ: {folders}")
        report['wo'] = folders[0] if folders else None
        check(any('board2' in w and '途切れ' in w for w in app.last_record.meta['warnings']),
              "board2 の受信の途切れを警告として記録")
        fake.send_target(INIT)
        # 4. 比較
        for key, name in (('csv1', 'w'), ('csv2', 'wo')):
            app.csv_vars[key].set(os.path.join(report[name], os.path.basename(report[name]) + '.csv'))
        app.ref_list.delete(0, 'end')
        app.load_compare()
        check(list(app.ref_list.get(0, 'end')) == [app.csv_vars['csv2'].get()], "基準（FF=0）に2つ目（wo）が自動で入る")
        app.cmp_vars['T'].set('1.0')
        app.cmp_vars['eps'].set('5')
        app.cmp_vars['E'].set('5')
        app.run_compare()
        ok = yield from wait(lambda: app.cmp_job is None, 300, 0.5)
        cf = sorted(glob.glob(os.path.join(work, 'cmp', 'cf-*')))
        check(ok and len(cf) == 1, f"比較フォルダ: {cf}")
        report['cf'] = cf[0] if cf else None
        if cf:
            import csv as _csv
            tpath = os.path.join(cf[0], g.TARGET_CSV)
            check(os.path.exists(tpath), "比較フォルダの直下に 目標軌道.csv")
            with open(tpath, encoding='utf-8-sig') as f:
                head = next(_csv.reader(f))
            check(all(c in head for c in ('Target3', 'POT3(1)', 'POT3(2)', 'Target20')), f"目標軌道.csv の列 {head[:6]}…")
            with open(os.path.join(cf[0], 'analysis', g.ANALYSIS_CSV), encoding='utf-8-sig') as f:
                arows = list(_csv.reader(f))
            i = next((k for k, r in enumerate(arows) if r and r[0].startswith('全体の評価値 J')), None)
            check(os.path.exists(os.path.join(cf[0], 'analysis', g.SHARE_PNG)), "analysis に割合の棒グラフの PNG")
            check(i is not None and arows[i + 1] == list(g.J_COLUMNS) and arows[i + 2][4] in ('1', '2', '同じ'),
                  "評価値.csv に全体の J(1)・J(2)・差・小さい方")
            check(bool(app.cmp_targets) and 'J' in app.cmp_jtext, f"画面に目標軌道と全体の J: {app.cmp_jtext}")
            ax = app.cmp_fig.axes[0]
            check(any(ln.get_label() == g.TARGET_LABEL and ln.get_linestyle() == '-.' and ln.get_color() == 'green'
                      for ln in ax.get_lines()), "画面の POT グラフに緑の一点鎖線の目標軌道")
        report['tree'] = [app.tree.item(i, 'values') for i in app.tree.get_children()]
        # 5. Optuna の読み込み
        app.optuna_dir_var.set(OPTUNA)
        app.load_optuna()
        check(app.optuna is not None, "Optuna の結果を読み込む")
        app.apply_optuna_all('1')
        best = app.optuna.best_overall()
        m0 = app.models[0]
        check(m0.enabled and best['n'] == 902 and m0.exact_valid() and f"試行{best['n']}" in m0.source,
              f"① 全自由度に J 最小の試行 {best['n']} を読み込む")
        check(not app.models[13].enabled, "Optuna で最適化していない DOF 13 は変えない")
        app.rows[5]['trial'].delete(0, 'end')
        app.rows[5]['trial'].insert(0, '476')
        app.apply_optuna_one(5, '3')
        check(app.models[5].exact['t1'] == app.optuna.trial(476)['dof'][5]['t1_sent'], "③ DOF 5 に試行 476 を読み込む")
        app.apply_optuna_one(8, '2')
        b8 = app.optuna.best_for_dof(8)
        check(f"試行{b8['n']}" in app.models[8].source, f"② DOF 8 に J_i 最小の試行 {b8['n']}")
        app.send_ff()
        yield 0.5
        sent_b1 = fake.ff_msgs[1][-1]
        exp = np.array([best['dof'][0][k] for k in ('a', 'b', 'c', 'd', 'e')], dtype=np.float32)
        check(np.array_equal(np.asarray(sent_b1[0:5], dtype=np.float32), exp) and sent_b1[5] == 3.0,
              "DOF 0 には試行 902 で Optuna が送った係数がそのまま届く")
        # 6. グループごとの best を送信（verify_optuna_20261122: 24 グループ）
        og = os.path.expanduser('~/ArmRobot/optimize_data/verify_optuna_20261122/optimize_20261122_1306')
        if os.path.exists(og):
            app.optuna_dir_var.set(og)
            app.load_optuna()
            n_before = {k: len(v) for k, v in fake.ff_msgs.items()}
            app.send_group_best()
            yield 0.5
            res = app.optuna
            okc = True
            for d in res.dofs:
                k, b, local, _c = g.DOF_BOARD[d]
                t, _v = res.group_best(res.group_of(d)[0])
                exp = np.array([t['dof'][d][c] for c in ('a', 'b', 'c', 'd', 'e')], dtype=np.float32)
                got = np.asarray(fake.ff_msgs[k][-1][6 * local:6 * local + 5], dtype=np.float32)
                okc &= np.array_equal(got, exp) and f"試行{t['n']}" in app.models[d].source
            check(all(len(fake.ff_msgs[k]) > n_before[k] for k in n_before) and okc,
                  "④ グループごとの best を送信: 全ボードに、各グループの best の試行で Optuna が送った係数が届く")
        report['dialogs'] = dialogs
        app.root.after(100, app.root.destroy)

    gen = script()

    def tick(delay=None):
        try:
            d = next(gen)
        except StopIteration:
            return
        except Exception:
            import traceback
            traceback.print_exc()
            errors.append('例外')
            app.root.destroy()
            return
        app.root.after(int(1000 * d), tick)

    app.root.after(500, tick)
    app.run()
    for ex in exes:
        ex.shutdown()
    json.dump(report, open(os.path.join(work, 'report.json'), 'w'), ensure_ascii=False, indent=1, default=str)
    print("NG:", errors)
    os._exit(1 if errors else 0)


if __name__ == '__main__':
    main(sys.argv[1])
