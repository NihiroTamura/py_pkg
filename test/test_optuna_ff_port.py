"""EL2（optimize_EL2_common.py ほか @f8fba60）と移植版（optuna_ff_common.py・optimize_optuna_ff_ROS2.py）の比較テスト（計画書 §10.2）

EL2 を import するのはこのファイルだけ。移植でわざと変えた点は「差異として期待どおりか」を確かめる。
実行: cd ~/ArmRobot/src/py_pkg && python3 -m pytest -q test/test_optuna_ff_port.py
送信メッセージの比較は ROS_DOMAIN_ID=87・ROS_LOCALHOST_ONLY=1 の同一プロセス内で行う（実機の DOMAIN 16 には送らない）。
"""
import csv
import os
import sys
import time

import numpy as np
import pytest

os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, '..', 'py_pkg'))
import optuna_ff_common as c  # noqa: E402
import optimize_EL2_common as el2  # noqa: E402

DATA = [os.path.expanduser(f'~/ArmRobot/optimize_data/{n}/trial_00.npz')
        for n in ('tuning_20261022_1918', 'tuning_20261022_1925')]
DATA = [p for p in DATA if os.path.exists(p)]


# ------------------------------------------------------------------ 定数・計算
def test_constants_equal():
    for name in ('N_DOF', 'BOARD_LAYOUT', 'FF_BOARD_DOFS', 'FF_DOF_MAP', 'FF_INDEX', 'N_FF', 'NO_FF_DOFS', 'DOF_BOARD',
                 'TOPIC_TARGET', 'TOPIC_FF', 'TOPIC_POT', 'TOPIC_PWM', 'TOPIC_Z3', 'TOPIC_Z1', 'TOPIC_Z2', 'TOPIC_DER',
                 'TOPIC_PARAM', 'KINDS', 'PUBLISH_DT', 'PWM_LIMIT', 'EVAL_FACTOR', 'DEFAULT_ID_BOUNDS', 'CSV_ENCODING',
                 'CSV_KIND', 'RING_SEC'):
        assert getattr(c, name) == getattr(el2, name), name


def test_quintic_functions_equal():
    rng = np.random.default_rng(0)
    for _ in range(500):
        T = rng.uniform(0.2, 3.0)
        t1 = rng.uniform(0.02, 0.6) * T
        t2 = rng.uniform(t1 / T + 0.05, 0.98) * T
        y1, y2 = rng.uniform(-255, 255), rng.uniform(-255, 255)
        a = c.quintic_from_extrema(T, t1, t2, y1, y2)
        b = el2.quintic_from_extrema(T, t1, t2, y1, y2)
        assert np.array_equal(a, b)
        assert np.array_equal(c.quintic_matrix(T, t1, t2), el2.quintic_matrix(T, t1, t2))
        assert c.extrema_ok(a, T) == el2.extrema_ok(b, T)
        assert c.extrema_of(a, T) == el2.extrema_of(b, T)
        assert c.interior_critical_points(a, T) == el2.interior_critical_points(b, T)
        assert c.feasible(T, (t1, t2, y1, y2)) == el2.feasible(T, (t1, t2, y1, y2))
        assert c.coeff_e(*a[:4], T) == el2.coeff_e(*b[:4], T)
        tt = np.linspace(-0.1, 1.2 * T, 200)
        assert np.array_equal(c.ff_eval(a, T, tt), el2.ff_eval(b, T, tt))
        assert np.array_equal(c.ff_samples(a, T, 0.010, 50), el2.ff_samples(b, T, 0.010, 50))
    P = rng.normal(0, 100, (300, 5))
    assert np.array_equal(c.count_interior_batch(P, 1.3), el2.count_interior_batch(P, 1.3))
    for _ in range(40):
        T = 1.0
        t1, t2 = rng.uniform(0.05, 0.4), rng.uniform(0.5, 0.95)
        assert c.ratio_intervals(T, t1, t2) == el2.ratio_intervals(T, t1, t2)
        p = (t1, t2, rng.uniform(10, 200), -rng.uniform(10, 200))
        ra, rb = c.retract(T, p), el2.retract(T, p)
        assert (ra is None and rb is None) or np.array_equal(ra, rb)


def test_target_and_error_equal():
    rng = np.random.default_rng(1)
    for L in (0.010, 0.011):
        for _ in range(5):
            T1, wn, y0 = rng.uniform(0.02, 0.5), rng.uniform(3, 20), rng.uniform(-200, 200)
            n = el2.eval_steps(1.5, L)
            assert c.eval_steps(1.5, L) == n
            assert np.array_equal(c.target_states(T1, wn, y0, L, n), el2.target_states(T1, wn, y0, L, n))
            y = el2.target_states(T1, wn, y0, L, n)[:, 0] + rng.normal(0, 0.5, n + 1)
            v = rng.uniform(size=n + 1) > 0.05
            fa = c.fit_target(y, v, y0, L, c.DEFAULT_ID_BOUNDS, prev=None)
            fb = el2.fit_target(y, v, y0, L, el2.DEFAULT_ID_BOUNDS, prev=None)
            assert fa == fb
            tgt = el2.target_states(T1, wn, y0, L, n)[:, 0]
            assert c.normalized_error(y, tgt, 100.0, v) == el2.normalized_error(y, tgt, 100.0, v)


@pytest.mark.skipif(not DATA, reason='10/22 の実測データがない')
def test_fit_target_on_real_data():
    """実測（10/22）の POT を入力にして、自由フィットの段（移植した fit_target）が EL2 と一致する"""
    for path in DATA:
        rec = c.load_trial_npz(path)
        dofs = [d for d in range(c.N_DOF) if rec.initial[d] != rec.target[d] and d not in c.NO_FF_DOFS]
        ser, _info = c.extract_series(rec.raw, rec.initial, rec.target, dofs, rec.t_target, rec.T)
        for d, s in ser.items():
            y = s.pot[s.win()] - s.Pf
            v = np.isfinite(y)
            a = c.fit_target(y, v, s.P0 - s.Pf, c.TICK_DT, c.DEFAULT_ID_BOUNDS, prev=None)
            b = el2.fit_target(y, v, s.P0 - s.Pf, c.TICK_DT, el2.DEFAULT_ID_BOUNDS, prev=None)
            assert a == b
            sp = c.fit_target_constrained(d, [y], [v], [s.P0], s.Pi, s.Pf, rec.T, 0.02)
            if not sp.active:
                assert (sp.T1, sp.wn) == a[0]                         # 拘束が効かないときは自由フィットそのもの


# ------------------------------------------------------------------ ファイル
def _rec_from(path):
    return c.load_trial_npz(path), el2.load_trial_npz(path)


def test_results_csv_bytes_equal(tmp_path):
    rng = np.random.default_rng(2)
    ff = np.zeros((c.N_FF, 5))
    for n in (0, 3, 10):
        ff[n] = c.quintic_from_extrema(1.5, 0.2, 1.2, rng.uniform(50, 200), -rng.uniform(50, 200))
    ini = list(rng.uniform(100, 600, c.N_DOF))
    tgt = list(rng.uniform(100, 600, c.N_DOF))
    a, b = tmp_path / 'a.csv', tmp_path / 'b.csv'
    c.write_results_csv(str(a), ini, tgt, 1.5, ff)
    el2.write_results_csv(str(b), ini, tgt, 1.5, ff)
    assert a.read_bytes() == b.read_bytes()
    c.write_results_csv(str(a), ini, tgt, 1.5, ff, append=True)
    el2.write_results_csv(str(b), ini, tgt, 1.5, ff, append=True)
    assert a.read_bytes() == b.read_bytes()


@pytest.mark.skipif(not DATA, reason='10/22 の実測データがない')
def test_npz_cross_read(tmp_path):
    for path in DATA:
        ra, rb = _rec_from(path)
        pa, pb = tmp_path / 'a.npz', tmp_path / 'b.npz'
        c.save_trial_npz(str(pa), ra)                                   # 移植版で保存 → EL2 で読む
        el2.save_trial_npz(str(pb), rb)                                 # EL2 で保存 → 移植版で読む
        x, y = el2.load_trial_npz(str(pa)), c.load_trial_npz(str(pb))
        for r1, r2 in ((x, rb), (y, ra)):
            assert r1.T == r2.T and r1.initial == r2.initial and r1.target == r2.target
            for board in r2.raw:
                for kind in r2.raw[board]:
                    assert np.array_equal(r1.raw[board][kind][0], r2.raw[board][kind][0])
                    assert np.array_equal(r1.raw[board][kind][1], r2.raw[board][kind][1])
        pe = tmp_path / 'e.npz'
        c.save_trial_npz_ext(str(pe), ra, {'ff_sent_rows': ra.ff, 'note': 'x'})   # 足した x_ の項目があっても EL2 で読める
        z = el2.load_trial_npz(str(pe))
        assert z.T == ra.T


@pytest.mark.skipif(not DATA, reason='10/22 の実測データがない')
def test_timeseries_csv_equal_except_ff(tmp_path):
    """write_trial_timeseries は FF{d}・ADRC{d}（変更: 配信の時刻で計算）以外の列が EL2 と一致する"""
    for path in DATA:
        ra, rb = _rec_from(path)
        a, b = tmp_path / 'a.csv', tmp_path / 'b.csv'
        c.write_trial_timeseries(str(a), ra, {})
        el2.write_trial_timeseries(str(b), rb, {})
        with open(a, newline='', encoding='utf-8-sig') as f:
            A = list(csv.reader(f))
        with open(b, newline='', encoding='utf-8-sig') as f:
            B = list(csv.reader(f))
        assert A[0] == B[0]
        keep = [i for i, h in enumerate(A[0]) if not (h.startswith('FF') or h.startswith('ADRC'))]
        assert [[r[i] for i in keep] for r in A] == [[r[i] for i in keep] for r in B]


# ------------------------------------------------------------------ 送信メッセージ（同一プロセス、DOMAIN 87）
@pytest.fixture(scope='module')
def ros():
    rclpy = pytest.importorskip('rclpy')
    from std_msgs.msg import Float32MultiArray
    link = c.OptunaLink('test_optuna_ff_port')                          # 変更: SIGINT ハンドラなしで rclpy を初期化する
    old = el2.RobotLink('test_el2_port')
    node = rclpy.create_node('test_listener')
    got = []
    for topic in [c.TOPIC_TARGET] + [c.TOPIC_FF.format(k) for k in range(1, 6)]:
        node.create_subscription(Float32MultiArray, topic, lambda m, tp=topic: got.append((tp, list(m.data))), 50)
    from rclpy.executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    yield link, old, ex, got
    link.close()
    old.close()
    node.destroy_node()
    rclpy.try_shutdown()


def _collect(ex, got, sec=0.5):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.02)
    out = list(got)
    got.clear()
    return out


def test_signal_option(ros):
    import rclpy.signals as sg
    assert sg.get_current_signal_handlers_options() == sg.SignalHandlerOptions.NO


def test_messages_equal(ros):
    link, old, ex, got = ros
    _collect(ex, got, 0.3)
    ff = np.zeros((c.N_FF, 5))
    ff[3] = c.quintic_from_extrema(1.5, 0.3, 1.1, 123.4567, -98.7654)
    ff[20] = c.quintic_from_extrema(1.5, 0.1, 0.9, -50.123, 80.987)
    pose = [float(v) + 0.123 for v in range(300, 326)]
    link.send_ff(ff, 1.5)
    link.send_targets(pose)
    mine = _collect(ex, got)
    old.send_ff(ff, 1.5)
    old.send_targets(pose)
    theirs = _collect(ex, got)
    key = lambda xs: sorted(xs, key=lambda x: x[0])
    assert key(mine) == key(theirs) and len(mine) == 6
    assert link.last_ff[1] == [float(v) for v in np.asarray(dict(mine)[c.TOPIC_FF.format(1)], dtype=np.float32)]


def test_stop_order_intended_difference(ros):
    """わざと変えた点: 停止は「係数そのまま T=0 → 係数0・T=0 → 静止を待つ → 初期姿勢」（EL2 は「係数0・T → 0.1 s → 初期姿勢」）"""
    link, old, ex, got = ros
    _collect(ex, got, 0.3)
    ff = np.zeros((c.N_FF, 5))
    ff[3] = c.quintic_from_extrema(1.5, 0.3, 1.1, 100.0, -80.0)
    link.send_ff(ff, 1.5)
    _collect(ex, got, 0.3)
    pose = [300.0] * c.N_DOF
    t_mark = time.monotonic()
    c.safe_return(link, pose, [3], c.SafetyConfig(still_wait=0.0), lambda *a: None)
    # 順序は送信側の記録で確かめる（受信側では別のトピックどうしの到着順は保証されない）
    sent = [(tp, list(v)) for t, tp, v in list(link.own) if t >= t_mark and tp in (c.TOPIC_FF.format(1), c.TOPIC_TARGET)]
    kinds = [tp for tp, _v in sent]
    ffs = [v[18:24] for tp, v in sent if tp == c.TOPIC_FF.format(1)]
    assert ffs[0][5] == 0.0 and np.allclose(ffs[0][:5], np.asarray(ff[3], dtype=np.float32))   # ① 係数そのまま T=0
    assert ffs[1] == [0.0] * 6                                                                # ② 係数0・T=0
    assert kinds[:2] == [c.TOPIC_FF.format(1)] * 2 and kinds[2] == c.TOPIC_TARGET            # ③ その後に初期姿勢
    _collect(ex, got, 0.5)
    el2.return_to_initial(old, pose, 1.5)
    theirs = [x for x in _collect(ex, got, 0.5) if x[0] in (c.TOPIC_FF.format(1), c.TOPIC_TARGET)]
    assert dict(theirs)[c.TOPIC_FF.format(1)][18:24] == [0.0, 0.0, 0.0, 0.0, 0.0, 1.5]      # EL2 は T を残す


def test_capture_counts_intended_difference(ros):
    """わざと変えた点: 測定の終了条件は「/tk の受信数（配信数）」。EL2 は PWM の値の変化の数（周回数）"""
    link, _old, _ex, _got = ros
    link.start_capture(0.0)
    t0 = time.monotonic()
    for k in range(5):
        link._cb('board1', 'pot', type('M', (), {'data': [300 + (k > 2)] * 12})())
        link._cb('board1', 'pwm', type('M', (), {'data': [1.0] * 6})())
    cnt = link.capture_counts_since(t0 - 1.0)
    old_cnt = link.capture_counts(t0 - 1.0)
    link.stop_capture()
    assert cnt['board1'] == 5 and old_cnt['board1'] == 0             # 同じ PWM の値が続くと EL2 の数え方では 0


# ------------------------------------------------------------------ GUI 部品
def test_gui_parts_equal():
    tk = pytest.importorskip('tkinter')
    if not os.environ.get('DISPLAY'):
        pytest.skip('DISPLAY がない')
    import optimize_EL2_gui as el2g
    import optimize_optuna_ff_ROS2 as app
    try:
        ra = app.setup_root('A')
    except tk.TclError:
        pytest.skip('Tk を開けない')
    fa = app.UI_FONT
    ra.destroy()
    rb = el2g.setup_root('B')
    import GUI_android_tk as gat
    fb = gat.UI_FONT
    rb.destroy()
    assert fa == fb

    def tree(w):
        return [type(w).__name__] + [tree(ch) for ch in w.winfo_children()]

    trees = []
    for mod, lg in ((app, app.Logger()), (el2g, el2g.Logger())):
        root = tk.Tk()
        mod.LogPane(root, lg)
        mod.Fields(root, 'x').add('a', 'A', 1)
        mod.TargetTable(root, [300.0] * c.N_DOF, [300.0] * c.N_DOF)
        trees.append(tree(root))
        root.destroy()
    assert trees[0] == trees[1]
