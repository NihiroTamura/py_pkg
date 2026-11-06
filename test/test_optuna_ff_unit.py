"""optuna_ff_common.py のオフライン単体テスト（ROS は使わない。計画書 §10.1）

実行: cd ~/ArmRobot/src/py_pkg && python3 -m pytest -q test/test_optuna_ff_unit.py
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_pkg'))
import optuna_ff_common as c  # noqa: E402


@pytest.fixture(scope='session')
def table(tmp_path_factory):
    return c.FFMapTable.cached(cache_dir=str(tmp_path_factory.mktemp('ffmap')))


@pytest.fixture(scope='session')
def ffmap(table):
    return c.FFMap(table, 255.0)


# ------------------------------------------------------------------ 写像
def test_counterexample_rejected():
    """α = γ = −1, β = 10 は「α・γ が同符号かつ β > −√(αγ)」なら通ってしまうが、Q は (0,1) に根を2つ持つので棄却する"""
    assert not bool(c.bernstein_positive(-1.0, 10.0, -1.0))
    # Q(s) = −(1−s)² + 20 s(1−s) − s² は (0,1) に根を2つ持つ
    s = np.linspace(0, 1, 1001)
    q = -(1 - s) ** 2 + 20 * s * (1 - s) - s ** 2
    assert np.sum(np.diff(np.sign(q)) != 0) == 2


def test_r_intervals_match_root_counting():
    rng = np.random.default_rng(0)
    mis = 0
    for _ in range(3000):
        s1 = rng.uniform(0.02, 0.9)
        s2 = rng.uniform(s1 + 0.02, 0.98)
        r = 10 ** rng.uniform(-2.5, 2.5)
        lo, hi, cont = c.r_intervals(np.array([s1]), np.array([s2]))
        assert bool(cont[0])
        pred = bool(np.isfinite(lo[0]) and lo[0] < r < hi[0])
        true = c.extrema_ok(c.quintic_from_extrema(1.0, s1, s2, 1.0, -r), 1.0) is not None
        mis += pred != true
    assert mis == 0


def test_table_bounds_exact_and_nonempty(table):
    """写像の全域（s1 を 4001 点 × u2 を 201 点）で、余裕を取った r の区間が空でない（r_lo < r_hi）"""
    assert table.verify(4001, 201) == 0
    for s1 in np.linspace(table.s1min, table.s1max, 501):
        lo, hi = table.bounds(s1)
        s2 = lo + np.linspace(0, 1, 51) * (hi - lo)
        rl, rh = c.r_interval_margin(np.full(51, s1), s2)
        assert np.all(np.isfinite(rl)) and np.all(rh > rl)


def test_random_designs_satisfy_conditions(ffmap):
    """u をランダムに 10⁵ 点取り、写像した候補がすべて「極値ちょうど2個（根の数え上げと閉形式が一致）・|FF| ≤ A_max・端点 0」"""
    rng = np.random.default_rng(1)
    n = int(os.environ.get('OPTUNA_FF_N_RANDOM', '100000'))
    T = 1.0
    bad = 0
    P = []
    for _ in range(n):
        u = rng.uniform(0, 1, 3)
        A = rng.uniform(-255, 255)
        d = ffmap.design(u[0], u[1], u[2], A, T)
        co = d['coeffs']
        P.append(co)
        a, b, cc, dd, e = co
        if abs(np.polyval([a, b, cc, dd, e, 0.0], T)) > 1e-9 or max(abs(d['y1']), abs(d['y2'])) > 255 + 1e-9:
            bad += 1
        al, be, ga = c._q_bernstein(np.asarray(co), d['s1'], d['s2'])
        if not bool(c.bernstein_positive(al * np.sign(d['y1']), be * np.sign(d['y1']), ga * np.sign(d['y1']))):
            bad += 1
    assert bad == 0
    counts = c.count_interior_batch(np.array(P), T)
    assert np.all(counts == 2)


def test_presend_and_float32(ffmap):
    rng = np.random.default_rng(2)
    for _ in range(300):
        u = rng.uniform(0, 1, 3)
        T = rng.uniform(0.3, 3.0)
        d = ffmap.design(u[0], u[1], u[2], rng.uniform(-255, 255), T)
        chk = c.presend_check(d['coeffs'], T, 255.0)
        assert chk['ok'], chk['reasons']
        t_ms = np.arange(0, int(1000 * T))
        f32 = c.firmware_ff(d['coeffs'], T, t_ms).astype(float)
        f64 = c.ff_eval(d['coeffs'], T, t_ms / 1000.0)
        assert np.max(np.abs(f32 - f64)) < 0.05      # float32 の再現と float64 の差（観測した最大は約 0.013 PWM）


def test_round_trip(ffmap):
    """極値 → 係数 → float32 → extrema_of → 係数 が、送った係数と許容内（FF の最大差 0.5 PWM）で一致する"""
    rng = np.random.default_rng(3)
    for _ in range(300):
        u = rng.uniform(0, 1, 3)
        T = rng.uniform(0.3, 3.0)
        d = ffmap.design(u[0], u[1], u[2], rng.uniform(-255, 255), T)
        sent = d['coeffs'].astype(np.float32).astype(float)
        t1, y1, t2, y2 = c.extrema_of(sent, T)                 # 結果 CSV と同じく、送った float32 の係数と T から求める
        back = c.quintic_from_extrema(T, t1, t2, y1, y2)
        tt = np.arange(0, int(1000 * T)) / 1000.0
        # 送った係数は float32 に丸めてあり f(T)=0 を厳密には満たさないので、その分を許容する（送信前の検証の |f(T)| と同じ
        # 0.5 PWM。観測した最大は約 0.2 PWM）
        assert np.max(np.abs(c.ff_eval(back, T, tt) - c.ff_eval(sent, T, tt))) < 0.5
        inv, why = ffmap.inverse(d['t1'], d['y1'], d['t2'], d['y2'], T)
        assert inv is not None, why
        assert max(abs(inv['u1'] - u[0]), abs(inv['u2'] - u[1]), abs(inv['u3'] - u[2])) < 1e-8


def test_signed_amplitude_min(ffmap):
    d = ffmap.design(0.5, 0.5, 0.5, 0.0, 1.0)
    assert abs(max(abs(d['y1']), abs(d['y2'])) - 1.0) < 1e-12
    d = ffmap.design(0.5, 0.5, 0.5, -100.0, 1.0)
    assert d['y1'] < 0 < d['y2'] and abs(max(abs(d['y1']), abs(d['y2'])) - 100.0) < 1e-9


# ------------------------------------------------------------------ 目標軌道
def _synthetic(T1, wn, y0, T, noise=0.3, seed=0):
    N = c.n_eval(T)
    y = c.target_states(T1, wn, y0, c.TICK_DT, N)[:, 0]
    return y + np.random.default_rng(seed).normal(0, noise, N + 1)


def test_theta_monotone_and_large_kappa():
    eps = 0.02
    for kappa in (1e-3, 0.1, 1.0, 10.0, 1e3):
        th = c.completion_theta(kappa, eps)
        assert abs(c.phi_at(kappa, 1.0, th) - eps) < 1e-9
    assert c.completion_theta(1e3, eps) > 3000                       # 区間を広げて求まる
    assert c.completion_theta(1.0, 1.5) == 0.0                        # ε_eff ≥ 1


def test_constrained_fit_active():
    T = 1.0
    ys = [_synthetic(0.25, 6.0, -100.0, T, seed=s) for s in range(2)]
    sp = c.fit_target_constrained(0, ys, [np.isfinite(y) for y in ys], [200.0, 200.0], 200.0, 300.0, T, 0.02)
    assert sp.active
    assert c.completion_time(sp.T1, sp.wn, sp.eps_eff) <= T * (1 + 1e-6)
    seq = sp.seq() - sp.Pf
    k = int(np.ceil(T / c.TICK_DT))
    assert np.all(np.abs(seq[k:]) <= 0.02 * 100.0 + 1e-9)            # T 以後は許容帯の中


def test_constrained_fit_inactive():
    T = 2.0
    ys = [_synthetic(0.05, 12.0, -100.0, T)]
    sp = c.fit_target_constrained(0, ys, [np.isfinite(ys[0])], [200.0], 200.0, 300.0, T, 0.02)
    assert not sp.active and sp.method == 'free' and sp.warnings


def test_constrained_fit_eps_eff_ge_1():
    T = 1.0
    ys = [_synthetic(0.1, 8.0, -1.0, T, noise=0.01)]
    sp = c.fit_target_constrained(0, ys, [np.isfinite(ys[0])], [299.0], 290.0, 300.0, T, 0.2)
    assert sp.eps_eff >= 1.0 and not sp.active


# ------------------------------------------------------------------ 100 Hz の時間軸
def _arrivals(n, drop_rate=0.03, clock=0.95, burst=True, seed=0, edge_drop=True):
    rng = np.random.default_rng(seed)
    j = np.arange(n)
    keep = rng.uniform(size=n) > drop_rate
    keep[0] = True
    if edge_drop:
        keep[-2] = False
    lat = rng.exponential(0.0008, n) + 0.0003
    if burst:
        for b in rng.choice(np.arange(20, n - 20), 5, replace=False):
            lat[b:b + 3] += np.array([0.025, 0.015, 0.005])         # 一時的な遅れ → まとまって届く
    t = clock * (j * c.TICK_DT + lat)
    t = np.maximum.accumulate(t)
    return t[keep], j[keep]


@pytest.mark.parametrize('seed', range(10))
def test_assign_ticks_drops_only(seed):
    """PC の時計 0.95 倍・欠損 3%・端の欠損（まとまった到着なし）では、配信番号が完全に一致する"""
    t, jt = _arrivals(400, seed=seed, burst=False)
    j, u, n_drop = c.assign_ticks(t)
    assert np.array_equal(j - j[0], jt - jt[0])
    assert abs(u - 0.95 * c.TICK_DT) < 2e-4


@pytest.mark.parametrize('seed', range(10))
def test_assign_ticks_with_bursts(seed):
    """一時的な遅れ（まとまった到着）を足すと、その直後の欠損の位置は受信時刻だけでは決まらないので ±2 配信までずれてよい。
    ずれは局所的で、たまらない（最後は一致に戻る）"""
    t, jt = _arrivals(400, seed=seed, burst=True)
    j, u, n_drop = c.assign_ticks(t)
    diff = (j - j[0]) - (jt - jt[0])
    assert np.all(np.abs(diff) <= 2)
    assert np.mean(diff != 0) <= 0.03
    assert diff[-1] == 0


def test_assign_ticks_burst_without_drop():
    """欠損なしで一時的な遅れだけなら完全に一致する"""
    t, jt = _arrivals(400, seed=7, burst=True, drop_rate=0.0, edge_drop=False)
    j, _u, n_drop = c.assign_ticks(t)
    assert np.array_equal(j - j[0], jt - jt[0]) and n_drop == 0


def _rate_change(n, seed, drop_rate, u_a, u_b, n_change=250):
    """受信間隔が途中で u_a → u_b に変わる（2026-10-27 の実機で見た PC 側の受信の変化。例 9.575 → 9.88 ms）"""
    rng = np.random.default_rng(seed)
    j = np.arange(n)
    lat = rng.exponential(0.0008, n) + 0.0003
    t = np.concatenate([[0.0], np.cumsum(np.where(j < n_change, u_a, u_b)[1:])]) + lat
    keep = rng.uniform(size=n) > drop_rate
    keep[0] = True
    keep[-2] = False
    return np.maximum.accumulate(t)[keep], j[keep]


@pytest.mark.parametrize('u_a,u_b', [(0.009575, 0.00988), (0.0101, 0.00947), (0.0094, 0.0101)])
@pytest.mark.parametrize('seed', range(5))
@pytest.mark.parametrize('drop_rate', [0.0, 0.03])
def test_assign_ticks_rate_change(u_a, u_b, seed, drop_rate):
    """受信間隔が途中で変わっても欠損と数えない（位相法は区間全体で1つの周期を仮定したので、ここで存在しない欠損を数えた）"""
    t, jt = _rate_change(400, seed, drop_rate, u_a, u_b)
    j, _u, n_drop = c.assign_ticks(t)
    assert np.array_equal(j - j[0], jt - jt[0])
    assert n_drop == jt[-1] - jt[0] - (len(jt) - 1)


def test_count_drops_phase_overcounts_on_rate_change():
    """検算用の位相法は、欠損だけなら正しく数えるが、受信間隔が途中で変わると多めに数える（新方式に替えた理由）"""
    t, jt = _arrivals(400, seed=0, burst=False)
    assert c.count_drops_phase(t)[0] == jt[-1] - jt[0] - (len(jt) - 1)
    t, jt = _rate_change(400, 0, 0.0, 0.009575, 0.00988)        # 欠損は端の1個だけ
    n_true = jt[-1] - jt[0] - (len(jt) - 1)
    assert c.count_drops_phase(t)[0] > n_true and c.assign_ticks(t)[2] == n_true


def test_assign_ticks_drop_right_after_delay():
    """遅れてまとまって届いた直後の欠損：大きな間隔（3.5 配信）は遅れ、その後の 2 配信の間隔が欠損"""
    n = 40
    lat = np.full(n, 0.0003)
    lat[10:13] += [0.025, 0.015, 0.005]
    t = np.maximum.accumulate(np.arange(n) * c.TICK_DT + lat)
    keep = np.ones(n, dtype=bool)
    keep[15] = False
    j, _u, n_drop = c.assign_ticks(t[keep])
    assert np.array_equal(j, np.arange(n)[keep]) and n_drop == 1


def test_assign_ticks_long_stall_then_burst():
    """PC が 200 ms 止まって、その間の配信がまとめて届く（欠損なし）。間隔は 21 配信分だが、追いつくので欠損 0"""
    n = 120
    lat = np.full(n, 0.0003)
    lat[50:70] += 0.200 - np.arange(20) * c.TICK_DT
    t = np.maximum.accumulate(np.arange(n) * c.TICK_DT + lat)
    j, _u, n_drop = c.assign_ticks(t)
    assert np.array_equal(j, np.arange(n)) and n_drop == 0
    keep = np.ones(n, dtype=bool)
    keep[30:50] = False                                   # 本当に 20 配信欠けた場合
    j, _u, n_drop = c.assign_ticks(t[keep])
    assert n_drop == 20 and j[-1] == n - 1


def _slow_catchup(n, seed, drop_rate):
    """大きな遅れの後、6 受信ほどかけて少しずつ追いつく（偽ボードの通しテストで見た形。遅れ 25→27→20→12→6→1 ms）"""
    rng = np.random.default_rng(seed)
    j = np.arange(n)
    lat = rng.exponential(0.0008, n) + 0.0003
    for b in rng.choice(np.arange(20, n - 30), 4, replace=False):
        lat[b:b + 6] += np.array([0.025, 0.027, 0.020, 0.012, 0.006, 0.001]) * rng.uniform(0.6, 1.4)
    t = np.maximum.accumulate(j * c.TICK_DT + lat)
    keep = rng.uniform(size=n) > drop_rate
    keep[0] = True
    return t[keep], j[keep]


@pytest.mark.parametrize('seed', range(10))
def test_assign_ticks_slow_catchup(seed):
    """ゆっくり追いつく遅れは欠損と数えない（後ろの窓を追いつきの途中の受信の先まで伸ばす）"""
    t, jt = _slow_catchup(400, seed, 0.0)
    j, _u, n_drop = c.assign_ticks(t)
    assert np.array_equal(j, jt) and n_drop == 0


def test_local_intervals_matches_loop():
    rng = np.random.default_rng(3)
    d = 0.0095 + rng.normal(0, 0.0004, 300)
    d[[20, 21, 150]] = [0.03, 0.0, 0.019]
    ref = []
    for i in range(len(d)):
        w = d[max(0, i - 50):i + 51]
        med = np.median(w)
        w = w[(w > 0.5 * med) & (w < 1.5 * med)]
        ref.append(np.median(w))
    assert np.allclose(c.local_intervals(d), ref, rtol=0, atol=0)


_REAL_DIR = os.path.expanduser('~/ArmRobot/optimize_data/verify_optuna_20261027/optimize_20261027_1451/trials')


@pytest.mark.skipif(not os.path.isdir(_REAL_DIR), reason='2026-10-27 の実機の記録がない')
@pytest.mark.parametrize('name', ['trial_069_a0.npz', 'trial_069_a1.npz', 'trial_069_a2.npz', 'trial_069_a3.npz',
                                  'trial_014_a0.npz', 'trial_045_a0.npz'])
def test_assign_ticks_real_remeasured(name):
    """2026-10-27 の実機（D=1000）で位相法が欠損を数えて測り直し、comm で止まった記録。新方式では全ボード欠損 0"""
    z = np.load(os.path.join(_REAL_DIR, name))
    for b in ('board1', 'board2', 'board3', 'board4', 'board5'):
        assert c.assign_ticks(z[f'{b}_pot_t'])[2] == 0


def _fake_raw(j0_true, n, initial, target, dofs, t_send, seed=0):
    """board1 の /tk・PWM を合成する（POT は目標へ一次遅れ、エコーは j0 から新しい目標値）"""
    rng = np.random.default_rng(seed)
    t_all = np.arange(n) * c.TICK_DT
    V = np.zeros((n, 12))
    W = np.zeros((n, 6))
    for d in range(6):
        P = np.full(n, float(initial[d]))
        for k in range(j0_true, n):
            P[k] = target[d] + (initial[d] - target[d]) * math.exp(-(k - j0_true) * c.TICK_DT / 0.2)
        V[:, d] = np.round(P)
        V[:, 6 + d] = np.where(np.arange(n) >= j0_true, round(target[d]), round(initial[d]))
        W[:, d] = rng.normal(0, 1, n)
    t = t_all + rng.exponential(0.0005, n)
    return {'board1': {'pot': (t, V), 'pwm': (t + 0.0002, W)}}


def test_save_raw_attempt_keeps_params_and_sent_ff(tmp_path):
    """測り直した回・中断した回の npz にも、送った FF（float32）と試行開始時・終了時の ADRC パラメータが入る"""
    rng = np.random.default_rng(0)
    raw = {'board1': {'pot': (np.arange(5) * 0.01, rng.integers(0, 1000, (5, 12)).astype(float))}}
    ff = np.zeros((c.N_FF, 5))
    ff[0] = [1234.5678901, -2345.6789012, 3456.7890123, -456.78901234, 56.789012345]
    before = {0: c.AdrcParams(1000.0, 144.0, 50000.0, 700.0), 1: None}
    after = {0: c.AdrcParams(1000.0, 144.0, 50000.0, 650.0)}
    path = str(tmp_path / 'trial_001_a0.npz')
    c.save_raw_attempt(path, raw, [300.0] * c.N_DOF, [310.0] * c.N_DOF, ff, 1.0, 0.5, '測り直し', before, after)
    with np.load(path) as z:
        assert np.array_equal(z['param_0'], [1000.0, 144.0, 50000.0, 700.0]) and 'param_1' not in z.files
        assert np.array_equal(z['x_param_after_0'], [1000.0, 144.0, 50000.0, 650.0])
        assert np.array_equal(z['x_ff_sent_rows'], ff.astype(np.float32).astype(float))
        assert str(z['x_note']) == '測り直し'
    rec = c.load_trial_npz(path)
    assert rec.params[0].lam == 700.0 and rec.params[1] is None and rec.aborted
    # ADRC パラメータを読む前に失敗した回（params が None）でも保存できる
    path2 = str(tmp_path / 'trial_002_stop.npz')
    c.save_raw_attempt(path2, raw, [300.0] * c.N_DOF, [310.0] * c.N_DOF, ff, 1.0, 0.5, '停止', None, None)
    with np.load(path2) as z:
        assert not any(k.startswith('param_') or k.startswith('x_param_after_') for k in z.files)
        assert 'x_ff_sent_rows' in z.files


def test_extract_series_j0_and_window():
    initial = [300.0] * c.N_DOF
    target = list(initial)
    target[3] = 400.0
    T = 1.0
    j0_true = 150
    raw = _fake_raw(j0_true, 150 + c.n_eval(T) + 40, initial, target, [3], t_send=j0_true * c.TICK_DT - 0.004)
    series, info = c.extract_series(raw, initial, target, [3], j0_true * c.TICK_DT - 0.004, T)
    s = series[3]
    assert info['board1']['j0'] == j0_true
    assert s.missing() == 0 and abs(s.P0 - 300.0) < 1e-9
    assert s.pot[s.H] == 300.0 and s.echo[s.H] == 400.0 and s.echo[s.H - 1] == 300.0


def test_extract_series_echo_missing():
    initial = [300.0] * c.N_DOF
    target = list(initial)
    target[3] = 400.0
    raw = _fake_raw(10 ** 6, 300, initial, initial, [3], 0.0)
    with pytest.raises(c.TrialIssue) as e:
        c.extract_series(raw, initial, target, [3], 0.5, 1.0)
    assert e.value.kind == 'comm'


# ------------------------------------------------------------------ FAIL の管理
@pytest.fixture()
def study(tmp_path, ffmap):
    return c.FFStudy(str(tmp_path), [3], 255.0, 'TPE', 0, 12, ffmap, rho=0.05, tabu_on=True, log=lambda *a: None)


def test_fail_is_fail_and_attrs_before_tell(study):
    TS = study.TS()
    t = study.ask()
    study.stage(t, 'measuring')
    study.tell_fail(t, 'unsafe', 'DOF 3: 範囲外', {'files': ['x.npz']})
    ft = study.trials()[t.number]
    assert ft.state == TS.FAIL and ft.value is None
    assert ft.user_attrs['fail_class'] == 'unsafe' and ft.user_attrs['fail_stage'] == 'measuring'
    assert ft.user_attrs['files'] == ['x.npz']
    assert study.tabu_hit(study.u_of(ft.params)) == 0.0                # タブーに登録される


def test_best_ignores_fail_and_nan(study):
    TS = study.TS()
    t1 = study.ask()
    study.tell_complete(t1, 0.02, {})
    t2 = study.ask()
    study.tell_fail(t2, 'eval', 'J が NaN')
    t3 = study.ask()
    study.tell_complete(t3, 0.01, {})
    assert study.best().number == t3.number
    assert study.n_complete() == 2
    assert study.trials()[t2.number].state == TS.FAIL


def test_retry_rules(study):
    TS = study.TS()
    t = study.ask()
    study.tell_fail(t, 'comm', '測り直しの上限')
    ft = study.trials()[t.number]
    assert study.retry(ft, 'test') is True
    assert study.retry(ft, 'test') is False                            # 1回まで
    t2 = study.ask()                                                    # 再投入された試行
    assert t2.user_attrs.get('retry_of') == t.number
    assert t2.params == ft.params
    study.tell_fail(t2, 'comm', 'again')
    assert study.retry(study.trials()[t2.number], 'test') is False     # 再投入した試行が失敗しても入れない
    t3 = study.ask()
    study.tell_fail(t3, 'unsafe', 'x')
    assert study.retry(study.trials()[t3.number], 'test') is False     # unsafe は再投入しない
    assert TS.FAIL == study.trials()[t3.number].state


def test_orphan_on_resume(tmp_path, ffmap):
    st = c.FFStudy(str(tmp_path), [3], 255.0, 'TPE', 0, 12, ffmap, log=lambda *a: None)
    t = st.ask()
    st.stage(t, 'measuring')                                            # RUNNING のまま（異常終了）
    st2 = c.FFStudy(str(tmp_path), [3], 255.0, 'TPE', 0, 12, ffmap, resume=True, log=lambda *a: None)
    assert st2.handle_orphans() == 1
    ft = st2.trials()[t.number]
    assert ft.state == st2.TS().FAIL and ft.user_attrs['fail_class'] == 'orphan'
    nxt = st2.ask()
    assert nxt.user_attrs.get('retry_of') == t.number and nxt.params == ft.params


def test_confirm_and_past(study):
    t = study.ask()
    study.tell_complete(t, 0.0001, {})
    ft = study.add_confirm(dict(t.params), 0.0002, {'confirm_of': t.number})
    assert ft.state == study.TS().COMPLETE and ft.user_attrs['confirm_of'] == t.number
    ff = study.add_confirm(dict(t.params), attrs={'confirm_of': t.number}, fail_class='stopped', reason='停止')
    assert ff.state == study.TS().FAIL and ff.value is None


def test_n_startup_counts():
    """TPE は COMPLETE と PRUNED、GP は COMPLETE だけを数える（FAIL は数えない）ので n_startup = 事前試行数 + max(10, 2D)"""
    import inspect
    import optuna
    src = inspect.getsource(optuna.samplers.TPESampler._sample_relative)
    assert 'TrialState.COMPLETE, TrialState.PRUNED' in src


def test_eval_time_at_least_5s():
    """評価区間 = max(1.5T, 5 秒)（1.5T が5秒未満なら5秒、超えるなら 1.5T）"""
    assert c.eval_time(1.0) == 5.0 and c.n_eval(1.0) == 500
    assert c.eval_time(0.3) == 5.0 and c.n_eval(0.3) == 500
    assert c.eval_time(10.0 / 3.0) == pytest.approx(5.0) and c.n_eval(10.0 / 3.0) == 500
    assert c.eval_time(4.0) == 6.0 and c.n_eval(4.0) == 600


def test_eval_time_fixed_value_from_gui():
    """評価区間 = max(1.5T, 固定値)。固定値は GUI で変えられ、既定は従来どおり 5 秒"""
    assert c.eval_time(1.0) == c.eval_time(1.0, c.EVAL_MIN_S) == 5.0
    assert c.eval_time(1.0, 3.0) == 3.0 and c.n_eval(1.0, 3.0) == 300
    assert c.eval_time(1.0, 8.0) == 8.0 and c.n_eval(1.0, 8.0) == 800
    assert c.eval_time(4.0, 3.0) == 6.0 and c.n_eval(4.0, 3.0) == 600      # 1.5T の方が長ければ 1.5T
    assert c.eval_time(1.0, 0.0) == 1.5 and c.n_eval(1.0, 0.0) == 150


@pytest.mark.parametrize('E', [2.0, 5.0, 7.5])
def test_extract_series_window_follows_eval_min(E):
    """切り出す評価区間の配信数が max(1.5T, 固定値) に従う（T は動作完了時間）"""
    initial = [300.0] * c.N_DOF
    target = list(initial)
    target[3] = 400.0
    T, j0_true = 1.0, 150
    t_send = j0_true * c.TICK_DT - 0.004
    raw = _fake_raw(j0_true, 150 + c.n_eval(T, E) + 40, initial, target, [3], t_send=t_send)
    series, _info = c.extract_series(raw, initial, target, [3], t_send, T, eval_min=E)
    s = series[3]
    assert s.N_e == c.n_eval(T, E) == int(round(max(1.5 * T, E) / c.TICK_DT))
    assert len(s.pot[s.win()]) == s.N_e + 1 and s.missing() == 0


def _rec(T, T_ff=None, eval_min=c.EVAL_MIN_S, ff=None, raw=None):
    return c.TrialRecord(raw=raw or {}, initial=[300.0] * c.N_DOF, target=[310.0] * c.N_DOF,
                         ff=np.zeros((c.N_FF, 5)) if ff is None else ff, T=T, L=c.TICK_DT, params={}, params_after={},
                         t_target=0.0, started_at='2026-11-04 00:00:00', T_ff=T_ff, eval_min=eval_min)


def test_npz_keeps_T_ff_and_eval_min(tmp_path):
    """npz に動作完了時間 T・FF 入力時間 T_FF・評価時間の固定値を別々に残し、読み戻せる。分ける前の記録は T_FF = T、5 秒"""
    assert _rec(0.6).T_ff == 0.6                                    # T_FF を与えなければ T と同じ
    p = str(tmp_path / 'trial_000.npz')
    c.save_trial_npz_ext(p, _rec(0.6, 0.9, 2.0), {})
    r = c.load_trial_npz(p)
    assert (r.T, r.T_ff, r.eval_min) == (0.6, 0.9, 2.0)
    p_old = str(tmp_path / 'old.npz')
    c.save_trial_npz(p_old, _rec(0.6, 0.9, 2.0))                   # 移植元と同じ形式（x_T_ff・x_eval_min がない）
    r = c.load_trial_npz(p_old)
    assert (r.T, r.T_ff, r.eval_min) == (0.6, 0.6, c.EVAL_MIN_S)
    p_a = str(tmp_path / 'trial_001_a0.npz')
    c.save_raw_attempt(p_a, {}, [300.0] * c.N_DOF, [310.0] * c.N_DOF, np.zeros((c.N_FF, 5)), 0.6, 0.5, '測り直し',
                       T_ff=0.9, eval_min=2.0)
    r = c.load_trial_npz(p_a)
    assert (r.T, r.T_ff, r.eval_min) == (0.6, 0.9, 2.0)


def test_timeseries_ff_uses_T_ff(tmp_path, ffmap):
    """best_trial.csv の FF{d} 列は FF 入力時間 T_FF で切れる（動作完了時間 T では切れない）"""
    import csv
    T, T_ff = 0.6, 0.9
    ff = np.zeros((c.N_FF, 5))
    ff[c.FF_INDEX[3]] = ffmap.design(0.5, 0.5, 0.5, 100.0, T_ff)['coeffs']
    n = 151
    t = np.arange(n) * c.TICK_DT
    raw = {'board1': {'pot': (t, np.zeros((n, 12))), 'pwm': (t, np.zeros((n, 6)))}}
    path = str(tmp_path / 'best_trial.csv')
    c.write_trial_timeseries(path, _rec(T, T_ff, ff=ff, raw=raw), {'board1': t})
    with open(path, newline='', encoding='utf-8-sig') as f:
        rows = list(csv.DictReader(f))
    ffv = np.array([float(r['FF3']) for r in rows[:n]])
    assert np.max(np.abs(ffv[(t > T + 0.02) & (t < T_ff - 0.02)])) > 1.0      # T を過ぎても T_FF までは FF がある
    assert np.all(ffv[t >= T_ff] == 0.0)
    assert np.allclose(ffv, c.ff_eval(ff[c.FF_INDEX[3]], T_ff, t), atol=1e-4 * np.max(np.abs(ffv)))


def test_cond_hash_unchanged_when_T_ff_equals_T():
    """T_FF = T・評価時間の固定値 5 秒なら条件 hash は分ける前と同じ（それまでの study を再開できる）。違えば hash も変わる"""
    import optimize_optuna_ff_ROS2 as app
    old = {'m': 0.02, 'sep': 0.02, 'A_max': 255.0, 'rho': 0.05, 'tabu_on': True, 'dofs': [3], 'T': 1.5, 'eps': 0.02,
           'initial': [300.0] * c.N_DOF, 'target': [310.0] * c.N_DOF}
    params = {3: c.AdrcParams(1000.0, 144.0, 50000.0, 700.0)}

    def h(cfg):
        return c.condition_hash(app.OptimizationRun.cond_dict(cfg, params, 'th'))
    assert app.cfg_T_ff(old) == 1.5 and app.cfg_eval_min(old) == c.EVAL_MIN_S and app.cfg_eval_time(old) == 5.0
    new = dict(old, T_ff=1.5, eval_min=5.0)
    assert h(new) == h(old)
    assert h(dict(new, T_ff=0.9)) != h(old)
    assert h(dict(new, eval_min=3.0)) != h(old)
    assert app.OptimizationRun.cond_dict(dict(new, eval_min=3.0), params, 'th')['eval_rule'] == 'max(1.5T, 3 s)'


def test_saved_png_english_and_eval_range():
    """保存する画像（g1・g4・g5）：文字は英語、軸ラベルは指定どおり、横軸は 0 秒〜評価時間（g2・g3 も文字は英語）"""
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib.figure import Figure
    import optimize_optuna_ff_ROS2 as app
    t = np.arange(-120, 516) * c.TICK_DT
    p = {'T': 1.0, 'T_ff': 3.0, 't_eval': 5.0, 't': t, 'pot': 300 + 0 * t, 't_win': t[120:621], 'target': 300 + 0 * t[120:621],
         'ref': 300 + 0 * t, 'best': None, 'pwm': 0 * t, 'adrc': 0 * t, 'ff_t': np.arange(0, 5001) / 1000.0,
         'ff': np.zeros(5001), 'ext': (0.5, 10.0, 2.0, -10.0)}
    want = {'g1': ('POT [-]', app.plot_g1), 'g4': ('PWM [-]', app.plot_g4), 'g5': ('PWM [-]', app.plot_g5)}
    for key, (ylab, fn) in want.items():
        ax = Figure().add_subplot(111)
        fn(ax, app.SAVE_LANG, 3, p, 7, final=True)
        assert (ax.get_xlabel(), ax.get_ylabel()) == ('Time [s]', ylab), key
        assert ax.get_xlim() == (0.0, 5.0), key
        texts = [ax.get_title()] + [x.get_text() for x in ax.get_legend().get_texts()] + [x.get_text() for x in ax.texts]
        assert all(ch.isascii() for s in texts for ch in s), (key, texts)
        assert all(np.min(ln.get_xdata()) >= 0.0 and np.max(ln.get_xdata()) <= 5.0 + 1e-9
                   for ln in ax.get_lines() if len(ln.get_xdata()) > 2), key        # y 軸の範囲も評価区間の値で決まる
    hist = [{'n': 0, 'state': 'COMPLETE', 'J': 0.5, 'J_i': {'3': 0.4}, 'ext': {'3': [0.5, 10.0, 2.0, -10.0]}}]
    ax = Figure().add_subplot(111)
    app.plot_g2(ax, app.SAVE_LANG, 3, hist, 0.01)
    ax2 = Figure().add_subplot(111)
    app.plot_g3(ax2, ax2.twinx(), app.SAVE_LANG, 3, hist)
    for a in (ax, ax2):
        texts = [a.get_title(), a.get_xlabel(), a.get_ylabel()] + [x.get_text() for x in a.get_legend().get_texts()]
        assert all(ch.isascii() for s in texts for ch in s), texts


def test_cost_png_has_reference_and_progress_png(tmp_path):
    """g2 にその自由度の FF=0 の基準試行（J_i の平均）の横線だけを描き（全体の J の基準は描かない）、
    終了時に分析フォルダ analysis_YYYYMMDD_HHMM/J_progress.png を作る"""
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib.figure import Figure
    import optimize_optuna_ff_ROS2 as app
    hist = [{'n': k, 'state': 'COMPLETE', 'J': 1.0 / (k + 1), 'J_i': {'3': 0.5 / (k + 1)}, 'ext': {'3': [0.5, 10.0, 2.0, -10.0]}}
            for k in range(6)] + [{'n': 6, 'state': 'FAIL', 'fail_class': 'unsafe'}]
    ref = {'J_i': [0.02, 0.04]}
    ax = Figure().add_subplot(111)
    app.plot_g2(ax, app.SAVE_LANG, 3, hist, 0.01, ref)
    ys = sorted(round(float(ln.get_ydata()[0]), 6) for ln in ax.get_lines() if len(set(np.atleast_1d(ln.get_ydata()))) == 1
                and len(ln.get_xdata()) == 2)
    assert ys == [0.03]                                                       # 基準の J_3 の平均の横線だけ（J と閾値は描かない）
    labels = [x.get_text() for x in ax.get_legend().get_texts()]
    assert labels == ['J_3', 'J_3 at best J', 'reference J_3 (FF=0) = 0.03']
    ref_line = [ln for ln in ax.get_lines() if ln.get_label().startswith('reference')][0]
    assert ref_line.get_color() == 'k' and ref_line.get_linestyle() == '-.'                  # 黒の一点鎖線
    assert all(ch.isascii() for s in labels for ch in s)
    folder = tmp_path / 'optimize_20261104_1504'
    info = {'J_ref': [0.08, 0.20], 'J_thr': 0.01, 'rms_thr': 10.0, 'n_startup': 3, 'T': 1.0, 'T_ff': 3.0}
    app.render_end_pngs(str(folder), [3], hist, 0.01, {3: ref}, info)
    assert app.analysis_folder(str(folder)) == str(tmp_path / 'analysis_20261104_1504')
    assert (tmp_path / 'analysis_20261104_1504' / 'J_progress.png').stat().st_size > 1000
    if app.setup_mpl_japanese() == 'ja':                                        # 日本語のフォントがあれば日本語版も
        assert (tmp_path / 'analysis_20261104_1504' / 'J_progress_ja.png').stat().st_size > 1000
    assert (folder / 'DOF' / 'DOF3' / 'D3_g2_cost.png').exists() and (folder / 'DOF' / 'DOF3' / 'D3_g3_extrema.png').exists()
    ax = Figure().add_subplot(111)
    app.plot_progress(ax, hist, info)
    labels = [x.get_text() for x in ax.get_legend().get_texts()]
    assert 'sampler starts using the model (trial 3)' in labels and 'reference (FF=0) J = 0.140' in labels and 'FAIL' in labels
    assert all(ch.isascii() for s in labels + [ax.get_title(), ax.get_xlabel(), ax.get_ylabel()] for ch in s)



def test_saved_png_rules_20261106():
    """2026-11-06 の写真の決まり：POT の基準は黒の一点鎖線、cost と FF の極値の推移は凡例を x 軸の下、極値の推移の右軸ラベル、
    FF の凡例は「FF」、極値の値は軸ラベルと同じ大きさで線に重ならない。GUI（日本語）も同じ描き方"""
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    import optimize_optuna_ff_ROS2 as app
    t = np.arange(-120, 516) * c.TICK_DT
    ff_t = np.arange(0, 5001) / 1000.0
    co = c.quintic_from_extrema(3.0, 0.5, 2.0, 120.0, -60.0) * 1.0
    fv = c.ff_eval(co, 3.0, ff_t)
    ext = c.extrema_of(co, 3.0)
    p = {'T': 1.0, 'T_ff': 3.0, 't_eval': 5.0, 't': t, 'pot': 300 + 0 * t, 't_win': t[120:621], 'target': 300 + 0 * t[120:621],
         'ref': 300 + 0 * t, 'best': None, 'pwm': 0 * t, 'adrc': 0 * t, 'ff_t': ff_t, 'ff': fv, 'ext': ext, 'ref_Ji': [0.1]}
    hist = [{'n': k, 'state': 'COMPLETE', 'J': 1.0 / (k + 1), 'J_i': {'3': 0.5 / (k + 1)}, 'ext': {'3': list(ext)}} for k in range(5)]
    for L in ('en', 'ja'):
        fig = Figure(figsize=(7.0, 4.2), dpi=110); FigureCanvasAgg(fig)
        ax = fig.add_subplot(111)
        app.plot_g1(ax, L, 3, p, 1, final=True)
        ref = [ln for ln in ax.get_lines() if ln.get_label() == app.TXT[L]['ref']][0]
        assert ref.get_color() == 'k' and ref.get_linestyle() == '-.'           # 黒の一点鎖線
        for key in ('g2', 'g3'):
            fig = Figure(figsize=(7.0, 4.2), dpi=110); FigureCanvasAgg(fig)
            ax = fig.add_subplot(111)
            if key == 'g2':
                app.plot_g2(ax, L, 3, hist, 0.01, {'J_i': [0.1]})
            else:
                ax2 = ax.twinx()
                app.plot_g3(ax, ax2, L, 3, hist)
                assert ax2.get_ylabel() == app.TXT[L]['ext_y']
            fig.tight_layout(); app._fit_legends(fig)
            fig.canvas.draw()                                                   # 余白を変えた後の位置で測る
            rend = fig.canvas.get_renderer()
            leg = ax.get_legend().get_window_extent(rend)
            xl = ax.xaxis.label.get_window_extent(rend)
            assert leg.y1 <= xl.y0 + 1 and leg.y0 >= 0, (L, key)                  # 凡例は x 軸ラベルの下で、画像の中
        fig = Figure(figsize=(7.0, 4.2), dpi=110); FigureCanvasAgg(fig)
        ax = fig.add_subplot(111)
        app.plot_g5(ax, L, 3, p, 1, final=True)
        assert [x.get_text() for x in ax.get_legend().get_texts()] == ['FF']
        rend = fig.canvas.get_renderer()
        pts = ax.transData.transform(np.column_stack([ff_t, fv]))
        for tx in ax.texts:
            assert tx.get_fontsize() == ax.xaxis.label.get_fontsize()             # 軸ラベルと同じ大きさ
            bb = tx.get_window_extent(rend)
            assert not np.any((pts[:, 0] >= bb.x0) & (pts[:, 0] <= bb.x1) & (pts[:, 1] >= bb.y0) & (pts[:, 1] <= bb.y1))
            assert ax.get_window_extent(rend).contains(bb.x0, bb.y0) and ax.get_window_extent(rend).contains(bb.x1, bb.y1)
    assert app.TXT['en']['ext_y'] == 'extrema value (PWM) [-]' and app.TXT['en']['ff'] == 'FF'
    fig = Figure(); FigureCanvasAgg(fig)
    ax = fig.add_subplot(111)
    app.plot_progress(ax, hist, {'J_ref': [0.1], 'J_thr': 0.01, 'rms_thr': 10, 'n_startup': 2, 'T': 1.0, 'T_ff': 3.0}, 'ja')
    assert ax.get_xlabel() == '試行番号' and 'J の推移' in ax.get_title()


def test_share_pngs_ff0_and_best(tmp_path):
    """分析フォルダに、J に占める各自由度の割合（J_i / Σ J_i）の写真を FF=0 と best の2種類保存する。
    best は過去の試行を除いた COMPLETE の J 最小（study の best と同じ）"""
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib.figure import Figure
    import optimize_optuna_ff_ROS2 as app
    dofs = [3, 8]
    hist = [{'n': 0, 'state': 'COMPLETE', 'J': 0.5, 'J_i': {'3': 0.2, '8': 0.8}},
            {'n': 1, 'state': 'COMPLETE', 'J': 0.2, 'J_i': {'3': 0.3, '8': 0.1}},
            {'n': 2, 'state': 'COMPLETE', 'J': 0.01, 'J_i': {'3': 0.01, '8': 0.01}, 'source': 'past'},
            {'n': 3, 'state': 'FAIL', 'fail_class': 'unsafe'}]
    refs = {3: {'J_i': [0.1, 0.3]}, 8: {'J_i': [0.6, 1.0]}}
    ref_Ji, n_ref, best = app.share_inputs(dofs, hist, refs)
    assert ref_Ji == {3: 0.2, 8: 0.8} and n_ref == 2
    assert best[0] == 1 and best[1] == 0.2 and best[2] == {3: 0.3, 8: 0.1}       # 過去の試行（J 0.01）は除く
    ax = Figure().add_subplot(111)
    share = app.plot_share(ax, 'en', dofs, ref_Ji, 'x')
    assert np.allclose(share, [20.0, 80.0]) and ax.get_ylim() == (0.0, 108.0)
    assert [t.get_text() for t in ax.texts] == ['20.0', '80.0']
    saved = app.save_share_pngs(str(tmp_path), dofs, hist, refs)
    assert saved == ['J_share_FF0.png', 'J_share_best.png']
    assert all((tmp_path / f).stat().st_size > 1000 for f in saved)



def test_cost_png_best_ji_track():
    """cost の緑の線は、その時点の best J の試行の J_i（best が更新されたときだけ変わる。過去の試行は除く）"""
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib.figure import Figure
    import optimize_optuna_ff_ROS2 as app
    hist = [{'n': 0, 'state': 'COMPLETE', 'J': 1.0, 'J_i': {'3': 0.5}},
            {'n': 1, 'state': 'COMPLETE', 'J': 2.0, 'J_i': {'3': 0.01}},       # J が悪いので best ではない（J_i が小さくても）
            {'n': 2, 'state': 'FAIL', 'fail_class': 'unsafe'},
            {'n': 3, 'state': 'COMPLETE', 'J': 0.5, 'J_i': {'3': 0.9}},        # best が更新される（J_i は大きくなる）
            {'n': 4, 'state': 'COMPLETE', 'J': 0.1, 'J_i': {'3': 0.05}, 'source': 'past'},   # 過去の試行は best にしない
            {'n': 5, 'state': 'COMPLETE', 'J': 0.6, 'J_i': {'3': 0.2}}]
    ns, vals = app.best_ji_track(hist, 3)
    assert ns == [0, 1, 3, 5] and vals == [0.5, 0.5, 0.9, 0.9]
    for L in ('en', 'ja'):
        ax = Figure().add_subplot(111)
        app.plot_g2(ax, L, 3, hist, None, {'J_i': [0.1]})
        line = [ln for ln in ax.get_lines() if ln.get_label() == app.TXT[L]['best_Ji'].format(3)][0]
        assert list(line.get_xdata()) == ns and list(line.get_ydata()) == vals and line.get_drawstyle() == 'steps-post'
    assert app.TXT['en']['best_Ji'].format(3) == 'J_3 at best J'
