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
