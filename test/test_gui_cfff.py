"""GUI_android_tk_cfFF.py のオフライン単体テスト（ROS・画面は使わない）

GUI_android_tk_cfFF.py は既存プログラムを import しない単体のプログラム。ここでは、移植した計算が移植元
（GUI_android_tk_FFsetting.py・optuna_ff_common.py）と同じ結果になるかを、テストの中だけ移植元を import して比べる。
実機の記録（~/ArmRobot/optimize_data/verify_optuna_20261104）がない環境では、その部分は飛ばす。

実行: cd ~/ArmRobot/src/py_pkg && python3 -m pytest -q test/test_gui_cfff.py
"""
import csv
import json
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_pkg'))
import GUI_android_tk_cfFF as g  # noqa: E402

OPTUNA = os.path.expanduser('~/ArmRobot/optimize_data/verify_optuna_20261104/optimize_20261104_1504')
need_data = pytest.mark.skipif(not os.path.exists(os.path.join(OPTUNA, 'trials', 'ref_00.npz')),
                               reason='実機の Optuna の記録がない')


# ------------------------------------------------------------------ 単体で動く
def test_no_import_of_existing_programs():
    """既存プログラムを import していない（単体で動く）"""
    src = open(g.__file__, encoding='utf-8').read()
    for name in ('GUI_android_tk_FFsetting', 'optuna_ff_common', 'optimize_optuna_ff_ROS2', 'adrc_param_common'):
        assert f'import {name}' not in src and f'from . import {name}' not in src
    assert 'import GUI_android_tk as' not in src


# ------------------------------------------------------------------ FF（GUI_android_tk_FFsetting.py と同じ）
def test_ff_math_matches_ffsetting():
    pytest.importorskip('rclpy')
    import GUI_android_tk_FFsetting as fs
    rng = np.random.default_rng(1)
    for _ in range(300):
        T = rng.uniform(0.3, 5.0)
        t1 = rng.uniform(0.02, 0.6) * T
        t2 = rng.uniform(t1 / T + 0.05, 0.98) * T
        y1 = rng.uniform(-80, 80)
        y2 = -np.sign(y1) * rng.uniform(1, 80)
        a = g.check_ff(T, t1, y1, t2, y2, -100, 100)
        b = fs.check_ff(T, t1, y1, t2, y2, -100, 100)
        assert a.ok == b.ok and a.errors == b.errors
        if a.coeffs is not None:
            assert np.array_equal(a.coeffs, b.coeffs)
    assert g.FF_DOF_MAP == fs.FF_DOF_MAP
    assert g.BOARD_LAYOUT == fs.BOARD_LAYOUT and g.FF_BOARD_DOFS == fs.FF_BOARD_DOFS


def test_ff_internal_restart_rule():
    """内部FFは、その自由度の目標値が変わったときだけ t=0 から掛け直す（.ino の POT_desired_change と同じ）"""
    ff6 = list(g.quintic_from_extrema(1.0, 0.22, 30.0, 0.78, -25.0)[0][0]) + [1.0]
    init = [300.0] * g.N_DOF
    cmd_t = [-0.5, 0.0, 0.4, 2.0]
    v0, v1 = [300.0] * g.N_DOF, [300.0] * g.N_DOF
    v1[3] = 400.0
    v2 = list(v1)
    v2[5] = 250.0                       # DOF 3 は同じ値の再送 → 掛け直さない
    v3 = list(v1)
    v3[3] = 300.0                       # 2.0 s に DOF 3 が戻る → 掛け直す
    cmd_v = [v0, v1, v2, v3]
    r = g.target_restarts(cmd_t, cmd_v, init, 3)
    assert r == [0.0, 2.0]
    t = np.array([-0.1, 0.0, 0.5, 0.9, 1.5, 2.3])
    f = g.ff_internal(ff6, r, t)
    assert np.isnan(f[0])
    assert f[1] == 0.0 and f[4] == 0.0
    assert f[2] == pytest.approx(float(g.ff_poly(ff6[:5], 0.5)))
    assert f[5] == pytest.approx(float(g.ff_poly(ff6[:5], 0.3)))
    assert g.target_restarts(cmd_t, cmd_v, init, 0) == []


def test_ff_internal_ticks():
    """内部FF（ADRC 用）の時刻は配信番号で数える: (j − j_r)×0.010 − 0.010。j_r はボードの目標値が変わった配信"""
    ff6 = list(g.quintic_from_extrema(1.0, 0.22, 30.0, 0.78, -25.0)[0][0]) + [1.0]
    # 配信 0..399、PC の時計は 4% 遅い（受信間隔 9.6 ms）。ボードの目標値は配信 120 で 300→400、配信 300 で 400→300
    j_tk = np.arange(400)
    t_tk = (j_tk - 120) * 0.0096 + 0.012
    echo = np.where(j_tk < 120, 300.0, np.where(j_tk < 300, 400.0, 300.0))
    R = g.echo_restart_ticks(t_tk, j_tk, echo)
    assert list(R) == [120, 300]
    j_pw = np.array([100, 120, 121, 150, 219, 222, 310])
    t_pw = (j_pw - 120) * 0.0096 + 0.013
    f = g.ff_internal_ticks(ff6, R, j_pw, t_pw)
    assert np.isnan(f[0])                                  # 記録開始より前は分からない
    assert f[1] == 0.0 and f[2] == 0.0                     # 時刻 −0.010, 0.000
    assert f[3] == pytest.approx(float(g.ff_poly(ff6[:5], 0.29)))
    assert f[4] == pytest.approx(float(g.ff_poly(ff6[:5], 0.98)))
    assert f[5] == 0.0                                     # T 以後は 0
    assert f[6] == pytest.approx(float(g.ff_poly(ff6[:5], 0.09)))     # 配信 300 で掛け直し


@need_data
def test_internal_ff_matches_optuna_best_trial_csv():
    """実機の試行902 から作った内部FFが、Optuna が保存した best_trial.csv の FF{d} 列と一致する（同じ時刻の数え方）"""
    c = pytest.importorskip('optuna_ff_common')
    rec = c.load_trial_npz(os.path.join(OPTUNA, 'trials', 'trial_902.npz'))
    series = {b: {k: (np.asarray(rec.raw[b][k][0]) - rec.t_target, np.asarray(rec.raw[b][k][1], dtype=float))
                  for k in ('pot', 'pwm')} for b, _o, _c in g.BOARD_LAYOUT}
    table = np.zeros((g.N_FF, 6))
    table[:, :5] = rec.ff
    table[:, 5] = rec.T_ff
    r = g.Record({'init_target': list(rec.initial), 'target': list(rec.target), 'ff_table': table.tolist(),
                  'warnings': []}, series, [0.0], [list(rec.target)])
    g.compute_internal_ff(r)
    with open(os.path.join(OPTUNA, 'optimize_20261104_1504_best_trial.csv'), encoding='utf-8-sig') as f:
        rd = csv.reader(f)
        head = next(rd)
        rows = list(rd)
    idx = {h: i for i, h in enumerate(head)}
    for k, (b, off, cnt) in enumerate(g.BOARD_LAYOUT, start=1):
        sel_rows = [x for x in rows if x[idx[f'Time{k}_PWM']] != '']
        t, F = r.series[b]['ff']
        assert len(sel_rows) == len(t)
        for j in range(cnt):
            d = off + j
            if d in g.FF_INDEX:
                fc = np.array([float(x[idx[f'FF{d}']]) for x in sel_rows])
                assert np.max(np.abs(F[t >= 0, j] - fc[t >= 0])) < 1e-4


# ------------------------------------------------------------------ 記録CSV・FFの判定
def _synthetic_record(ff_on=True, recv=True):
    rng = np.random.default_rng(0)
    series = {}
    for board, _o, cnt in g.BOARD_LAYOUT:
        t = np.arange(-1.2, 6.0, 0.01) + rng.uniform(0, 0.002, 720)
        V = np.zeros((len(t), 2 * cnt))
        V[:, :cnt] = 300 + rng.integers(0, 3, (len(t), cnt))
        V[:, cnt:] = 300
        s = {'pot': (t, V), 'pwm': (t + 0.001, rng.normal(0, 5, (len(t), cnt))),
             'z3': (t + 0.002, rng.normal(0, 1, (len(t), cnt)))}
        F = np.zeros((len(t), cnt))
        if ff_on and board == 'board1':
            F[(t > 0) & (t < 1), 3] = 10.0
        s['ffrecv'] = (t + 0.003, F) if recv else (np.zeros(0), np.zeros((0, cnt)))
        series[board] = s
    table = g.zero_ff_table()
    setting = [None] * g.N_FF
    if ff_on:
        table[3] = list(g.quintic_from_extrema(1.0, 0.22, 30.0, 0.78, -25.0)[0][0]) + [1.0]
        setting[3] = {'T': 1.0, 't1': 0.22, 'y1': 30.0, 't2': 0.78, 'y2': -25.0}
    init = [300.0] * g.N_DOF
    target = list(init)
    target[3] = 400.0
    meta = {'name': 'x', 'start': '2026-11-10T12:00:00', 'stamp': '20261110_120000', 'duration': 6.0,
            'history_s': 1.2, 'init_target': init, 'target': target, 'ff_table': table.tolist(), 'ff_setting': setting,
            'ff_source': ['手動'] * g.N_FF, 'ff_status': {'1': ['sent', '12:00:00']},
            'adrc': {'3': [9000.0, 500.0, 2e5, 700.0]}, 'warnings': ['テストの警告']}
    rec = g.Record(meta, series, [-0.5, 0.0], [init, target])
    g.compute_internal_ff(rec)
    return rec


def _with_pre_ff():
    rec = _synthetic_record(ff_on=False)
    rec.series['board2']['ffrecv'][1][100, 1] = 5.0         # t ≈ −0.2 s（目標値が変わる前の履歴）
    return rec


def test_ff_label_by_received_ff():
    assert g.judge_ff_label(_synthetic_record(ff_on=True)) == 'w'
    assert g.judge_ff_label(_synthetic_record(ff_on=False)) == 'wo'
    rec = _synthetic_record(ff_on=True, recv=False)          # 受信できなければ送信した設定で判定し、警告する
    assert g.judge_ff_label(rec) == 'w'
    assert any('受信できませんでした' in w for w in rec.meta['warnings'])
    rec = _synthetic_record(ff_on=False)
    rec.series['board2']['ffrecv'][1][300, 1] = 5.0         # 記録中に送っていないFFを受信 → w、他プログラムの警告
    assert g.judge_ff_label(_with_pre_ff()) == 'wo'           # 記録前（負の時刻）のFFは判定に使わない
    assert g.judge_ff_label(rec) == 'w'
    assert any('他のプログラム' in w for w in rec.meta['warnings'])


def test_record_csv_roundtrip(tmp_path):
    rec = _synthetic_record()
    g.judge_ff_label(rec)
    rec.meta['name'] = g.record_folder_name(rec)
    assert rec.meta['name'] == 'FF(w_20261110_120000)'
    path = tmp_path / (rec.meta['name'] + '.csv')
    g.write_record_csv(rec, str(path))
    r2 = g.read_record_csv(str(path))
    assert r2.meta['ff_label'] == 'w' and r2.legend() == 'w/'
    assert r2.meta['target'][3] == 400.0 and r2.meta['init_target'][3] == 300.0
    assert np.allclose(r2.meta['ff_table'][3], rec.meta['ff_table'][3], rtol=1e-8)
    assert r2.meta['ff_setting'][3]['t1'] == 0.22 and r2.meta['ff_setting'][0] is None
    assert r2.meta['adrc']['3'] == [9000.0, 500.0, 2e5, 700.0]
    assert r2.meta['warnings'] == rec.meta['warnings']
    for board, _o, _c in g.BOARD_LAYOUT:
        for kind in ('pot', 'pwm', 'z3', 'ffrecv', 'ff'):
            t1, V1 = rec.series[board][kind]
            t2, V2 = r2.series[board][kind]
            assert np.allclose(t1, t2, atol=1e-6)
            assert np.allclose(V1, V2, rtol=1e-6, atol=1e-6, equal_nan=True)
    t, a = r2.dof_series(3, 'adrc')
    tp, p = r2.dof_series(3, 'pwm')
    tf, f = r2.dof_series(3, 'ff')
    assert np.allclose(a, p - f, equal_nan=True)
    with open(path, encoding='utf-8-sig') as fh:
        head = next(csv.reader(fh))
    for col in ('Time1', 'POT3', 'POTdesired3', 'Time1_PWM', 'PWM3', 'FF3', 'ADRC3', 'Time1_z3', 'z3_3',
                'Time1_FFrecv', 'FFrecv_3', 'Time_cmd', 'cmd25', 'cond_DOF', 'meta_key'):
        assert col in head


def test_read_rejects_other_csv(tmp_path):
    p = tmp_path / 'other.csv'
    p.write_text('Time1,POT0\n0.0,300\n', encoding='utf-8')
    with pytest.raises(ValueError):
        g.read_record_csv(str(p))


def test_merge_warnings():
    out = g.merge_dof_warnings(['DOF 0: A', 'DOF 3: A', 'DOF 3: B', 'データ1: C', 'DOF 5: A'])
    assert out == ['DOF 0, 3, 5: A', 'DOF 3: B', 'データ1: C']


# ------------------------------------------------------------------ 評価（optuna_ff_common.py と同じ）
def test_assign_ticks_matches_optuna():
    c = pytest.importorskip('optuna_ff_common')
    rng = np.random.default_rng(3)
    for _ in range(20):
        d = np.full(800, 0.0099) + rng.exponential(0.0006, 800)
        d[rng.integers(0, 800, 5)] += 0.010                  # 欠損
        d[rng.integers(0, 800, 3)] += 0.02                   # 遅れ
        t = np.cumsum(d)
        a = g.assign_ticks(t)
        b = c.assign_ticks(t)
        assert np.array_equal(a[0], b[0]) and a[2] == b[2]


@need_data
def test_target_and_J_match_optuna_target_json():
    """基準試行2回（FF=0）から作った目標軌道と J_ref が、Optuna の target.json と一致する（DOF 0, 8）"""
    c = pytest.importorskip('optuna_ff_common')
    cfg = json.load(open(os.path.join(OPTUNA, 'config.json')))
    tj = json.load(open(os.path.join(OPTUNA, 'target.json')))
    refs = [c.load_trial_npz(os.path.join(OPTUNA, 'trials', f'ref_0{i}.npz')) for i in range(2)]
    dofs = [0, 8]
    ser = []
    for rec in refs:
        raw = {b: {'pot': (k['pot'][0] - rec.t_target, k['pot'][1])} for b, k in rec.raw.items()}
        s, _info, err = g.extract_series(raw, cfg['initial'], cfg['target'], dofs, 0.0, cfg['T'],
                                         eval_min=cfg['eval_min'])
        assert not err
        ser.append(s)
    for d in dofs:
        ys = [s[d].pot[s[d].win()] - s[d].Pf for s in ser]
        sp = g.fit_target_constrained(d, ys, [np.isfinite(y) for y in ys], [s[d].P0 for s in ser],
                                      cfg['initial'][d], cfg['target'][d], cfg['T'], cfg['eps'])
        ref = tj['targets'][str(d)]
        assert sp.T1 == pytest.approx(ref['T1'], rel=1e-9) and sp.wn == pytest.approx(ref['wn'], rel=1e-9)
        Jr = [g.normalized_error(s[d].pot[s[d].win()], sp.seq(), s[d].dP, np.isfinite(s[d].pot[s[d].win()]))
              for s in ser]
        assert Jr == pytest.approx(ref['J_ref'], rel=1e-9)


def _npz_to_record(c, path, label, name, keep):
    rec = c.load_trial_npz(path)
    series = {}
    for b, _o, cnt in g.BOARD_LAYOUT:
        s = {k: (np.asarray(rec.raw[b][k][0]) - rec.t_target, np.asarray(rec.raw[b][k][1], dtype=float))
             for k in ('pot', 'pwm', 'z3')}
        s['ffrecv'] = (np.zeros(0), np.zeros((0, cnt)))
        series[b] = s
    table = np.zeros((g.N_FF, 6))
    table[:, :5] = rec.ff
    table[:, 5] = rec.T_ff
    init = [float(v) for v in rec.initial]
    target = [float(v) if d in keep else float(rec.initial[d]) for d, v in enumerate(rec.target)]
    meta = {'name': name, 'start': '', 'stamp': '', 'duration': float(series['board1']['pot'][0][-1]), 'history_s': 1.2,
            'init_target': init, 'target': target, 'ff_table': table.tolist(), 'ff_setting': [None] * g.N_FF,
            'ff_source': [''] * g.N_FF, 'ff_status': {}, 'adrc': {}, 'warnings': [], 'ff_label': label,
            'ff_judge': ''}
    r = g.Record(meta, series, [0.0], [target])
    g.compute_internal_ff(r)
    return r


@need_data
def test_compare_through_csv_matches_optuna_trials(tmp_path):
    """実機の記録を記録CSVに書いて読み直し、比較の J_i が Optuna の trials.csv（試行902）・J_ref と一致する（DOF 0, 8）"""
    c = pytest.importorskip('optuna_ff_common')
    cfg = json.load(open(os.path.join(OPTUNA, 'config.json')))
    keep = {0, 8}
    recs = {}
    for key, fn, label in (('ref0', 'ref_00', 'wo'), ('ref1', 'ref_01', 'wo'), ('t902', 'trial_902', 'w')):
        r = _npz_to_record(c, os.path.join(OPTUNA, 'trials', fn + '.npz'), label, key, keep)
        p = str(tmp_path / (key + '.csv'))
        g.write_record_csv(r, p)
        recs[key] = g.read_record_csv(p)
    res = g.compare_records(recs['ref0'], recs['t902'], [recs['ref0'], recs['ref1']], cfg['T'], cfg['eps'],
                            cfg['eval_min'])
    with open(os.path.join(OPTUNA, 'optimize_20261104_1504_trials.csv'), encoding='utf-8-sig') as f:
        row = next(r for r in csv.DictReader(f) if r['trial'] == '902')
    tj = json.load(open(os.path.join(OPTUNA, 'target.json')))
    assert [r['DOF'] for r in res['rows']] == [0, 8]
    for r in res['rows']:
        d = r['DOF']
        assert r['J_i(1)'] == pytest.approx(tj['targets'][str(d)]['J_ref'][0], rel=1e-9)
        assert r['J_i(2)'] == pytest.approx(float(row[f'd{d}_J_i']), rel=1e-7)
        assert r['評価値が小さい方'] in ('1', '2')
    # 変化率（1つ目に対する2つ目）と J に占める割合（合計 100%）
    for r in res['rows']:
        assert r['変化率[%]'] == pytest.approx((r['J_i(2)'] - r['J_i(1)']) / r['J_i(1)'] * 100)
        assert r['改善/悪化'] == ('改善' if r['J_i(2)'] < r['J_i(1)'] else '悪化')
    for i in (1, 2):
        assert sum(r[f'Jに占める割合({i})[%]'] for r in res['rows']) == pytest.approx(100.0)
    ov = res['overall']
    assert ov['pct'] == pytest.approx((ov['J2'] - ov['J1']) / ov['J1'] * 100)
    # 比較する自由度を選ぶと、選ばなかった自由度は評価しない
    res0 = g.compare_records(recs['ref0'], recs['t902'], [recs['ref0'], recs['ref1']], cfg['T'], cfg['eps'],
                             cfg['eval_min'], selected=[0, 5])
    assert [r['DOF'] for r in res0['rows']] == [0] and res0['overall']['dofs'] == [0]
    assert res0['rows'][0]['Jに占める割合(1)[%]'] == pytest.approx(100.0)
    assert res0['overall']['J2'] == pytest.approx(res['rows'][0]['J_i(2)'])
    assert any('DOF 8' in w and '選ばなかった' in w for w in res0['warnings'])
    # ΔP で割らない評価値: J_i = mean((POT − Target)²) = (割る J_i) × ΔP²。目標軌道は同じ
    resn = g.compare_records(recs['ref0'], recs['t902'], [recs['ref0'], recs['ref1']], cfg['T'], cfg['eps'],
                             cfg['eval_min'], normalize=False)
    for a, b in zip(res['rows'], resn['rows']):
        assert b['目標軌道T1[s]'] == a['目標軌道T1[s]']
        for k in ('J_i(1)', 'J_i(2)'):
            assert b[k] == pytest.approx(a[k] * a['ΔP'] ** 2, rel=1e-9)
        assert b['RMS%(2)'] == pytest.approx(a['RMS%(2)'])
    assert resn['overall']['J2'] == pytest.approx(np.mean([r['J_i(2)'] for r in resn['rows']]))
    assert not resn['normalize'] and 'ΔPで割らない' in resn['rows'][0]['評価値の定義']
    out = tmp_path / g.ANALYSIS_CSV
    g.write_analysis_csv(str(out), res)
    g.save_share_chart(str(tmp_path / g.SHARE_PNG), res)
    assert (tmp_path / g.SHARE_PNG).stat().st_size > 10000
    with open(out, encoding='utf-8-sig') as f:
        rows = list(csv.reader(f))
    assert rows[0] == list(g.ANALYSIS_COLUMNS)
    # 全体の評価値 J の表（自由度の表の後）。J は Optuna の J と同じ定義 mean(J_i)
    i = next(k for k, r in enumerate(rows) if r and r[0].startswith('全体の評価値 J'))
    assert rows[i + 1] == list(g.J_COLUMNS)
    jrow = dict(zip(rows[i + 1], rows[i + 2]))
    J2 = np.mean([float(row[f'd{d}_J_i']) for d in sorted(keep)])
    assert float(jrow['J(2)']) == pytest.approx(J2, rel=1e-7)
    assert float(jrow['J(1)']) == pytest.approx(np.mean([tj['targets'][str(d)]['J_ref'][0] for d in sorted(keep)]),
                                                rel=1e-9)
    assert jrow['Jが小さい方'] == ('1' if float(jrow['J(1)']) < float(jrow['J(2)']) else '2')
    assert jrow['平均した自由度'] == '0 8' and jrow['平均した自由度数'] == '2'

    # 比較グラフの時間軸（配信番号）と 目標軌道.csv: 評価と同じ点どうしが並ぶ
    tr1 = g.tick_record(recs['ref0'], res['j0'][0])
    tr2 = g.tick_record(recs['t902'], res['j0'][1])
    xmax = max(g.tick_end(tr1), g.tick_end(tr2))
    path = tmp_path / g.TARGET_CSV
    g.write_target_csv(str(path), res, tr1, tr2, xmax)
    with open(path, encoding='utf-8-sig') as f:
        trows = list(csv.DictReader(f))
    assert len(trows) == int(np.floor(xmax / g.TICK_DT + 1e-9)) + 1 and xmax > res['t_eval']
    win = [r for r in trows if r['評価区間'] == '1']
    assert len(win) == res['N_e'] + 1
    for r in res['rows']:
        d = r['DOF']
        dP = r['ΔP']
        tgt = np.array([float(x[f'Target{d}']) for x in win])
        tp = res['targets'][d]
        seq = g.target_states(tp['T1'], tp['wn'], tp['y0_ref'], g.TICK_DT, res['N_e'])[:, 0] + tp['Pf']
        assert np.allclose(tgt, seq, rtol=1e-8)
        for i, key in ((1, 'J_i(1)'), (2, 'J_i(2)')):
            pot = np.array([float(x[f'POT{d}({i})']) if x[f'POT{d}({i})'] else np.nan for x in win])
            assert g.normalized_error(pot, tgt, dP, np.isfinite(pot)) == pytest.approx(r[key], rel=1e-7)
        # 目標軌道は評価区間の後も同じ3次遅れ系の続き（グラフの右端まで）
        t_full, y_full = g.target_curve(tp, len(trows) - 1)
        assert np.allclose(y_full[:res['N_e'] + 1], seq) and len(y_full) == len(trows)


# ------------------------------------------------------------------ Optuna の結果の読み込み
@need_data
def test_optuna_loader_matches_results_csv():
    res = g.OptunaResult(OPTUNA)
    best = res.best_overall()
    assert best['n'] == 902
    with open(os.path.join(OPTUNA, 'optimize_20261104_1504_results.csv'), encoding='utf-8-sig') as f:
        rr = list(csv.DictReader(f))[-1]
    for d in res.dofs:
        n = g.FF_INDEX[d] + 1
        T, t1, y1, t2, y2, coeffs = res.extrema(best, d)
        assert (T, t1, y1, t2, y2) == pytest.approx(tuple(float(rr[f'{k}_{n}']) for k in ('T', 't1', 'y1', 't2', 'y2')),
                                                   abs=1e-6)
        assert g.check_ff(T, t1, y1, t2, y2, -255, 255).ok
    assert 13 not in res.dofs and res.extrema(best, 13) is None
    for d in (0, 8, 24):
        b = res.best_for_dof(d)
        assert b['dof'][d]['J_i'] == min(t['dof'][d]['J_i'] for t in res.candidates())
    assert res.trial(476)['n'] == 476 and res.trial(99999) is None


def test_optuna_loader_rejects_other_folder(tmp_path):
    (tmp_path / 'x_trials.csv').write_text('loop,J\n0,1\n', encoding='utf-8')
    with pytest.raises(ValueError):
        g.OptunaResult(str(tmp_path))


def test_exact_coefficients_used_until_edited():
    m = g.DofFF(0)
    co = list(g.quintic_from_extrema(3.0, 1.0, 50.0, 2.5, -20.0)[0][0] * 1.0001)
    m.update(enabled=True, T=3.0, ylo=-255, yhi=255, t1=1.0, y1=50.0, t2=2.5, y2=-20.0,
             exact={'T': 3.0, 't1': 1.0, 'y1': 50.0, 't2': 2.5, 'y2': -20.0, 'coeffs': co})
    assert m.coeffs6()[:5] == pytest.approx(co, rel=0, abs=0)
    m.update(y1=49.0)
    assert not m.exact_valid()
    assert m.coeffs6()[:5] == pytest.approx(list(g.quintic_from_extrema(3.0, 1.0, 49.0, 2.5, -20.0)[0][0]))
