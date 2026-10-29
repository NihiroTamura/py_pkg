#!/usr/bin/env python3
"""
optimize_EL_ff_ROS2_2.py の評価関数の重みを自動調整するプログラム（GUI）

<目的>
  最適制御で計算される FF u(t)（5次関数）の振幅 max|FF(t)|（極値ちょうど2個なので max(|y1|,|y2|)）が、
  GUI で指定した範囲 [A_min, A_max] に入る重みを探す。重みは全自由度で共通の1組（本体の「評価関数の重み」に入れて使う）。
<ロボットは1回だけ動かす>
  1. ADRC のみ（FF=0）で目標値を送り、max(1.5T, 5 s) 測定する（optimize_EL_ff_ROS2_2.py の Step 1 と同じ手順・安全手順）
  2. 実測に目標軌道（ζ=1）とシステムモデルを同定し、そのシステムモデルを「真のモデル」とみなす
  3. 以降はロボットを動かさず、内部ADRCシミュレーションの上で重みを探す
  保存した試行（本体の trials/trial_NN.npz も可）を読み込めば、ロボットを動かさずに探し直せる。
<探索（相談 D の D3。全自由度で共通の重み）>
  J は (Q, Sf, R) を同じ倍率にしても同じ解を与えるので、Q の位置成分は初期値のまま固定する。
  条件: すべての対象自由度で振幅が [A_min, A_max] に入る（と内部閉ループの増幅率がゲート以下）。
  外側: q_v = Q_速度/Q_位置, q_a = Q_加速度/Q_位置（と Sf の倍率）を対数で Nelder-Mead で探す。
  内側: 自由度ごとに、R を対数の二分法で動かして振幅が範囲に入る R の区間を求め（2次計画では ‖u*‖ が R について
        単調減少することを利用）、その共通部分の中の数点で目的が最小の R を選ぶ。共通部分が空なら実行不可能。
  目的関数: 予測 J_i の自由度平均（本体の J と同じ C2 の定義。そのFFを与えたモデルの応答に ζ=1 の目標軌道を
            当てはめ直して計算）。範囲に入る R が無ければ +∞。
  最適制御の計算（EL・5次関数・評価関数・ADRC内部シミュレーション・周回間隔）は本体と同じ関数（optimize_EL2_common.py）を使う。
  範囲に入る重みが1つも見つからない・探索が収束しない場合は、その旨を表示し、成功扱いにしない。
<保存>  <保存先>/tuning_YYYYMMDD_HHMM/
    trial_00.npz（実測）, …_conditions.csv, …_models.csv（同定したモデル）, …_search.csv（探索した全ての重み）,
    …_result.csv（決定した共通の重みと、自由度ごとの振幅・予測 J_i・状態）,
    weights.json（本体の「重みを読み込む」用。共通の重み）, run.log
"""
import math
import os
import threading
import time
import traceback

import numpy as np
import scipy.optimize

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

if __package__:
    from . import optimize_EL2_common as c
    from . import optimize_EL2_gui as g
else:
    import optimize_EL2_common as c
    import optimize_EL2_gui as g

SETTINGS_FILE = os.path.expanduser('~/.armrobot_el2_tuning_settings.json')


# ==============================================================================
# 全自由度で共通の重みの探索
# ==============================================================================
class DofEval:
    """1自由度ぶんの「重み → 最適制御のFF → 振幅・予測 J_i」の計算（近い R の解を次の初期値に使う）"""

    def __init__(self, da, cfg, stop):
        self.da = da
        self.cfg = cfg
        self.stop = stop
        self.warm = {}

    def solve(self, w, quick=True):
        if self.stop.is_set():
            raise c.StopRequested()
        warm = None
        if self.warm:
            Rk = min(self.warm, key=lambda r: abs(math.log(r) - math.log(w.R)))
            warm = self.warm[Rk]
        res, info = c.optimize_ff(self.da, w, self.cfg['T'], self.cfg['L'], self.cfg['eps'], self.cfg['el_max_iter'],
                                  self.stop, lambda m: None, warm=warm, grid_n=10 if quick else self.cfg['grid_n'],
                                  n_starts=1 if quick else self.cfg['n_starts'])
        t1, y1, t2, y2 = res.extrema
        amp = max(abs(y1), abs(y2)) if res.status != 'zero' else 0.0
        if res.status != 'zero':
            self.warm[w.R] = np.array([t1, t2, y1, y2])
        Jp = c.predicted_J(self.da, res.coeffs, self.cfg['T'], self.cfg['L'], self.cfg['bounds'])[0]
        return amp, Jp, res, info


class CommonTuner:
    """全自由度で共通の重み (Q, Sf, R) を1組探す。

    条件: すべての対象自由度で、振幅 max|FF| が [A_min, A_max] に入り、内部閉ループの増幅率がゲート以下
    目的: 予測 J_i の自由度平均（本体の J と同じ定義）
    外側: q_v = Q_速度/Q_位置, q_a = Q_加速度/Q_位置（と Sf の倍率）を Nelder-Mead で探す（Q_位置は初期値で固定）
    内側: 自由度ごとに「振幅が範囲に入る R の区間」を二分法で求め、その共通部分の中の数点で目的が最小の R を選ぶ。
          R は共通なので、共通部分が空なら（ある自由度の上限と別の自由度の下限が両立しない）その形は実行不可能。
    """

    def __init__(self, analysis, w0, cfg, stop, log, report):
        self.dofs = sorted(analysis)
        self.ev = {d: DofEval(analysis[d], cfg, stop) for d in self.dofs}
        self.w0 = w0
        self.cfg = cfg
        self.stop = stop
        self.log = log
        self.report = report                                     # report(kind, dict) … GUI の表を更新する
        self.A_min, self.A_max = cfg['A_range']
        self.qp = w0.Q[0] if w0.Q[0] > 0 else 1.0
        self.sf_on = any(v > 0 for v in w0.Sf)
        self.evals = []                                          # すべての評価（保存用。1行 = 1つの重み × 1自由度）
        self.n_eval = 0
        self.best = None

    def weights(self, x, R):
        Q = [self.qp, self.qp * math.exp(x[0]), self.qp * math.exp(x[1])]
        Sf = [v * (math.exp(x[2]) if self.sf_on else 1.0) for v in self.w0.Sf]
        return c.Weights(Q=Q, Sf=Sf, R=float(R))

    def x0(self):
        q = self.w0.Q
        r1 = q[1] / self.qp if q[1] > 0 else 1e-2
        r2 = q[2] / self.qp if q[2] > 0 else 1e-4
        return np.array([math.log(r1), math.log(r2)] + ([0.0] if self.sf_on else []))

    def _row(self, w, d, amp, Jp, res, info, quick, stage):
        row = {'eval': self.n_eval, 'stage': stage, 'DOF': d, 'q_v': w.Q[1] / self.qp, 'q_a': w.Q[2] / self.qp,
               'Q': w.Q, 'Sf': w.Sf, 'R': w.R, 'amplitude': amp, 'J_pred': Jp, 'J_EL': res.J, 'EL_status': res.status,
               'in_range': self.A_min <= amp <= self.A_max, 'gate_ok': info['gate_ok'], 'quick': quick,
               'extrema': res.extrema, 'coeffs': res.coeffs}
        self.evals.append(row)
        return row

    def amp(self, d, x, R, stage):
        w = self.weights(x, R)
        amp, Jp, res, info = self.ev[d].solve(w)
        self.n_eval += 1
        row = self._row(w, d, amp, Jp, res, info, True, stage)
        self.report('current', row)
        return amp

    def evaluate_all(self, w, quick=True, stage='candidate'):
        """共通の重み w を全自由度で計算する。戻り値 (全自由度が条件を満たすか, 平均 J_i, {DOF: (振幅, J_i, res, info)})"""
        per = {}
        for d in self.dofs:
            amp, Jp, res, info = self.ev[d].solve(w, quick)
            self.n_eval += 1
            self.report('current', self._row(w, d, amp, Jp, res, info, quick, stage))
            per[d] = (amp, Jp, res, info)
        ok = all(self.A_min <= p[0] <= self.A_max and p[3]['gate_ok'] for p in per.values())
        Jm = float(np.mean([p[1] for p in per.values()]))
        return ok, Jm, per

    def _bisect(self, f, R_small, R_large, target, n=12):
        """f(R_small) > target ≥ f(R_large) の間で f(R) = target となる R を挟み込む（両端を返す）"""
        for _ in range(n):
            if R_large / R_small < 1.05:
                break
            mid = math.sqrt(R_small * R_large)
            if f(mid) > target:
                R_small = mid
            else:
                R_large = mid
        return R_small, R_large

    def r_lower(self, d, x, R0):
        """自由度 d で振幅が A_max 以下になる最小の R（振幅は R について単調減少）"""
        span = self.cfg['R_span']
        lo_lim, hi_lim = R0 / span, R0 * span
        f = lambda R: self.amp(d, x, R, 'R_search')
        if f(R0) > self.A_max:
            R = R0
            while R < hi_lim and f(R * 10) > self.A_max:
                R *= 10
            return self._bisect(f, R, min(R * 10, hi_lim), self.A_max)[1]
        R = R0
        while R > lo_lim and f(R / 10) <= self.A_max:
            R /= 10
        return R if R <= lo_lim else self._bisect(f, R / 10, R, self.A_max)[1]

    def r_upper(self, d, x, R_from):
        """自由度 d で振幅が A_min 以上を保つ最大の R（R_from では A_min 以上であること）"""
        hi_lim = R_from * self.cfg['R_span']
        f = lambda R: self.amp(d, x, R, 'R_search')
        R = R_from
        while R < hi_lim and f(R * 10) >= self.A_min:
            R *= 10
        if R >= hi_lim:
            return hi_lim
        return self._bisect(f, R, R * 10, self.A_min - 1e-12)[0]

    def r_interval(self, x, R0):
        """全自由度で振幅が範囲に入る共通の R の区間（無ければ (None, 理由)）"""
        lows = {d: self.r_lower(d, x, R0) for d in self.dofs}
        R_lo = max(lows.values())
        short = {}
        for d in self.dofs:
            a = self.amp(d, x, R_lo, 'R_search')
            if a < self.A_min:
                short[d] = a
        if short:
            hold = max(lows, key=lows.get)
            why = (f"R ≥ {R_lo:.3g}（DOF {hold} の振幅を {self.A_max:g} 以下にする R）では "
                   + ", ".join(f"DOF {d} の振幅 {a:.2f}" for d, a in short.items()) + f" が {self.A_min:g} に届きません")
            return None, why
        R_hi = min(self.r_upper(d, x, R_lo) for d in self.dofs)
        return (R_lo, max(R_lo, R_hi)), ''

    def inner(self, x):
        R0 = self.best['w'].R if self.best else self.w0.R
        iv, why = self.r_interval(x, R0)
        if iv is None:
            self.log(f"   q_v={math.exp(x[0]):.3g} q_a={math.exp(x[1]):.3g}: 全自由度を範囲に入れる共通の R がありません（{why}）")
            return math.inf, None
        best = None
        for R in np.exp(np.linspace(math.log(iv[0]), math.log(iv[1]), 4)):
            w = self.weights(x, R)
            ok, Jm, per = self.evaluate_all(w)
            if ok and (best is None or Jm < best[0]):
                best = (Jm, w, per)
        if best is None:
            return math.inf, None
        return best[0], best

    def run(self):
        self.log(f"=== 全自由度で共通の重みの探索（対象 DOF {self.dofs}、振幅 {self.A_min:g}〜{self.A_max:g} PWM）")
        status = 'no_solution'

        def f(x):
            J, b = self.inner(x)
            if b is not None and (self.best is None or J < self.best['J']):
                self.best = {'J': J, 'w': b[1], 'per': b[2], 'x': np.array(x)}
                self.report('best', self._summary(b[1], J, b[2]))
                self.log(f"   最良を更新: 平均 J_i(予測)={J:.4g}, Q={['%.4g' % v for v in b[1].Q]}, "
                         f"Sf={['%.4g' % v for v in b[1].Sf]}, R={b[1].R:.4g}, 振幅 "
                         + ", ".join(f"DOF{d} {p[0]:.2f}" for d, p in b[2].items()))
            return J

        converged = False
        try:
            res = scipy.optimize.minimize(f, self.x0(), method='Nelder-Mead',
                                          options={'maxfev': self.cfg['outer_max'], 'xatol': 0.05, 'fatol': 1e-12,
                                                   'initial_simplex': self._simplex()})
            converged = bool(res.success)
        except c.StopRequested:
            status = 'aborted'
        if self.best is not None and status != 'aborted':
            # 本体と同じ設定（格子・開始点の数）で全自由度を確かめ直す
            try:
                ok, Jm, per = self.evaluate_all(self.best['w'], quick=False, stage='verify')
            except c.StopRequested:
                ok, status = False, 'aborted'
            if status != 'aborted':
                if ok:
                    self.best.update({'J': Jm, 'per': per})
                    status = 'success' if converged else 'success_not_converged'
                else:
                    status = 'verify_failed'
                    self.log("   本体と同じ設定で計算し直すと範囲外の自由度がありました: "
                             + ", ".join(f"DOF{d} {p[0]:.2f}" for d, p in per.items()))
        out = {'status': status, 'evaluations': self.n_eval, 'dofs': self.dofs}
        if self.best is not None:
            out.update(self._summary(self.best['w'], self.best['J'], self.best['per']))
        out['J_pred_FF0'] = {d: c.predicted_J(self.ev[d].da, np.zeros(5), self.cfg['T'], self.cfg['L'],
                                              self.cfg['bounds'])[0] for d in self.dofs}
        msg = {'success': '成功', 'success_not_converged': '範囲内の重みは見つかったが外側の探索は収束していない',
               'no_solution': '全自由度を範囲に入れる共通の重みが見つかりません（成功扱いにしません）', 'aborted': '停止',
               'verify_failed': '確認計算で範囲外（成功扱いにしません）'}[status]
        self.log(f"=== {msg}（EL の計算 {self.n_eval} 回、FF=0 の予測 J_i: "
                 + ", ".join(f"DOF{d} {v:.4g}" for d, v in out['J_pred_FF0'].items()) + "）")
        out['status_text'] = msg
        self.report('final', out)
        return out

    def _summary(self, w, J, per):
        return {'Q': w.Q, 'Sf': w.Sf, 'R': w.R, 'q_v': w.Q[1] / self.qp, 'q_a': w.Q[2] / self.qp, 'J_pred': J,
                'per': {d: {'amplitude': p[0], 'J_pred': p[1], 'extrema': p[2].extrema, 'coeffs': p[2].coeffs}
                        for d, p in per.items()}}

    def _simplex(self):
        x0 = self.x0()
        pts = [x0]
        for i in range(len(x0)):
            p = x0.copy()
            p[i] += 1.5
            pts.append(p)
        return np.array(pts)


# ==============================================================================
# 一連の処理（ワーカースレッド）
# ==============================================================================
class TuningJob:
    def __init__(self, app, cfg):
        self.app = app
        self.cfg = cfg
        self.log = app.log
        self.stop = app.stop_event

    def ensure_folder(self):
        if self.app.folder is None:
            self.app.folder = c.make_run_folder(self.cfg['base'], 'tuning')
            self.log.open_file(os.path.join(self.app.folder, 'run.log'))
            self.log(f"保存先: {self.app.folder}")
        return self.app.folder

    def path(self, suffix):
        return os.path.join(self.app.folder, f"{os.path.basename(self.app.folder)}_{suffix}")

    def measure(self):
        cfg = self.cfg
        folder = self.ensure_folder()
        self.log("=== 実測（ADRC のみ、1回だけ）")
        rec = c.run_trial(self.app.link, cfg['initial'], cfg['target'], np.zeros((c.N_FF, 5)), cfg['T'], cfg['L'],
                          cfg['wait'], self.stop, self.log, cfg['opt_dofs'])
        c.save_trial_npz(os.path.join(folder, 'trial_00.npz'), rec)
        if rec.aborted:
            self.log("停止しました")
            return
        self.analyze(rec)

    def load(self, path):
        self.ensure_folder()
        rec = c.load_trial_npz(path, L=self.cfg['L'])
        if rec.T != self.cfg['T']:
            self.log(f"注意: 試行の T = {rec.T} s を使います（GUI の T = {self.cfg['T']} s ではなく）")
            self.cfg['T'] = rec.T
        rec.T = self.cfg['T']
        self.log(f"試行を読み込みました: {path}")
        c.save_trial_npz(os.path.join(self.app.folder, 'trial_00.npz'), rec)
        self.analyze(rec)

    def analyze(self, rec):
        cfg = self.cfg
        opt = c.opt_dofs_from(rec.initial, rec.target)
        analysis = c.analyze_trial(rec, opt, cfg['bounds'], cfg['id_weights'], {}, {}, self.log,
                                   sens_L=cfg['sens_L'], jitter_sd=cfg['jitter'], n_jitter=10 if cfg['jitter'] > 0 else 0)
        self.app.analysis = analysis
        self.app.rec = rec
        rows = []
        for d, a in sorted(analysis.items()):
            m = a.ident['metrics']
            rows.append({'DOF': d, 'Pi': a.trial.Pi, 'Pf': a.trial.Pf, 'P0': a.trial.P0, 'u_hold': a.trial.u_hold,
                         'J_i_measured': a.J_i, 'tgt_T1': a.tgt[0], 'tgt_wn': a.tgt[1],
                         'sys_T1': a.ident['plant'][0], 'sys_zeta': a.ident['plant'][1], 'sys_wn': a.ident['plant'][2],
                         'sys_b0': a.ident['plant'][3], 'repro_POT_rms': m['pot'], 'repro_uADRC_rms': m['pwm'],
                         'repro_z3_rms_PWMeq': m['z3_pwm'], 'rho': m['rho'], 'kp': a.trial.prm.kp, 'kd': a.trial.prm.kd,
                         'ic': a.trial.prm.ic, 'lambda0': a.trial.prm.lam, 'L': a.trial.L, 'T': rec.T})
        c.write_dict_rows(self.path('models.csv'), rows)
        self.write_conditions()
        self.log(f"同定が終わりました（DOF {sorted(analysis)}）。「重みを探索」で探索を始められます")

    def write_conditions(self):
        cfg = self.cfg
        w = cfg['weights']
        rows = [('実行', 'プログラム', 'optimize_TuningEL_ff_ROS2_2.py', '計算は optimize_EL2_common.py'),
                ('時間', 'T', cfg['T'], ''), ('時間', '周回間隔 L', cfg['L'], '仮定値'),
                ('時間', 'ESO dt', c.ESO_DT, ''), ('時間', 'derivatives h', c.DIFF_H, ''),
                ('重み', '初期値 Q', ' '.join(f'{v:g}' for v in w.Q), '探索の初期値'),
                ('重み', '初期値 Sf', ' '.join(f'{v:g}' for v in w.Sf), ''), ('重み', '初期値 R', w.R, ''),
                ('調整', '振幅の範囲', f"{cfg['A_range'][0]:g} .. {cfg['A_range'][1]:g}", 'max|FF| [PWM]'),
                ('調整', '外側の評価回数の上限', cfg['outer_max'], 'Nelder-Mead'), ('調整', 'R の探索幅', cfg['R_span'], '初期値の ×/÷'),
                ('最適制御', 'ε', cfg['eps'], ''), ('最適制御', '最大反復', cfg['el_max_iter'], ''),
                ('最適制御', '開始点の数', cfg['n_starts'], ''), ('最適制御', '格子', cfg['grid_n'], '')]
        for key, (lo, hi) in cfg['bounds'].items():
            rows.append(('同定', key, f'{lo:g} .. {hi:g}', ''))
        if self.app.rec is not None:
            for d in range(c.N_DOF):
                p = self.app.rec.params.get(d)
                rows.append(('目標値', f'DOF {d}', f"{self.app.rec.initial[d]:g} -> {self.app.rec.target[d]:g}", ''))
                if p is not None:
                    rows.append(('ADRC', f'DOF {d}', f'kp={p.kp:g} kd={p.kd:g} ic={p.ic:g} lambda0={p.lam:g}', ''))
        c.write_kv_csv(self.path('conditions.csv'), rows)

    def search(self):
        cfg = self.cfg
        if not self.app.analysis:
            self.log("先に実測（または試行の読み込み）と同定を行ってください")
            return
        self.ensure_folder()
        self.write_conditions()
        tuner = CommonTuner(self.app.analysis, cfg['weights'], cfg, self.stop, self.log, self.app.report)
        try:
            out = tuner.run()
        finally:
            c.write_dict_rows(self.path('search.csv'), [self._flat(r) for r in tuner.evals])
        rows = []
        for d in tuner.dofs:
            p = out.get('per', {}).get(d, {})
            rows.append({'status': out['status'], 'status_text': out['status_text'], 'DOF': d,
                         'Q': out.get('Q'), 'Sf': out.get('Sf'), 'R': out.get('R'), 'mean_J_pred': out.get('J_pred'),
                         'amplitude': p.get('amplitude'), 'J_pred': p.get('J_pred'), 'J_pred_FF0': out['J_pred_FF0'][d],
                         'extrema': p.get('extrema'), 'coeffs': p.get('coeffs'), 'evaluations': out['evaluations']})
        c.write_dict_rows(self.path('result.csv'), [self._flat(r) for r in rows])
        ok = out['status'] in ('success', 'success_not_converged')
        data = {'source': 'optimize_TuningEL_ff_ROS2_2.py', 'saved_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
                'status': out['status'], 'amplitude_range': list(cfg['A_range']), 'T': cfg['T'], 'L': cfg['L'],
                'dofs': tuner.dofs, 'initial_weights': cfg['weights'].as_dict()}
        if ok:
            data['weights'] = {'Q': out['Q'], 'Sf': out['Sf'], 'R': out['R']}
            data['per_dof'] = {str(d): {'amplitude': float(v['amplitude']), 'J_pred': float(v['J_pred'])}
                               for d, v in out['per'].items()}
        g.save_json(os.path.join(self.app.folder, 'weights.json'), data)
        self.log(f"保存しました: {self.app.folder}（{'共通の重みを weights.json に保存' if ok else '成功した重みはありません'}）")

    @staticmethod
    def _flat(r):
        out = {}
        for k, v in r.items():
            if v is None:
                out[k] = None
            elif isinstance(v, (list, tuple, np.ndarray)):
                arr = np.asarray(v, dtype=float).ravel()
                for i, x in enumerate(arr):
                    out[f'{k}{i}'] = float(x)
            else:
                out[k] = v
        return out


# ==============================================================================
# GUI
# ==============================================================================
class App:
    COLS = ('state', 'cur_w', 'cur_amp', 'cur_J', 'best_w', 'best_amp', 'best_J', 'final')

    def __init__(self):
        self.root = g.setup_root("EL2 weight tuning")
        self.settings = g.load_json(SETTINGS_FILE, {})
        self.log = g.Logger()
        self.stop_event = threading.Event()
        self.worker = None
        self.folder = None
        self.analysis = {}
        self.rec = None
        self.link = None
        self.pending = []
        self.lock = threading.Lock()
        try:
            self.link = c.RobotLink('el2_tuning_gui')
        except Exception as exc:
            messagebox.showerror("ROS2", f"ROS2 に接続できません（保存した試行の読み込みだけ使えます）:\n{exc}")
        s = self.settings
        nb = ttk.Notebook(self.root)
        nb.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        tab = ttk.Frame(nb)
        nb.add(tab, text="基本")
        self.basic = g.Fields(tab, "実測・保存")
        self.basic.add('T', "動作完了時間 T [s]", s.get('T', 1.5))
        self.basic.add('wait', "初期姿勢に戻った後の待機時間 [s]", s.get('wait', 5.0))
        self.basic.add('base', "保存先フォルダ", s.get('base', os.path.expanduser('~/ArmRobot/optimize_data')), width=50)
        ttk.Button(self.basic.frame, text="参照…", command=lambda: g.ask_folder(self.basic.vars['base'])).grid(
            row=self.basic.row - 1, column=2, sticky='w')
        self.timing = g.build_timing(tab, s)
        self.table = g.build_target_tab(nb, s, lambda: self.link, self.log)
        tab_w = ttk.Frame(nb)
        nb.add(tab_w, text="初期の重み・最適制御")
        self.weights = g.build_weights(tab_w, s)
        self.elf = g.Fields(tab_w, "勾配法（optimize_EL_ff_ROS2_2.py と同じにすること）")
        self.elf.add('eps', "終了閾値 ε（∫‖Π∂H/∂u‖²dt）", s.get('eps', 1e-3))
        self.elf.add('el_max_iter', "最大反復回数", s.get('el_max_iter', 300))
        self.elf.add('n_starts', "開始点の数", s.get('n_starts', 3))
        self.elf.add('grid_n', "初期値の格子", s.get('grid_n', 22))
        tab_t = ttk.Frame(nb)
        nb.add(tab_t, text="調整")
        self.tune = g.Fields(tab_t, "FF の許容振幅と探索")
        self.tune.add('A_min', "振幅 max|FF| の下限 [PWM]", s.get('A_min', 10.0))
        self.tune.add('A_max', "振幅 max|FF| の上限 [PWM]", s.get('A_max', 30.0))
        self.tune.add('outer_max', "外側（Q・Sf の比）の評価回数の上限", s.get('outer_max', 20))
        self.tune.add('R_span', "R を探す範囲（初期値の ×/÷ この値）", s.get('R_span', 1e6))
        tab_i = ttk.Frame(nb)
        nb.add(tab_i, text="同定")
        self.bounds = g.build_bounds(tab_i, s)

        ctrl = ttk.Frame(self.root)
        ctrl.pack(fill=tk.X, padx=6, pady=2)
        self.btns = []
        for text, cmd in (("実測1回＋同定（ロボットが動きます）", self.do_measure),
                          ("保存した試行を読み込んで同定…", self.do_load),
                          ("重みを探索", self.do_search)):
            b = ttk.Button(ctrl, text=text, command=cmd)
            b.pack(side=tk.LEFT, padx=3)
            self.btns.append(b)
        self.btn_stop = ttk.Button(ctrl, text="停止", command=self.stop_event.set, state='disabled')
        self.btn_stop.pack(side=tk.LEFT, padx=3)
        self.status = ttk.Label(ctrl, text="待機中")
        self.status.pack(side=tk.LEFT, padx=10)

        frame = ttk.LabelFrame(self.root, text="探索の状況（重みは全自由度で共通。1行目が共通の重み、2行目以降が自由度ごとの振幅と予測 J_i）")
        frame.pack(fill=tk.X, padx=6, pady=4)
        heads = {'state': '状態', 'cur_w': '探索中の重み (q_v, q_a, R)', 'cur_amp': '振幅', 'cur_J': '予測J_i',
                 'best_w': '最良の重み (q_v, q_a, R)', 'best_amp': '振幅', 'best_J': '予測J_i（共通行は平均）',
                 'final': '決定した重み'}
        self.tree = ttk.Treeview(frame, columns=self.COLS, show='tree headings', height=6)
        self.tree.heading('#0', text='')
        self.tree.column('#0', width=90)
        for k in self.COLS:
            self.tree.heading(k, text=heads[k])
            self.tree.column(k, width=180 if k.endswith('_w') or k == 'final' else 90)
        self.tree.pack(fill=tk.X)
        self.logpane = g.LogPane(self.root, self.log, height=12)
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)
        self._after = self.root.after(200, self.tick)

    # ワーカースレッドから呼ばれる（表の更新は GUI スレッドで行う）
    def report(self, kind, row):
        with self.lock:
            self.pending.append((kind, dict(row)))

    def _row(self, iid, text):
        if not self.tree.exists(iid):
            self.tree.insert('', tk.END, iid=iid, text=text, values=[''] * len(self.COLS))
        return dict(zip(self.COLS, self.tree.item(iid, 'values')))

    def apply_reports(self):
        with self.lock:
            items, self.pending = self.pending, []
        fmt_w = lambda r: f"{r.get('q_v', float('nan')):.3g}, {r.get('q_a', float('nan')):.3g}, {r.get('R', float('nan')):.3g}"
        for kind, r in items:
            common = self._row('all', '共通の重み')
            if kind == 'current':                                       # 1つの自由度を1つの重みで計算した結果
                common.update({'state': '探索中', 'cur_w': fmt_w(r)})
                d = r['DOF']
                vals = self._row(f"d{d}", f"DOF {d}")
                vals.update({'state': '探索中', 'cur_w': '（共通）', 'cur_amp': f"{r['amplitude']:.2f}",
                             'cur_J': f"{r['J_pred']:.3g}"})
                self.tree.item(f"d{d}", values=[vals[k] for k in self.COLS])
            elif kind in ('best', 'final'):
                if kind == 'best' or 'per' in r:
                    common.update({'best_w': fmt_w(r), 'best_J': f"{r['J_pred']:.3g}"})
                    for d, p in r.get('per', {}).items():
                        vals = self._row(f"d{d}", f"DOF {d}")
                        vals.update({'best_w': '（共通）', 'best_amp': f"{p['amplitude']:.2f}",
                                     'best_J': f"{p['J_pred']:.3g}"})
                        self.tree.item(f"d{d}", values=[vals[k] for k in self.COLS])
                if kind == 'final':
                    st = r['status']
                    text = {'success': '成功', 'success_not_converged': '成功（未収束）', 'no_solution': '解なし',
                            'aborted': '停止', 'verify_failed': '確認で範囲外'}[st]
                    ok = st in ('success', 'success_not_converged')
                    common['state'] = text
                    common['final'] = (f"Q={['%.3g' % v for v in r['Q']]} Sf={['%.3g' % v for v in r['Sf']]} R={r['R']:.3g}"
                                       if ok else '（決定できず）')
                    for d in r.get('dofs', []):
                        vals = self._row(f"d{d}", f"DOF {d}")
                        vals['state'] = text
                        vals['final'] = '（共通）' if ok else ''
                        self.tree.item(f"d{d}", values=[vals[k] for k in self.COLS])
            self.tree.item('all', values=[common[k] for k in self.COLS])

    def collect(self):
        T = g.parse_float(self.basic.get('T'), 'T', lo=0.05, hi=10.0)
        wait = g.parse_float(self.basic.get('wait'), '待機時間', lo=0.0)
        base = self.basic.get('base').strip()
        if not base:
            raise ValueError("保存先フォルダを指定してください")
        os.makedirs(base, exist_ok=True)
        L, sens, jit = g.read_timing(self.timing)
        initial, target = self.table.get()
        bounds, idw = g.read_bounds(self.bounds)
        A_min = g.parse_float(self.tune.get('A_min'), 'A_min', lo=0.0)
        A_max = g.parse_float(self.tune.get('A_max'), 'A_max', positive=True)
        if not A_min < A_max:
            raise ValueError("振幅の下限は上限より小さくしてください")
        return {'T': T, 'wait': wait, 'base': base, 'L': L, 'sens_L': sens, 'jitter': jit, 'initial': initial,
                'target': target, 'opt_dofs': c.opt_dofs_from(initial, target), 'bounds': bounds, 'id_weights': idw,
                'weights': g.read_weights(self.weights),
                'eps': g.parse_float(self.elf.get('eps'), 'ε', lo=0.0),
                'el_max_iter': int(g.parse_float(self.elf.get('el_max_iter'), '最大反復', lo=1)),
                'n_starts': int(g.parse_float(self.elf.get('n_starts'), '開始点の数', lo=1)),
                'grid_n': int(g.parse_float(self.elf.get('grid_n'), '格子', lo=4)),
                'A_range': (A_min, A_max), 'outer_max': int(g.parse_float(self.tune.get('outer_max'), '評価回数', lo=1)),
                'R_span': g.parse_float(self.tune.get('R_span'), 'R の探索幅', lo=10.0)}

    def save_settings(self, cfg):
        data = {k: cfg[k] for k in ('T', 'wait', 'base', 'L', 'initial', 'target', 'eps', 'el_max_iter', 'n_starts',
                                    'grid_n', 'outer_max', 'R_span', 'id_weights')}
        data.update({'A_min': cfg['A_range'][0], 'A_max': cfg['A_range'][1], 'jitter': cfg['jitter'] * 1e3,
                     'sens_L': ", ".join(f"{v:g}" for v in cfg['sens_L']), 'weights': cfg['weights'].as_dict(),
                     'bounds': {k: list(v) for k, v in cfg['bounds'].items()}})
        g.save_json(SETTINGS_FILE, data)

    def launch(self, fn):
        if self.worker is not None and self.worker.is_alive():
            return
        try:
            cfg = self.collect()
        except ValueError as exc:
            messagebox.showerror("Settings", str(exc))
            return
        self.save_settings(cfg)
        self.stop_event.clear()
        job = TuningJob(self, cfg)

        def body():
            try:
                fn(job)
            except c.StopRequested:
                self.log("停止しました")
            except Exception as exc:
                self.log(f"エラー: {exc}\n{traceback.format_exc()}")
        self.worker = threading.Thread(target=body, daemon=True)
        self.worker.start()
        for b in self.btns:
            b.configure(state='disabled')
        self.btn_stop.configure(state='normal')
        self.status.configure(text="実行中")

    def do_measure(self):
        if self.link is None:
            messagebox.showerror("ROS2", "ROS2 に接続していません")
            return
        try:
            cfg = self.collect()
        except ValueError as exc:
            messagebox.showerror("Settings", str(exc))
            return
        if not cfg['opt_dofs']:
            messagebox.showerror("Target", "目標値が初期値と違う自由度がありません")
            return
        moves = "\n".join(f"  DOF {d}: {cfg['initial'][d]:g} → {cfg['target'][d]:g}" for d in cfg['opt_dofs'])
        if not messagebox.askokcancel("Measure", f"ロボットが1回動きます（ADRC のみ、FF=0）。\n\n{moves}\n\n開始しますか？"):
            return
        self.folder = None
        self.analysis = {}
        self.tree.delete(*self.tree.get_children())
        self.launch(lambda job: job.measure())

    def do_load(self):
        path = filedialog.askopenfilename(title="Load trial", filetypes=[('trial npz', '*.npz')])
        if not path:
            return
        self.folder = None
        self.analysis = {}
        self.tree.delete(*self.tree.get_children())
        self.launch(lambda job: job.load(path))

    def do_search(self):
        if not self.analysis:
            messagebox.showinfo("Search", "先に実測（または試行の読み込み）と同定を行ってください")
            return
        self.launch(lambda job: job.search())

    def tick(self):
        self.logpane.poll()
        self.apply_reports()
        if self.worker is not None and not self.worker.is_alive():
            self.worker = None
            for b in self.btns:
                b.configure(state='normal')
            self.btn_stop.configure(state='disabled')
            self.status.configure(text="待機中")
        self._after = self.root.after(200, self.tick)

    def on_close(self):
        if self.worker is not None and self.worker.is_alive():
            if not messagebox.askokcancel("Quit", "実行中です。停止して終了しますか？"):
                return
            self.stop_event.set()
            self.worker.join(timeout=30)
        try:
            self.root.after_cancel(self._after)
        except Exception:
            pass
        if self.link is not None:
            self.link.close()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    app = App()
    try:
        app.run()
    finally:
        if app.link is not None:
            app.link.close()


if __name__ == '__main__':
    main()
