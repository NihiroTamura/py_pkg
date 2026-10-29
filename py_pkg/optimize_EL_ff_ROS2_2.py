#!/usr/bin/env python3
"""
ADRCに加えるFF入力を、内部ADRCシミュレーションを組み込んだ離散時間オイラー・ラグランジュ法で求めるプログラム（GUI）

<処理の流れ>
  Step 1  ADRCのみ（全自由度 FF=0）で目標値を送り、max(1.5T, 5 s) のあいだ POT・PWM・z1・z2・z3・derivatives を記録する。
          測定後は FF=0 → 初期姿勢へ戻し、GUIで設定した時間だけ待つ。
  Step 2  実測 POT（y = P − Pf）に ζ=1 の3次遅れ系（目標軌道）を当てはめ、累積誤差 J_i を計算する。
  Step 3  実測（z = P − P0、入力は実際の PWM）にシステムモデル（3次遅れ系）を同定する。
          開ループ同定 → POT・u_ADRC・z3 の3チャネル閉ループ同定。ファーム複製・再現精度・周回間隔の感度も記録する。
  Step 4  システムモデル＋実機ADRC の閉ループで、目標軌道との誤差 e と FF の大きさの評価関数を最小にする
          5次関数FF（極値 (t1,y1),(t2,y2) を変数、極値ちょうど2個）を勾配法（随伴方程式）で求めて送り、max(1.5T, 5 s) 測定する。
          評価関数は 0 ≤ t < T で誤差と FF、T ≤ t ≤ t_f では誤差だけを評価する。
          J ≤ 閾値なら終了。超えたら Step 3 から繰り返す（同定・目標軌道は直前の試行の実測でやり直す）。
<計算の定義> optimize_EL2_common.py（3プログラム共通）を参照。
<対応Arduino プログラム> ADRC_ROS2_tk_FF_ADRCparamChange-board1〜5（変更しない）
<ビューア> view_optimization_2.py   <重みの自動調整> optimize_TuningEL_ff_ROS2_2.py

<保存>  <保存先>/optimize_YYYYMMDD_HHMM/
    optimize_…_results.csv        … 最も累積誤差が小さかったFFありの試行で送ったFFの極値（既存 optimize_EL_ff_ROS2.py と同じ列）
    optimize_…_conditions.csv     … 再現に必要な条件（T・周回間隔・dt・h・重み・同定の上下限・閾値・ADRCパラメータ・目標値 …）
    optimize_…_loops.csv          … 試行（ループ）ごと・自由度ごとの評価値・FF・同定結果・再現精度・次に送ったFF
    optimize_…_best_trial.csv     … 最良の試行の受信データ（GUI_android_tk.py の記録CSVと同じ列＋FF・ADRC・z1・z2・derivatives）
    optimize_…_best_trial_grid.csv… 最良の試行を周回番号でそろえた解析用データ（実測・目標軌道・モデル・入力・z3）
    trials/trial_NN.npz           … 各試行の受信データ（再解析用）
    loops/loop_NN.npz             … ビューア用
    run.log                       … ログ
"""
import os
import threading
import time
import traceback

import numpy as np

import tkinter as tk
from tkinter import ttk, messagebox

if __package__:
    from . import optimize_EL2_common as c
    from . import optimize_EL2_gui as g
else:
    import optimize_EL2_common as c
    import optimize_EL2_gui as g

SETTINGS_FILE = os.path.expanduser('~/.armrobot_el2_settings.json')


# ==============================================================================
# 最適化の手順（ワーカースレッド）
# ==============================================================================
class OptimizationRun:
    def __init__(self, link, cfg, log, stop_event):
        self.link = link
        self.cfg = cfg
        self.log = log
        self.stop = stop_event
        self.folder = None
        self.name = None
        self.loop_rows = []
        self.trial_rows = []
        self.best = None                        # (J, n, rec, analysis) … FF ありの試行の最良
        self.adrc_only = None                   # (J, rec, analysis)
        self.session = time.strftime('%Y%m%d_%H%M%S')
        self.status = '実行中'

    # ---------------- 保存 ----------------
    def path(self, suffix):
        return os.path.join(self.folder, f"{self.name}_{suffix}")

    def conditions_rows(self, params):
        cfg = self.cfg
        w = cfg['weights']
        rows = [
            ('実行', '開始日時', self.session, ''),
            ('実行', '状態', self.status, ''),
            ('実行', 'プログラム', 'optimize_EL_ff_ROS2_2.py', '計算は optimize_EL2_common.py'),
            ('時間', 'T', cfg['T'], '動作完了時間 [s]（FF は 0 ≤ t < T）'),
            ('時間', '測定・評価区間', c.eval_time(cfg['T']), f'max({c.EVAL_FACTOR}T, {c.EVAL_MIN_TIME} s) [s]。FF の項は t < T だけ'),
            ('時間', '制御の周回間隔 L', cfg['L'], '仮定値 [s]。ESO・ADRC は1周に1回'),
            ('時間', 'ESO の dt', c.ESO_DT, 'ファームの式の中の係数 [s]'),
            ('時間', 'derivatives の h', c.DIFF_H, 'ファームの式の中の係数 [s]'),
            ('時間', 'derivatives の RCローパス係数', c.OMEGA_LPF, ''),
            ('時間', 'ROS2 配信周期（参考）', c.PUBLISH_DT, '周回間隔としては使わない'),
            ('時間', '評価区間の最終ステップ N', c.eval_steps(cfg['T'], cfg['L']), 'k = 0..N'),
            ('時間', 'FF の周回数 N_ff', c.ff_steps(cfg['T'], cfg['L']), '#{k : kL < T}'),
            ('時間', '初期姿勢での待機時間', cfg['wait'], '[s]（実際は max(待機時間, T+0.5)）'),
            ('時間', '受信時刻', 'PC（WSL2）の時計', '周回の復元では欠落の判定にだけ使う。CSV の時刻は数%遅れることがある'),
            ('重み', 'Q', ' '.join(f'{v:g}' for v in w.Q), '位置・速度・加速度の誤差'),
            ('重み', 'Sf', ' '.join(f'{v:g}' for v in w.Sf), '終端'),
            ('重み', 'R', w.R, 'FF の大きさ'),
            ('重み', '重みの出どころ', cfg.get('weights_source', 'GUI に入力'), '全自由度で共通'),
            ('最適制御', 'ε（∫‖Π∂H/∂u‖²dt）', cfg['eps'], '勾配法の終了閾値'),
            ('最適制御', '勾配法の最大反復', cfg['el_max_iter'], ''),
            ('最適制御', '開始点の数', cfg['n_starts'], ''),
            ('最適制御', '初期値の格子', cfg['grid_n'], '(t1,t2) の分割数'),
            ('最適制御', 'FF の変数', '極値 (t1, t2, y1, y2)', 'f(0)=f(T)=0, (0,T) の極値ちょうど2個'),
            ('最適制御', '内部閉ループの増幅率の上限 ρ^N', c.CL_GROWTH_MAX, '超えた自由度は FF を更新しない'),
            ('評価', '累積誤差', 'J_i = mean_k ((POT − Target)/ΔP)²', 'J = 対象自由度の平均'),
            ('評価', 'J の終了閾値', cfg['J_thr'], ''),
            ('評価', '最大ループ数', cfg['max_loops'], 'FF ありの試行の回数の上限'),
        ]
        for key, (lo, hi) in cfg['bounds'].items():
            rows.append(('同定', key, f'{lo:g} .. {hi:g}', '上下限'))
        rows += [('同定', 'u_ADRC 残差の重み', cfg['id_weights']['w_pwm'], ''),
                 ('同定', 'z3 残差の重み', cfg['id_weights']['w_z3'], ''),
                 ('評価', '感度評価の周回間隔', ' '.join(f'{v:g}' for v in cfg['sens_L']), ''),
                 ('評価', '周回間隔のばらつき σ', cfg['jitter'], '[s]')]
        for d in range(c.N_DOF):
            p = params.get(d) if params else None
            rows.append(('目標値', f'DOF {d}', f"{cfg['initial'][d]:g} -> {cfg['target'][d]:g}",
                         '最適化対象' if d in cfg['opt_dofs'] else ''))
            if p is not None:
                rows.append(('ADRC', f'DOF {d}', f'kp={p.kp:g} kd={p.kd:g} ic={p.ic:g} lambda0={p.lam:g}', '実機から受信'))
        return rows

    def save_trial_raw(self, n, rec):
        os.makedirs(os.path.join(self.folder, 'trials'), exist_ok=True)
        c.save_trial_npz(os.path.join(self.folder, 'trials', f'trial_{n:02d}.npz'), rec)

    def save_all(self, params):
        """途中で止めても呼ぶ。今まで得たものを全部書く"""
        if self.folder is None:
            return
        try:
            c.write_kv_csv(self.path('conditions.csv'), self.conditions_rows(params))
            if self.loop_rows:
                c.write_dict_rows(self.path('loops.csv'), self.loop_rows)
            if self.trial_rows:
                c.write_dict_rows(self.path('trials.csv'), self.trial_rows)
            best = self.best or (None if self.adrc_only is None else (self.adrc_only[0], 0) + self.adrc_only[1:])
            if best is not None:
                J, n, rec, analysis, kc_map = best
                c.write_results_csv(self.path('results.csv'), rec.initial, rec.target, rec.T, rec.ff)
                c.write_trial_timeseries(self.path('best_trial.csv'), rec, kc_map)
                c.write_trial_grid(self.path('best_trial_grid.csv'), analysis, rec.L)
                self.log(f"保存: 最良の試行は 試行{n}（J = {J:.4g}）")
            self.log(f"保存先: {self.folder}")
        except Exception as exc:
            self.log(f"保存に失敗しました: {exc}\n{traceback.format_exc()}")

    # ---------------- 本体 ----------------
    def run(self):
        cfg = self.cfg
        log = self.log
        params = None
        try:
            self.folder = c.make_run_folder(cfg['base'], 'optimize')
            self.name = os.path.basename(self.folder)
            log.open_file(os.path.join(self.folder, 'run.log'))
            c.write_pointer(self.folder, self.session)
            log(f"=== 開始: 保存先 {self.folder}")
            log(f"最適化対象 DOF {cfg['opt_dofs']}  T = {cfg['T']} s, 周回間隔 L = {cfg['L']} s（仮定値）, "
                f"評価区間 max({c.EVAL_FACTOR}T, {c.EVAL_MIN_TIME:g} s) = {c.eval_time(cfg['T']):.3f} s（N = {c.eval_steps(cfg['T'], cfg['L'])}）")
            params = self.link.adrc_params()
            c.write_kv_csv(self.path('conditions.csv'), self.conditions_rows(params))
            ff = np.zeros((c.N_FF, 5))
            prev_tgt, prev_sys, prev_p = {}, {}, {}
            n = 0
            while True:
                label = 'ADRCのみ' if n == 0 else f'ループ{n}'
                log(f"=== 試行 {n}（{label}）===")
                for d in cfg['opt_dofs']:
                    t1, y1, t2, y2 = c.extrema_of(ff[c.FF_INDEX[d]], cfg['T'])
                    if n > 0:
                        log(f"  DOF {d}: 送るFF 極値 ({t1:.3f} s, {y1:+.2f}) / ({t2:.3f} s, {y2:+.2f})")
                rec = c.run_trial(self.link, cfg['initial'], cfg['target'], ff, cfg['T'], cfg['L'], cfg['wait'],
                                  self.stop, log, cfg['opt_dofs'])
                params = rec.params
                self.save_trial_raw(n, rec)
                if rec.aborted:
                    log("停止しました（この試行の測定は中断）")
                    self.status = '停止'
                    break
                changed = [d for d in cfg['opt_dofs'] if rec.params_after.get(d) is None or rec.params.get(d) is None
                           or rec.params_after[d].as_list() != rec.params[d].as_list()]
                if changed:
                    log(f"  警告: 試行中に ADRC パラメータが変わった（または受信できない）自由度があります: DOF {changed}")
                analysis = c.analyze_trial(rec, cfg['opt_dofs'], cfg['bounds'], cfg['id_weights'], prev_tgt, prev_sys,
                                           log, sens_L=cfg['sens_L'], jitter_sd=cfg['jitter'],
                                           n_jitter=10 if cfg['jitter'] > 0 else 0)
                if not analysis:
                    log("解析できる自由度がありませんでした。終了します")
                    self.status = '解析失敗'
                    break
                kc_map = self._kc_map(rec, analysis)
                for d, a in analysis.items():
                    prev_tgt[d] = a.tgt
                    prev_sys[d] = a.ident['plant']
                J = c.mean_J(analysis)
                log(f"  累積誤差 J = {J:.4g}（RMS {c.rms_percent(J):.2f}% of ΔP）  "
                    + "  ".join(f"DOF{d}: {a.J_i:.4g}" for d, a in sorted(analysis.items())))
                if n == 0:
                    self.adrc_only = (J, rec, analysis, kc_map)
                elif self.best is None or J < self.best[0]:
                    self.best = (J, n, rec, analysis, kc_map)
                    log(f"  最良を更新しました（試行{n}）")
                best_n = self.best[1] if self.best else None
                done = n >= 1 and np.isfinite(J) and J <= cfg['J_thr']
                last = n >= cfg['max_loops']
                el = {}
                if not done and not last and not self.stop.is_set():
                    log("  Step 4: 最適制御（EL）でFFを計算します")
                    ff_next = ff.copy()
                    for d in sorted(analysis):
                        if self.stop.is_set():
                            break
                        w = cfg['weights']                                  # 全自由度で共通の重み
                        log(f"  DOF {d}: 重み Q={w.Q} Sf={w.Sf} R={w.R:g}")
                        res, info = c.optimize_ff(analysis[d], w, cfg['T'], cfg['L'], cfg['eps'], cfg['el_max_iter'],
                                                  self.stop, log, warm=prev_p.get(d), grid_n=cfg['grid_n'],
                                                  n_starts=cfg['n_starts'])
                        el[d] = (res, info)
                        t1, y1, t2, y2 = res.extrema
                        log(f"  DOF {d}: EL [{res.status}] J_EL = {res.J:.4g}（FF=0 なら {info['J_zero']:.4g}）, "
                            f"∫‖Π∂H/∂u‖²dt = {res.proj_grad2:.3g}, 反復 {res.iters}, 極値 ({t1:.3f}, {y1:+.2f}) / "
                            f"({t2:.3f}, {y2:+.2f}), 予測PWMの最大 {info['pwm_peak']:.1f}, ρ^N = {info['growth']:.3g}")
                        if res.status == 'aborted':
                            continue
                        if not info['gate_ok']:
                            log(f"  DOF {d}: 内部閉ループの増幅率 ρ^N = {info['growth']:.3g} > {c.CL_GROWTH_MAX:g} のため、"
                                f"このモデルは信用できません。FF は前回のままにします")
                            continue
                        if info['pwm_peak'] > c.PWM_LIMIT:
                            log(f"  DOF {d}: 警告: 予測の PWM が ±{c.PWM_LIMIT:g} を超えます（弁で飽和します）")
                        ff_next[c.FF_INDEX[d]] = res.coeffs
                        if res.status != 'zero':
                            prev_p[d] = np.array([t1, t2, y1, y2])
                else:
                    ff_next = ff
                self._record(n, label, rec, analysis, J, el, best_n, ff_next)
                c.save_loop_npz(self.folder, n, c.loop_payload(n, rec, analysis, J, el, self.session,
                                                               is_best=(best_n == n), label=label))
                c.write_dict_rows(self.path('loops.csv'), self.loop_rows)
                c.write_dict_rows(self.path('trials.csv'), self.trial_rows)
                if done:
                    log(f"=== J = {J:.4g} ≤ 閾値 {cfg['J_thr']:g} なので終了します")
                    self.status = '収束'
                    break
                if last:
                    log(f"=== 最大ループ数 {cfg['max_loops']} に達したので終了します")
                    self.status = '最大ループ数'
                    break
                if self.stop.is_set():
                    log("停止しました")
                    self.status = '停止'
                    break
                ff = ff_next
                n += 1
        except Exception as exc:
            log(f"エラー: {exc}\n{traceback.format_exc()}")
            self.status = f'エラー: {exc}'
            try:
                c.return_to_initial(self.link, cfg['initial'], cfg['T'], log)
            except Exception:
                pass
        finally:
            self.save_all(params)
            if self.folder:
                try:
                    with open(os.path.join(self.folder, c.FINISHED_FILE), 'w', encoding='utf-8') as f:
                        f.write('{"state": "%s"}' % self.status)
                    c.write_pointer(self.folder, self.session, state='finished')
                except OSError:
                    pass
            log(f"=== 終了（{self.status}）。次の試行を開始できます")
            log.close_file()

    def _kc_map(self, rec, analysis):
        """ボードごとの「目標値が変わった周回」（時系列CSVの FF 列に使う）。DofTrial から逆算する"""
        out = {}
        for board, offset, count in c.BOARD_LAYOUT:
            raw = rec.raw.get(board, {})
            dofs = [d for d in analysis if offset <= d < offset + count]
            if not dofs or 'pwm' not in raw:
                continue
            labels = c._message_iterations(raw['pwm'][1])
            # 周回の番号の原点はメッセージの値の変化の数え方で決まるので、比例キックの周回を探し直す
            tr = analysis[dofs[0]].trial
            V = raw['pwm'][1][:, dofs[0] - offset]
            target = tr.pwm[0]
            hit = np.flatnonzero(np.abs(V - target) < 1e-4)
            if len(hit):
                out[board] = int(labels[hit[0]])
        return out

    def _record(self, n, label, rec, analysis, J, el, best_n, ff_next):
        T = self.cfg['T']
        self.trial_rows.append({'trial': n, 'label': label, 'started_at': rec.started_at, 'J': J,
                                'RMS_percent': c.rms_percent(J), 'best_FF_trial_so_far': best_n,
                                'n_dof': len(analysis)})
        for d, a in sorted(analysis.items()):
            tr, idn = a.trial, a.ident
            m = idn['metrics']
            ext = c.extrema_of(tr.coeffs, T)
            row = {'trial': n, 'label': label, 'DOF': d, 'FF_no': c.FF_INDEX[d] + 1, 'Pi': tr.Pi, 'Pf': tr.Pf,
                   'dP': tr.dP, 'P0': tr.P0, 'u_hold': tr.u_hold, 'J_i': a.J_i, 'RMS_percent': c.rms_percent(a.J_i),
                   'J': J, 'is_best_FF_trial': best_n == n,
                   'ff_a': tr.coeffs[0], 'ff_b': tr.coeffs[1], 'ff_c': tr.coeffs[2], 'ff_d': tr.coeffs[3],
                   'ff_e': tr.coeffs[4], 't1': ext[0], 'y1': ext[1], 't2': ext[2], 'y2': ext[3],
                   'tgt_T1': a.tgt[0], 'tgt_wn': a.tgt[1], 'tgt_fit_rms': a.tgt_rms,
                   'sys_T1': idn['plant'][0], 'sys_zeta': idn['plant'][1], 'sys_wn': idn['plant'][2],
                   'sys_b0': idn['plant'][3], 'sys_closed_loop': idn['closed_loop'],
                   'sys_channels': '+'.join(idn['channels']), 'sys_pinned': '+'.join(idn['pinned']),
                   'repro_POT_rms': m['pot'], 'repro_uADRC_rms': m['pwm'], 'repro_z3_rms_PWMeq': m['z3_pwm'],
                   'rho': m['rho'], 'replay_uADRC_rms': a.replay['uA_rms'], 'replay_z3_rms_PWMeq': a.replay['z3_rms_pwm'],
                   'POT_from_ESO_ratio': a.replay['float_ratio'], 'board_alignment': a.board_method,
                   'drops': tr.n_drops, 'L': tr.L}
            for r in (a.sens or []):
                row[f"sens_L{r['L'] * 1000:.2f}ms_POT"] = r['same_plant']['pot']
                row[f"sens_L{r['L'] * 1000:.2f}ms_uADRC"] = r['same_plant']['pwm']
                row[f"sens_L{r['L'] * 1000:.2f}ms_z3"] = r['same_plant']['z3_pwm']
            if a.jitter:
                row.update({'jitter_POT': a.jitter['pot'], 'jitter_uADRC': a.jitter['pwm'], 'jitter_z3': a.jitter['z3_pwm']})
            if d in el:
                res, info = el[d]
                ext2 = res.extrema
                row.update({'next_status': res.status, 'next_J_EL': res.J, 'next_J_EL_FF0': info['J_zero'],
                            'next_proj_grad2': res.proj_grad2, 'next_free_grad2': res.free_grad2,
                            'next_iters': res.iters, 'next_t1': ext2[0], 'next_y1': ext2[1], 'next_t2': ext2[2],
                            'next_y2': ext2[3], 'next_pwm_peak': info['pwm_peak'], 'next_growth': info['growth'],
                            'next_sent': bool(np.allclose(ff_next[c.FF_INDEX[d]], res.coeffs))})
            self.loop_rows.append(row)


# ==============================================================================
# GUI
# ==============================================================================
class App:
    def __init__(self):
        self.root = g.setup_root("EL2 FF optimizer")
        self.settings = g.load_json(SETTINGS_FILE, {})
        self.log = g.Logger()
        self.link = None
        self.worker = None
        self.stop_event = threading.Event()
        try:
            self.link = c.RobotLink('el2_optimizer_gui')
        except Exception as exc:
            messagebox.showerror("ROS2", f"ROS2 に接続できません:\n{exc}")

        top = ttk.Frame(self.root)
        top.pack(fill=tk.BOTH, expand=True)
        nb = ttk.Notebook(top)
        nb.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        tab = ttk.Frame(nb)
        nb.add(tab, text="基本")
        s = self.settings
        self.basic = g.Fields(tab, "動作・終了条件・保存")
        self.basic.add('T', "動作完了時間 T [s]", s.get('T', 1.5), note=f"測定・評価は max({c.EVAL_FACTOR}T, {c.EVAL_MIN_TIME:g} s)")
        self.basic.add('wait', "初期姿勢に戻った後の待機時間 [s]", s.get('wait', 5.0), note="実際は max(待機時間, T+0.5)")
        self.basic.add('max_loops', "最大ループ数（FFありの試行）", s.get('max_loops', 10))
        self.basic.add('J_thr', "累積誤差 J の終了閾値", s.get('J_thr', 1e-4),
                       note="J = mean((POT−目標軌道)/ΔP)²。1e-4 は RMS 1% of ΔP")
        self.basic.add('base', "保存先フォルダ", s.get('base', os.path.expanduser('~/ArmRobot/optimize_data')), width=50)
        ttk.Button(self.basic.frame, text="参照…", command=lambda: g.ask_folder(self.basic.vars['base'])).grid(
            row=self.basic.row - 1, column=2, sticky='w')
        self.timing = g.build_timing(tab, s)

        self.table = g.build_target_tab(nb, s, lambda: self.link, self.log)

        tab_w = ttk.Frame(nb)
        nb.add(tab_w, text="最適制御")
        self.weights = g.build_weights(tab_w, s)
        self.elf = g.Fields(tab_w, "勾配法")
        self.elf.add('eps', "終了閾値 ε（∫‖Π∂H/∂u‖²dt）", s.get('eps', 1e-3))
        self.elf.add('el_max_iter', "最大反復回数", s.get('el_max_iter', 300))
        self.elf.add('n_starts', "開始点の数（多点開始）", s.get('n_starts', 3))
        self.elf.add('grid_n', "初期値を探す (t1,t2) 格子の分割数", s.get('grid_n', 22))
        wbar = ttk.Frame(tab_w)
        wbar.pack(fill=tk.X, padx=6, pady=4)
        self.weights_source = s.get('weights_source', 'GUI に入力')
        self.wlabel = ttk.Label(wbar, text="")
        ttk.Button(wbar, text="重みを読み込む（重み調整の結果 weights.json）…", command=self.load_weights).pack(side=tk.LEFT)
        self.wlabel.pack(side=tk.LEFT, padx=6)
        self.update_wlabel()

        tab_i = ttk.Frame(nb)
        nb.add(tab_i, text="同定")
        self.bounds = g.build_bounds(tab_i, s)

        ctrl = ttk.Frame(self.root)
        ctrl.pack(fill=tk.X, padx=6, pady=4)
        self.btn_start = ttk.Button(ctrl, text="開始", command=self.start)
        self.btn_start.pack(side=tk.LEFT, padx=4)
        self.btn_stop = ttk.Button(ctrl, text="停止", command=self.request_stop, state='disabled')
        self.btn_stop.pack(side=tk.LEFT, padx=4)
        ttk.Button(ctrl, text="設定を保存", command=self.save_settings).pack(side=tk.LEFT, padx=4)
        self.status = ttk.Label(ctrl, text="待機中")
        self.status.pack(side=tk.LEFT, padx=12)
        self.logpane = g.LogPane(self.root, self.log)
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)
        self._after = self.root.after(200, self.tick)

    def update_wlabel(self):
        self.wlabel.configure(text=f"重みは全自由度で共通（出どころ: {self.weights_source}）")

    def load_weights(self):
        """optimize_TuningEL_ff_ROS2_2.py が保存した weights.json の共通の重みを、上の欄に入れる"""
        from tkinter import filedialog
        path = filedialog.askopenfilename(title="Load weights", filetypes=[('JSON', '*.json')])
        if not path:
            return
        data = g.load_json(path, {})
        try:
            w = c.Weights.from_dict(data['weights'])
        except (KeyError, TypeError, ValueError):
            messagebox.showerror("Weights", f"共通の重み（weights）がありません（探索が成功していない結果の可能性）:\n{path}")
            return
        for i in range(3):
            self.weights.set(f'Q{i}', f"{w.Q[i]:.6g}")
            self.weights.set(f'Sf{i}', f"{w.Sf[i]:.6g}")
        self.weights.set('R', f"{w.R:.6g}")
        self.weights_source = path
        self.update_wlabel()
        self.log(f"重みを読み込みました: Q={w.Q} Sf={w.Sf} R={w.R:g}（{path}）")

    def collect(self):
        T = g.parse_float(self.basic.get('T'), 'T', lo=0.05, hi=10.0)
        wait = g.parse_float(self.basic.get('wait'), '待機時間', lo=0.0, hi=600.0)
        max_loops = int(g.parse_float(self.basic.get('max_loops'), '最大ループ数', lo=1, hi=200))
        J_thr = g.parse_float(self.basic.get('J_thr'), 'J の閾値', lo=0.0)
        base = self.basic.get('base').strip()
        if not base:
            raise ValueError("保存先フォルダを指定してください")
        os.makedirs(base, exist_ok=True)
        L, sens, jit = g.read_timing(self.timing)
        initial, target = self.table.get()
        opt = c.opt_dofs_from(initial, target)
        if not opt:
            raise ValueError("目標値が初期値と違う自由度がありません")
        bounds, idw = g.read_bounds(self.bounds)
        return {'T': T, 'wait': wait, 'max_loops': max_loops, 'J_thr': J_thr, 'base': base, 'L': L, 'sens_L': sens,
                'jitter': jit, 'initial': initial, 'target': target, 'opt_dofs': opt,
                'weights': g.read_weights(self.weights), 'weights_source': self.weights_source,
                'eps': g.parse_float(self.elf.get('eps'), 'ε', lo=0.0),
                'el_max_iter': int(g.parse_float(self.elf.get('el_max_iter'), '最大反復', lo=1, hi=10000)),
                'n_starts': int(g.parse_float(self.elf.get('n_starts'), '開始点の数', lo=1, hi=20)),
                'grid_n': int(g.parse_float(self.elf.get('grid_n'), '格子', lo=4, hi=80)),
                'bounds': bounds, 'id_weights': idw}

    def save_settings(self, cfg=None):
        try:
            cfg = cfg or self.collect()
        except ValueError as exc:
            messagebox.showerror("Settings", str(exc))
            return
        data = {k: cfg[k] for k in ('T', 'wait', 'max_loops', 'J_thr', 'base', 'L', 'jitter', 'initial', 'target',
                                    'eps', 'el_max_iter', 'n_starts', 'grid_n', 'id_weights')}
        data['jitter'] = cfg['jitter'] * 1e3
        data['sens_L'] = ", ".join(f"{v:g}" for v in cfg['sens_L'])
        data['weights'] = cfg['weights'].as_dict()
        data['weights_source'] = cfg['weights_source']
        data['bounds'] = {k: list(v) for k, v in cfg['bounds'].items()}
        g.save_json(SETTINGS_FILE, data)

    def start(self):
        if self.worker is not None and self.worker.is_alive():
            return
        if self.link is None:
            messagebox.showerror("ROS2", "ROS2 に接続していないので開始できません")
            return
        try:
            cfg = self.collect()
        except ValueError as exc:
            messagebox.showerror("Settings", str(exc))
            return
        recv = self.link.receiving()
        prm = self.link.adrc_params()
        missing = [d for d in cfg['opt_dofs'] if prm.get(d) is None]
        if missing:
            messagebox.showerror("ADRC", f"実機から ADRC パラメータを受信できていない最適化対象があります: DOF {missing}\n"
                                         f"（ボードの受信状況: {recv}）")
            return
        cur = self.link.board_targets(max_age=2.0)
        far = [d for d in range(c.N_DOF) if cur[d] is not None and abs(cur[d] - cfg['initial'][d]) > 50]
        moves = "\n".join(f"  DOF {d}: {cfg['initial'][d]:g} → {cfg['target'][d]:g}" for d in cfg['opt_dofs'])
        warn = (f"\n\n注意: 今の目標値と初期値が 50 以上違う自由度があります（最初に初期値へ移動します）: DOF {far}"
                if far else "")
        if not messagebox.askokcancel("Start", f"ロボットが動きます。\n\n最適化対象:\n{moves}\n\n"
                                               f"T = {cfg['T']} s, 最大ループ数 {cfg['max_loops']}{warn}\n\n開始しますか？"):
            return
        self.save_settings(cfg)
        self.stop_event.clear()
        run = OptimizationRun(self.link, cfg, self.log, self.stop_event)
        self.worker = threading.Thread(target=run.run, daemon=True)
        self.worker.start()
        self.btn_start.configure(state='disabled')
        self.btn_stop.configure(state='normal')
        self.status.configure(text="実行中")

    def request_stop(self):
        self.stop_event.set()
        self.status.configure(text="停止中（FF=0 → 初期姿勢 → 保存）…")
        self.btn_stop.configure(state='disabled')

    def tick(self):
        self.logpane.poll()
        if self.worker is not None and not self.worker.is_alive():
            self.worker = None
            self.btn_start.configure(state='normal')
            self.btn_stop.configure(state='disabled')
            self.status.configure(text="待機中（次の試行を開始できます）")
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
