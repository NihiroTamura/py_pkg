#!/usr/bin/env python3
"""test_code7.py の評価関数の重みを自動チューニングするプログラム

test_code7.py は離散時間オイラー・ラグランジュ法のコスト
    J = Σ e_k^T Q_k e_k + R Σ u_k² + R_du Σ (Δu_k)²
の重みを手動で調整している。欲しいFF（実機で意味を持つ大きさの山と谷を持つ5次関数）を
手作業で見つけるのは難しいため、本プログラムは重みを Optuna で自動探索する。

【ロボットは1回しか動かさない】
  1. 初期姿勢へ移動 → 励振用の初期FFを印加して SIM_TIME 秒間の実測データを1回だけ取得する
  2. その実測データから目標モデル・システムモデル・u_hold・z3(0⁻) を同定／取得し、
     同定したシステムモデルを以降ずっと「真のモデル」とみなす
  3. 以降はロボットを一切動かさず、すべてシミュレーションのみで重みを探索する
     （同定結果はJSONへ保存されるので、モード2で読み込めば再探索にロボットは不要）

【必須条件】近似後FFの2つの極値が、それぞれ指定した範囲に入っていること
    ① 5次近似が極値条件を満たしている（(0,T)内に極値ちょうど2個。退避形に落ちていない）
    ② y1 ∈ [Y1_LO, Y1_HI]     正の極大値
    ③ y2 ∈ [Y2_LO, Y2_HI]     負の極小値
  test_code4.py は自由入力列 u_opt の max/min に対する「または」条件を課していたが、
  これだと min が帯に入っていれば max がいくら大きくても条件を満たしてしまう
  （例: min=-20 で条件成立、しかし max=249）。本プログラムは実機へ送る5次関数の
  2つの極値それぞれに独立の範囲を「かつ」で課すので、出荷するものがそのまま評価対象になる。

【目的関数】重みに依存しない指標であること
  探索している重みを目的関数に使ってはいけない（J は Q_k, R, R_du の関数なのでトライアル間で
  比較できない）。5次近似後のFFを与えたときの応答と目標軌道の二乗誤差 Σ(z_sys - z_tgt)² を使う。
  自由入力列 u_FF_opt ではなく近似後のFFで評価する点が重要（実機に送るのはそちらのため）。

【探索変数】7変数（すべて対数スケール）
    Q_post = diag(  1  ,      q_v ,      q_a )          整定区間（k >= k_s）
    Q_pre  = diag( γ_p , γ_v·q_v , γ_a·q_a )            過渡区間（k <  k_s）
    探索: q_v, q_a, γ_p, γ_v, γ_a, R, R_du
  J は (Q,R,R_du) → c·(Q,R,R_du) に対して同じ u_FF を与える（スケール不変性）ので、
  q_p_post ≡ 1 に固定して冗長性を1つ除いてある。γ_* は「過渡区間の重みが整定区間の何倍か」で、
  γ_p < 1 なら過渡区間で位置の追従を緩めること、γ_v/γ_p や γ_a/γ_p が 1 より大きいなら
  過渡区間で位置より速度・加速度（＝振動）を重視することを意味する。要求（過渡は厳密追従より
  滑らかさ、整定は一致を最重要）がこの比にそのまま対応するので、探索結果がそのまま解釈できる。
  成分ごとに独立な比を持たせているのは、γ_v と γ_a を1変数にまとめると任意の
  (Q_pre, Q_post) の組を表現できず、手動値を初回トライアルとして正確に評価できなくなるためである。

得られた重みは test_code7.py の COST_Q_PRE / COST_Q_POST / COST_R / COST_R_DU へ貼り替えて使う。
"""
import os                                                       # ファイルパス操作を行う標準ライブラリ
import sys                                                      # プログラム終了（sys.exit）を扱う標準ライブラリ
import json                                                     # 同定結果（真のモデル）の保存・読み込み
import time                                                     # 時刻の取得
import threading                                                # スレッド処理ライブラリ
import traceback                                                # エラー内容表示するライブラリ

import numpy as np                                              # 数値計算ライブラリ
import pandas as pd                                             # データフレーム操作およびCSVへの保存を行うライブラリ
import optuna                                                   # ハイパーパラメータ自動探索ライブラリ

import rclpy                                                    # ROS2ライブラリ
from rclpy.executors import MultiThreadedExecutor               # トピック購読やタイマーをマルチスレッドで並列実行する実行器

import tkinter as tk                                            # GUIライブラリ
from tkinter import filedialog                                  # GUIでファイルやフォルダを選択するためのモジュール

import py_pkg.test_code7 as tc7                                 # 最適制御本体（ソルバー・ROS2ノード・定数をすべて再利用する）
from py_pkg.test_code7 import (                                 # よく使うものは直接取り込む
    MathematicalSolver, OptimalControlSequencer, OPT_DOF_MASK, OPT_DOF_IDS, N_DOF_ALL,
    ADRC_KP, ADRC_KD, ADRC_INPUT_COEF, ADRC_OBS_POLE,
    CTRL_DT, SIM_TIME, PWM_LIMIT, STEP_MIN, U_MAX_INIT, GAP_WARN_RATIO,
)

# ==============================================================================
# チューニング対象の自由度と必須条件（チューニング要素）
#   TUNE_DOF_IDS : 必須条件を課し、目的関数を評価する自由度番号（1始まり）。
#                  test_code7.py の OPT_DOF_IDS ⊇ TUNE_DOF_IDS でなければならない
#                  （最適化していない自由度をチューニング対象にしても意味がないため、起動時に検証する）。
#   極値の範囲   : 実機へ送る5次関数の 正の極大値 y1 と 負の極小値 y2 それぞれに課す範囲 [PWM]。
#                  自由度ごとに TUNE_BAND_OVERRIDE で上書きできる。
#                  既定値は test_code4.py の実績帯（振幅 10〜30 PWM）を踏襲したもので、
#                  実測同定モデル(DOF4)での内点最適解 (+16.8, -24.4) はこの帯に収まる。
# ==============================================================================
TUNE_DOF_IDS = [4]                              # チューニング対象の自由度番号（1始まり）
TUNE_Y1_BAND = (5.0, 15.0)                     # 正の極大値 y1 の許容範囲 [PWM]
TUNE_Y2_BAND = (-15.0, -5.0)                   # 負の極小値 y2 の許容範囲 [PWM]
TUNE_BAND_OVERRIDE = {}                         # 自由度ごとの上書き {DOF番号: (y1_lo, y1_hi, y2_lo, y2_hi)}

TUNE_U_MAX = PWM_LIMIT                          # 探索中のFF振幅上限 [PWM]（帯で絞るのでトラストリージョンは使わない）
TUNE_PENALTY = 1e12                             # 必須条件を満たさない候補へ与える大ペナルティ（実現しうる最大の目的関数値より十分大きい値）

# ==============================================================================
# 探索の設定（チューニング要素）
#   探索範囲は test_code4.py の実績値を土台にしている。R_du は目安 R_du ≈ ρ·R·N_ff²
#   （N_ff = T/CTRL_DT, ρ=1〜10）を含む広さが必要。
# ==============================================================================
TUNE_N_TRIALS = 200                             # Optunaの探索回数の既定値（実行時に入力で指定できる）
TUNE_SAMPLER = 'tpe'                            # Optunaのサンプラー（'tpe': TPESampler / 'cmaes': CmaEsSampler）
TUNE_SEED = 0                                   # サンプラーの乱数シード（Noneで毎回変化）
TUNE_QV_RANGE = (1e-3, 1e6)                     # 探索範囲 q_v（整定区間の速度重み。対数スケール）
TUNE_QA_RANGE = (1e-4, 1e4)                     # 探索範囲 q_a（整定区間の加速度重み。対数スケール）
TUNE_GP_RANGE = (1e-5, 1.0)                     # 探索範囲 γ_p（過渡区間の位置重み / 整定区間の位置重み。1以下で追従を緩める）
TUNE_GV_RANGE = (1e-5, 1e2)                     # 探索範囲 γ_v（過渡区間の速度重み / 整定区間の速度重み）
TUNE_GA_RANGE = (1e-5, 1e2)                     # 探索範囲 γ_a（過渡区間の加速度重み / 整定区間の加速度重み）
TUNE_R_RANGE = (1e-6, 1e3)                      # 探索範囲 R（FF入力の大きさへの重み。対数スケール）
TUNE_R_DU_RANGE = (1e-4, 1e10)                  # 探索範囲 R_du（FF入力の変化量への重み。対数スケール）
TUNE_SEED_MANUAL = True                         # Trueなら test_code7.py の手動値を初回トライアルとして必ず評価し、比較対象にする

TUNE_DOF_IDX = sorted({int(v) - 1 for v in TUNE_DOF_IDS})                                       # 0始まりの添字へ変換（重複は除く）
if not TUNE_DOF_IDX or TUNE_DOF_IDX[0] < 0 or TUNE_DOF_IDX[-1] >= N_DOF_ALL:                    # 書き間違いに気付かないまま実験するのを防ぐ
    raise ValueError(f"TUNE_DOF_IDS は 1～{N_DOF_ALL} の自由度番号を1つ以上指定してください: {TUNE_DOF_IDS!r}")
if not all(OPT_DOF_MASK[d] for d in TUNE_DOF_IDX):                                              # 最適化していない自由度は探索しても意味がない
    raise ValueError(
        f"TUNE_DOF_IDS {TUNE_DOF_IDS} は test_code7.py の OPT_DOF_IDS {OPT_DOF_IDS} に"
        f"含まれている必要があります（最適化対象外の自由度はFFが常に0のため）"
    )


# ==============================================================================
# 重みの構成（探索変数 → Q_pre, Q_post, R, R_du）
# ==============================================================================

# 7個の探索変数から重み行列を組み立てる関数
def build_weights(q_v, q_a, g_p, g_v, g_a, R, R_du):                                           # 引数(整定区間の速度重み, 整定区間の加速度重み, 過渡の位置比, 過渡の速度比, 過渡の加速度比, 入力重み, 入力レート重み)
    """スケール不変性を使って q_p_post ≡ 1 に固定した、解釈しやすい重み構成。

        Q_post = diag(  1  ,      q_v ,      q_a )          整定区間（k >= k_s）
        Q_pre  = diag( γ_p , γ_v·q_v , γ_a·q_a )            過渡区間（k <  k_s）

    γ_* は「過渡区間の重みが整定区間の何倍か」である。γ_p < 1 なら過渡区間で位置の追従を緩め、
    γ_v/γ_p や γ_a/γ_p が 1 より大きいなら過渡区間で位置より速度・加速度（＝振動）を重視する。
    要求「過渡は厳密追従より滑らかさ、整定は一致を最重要」がこの比にそのまま対応する。
    成分ごとに独立な比にしてあるのは、1変数にまとめると任意の (Q_pre, Q_post) を表現できず、
    手動値を初回トライアルとして正確に評価できなくなるためである。
    """
    Q_post = np.diag([1.0, float(q_v), float(q_a)])                                             # 整定区間の状態重み
    Q_pre = np.diag([float(g_p), float(g_v) * float(q_v), float(g_a) * float(q_a)])              # 過渡区間の状態重み
    return Q_pre, Q_post, np.array([[float(R)]]), float(R_du)                                   # ソルバーへ渡す形で返す


# test_code7.py の手動値を探索変数の形へ戻す関数
def manual_params():                                                                            # 引数なし（test_code7.py の定数を読むだけ）
    """TUNE_SEED_MANUAL=True のときに初回トライアルとして評価する、手動調整済みの重み。

    test_code7.py の COST_Q_PRE / COST_Q_POST / COST_R / COST_R_DU を、
    q_p_post ≡ 1 へ正規化してから7変数へ分解する。7変数は任意の (Q_pre, Q_post) を
    表現できるので、この分解は厳密であり、build_weights() で元の重みへ戻る。
    """
    qpre = np.diag(tc7.COST_Q_PRE).astype(float)                                                # 過渡区間の重み [位置, 速度, 加速度]
    qpost = np.diag(tc7.COST_Q_POST).astype(float)                                              # 整定区間の重み [位置, 速度, 加速度]
    s = qpost[0] if qpost[0] > 0 else 1.0                                                       # q_p_post = 1 にするための正規化係数
    return {
        'q_v': float(qpost[1] / s),                                                             # 整定区間の速度重み
        'q_a': float(qpost[2] / s),                                                             # 整定区間の加速度重み
        'g_p': float(qpre[0] / s),                                                              # 過渡区間の位置重み / 整定区間の位置重み
        'g_v': float(qpre[1] / qpost[1]) if qpost[1] > 0 else 1.0,                              # 過渡区間の速度重み / 整定区間の速度重み
        'g_a': float(qpre[2] / qpost[2]) if qpost[2] > 0 else 1.0,                              # 過渡区間の加速度重み / 整定区間の加速度重み
        'R': float(tc7.COST_R[0, 0] / s),                                                       # FF入力の大きさへの重み
        'R_du': float(tc7.COST_R_DU / s),                                                       # FF入力の変化量への重み
    }


# ==============================================================================
# 実測データを1回だけ取得して同定結果をJSONへ保存するROS2ノード
# ==============================================================================
class ModelCaptureSequencer(OptimalControlSequencer):
    """test_code7.py のノードをそのまま使い、最適化パイプラインだけ「同定して保存」に差し替える。

    通信仕様・状態機械・データ収集・欠測判定・保持PWM/z3の取得はすべて test_code7.py と同一である
    （継承しているので実装も共有される）。ロボットを動かすのはこのノードの1周だけ。
    """

    # コンストラクタ
    def __init__(self, json_path, T):                                                           # 引数(同定結果の保存先JSON, FF制御入力時間)
        super().__init__(csv_path=os.devnull, T=T, max_iter=1, target_mode="1")                  # 親のノードを1外側ループ・プリセット目標値で生成
        self.json_path = json_path                                                              # 同定結果の保存先
        self.models = None                                                                      # 同定した「真のモデル」（24自由度分）
        self.capture_ok = False                                                                 # 同定が成功したか

    # 実測データから「真のモデル」を同定してJSONへ保存する関数（親の最適化パイプラインを置き換える）
    def dispatch_optimization_pipeline(self):                                                   # 引数なし（親クラスのスレッド関数を差し替える）
        try:
            dof_map = self.build_dof_map()                                                      # 24自由度 → 26要素配列インデックスの対応表
            solver = MathematicalSolver(self.T, self.dt)                                        # 数学ソルバーの生成
            M, t_grid = solver.M, solver.t_eval                                                 # 評価ホライズンのサンプル数と時間軸

            # 親と同じ手順で、受信バッファを24自由度分の時間軸へ揃える
            pwm_raw, _ = self._build_24(self.buffer_pwm, "PWM", t_grid, quiet=True)             # 遅れ推定用のPWM
            kick_delay = []                                                                     # 判定できた自由度の通信遅れ
            for d_i in TUNE_DOF_IDX:                                                            # チューニング対象の自由度だけ判定する
                u_d, hold_d = pwm_raw[d_i], (self.hold_pwm_24[d_i] if self.hold_pwm_24 else None)
                if u_d is None or hold_d is None:                                               # PWMか保持PWMが取れなかった場合
                    continue
                step_d = self.target_pot[dof_map[d_i]] - self.initial_pot[dof_map[d_i]]         # 目標変位 Pf - Pi
                lag_d, _ = solver.detect_adrc_kick(u_d, hold_d, ADRC_KP[d_i], ADRC_INPUT_COEF[d_i], step_d)
                if np.isfinite(lag_d):                                                          # 判定できた場合
                    kick_delay.append(lag_d)
            lag = int(round(float(np.median(kick_delay)))) if kick_delay else 0                 # 通信遅れは中央値を採用
            self.get_logger().info(f"通信遅れ δ = {lag} サンプル ({lag * self.dt * 1000:.0f} ms)")

            t_shift = t_grid + lag * self.dt                                                    # 前詰めした時間軸
            pot_24, gap_24 = self._build_24(self.buffer_pot, "POT", t_shift)                    # 実測POT値を24自由度分へ変換
            pwm_24, _ = self._build_24(self.buffer_pwm, "PWM", t_shift)                         # 実制御入力（PWM）を24自由度分へ変換

            models = []                                                                         # 24自由度分の同定結果
            for dof_idx in range(N_DOF_ALL):                                                    # 24自由度ループ
                Pi = self.initial_pot[dof_map[dof_idx]]                                         # 初期位置
                Pf = self.target_pot[dof_map[dof_idx]]                                          # 目標位置
                y0, step = Pi - Pf, Pf - Pi                                                     # 初期偏差・目標変位
                entry = {'dof': dof_idx + 1, 'Pi': float(Pi), 'Pf': float(Pf), 'y0': float(y0)}  # この自由度の記録

                if dof_idx not in TUNE_DOF_IDX or pot_24[dof_idx] is None or abs(step) < STEP_MIN:  # 対象外・未受信・変位が小さい場合
                    entry['valid'] = False                                                      # 探索に使えない自由度として記録
                    models.append(entry)                                                        # 使えない自由度も枠は残す（24自由度の並びを保つ）
                    continue

                raw_y = pot_24[dof_idx]                                                         # M点に揃え済みの実測データ
                y_shifted, z_data = raw_y - Pf, raw_y - Pi                                      # 誤差系・z形式の実測データ
                a, b, c, d, _ = self.current_ff_matrix[dof_idx]                                 # 印加した初期励振FFのパラメータ
                u_ff = np.zeros(M)                                                              # 印加したFF入力の時系列
                u_ff[:solver.N_ff] = solver.poly(t_grid[:solver.N_ff], a, b, c, d)              # FF入力区間だけ5次関数の値を格納

                u_pwm = pwm_24[dof_idx]                                                         # 実測PWM
                if u_pwm is None:                                                               # PWMを受信できなかった場合
                    self.get_logger().warn(f"DOF {dof_idx + 1:02d}: PWMを受信できませんでした。FF入力で代用します。")
                    u_pwm, u_hold = u_ff.copy(), 0.0                                            # FF入力で代用（基準値も0）
                else:                                                                           # PWMを受信できた場合
                    u_hold = self.hold_pwm_24[dof_idx] if self.hold_pwm_24 else float(u_pwm[0])  # 目標値切替直前の保持PWM
                u_ident = u_pwm - u_hold                                                        # 同定に使用する入力（保持分を除去）
                z3_init = self.hold_z3_24[dof_idx] if self.hold_z3_24 else None                 # ζ3(0) に使う実測 z3(0⁻)
                adrc = (ADRC_KP[dof_idx], ADRC_KD[dof_idx], ADRC_INPUT_COEF[dof_idx], ADRC_OBS_POLE[dof_idx])  # ADRCパラメータ

                tgt = solver.fit_target_model(y_shifted, y0)                                    # 目標モデルの同定
                sysp = solver.fit_system_model(z_data, u_ident, 0.0)                            # システムモデルの開ループ同定
                cl_skip = (pwm_24[dof_idx] is None) or (gap_24[dof_idx].mean() > GAP_WARN_RATIO)  # 仕上げを行えない自由度か判定
                if not cl_skip:                                                                 # 仕上げを行える場合
                    u_adrc_meas = u_pwm - u_ff                                                  # 実測ADRC出力（絶対PWM）
                    sysp, cl_ok, _, _ = solver.refine_system_model_closed_loop(                 # 閉ループで再フィット
                        sysp, adrc, step, u_hold, z_data, u_adrc_meas, u_ff, z3_init)
                else:
                    cl_ok = False                                                               # 仕上げなし

                entry.update({                                                                  # 同定結果を記録する
                    'valid': True,                                                              # 探索に使える自由度
                    'tgt_params': [float(v) for v in tgt],                                      # 目標モデル [T1, wn]
                    'sys_params': [float(v) for v in sysp],                                     # システムモデル [T1, zeta, wn, b0]
                    'u_hold': float(u_hold),                                                    # 初期姿勢の保持PWM
                    'z3_init': (float(z3_init) if z3_init is not None else None),                # ζ3(0) に使う実測 z3(0⁻)
                    'cl_ok': bool(cl_ok),                                                       # 閉ループ同定の仕上げを採用したか
                    'y_data': [float(v) for v in raw_y],                                        # 実測POT値（参考用）
                    'u_ident': [float(v) for v in u_ident],                                     # 同定入力（参考用）
                })
                self.get_logger().info(
                    f"DOF {dof_idx + 1:02d} 同定完了: tgt=[{tgt[0]:.4f}, {tgt[1]:.4f}]  "
                    f"sys=[{sysp[0]:.5f}, {sysp[1]:+.3f}, {sysp[2]:.3f}, {sysp[3]:.1f}]  "
                    f"u_hold={u_hold:.2f}  z3(0⁻)={entry['z3_init']}  仕上げ={'採用' if cl_ok else '不採用'}"
                )
                models.append(entry)                                                            # この自由度の同定結果を積む

            self.models = models                                                                # 同定結果を保持する
            save_identified_models(self.json_path, self.T, self.dt, self.initial_pot, self.target_pot, models)  # JSONへ保存
            self.get_logger().info(f"同定結果を保存しました: {self.json_path}")
            self.capture_ok = any(m.get('valid') for m in models)                               # 1つでも有効な自由度があれば成功
            if not self.capture_ok:                                                             # 有効な自由度が1つも無い場合
                self.get_logger().error("探索に使える自由度がありません（目標変位が小さすぎるか、受信できていません）")

        except Exception as exc:                                                                # 何かエラーが出たときの処理
            self.get_logger().error(f"同定パイプライン異常: {exc}\n{traceback.format_exc()}")
        finally:
            self.state = "FINISHED"                                                             # ロボットは1回しか動かさないので必ず終了する
            self.state_start_time = self.get_clock().now()

    # 受信バッファを24自由度分の時間軸へ揃える関数（親のパイプライン内のローカル関数を切り出したもの）
    def _build_24(self, buffer, label, grid, quiet=False):                                      # 引数(受信バッファ, ログ表示名, 揃える時間軸, 警告を出さないか)
        out, gaps = [], []                                                                      # 24自由度分の値と欠測マスクの空リスト
        for b_id in range(1, 6):                                                                # board1からboard5の順に処理
            samples = list(buffer[f'board{b_id}'])                                              # 各boardに対応する受信データ（受信時刻, 値）
            n_dof = 3 if b_id in [4, 5] else 6                                                  # board4と5は3自由度、それ以外は6自由度
            if not samples:                                                                     # 1点も受信できなかった場合
                out.extend([None] * n_dof); gaps.extend([None] * n_dof)                         # 未受信として格納
                if not quiet:
                    self.get_logger().warn(f"{label}-board{b_id}: 1点も受信できませんでした")
                continue
            stamps = np.array([s[0] for s in samples], dtype=float)                             # 受信時刻の配列
            arr = np.array([s[1] for s in samples], dtype=float)                                # 受信値の配列
            gap = self.gap_mask(stamps, grid)                                                   # 欠測マスク
            if not quiet:
                self.report_gaps(label, b_id, stamps, gap, grid)                                # 欠測があれば警告ログを出す
            for j in range(n_dof):                                                              # そのboardの自由度の順に処理
                out.append(self.resample_by_time(stamps, arr[:, j], grid))                      # 受信時刻で補間して格納
                gaps.append(gap)
        return out, gaps                                                                        # 24自由度分の値と欠測マスクを返す


# ==============================================================================
# 同定結果（真のモデル）の保存・読み込み
# ==============================================================================

# 同定結果をJSONへ保存する関数
def save_identified_models(path, T, dt, initial_pot, target_pot, models):                       # 引数(保存先, FF制御入力時間, 刻み, 初期姿勢, 目標値, 24自由度分の同定結果)
    """モード2で読み込めば、ロボットを動かさずに再探索できる"""
    payload = {
        'T': float(T), 'dt': float(dt), 'sim_time': float(SIM_TIME),                            # FF制御入力時間・刻み・評価時間
        'ctrl_dt': float(CTRL_DT), 'eso_dt': float(tc7.ESO_DT),                                 # 実機の制御周期・ESOの式の中の係数（再現性のため残す）
        'initial_pot': [float(v) for v in initial_pot],                                         # 初期姿勢（26要素）
        'target_pot': [float(v) for v in target_pot],                                           # 目標値（26要素）
        'tune_dof': list(TUNE_DOF_IDS),                                                         # このとき同定したチューニング対象の自由度
        'time': time.strftime('%Y-%m-%d %H:%M:%S'),                                             # 同定した日時
        'models': models,                                                                       # 24自由度分の同定結果
    }
    with open(path, 'w', encoding='utf-8') as fp:                                               # JSONへ書き出す
        json.dump(payload, fp, ensure_ascii=False, indent=1)                                    # 日本語をそのまま残して書き出す


# 同定結果をJSONから読み込む関数
def load_identified_models(path):                                                               # 引数(読み込むJSONのパス)
    """保存済みの「真のモデル」を読み込む（ロボットは動かさない）"""
    with open(path, 'r', encoding='utf-8') as fp:                                               # JSONを読み込む
        payload = json.load(fp)                                                                 # 辞書へ展開する
    if abs(float(payload.get('ctrl_dt', CTRL_DT)) - CTRL_DT) > 1e-12:                           # 刻みが違うと同定結果をそのまま使えない
        raise ValueError(
            f"JSONの ctrl_dt={payload.get('ctrl_dt')} が現在の CTRL_DT={CTRL_DT} と一致しません。"
            f"同じ時間軸で同定し直してください"
        )
    return payload                                                                              # 読み込んだ辞書を返す


# ==============================================================================
# 重みの自動探索
# ==============================================================================
class WeightTuner:
    # コンストラクタ
    def __init__(self, payload):                                                                # 引数(load_identified_models が返した辞書)
        self.T = float(payload['T'])                                                            # FF制御入力時間
        self.dt = float(payload.get('ctrl_dt', CTRL_DT))                                        # 1ステップの刻み
        self.models = payload['models']                                                         # 24自由度分の同定結果
        self.dofs = [d for d in TUNE_DOF_IDX if self.models[d].get('valid')]                    # 探索に使える自由度
        if not self.dofs:                                                                       # 1つも使えない場合
            raise ValueError("JSONに探索へ使える自由度がありません（valid な自由度が1つもない）")
        self.best = None                                                                        # 最良トライアルの記録
        self.trials = []                                                                        # 全トライアルの記録

    # 自由度ごとの極値の許容範囲を返す関数
    @staticmethod
    def band_of(dof_idx):                                                                       # 引数(自由度の添字, 0始まり)
        """既定の帯を、TUNE_BAND_OVERRIDE に指定があればそれで上書きして返す"""
        if (dof_idx + 1) in TUNE_BAND_OVERRIDE:                                                 # 自由度ごとの上書きがある場合
            y1_lo, y1_hi, y2_lo, y2_hi = TUNE_BAND_OVERRIDE[dof_idx + 1]
            return (float(y1_lo), float(y1_hi)), (float(y2_lo), float(y2_hi))
        return TUNE_Y1_BAND, TUNE_Y2_BAND                                                       # 既定の帯

    # 値が帯からどれだけ外れているかを返す関数
    @staticmethod
    def band_distance(v, band):                                                                 # 引数(値, 許容範囲(lo, hi))
        """帯の中なら0、外なら最も近い端までの距離。帯から遠いほどペナルティを大きくするために使う"""
        lo, hi = band                                                                           # 許容範囲の下限・上限
        if not np.isfinite(v):                                                                  # 値が異常な場合
            return float(abs(hi - lo)) * 10.0                                                   # 大きな距離として扱う
        return float(max(lo - v, 0.0) + max(v - hi, 0.0))                                       # 下に外れた分 + 上に外れた分

    # 1候補（6変数の重み）を評価する関数
    def evaluate(self, q_v, q_a, g_p, g_v, g_a, R, R_du):                                       # 引数(7個の探索変数)
        """重みを与えて全対象自由度でEL最適制御＋5次近似を回し、目的関数と必須条件を計算する。

        目的関数は「5次近似後のFFを与えたときの応答」と目標軌道の二乗誤差である。
        自由入力列 u_FF_opt ではなく近似後のFFで評価する点が重要（実機に送るのはそちらのため）。
        """
        Q_pre, Q_post, Rm, Rdu = build_weights(q_v, q_a, g_p, g_v, g_a, R, R_du)                # 重み行列を組み立てる
        total_J, violation, per_dof = 0.0, 0.0, []                                              # 目的関数・違反量・自由度ごとの記録
        for dof_idx in self.dofs:                                                               # 対象自由度のループ
            m = self.models[dof_idx]                                                            # その自由度の同定結果
            y0, Pi, Pf = m['y0'], m['Pi'], m['Pf']                                              # 初期偏差・初期位置・目標位置
            step = Pf - Pi                                                                      # 目標変位
            adrc = (ADRC_KP[dof_idx], ADRC_KD[dof_idx], ADRC_INPUT_COEF[dof_idx], ADRC_OBS_POLE[dof_idx])  # ADRCパラメータ
            solver = MathematicalSolver(self.T, self.dt, Q_pre, Q_post, Rm, Rdu)                # この重みでソルバーを作る
            try:
                (ff, extrema, u_pred, _, _, u_ff_opt, info) = solver.calculate_el_ff(           # EL最適制御 + 5次近似
                    m['tgt_params'], m['sys_params'], y0, adrc, m['u_hold'],
                    m.get('z3_init'), TUNE_U_MAX)
                _, y_tgt = solver.build_target_traj(m['tgt_params'], y0, step)                  # 誤差系の目標出力
                z_pred, _, _ = solver.simulate_closed_loop(                                     # 5次近似後のFFを与えたときの応答
                    m['sys_params'], adrc, step, m['u_hold'], u_pred, m.get('z3_init'))
                J_ff = solver.compute_J(y_tgt, z_pred + Pi - Pf)                                # 重み非依存の目的関数
            except Exception:                                                                   # 数値的に破綻した候補
                return {'score': TUNE_PENALTY * 100.0, 'J': float('inf'), 'violation': float('inf'), 'per_dof': []}

            t1, y1, t2, y2 = [float(v) for v in extrema]                                        # 近似後FFの極値
            b1, b2 = self.band_of(dof_idx)                                                      # この自由度の許容範囲
            v_dof = self.band_distance(y1, b1) + self.band_distance(y2, b2)                     # 帯からの外れ量
            if info['fit_mode'] not in (0, 1) or y1 <= 0.0 or y2 >= 0.0:                        # 極値条件を満たしていない・山谷になっていない
                v_dof += abs(b1[1] - b1[0]) + abs(b2[1] - b2[0])                                # 帯幅ぶんの追加ペナルティ
            total_J += J_ff                                                                     # 目的関数へ加算
            violation += v_dof                                                                  # 違反量へ加算
            per_dof.append({                                                                    # 自由度ごとの記録（CSV保存と表示に使う）
                'dof': dof_idx + 1, 'J': float(J_ff), 'violation': float(v_dof),
                't1': t1, 'y1': y1, 't2': t2, 'y2': y2,
                'fit_mode': int(info['fit_mode']), 'cl_radius': float(info['cl_radius']),
                'k_s': int(info['k_s']), 'ff': [float(v) for v in ff],
                'u_ff_opt_max': float(np.max(u_ff_opt)), 'u_ff_opt_min': float(np.min(u_ff_opt)),
            })

        scale = max(abs(TUNE_Y1_BAND[1]), abs(TUNE_Y2_BAND[0]), 1.0)                            # ペナルティの正規化スケール
        score = total_J if violation <= 0.0 else TUNE_PENALTY * (1.0 + violation / scale)       # 必須条件を満たさない候補は必ず劣後させる
        return {'score': float(score), 'J': float(total_J), 'violation': float(violation), 'per_dof': per_dof}

    # Optunaの目的関数
    def objective(self, trial):                                                                 # 引数(Optunaのトライアル)
        q_v = trial.suggest_float('q_v', *TUNE_QV_RANGE, log=True)                              # 整定区間の速度重み
        q_a = trial.suggest_float('q_a', *TUNE_QA_RANGE, log=True)                              # 整定区間の加速度重み
        g_p = trial.suggest_float('g_p', *TUNE_GP_RANGE, log=True)                              # 過渡区間の位置重み / 整定区間の位置重み
        g_v = trial.suggest_float('g_v', *TUNE_GV_RANGE, log=True)                               # 過渡区間の速度重み / 整定区間の速度重み
        g_a = trial.suggest_float('g_a', *TUNE_GA_RANGE, log=True)                               # 過渡区間の加速度重み / 整定区間の加速度重み
        R = trial.suggest_float('R', *TUNE_R_RANGE, log=True)                                   # FF入力の大きさへの重み
        R_du = trial.suggest_float('R_du', *TUNE_R_DU_RANGE, log=True)                          # FF入力の変化量への重み

        res = self.evaluate(q_v, q_a, g_p, g_v, g_a, R, R_du)                                   # 候補を評価する
        trial.set_user_attr('J', res['J'])                                                      # 目的関数値（必須条件を無視した素の値）
        trial.set_user_attr('violation', res['violation'])                                      # 帯からの外れ量
        if res['per_dof']:                                                                      # 代表として先頭の自由度の極値を記録する
            p = res['per_dof'][0]                                                               # 先頭の自由度の結果
            for k in ('t1', 'y1', 't2', 'y2', 'fit_mode', 'cl_radius'):                         # 極値とモデルの状態を記録する
                trial.set_user_attr(k, p[k])                                                    # Optunaのトライアルへ属性として残す

        if self.best is None or res['score'] < self.best['score']:                              # 最良トライアルを更新する
            self.best = dict(res, params={'q_v': q_v, 'q_a': q_a, 'g_p': g_p, 'g_v': g_v,
                                          'g_a': g_a, 'R': R, 'R_du': R_du},
                             number=trial.number)
        self.trials.append({                                                                    # 全トライアルを記録する
            'number': trial.number, 'score': res['score'], 'J': res['J'], 'violation': res['violation'],
            'q_v': q_v, 'q_a': q_a, 'g_p': g_p, 'g_v': g_v, 'g_a': g_a, 'R': R, 'R_du': R_du,
            **({f"dof{p['dof']}_{k}": p[k] for p in res['per_dof']
                for k in ('J', 'violation', 't1', 'y1', 't2', 'y2', 'fit_mode', 'cl_radius')}),
        })
        return res['score']                                                                     # Optunaが最小化する値

    # 探索を実行する関数
    def run(self, n_trials):                                                                    # 引数(探索回数)
        sampler = (optuna.samplers.CmaEsSampler(seed=TUNE_SEED) if TUNE_SAMPLER == 'cmaes'      # サンプラーの選択
                   else optuna.samplers.TPESampler(seed=TUNE_SEED))
        optuna.logging.set_verbosity(optuna.logging.WARNING)                                    # トライアルごとのログを抑える
        study = optuna.create_study(direction='minimize', sampler=sampler)                      # 最小化の探索を作る
        if TUNE_SEED_MANUAL:                                                                    # 手動値を初回トライアルとして必ず評価する
            study.enqueue_trial(manual_params())
        print(f"探索開始: {n_trials} トライアル / 対象DOF={[d + 1 for d in self.dofs]} "
              f"/ y1∈{TUNE_Y1_BAND} y2∈{TUNE_Y2_BAND}")
        study.optimize(self.objective, n_trials=n_trials, show_progress_bar=True)               # 探索を実行する
        return study                                                                            # 探索結果（Optunaのstudy）を返す

    # 最良の重みをCSVへ保存する関数
    def save_best_to_csv(self, path):                                                           # 引数(保存先パス)
        if self.best is None:                                                                   # 最良トライアルが無い場合
            return
        b = self.best                                                                           # 最良トライアル
        row = {
            'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'trial': b['number'],                   # 保存日時・トライアル番号
            'T': self.T, 'ctrl_dt': self.dt, 'tune_dof': str([d + 1 for d in self.dofs]),       # 条件
            'y1_lo': TUNE_Y1_BAND[0], 'y1_hi': TUNE_Y1_BAND[1],                                 # 極大値の許容範囲
            'y2_lo': TUNE_Y2_BAND[0], 'y2_hi': TUNE_Y2_BAND[1],                                 # 極小値の許容範囲
            **b['params'],                                                                      # 最良の重み（6変数）
            'score': b['score'], 'J': b['J'], 'violation': b['violation'],                      # 評価値
        }
        for p in b['per_dof']:                                                                  # 自由度ごとの結果も1行に横並びする
            d = p['dof']                                                                        # 自由度番号（1始まり）
            for k in ('J', 't1', 'y1', 't2', 'y2', 'fit_mode', 'cl_radius', 'k_s'):             # 自由度ごとのスカラー値
                row[f'dof{d}_{k}'] = p[k]                                                       # DOF番号を付けた列名で1行に横並びする
            for j, name in enumerate('abcde'):                                                  # 5次関数の係数
                row[f'dof{d}_{name}'] = p['ff'][j]                                              # 実機へ送る係数もCSVへ残す
        pd.DataFrame([row]).to_csv(path, mode='a', header=not os.path.exists(path), index=False)  # CSVへ追記
        print(f"最良の重みを保存しました: {path}")

    # 全トライアルをCSVへ保存する関数
    def save_trials_to_csv(self, path):                                                         # 引数(保存先パス)
        if not self.trials:                                                                     # トライアルが無い場合
            return
        pd.DataFrame(self.trials).to_csv(path, index=False)                                     # CSVへ保存
        print(f"全トライアルを保存しました: {path}")

    # 最良結果を表示する関数
    def print_best(self):                                                                       # 引数なし（self.best を表示する）
        if self.best is None:                                                                   # 最良トライアルが無い場合
            print("有効なトライアルがありませんでした。")
            return
        b = self.best; p = b['params']                                                          # 最良トライアルとその重み
        feasible = b['violation'] <= 0.0                                                        # 必須条件を満たしているか
        verdict = "必須条件を満たしています" if feasible else f"必須条件 違反量={b['violation']:.4g}"    # 判定の文言
        print("\n" + "=" * 78)
        print(f"  最良トライアル #{b['number']}   目的関数 J = {b['J']:.6g}   {verdict}")
        print("=" * 78)
        print("  test_code7.py へ貼り替える値:")
        s_qv, s_qa = p['q_v'], p['q_a']                                                         # 整定区間の速度・加速度重み
        print(f"    COST_Q_PRE  = np.diag([{p['g_p']:.6g}, {p['g_v'] * s_qv:.6g}, {p['g_a'] * s_qa:.6g}])")
        print(f"    COST_Q_POST = np.diag([1.0, {s_qv:.6g}, {s_qa:.6g}])")
        print(f"    COST_R      = np.array([[{p['R']:.6g}]])")
        print(f"    COST_R_DU   = {p['R_du']:.6g}")
        print("\n  自由度ごとの結果:")
        for q in b['per_dof']:
            print(f"    DOF {q['dof']:02d}: J={q['J']:.6g}  極値 ({q['t1']:.3f}, {q['y1']:+.2f}) / "
                  f"({q['t2']:.3f}, {q['y2']:+.2f})  ρ(A_cl)={q['cl_radius']:.5f}  "
                  f"k_s={q['k_s']}  fit={q['fit_mode']}  違反量={q['violation']:.4g}")
        print("=" * 78 + "\n")


# ==============================================================================
# 起動処理
# ==============================================================================

# モード1: ロボットを1回だけ動かして同定結果をJSONへ保存する関数
def capture_models_from_robot(json_path, T, args=None):                                         # 引数(保存先JSON, FF制御入力時間, ROS2引数)
    """test_code7.py と同じ通信・データ収集で実測データを1回だけ取得し、同定してJSONへ保存する"""
    rclpy.init(args=args)                                                                       # ROS2の初期化
    node = ModelCaptureSequencer(json_path, T)                                                  # 同定専用ノードの生成
    executor = MultiThreadedExecutor(num_threads=17)                                            # 購読15 + タイマー1 + 余裕1
    executor.add_node(node)                                                                     # 実行器へノードを登録する
    try:
        while rclpy.ok() and not node.shutdown_event.is_set():                                  # 終了要求が来るまで回し続ける
            executor.spin_once(timeout_sec=0.1)                                                 # 受信とタイマーを1回分処理する
    except KeyboardInterrupt:
        pass
    finally:
        ok = node.capture_ok                                                                    # 同定が成功したか
        executor.shutdown()                                                                     # 実行器を止める
        node.destroy_node()                                                                     # ノードを破棄する
        if rclpy.ok():
            rclpy.shutdown()                                                                    # ROS2を終了する
    return ok                                                                                   # 同定が成功したかを返す


def main(args=None):                                                                            # 引数(ROS2へ渡すコマンドライン引数)
    root = tk.Tk(); root.withdraw()                                                             # Tkのダイアログだけ使う（ウィンドウは表示しない）
    print("【モードを選択してください】")
    print("  1: ロボットを1回動かして同定し、そのモデルで重みを探索する")
    print("  2: 保存済みJSONを読み込んで重みだけ探索する（ロボット不要）")
    mode = input("> ").strip()
    while mode not in ['1', '2']:
        mode = input("無効な入力です。1 または 2 を入力してください: ").strip()

    if mode == '1':                                                                             # ロボットを1回動かす場合
        folder = filedialog.askdirectory(title="同定結果と探索結果を保存するフォルダを選択してください")
        if not folder:
            print("フォルダ未選択のため終了します。")
            sys.exit(1)                                                                         # 続行できないので終了する
        name = input("保存名を入力してください (例: tune0911): ").strip() or time.strftime('%Y%m%d_%H%M%S')  # 未入力なら日時を使う
        base = os.path.join(folder, name)                                                       # 保存ファイルの共通部分
        json_path = base + "_identified_models.json"                                            # 同定結果の保存先
        print("【FF制御時間 T を入力してください】")
        T = float(input("> "))                                                                  # FF制御入力時間
        print("\nロボットを1回だけ動かします。初期姿勢への移動と5秒間のデータ収集を行います。")
        if not capture_models_from_robot(json_path, T, args):                                    # 実測データの取得と同定
            print("同定に失敗しました。終了します。")
            sys.exit(1)                                                                         # 続行できないので終了する
        payload = load_identified_models(json_path)                                              # 保存したJSONを読み直す
    else:                                                                                       # 保存済みJSONを使う場合
        json_path = filedialog.askopenfilename(title="同定結果のJSONを選択してください",
                                               filetypes=[("JSON", "*.json")])
        if not json_path:
            print("ファイル未選択のため終了します。")
            sys.exit(1)                                                                         # 続行できないので終了する
        base = json_path.replace("_identified_models.json", "")                                 # 保存ファイルの共通部分
        payload = load_identified_models(json_path)                                              # 同定結果を読み込む
        print(f"読み込みました: T={payload['T']}s, 同定日時={payload.get('time', '?')}, "
              f"対象DOF={payload.get('tune_dof', '?')}")

    print(f"\n【探索回数を入力してください（既定 {TUNE_N_TRIALS}）】")
    txt = input("> ").strip()                                                                   # 探索回数の入力
    n_trials = int(txt) if txt else TUNE_N_TRIALS                                               # 未入力なら既定値

    tuner = WeightTuner(payload)                                                                # 探索器を生成
    tuner.run(n_trials)                                                                         # 探索を実行
    tuner.print_best()                                                                          # 最良結果を表示
    tuner.save_best_to_csv(base + "_weight_best.csv")                                           # 最良の重みをCSVへ保存
    tuner.save_trials_to_csv(base + "_weight_trials.csv")                                       # 全トライアルをCSVへ保存


if __name__ == '__main__':
    main()
