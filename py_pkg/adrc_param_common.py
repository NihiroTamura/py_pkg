#!/usr/bin/env python3
"""
ADRCパラメータ（kp, kd, input_coef, lambda_0）の送受信・保存の共通部
（GUI_android_tk_ADRCparamChg.py と GUI_android_tk.py で共有する）

<対応Arduinoプログラム> ADRC_ROS2_tk_FF_ADRCparamChange-board1〜5

・トピック（board{k} ごと。Float32MultiArray。i = board内の自由度 0〜5、p = 0:kp, 1:kd, 2:input_coef, 3:lambda_0）
    /board{k}_tk_ADRCparam_float/sub  GUI → Teensy  25要素 [seq, (kp, kd, input_coef, lambda_0) × 6]  要素 1+4i+p
                                       NaN の要素は「変更しない」。seq は1以上の整数（応答の確認に使う）
    /board{k}_tk_ADRCparam_float/pub  Teensy → GUI  50要素（200msごと＋受信を処理した直後）
                                       [0] 最後に処理した seq, [1] 結果（RESULT_TEXT）,
                                       [2+4i+p] 現在の値, [26+4i+p] .ino の初期値（変更できない自由度は NaN）
    /board{k}_tk_z1_float/pub, /board{k}_tk_derivatives_float/pub, /board{k}_tk_z2_float/pub
                                       Teensy → GUI  6要素（並びは /board{k}_tk_z3_float/pub と同じ）
・GUIが「送信した値」と、Teensy が状態トピックで返す「現在設定値」は別に扱う。
  送信後の「設定完了」は、Teensy が送った seq を処理済み（結果=反映）と返し、かつ現在設定値が送信値と一致したときだけ。
・保存ファイル PARAM_FILE（JSON）: GUI_android_tk_ADRCparamChg.py が保存し、GUI_android_tk.py が起動時に読み込む。
    {"version": 1, "saved_at": "YYYY-MM-DDTHH:MM:SS",
     "dofs": [{"dof": 0, "board": "board1", "local": 0, "control_target": true,
               "kp": …, "kd": …, "input_coef": …, "lambda_0": …}, …（26自由度）]}
    control_target が false の自由度（制御対象外）は送信しない。値の範囲の検査も制御対象の自由度だけ行う。
"""
import csv
import json
import math
import os
import random
import threading
import time
from collections import OrderedDict, deque
from datetime import datetime

import numpy as np
from std_msgs.msg import Float32MultiArray

import tkinter as tk
from tkinter import ttk

# ------------------------------
# 自由度とボード・パラメータ
# ------------------------------
# (board名, 先頭DOF, DOF数)。GUI_android_tk.py の BOARD_LAYOUT と同じ
BOARD_LAYOUT = [
    ('board1',  0, 6),
    ('board2',  6, 6),
    ('board3', 12, 6),
    ('board4', 18, 4),
    ('board5', 22, 4),
]
NUM_DOF = 26
SLOTS = 6               # 1ボードのメッセージ上の自由度スロット数（board4・5 も6スロット）

PARAM_KEYS = ('kp', 'kd', 'input_coef', 'lambda_0')     # .ino の変数名は lamda_0
# 受け付ける範囲（.ino の KP_MIN〜LAMDA_0_MAX と同じにすること）
PARAM_LIMITS = {
    'kp': (0.0, 100000.0),
    'kd': (0.0, 10000.0),
    'input_coef': (1000.0, 1000000.0),
    'lambda_0': (1.0, 1000.0),      # ESOは dt=0.001 の前進オイラーなので lambda_0*dt < 2 が安定条件。余裕をみて1000まで
}

# 各 .ino（ADRC_ROS2_tk_FF_ADRCparamChange-board1〜5）の初期値（board4 は「自由度22(腹部前後)のみを動かす場合」の設定）
#   Teensy が状態トピックで初期値を送ってきたときはそちらを使い、この表と違えば警告する
INO_DEFAULTS = {
    'kp': [
        2200.0, 12000.0, 8000.0, 9000.0, 6000.0, 6000.0,      # board1 (DOF0-5)
        2000.0, 14000.0, 9000.0, 9000.0, 6000.0, 6000.0,      # board2 (DOF6-11)
        10000.0, 8000.0, 8000.0, 10000.0, 8000.0, 8500.0,     # board3 (DOF12-17)
        7000.0, 6000.0, 6000.0, 0.0,                          # board4 (DOF18-21)
        4000.0, 19000.0, 8000.0, 4000.0,                      # board5 (DOF22-25)
    ],
    'kd': [
        115.0, 650.0, 225.0, 500.0, 420.0, 520.0,
        100.0, 600.0, 300.0, 500.0, 300.0, 450.0,
        230.0, 250.0, 400.0, 230.0, 250.0, 230.0,
        300.0, 280.0, 230.0, 0.0,
        150.0, 240.0, 200.0, 50.0,
    ],
    'input_coef': [
        40000.0, 20000.0, 30000.0, 200000.0, 30000.0, 30000.0,
        40000.0, 20000.0, 30000.0, 200000.0, 30000.0, 30000.0,
        40000.0, 60000.0, 30000.0, 40000.0, 40000.0, 50000.0,
        30000.0, 50000.0, 20000.0, 0.0,
        60000.0, 30000.0, 60000.0, 60000.0,
    ],
    'lambda_0': [
        800.0, 300.0, 300.0, 700.0, 300.0, 300.0,
        800.0, 300.0, 300.0, 700.0, 300.0, 300.0,
        300.0, 300.0, 300.0, 300.0, 300.0, 500.0,
        300.0, 500.0, 500.0, 0.0,
        800.0, 300.0, 800.0, 800.0,
    ],
}

# 制御対象の自由度の既定（DOF21・DOF25 は現在制御していない）
DEFAULT_NON_TARGETS = (21, 25)

TOPIC_PARAM_CMD = '/{}_tk_ADRCparam_float/sub'
TOPIC_PARAM_STATUS = '/{}_tk_ADRCparam_float/pub'
TOPIC_Z1 = '/{}_tk_z1_float/pub'
TOPIC_DERIVATIVES = '/{}_tk_derivatives_float/pub'
TOPIC_Z2 = '/{}_tk_z2_float/pub'

CMD_LEN = 1 + 4 * SLOTS             # 25
STATUS_LEN = 2 + 2 * 4 * SLOTS      # 50
SEQ_MAX = 16777216                  # float32 で正確に表せる整数の上限

# 状態トピックの結果
RESULT_TEXT = {
    1: "反映",
    0: "起動後未受信",
    -1: "要素数が違う",
    -2: "seqが不正",
    -3: "inf",
    -4: "範囲外",
    -5: "変更できない自由度",
}

STATUS_STALE_SEC = 2.0      # 状態トピック（200msごと）がこの時間届かなければ「未受信」として扱う
ACK_TIMEOUT_SEC = 3.0       # 送信後この時間内に応答（処理した seq）が届かなければ「応答なし」

PARAM_FILE = os.path.expanduser('~/.armrobot_gui_adrc_params.json')
PARAM_FILE_VERSION = 1


def dof_slot(dof):
    """DOF -> (board名, board内の番号)"""
    for board, offset, count in BOARD_LAYOUT:
        if offset <= dof < offset + count:
            return board, dof - offset
    raise ValueError(f"DOF {dof} はどのボードにもありません")


def board_dofs(board):
    for b, offset, count in BOARD_LAYOUT:
        if b == board:
            return list(range(offset, offset + count))
    raise ValueError(board)


def f32(v):
    """Float32MultiArray で送受信したときと同じ値"""
    return float(np.float32(v))


def same_value(a, b):
    return a is not None and b is not None and f32(a) == f32(b)


def fmt_param(v):
    """パラメータの表示用文字列（整数なら小数点なし）。None・NaN は空欄"""
    if v is None or v != v:
        return ""
    v = float(v)
    if v.is_integer() and abs(v) < 1e15:
        return str(int(v))
    return f"{v:.6g}"


def check_value(key, v):
    """値が送信できるか。できなければ理由の文字列"""
    if v is None or not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(v):
        return "数値ではありません"
    lo, hi = PARAM_LIMITS[key]
    if not (lo <= v <= hi):
        return f"範囲 [{fmt_param(lo)}, {fmt_param(hi)}] の外です"
    return None


# ------------------------------
# パラメータの組（送信予定値・保存値）
# ------------------------------
#   {'values': {key: [26個の float]}, 'targets': [26個の bool], 'saved_at': 保存日時の文字列 or None}
def make_param_set(values, targets, saved_at=None):
    return {
        'values': {key: [float(v) for v in values[key]] for key in PARAM_KEYS},
        'targets': [bool(t) for t in targets],
        'saved_at': saved_at,
    }


def default_param_set():
    return make_param_set(INO_DEFAULTS, [d not in DEFAULT_NON_TARGETS for d in range(NUM_DOF)])


def copy_param_set(ps):
    return make_param_set(ps['values'], ps['targets'], ps.get('saved_at'))


def param_sets_equal(a, b):
    if a is None or b is None:
        return a is b
    if list(a['targets']) != list(b['targets']):
        return False
    return all(f32(x) == f32(y) for key in PARAM_KEYS for x, y in zip(a['values'][key], b['values'][key]))


def dof_values(ps, dof):
    return {key: ps['values'][key][dof] for key in PARAM_KEYS}


def validate_param_set(ps):
    """制御対象の自由度で送信できない値の [(dof, key, 理由), ...]"""
    errors = []
    for dof in range(NUM_DOF):
        if not ps['targets'][dof]:
            continue
        for key in PARAM_KEYS:
            reason = check_value(key, ps['values'][key][dof])
            if reason:
                errors.append((dof, key, reason))
    return errors


def load_param_file(path=None):
    """保存ファイルを読む。戻り値 (パラメータの組 or None, エラー文 or None)"""
    path = path or PARAM_FILE
    if not os.path.exists(path):
        return None, f"{path} がありません（GUI_android_tk_ADRCparamChg.py で保存してください）"
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError) as e:
        return None, f"{path} を読めません:\n{e}"

    try:
        if not isinstance(data, dict) or data.get('version') != PARAM_FILE_VERSION:
            raise ValueError(f"version が {PARAM_FILE_VERSION} ではありません")
        entries = data.get('dofs')
        if not isinstance(entries, list) or len(entries) != NUM_DOF:
            raise ValueError(f"dofs が {NUM_DOF} 自由度ぶんありません")
        values = {key: [None] * NUM_DOF for key in PARAM_KEYS}
        targets = [None] * NUM_DOF
        for e in entries:
            dof = e.get('dof') if isinstance(e, dict) else None
            if not isinstance(dof, int) or isinstance(dof, bool) or not 0 <= dof < NUM_DOF or targets[dof] is not None:
                raise ValueError(f"dof の番号が不正です: {e}")
            board, local = dof_slot(dof)
            if e.get('board') != board or e.get('local') != local:
                raise ValueError(f"DOF {dof} の board・local が {board} #{local} と一致しません"
                                 f"（ファイル: {e.get('board')} #{e.get('local')}）")
            if not isinstance(e.get('control_target'), bool):
                raise ValueError(f"DOF {dof} の control_target が true/false ではありません")
            targets[dof] = e['control_target']
            for key in PARAM_KEYS:
                v = e.get(key)
                if not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(v):
                    raise ValueError(f"DOF {dof} の {key} が数値ではありません: {v!r}")
                values[key][dof] = float(v)
        ps = make_param_set(values, targets, data.get('saved_at'))
        errors = validate_param_set(ps)
        if errors:
            raise ValueError("制御対象の自由度に範囲外の値があります:\n"
                             + "\n".join(f"  DOF {d} {k}: {r}" for d, k, r in errors[:10]))
    except ValueError as e:
        return None, f"{path} の内容が不正なため読み込みません:\n{e}"
    return ps, None


def save_param_file(ps, path=None):
    """保存する（途中で落ちても壊れないよう一時ファイルに書いてから置き換える）。戻り値: 保存日時の文字列"""
    path = path or PARAM_FILE
    saved_at = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
    data = {
        'version': PARAM_FILE_VERSION,
        'saved_at': saved_at,
        'note': "GUI_android_tk_ADRCparamChg.py で保存。GUI_android_tk.py が起動時に読み込む",
        'dofs': [],
    }
    for dof in range(NUM_DOF):
        board, local = dof_slot(dof)
        entry = {'dof': dof, 'board': board, 'local': local, 'control_target': bool(ps['targets'][dof])}
        for key in PARAM_KEYS:
            entry[key] = float(ps['values'][key][dof])
        data['dofs'].append(entry)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    os.replace(tmp, path)
    return saved_at


# ------------------------------
# メッセージ
# ------------------------------
def encode_command(seq, local_values):
    """送信メッセージの data。local_values: {board内の番号: {key: 値}}（無い自由度・key は NaN = 変更しない）"""
    data = [float('nan')] * CMD_LEN
    data[0] = float(seq)
    for local, values in local_values.items():
        for p, key in enumerate(PARAM_KEYS):
            if key in values and values[key] is not None:
                data[1 + 4 * local + p] = float(values[key])
    return data


def decode_status(data, count):
    """状態トピックの data -> {'ack_seq', 'result', 'current': [{key: 値 or None} × count], 'default': [...]}"""
    def values(base, local):
        out = {}
        for p, key in enumerate(PARAM_KEYS):
            v = float(data[base + 4 * local + p])
            out[key] = v if math.isfinite(v) else None
        return out

    return {
        'ack_seq': int(data[0]) if math.isfinite(data[0]) else 0,
        'result': int(data[1]) if math.isfinite(data[1]) else 0,
        'current': [values(2, local) for local in range(count)],
        'default': [values(2 + 4 * SLOTS, local) for local in range(count)],
    }


# ------------------------------
# ROS2: 送信と応答の追跡
# ------------------------------
class AdrcParamLink:
    """ADRCパラメータの送信と、Teensy の状態トピックの受信（受信はROSスレッド、参照はGUIスレッド）"""

    def __init__(self, node, depth=10):
        self.lock = threading.Lock()
        self.status = {board: None for board, _, _ in BOARD_LAYOUT}     # 最新の状態（'t' = 受信時刻を追加）
        self.bad_status = {board: None for board, _, _ in BOARD_LAYOUT}  # 要素数が違う状態メッセージ（要素数）
        self.acks = {board: OrderedDict() for board, _, _ in BOARD_LAYOUT}   # 状態トピックで見た seq -> 結果
        self.sent = {board: deque(maxlen=64) for board, _, _ in BOARD_LAYOUT}  # 送信 {'seq', 't', 'values'}
        self.publishers = {}
        # 再起動ごとに違う番号から始める（同じ seq の古い応答を自分の送信の応答と取り違えないため）
        self._seq = random.randint(1, SEQ_MAX // 4)
        for board, _offset, count in BOARD_LAYOUT:
            self.publishers[board] = node.create_publisher(Float32MultiArray, TOPIC_PARAM_CMD.format(board), depth)
            node.create_subscription(
                Float32MultiArray, TOPIC_PARAM_STATUS.format(board),
                lambda msg, b=board, c=count: self._status_cb(msg, b, c), depth)

    def _status_cb(self, msg, board, count):
        t = time.monotonic()
        data = list(msg.data)
        with self.lock:
            if len(data) != STATUS_LEN:
                self.bad_status[board] = len(data)
                return
            st = decode_status(data, count)
            st['t'] = t
            self.status[board] = st
            self.bad_status[board] = None
            if st['ack_seq'] > 0:
                acks = self.acks[board]
                acks[st['ack_seq']] = st['result']
                acks.move_to_end(st['ack_seq'])
                while len(acks) > 256:
                    acks.popitem(last=False)

    def send(self, board, local_values):
        """board へ送信する。local_values: {board内の番号: {key: 値}}。戻り値: seq"""
        with self.lock:
            self._seq = self._seq % (SEQ_MAX - 1) + 1
            seq = self._seq
            self.sent[board].append({
                'seq': seq, 't': time.monotonic(),
                'values': {local: {key: f32(v) for key, v in vals.items()} for local, vals in local_values.items()},
            })
        msg = Float32MultiArray()
        msg.data = encode_command(seq, local_values)
        self.publishers[board].publish(msg)
        return seq

    # -------- 参照（GUIスレッド） --------
    def board_status(self, board, now=None, allow_stale=False):
        """最新の状態。届いていない・古い（STATUS_STALE_SEC 以上前）ときは None"""
        now = time.monotonic() if now is None else now
        with self.lock:
            st = self.status[board]
        if st is None or (not allow_stale and now - st['t'] > STATUS_STALE_SEC):
            return None
        return st

    def current(self, dof, now=None):
        """Teensy の現在設定値 {key: 値}（状態が届いていなければ None）"""
        board, local = dof_slot(dof)
        st = self.board_status(board, now)
        return None if st is None else st['current'][local]

    def teensy_default(self, dof):
        """Teensy が送ってきた .ino の初期値 {key: 値}（一度も届いていなければ None）"""
        board, local = dof_slot(dof)
        st = self.board_status(board, allow_stale=True)
        return None if st is None else st['default'][local]

    def ack_result(self, board, seq):
        with self.lock:
            return self.acks[board].get(seq)

    def last_send_for_dof(self, dof):
        """この自由度を含む最後の送信 (送信, 送った値 {key: 値}) or (None, None)"""
        board, local = dof_slot(dof)
        with self.lock:
            for entry in reversed(self.sent[board]):
                if local in entry['values']:
                    return entry, entry['values'][local]
        return None, None

    def snapshot(self, targets=None):
        """記録用: 全自由度の現在設定値 [{'dof', 'board', 'local', 'control_target', 'status', key...}, ...]"""
        now = time.monotonic()
        rows = []
        for dof in range(NUM_DOF):
            board, local = dof_slot(dof)
            st = self.board_status(board, now, allow_stale=True)
            row = {'dof': dof, 'board': board, 'local': local,
                   'control_target': None if targets is None else bool(targets[dof])}
            if st is None:
                row['status'] = 'not_received'
                row.update({key: None for key in PARAM_KEYS})
            else:
                row['status'] = 'received' if now - st['t'] <= STATUS_STALE_SEC else 'stale'
                row.update(st['current'][local])
            rows.append(row)
        return rows


def dof_state(link, dof, planned, is_target, now=None):
    """自由度の状態 (コード, 表示文)。planned = 送信予定値・保存値 {key: 値}

    コード: not_target / waiting / rejected / timeout / no_status / applied / match / differ
      applied … この値を送って Teensy が反映を返し、現在設定値も一致している（設定完了）
      match   … 現在設定値が planned と一致している（送信の有無によらない）
    """
    if not is_target:
        return 'not_target', "対象外"
    now = time.monotonic() if now is None else now
    board, _local = dof_slot(dof)
    entry, sent = link.last_send_for_dof(dof)
    sent_now = entry is not None and all(same_value(planned[k], sent.get(k)) for k in sent)
    result = link.ack_result(board, entry['seq']) if entry is not None else None
    cur = link.current(dof, now)
    equal = cur is not None and all(same_value(planned[k], cur[k]) for k in PARAM_KEYS)

    if sent_now:
        if result is None:
            if now - entry['t'] < ACK_TIMEOUT_SEC:
                return 'waiting', "応答待ち"
            if not equal:
                return 'timeout', "応答なし"
        elif result < 0:
            return 'rejected', f"拒否（{RESULT_TEXT.get(result, result)}）"
    if cur is None:
        return 'no_status', "Teensy未受信"
    if equal:
        if sent_now and result == 1:
            return 'applied', "設定完了"
        return 'match', "一致"
    return 'differ', "不一致"


# ------------------------------
# 記録: パラメータ表（記録フォルダに <フォルダ名>_ADRCparam.csv として保存）
# ------------------------------
PARAM_SHEET_SUFFIX = '_ADRCparam.csv'


def write_param_sheet(path, start_rows, end_rows):
    """記録開始時・終了時の Teensy の現在設定値を、DOFごとの表で書く

    列: DOF, board, local, control_target, status_start, kp_start, … , status_end, kp_end, …, changed_during_record
      status: received（受信中）/ stale（STATUS_STALE_SEC 以上届いていない。最後に届いた値）/ not_received（未受信。値は空欄）
    """
    header = ['DOF', 'board', 'local', 'control_target', 'status_start'] + [f"{k}_start" for k in PARAM_KEYS] \
        + ['status_end'] + [f"{k}_end" for k in PARAM_KEYS] + ['changed_during_record']

    def cell(v):
        return "" if v is None else "%.7g" % v

    end_rows = end_rows or [None] * NUM_DOF
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for s, e in zip(start_rows, end_rows):
            target = s['control_target']
            row = [s['dof'], s['board'], s['local'], "" if target is None else int(target), s['status']]
            row += [cell(s[k]) for k in PARAM_KEYS]
            if e is None:
                row += ['not_recorded'] + [""] * len(PARAM_KEYS) + [""]
            else:
                row += [e['status']] + [cell(e[k]) for k in PARAM_KEYS]
                if s['status'] == 'not_received' or e['status'] == 'not_received':
                    row.append("")
                else:
                    row.append(int(any(not same_value(s[k], e[k]) for k in PARAM_KEYS)))
            writer.writerow(row)


# ------------------------------
# 一覧表（ttk.Treeview。読み取り専用）
# ------------------------------
STATE_TAG_STYLE = {
    # コード: (背景色, 文字色)
    'not_target': ("#f2f2f2", "#909090"),
    'waiting': ("#ffe0b0", "black"),
    'rejected': ("#ffc0c0", "black"),
    'timeout': ("#ffc0c0", "black"),
    'no_status': ("white", "#808080"),
    'applied': ("#d8f0d8", "black"),
    'match': ("white", "black"),
    'differ': ("#fff3b0", "black"),
    'invalid': ("#ffc0c0", "black"),
    'error': ("#d00000", "white"),     # 値の発散・描画できない（GUI_android_tk_ADRCparamChg.py）
}


class ParamTable:
    """ボードごとに全自由度のADRCパラメータを並べる表

    sources: [(見出し, getter(dof, key) -> 値 or None), ...] … パラメータごとに、この順で列を並べる
    """

    def __init__(self, parent, sources, font=None, height=14):
        self.sources = sources
        frame = tk.Frame(parent)
        frame.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        cols = ['target'] + [f"{key}:{i}" for key in PARAM_KEYS for i in range(len(sources))] + ['state']
        self.tree = ttk.Treeview(frame, columns=cols, show='tree headings', height=height, selectmode='browse')
        vbar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=self.tree.yview)
        hbar = ttk.Scrollbar(frame, orient=tk.HORIZONTAL, command=self.tree.xview)
        self.tree.configure(yscrollcommand=vbar.set, xscrollcommand=hbar.set)
        self.tree.grid(row=0, column=0, sticky="nsew")
        vbar.grid(row=0, column=1, sticky="ns")
        hbar.grid(row=1, column=0, sticky="ew")

        self.tree.heading('#0', text="DOF", anchor="w")
        self.tree.column('#0', width=150, minwidth=120, stretch=False)
        self.tree.heading('target', text="制御対象")
        self.tree.column('target', width=64, anchor="center", stretch=False)
        for key in PARAM_KEYS:
            for i, (label, _) in enumerate(sources):
                self.tree.heading(f"{key}:{i}", text=f"{key} {label}")
                self.tree.column(f"{key}:{i}", width=92, anchor="e", stretch=False)
        self.tree.heading('state', text="状態", anchor="w")
        self.tree.column('state', width=170, minwidth=120, anchor="w", stretch=True)

        for code, (bg, fg) in STATE_TAG_STYLE.items():
            self.tree.tag_configure(code, background=bg, foreground=fg)
        self.tree.tag_configure('board', background="#e4e8f0", font=font)

        for board, offset, count in BOARD_LAYOUT:
            self.tree.insert('', tk.END, iid=board, text=f"{board}  (DOF {offset}〜{offset + count - 1})",
                             open=True, tags=('board',))
            for local in range(count):
                dof = offset + local
                self.tree.insert(board, tk.END, iid=f"dof{dof}", text=f"DOF {dof}  ({board} #{local})")
        self._last = {}

    def refresh(self, target_fn, state_fn, board_text_fn=None):
        """target_fn(dof) -> bool, state_fn(dof) -> (コード, 表示文), board_text_fn(board) -> ボード行の状態欄"""
        for board, offset, count in BOARD_LAYOUT:
            if board_text_fn is not None:
                text = board_text_fn(board)
                if self._last.get(board) != text:
                    self.tree.item(board, values=[""] * (1 + 4 * len(self.sources)) + [text])
                    self._last[board] = text
            for dof in range(offset, offset + count):
                code, text = state_fn(dof)
                values = ["○" if target_fn(dof) else "－"]
                for key in PARAM_KEYS:
                    for _, getter in self.sources:
                        values.append(fmt_param(getter(dof, key)))
                values.append(text)
                item = (tuple(values), code)
                if self._last.get(dof) != item:
                    self.tree.item(f"dof{dof}", values=values, tags=(code,))
                    self._last[dof] = item
