import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout


def _clamp(value: float, low: float, high: float) -> float:
    return float(max(low, min(high, float(value))))


def _random_with_min_delta(low: int, high: int, reference: float, min_delta: float) -> float:
    """reference から |差| が min_delta 以上になるよう範囲内で乱数を生成する。"""
    ref = float(reference)
    d = float(min_delta)

    if d <= 0:
        return float(random.randint(low, high))

    # |v - ref| >= d を満たす区間
    candidates = []
    upper_end = int(ref - d)
    if low <= upper_end:
        candidates.append((low, upper_end))
    lower_start = int(ref + d)
    if lower_start <= high:
        candidates.append((lower_start, high))

    if candidates:
        seg_low, seg_high = random.choice(candidates)
        return float(random.randint(seg_low, seg_high))

    # 範囲が狭く条件を満たせない場合は、reference から最も離れた端を選ぶ
    if abs(low - ref) >= abs(high - ref):
        return float(_clamp(low, low, high))
    return float(_clamp(high, low, high))


class TargetValuePublisher(Node):
    # 送信間隔(秒)
    PUBLISH_INTERVAL = 5.0

    def __init__(self, send_count, use_random, preset_messages, min_delta=None, initial_pot_desired=None):
        super().__init__('target_value_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/board_android_float/sub', 10)
        self.send_count = send_count
        self.sent_count = 0
        self.use_random = use_random
        self.preset_messages = preset_messages
        self.min_delta = min_delta
        self.initial_pot_desired = initial_pot_desired
        self.last_published = None
        self._start_time = time.monotonic()
        self._timer = None

        # 目標値の範囲（ランダムモード用）
        #self.pot_desired_range = [(185, 700), (135, 550), (130, 680), (10, 734), (66, 259), (192, 389), (70, 600), (60, 465), (115, 619), (22, 794), (239, 430), (205, 395), (30, 660), (30, 690), (110, 830), (3, 630), (3, 700), (9, 660), (275, 360), (115, 785), (192, 440), (284, 557), (323, 580), (188, 630), (375, 500), (300, 490)]
        self.pot_desired_range = [(450, 700), (135, 550), (500, 680), (250, 700), (66, 259), (192, 389), (70, 200), (60, 465), (115, 200), (100, 550), (239, 430), (205, 395), (30, 660), (30, 690), (110, 830), (3, 630), (3, 700), (9, 660), (275, 360), (115, 785), (192, 440), (284, 557), (323, 580), (188, 630), (375, 500), (300, 490)]

        if self.use_random:
            if self.min_delta is None or self.initial_pot_desired is None:
                raise ValueError('ランダムモードでは min_delta と initial_pot_desired が必要です')
            if len(self.initial_pot_desired) != len(self.pot_desired_range):
                raise ValueError(
                    f'initial_pot_desired の長さ({len(self.initial_pot_desired)})が '
                    f'pot_desired_range({len(self.pot_desired_range)})と一致しません'
                )
            self.initial_pot_desired = [float(v) for v in self.initial_pot_desired]

        self._schedule_next_publish()
        if self.use_random:
            self.get_logger().info(
                f'開始します。{self.send_count} 回目標値を送信します。'
                f' 最小変化量={self.min_delta}'
            )
        else:
            self.get_logger().info(f'開始します。{self.send_count} 回目標値を送信します。')

    def _generate_random_pot_desired(self):
        pot_desired = []
        for i, (low, high) in enumerate(self.pot_desired_range):
            if self.last_published is None:
                reference = self.initial_pot_desired[i]
            else:
                reference = self.last_published[i]

            value = _random_with_min_delta(low, high, reference, self.min_delta)
            value = _clamp(value, low, high)
            pot_desired.append(float(value))
        return pot_desired

    def _schedule_next_publish(self):
        """開始時刻からの絶対時刻に合わせて次回パブリッシュをスケジュールする。"""
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None

        next_deadline = self._start_time + (self.sent_count + 1) * self.PUBLISH_INTERVAL
        delay = max(0.0, next_deadline - time.monotonic())
        self._timer = self.create_timer(delay, self.timer_callback)

    def timer_callback(self):
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None

        if self.sent_count >= self.send_count:
            self.get_logger().info('全ての目標値を送信しました。ノードを終了します。')
            rclpy.shutdown()
            return

        msg = Float32MultiArray()

        # レイアウト設定
        msg.layout = MultiArrayLayout()
        dim = MultiArrayDimension()
        dim.label = 'example'
        dim.size = 26
        dim.stride = 26
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0

        # データ設定
        if self.use_random:
            pot_desired = self._generate_random_pot_desired()
            self.last_published = list(pot_desired)
        else:
            pot_desired = self.preset_messages[self.sent_count]

        msg.data = [float(v) for v in pot_desired]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        self.sent_count += 1

        if self.sent_count >= self.send_count:
            self.get_logger().info('全ての目標値を送信しました。ノードを終了します。')
            rclpy.shutdown()
            return

        self._schedule_next_publish()


def main(args=None):
    rclpy.init(args=args)

    try:
        send_count = int(input('送信する目標値の回数を入力してください: '))
    except ValueError:
        print('整数を入力してください。')
        return

    mode = input('random(y) or setup(n) ? ').strip().lower()
    use_random = True if mode == 'y' else False

    min_delta = None
    initial_pot_desired = None
    if use_random:
        try:
            min_delta = float(input('前回値からの最小変化量（各自由度共通）を入力してください: '))
        except ValueError:
            print('数値を入力してください。')
            return

        # 1回目の基準値（前回値の代わりに |差| を計算する基準）
        initial_pot_desired = [
            500.0, 200.0, 500.0, 300.0, 170.0, 300.0,
            160.0, 410.0, 200.0, 500.0, 350.0, 220.0,
            300.0, 250.0, 400.0, 350.0, 420.0, 400.0,
            325.0, 370.0, 280.0, 420.0,
            360.0, 390.0, 420.0, 390.0,
        ]

    # プリセットメッセージの定義（setupモード用）
    preset_messages = [
        # [214.0, 257.0, 218.0, 306.0, 345.0, 466.0, 228.0, 188.0, 324.0, 289.0, 0.0, 0.0, 672.0, 463.0, 579.0, 592.0, 441.0, 518.0, 218.0, 146.0, 289.0, 540.0, 440.0, 349.0, 518.0, 253.0],
    ]

    if not use_random:
        if send_count > len(preset_messages):
            print(f'プリセットの数は {len(preset_messages)} 個しかありません。送信回数を減らしてください。')
            return

    input('エンターキーを押すと送信を開始します...')

    node = TargetValuePublisher(
        send_count, use_random, preset_messages, min_delta, initial_pot_desired
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
