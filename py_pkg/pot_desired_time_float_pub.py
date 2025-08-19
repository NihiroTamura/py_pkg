import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import random

class TargetValuePublisher(Node):
    def __init__(self, send_count, use_random, preset_messages):
        super().__init__('target_value_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/board_float/sub', 10)
        self.send_count = send_count
        self.sent_count = 0
        self.use_random = use_random
        self.preset_messages = preset_messages

        # 目標値の範囲（ランダムモード用）
        self.pot_desired_range = [(210, 330), (310, 430), (120, 430), (160, 560), (140, 810), (80, 700), (40, 570)]

        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info(f'開始します。{self.send_count} 回目標値を送信します。')

    def timer_callback(self):
        if self.sent_count >= self.send_count:
            self.get_logger().info('全ての目標値を送信しました。ノードを終了します。')
            rclpy.shutdown()
            return

        msg = Float32MultiArray()

        # レイアウト設定
        msg.layout = MultiArrayLayout()
        dim = MultiArrayDimension()
        dim.label = 'example'
        dim.size = 7
        dim.stride = 7
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0

        # データ設定
        if self.use_random:
            pot_desired = [float(random.randint(r[0], r[1])) for r in self.pot_desired_range]
        else:
            pot_desired = self.preset_messages[self.sent_count]

        msg.data = pot_desired
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        self.sent_count += 1

def main(args=None):
    rclpy.init(args=args)

    try:
        send_count = int(input('送信する目標値の回数を入力してください: '))
    except ValueError:
        print('整数を入力してください。')
        return

    mode = input('random(y) or setup(n) ? ').strip().lower()
    use_random = True if mode == 'y' else False

    # プリセットメッセージの定義（setupモード用）
    preset_messages = [
        [214.0, 310.0, 342.0, 554.0, 228.0, 372.0, 183.0],
        [230.0, 359.0, 230.0, 296.0, 650.0, 288.0, 518.0],
        [250.0, 400.0, 420.0, 168.0, 154.0, 464.0, 351.0],
        [279.0, 430.0, 294.0, 285.0, 265.0, 84.0, 489.0],
        [330.0, 410.0, 397.0, 432.0, 771.0, 231.0, 40.0],

        [330.0, 430.0, 414.0, 210.0, 400.0, 449.0, 349.0],
        [307.0, 396.0, 120.0, 160.0, 470.0, 556.0, 47.0],
        [267.0, 352.0, 130.0, 163.0, 387.0, 343.0, 98.0],
        [271.0, 404.0, 430.0, 235.0, 809.0, 277.0, 208.0],
        [233.0, 317.0, 373.0, 480.0, 140.0, 694.0, 109.0],

        [320.0, 330.0, 380.0, 300.0, 450.0, 300.0, 300.0],
        [220.0, 400.0, 120.0, 400.0, 500.0, 350.0, 250.0],
        [280.0, 340.0, 170.0, 250.0, 650.0, 420.0, 270.0],
        [250.0, 420.0, 400.0, 350.0, 530.0, 430.0, 350.0],
        [310.0, 360.0, 190.0, 200.0, 590.0, 510.0, 290.0]
    ]

    if not use_random:
        if send_count > len(preset_messages):
            print(f'プリセットの数は {len(preset_messages)} 個しかありません。送信回数を減らしてください。')
            return

    input('エンターキーを押すと送信を開始します...')

    node = TargetValuePublisher(send_count, use_random, preset_messages)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
