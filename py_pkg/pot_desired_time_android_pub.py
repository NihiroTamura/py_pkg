import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import random

class TargetValuePublisher(Node):
    def __init__(self, send_count, use_random, preset_messages):
        super().__init__('target_value_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/board_android_float/sub', 10)
        self.send_count = send_count
        self.sent_count = 0
        self.use_random = use_random
        self.preset_messages = preset_messages

        # 目標値の範囲（ランダムモード用）
        self.pot_desired_range = [(400, 650), (150, 700), (120, 220), (200, 370), (240, 400), (350, 500), (150, 250), (150, 700), (300, 400), (170, 350), (500, 500), (500, 500), (100, 600), (150, 600), (150, 800), (150, 600), (300, 350), (100, 600), (150, 600), (50, 650), (150, 600), (120, 800), (180, 500), (160, 430), (350, 580), (250, 530)]

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
        dim.size = 26
        dim.stride = 26
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
        # 1つ目
        [500.0, 300.0, 170.0, 300.0, 280.0, 420.0, 200.0, 500.0, 350.0, 180.0, 500.0, 500.0, 300.0, 250.0, 400.0, 505.0, 325.0, 350.0, 420.0, 400.0, 160.0, 370.0, 200.0, 410.0, 360.0, 390.0],
        [500.0, 300.0, 170.0, 300.0, 280.0, 420.0, 200.0, 500.0, 350.0, 180.0, 500.0, 500.0, 300.0, 250.0, 400.0, 505.0, 325.0, 350.0, 420.0, 400.0, 160.0, 370.0, 200.0, 410.0, 360.0, 390.0],
        [500.0, 300.0, 170.0, 300.0, 280.0, 420.0, 200.0, 500.0, 350.0, 180.0, 500.0, 500.0, 300.0, 250.0, 400.0, 505.0, 325.0, 350.0, 420.0, 400.0, 160.0, 370.0, 200.0, 410.0, 360.0, 390.0],
        [500.0, 300.0, 170.0, 300.0, 280.0, 420.0, 200.0, 500.0, 350.0, 180.0, 500.0, 500.0, 300.0, 250.0, 400.0, 505.0, 325.0, 350.0, 420.0, 400.0, 160.0, 370.0, 200.0, 410.0, 360.0, 390.0],
        [500.0, 300.0, 170.0, 300.0, 280.0, 420.0, 200.0, 500.0, 350.0, 180.0, 500.0, 500.0, 300.0, 250.0, 400.0, 505.0, 325.0, 350.0, 420.0, 400.0, 160.0, 370.0, 200.0, 410.0, 360.0, 390.0]
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
