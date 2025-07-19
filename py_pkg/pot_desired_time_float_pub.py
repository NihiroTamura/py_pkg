import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import random
import time

class TargetValuePublisher(Node):
    def __init__(self, send_count):
        super().__init__('target_value_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/board_float/sub', 10)
        self.send_count = send_count
        self.sent_count = 0

        #   目標値の範囲
        self.pot_desired_range = [(210, 370), (300, 500), (100, 480), (140, 600), (120, 880), (70, 800), (20, 620)]

        self.timer = self.create_timer(10.0, self.timer_callback)
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
        self.pot_desired = [float(random.randint(r[0], r[1])) for r in self.pot_desired_range]
        msg.data = self.pot_desired

        # メッセージ送信
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        self.sent_count += 1

def main(args=None):
    rclpy.init(args=args)

    try:
        send_count = int(input('送信する目標値の回数を入力してください: '))
        input('エンターキーを押すと送信を開始します...')
    except ValueError:
        print('整数を入力してください。')
        return

    node = TargetValuePublisher(send_count)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
