import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout


class Float32MultiArrayPublisher(Node):
    def __init__(self):
        super().__init__('Float32_multi_array_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/board_float/sub', 10)
        self.get_logger().info('Press Enter to publish next desired Angle')

        #[260.0, 400.0, 180.0, 500.0, 600.0, 500.0, 500.0]
        #[(210, 330), (310, 430), (120, 430), (160, 560), (140, 810), (80, 700), (40, 570)]

        # あらかじめ指定する送信データ（例: 3パターン）
        self.predefined_data = [
            [330.0, 330.0, 250.0, 430.0, 530.0, 430.0, 430.0, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2],
            [250.0, 400.0, 180.0, 500.0, 600.0, 500.0, 490.0, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2],
        ]
        self.index = 0  # 送信する配列のインデックス

    def publish_message(self):
        msg = Float32MultiArray()

        # レイアウト設定
        msg.layout = MultiArrayLayout()
        dim = MultiArrayDimension()
        dim.label = 'example'
        dim.size = 7
        dim.stride = 7
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0

        # データ設定（あらかじめ決めた値を送信）
        msg.data = self.predefined_data[self.index]

        # メッセージ送信
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        # 次のデータへ（最後まで行ったら0に戻る）
        self.index = (self.index + 1) % len(self.predefined_data)


def main(args=None):
    rclpy.init(args=args)
    node = Float32MultiArrayPublisher()
    try:
        while rclpy.ok():
            input()  # エンターキー待ち
            node.publish_message()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
