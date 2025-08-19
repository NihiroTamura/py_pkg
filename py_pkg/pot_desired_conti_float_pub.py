import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import random

class Float32MultiArrayPublisher(Node):
    def __init__(self):
        super().__init__('Float32_multi_array_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/board_float/sub', 10)
        self.get_logger().info('Press Enter to publish desired Angle')

        #   目標値の範囲
        self.pot_desired_range = [(210, 330), (310, 430), (120, 430), (160, 560), (140, 810), (80, 700), (40, 570)]

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
        
        # データ設定
        self.pot_desired = [float(random.randint(r[0], r[1])) for r in self.pot_desired_range]
        msg.data = self.pot_desired

        # メッセージ送信
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

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
