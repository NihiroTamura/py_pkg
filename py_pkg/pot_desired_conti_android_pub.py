import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import random

class Float32MultiArrayPublisher(Node):
    def __init__(self):
        super().__init__('Float32_multi_array_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/board_android_float/sub', 10)
        self.get_logger().info('Press Enter to publish desired Angle')

        #   目標値の範囲
        self.pot_desired_range = [(400, 650), (150, 700), (120, 220), (200, 370), (240, 400), (350, 500), (150, 250), (150, 700), (300, 400), (170, 350), (500, 500), (500, 500), (100, 600), (150, 600), (150, 800), (150, 600), (300, 350), (100, 600), (150, 600), (50, 650), (150, 600), (120, 800), (180, 500), (160, 430), (350, 580), (250, 530)]

    def publish_message(self):
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
