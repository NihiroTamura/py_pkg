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
        self.pot_desired_range = [(130, 680), (10, 734), (66, 259), (192, 389), (192, 445), (284, 557), (115, 619), (22, 794), (239, 445), (142, 395), (300, 500), (420, 600), (10, 685), (10, 714), (105, 845), (120, 700), (275, 370), (3, 670), (3, 750), (9, 680), (70, 630), (115, 785), (135, 565), (60, 485), (323, 600), (188, 645)]

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
