import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension, MultiArrayLayout

class UInt16MultiArrayPublisher(Node):
    def __init__(self):
        super().__init__('U_int16_multi_array_publisher')
        self.publisher_ = self.create_publisher(UInt16MultiArray, '/board/sub', 10)
        timer_period = 2  # 秒
        self.timer = self.create_timer(timer_period, self.publish_message)

    def publish_message(self):
        msg = UInt16MultiArray()
        # レイアウト設定
        msg.layout = MultiArrayLayout()
        dim = MultiArrayDimension()
        dim.label = 'example'
        dim.size = 35
        dim.stride = 35
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0
        
        # データ設定
        #msg.data = [250, 550, 400, 400, 500, 600, 500]
        msg.data = [250, 550, 400, 400, 500, 600, 500, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,25, 26, 27, 28]
        #ポテンショメータの値の範囲
        #[腕の閉190-390開, 腕の下286-514上, 上腕の旋回内86-500外, 肘の伸123-628曲, 前腕の旋回内98-900外, 小指側縮48-822伸, 親指側縮1-649伸]

        # メッセージ送信
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = UInt16MultiArrayPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
