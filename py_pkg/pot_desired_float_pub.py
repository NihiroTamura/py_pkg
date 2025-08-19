import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

class Float32MultiArrayPublisher(Node):
    def __init__(self):
        super().__init__('Float32_multi_array_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/board_float/sub', 10)
        timer_period = 2  # 秒
        self.timer = self.create_timer(timer_period, self.publish_message)

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
        # 初期値
        #msg.data = [260.0, 400.0, 180.0, 500.0, 600.0, 500.0, 500.0]
        # 目標値1
        msg.data = [350.0, 300.0, 380.0, 300.0, 450.0, 300.0, 300.0]
        # 目標値2
        #msg.data = [200.0, 400.0, 120.0, 450.0, 420.0, 200.0, 200.0]
        # 目標値3
        #msg.data = [230.0, 370.0, 150.0, 520.0, 600.0, 500.0, 500.0]
        # 目標値4
        #msg.data = [300.0, 460.0, 420.0, 200.0, 100.0, 100.0, 100.0]
        # 目標値5
        #msg.data = [260.0, 300.0, 480.0, 150.0, 800.0, 300.0, 400.0]
        
        #ポテンショメータの値の範囲
        #[(210, 330), (310, 430), (120, 430), (160, 560), (140, 810), (80, 700), (40, 570)]
        #[腕の閉190-390開, 腕の下286-514上, 上腕の旋回内86-500外, 肘の伸123-628曲, 前腕の旋回内98-900外, 小指側縮48-822伸, 親指側縮1-649伸]

        # メッセージ送信
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = Float32MultiArrayPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
