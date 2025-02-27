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
        #msg.data = [350.0, 300.0, 380.0, 300.0, 450.0, 300.0, 300.0]
        #msg.data = [200.0, 400.0, 120.0, 450.0, 420.0, 200.0, 200.0]
        msg.data = [230.0, 370.0, 150.0, 520.0, 600.0, 500.0, 500.0]
        #msg.data = [400.0, 550.0, 400.0, 400.0, 500.0, 600.0, 500.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0, 39.0, 40.0, 41.0, 42.0, 43.0, 44.0, 45.0, 46.0, 47.0, 48.0, 49.0, 50.0, 51.0, 52.0, 53.0, 54.0, 55.0, 56.0]
        #msg.data = [350.0, 400.0, 450.0, 450.0, 500.0, 600.0, 500.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0]
        #msg.data = [220.0, 330.0, 300.0, 200.0, 500.0, 300.0, 200.0, 0.396, 0.183, 4.651, -0.748, 0.422, 0.122, 4.345, 1.356, 0.292, 0.131, 4.049, 1.124, 0.291, 0.117, 5.421, 0.047, 0.376, 0.194, 3.993, -0.558, 0.425, 0.253, 4.23, -0.077, 0.25, 0.307, 3.865, -0.03, 0.392, 0.357, 4.076, -1.046, 0.474, 0.235, 3.032, 0.022, 0.442, 0.169, 3.944, 0.081, 0.445, 0.302, 3.806, -0.518, 0.553, 0.392, 4.662, 0.872, 0.389, 0.186, 3.278, -1.8, 0.424, 0.347, 3.698, 0.136]
        #msg.data = [220.0, 330.0, 300.0, 200.0, 500.0, 300.0, 200.0, 5.691, 2.534, 6.074, -0.118, 5.465, -1.743, 5.368, 1.545, 6.861, -4.24, 7.202, -0.628, 4.148, 0.268, 6.184, 1.584, 4.78, 0.684, 3.676, 4.291, 7.109, -2.266, 3.156, 0.643, 6.216, 0.692, 5.734, 3.751, 1.898, 0.065, 5.191, 2.18, 3.795, 0.725, 2.765, 2.174, 2.395, -1.765, 3.794, -0.463, 8.095, -2.244, 7.262, -1.765, 2.322, -0.373, 6.204, -1.728, 4.933, -1.053, 6.682, 2.437, 4.964, 2.325, 7.015, -0.319]
        #ポテンショメータの値の範囲
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
