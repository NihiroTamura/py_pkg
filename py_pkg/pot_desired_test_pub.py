import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, Float32MultiArray, MultiArrayDimension, MultiArrayLayout

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        #self.publisher_ = self.create_publisher(UInt16MultiArray, '/board/sub', 10)
        self.publisher_ = self.create_publisher(Float32MultiArray, '/board1_FFparam_float/sub', 10)
        timer_period = 0.1  # 秒
        self.timer = self.create_timer(timer_period, self.publish_message)

    def publish_message(self):
        #msg = UInt16MultiArray()
        msg = Float32MultiArray()

        # レイアウト設定
        msg.layout = MultiArrayLayout()
        dim = MultiArrayDimension()
        dim.label = 'example'
        dim.size = 36
        dim.stride = 36
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0
        
        # データ設定
        #msg.data = [250, 550, 400, 400, 500, 600, 500]
        #msg.data = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0]
        #msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5, -4.8752, 36.525, -76.388, 48.856, 0.000019215, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5]
        # -5.0409e+01    1.3625e+02    1.2987e+02   -4.9052e+02    2.3893e+02
        # a=1.6e+01, b=3.0e+01, c=-1.4e+02, d=8.2e+01, e=1.0e-07

        # optimize_LQR_ROS2_20260703_1620.csv
        msg.data = [2.3712, -2.1633, -4.4866, -3.9494, 11.316, 1.5, -3.6728, 6.2367, 6.7571, -12.787, 1.5209, 1.5, -10.231, 43.352, -29.481, -25.260, 9.7057, 1.5, -4.8752, 36.525, -76.388, 48.856, 0.000019215, 1.5, 3.8835, -2.0942, -10.681, 0.29727, 10.994, 1.5, 236.71, -596.07, 336.92, 32.048, 7.2413, 1.5] # board1
        #msg.data = [2.5185, 5.1752, -21.399, 11.551, 0.60559, 1.5, 10.520, -31.451, 29.368, -10.163, 2.0555, 1.5, 0.092616, -0.13732, 0.18740, -0.28470, 0.000000013007, 1.5, -8.3057, 22.973, -9.1985, -14.671, 7.2173, 1.5, 1.6798, 2.5373, -9.3486, 1.2727, 2.0578, 1.5, 0.33991, 1.0982, -3.8415, 2.1440, 0.0000000055140, 1.5] # board2
        #msg.data = [29.872, -63.308, 27.329, -25.660, 39.438, 1.5, 7.4109, -2.1501, -8.4105, -19.636, 18.116, 1.5, -14.201, 56.863, -72.582, 28.862, 0.000078204, 1.5, -0.85045, -3.0720, 31.334, -51.756, 21.805, 1.5, 0.31750, -0.28253, -0.60842, -0.55391, 1.5460, 1.5, 0.064527, 0.10082, -0.47742, 0.27151, 0.0000000083416, 1.5] # board3
        #msg.data = [0.88516, 1.4704, -6.6800, 3.7241, 0.0000022750, 1.5, 0.39277, -0.32870, -0.84695, -0.54280, 1.8408, 1.5, 0.56030, -5.0268, 15.555, -13.913, 0.000000000050132, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5] # board4
        #msg.data = [2.9048, -4.1942, -0.57721, 0.49913, 0.0000000036109, 1.5, 10.339, -32.349, 35.919, -19.901, 5.8733, 1.5, 8.0209, 15.574, -62.664, 27.909, 5.9633, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5] # board5

        # メッセージ送信
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
