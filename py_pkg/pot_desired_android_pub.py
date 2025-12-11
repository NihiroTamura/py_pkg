import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

class Float32MultiArrayPublisher(Node):
    def __init__(self):
        super().__init__('Float32_multi_array_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/board_android_float/sub', 10)
        timer_period = 2  # 秒
        self.timer = self.create_timer(timer_period, self.publish_message)

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
        # 番号
        #msg.data = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0, 130.0, 140.0, 150.0, 160.0, 170.0, 180.0, 190.0, 200.0, 210.0, 220.0, 230.0, 240.0, 250.0, 260.0]
        # 初期値
        msg.data = [500.0, 300.0, 170.0, 300.0, 280.0, 420.0, 200.0, 500.0, 350.0, 180.0, 500.0, 500.0, 300.0, 250.0, 400.0, 505.0, 325.0, 350.0, 420.0, 400.0, 160.0, 370.0, 200.0, 410.0, 360.0, 390.0]
        # 目標値1
        #msg.data = [500.0, 300.0, 170.0, 300.0, 280.0, 420.0, 200.0, 500.0, 350.0, 180.0, 500.0, 500.0, 300.0, 250.0, 400.0, 505.0, 325.0, 350.0, 600.0, 400.0, 160.0, 370.0, 200.0, 410.0, 360.0, 390.0]
        # 目標値2
        #msg.data = [500.0, 300.0, 170.0, 300.0, 280.0, 420.0, 200.0, 500.0, 350.0, 180.0, 500.0, 500.0, 300.0, 250.0, 400.0, 505.0, 325.0, 350.0, 150.0, 400.0, 160.0, 370.0, 200.0, 410.0, 360.0, 390.0]
        # 目標値3
        #msg.data = [500.0, 300.0, 170.0, 300.0, 280.0, 420.0, 200.0, 500.0, 350.0, 180.0, 500.0, 500.0, 300.0, 250.0, 400.0, 505.0, 325.0, 350.0, 420.0, 400.0, 160.0, 370.0, 200.0, 410.0, 360.0, 390.0]

        
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
