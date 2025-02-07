import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

class Float32MultiArrayPublisher(Node):
    def __init__(self):
        super().__init__('Float32_multi_array_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/board_float/sub', 10)
        timer_period = 5  # 秒
        self.timer = self.create_timer(timer_period, self.publish_message)

    def publish_message(self):
        msg = Float32MultiArray()
        # レイアウト設定
        msg.layout = MultiArrayLayout()
        dim = MultiArrayDimension()
        dim.label = 'example'
        dim.size = 42
        dim.stride = 42
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0
        
        # データ設定
        #msg.data = [250, 550, 400, 400, 500, 600, 500]
        msg.data = [400.0, 550.0, 400.0, 400.0, 500.0, 600.0, 500.0, 100.0, 110.0, 120.0, 50.0, 60.0, 70.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0]
        #ポテンショメータの値の範囲
        #[0, 腕の閉223-482開, 腕の下344-619上, 上腕の旋回内95-605外, 肘の伸144-740曲, 前腕の旋回内111-962外, 小指側縮62-895伸, 親指側縮0-740伸, 0, 0, 0, 0]
        #小指側のdesired = 親指側のdesired+70
        # [0, 350, 380, 150, 300, 900, 670, 600, 0, 0, 0, 0]
        # [0, 800, 700, 600, 500, 400, 300, 200, 0, 0, 0, 0]

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
