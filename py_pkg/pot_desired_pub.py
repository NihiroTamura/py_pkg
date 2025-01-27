import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension, MultiArrayLayout

class UInt16MultiArrayPublisher(Node):
    def __init__(self):
        super().__init__('U_int16_multi_array_publisher')
        self.publisher_ = self.create_publisher(UInt16MultiArray, '/board/sub', 10)
        timer_period = 5  # 秒
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
        msg.data = [400, 400, 150, 300, 900, 390, 300, 140, 100, 1500, 1000, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 100, 200, 300, 400]
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
