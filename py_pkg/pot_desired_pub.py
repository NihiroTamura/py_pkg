import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension, MultiArrayLayout

class UInt16MultiArrayPublisher(Node):
    def __init__(self):
        super().__init__('U_int16_multi_array_publisher')
        self.publisher_ = self.create_publisher(UInt16MultiArray, '/POT/desired', 10)
        timer_period = 0.0125  # 秒
        self.timer = self.create_timer(timer_period, self.publish_message)

    def publish_message(self):
        msg = UInt16MultiArray()
        # レイアウト設定
        msg.layout = MultiArrayLayout()
        dim = MultiArrayDimension()
        dim.label = 'example'
        dim.size = 12
        dim.stride = 12
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0
        
        # データ設定
        msg.data = [0, 350, 380, 150, 300, 900, 670, 600, 0, 0, 0, 0]
        #ポテンショメータの値の範囲
        #[0, 腕の閉223-482開, 腕の下334-609上, 上腕の旋回内95-605外, 肘の伸144-740曲, 前腕の旋回内111-962外, 小指側縮62-895伸, 親指側縮0-740伸, 0, 0, 0, 0]
        #小指側のdesired = 親指側のdesired+70
        
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
