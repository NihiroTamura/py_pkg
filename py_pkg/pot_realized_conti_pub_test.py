import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension, MultiArrayLayout

class UInt16MultiArrayPublisher(Node):
    def __init__(self):
        super().__init__('U_int16_multi_array_publisher')
        self.publisher_ = self.create_publisher(UInt16MultiArray, '/POT/realized', 10)
        self.values = [0] + [100] * 7  # 2番目から8番目の初期値は100、1番目は固定値0
        self.increment = 20
        self.direction = [1] * 7  # 各要素の増加・減少の方向
        timer_period = 1  # 秒
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
        
        # データ更新
        for i in range(1, 8):  # 2番目から8番目の要素を操作
            if self.direction[i - 1] == 1:
                self.values[i] += self.increment
                if self.values[i] >= 900:
                    self.direction[i - 1] = -1  # 増加から減少へ転換
            else:
                self.values[i] -= self.increment
                if self.values[i] <= 100:
                    self.direction[i - 1] = 1  # 減少から増加へ転換

        # データ設定
        msg.data = self.values + [0, 0, 0, 0]  # 9番目以降は固定値
        
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
