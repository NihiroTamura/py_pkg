import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension, MultiArrayLayout
import numpy as np

class UInt16MultiArrayPublisher(Node):
    def __init__(self):
        super().__init__('u_int16_multi_array_publisher')
        self.publisher_ = self.create_publisher(UInt16MultiArray, '/POT/desired', 10)
        timer_period = 0.0125  # 秒
        self.timer = self.create_timer(timer_period, self.publish_message)
        self.increment = True
        self.value = 200  # 初期値

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
        
        # データ設定（連続的に変化させる）
        base_data = [0, 800, 700, 600, 500, 400, 300, 200, 0, 0, 0, 0]
        msg_data = []

        # 値の増加・減少の管理
        if self.increment:
            self.value += 10
            if self.value >= 600:
                self.increment = False
        else:
            self.value -= 10
            if self.value <= 300:
                self.increment = True

        # msg.dataの更新
        for i in range(len(base_data)):
            if i == 0:
                msg_data.append(self.value)
            elif base_data[i] != 0:
                if self.increment:
                    msg_data.append(min(base_data[i] + (self.value - 200), 600))
                else:
                    msg_data.append(max(base_data[i] - (800 - self.value), 300))
            else:
                msg_data.append(0)

        msg.data = msg_data
        
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
