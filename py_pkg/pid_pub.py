import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, MultiArrayLayout, MultiArrayDimension
from collections import deque

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # 各エラーチャネルごとにデフォルトのPIDゲインを設定
        self.kp = self.declare_parameter('kp', [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]).value
        self.ki = self.declare_parameter('ki', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).value
        self.kd = self.declare_parameter('kd', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).value

        # サブスクリプション
        self.realized_subscription = self.create_subscription(
            UInt16MultiArray,
            '/POT/realized',
            self.realized_callback,
            10)
        
        self.desired_subscription = self.create_subscription(
            UInt16MultiArray,
            '/POT/desired',
            self.desired_callback,
            10)
        
        self.publisher1 = self.create_publisher(UInt16MultiArray, '/VEAB1/desired', 10)
        self.publisher2 = self.create_publisher(UInt16MultiArray, '/VEAB2/desired', 10)

        # データを保存するキュー
        self.realized_queue = deque(maxlen=10)
        self.desired_queue = deque(maxlen=10)

        # 各要素に対する前回のエラーと積分値
        self.previous_errors = [0.0] * 8
        self.integral = [0.0] * 8

        # タイマーを設定して一定間隔でcalculate_and_publishを実行
        self.timer_period = 0.5  # 秒
        self.timer = self.create_timer(self.timer_period, self.calculate_and_publish)  # Nodeのメソッド

    def realized_callback(self, msg):
        self.realized_queue.append(msg.data)

    def desired_callback(self, msg):
        self.desired_queue.append(msg.data)

    def calculate_and_publish(self):
        if not self.realized_queue or not self.desired_queue:
            return

        realized_data = self.realized_queue[-1]
        desired_data = self.desired_queue[-1]

        current_errors = [(desired_data[i] - realized_data[i]) for i in range(1, 8)]
        pid_outputs = []

        for i, error in enumerate(current_errors):
            self.integral[i] += error
            derivative = error - self.previous_errors[i]

            pid_output = (self.kp[i] * error +
                          self.ki[i] * self.integral[i] +
                          self.kd[i] * derivative)

            pid_outputs.append(pid_output)
            self.previous_errors[i] = error

        veab1_values = []
        veab2_values = []

        for i, val in enumerate(pid_outputs):
            if i < 6:
                veab1_values.extend(self.calculate_veab_values(val))
            else:
                veab2_values.extend(self.calculate_veab_values(val))

        self.publish_values(self.publisher1, veab1_values)
        self.publish_values(self.publisher2, veab2_values)

    def calculate_veab_values(self, difference):
        average = 128.0
        veab1 = average + difference / 2
        veab2 = average - difference / 2
        # 値をクリップし、整数型にキャスト
        veab1 = max(0, min(65535, int(veab1)))
        veab2 = max(0, min(65535, int(veab2)))
        return [veab1, veab2]

    def publish_values(self, publisher, data):
        msg = UInt16MultiArray()
        msg.layout = MultiArrayLayout()
        dim = MultiArrayDimension()
        dim.label = 'example'
        dim.size = 12
        dim.stride = 12
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0
        msg.data = data
        publisher.publish(msg)

def main(args=None):
    #ROS通信の初期化
    rclpy.init(args=args)
    #ノードの生成
    pid_controller = PIDControllerNode()
    #ノード終了の待機
    rclpy.spin(pid_controller)
    #ノードの破棄
    pid_controller.destroy_node()
    #ROS通信の終了
    rclpy.shutdown()

if __name__ == '__main__':
    main()
