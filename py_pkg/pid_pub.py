import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, MultiArrayLayout, MultiArrayDimension
from collections import deque

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')

        ## 各自由度ごとにデフォルトのPIDゲインを設定
        self.kp = self.declare_parameter('kp', [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]).value
        self.ki = self.declare_parameter('ki', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).value
        self.kd = self.declare_parameter('kd', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).value

        ## 各自由度ごとの圧力の正方向とポテンショメータの正方向の対応を整理
        self.sine = self.declare_parameter('sine', [-1.0, -1.0, 1.0, -1.0, -1.0, -1.0, -1.0]).value
        
        ## サブスクリプション
        # ポテンショメータの実現値を読み込む
        self.realized_subscription = self.create_subscription(
            UInt16MultiArray,
            '/POT/realized',
            self.realized_callback,
            10)
        
        # ポテンショメータの目標値を読み込む
        self.desired_subscription = self.create_subscription(
            UInt16MultiArray,
            '/POT/desired',
            self.desired_callback,
            10)

        # ポテンショメータのデータ(実現値と目標値)を保存するキューを設定
        self.realized_queue = deque(maxlen=10)
        self.desired_queue = deque(maxlen=10)

        ## 各要素(自由度)に対する前回の誤差と誤差の積分値
        self.previous_errors = [0.0] * 8
        self.integral = [0.0] * 8

        ## パブリッシャー作成
        self.publisher1 = self.create_publisher(UInt16MultiArray, '/VEAB1/desired', 10)
        self.publisher2 = self.create_publisher(UInt16MultiArray, '/VEAB2/desired', 10)

        ## タイマーを設定して一定間隔でcalculate_and_publishを実行
        self.timer_period = 0.03  # 秒(1/0.03=33.3333 Hz)
        self.timer = self.create_timer(self.timer_period, self.calculate_and_publish)  # Nodeのメソッド

    ## 時系列のポテンショメータの値をキューに追加
    # 実現値をキューに追加
    def realized_callback(self, msg):
        self.realized_queue.append(msg.data)

    # 実現値をキューに追加
    def desired_callback(self, msg):
        self.desired_queue.append(msg.data)

    ## calculate_and_publish (VEAB1とVEAB2に送る圧力のPWMの値を計算しパブリッシュ)
    def calculate_and_publish(self):
        if not self.realized_queue or not self.desired_queue:
            return

        # 最新のポテンショメータのキューに入っている値を取り出す
        realized_data = self.realized_queue[-1]
        desired_data = self.desired_queue[-1]

        # 誤差を計算
        current_errors = [(desired_data[i] - realized_data[i]) for i in range(1, 8)]

        # VEAB1とVEAB2に与える圧力差の初期化
        pid_outputs = []

        # VEAB1とVEAB2に与える圧力差の計算
        for i, error in enumerate(current_errors):
            self.integral[i] += error
            derivative = error - self.previous_errors[i]

            pid_output = (self.kp[i] * error +
                          self.ki[i] * self.integral[i] +
                          self.kd[i] * derivative)
            
            pid_output = self.sine[i] * pid_output

            pid_outputs.append(pid_output)

            # 計算に用いた誤差を前回の誤差に変更(次の誤差計算用)
            self.previous_errors[i] = error

        # VEAB1とVEAB2に与えるPWMの値の初期化
        veab1_values = []
        veab2_values = []

        # VEAB1とVEAB2に与えるPWMの値を計算し格納
        for i, val in enumerate(pid_outputs):
            if i < 6:
                veab1_values.extend(self.calculate_veab_values(val))
            else:
                veab2_values.extend(self.calculate_veab_values(val))

        # publish_values関数を用いてVEAB1とVEAB2に与えるPWMの値をパブリッシュする
        self.publish_values(self.publisher1, veab1_values)
        self.publish_values(self.publisher2, veab2_values)

    # VEAB1とVEAB2に与えるPWMの値を計算
    def calculate_veab_values(self, difference):
        # 両ポートの圧力の平均を定義
        average = 128.0

        # PWMの値を計算
        veab1 = average + (difference / 2.0)
        veab2 = average - (difference / 2.0)

        # 値をクリップし、整数型に変換
        veab1 = max(0, min(255, int(veab1)))
        veab2 = max(0, min(255, int(veab2)))

        return [veab1, veab2]

    # publish_values関数
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
