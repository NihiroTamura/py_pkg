import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, MultiArrayLayout, MultiArrayDimension
from collections import deque
import numpy as np

# ローパスフィルタ(移動平均法)関数
def LPF_MAM(x, times, tau=0.01):
    k = np.round(tau / (times[1] - times[0])).astype(int)
    x_mean = np.zeros(x.shape)
    N = x.shape[0]
    for i in range(N):
        if i - k // 2 < 0:
            x_mean[i] = x[: i - k // 2 + k].mean()
        elif i - k // 2 + k >= N:
            x_mean[i] = x[i - k // 2 :].mean()
        else:
            x_mean[i] = x[i - k // 2 : i - k // 2 + k].mean()
    return x_mean

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')

        ## 各自由度ごとにデフォルトのPIDゲインを設定
        self.kp = self.declare_parameter('kp', [0.15, 0.3, 0.25, 0.005, 0.25, 0.1, 0.1]).value
        self.ki = self.declare_parameter('ki', [0.0005, 0.004, 0.0003, 0.0003, 0.0, 0.0, 0.001]).value
        self.kd = self.declare_parameter('kd', [0.1, 0.2, 0.1, 0.1, 0.1, 0.1, 0.1]).value

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
        self.previous_errors = [0.0] * 7
        self.integral = [0.0] * 7

        ## パブリッシャー作成
        self.publisher1 = self.create_publisher(UInt16MultiArray, '/VEAB1/desired', 10)
        self.publisher2 = self.create_publisher(UInt16MultiArray, '/VEAB2/desired', 10)

        ## タイマーを設定して一定間隔でcalculate_and_publishを実行
        self.timer_period = 0.0125  # 秒 (1/0.0125=80 Hz)
        self.timer = self.create_timer(self.timer_period, self.calculate_and_publish)

        ## タイマーの間隔からフィルタリングに使用する時間列を生成
        self.times = np.arange(0, self.timer_period * 10, self.timer_period)  # 10回分の時間列

    ## 時系列のポテンショメータの値をキューに追加
    # 実現値をキューに追加
    def realized_callback(self, msg):
        self.realized_queue.append(msg.data)

    # 目標値をキューに追加
    def desired_callback(self, msg):
        self.desired_queue.append(msg.data)

    ## 停止モードの条件を満たすか確認する関数
    def check_conditions(self, realized_value, previous_realized_value, desired_value):
        error_margin = 30
        rate_of_change = abs(realized_value - previous_realized_value)
        if abs(realized_value - desired_value) <= error_margin and rate_of_change >= 10:
            return True
        return False

    ## calculate_and_publish (VEAB1とVEAB2に送る圧力のPWMの値を計算しパブリッシュ)
    def calculate_and_publish(self):
        if not self.realized_queue or not self.desired_queue:
            return

        # 最新のポテンショメータのキューに入っている値を取り出す
        realized_data = self.realized_queue[-1]
        desired_data = self.desired_queue[-1]

        # 直前のポテンショメータの実測値を取得
        previous_realized_data = self.realized_queue[-2] if len(self.realized_queue) > 1 else realized_data

        # 誤差を計算
        current_errors = [(desired_data[i] - realized_data[i]) for i in range(1, 8)]

        # VEAB1とVEAB2に与える圧力差の初期化
        pid_outputs = []

        # VEAB1とVEAB2に与えるPWMの値の初期化
        veab1_values = []
        veab2_values = []

        # 停止モードの値の設定
        conditions = [
            (134, 122),
            (159, 97),
            (128, 128),
            (130, 126),
            (128, 128),
            (135, 121),
            (132, 124)
        ]

        for i in range(len(conditions)):
            veab_val1, veab_val2 = conditions[i]

            # 停止モードの条件を満たすか確認
            if self.check_conditions(realized_data[i+1], previous_realized_data[i+1], desired_data[i+1]):
                if i < 6:
                    veab1_values.extend([veab_val1, veab_val2])
                else:
                    veab2_values.extend([veab_val1, veab_val2])
            else:
                # PID制御を行う(VEAB1とVEAB2に与える圧力差の計算)
                error = current_errors[i]
                self.integral[i] += error
                derivative = error - self.previous_errors[i]

                pid_output = (self.kp[i] * error +
                              self.ki[i] * self.integral[i] +
                              self.kd[i] * derivative)
            
                pid_output = self.sine[i] * pid_output

                veab_values = self.calculate_veab_values(pid_output)
                if i < 6:
                    veab1_values.extend(veab_values)
                else:
                    veab2_values.extend(veab_values)

                # 計算に用いた誤差を前回の誤差に変更(次の誤差計算用)
                self.previous_errors[i] = error

        # veab1_values と veab2_values にローパスフィルタを適用
        filtered_veab1_values = LPF_MAM(np.array(veab1_values), self.times).astype(int)
        filtered_veab2_values = LPF_MAM(np.array(veab2_values), self.times).astype(int)

        # 各値を 0 から 65535 の範囲にクリップ
        filtered_veab1_values = np.clip(filtered_veab1_values, 0, 65535).tolist()
        filtered_veab2_values = np.clip(filtered_veab2_values, 0, 65535).tolist()

        # publish_values関数を用いてVEAB1とVEAB2に与えるPWMの値をパブリッシュする
        self.publish_values(self.publisher1, filtered_veab1_values)
        self.publish_values(self.publisher2, filtered_veab2_values)

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
    #ROSのスピン開始
    rclpy.spin(pid_controller)
    #ROSの終了処理
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()