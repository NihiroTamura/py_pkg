import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, MultiArrayLayout, MultiArrayDimension
from collections import deque
import numpy as np

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')

        ## 各自由度ごとにデフォルトのPIDゲインを設定
        self.kp = np.array(self.declare_parameter('kp', [0.18, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).value)
        self.ki = np.array(self.declare_parameter('ki', [0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).value)
        self.kd = np.array(self.declare_parameter('kd', [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).value)

        ## 各自由度ごとの圧力の正方向とポテンショメータの正方向の対応を整理
        self.sine = np.array(self.declare_parameter('sine', [-1.0, -1.0, 1.0, -1.0, -1.0, -1.0, -1.0]).value)
        
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
        self.realized_queue = deque(maxlen=2)  # 変化率計算のために最新2つのデータを保持
        self.desired_queue = deque(maxlen=1)

        ## 各要素(自由度)に対する前回の誤差と誤差の積分値
        self.previous_errors = np.zeros(7)
        self.integral = np.zeros(7)

        ## パブリッシャー作成
        self.publisher1 = self.create_publisher(UInt16MultiArray, '/VEAB1/desired', 10)
        self.publisher2 = self.create_publisher(UInt16MultiArray, '/VEAB2/desired', 10)

        ## タイマーを設定して一定間隔でcalculate_and_publishを実行
        self.timer_period = 0.0125  # 秒(1/0.0125=80 Hz)
        self.timer = self.create_timer(self.timer_period, self.calculate_and_publish)  # Nodeのメソッド

        # VEAB1とVEAB2の初期化
        self.veab1_values = np.zeros(12, dtype=int)
        self.veab2_values = np.zeros(2, dtype=int)

    ## 時系列のポテンショメータの値をキューに追加
    # 実現値をキューに追加
    def realized_callback(self, msg):
        self.realized_queue.append(np.array(msg.data[1:8]))

    # 目標値をキューに追加
    def desired_callback(self, msg):
        self.desired_queue.append(np.array(msg.data[1:8]))

    ## calculate_and_publish (VEAB1とVEAB2に送る圧力のPWMの値を計算しパブリッシュ)
    def calculate_and_publish(self):
        if len(self.realized_queue) < 2 or not self.desired_queue:
            return

        # 最新のポテンショメータのキューに入っている値を取り出す
        realized_data_current = self.realized_queue[-1]
        realized_data_previous = self.realized_queue[-2]
        desired_data = self.desired_queue[-1]

        # 誤差を計算
        current_errors = desired_data - realized_data_current

        # 実現値の変化率を計算
        realized_rate_of_change = np.abs(realized_data_current - realized_data_previous)

        # 条件をチェック：目標値 ±20 に収まるかつ実現値の変化率が10以上
        if np.all(np.abs(current_errors) <= 20) and np.all(realized_rate_of_change >= 10):
            # 条件を満たす場合、固定のPWM値を与える
            veab1 = np.array([134, 122, 159, 97, 128, 128, 130, 126, 128, 128, 135, 121], dtype=int)
            veab2 = np.array([132, 124], dtype=int)
        else:
            # 条件を満たさない場合は通常のPID制御を行う
            self.integral += current_errors
            derivative = current_errors - self.previous_errors
            pid_outputs = self.kp * current_errors + self.ki * self.integral + self.kd * derivative
            pid_outputs = self.sine * pid_outputs

            veab1, veab2 = self.calculate_veab_values(pid_outputs)

        # 計算に用いた誤差を前回の誤差に更新
        self.previous_errors = current_errors

        # VEAB1とVEAB2の値を更新
        self.veab1_values[:len(veab1)] = veab1
        self.veab2_values[:len(veab2)] = veab2

        # publish_values関数を用いてVEAB1とVEAB2に与えるPWMの値をパブリッシュする
        self.publish_values(self.publisher1, self.veab1_values)
        self.publish_values(self.publisher2, self.veab2_values)

    # VEAB1とVEAB2に与えるPWMの値を計算
    def calculate_veab_values(self, pid_outputs):
        # 両ポートの圧力の平均を定義
        average = 128.0

        # PWMの値を計算
        veab1 = np.clip(average + (pid_outputs[:6] / 2.0), 0, 255).astype(int)
        veab2 = np.clip(average - (pid_outputs[6:] / 2.0), 0, 255).astype(int)

        return veab1, veab2

    # publish_values関数
    def publish_values(self, publisher, data):
        msg = UInt16MultiArray()
        msg.layout = MultiArrayLayout()
        dim = MultiArrayDimension()
        dim.label = 'example'
        dim.size = len(data)
        dim.stride = len(data)
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0
        msg.data = data.tolist()
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
