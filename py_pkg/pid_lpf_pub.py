import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, MultiArrayLayout, MultiArrayDimension
from collections import deque
import numpy as np

## ローパスフィルタ関数
def LPF_MAM(veab_values, previous_veab_values):
    # 重み設定
    a = 0.8

    # ローパスフィルタ語の値を格納するリストを初期化
    filtered_veab_values = [0] * len(veab_values)

    # <１ステップ前のポートの値> × a + <今計算されたポートの値> × (1-a)
    for i in range(len(veab_values)):
        filtered_veab_values[i] = previous_veab_values[i] * a + veab_values[i]*(1-a)

    return filtered_veab_values

### 以下のプログラムからスタート
class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')

        ## 各自由度ごとにデフォルトのPIDゲインを設定
        #ゲインチューニング用
        self.kp = self.declare_parameter('kp', [0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0]).value
        self.ki = self.declare_parameter('ki', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).value
        self.kd = self.declare_parameter('kd', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).value
        #ローパスフィルタ適用後(a=0.8)
        #self.kp = self.declare_parameter('kp', [0.32, 0.75, 0.3, 0.0, 0.35, 0.4, 0.3]).value
        #self.ki = self.declare_parameter('ki', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).value
        #self.kd = self.declare_parameter('kd', [0.3, 0.3, 0.3, 0.0, 0.1, 0.2, 0.1]).value

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

        ## veab1とveab2の値を保存するキューを設定
        self.veab1_queue = deque(maxlen=10)
        self.veab2_queue = deque(maxlen=10)

        ## 各要素(自由度)に対する前回の誤差と誤差の積分値
        self.previous_errors = [0.0] * 7
        self.integral = [0.0] * 7

        ## パブリッシャー作成
        self.publisher1 = self.create_publisher(UInt16MultiArray, '/VEAB1/desired', 10)
        self.publisher2 = self.create_publisher(UInt16MultiArray, '/VEAB2/desired', 10)

        ## タイマーを設定して一定間隔でcalculate_and_publishを実行
        self.timer_period = 0.0125  # 秒(1/0.0125=80 Hz)
        self.timer = self.create_timer(self.timer_period, self.calculate_and_publish)  # Nodeのメソッド

    ## 時系列のポテンショメータの値をキューに追加
    # 実現値をキューに追加
    def realized_callback(self, msg):
        self.realized_queue.append(msg.data)

    # 目標値をキューに追加
    def desired_callback(self, msg):
        self.desired_queue.append(msg.data)

    ## calculate_and_publish (VEAB1とVEAB2に送る圧力のPWMの値を計算しパブリッシュ)
    def calculate_and_publish(self):
        #実現値と目標値がパブリッシュされていなければ動かない
        if not self.realized_queue or not self.desired_queue:
            return

        # 最新のポテンショメータのキューに入っている値を取り出す
        realized_data = self.realized_queue[-1]
        desired_data = self.desired_queue[-1]

        # 誤差を計算(目標値-実現値)
        current_errors = [(desired_data[i] - realized_data[i]) for i in range(1, 8)] #ポテンショメータの実現値の配列のうち1番目から7番目までが各自由度に対応する

        # VEAB1とVEAB2に与える圧力差の初期化
        pid_outputs = []

        # VEAB1とVEAB2に与える圧力差の計算
        for i, error in enumerate(current_errors):
            #誤差の積分値計算
            self.integral[i] += error

            #誤差の微分値計算
            derivative = error - self.previous_errors[i]

            #PID制御計算
            pid_output = (self.kp[i] * error +
                          self.ki[i] * self.integral[i] +
                          self.kd[i] * derivative)
            
            # 各自由度ごとの圧力の正方向とポテンショメータの正方向の対応を整理(32行目のself.sine[i])
            pid_output = self.sine[i] * pid_output

            # PID制御計算された値をVEAB1とVEAB2に与える圧力差の配列に追加
            pid_outputs.append(pid_output)

            # 計算に用いた誤差を前回の誤差に変更(次の誤差計算用)
            self.previous_errors[i] = error

        # VEAB1とVEAB2に与えるPWMの値の初期化
        veab1_values = []
        veab2_values = []

        # VEAB1とVEAB2に与えるPWMの値を計算し格納
        for i, val in enumerate(pid_outputs):
            if i < 6:
                veab1_values.extend(self.calculate_veab_values(val, i))
            else:
                veab2_values.extend(self.calculate_veab_values(val, i))
    
        ## veab1とveab2の値をキューに追加
        self.veab1_queue.append(veab1_values)  
        self.veab2_queue.append(veab2_values)

        # 1ステップ前のveab1とveab2の値を取得
        previous_veab1_values = self.veab1_queue[-2] if len(self.veab1_queue) > 1 else veab1_values
        previous_veab2_values = self.veab2_queue[-2] if len(self.veab2_queue) > 1 else veab2_values
        
        # veab1_values と veab2_values にローパスフィルタを適用
        filtered_veab1_values = [int(value) for value in LPF_MAM(veab1_values, previous_veab1_values)]
        filtered_veab2_values = [int(value) for value in LPF_MAM(veab2_values, previous_veab2_values)]

        ## ローパスフィルタを適用したveab1とveab2の値をキューに追加
        self.veab1_queue.append(filtered_veab1_values)  
        self.veab2_queue.append(filtered_veab2_values)

        # publish_values関数を用いてVEAB1とVEAB2に与えるPWMの値をパブリッシュする
        self.publish_values(self.publisher1, filtered_veab1_values)
        self.publish_values(self.publisher2, filtered_veab2_values)

    # VEAB1とVEAB2に与えるPWMの値を計算(停止モードにおける両ポートの値を基準に足し引きを行う)
    def calculate_veab_values(self, difference, i):
        if i == 0:
            #腕の開閉
            veab1 = 137 + (difference / 2.0)
            veab2 = 119 - (difference / 2.0)
        elif i == 1:
            #腕の上下
            veab1 = 140 + (difference / 2.0)
            veab2 = 116 - (difference / 2.0)
        elif i == 2:
            #上腕の旋回
            veab1 = 128 + (difference / 2.0)
            veab2 = 128 - (difference / 2.0)
        elif i == 3:
            #肘の曲げ伸ばし
            veab1 = 130 + (difference / 2.0)
            veab2 = 126 - (difference / 2.0)
        elif i == 4:
            #前腕の旋回
            veab1 = 132 + (difference / 2.0)
            veab2 = 124 - (difference / 2.0)
        elif i == 5:
            #小指側伸縮
            veab1 = 135 + (difference / 2.0)
            veab2 = 121 - (difference / 2.0)
        else:
            #親指側伸縮
            veab1 = 130 + (difference / 2.0)
            veab2 = 126 - (difference / 2.0)
        
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
