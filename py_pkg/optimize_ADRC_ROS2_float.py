import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import optuna
import random
import time
import numpy as np

class OptimizationNode(Node):
    def __init__(self):
        super().__init__('optimization_node_float')

        #   publisherの作成
        self.publisher = self.create_publisher(Float32MultiArray, '/board_float/sub', 10)

        #   目標値の範囲と目標値の決定
        self.pot_desired_range = [(200, 350), (285, 480), (85, 470), (140, 580), (100, 880), (60, 790), (2, 620)]
        self.pot_desired =  [float(random.randint(r[0], r[1])) for r in self.pot_desired_range]

        #   ポテンショメータの値の配列
        self.pot_realized_board1 = [0] * 6
        self.pot_realized_board2 = 0

        #   ポテンショメータの角速度の配列
        #self.omega_board1 = [0] * 6
        #self.omega_board2 = 0

        #   subscribeの初期化および開始と停止のフラグ
        self.sub_board1 = None
        self.sub_board2 = None
        self.is_subscribing = False

        #   各自由度ごとのSSE（二乗和誤差）
        self.sse_accumulator = [0] * 7

        #   各自由度ごとの角速度の積分値
        #self.sum_omega = [0] * 7

        #   目標値を切り替えるrepeat回数と最適化のtrial回数
        self.num_repeats = 5
        self.num_trials = 300

        #   PDゲインの初期化
        self.PD_values = None

        #   最適化パラメータの初期化
        self.best_params = None
    
    def start_subscribers(self):
        if not self.is_subscribing:
            #   subscriberの作成
            self.sub_board1 = self.create_subscription(
                UInt16MultiArray, '/board1/pub', self.board1_callback, 10)
            self.sub_board2 = self.create_subscription(
                UInt16MultiArray, '/board2/pub', self.board2_callback, 10)
            
            #   subscribe開始フラグに変更
            self.is_subscribing = True
            self.get_logger().info('Subscribed to topic.')

    def stop_subscribers(self):
        if self.is_subscribing:
            #   subscriberの削除
            if self.sub_board1 is not None:
                self.destroy_subscription(self.sub_board1)
                self.sub_board1 = None
            
            if self.sub_board2 is not None:
                self.destroy_subscription(self.sub_board2)
                self.sub_board2 = None

            self.is_subscribing = False

            self.get_logger().info('Unsubscribed from topic.')
    
    def board1_callback(self, msg):
        #   /board1/pubのsubscribe
        self.pot_realized_board1 = msg.data[:6]
        #self.omega_board1 = msg.data[6:12]

        #   自由度ごとに「SSE」,「角速度の積分値」を計算
        for i in range(6):
            #   SSE
            sse_per_dof = (self.pot_desired[i] - self.pot_realized_board1[i]) ** 2
            self.sse_accumulator[i] += sse_per_dof

            #   角速度の積分値
            #self.sum_omega[i] += self.omega_board1[i]

        #self.get_logger().info(f"Received board1: {self.pot_realized_board1}")
    
    def board2_callback(self, msg):
        #   /board2/pubのsubscribe
        self.pot_realized_board2 = msg.data[0]
        #self.omega_board2 = msg.data[6]

        #   自由度ごとに「SSE」,「角速度の積分値」、「PID&BallisticMode回数」を計算
        #   SSE
        sse_per_dof_6 = (self.pot_desired[6] - self.pot_realized_board2) ** 2
        self.sse_accumulator[6] += sse_per_dof_6

        #   角速度の積分値
        #self.sum_omega[6] += self.omega_board2

        #self.get_logger().info(f"Received board2: {self.pot_realized_board2}")

    def publish_function(self, repeat):
        msg = Float32MultiArray()
        msg.layout = MultiArrayLayout()
        dim = MultiArrayDimension()
        dim.label = 'param'
        dim.size = 21
        dim.stride = 21
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0

        if repeat == 1:
            self.get_logger().info("repeat1: 目標値とPDゲイン送信")
        else:
            self.get_logger().info(f"repeat{repeat}: 目標値のみ更新して送信")
        
        #   目標値とPDゲインのpublish
        msg.data = self.pot_desired + self.PD_values
        self.publisher.publish(msg)

    def update_pot_desired(self):
        #   目標値をランダムに変更
        self.pot_desired = [float(random.randint(r[0], r[1])) for r in self.pot_desired_range]

    def objective(self, trial):
        params = []

        #   各自由度で独自の探索範囲を設定
        kp_range = [(500, 5000), (500, 5000), (500, 5000), (200, 2000), (500, 5000), (150, 1500), (150, 1500)]
        kd_range = [(10, 500), (10, 500), (10, 500), (0, 200), (10, 500), (0, 50), (0, 50)]

        #   各自由度ごとに最適化対象パラメータの決定
        for i in range(7):
            kp_min, kp_max = kp_range[i]
            kp = trial.suggest_float(f'kp_{i+1}', kp_min, kp_max)

            kd_min, kd_max = kd_range[i]
            kd = trial.suggest_float(f'kd_{i+1}', kd_min, kd_max)

            #   探索結果をリストに追加
            params.append((kp, kd))
        
        #   PDゲインのタプルを解放
        self.PD_values = [val for p in params for val in p]

        # 評価関数の初期化
        total_sse_sum = 0

        for repeat in range(1, self.num_repeats + 1):
            # SSE計算の初期化
            total_sse = 0

            #   角速度の積分値計算の初期化
            #total_omega = [0] * 7
            #omega_penalty = [0] * 7
            #total_omega_penalty = 0
            #threshold_omega = [200000, 250000, 440000, 550000, 880000, 880000, 660000]

            #   目標値を更新
            self.update_pot_desired()

            #   目標値とPDゲインのpublish
            self.publish_function(repeat)

            #   subscribeの開始
            self.start_subscribers()

            #   メッセージの受信
            self.start_time = time.time()
            while True:
                rclpy.spin_once(self, timeout_sec=0.01)
                self.current_time = time.time()

                if (self.current_time - self.start_time >= 10):
                    break

            #   subscribeの停止
            self.stop_subscribers()

            #   SSEを計算
            total_sse = sum(self.sse_accumulator)
            total_sse_sum += total_sse

            #   角速度の積分値を計算
            #for i in range(7):
                #total_omega[i] = self.sum_omega[i] - threshold_omega[i]
                #omega_penalty[i] = max(0, total_omega[i])
            #total_omega_penalty = sum(omega_penalty)
           # total_sse_sum += total_omega_penalty*10000000

            self.get_logger().info(f'Total SSE at repeat {repeat}: {total_sse_sum}')

            #   repeatごとに各自由度ごとの「SSE」,「角速度の積分値」を初期化
            self.sse_accumulator = [0] * 7
            #self.sum_omega = [0] * 7
        
        return total_sse_sum

    def sampler_monitor(self, study, trial):
        current_sampler = study.sampler
        print(f"試行{trial.number}:使用中のサンプラー → {current_sampler}")


    def optimize(self):
        sampler = optuna.samplers.CmaEsSampler()
        study = optuna.create_study(sampler = sampler, direction='minimize')
        study.optimize(self.objective, n_trials = self.num_trials, callbacks=[self.sampler_monitor])

        #最適化パラメータの表示
        self.best_params = study.best_params
        self.get_logger().info(f'最適化終了。試行回数中の最小評価関数値: {study.best_value:.2f}')
        self.get_logger().info(f'最適な試行回数（trial number）は: {study.best_trial.number}')
        for i in range(7):
            kp = self.best_params[f'kp_{i+1}']
            kd = self.best_params[f'kd_{i+1}']

            self.get_logger().info(
                f'POT{i + 1}: kp = {kp}, kd = {kd}'
            )

def main(args=None):
    #   ROS通信の初期化
    rclpy.init(args=args)
    #   インスタンスの生成
    node = OptimizationNode()
    try:
        node.optimize()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()