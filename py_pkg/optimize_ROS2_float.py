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
        self.pot_desired_range = [(190, 390), (286, 514), (86, 500), (123, 628), (98, 900), (48, 822), (1, 649)]
        self.pot_desired =  [float(random.randint(r[0], r[1])) for r in self.pot_desired_range]

        #   ポテンショメータの値の配列
        self.pot_realized_board1 = [0] * 6
        self.pot_realized_board2 = 0

        #   subscribeの初期化および開始と停止のフラグ
        self.sub_board1 = None
        self.sub_board2 = None
        self.is_subscribing = False

        #   各自由度ごとのSSE（二乗和誤差）
        self.sse_accumulator = [0] * 7

        #   目標値を切り替えるrepeat回数と最適化のtrial回数
        self.num_repeats = 5
        self.num_trials = 150

        #   Ballistic Modeパラメータの初期化
        self.ballistic_values = None

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

        #   自由度ごとにSSEを計算
        for i in range(6):
            sse_per_dof = (self.pot_desired[i] - self.pot_realized_board1[i]) ** 2
            self.sse_accumulator[i] += sse_per_dof

        #self.get_logger().info(f"Received board1: {self.sse_accumulator}")
    
    def board2_callback(self, msg):
        #   /board2/pubのsubscribe
        self.pot_realized_board2 = msg.data[0]

        #   自由度ごとにSSEを計算
        sse_per_dof_6 = (self.pot_desired[6] - self.pot_realized_board2) ** 2
        self.sse_accumulator[6] += sse_per_dof_6

        #self.get_logger().info(f"Received board2: {self.sse_accumulator[6]}")

    def publish_function(self, repeat):
        msg = Float32MultiArray()
        msg.layout = MultiArrayLayout()
        dim = MultiArrayDimension()
        dim.label = 'param'
        dim.size = 63
        dim.stride = 63
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0

        if repeat == 1:
            self.get_logger().info("repeat1: 目標値とBallistic Modeパラメータ送信")
        else:
            self.get_logger().info(f"repeat{repeat}: 目標値のみ更新して送信")
        
        #   目標値とBallistic Modeパラメータのpublish
        msg.data = self.pot_desired + self.ballistic_values
        self.publisher.publish(msg)

    def update_pot_desired(self):
        #   目標値をランダムに変更
        self.pot_desired = [float(random.randint(r[0], r[1])) for r in self.pot_desired_range]

    def objective(self, trial):
        params = []

        #   各自由度で独自の探索範囲を設定
        param_func_AE_range = [(0, 0.3), (0, 0.3), (0, 0.3), (0, 0.3), (0, 0.3), (0, 0.3), (0, 0.3)]
        param_func_AdeltaE_range = [(0, 0.6), (0, 0.6), (0, 0.6), (0, 0.6), (0, 0.6), (0, 0.6), (0, 0.6)]
        param_func_AV_range = [(0, 3), (0, 3), (0, 3), (0, 3), (0, 3), (0, 3), (0, 3)]
        param_func_AdeltaV_range = [(-3, 3), (-3, 3), (-3, 3), (-3, 3), (-3, 3), (-3, 3), (-3, 3)]

        param_func_BE_range = [(0, 0.3), (0, 0.3), (0, 0.3), (0, 0.3), (0, 0.3), (0, 0.3), (0, 0.3)]
        param_func_BdeltaE_range = [(0, 0.6), (0, 0.6), (0, 0.6), (0, 0.6), (0, 0.6), (0, 0.6), (0, 0.6)]
        param_func_BV_range = [(0, 3), (0, 3), (0, 3), (0, 3), (0, 3), (0, 3), (0, 3)]
        param_func_BdeltaV_range = [(-3, 3), (-3, 3), (-3, 3), (-3, 3), (-3, 3), (-3, 3), (-3, 3)]

        #   各自由度ごとに最適化対象パラメータの決定
        for i in range(7):
            #   A*POT_desired + B*POT_realized = P
            #   Aについて
            param_func_AE_min, param_func_AE_max = param_func_AE_range[i]
            param_func_AE = trial.suggest_float(f'param_func_AE_{i+1}', param_func_AE_min, param_func_AE_max, step=0.001)

            param_func_AdeltaE_min, param_func_AdeltaE_max = param_func_AdeltaE_range[i]
            param_func_AdeltaE = trial.suggest_float(f'param_func_AdeltaE_{i+1}', param_func_AdeltaE_min, param_func_AdeltaE_max, step=0.001)

            param_func_AV_min, param_func_AV_max = param_func_AV_range[i]
            param_func_AV = trial.suggest_float(f'param_func_AV_{i+1}', param_func_AV_min, param_func_AV_max)

            param_func_AdeltaV_min, param_func_AdeltaV_max = param_func_AdeltaV_range[i]
            param_func_AdeltaV = trial.suggest_float(f'param_func_AdeltaV_{i+1}', param_func_AdeltaV_min, param_func_AdeltaV_max, step=0.001)

            #   Bについて
            param_func_BE_min, param_func_BE_max = param_func_BE_range[i]
            param_func_BE = trial.suggest_float(f'param_func_BE_{i+1}', param_func_BE_min, param_func_BE_max, step=0.001)

            param_func_BdeltaE_min, param_func_BdeltaE_max = param_func_BdeltaE_range[i]
            param_func_BdeltaE = trial.suggest_float(f'param_func_BdeltaE_{i+1}', param_func_BdeltaE_min, param_func_BdeltaE_max, step=0.001)

            param_func_BV_min, param_func_BV_max = param_func_BV_range[i]
            param_func_BV = trial.suggest_float(f'param_func_BV_{i+1}', param_func_BV_min, param_func_BV_max, step=0.001)

            param_func_BdeltaV_min, param_func_BdeltaV_max = param_func_BdeltaV_range[i]
            param_func_BdeltaV = trial.suggest_float(f'param_func_BdeltaV_{i+1}', param_func_BdeltaV_min, param_func_BdeltaV_max, step=0.001)

            #   探索結果をリストに追加
            params.append((param_func_AE, param_func_AdeltaE, param_func_AV, param_func_AdeltaV, param_func_BE, param_func_BdeltaE, param_func_BV, param_func_BdeltaV))
        
        #   Ballistic Modeパラメータのタプルを解放
        self.ballistic_values = [val for p in params for val in p]

        # SSE 計算の初期化
        total_sse_sum = 0

        for repeat in range(1, self.num_repeats + 1):
            # SSE 計算の初期化
            total_sse = 0

            #   目標値を更新
            self.update_pot_desired()

            #   目標値とBallistic Modeパラメータのpublish
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
            self.get_logger().info(f'Total SSE at repeat {repeat}: {total_sse_sum}')

            #   repeatごとに各自由度ごとのSSEを初期化
            self.sse_accumulator = [0] * 7
        
        return total_sse_sum

    def optimize(self):
        sampler = optuna.samplers.CmaEsSampler()
        study = optuna.create_study(sampler = sampler, direction='minimize')
        study.optimize(self.objective, n_trials = self.num_trials)

        #最適化パラメータの表示
        self.best_params = study.best_params
        self.get_logger().info(f'最適化終了。試行回数中の最小評価関数値: {study.best_value:.2f}')
        self.get_logger().info(f'最適な試行回数（trial number）は: {study.best_trial.number}')
        for i in range(7):
            param_func_AE = np.round(self.best_params[f'param_func_AE_{i+1}'], 3)
            param_func_AdeltaE = np.round(self.best_params[f'param_func_AdeltaE_{i+1}'], 3)
            param_func_AV = np.round(self.best_params[f'param_func_AV_{i+1}'], 3)
            param_func_AdeltaV = np.round(self.best_params[f'param_func_AdeltaV_{i+1}'], 3)

            param_func_BE = np.round(self.best_params[f'param_func_BE_{i+1}'], 3)
            param_func_BdeltaE = np.round(self.best_params[f'param_func_BdeltaE_{i+1}'], 3)
            param_func_BV = np.round(self.best_params[f'param_func_BV_{i+1}'], 3)
            param_func_BdeltaV = np.round(self.best_params[f'param_func_BdeltaV_{i+1}'], 3)

            self.get_logger().info(
                f'POT{i + 1}: param_func_AE = {param_func_AE}, param_func_AdeltaE = {param_func_AdeltaE}, '
                f'param_func_AV = {param_func_AV}, param_func_AdeltaV = {param_func_AdeltaV}, '
                f'param_func_BE = {param_func_BE}, param_func_BdeltaE = {param_func_BdeltaE}, '
                f'param_func_BV = {param_func_BV}, param_func_BdeltaV = {param_func_BdeltaV}'
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