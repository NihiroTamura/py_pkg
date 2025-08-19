import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension, MultiArrayLayout
import optuna
import random
import time

class OptimizationNode(Node):
    def __init__(self):
        super().__init__('optimization_node')

        #   publisherの作成
        self.publisher = self.create_publisher(UInt16MultiArray, '/board/sub', 10)

        #   目標値の範囲と目標値の決定
        self.pot_desired_range = [(190, 390), (286, 514), (86, 500), (123, 628), (98, 900), (48, 822), (1, 649)]
        self.pot_desired =  [random.randint(r[0], r[1]) for r in self.pot_desired_range]

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
        self.num_trials = 100

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
        msg = UInt16MultiArray()
        msg.layout = MultiArrayLayout()
        dim = MultiArrayDimension()
        dim.label = 'param'
        dim.size = 35
        dim.stride = 35
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
        self.pot_desired = [random.randint(r[0], r[1]) for r in self.pot_desired_range]

    def objective(self, trial):
        params = []

        #   各自由度で独自の探索範囲を設定
        error_range = [(50, 150), (50, 150), (50, 150), (50, 150), (50, 150), (50, 150), (50, 150)]
        delta_error_range = [(20, 50), (20, 50), (20, 50), (20, 50), (20, 50), (20, 50), (20, 50)]
        omega_range = [(800, 3000), (800, 3000), (800, 3000), (800, 3000), (800, 3000), (800, 3000), (800, 3000)]
        delta_omega_range = [(100, 800), (100, 800), (100, 800), (100, 800), (100, 800), (100, 800), (100, 800)]

        #   各自由度ごとに最適化対象パラメータとBallistic Modeパラメータの決定
        for i in range(7):
            while True:
                #   errorとdelta_errorの決定
                error_min, error_max = error_range[i]
                error = trial.suggest_int(f'error_{i+1}', error_min, error_max)

                delta_error_min, delta_error_max = delta_error_range[i]
                delta_error = trial.suggest_int(f'delta_error_{i+1}', delta_error_min, delta_error_max)
                    
                #   error_startとerror_stopの計算
                error_start = error + (delta_error // 2)
                error_stop = error - (delta_error // 2)

                #   制約条件を満たすか確認
                if error_stop > 0 and error_start > error_stop:
                    break
            
            while True:
                #   omegaとdelta_omegaの決定
                omega_min, omega_max = omega_range[i]
                omega = trial.suggest_int(f'omega_{i+1}', omega_min, omega_max)

                delta_omega_min, delta_omega_max = delta_omega_range[i]
                delta_omega = trial.suggest_int(f'delta_omega_{i+1}', delta_omega_min, delta_omega_max)

                #   omega_startとomega_stopの計算
                omega_start = omega + (delta_omega // 2)
                omega_stop = omega - (delta_omega // 2)

                #   制約条件を満たすか確認
                if omega_stop > 0 and omega_start > omega_stop:
                    break

            #   探索結果をリストに追加
            params.append((error_start, error_stop, omega_start, omega_stop))
        
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
            error_start = self.best_params[f'error_{i+1}'] + (self.best_params[f'delta_error_{i+1}'] // 2)
            error_stop = self.best_params[f'error_{i+1}'] - (self.best_params[f'delta_error_{i+1}'] // 2)
            omega_start = self.best_params[f'omega_{i+1}'] + (self.best_params[f'delta_omega_{i+1}'] // 2)
            omega_stop = self.best_params[f'omega_{i+1}'] - (self.best_params[f'delta_omega_{i+1}'] // 2)
            self.get_logger().info(
                f'POT{i + 1}: error_start = {error_start}, error_stop = {error_stop}, '
                f'omega_start = {omega_start}, omega_stop = {omega_stop}'
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