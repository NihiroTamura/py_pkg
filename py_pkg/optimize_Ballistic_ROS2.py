import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import UInt16MultiArray
import optuna
import random
import threading
import time


class OptimizationNode(Node):
    def __init__(self):
        super().__init__('optimization_node')
        self.executor = MultiThreadedExecutor()  # Executorの初期化
        self.publisher = self.create_publisher(UInt16MultiArray, '/board/sub', 10)
        self.pot_realized_board1 = [0] * 6  # /board1/pubからのデータ
        self.pot_realized_board2 = 0       # /board2/pubからのデータ
        self.target_ranges = [
            (190, 390),
            (286, 514),
            (86, 500),
            (123, 628),
            (98, 900),
            (48, 822),
            (1, 649)
        ]
        self.num_trials = 100
        self.num_repeats = 5
        self.best_params = None
        self.lock = threading.Lock()
        self.sse_accumulator = [0] * 7
        self.targets = [random.randint(r[0], r[1]) for r in self.target_ranges]
        self.ballistic_values = None  # 初回のみランダムに設定

        # 同期制御用
        self.condition = threading.Condition()
        self.board1_updated = False
        self.board2_updated = False

        # 購読スレッド
        self.board1_thread = None
        self.board2_thread = None
        self.stop_event = threading.Event()

    def start_subscribers(self):
        """トピック /board1/pub と /board2/pub の購読を開始 (マルチスレッド)"""
        self.stop_event.clear()
        self.board1_thread = threading.Thread(target=self.subscribe_to_board1, daemon=True)
        self.board2_thread = threading.Thread(target=self.subscribe_to_board2, daemon=True)
        self.board1_thread.start()
        self.board2_thread.start()
        self.get_logger().info("Subscribed to /board1/pub and /board2/pub in separate threads")

    def stop_subscribers(self):
        """トピック /board1/pub と /board2/pub の購読を停止"""
        self.stop_event.set()
        if self.board1_thread and self.board1_thread.is_alive():
            self.board1_thread.join()
        if self.board2_thread and self.board2_thread.is_alive():
            self.board2_thread.join()
        self.get_logger().info("Unsubscribed from /board1/pub and /board2/pub")

    def subscribe_to_board1(self):
        """トピック /board1/pub の購読 (マルチスレッド)"""
        subscription = self.create_subscription(UInt16MultiArray, '/board1/pub', self.callback_board1, 10)
        while not self.stop_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(subscription)

    def subscribe_to_board2(self):
        """トピック /board2/pub の購読 (マルチスレッド)"""
        subscription = self.create_subscription(UInt16MultiArray, '/board2/pub', self.callback_board2, 10)
        while not self.stop_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(subscription)

    def callback_board1(self, msg):
        with self.condition:
            # /board1/pub のデータを保存
            self.pot_realized_board1 = msg.data[:6]
            self.board1_updated = True
            self.get_logger().info("Received data from /board1/pub")
            self.try_calculate_sse()

    def callback_board2(self, msg):
        with self.condition:
            # /board2/pub のデータを保存
            self.pot_realized_board2 = msg.data[0]
            self.board2_updated = True
            self.get_logger().info("Received data from /board2/pub")
            self.try_calculate_sse()

    def try_calculate_sse(self):
        """/board1/pub と /board2/pub のデータが揃ったら二乗和誤差を計算"""
        if self.board1_updated and self.board2_updated:
            # 二乗和誤差の計算
            sse_per_dof = [(self.targets[i] - self.pot_realized_board1[i]) ** 2 for i in range(6)]
            sse_per_dof.append((self.targets[6] - self.pot_realized_board2) ** 2)
            for i in range(7):
                self.sse_accumulator[i] += sse_per_dof[i]

            self.get_logger().info(f"Updated SSE: {self.sse_accumulator}")

            # 状態をリセット
            self.board1_updated = False
            self.board2_updated = False

    def publish_targets(self, repeat):
        """目標値とBallisticパラメータの送信"""
        msg = UInt16MultiArray()
        if repeat == 1:
            self.get_logger().info("repeat1: 目標値とBallistic パラメータ送信")
        else:
            self.get_logger().info(f"repeat{repeat}: 目標値のみ更新して送信")
        
        msg.data = self.targets + self.ballistic_values
        self.publisher.publish(msg)

    def update_targets(self):
        """目標値の更新"""
        with self.lock:
            self.targets = [random.randint(r[0], r[1]) for r in self.target_ranges]
            self.get_logger().info(f"Targets updated: {self.targets}")

    def evaluate(self, trial):
        """Optunaの評価関数"""
        params = []
        for i in range(7):
            error_start = trial.suggest_int(f'error_start_{i+1}', 10, 1000)
            error_stop = trial.suggest_int(f'error_stop_{i+1}', 1, error_start - 1)
            omega = trial.suggest_int(f'omega_{i+1}', 10, 1000)
            delta_omega = trial.suggest_int(f'delta_omega_{i+1}', 1, omega - 1)
            omega_start = omega + (delta_omega // 2)
            omega_stop = omega - (delta_omega // 2)
            params.append((error_start, error_stop, omega_start, omega_stop))

        self.ballistic_values = [val for p in params for val in p]
        total_sse_sum = 0

        for repeat in range(1, self.num_repeats + 1):
            self.update_targets()
            self.publish_targets(repeat)

            # 購読を開始
            self.start_subscribers()

            # データ収集とSSE計算のため5秒待機
            time.sleep(5)

            # 購読を停止
            self.stop_subscribers()

            with self.lock:
                total_sse = sum(self.sse_accumulator)
                total_sse_sum += total_sse
                self.get_logger().info(f'Total SSE at repeat {repeat}: {total_sse}')
                # repeatごとに二乗和誤差の累積値をリセット
                self.sse_accumulator = [0] * 7

        return total_sse_sum

    def optimize(self):
        """最適化の実行"""
        study = optuna.create_study(direction='minimize')
        study.optimize(self.evaluate, n_trials=self.num_trials)
        self.best_params = study.best_params

        self.get_logger().info(f'最適化終了。試行回数中の最小評価関数値: {study.best_value:.2f}')
        for i in range(7):
            error_start = self.best_params[f'error_start_{i+1}']
            error_stop = self.best_params[f'error_stop_{i+1}']
            omega_start = self.best_params[f'omega_{i+1}'] + (self.best_params[f'delta_omega_{i+1}'] // 2)
            omega_stop = self.best_params[f'omega_{i+1}'] - (self.best_params[f'delta_omega_{i+1}'] // 2)
            self.get_logger().info(
                f'POT{i + 1}: error_start = {error_start}, error_stop = {error_stop}, '
                f'omega_start = {omega_start}, omega_stop = {omega_stop}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = OptimizationNode()
    try:
        node.optimize()
    finally:
        rclpy.shutdown()
