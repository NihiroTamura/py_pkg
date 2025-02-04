import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.context import Context
from std_msgs.msg import UInt16MultiArray
import optuna
import random
import threading
import time


class OptimizationNode(Node):
    def __init__(self):
        super().__init__('optimization_node')
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

        # スレッド制御用
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
        """トピック /board1/pub の購読 (独立ノード)"""
        context = Context()
        rclpy.init(context=context)
        board1_node = rclpy.create_node('board1_node', context=context)
        subscription = board1_node.create_subscription(UInt16MultiArray, '/board1/pub', self.callback_board1, 10)
        executor = MultiThreadedExecutor(context=context)
        executor.add_node(board1_node)
        try:
            while not self.stop_event.is_set():
                executor.spin_once(timeout_sec=0.1)
        finally:
            board1_node.destroy_subscription(subscription)
            board1_node.destroy_node()
            rclpy.shutdown(context=context)

    def subscribe_to_board2(self):
        """トピック /board2/pub の購読 (独立ノード)"""
        context = Context()
        rclpy.init(context=context)
        board2_node = rclpy.create_node('board2_node', context=context)
        subscription = board2_node.create_subscription(UInt16MultiArray, '/board2/pub', self.callback_board2, 10)
        executor = MultiThreadedExecutor(context=context)
        executor.add_node(board2_node)
        try:
            while not self.stop_event.is_set():
                executor.spin_once(timeout_sec=0.1)
        finally:
            board2_node.destroy_subscription(subscription)
            board2_node.destroy_node()
            rclpy.shutdown(context=context)

    def callback_board1(self, msg):
        with self.lock:
            self.pot_realized_board1 = msg.data[:6]
            #self.get_logger().info("Received data from /board1/pub")
            sse_per_dof = [(self.targets[i] - self.pot_realized_board1[i]) ** 2 for i in range(6)]
            for i in range(6):
                self.sse_accumulator[i] += sse_per_dof[i]
            #self.get_logger().info(f"SSE for /board1/pub updated: {self.sse_accumulator[:6]}")

    def callback_board2(self, msg):
        with self.lock:
            self.pot_realized_board2 = msg.data[0]
            #self.get_logger().info("Received data from /board2/pub")
            sse_per_dof_6 = (self.targets[6] - self.pot_realized_board2) ** 2
            self.sse_accumulator[6] += sse_per_dof_6
            #self.get_logger().info(f"SSE for /board2/pub updated: {self.sse_accumulator[6]}")

    def publish_targets(self, repeat):
        msg = UInt16MultiArray()
        if repeat == 1:
            self.get_logger().info("repeat1: 目標値とBallistic パラメータ送信")
        else:
            self.get_logger().info(f"repeat{repeat}: 目標値のみ更新して送信")
        msg.data = self.targets + self.ballistic_values
        self.publisher.publish(msg)

    def update_targets(self):
        with self.lock:
            self.targets = [random.randint(r[0], r[1]) for r in self.target_ranges]
            #self.get_logger().info(f"Targets updated: {self.targets}")

    def evaluate(self, trial):
        params = []

        # 各要素で独自の探索範囲を設定
        error_start_ranges = [
            (2, 200), (2, 228), (2, 414), (2, 500), (2, 800), (2, 774), (2, 648)
        ]
        omega_ranges = [
            (1, 8500), (1, 10000), (1, 13500), (1, 17000), (1, 26000), (1, 30000), (1, 33000)
        ]

        for i in range(7):
            # error_start を error_start_ranges から決定
            error_start_min, error_start_max = error_start_ranges[i]
            error_start = trial.suggest_int(f'error_start_{i+1}', error_start_min, error_start_max)

            # error_stop を error_start に基づいて決定
            error_stop = trial.suggest_int(f'error_stop_{i+1}', 1, error_start - 1)

            # omega を omega_ranges から決定
            omega_min, omega_max = omega_ranges[i]
            omega = trial.suggest_int(f'omega_{i+1}', omega_min, omega_max)

            # delta_omega を omega に基づいて決定
            delta_omega = trial.suggest_int(f'delta_omega_{i+1}', 1, 2 * omega - 1)

            # omega_start と omega_stop を計算
            omega_start = omega + (delta_omega // 2)
            omega_stop = omega - (delta_omega // 2)

            # 探索結果をリストに追加
            params.append((error_start, error_stop, omega_start, omega_stop))

        # Ballistic パラメータの計算
        self.ballistic_values = [val for p in params for val in p]

        # SSE 計算の初期化
        total_sse_sum = 0

        # 繰り返し最適化試行
        for repeat in range(1, self.num_repeats + 1):
            self.update_targets()
            self.publish_targets(repeat)

            self.start_subscribers()
            time.sleep(5)  # 実験データを取得するまで待機
            self.stop_subscribers()

            with self.lock:
                total_sse = sum(self.sse_accumulator)
                total_sse_sum += total_sse
                self.get_logger().info(f'Total SSE at repeat {repeat}: {total_sse_sum}')
                self.sse_accumulator = [0] * 7

        return total_sse_sum



    def optimize(self):
        study = optuna.create_study(direction='minimize')
        study.optimize(self.evaluate, n_trials=self.num_trials)
        self.best_params = study.best_params

        self.get_logger().info(f'最適化終了。試行回数中の最小評価関数値: {study.best_value:.2f}')
        self.get_logger().info(f'最適な試行回数（trial number）は: {study.best_trial.number}')
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
