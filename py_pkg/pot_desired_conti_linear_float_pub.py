import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import threading

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/board_float/sub', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz
        self.lock = threading.Lock()

        # 初期目標値
        self.initial_goal = [260.0, 400.0, 180.0, 500.0, 600.0, 500.0, 500.0]
        self.current_goal = self.initial_goal.copy()

        # 自由度ごとの目標値制限
        self.min_limits = [210.0, 300.0, 100.0, 140.0, 120.0, 70.0, 20.0]
        self.max_limits = [370.0, 500.0, 480.0, 600.0, 880.0, 800.0, 620.0]

        # 10回分の目標値と傾き設定
        # それぞれ「[目標値のリスト]」「[傾きのリスト]」
        self.goal_sequence = [
            [289.0, 427.0, 324.0, 188.0, 130.0, 118.0, 355.0],
            [344.0, 323.0, 164.0, 460.0, 325.0, 572.0, 95.0],
            [219.0, 486.0, 122.0, 239.0, 627.0, 732.0, 297.0],
            [248.0, 315.0, 304.0, 556.0, 122.0, 427.0, 76.0],
            [253.0, 410.0, 216.0, 586.0, 746.0, 641.0, 144.0],
            [328.0, 398.0, 176.0, 144.0, 309.0, 658.0, 536.0],
            [211.0, 409.0, 353.0, 600.0, 690.0, 563.0, 568.0],
            [212.0, 346.0, 378.0, 211.0, 459.0, 634.0, 283.0],
            [266.0, 328.0, 439.0, 459.0, 595.0, 302.0, 546.0],
            [322.0, 474.0, 125.0, 412.0, 203.0, 341.0, 550.0]
        ]

        self.slope_sequence = [
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        ] * 10  # 全て同じ傾きならこう書ける

        self.current_step = -1  # まだ開始していない
        self.reached = True

        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def input_loop(self):
        while rclpy.ok():
            if self.current_step >= len(self.goal_sequence) - 1:
                print("✔ 全ての目標値を送信し終えました。終了するにはCtrl+Cを押してください。")
                break

            input(f"\n次の目標値を送信するにはEnterを押してください。（残り {len(self.goal_sequence) - (self.current_step + 1)} 回）")

            with self.lock:
                self.current_step += 1
                self.target_goal = self.goal_sequence[self.current_step]
                self.slope = self.slope_sequence[self.current_step]
                self.reached = False
                print(f"▶ {self.current_step+1}回目のパブリッシュを開始します。")
                print(f"  目標値: {self.target_goal}")
                print(f"  傾き: {self.slope}")

    def timer_callback(self):
        with self.lock:
            if self.reached or self.current_step == -1:
                return

            done = True
            for i in range(7):
                diff = self.target_goal[i] - self.current_goal[i]
                step = self.slope[i]
                if abs(diff) <= abs(step):
                    self.current_goal[i] = self.target_goal[i]
                else:
                    self.current_goal[i] += step if diff > 0 else -step
                    done = False

            # メッセージ作成
            msg = Float32MultiArray()
            msg.layout = MultiArrayLayout()
            dim = MultiArrayDimension()
            dim.label = 'goal'
            dim.size = 7
            dim.stride = 7
            msg.layout.dim = [dim]
            msg.layout.data_offset = 0
            msg.data = self.current_goal

            self.publisher_.publish(msg)
            self.get_logger().info(f'Published goal: {msg.data}')

            if done:
                print("✔ すべての自由度が目標値に到達しました。")
                self.reached = True

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
