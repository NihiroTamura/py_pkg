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

        # 初期目標値（ロボットの初期状態）
        self.initial_goal = [260.0, 400.0, 180.0, 500.0, 600.0, 500.0, 500.0]
        self.current_goal = self.initial_goal.copy()

        self.slope = [0.0] * 7
        self.target_goal = [0.0] * 7
        self.reached = True  # 初回は入力を許可

        self.first_time = True  # 初回かどうかのフラグ

        # 自由度ごとの目標値制限
        self.min_limits = [210.0, 300.0, 100.0, 140.0, 120.0, 70.0, 20.0]
        self.max_limits = [370.0, 500.0, 480.0, 600.0, 880.0, 800.0, 620.0]

        # 自由度有効フラグ（True=動かす, False=固定）
        self.enable_dof = [True] * 7

        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def input_loop(self):
        while rclpy.ok():
            with self.lock:
                if not self.reached:
                    continue

                print("=== 新しい目標値の入力 ===")
                self.enable_dof = []
                self.slope = []
                self.target_goal = []

                for i in range(7):
                    # 有効かどうかの入力
                    while True:
                        flag = input(f"自由度{i+1}を動かしますか？ [y/n]: ").strip().lower()
                        if flag in ['y', 'n']:
                            self.enable_dof.append(flag == 'y')
                            break
                        print("⚠ y か n を入力してください。")

                    if self.enable_dof[i]:
                        # 傾きの入力（正の数のみ）
                        while True:
                            try:
                                s = float(input(f"自由度{i+1}の増加の傾き（0.01秒ごと）: "))
                                if s <= 0:
                                    print("⚠ 傾きは正の数を入力してください。")
                                    continue
                                self.slope.append(s)
                                break
                            except ValueError:
                                print("⚠ 数値を入力してください。")

                        # 目標値の入力（範囲内・正の数）
                        while True:
                            try:
                                prev_val = self.current_goal[i]
                                prompt = f"自由度{i+1}の最終目標値（前回: {prev_val}, 範囲: {self.min_limits[i]} ～ {self.max_limits[i]}）: "
                                t = float(input(prompt))
                                if t < 0:
                                    print("⚠ 目標値は正の数を入力してください。")
                                    continue
                                if t < self.min_limits[i] or t > self.max_limits[i]:
                                    print(f"⚠ 自由度{i+1}の目標値は範囲外です。{self.min_limits[i]}～{self.max_limits[i]} の範囲で入力してください。")
                                    continue
                                self.target_goal.append(t)
                                break
                            except ValueError:
                                print("⚠ 数値を入力してください。")
                    else:
                        # 無効自由度は傾き0、目標値は現在値を維持
                        self.slope.append(0.0)
                        self.target_goal.append(self.current_goal[i])

                print("すべての設定が完了しました。「パブリッシュしますか？」 [Enter] を押してください。")
                input()

                if self.first_time:
                    self.first_time = False

                self.reached = False
                print("▶ パブリッシュを開始します。")

    def timer_callback(self):
        with self.lock:
            if self.reached:
                return

            done = True
            for i in range(7):
                if not self.enable_dof[i]:
                    # 固定自由度は目標値変更なし
                    continue
                
                diff = self.target_goal[i] - self.current_goal[i]
                step = self.slope[i]
                if abs(diff) <= abs(step):
                    self.current_goal[i] = self.target_goal[i]
                else:
                    self.current_goal[i] += step if diff > 0 else -step
                    done = False

            msg = Float32MultiArray()
            # レイアウト設定
            msg.layout = MultiArrayLayout()
            dim = MultiArrayDimension()
            dim.label = 'example'
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
