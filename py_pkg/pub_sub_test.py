import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
import optuna
import random
import time

class OptimizationNode(Node):
    def __init__(self):
        super().__init__('optimization_node')

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
        if self.is_subscribing and self.sub_board1 is not None and self.sub_board2 is not None:
            self.destroy_subscription(self.sub_board1)
            self.destroy_subscription(self.sub_board2)

            self.sub_board1 = None
            self.sub_board2 = None
            self.is_subscribing = False

            self.get_logger().info('Unsubscribed from topic.')
    
    def board1_callback(self, msg):
        self.pot_realized_board1 = msg.data[:6]
        self.get_logger().info(pot_realized_board1)
    
    def board2_callback(self, msg):
        self.pot_realized_board2 = msg.data[0]
        self.get_logger().info(pot_realized_board2)

    def optimize(self):
        for i in range(7):
            #   subscribeの開始
            self.start_subscribers()

            #   メッセージの受信
            time.sleep(5)

            #   subscribeの停止
            self.stop_subscribers()

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