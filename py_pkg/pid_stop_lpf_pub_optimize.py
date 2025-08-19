import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from std_msgs.msg import Float64, UInt16MultiArray
from collections import deque


class DisplayNode(Node):
    def __init__(self):
        super().__init__('pid_stop_lpf_pub_optimize')
        
        # パラメータの宣言 (初期値は0.0)
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('q', 0.0)
        
        # パラメータ変更のサービスサーバーを設定
        self.srv = self.create_service(SetParameters, '/pid_stop_lpf_pub_optimize/set_parameters', self.set_x_parameter)
        
        # パブリッシャーの設定 (Float64型のメッセージを送信)
        self.publisher_ = self.create_publisher(Float64, 'x_value_doubled', 10)

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

        # インスタンス変数としてパラメータを初期化
        self.x_value = 0.0
        self.y_value = 0.0
        self.z_value = 0.0
        self.q_value = 0.0
        
        # コールバック関数を呼び出すタイマー (1秒ごとに呼び出し)
        self.timer = self.create_timer(1.0, self.publish_doubled_value)
    
    ## 時系列のポテンショメータの値をキューに追加
    # 実現値をキューに追加
    def realized_callback(self, msg):
        self.realized_queue.append(msg.data)

    # 目標値をキューに追加
    def desired_callback(self, msg):
        self.desired_queue.append(msg.data)

    def set_x_parameter(self, request, response):
        for param in request.parameters:
            if param.name == 'x':
                # 受信した x の値をパラメータとして設定
                self.x_value = param.value.double_value
                self.set_parameters([Parameter('x', Parameter.Type.DOUBLE, self.x_value)])

                # x に2をかけた値をログに表示
                result_x = self.x_value * 2
                self.get_logger().info(f'設定されたxの値: {self.x_value}, xの2倍の値: {result_x}')
            elif param.name == 'y':
                # 受信したyの値をパラメータとして設定
                self.y_value = param.value.double_value
                self.set_parameters([Parameter('y', Parameter.Type.DOUBLE, self.y_value)])

                # yに2をかけた値をログに表示
                result_y = self.y_value * 2
                self.get_logger().info(f'設定されたyの値: {self.y_value}, yの2倍の値: {result_y}')
            elif param.name == 'z':
                # 受信したyの値をパラメータとして設定
                self.z_value = param.value.double_value
                self.set_parameters([Parameter('z', Parameter.Type.DOUBLE, self.z_value)])

                # yに2をかけた値をログに表示
                result_z = self.z_value * 2
                self.get_logger().info(f'設定されたzの値: {self.z_value}, zの2倍の値: {result_z}')
            else:
                # 受信したqの値をパラメータとして設定
                self.q_value = param.value.double_value
                self.set_parameters([Parameter('q', Parameter.Type.DOUBLE, self.q_value)])

                # yに2をかけた値をログに表示
                result_q = self.q_value * 2
                self.get_logger().info(f'設定されたqの値: {self.q_value}, qの2倍の値: {result_q}')
        
        return response

    def publish_doubled_value(self):
        """
        定期的に x_value の2倍の値をパブリッシャーで送信するコールバック関数
        """
        doubled_value_x = self.x_value * 2
        doubled_value_y = self.y_value * 2
        doubled_value_z = self.z_value * 2
        doubled_value_q = self.q_value * 2

        
        # パブリッシュする Float64 メッセージを作成して送信
        msg = Float64()
        msg.data = doubled_value_x
        self.publisher_.publish(msg)
        
        # 最新のポテンショメータのキューに入っている値を取り出す
        #realized_data = self.realized_queue[-1]
        desired_data = self.desired_queue[-1]

        # ログに送信した値を表示
        self.get_logger().info(f'パブリッシュされたxの2倍の値: {doubled_value_x}')
        self.get_logger().info(f'パブリッシュされたyの2倍の値: {doubled_value_y}')
        self.get_logger().info(f'パブリッシュされたzの2倍の値: {doubled_value_z}')
        self.get_logger().info(f'パブリッシュされたqの2倍の値: {doubled_value_q}')
        self.get_logger().info(f'目標値: {desired_data}')


def main(args=None):
    rclpy.init(args=args)
    display_node = DisplayNode()
    rclpy.spin(display_node)
    display_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
