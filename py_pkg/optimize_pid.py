import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from std_msgs.msg import UInt16MultiArray, MultiArrayLayout, MultiArrayDimension
import optuna
import time
import random


class OptimizerNode(Node):
    def __init__(self):
        super().__init__('optimizer_pid')
        
        # SetParameters サービスクライアントを作成
        self.cli = self.create_client(SetParameters, '/pid_stop_lpf_pub_optimize/set_parameters')
        
        # サービスが利用可能になるまで待機
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('サービスの待機中...')

        # UInt16MultiArray型のパブリッシャーを作成
        self.publisher_desired = self.create_publisher(UInt16MultiArray, '/POT/desired', 10)
        
        # CMA-ESで最適化を実行
        self.optimize()
    
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
        self.get_logger().info(f'パブリッシュ: {msg.data}')
    
    def desired_random(self):
        # パブリッシュのループ
        count_desired_value = 0
        while count_desired_value < 5:
            open_and_close = random.randint(223, 482)
            up_down = random.randint(350, 580)
            upperarm_rotation = random.randint(95, 605)
            elbow = random.randint(144, 740)
            forearm_rotation = random.randint(111, 962)
            wrist_littlefinger = random.randint(62, 895)
            wrist_thumb = random.randint(0, 740)

            random_data_desired = [0, open_and_close, up_down, upperarm_rotation, elbow, forearm_rotation, wrist_littlefinger, wrist_thumb, 0, 0, 0, 0]

            self.publish_values(self.publisher_desired, random_data_desired)

            # 5秒待機してから再度パブリッシュ
            time.sleep(5)
                
            count_desired_value += 1

    def objective(self, trial):
        # 最適化対象パラメータの範囲を定義
        x = trial.suggest_float('x', 0, 20)
        y = trial.suggest_float('y', 0, 20)
        z = trial.suggest_float('y', 0, 20)
        q = trial.suggest_float('y', 0, 20)
            
        # 評価関数 (最小化対象)
        evaluation = (x - 2)**2 + (y + 3)**2 + (z + 1)**2 + (q + 3)**2

        # ここで選ばれた x を display_node に送信
        self.send_parameter_to_display_node(x, y, z, q)

        # ランダムな目標値を作成しパブリッシュ
        self.desired_random()
            
        return evaluation
    
    def send_parameter_to_display_node(self, x_value, y_value, z_value, q_value):
        # パラメータリクエストを作成
        param_x = Parameter('x', Parameter.Type.DOUBLE, x_value)
        param_msg_x = param_x.to_parameter_msg()

        param_y = Parameter('y', Parameter.Type.DOUBLE, y_value)
        param_msg_y = param_y.to_parameter_msg()

        param_z = Parameter('z', Parameter.Type.DOUBLE, z_value)
        param_msg_z = param_z.to_parameter_msg()

        param_q = Parameter('q', Parameter.Type.DOUBLE, q_value)
        param_msg_q = param_q.to_parameter_msg()



        # リクエストに含める
        req = SetParameters.Request()
        req.parameters = [param_msg_x, param_msg_y, param_msg_z, param_msg_q]
        
        # サービスコールを実行
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'パラメータ送信成功: x={x_value}')
        else:
            self.get_logger().error('パラメータ送信失敗')

    def optimize(self):
        # CMA-ESアルゴリズムで最適化
        study = optuna.create_study(sampler=optuna.samplers.CmaEsSampler(), direction='minimize')
        study.optimize(self.objective, n_trials=10)
        
        # 最終的な最適化された x を取得
        best_params = study.best_params
        best_x, best_y, best_z, best_q = best_params['x'], best_params['y'], best_params['z'], best_params['q']
        
        # 最適なパラメータを display_node に送信
        self.send_parameter_to_display_node(best_x, best_y, best_z, best_q)
        self.get_logger().info(f'最適化完了: x={best_x}, y={best_y}, z={best_z}, q={best_q}')
        

def main(args=None):
    rclpy.init(args=args)
    optimizer_node = OptimizerNode()
    rclpy.spin(optimizer_node)
    optimizer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
