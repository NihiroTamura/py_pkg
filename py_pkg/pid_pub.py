import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        
        self.subscription_realized = self.create_subscription(
            UInt16MultiArray,
            '/POT/realized',
            self.realized_callback,
            10
        )
        self.subscription_desired = self.create_subscription(
            UInt16MultiArray,
            '/POT/desired',
            self.desired_callback,
            10
        )
        
        self.publisher_veab1 = self.create_publisher(UInt16MultiArray, '/VEAB1/desired', 10)
        self.publisher_veab2 = self.create_publisher(UInt16MultiArray, '/VEAB2/desired', 10)
        
        self.realized_data = None
        self.desired_data = None
        self.prev_error = [0] * 12
        self.integral = [0] * 12

        self.timer = self.create_timer(0.1, self.timer_callback)

    def realized_callback(self, msg):
        self.realized_data = msg.data

    def desired_callback(self, msg):
        self.desired_data = msg.data

    def timer_callback(self):
        if self.realized_data is not None and self.desired_data is not None:
            error = [desired - realized for realized, desired in zip(self.realized_data, self.desired_data)]
            self.integral = [i + e * 0.1 for i, e in zip(self.integral, error)]
            derivative = [(e - pe) / 0.1 for e, pe in zip(error, self.prev_error)]
            self.prev_error = error
            
            command = [int(self.kp * e + self.ki * i + self.kd * d) for e, i, d in zip(error, self.integral, derivative)]
            
            msg = UInt16MultiArray()
            msg.layout.dim.append(UInt16MultiArray().layout.dim[0])
            msg.layout.dim[0].size = 12
            msg.layout.dim[0].stride = 12
            msg.data = command
            
            self.publisher_veab1.publish(msg)
            self.publisher_veab2.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
