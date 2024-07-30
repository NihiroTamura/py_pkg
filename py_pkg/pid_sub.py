import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

def cb(msg):
    node.get_logger().info("VEAB Values: %d" %msg.data)

rclpy.init()
node = Node("pid_sub")
sub = node.create_subscription(Int16, "VEAB", cb, 10)
rclpy.spin(node)