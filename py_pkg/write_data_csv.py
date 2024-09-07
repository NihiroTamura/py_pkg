import rclpy
from rclpy.node import Node
import csv
import psutil
import time
from std_msgs.msg import UInt16MultiArray

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        
        # 購読するトピック名とメッセージ型を指定
        self.subscription = self.create_subscription(
            UInt16MultiArray,
            '/your_topic_name',
            self.listener_callback,
            10  # キューサイズ
        )
        self.subscription  # prevent unused variable warning

        # CSVファイルを開き、書き込み準備
        self.csv_file = open('/home/nihiro/ArmRobot/src/py_pkg/py_pkg/output.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # ヘッダー行を書き込む（必要に応じて変更）
        self.csv_writer.writerow(['timestamp'] + [f'data_{i}' for i in range(10)])  # 配列のサイズに合わせる

    def listener_callback(self, msg):
        # 受信したデータとタイムスタンプをCSVに書き込む
        current_time = self.get_clock().now().to_msg().sec  # 現在のタイムスタンプ
        data = msg.data  # UInt16MultiArrayのデータはリスト
        self.csv_writer.writerow([current_time] + list(data))  # タイムスタンプとデータを一緒に書き込む
        self.get_logger().info(f'Data written to CSV: {data}')

    def destroy_node(self):
        # ノードが終了するときにファイルを閉じる
        self.csv_file.close()
        super().destroy_node()

def is_pid_stop_lpf_pub_running():
    """pid_stop_lpf_pub.pyが実行中かどうかを確認"""
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            if 'ros2 run py_pkg pid_stop_lpf_pub.py' in ' '.join(proc.info['cmdline']):
                return True
        except psutil.NoSuchProcess:
            continue
    return False

def main(args=None):
    rclpy.init(args=args)

    # pid_stop_lpf_pub.pyが実行されるまで待つ
    print("Waiting for pid_stop_lpf_pub.py to start...")
    while not is_pid_stop_lpf_pub_running():
        time.sleep(1)

    print("pid_stop_lpf_pub.py is running. Starting data logger...")
    data_logger = DataLogger()

    try:
        rclpy.spin(data_logger)
    except KeyboardInterrupt:
        pass
    finally:
        data_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
