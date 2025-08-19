import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
import csv
from datetime import datetime

class DataSubscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')

        # /POT/realized トピックのデータ購読
        self.data_subscription = self.create_subscription(
            UInt16MultiArray,
            '/POT/realized',
            self.realized_callback,
            10)
        self.data_subscription  # prevent unused variable warning

        # /VEAB1/desired トピックのトリガー購読
        self.trigger_subscription = self.create_subscription(
            UInt16MultiArray,
            '/VEAB1/desired',
            self.trigger_callback,
            10)
        self.trigger_subscription  # prevent unused variable warning

        # /CheckMode トピックのデータ購読
        self.checkmode_subscription = self.create_subscription(
            UInt16MultiArray,
            '/CheckMode',
            self.checkmode_callback,
            10)
        self.checkmode_subscription  # prevent unused variable warning

        # CSVファイルの準備
        self.csv_file = open('output_data_with_timestamp.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Elapsed Time (s)', 'Source', 'Index', 'Value'])  # ヘッダ行を書き込む

        self.realized_data = []  # /POT/realized データ保存用
        self.checkmode_data = []  # /CheckMode データ保存用
        self.is_trigger_received = False  # トリガーのフラグ

        self.realized_start_time = None  # /POT/realized 記録開始時刻
        self.checkmode_start_time = None  # /CheckMode 記録開始時刻
        self.realized_first_received = False  # /POT/realized の初回受信フラグ
        self.checkmode_first_received = False  # /CheckMode の初回受信フラグ

    def realized_callback(self, msg):
        # /POT/realized トピックからのデータが届いたら保存
        if len(msg.data) == 12:  # 配列の長さが12であるか確認
            if not self.realized_first_received:
                self.realized_start_time = datetime.now()
                self.realized_first_received = True
            self.realized_data = msg.data

    def checkmode_callback(self, msg):
        # /CheckMode トピックからのデータが届いたら保存
        if len(msg.data) == 7:  # 配列の長さが7であるか確認
            if not self.checkmode_first_received:
                self.checkmode_start_time = datetime.now()
                self.checkmode_first_received = True
            self.checkmode_data = msg.data

    def trigger_callback(self, msg):
        # /VEAB1/desired トピックでメッセージを受信したら
        self.is_trigger_received = True
        self.write_data_to_csv()

    def write_data_to_csv(self):
        if self.is_trigger_received:
            current_time = datetime.now()

            # /POT/realized データをCSVに書き込む
            if self.realized_data:
                elapsed_time = (current_time - self.realized_start_time).total_seconds() if self.realized_first_received else 0.0
                for index, value in enumerate(self.realized_data):
                    self.csv_writer.writerow([f"{elapsed_time:.4f}", 'POT/realized', index, value])

            # /CheckMode データをCSVに書き込む
            if self.checkmode_data:
                elapsed_time = (current_time - self.checkmode_start_time).total_seconds() if self.checkmode_first_received else 0.0
                for index, value in enumerate(self.checkmode_data):
                    self.csv_writer.writerow([f"{elapsed_time:.4f}", 'CheckMode', index, value])

            self.is_trigger_received = False  # フラグをリセット

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    data_subscriber = DataSubscriber()

    try:
        rclpy.spin(data_subscriber)
    except KeyboardInterrupt:
        pass

    data_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
