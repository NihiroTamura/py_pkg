import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, Float32MultiArray
import csv
import time
import threading
from queue import Queue
from datetime import datetime
import os


class DataRecorderNode(Node):
    def __init__(self):
        super().__init__('data_recorder_node')

        # 現在の日時
        now = datetime.now()
        self.date_str = now.strftime("%m%d")
        self.time_str = now.strftime("%H%M")

        # 動的ファイル名
        self.filename_board1 = f"ADRC_ROS2_2026{self.date_str}_{self.time_str}_board1.csv"
        self.filename_board2 = f"ADRC_ROS2_2026{self.date_str}_{self.time_str}_board2.csv"
        self.filename_board3 = f"ADRC_ROS2_2026{self.date_str}_{self.time_str}_board3.csv"
        self.filename_board4 = f"ADRC_ROS2_2026{self.date_str}_{self.time_str}_board4.csv"
        self.filename_board5 = f"ADRC_ROS2_2026{self.date_str}_{self.time_str}_board5.csv"
        self.filename_combined = f"ADRC_ROS2_2026{self.date_str}_{self.time_str}.csv"

        self.get_logger().info(f"記録ファイル board1: {self.filename_board1}")
        self.get_logger().info(f"記録ファイル board2: {self.filename_board2}")
        self.get_logger().info(f"記録ファイル board3: {self.filename_board3}")
        self.get_logger().info(f"記録ファイル board4: {self.filename_board4}")
        self.get_logger().info(f"記録ファイル board5: {self.filename_board5}")
        self.get_logger().info(f"統合ファイル: {self.filename_combined}")

        # トリガートピックのサブスクライバ
        self.sub_trigger = self.create_subscription(
            Float32MultiArray, '/board_android_float/sub', self.trigger_callback, 10)

        self.recording_active = False
        self.start_time = None

        self.queue_board1 = Queue()
        self.queue_board2 = Queue()
        self.queue_board3 = Queue()
        self.queue_board4 = Queue()
        self.queue_board5 = Queue()

        # ヘッダ
        self.header1 = [
            "Time1",
            "POT0","POT1","POT2","POT3","POT4","POT5",
            "POTdesired0","POTdesired1","POTdesired2","POTdesired3","POTdesired4","POTdesired5"
        ]
        self.header2 = [
            "Time2",
            "POT6","POT7","POT8","POT9","POT10","POT11",
            "POTdesired6","POTdesired7","POTdesired8","POTdesired9","POTdesired10","POTdesired11"
        ]
        self.header3 = [
            "Time3",
            "POT12","POT13","POT14","POT15","POT16","POT17",
            "POTdesired12","POTdesired13","POTdesired14","POTdesired15","POTdesired16","POTdesired17"
        ]
        self.header4 = [
            "Time4",
            "POT18","POT19","POT20","POT21","POT22","POT23",
            "POTdesired18","POTdesired19","POTdesired20","POTdesired21","POTdesired22","POTdesired23"
        ]
        self.header5 = [
            "Time5",
            "POT24","POT25","POT26","POT27","POT28","POT29",
            "POTdesired24","POTdesired25","POTdesired26","POTdesired27","POTdesired28","POTdesired29"
        ]

        self.get_logger().info('/board_android_float/subへのメッセージ待機中')

        self.sub_board1 = None
        self.sub_board2 = None
        self.sub_board3 = None
        self.sub_board4 = None
        self.sub_board5 = None

        # キー入力監視スレッド
        self.input_thread = threading.Thread(target=self.wait_for_enter)
        self.input_thread.daemon = True
        self.input_thread.start()

    def trigger_callback(self, msg):
        if not self.recording_active:
            self.get_logger().info('/board_android_float/subトピックでメッセージを受信しました。記録を開始します。')
            self.recording_active = True
            self.start_time = time.time()

            self.sub_board1 = self.create_subscription(
                UInt16MultiArray, '/board1_tk/pub', self.board1_callback, 10)
            self.sub_board2 = self.create_subscription(
                UInt16MultiArray, '/board2_tk/pub', self.board2_callback, 10)
            self.sub_board3 = self.create_subscription(
                UInt16MultiArray, '/board3_tk/pub', self.board3_callback, 10)
            self.sub_board4 = self.create_subscription(
                UInt16MultiArray, '/board4_tk/pub', self.board4_callback, 10)
            self.sub_board5 = self.create_subscription(
                UInt16MultiArray, '/board5_tk/pub', self.board5_callback, 10)

            # 書き込みスレッドを起動
            self.thread_board1 = threading.Thread(
                target=self.write_to_csv, args=(self.filename_board1, self.queue_board1))
            self.thread_board2 = threading.Thread(
                target=self.write_to_csv, args=(self.filename_board2, self.queue_board2))
            self.thread_board3 = threading.Thread(
                target=self.write_to_csv, args=(self.filename_board3, self.queue_board3))
            self.thread_board4 = threading.Thread(
                target=self.write_to_csv, args=(self.filename_board4, self.queue_board4))
            self.thread_board5 = threading.Thread(
                target=self.write_to_csv, args=(self.filename_board5, self.queue_board5))
            self.thread_board1.daemon = True
            self.thread_board2.daemon = True
            self.thread_board3.daemon = True
            self.thread_board4.daemon = True
            self.thread_board5.daemon = True
            self.thread_board1.start()
            self.thread_board2.start()
            self.thread_board3.start()
            self.thread_board4.start()
            self.thread_board5.start()

            # ヘッダ
            self.queue_board1.put(('header', self.header1))
            self.queue_board2.put(('header', self.header2))
            self.queue_board3.put(('header', self.header3))
            self.queue_board4.put(('header', self.header4))
            self.queue_board5.put(('header', self.header5))

    def board1_callback(self, msg):
        if self.recording_active:
            self.enqueue_data(self.queue_board1, msg, 'board1')
        
    def board2_callback(self, msg):
        if self.recording_active:
            self.enqueue_data(self.queue_board2, msg, 'board2')
    
    def board3_callback(self, msg):
        if self.recording_active:
            self.enqueue_data(self.queue_board3, msg, 'board3')
    
    def board4_callback(self, msg):
        if self.recording_active:
            self.enqueue_data(self.queue_board4, msg, 'board4')
    
    def board5_callback(self, msg):
        if self.recording_active:
            self.enqueue_data(self.queue_board5, msg, 'board5')

    def enqueue_data(self, queue, msg, source):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        if len(msg.data) == 12:
            row = [elapsed_time] + list(msg.data)
            queue.put(('data', row))
        else:
            self.get_logger().error(f'{source}のデータ要素数が不正です: {len(msg.data)}')

    def write_to_csv(self, filename, queue):
        with open(filename, mode='w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            while rclpy.ok() and self.recording_active:
                try:
                    task_type, content = queue.get(timeout=1)
                    if task_type == 'header':
                        writer.writerow(content)
                    elif task_type == 'data':
                        writer.writerow(content)
                except:
                    continue

    def wait_for_enter(self):
        input("Enterキーを押すと記録を終了して統合ファイルを作成します。\n")
        self.get_logger().info("Enterキーが押されました。終了処理を開始します。")
        self.recording_active = False
        time.sleep(1)
        self.combine_csv_files()
        rclpy.shutdown()

    def combine_csv_files(self):
        self.get_logger().info("CSV統合を開始します...")
        rows_board1 = []
        rows_board2 = []
        rows_board3 = []
        rows_board4 = []
        rows_board5 = []

        # board1読み込み
        if os.path.exists(self.filename_board1):
            with open(self.filename_board1, newline='') as f1:
                reader1 = list(csv.reader(f1))
                rows_board1 = reader1[1:]  # ヘッダ除外
        else:
            self.get_logger().warn("board1ファイルが存在しません")

        # board2読み込み
        if os.path.exists(self.filename_board2):
            with open(self.filename_board2, newline='') as f2:
                reader2 = list(csv.reader(f2))
                rows_board2 = reader2[1:]  # ヘッダ除外
        else:
            self.get_logger().warn("board2ファイルが存在しません")
        
        # board3読み込み
        if os.path.exists(self.filename_board3):
            with open(self.filename_board3, newline='') as f3:
                reader3 = list(csv.reader(f3))
                rows_board3 = reader3[1:]  # ヘッダ除外
        else:
            self.get_logger().warn("board3ファイルが存在しません")
        
        # board4読み込み
        if os.path.exists(self.filename_board4):
            with open(self.filename_board4, newline='') as f4:
                reader4 = list(csv.reader(f4))
                rows_board4 = reader4[1:]  # ヘッダ除外
        else:
            self.get_logger().warn("board4ファイルが存在しません")
        
        # board5読み込み
        if os.path.exists(self.filename_board5):
            with open(self.filename_board5, newline='') as f5:
                reader5 = list(csv.reader(f5))
                rows_board5 = reader5[1:]  # ヘッダ除外
        else:
            self.get_logger().warn("board5ファイルが存在しません")

        max_rows = max(len(rows_board1), len(rows_board2), len(rows_board3), len(rows_board4), len(rows_board5))
        combined_header = self.header1 + self.header2 + self.header3 + self.header4 + self.header5

        with open(self.filename_combined, mode='w', newline='') as f_out:
            writer = csv.writer(f_out)
            writer.writerow(combined_header)
            for i in range(max_rows):
                row1 = rows_board1[i] if i < len(rows_board1) else [''] * len(self.header1)
                row2 = rows_board2[i] if i < len(rows_board2) else [''] * len(self.header2)
                row3 = rows_board3[i] if i < len(rows_board3) else [''] * len(self.header3)
                row4 = rows_board4[i] if i < len(rows_board4) else [''] * len(self.header4)
                row5 = rows_board5[i] if i < len(rows_board5) else [''] * len(self.header5)
                writer.writerow(row1 + row2 + row3 + row4 + row5)

        self.get_logger().info(f"統合CSVファイル {self.filename_combined} を作成しました。")

    def destroy_node(self):
        self.recording_active = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+Cにより終了')
        node.recording_active = False
        node.combine_csv_files()
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
