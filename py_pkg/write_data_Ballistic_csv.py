import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, Float32MultiArray
import csv
import time
import threading
from queue import Queue

class DataRecorderNode(Node):
    def __init__(self):
        super().__init__('data_recorder_node')

        # トリガートピックのサブスクライバ
        #self.sub_trigger = self.create_subscription(
            #UInt16MultiArray, '/board/sub', self.trigger_callback, 10)
        
        self.sub_trigger = self.create_subscription(
            Float32MultiArray, '/board_float/sub', self.trigger_callback, 10)

        # 動作トリガーフラグと記録開始時刻
        self.recording_active = False
        self.start_time = None

        # キューとスレッドの準備
        self.queue_board1 = Queue()
        self.queue_board2 = Queue()
        self.thread_board1 = threading.Thread(target=self.write_to_csv, args=('board1_data.csv', self.queue_board1))
        self.thread_board2 = threading.Thread(target=self.write_to_csv, args=('board2_data.csv', self.queue_board2))
        self.thread_board1.daemon = True
        self.thread_board2.daemon = True
        self.thread_board1.start()
        self.thread_board2.start()

        # ヘッダの準備
        self.get_logger().info('/board/subへのメッセージ待機中')

        # サブスクライバの初期化
        self.sub_board1 = None
        self.sub_board2 = None

    def trigger_callback(self, msg):
        if not self.recording_active:
            self.get_logger().info('/board/subトピックでメッセージを受信しました。記録を開始します。')
            self.recording_active = True
            self.start_time = time.time()  # 記録開始時刻を保存

            # board1とboard2のサブスクライバーを開始
            self.sub_board1 = self.create_subscription(
                UInt16MultiArray, '/board1/pub', self.board1_callback, 10)
            self.sub_board2 = self.create_subscription(
                UInt16MultiArray, '/board2/pub', self.board2_callback, 10)

            # CSVにヘッダを記録
            header = ['Time'] + [f'POT{i}' for i in range(1, 7)] + \
                     [f'velocity{i}' for i in range(1, 7)] + \
                     [f'check{i}' for i in range(1, 7)]
            self.queue_board1.put(('header', header))
            self.queue_board2.put(('header', header))

    def board1_callback(self, msg):
        if self.recording_active:
            self.enqueue_data(self.queue_board1, msg, 'board1')

    def board2_callback(self, msg):
        if self.recording_active:
            self.enqueue_data(self.queue_board2, msg, 'board2')

    def enqueue_data(self, queue, msg, source):
        # 経過時間を計算
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        # データをキューに追加
        if len(msg.data) == 18:
            row = [elapsed_time] + list(msg.data)
            queue.put(('data', row))
            self.get_logger().info(f'{source}のデータをキューに追加しました: {row}')
        else:
            self.get_logger().error(f'{source}のデータ要素数が不正です: {len(msg.data)}')

    def write_to_csv(self, filename, queue):
        with open(filename, mode='w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            while rclpy.ok():
                try:
                    task_type, content = queue.get(timeout=1)
                    if task_type == 'header':
                        writer.writerow(content)
                        self.get_logger().info(f'ヘッダを{filename}に書き込みました')
                    elif task_type == 'data':
                        writer.writerow(content)
                        self.get_logger().info(f'データを{filename}に書き込みました: {content}')
                except Exception as e:
                    pass

    def destroy_node(self):
        # ノード終了時の処理
        self.recording_active = False
        self.thread_board1.join(timeout=1)
        self.thread_board2.join(timeout=1)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('終了処理中...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
