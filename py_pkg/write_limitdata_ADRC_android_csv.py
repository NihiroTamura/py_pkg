import csv
import os
import threading
import time
from datetime import datetime
from queue import Queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt16MultiArray


class DataRecorderNode(Node):
    def __init__(self):
        super().__init__("data_recorder_node")

        # 現在の日時
        now = datetime.now()
        self.date_str = now.strftime("%m%d")
        self.time_str = now.strftime("%H%M")

        # topicごとの記録設定（0始まりインデックス）
        # 例: [0,1,2,3,6,7,8,9] は 1,2,3,4,7,8,9,10番目を意味する
        self.topic_configs = [
            {"name": "board1", "topic": "/board1_tk/pub", "selected_indices": list(range(12))},
            {"name": "board2", "topic": "/board2_tk/pub", "selected_indices": list(range(12))},
            {"name": "board3", "topic": "/board3_tk/pub", "selected_indices": list(range(12))},
            {"name": "board4", "topic": "/board4_tk/pub", "selected_indices": [0, 1, 2, 6, 7, 8]},
            {"name": "board5", "topic": "/board5_tk/pub", "selected_indices": [0, 1, 2, 6, 7, 8]},
        ]

        self._validate_topic_configs()
        self._build_headers()
        self._build_file_paths()

        self.recording_active = False
        self.start_time = None

        self.queues = {}
        self.topic_subscriptions = []
        self.writer_threads = []

        self.sub_trigger = self.create_subscription(
            Float32MultiArray, "/board_android_float/sub", self.trigger_callback, 10
        )
        self.get_logger().info("/board_android_float/subへのメッセージ待機中")

        self.input_thread = threading.Thread(target=self.wait_for_enter, daemon=True)
        self.input_thread.start()

    def _validate_topic_configs(self):
        for config in self.topic_configs:
            indices = config["selected_indices"]
            if not indices:
                raise ValueError(f"{config['name']} の selected_indices が空です")
            if any(index < 0 or index > 11 for index in indices):
                raise ValueError(
                    f"{config['name']} の selected_indices は0〜11で指定してください: {indices}"
                )
            if len(set(indices)) != len(indices):
                raise ValueError(f"{config['name']} の selected_indices に重複があります: {indices}")

    def _build_headers(self):
        self.headers = {}
        self.combined_header = []

        pot_counter = 0
        potdesired_counter = 0

        for idx, config in enumerate(self.topic_configs, start=1):
            header = [f"Time{idx}"]
            for data_index in config["selected_indices"]:
                if data_index < 6:
                    header.append(f"POT{pot_counter}")
                    pot_counter += 1
                else:
                    header.append(f"POTdesired{potdesired_counter}")
                    potdesired_counter += 1

            self.headers[config["name"]] = header
            self.combined_header.extend(header)

    def _build_file_paths(self):
        self.topic_file_map = {}
        for config in self.topic_configs:
            name = config["name"]
            filename = f"ADRC_ROS2_2026{self.date_str}_{self.time_str}_{name}.csv"
            self.topic_file_map[name] = filename
            self.get_logger().info(f"記録ファイル {name}: {filename}")

        # 既存コード互換のため個別属性名も保持する
        self.filename_board1 = self.topic_file_map["board1"]
        self.filename_board2 = self.topic_file_map["board2"]
        self.filename_board3 = self.topic_file_map["board3"]
        self.filename_board4 = self.topic_file_map["board4"]
        self.filename_board5 = self.topic_file_map["board5"]

        self.filename_combined = f"ADRC_ROS2_2026{self.date_str}_{self.time_str}.csv"
        self.get_logger().info(f"統合ファイル: {self.filename_combined}")

    def trigger_callback(self, _msg):
        if self.recording_active:
            return

        self.get_logger().info("/board_android_float/subトピックでメッセージを受信しました。記録を開始します。")
        self.recording_active = True
        # /board_android_float/sub受信時刻を0秒基準にする
        self.start_time = time.perf_counter()

        for config in self.topic_configs:
            name = config["name"]
            queue = Queue()
            self.queues[name] = queue

            subscription = self.create_subscription(
                UInt16MultiArray,
                config["topic"],
                self._make_topic_callback(name),
                10,
            )
            self.topic_subscriptions.append(subscription)

            thread = threading.Thread(
                target=self.write_to_csv,
                args=(self.topic_file_map[name], queue),
                daemon=True,
            )
            self.writer_threads.append(thread)
            thread.start()

            queue.put(("header", self.headers[name]))

    def _make_topic_callback(self, topic_name):
        def callback(msg):
            if self.recording_active:
                self.enqueue_data(topic_name, msg)

        return callback

    def enqueue_data(self, topic_name, msg):
        if not self.recording_active or self.start_time is None:
            return

        if len(msg.data) != 12:
            self.get_logger().error(f"{topic_name} のデータ要素数が不正です: {len(msg.data)}")
            return

        elapsed_time = max(0.0, time.perf_counter() - self.start_time)
        config = next(item for item in self.topic_configs if item["name"] == topic_name)
        selected_values = [msg.data[index] for index in config["selected_indices"]]
        row = [elapsed_time] + selected_values
        self.queues[topic_name].put(("data", row))

    def write_to_csv(self, filename, queue):
        with open(filename, mode="w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            while rclpy.ok() and self.recording_active:
                try:
                    task_type, content = queue.get(timeout=1)
                except Exception:
                    continue

                if task_type in ("header", "data"):
                    writer.writerow(content)

    def wait_for_enter(self):
        input("Enterキーを押すと記録を終了して統合ファイルを作成します。\n")
        self.get_logger().info("Enterキーが押されました。終了処理を開始します。")
        self.recording_active = False
        time.sleep(1)
        self.combine_csv_files()
        rclpy.shutdown()

    def combine_csv_files(self):
        self.get_logger().info("CSV統合を開始します...")

        rows_by_topic = {}
        for config in self.topic_configs:
            name = config["name"]
            filename = self.topic_file_map[name]
            if os.path.exists(filename):
                with open(filename, newline="") as file_obj:
                    reader = list(csv.reader(file_obj))
                    rows_by_topic[name] = reader[1:]  # ヘッダ除外
            else:
                self.get_logger().warn(f"{name}ファイルが存在しません")
                rows_by_topic[name] = []

        max_rows = max((len(rows) for rows in rows_by_topic.values()), default=0)

        with open(self.filename_combined, mode="w", newline="") as out_file:
            writer = csv.writer(out_file)
            writer.writerow(self.combined_header)

            for row_index in range(max_rows):
                merged_row = []
                for config in self.topic_configs:
                    name = config["name"]
                    rows = rows_by_topic[name]
                    if row_index < len(rows):
                        merged_row.extend(rows[row_index])
                    else:
                        merged_row.extend([""] * len(self.headers[name]))
                writer.writerow(merged_row)

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
        node.get_logger().info("Ctrl+Cにより終了")
        node.recording_active = False
        node.combine_csv_files()
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
