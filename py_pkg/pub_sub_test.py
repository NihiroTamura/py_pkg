#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt16MultiArray

import tkinter as tk
from tkinter import ttk
import time

# POT ranges
POT_RANGE = [
    (120, 700), (135, 565), (130, 680), (10, 734), (66, 259), (192, 389),
    (70, 630), (60, 485), (115, 619), (22, 794), (239, 445), (205, 395),
    (10, 685), (10, 714), (105, 845), (3, 670), (3, 750),(9, 680), 
    (275, 370), (115, 785), (192, 445), (284, 557), 
    (323, 600), (188, 645), (300, 500), (420, 600),
]

# Initial desired values
INITIAL_DESIRED = [
    500, 200, 500, 300, 170, 300,
    160, 410, 200, 500, 350, 220,
    300, 250, 400, 350, 420, 400,
    325, 370, 280, 420,
    360, 390, 350, 500
]

# ------------------------------
# ROS2 Node
# ------------------------------
class PotGuiNode(Node):
    def __init__(self):
        super().__init__('pot_gui_node')
        self.publisher = self.create_publisher(Float32MultiArray, '/board_android_float/sub', 10)

        self.desired = [float(v) for v in INITIAL_DESIRED]
        self.real_raw = [0.0]*26

        # boardごとのPOT個数
        self.POT_COUNT = {
            'board1': 6,
            'board2': 6,
            'board3': 6,
            'board4': 4,
            'board5': 4,
        }

        # Subscribers
        self.create_subscription(UInt16MultiArray, '/board1_tk/pub', lambda msg: self.board_cb(msg, 0, 'board1'), 10)
        self.create_subscription(UInt16MultiArray, '/board2_tk/pub', lambda msg: self.board_cb(msg, 6, 'board2'), 10)
        self.create_subscription(UInt16MultiArray, '/board3_tk/pub', lambda msg: self.board_cb(msg, 12, 'board3'), 10)
        self.create_subscription(UInt16MultiArray, '/board4_tk/pub', lambda msg: self.board_cb(msg, 18, 'board4'), 10)
        self.create_subscription(UInt16MultiArray, '/board5_tk/pub', lambda msg: self.board_cb(msg, 22, 'board5'), 10)

    def board_cb(self, msg, offset, board_name):
        # このボードのPOT個数だけ書き込む
        num_pot = self.POT_COUNT[board_name]
        for i in range(num_pot):
            idx = offset + i
            if idx < 26 and i < len(msg.data):
                self.real_raw[idx] = float(msg.data[i])

    def publish(self):
        msg = Float32MultiArray()
        msg.data = self.desired
        self.publisher.publish(msg)

    def publish_initial(self):
        self.desired = [float(v) for v in INITIAL_DESIRED]
        self.publish()

# ------------------------------
# Tkinter GUI
# ------------------------------
class PotGuiTk:
    def __init__(self, node: PotGuiNode):
        self.node = node
        self.current_dof = 0
        self.root = tk.Tk()
        self.root.title("POT GUI (time plot)")
        self.root.geometry("1200x600")  # GUI全体を大きく

        # DOF selector
        self.dof_box = ttk.Combobox(self.root, values=[f"DOF {i}" for i in range(26)], state="readonly")
        self.dof_box.current(0)
        self.dof_box.bind("<<ComboboxSelected>>", self.change_dof)
        self.dof_box.pack(pady=5)

        self.info_label = tk.Label(self.root, text="", font=("Arial", 10))
        self.info_label.pack()

        self.real_label = tk.Label(self.root, text="", font=("Arial", 10))
        self.real_label.pack()

        # Canvas for time plot
        self.canvas = tk.Canvas(self.root, width=800, height=300, bg="#eeeeee")
        self.canvas.pack(pady=10)

        # desired slider
        self.slider = tk.Scale(self.root, orient=tk.HORIZONTAL, length=800, command=self.on_slider)
        self.slider.pack()

        # Reset pose button
        self.init_button = tk.Button(self.root, text="Reset pose", command=self.send_initial)
        self.init_button.pack(pady=5)

        # Time-series data buffer
        self.buffer_len = 100  # 最新100点
        self.time_buffer = [0]*self.buffer_len
        self.real_buffer = [0]*self.buffer_len
        self.desired_buffer = [0]*self.buffer_len
        self.start_time = time.time()

        self.update_ui()
        self.update_plot()

    # -------------------------
    def change_dof(self, event):
        self.current_dof = self.dof_box.current()
        self.real_buffer = [0]*self.buffer_len
        self.desired_buffer = [0]*self.buffer_len
        self.time_buffer = [0]*self.buffer_len
        self.start_time = time.time()
        self.update_ui()

    def update_ui(self):
        mn, mx = POT_RANGE[self.current_dof]
        desired = int(self.node.desired[self.current_dof])
        self.slider.config(from_=mn, to=mx)
        self.slider.set(desired)
        self.info_label.config(text=f"Desired: {desired} Range: [{mn},{mx}]")
        raw = self.node.real_raw[self.current_dof]
        self.real_label.config(text=f"Real (raw): {int(raw)}")

    # -------------------------
    def on_slider(self, value):
        self.node.desired[self.current_dof] = float(value)
        self.node.publish()

    def send_initial(self):
        self.node.publish_initial()
        self.update_ui()

    # -------------------------
    def update_plot(self):
        dof = self.current_dof
        raw = self.node.real_raw[dof]
        desired = self.node.desired[dof]
        t = time.time() - self.start_time

        # バッファ更新
        self.time_buffer.append(t)
        self.time_buffer = self.time_buffer[-self.buffer_len:]
        self.real_buffer.append(raw)
        self.real_buffer = self.real_buffer[-self.buffer_len:]
        self.desired_buffer.append(desired)
        self.desired_buffer = self.desired_buffer[-self.buffer_len:]

        # Canvas描画
        self.canvas.delete("all")
        mn, mx = POT_RANGE[dof]
        width = int(self.canvas['width'])
        height = int(self.canvas['height'])
        margin = 50  # 上下左右マージン

        plot_width = width - 2*margin
        plot_height = height - 2*margin

        # ---------------------------
        # Y軸目盛り（5分割）
        y_div = 5
        for i in range(y_div + 1):
            y_val = mn + i*(mx - mn)/y_div
            y = margin + plot_height * (1 - (y_val - mn)/(mx - mn))
            self.canvas.create_line(margin, y, width - margin, y, fill="#cccccc", dash=(2,2))
            self.canvas.create_text(margin-10, y, text=str(int(y_val)), anchor="e", font=("Arial", 8))

        # X軸補助線（0.5秒ごと）
        t_start = self.time_buffer[0]
        t_end = self.time_buffer[-1]
        if t_end - t_start < 0.001:
            t_end = t_start + 1.0
        time_range = t_end - t_start
        x_interval = 0.5
        x = (t_start // x_interval) * x_interval
        while x <= t_end:
            x_pos = margin + (x - t_start)/time_range * plot_width
            self.canvas.create_line(x_pos, margin, x_pos, margin + plot_height, fill="#cccccc", dash=(2,2))
            x += x_interval

        # ---------------------------
        # 実線・破線描画
        if len(self.real_buffer) > 1:
            for i in range(1, len(self.real_buffer)):
                x1 = margin + (i-1)/self.buffer_len * plot_width
                x2 = margin + i/self.buffer_len * plot_width
                # 実現値 (赤実線)
                y1 = margin + plot_height * (1 - (self.real_buffer[i-1]-mn)/(mx-mn))
                y2 = margin + plot_height * (1 - (self.real_buffer[i]-mn)/(mx-mn))
                self.canvas.create_line(x1, y1, x2, y2, fill="red", width=2)
                # 目標値 (青破線)
                y1d = margin + plot_height * (1 - (self.desired_buffer[i-1]-mn)/(mx-mn))
                y2d = margin + plot_height * (1 - (self.desired_buffer[i]-mn)/(mx-mn))
                self.canvas.create_line(x1, y1d, x2, y2d, fill="blue", width=2, dash=(4,2))

        self.real_label.config(text=f"Real (raw): {int(raw)}  Desired: {int(desired)}")
        self.root.after(50, self.update_plot)

    def run(self):
        self.root.mainloop()

# ------------------------------
def main():
    rclpy.init()
    node = PotGuiNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    gui = PotGuiTk(node)
    gui.run()

if __name__ == '__main__':
    main()
