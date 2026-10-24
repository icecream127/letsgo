"""

                               _(\_/)
                             ,((((^`\
                             ((((  (6 \
                          ,((((( ,    \
       ,,,_              ,(((((  /"._  ,`,
      ((((\\ ,...       ,((((   /    `-.-'
      )))  ;'    `"'"'""((((   (
    (((  /            (((      \
      )) |                      |
     ((  |        .       '     |
     ))  \     _ '      `t   ,.')
     (   |   y;- -,-""'"-.\   \/
    )   / ./  ) /         `\  \
        |./   ( (           / /'
        ||     \\          //'|
        ||      \\       _//'||
        ||       ))     |_/  ||
        \_\     |_/          ||
        `'"                  \_\
                             `'"
"""
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from .zone_manager import zone_config_path
from .route_review import select_region
from std_srvs.srv import Trigger
import tkinter as tk
from tkinter import font


COMMAND_MAP = {
    "左转": 1,
    "前进": 2,
    "右转": 3,
    "后退": 4,
    "站立": 5,
    "小跳1": 6,
    "小跳3": 7,
    "小跳2": 8,
    "正常模式": 9,
    "台阶模式": 10,
    "小斜坡模式": 11,
    "木桥B模式": 12,
    "木桥A模式": 13,
    "大斜坡模式": 14,
    "匍匐模式": 15,
    "沙坑模式": 16,
    "木坑模式": 17,
    "绕杆模式": 18,
    "左平移": 19,
    "右平移": 20,
    "高墙跳": 21,
    "高墙模式": 22,
}


class TaskNode(Node):
    """
    负责 ROS 2 通信和调试状态汇总的节点类
    """

    def __init__(self):
        super().__init__("task_gui_node")

        self.pub = self.create_publisher(Float32MultiArray, "/task_params", 10)
        self.reset_index_client = self.create_client(Trigger, '/automation/reset_route')
        self.pos_subscription = self.create_subscription(
            Float32MultiArray,
            "/pos_trans",
            self.pos_callback,
            20,
        )
        self.local_coord_subscription = self.create_subscription(
            Float32MultiArray,
            "/ui_local_coord",
            self.local_coord_callback,
            20,
        )
        self.serial_subscription = self.create_subscription(
            Float32MultiArray,
            "/serial_send",
            self.serial_callback,
            20,
        )
        self.debug_subscription = self.create_subscription(
            String,
            "/ui_debug_state",
            self.debug_callback,
            20,
        )

        self._state_lock = threading.Lock()
        self.current_zone = "等待定位数据"
        self.current_global_coord = "X: ---, Y: ---, Yaw: ---"
        self.current_local_coord = "X: ---, Y: ---, Yaw: ---"
        self._zone_config_path = zone_config_path()
        self._zone_config_mtime = None
        self._last_zone_config_check = 0.0
        self.zones = {}
        self.normal_zones = {}
        self.load_zone_config(force=True)

        self.command_lookup = {value: key for key, value in COMMAND_MAP.items()}

        self.get_logger().info("任务控制 ROS 2 节点已启动...")

    def publish_cmd(self, task_value, task_name):
        """发布指令到话题"""
        msg = Float32MultiArray()
        msg.data = [float(task_value)]
        self.pub.publish(msg)
        self.get_logger().info(f"已下发任务: [{task_name}], 发送数据: {msg.data}")

    def publish_reset_index(self):
        if not self.reset_index_client.service_is_ready():
            self.get_logger().error('控制器未就绪，未清零路线')
            return
        future = self.reset_index_client.call_async(Trigger.Request())
        future.add_done_callback(self.reset_index_result)

    def reset_index_result(self, future):
        try:
            result = future.result()
            logger = self.get_logger().info if result.success else self.get_logger().error
            logger(result.message)
        except Exception as error:
            self.get_logger().error(f'路线清零失败：{error}')

    def get_debug_state(self):
        with self._state_lock:
            return {
                "zone": self.current_zone,
                "global_coord": self.current_global_coord,
                "local_coord": self.current_local_coord,
            }

    def pos_callback(self, msg):
        if len(msg.data) < 2:
            return

        x_pos = msg.data[0]
        y_pos = msg.data[1]
        yaw = msg.data[2] if len(msg.data) >= 3 else None

        zone_name, local_x, local_y = self.compute_local_coord(x_pos, y_pos)
        with self._state_lock:
            self.current_global_coord = self.format_coord_text(x_pos, y_pos, yaw)
            self.current_local_coord = self.format_coord_text(local_x, local_y, yaw)
            self.current_zone = zone_name

    def local_coord_callback(self, msg):
        if len(msg.data) < 2:
            return

        x_pos = msg.data[0]
        y_pos = msg.data[1]
        yaw = msg.data[2] if len(msg.data) >= 3 else None

        with self._state_lock:
            self.current_local_coord = self.format_coord_text(x_pos, y_pos, yaw)

    def load_zone_config(self, force=False):
        now = time.monotonic()
        if not force and now - self._last_zone_config_check < 1.0:
            return
        self._last_zone_config_check = now

        try:
            mtime = os.path.getmtime(self._zone_config_path)
        except OSError as exc:
            self.get_logger().warning(f"无法读取区域配置文件: {exc}")
            return

        if not force and self._zone_config_mtime == mtime:
            return

        try:
            with open(self._zone_config_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"区域配置加载失败: {exc}")
            return

        self.zones = data.get("zones", {})
        self.normal_zones = data.get("zone", {})
        self._zone_config_mtime = mtime

    def compute_local_coord(self, x_pos, y_pos):
        self.load_zone_config()
        zone_name, zone = select_region(dict(zones=self.zones, zone=self.normal_zones), x_pos, y_pos)
        if zone is not None:
            return zone_name, x_pos - zone['begin_x'], y_pos - zone['begin_y']
        return "未进入已配置区域", x_pos, y_pos

    @staticmethod
    def is_in_zone(x_pos, y_pos, zone):
        return (
            zone.get("x_min", float("-inf")) <= x_pos < zone.get("x_max", float("inf"))
            and zone.get("y_min", float("-inf")) <= y_pos < zone.get("y_max", float("inf"))
        )

    def serial_callback(self, msg):
        if len(msg.data) < 2:
            return

    def debug_callback(self, msg):
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("收到无法解析的调试状态消息")
            return

        with self._state_lock:
            self.current_zone = state.get("zone", self.current_zone)

    @staticmethod
    def format_coord_text(x_pos, y_pos, yaw=None):
        coord_text = f"X: {x_pos:.3f}, Y: {y_pos:.3f}"
        if yaw is None:
            return coord_text
        return f"{coord_text}, Yaw: {yaw:.3f}°"


class TaskGUIApp:
    """
    负责 Tkinter 界面渲染的 UI 类
    """

    def __init__(self, ros_node):
        self.ros_node = ros_node

        self.root = tk.Tk()
        self.root.title("任务控制与调试面板 (ROS 2)")
        self.root.geometry("520x780")
        self.root.configure(bg="#f4efe7")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)

        self.tasks = [
            ("起点", 1.0),
            ("终点", 2.0),
            ("木桥A", 3.0),
            ("木桥B", 4.0),
            ("大斜坡", 5.0),
            ("限高杆", 6.0),
            ("沙坑", 7.0),
            ("绕杆", 8.0),
            ("高墙", 9.0),
            ("台阶", 10.0),
        ]

        self.zone_var = tk.StringVar(value="等待定位数据")
        self.global_coord_var = tk.StringVar(value="X: ---, Y: ---, Yaw: ---")
        self.local_coord_var = tk.StringVar(value="X: ---, Y: ---, Yaw: ---")

        self.canvas = tk.Canvas(self.root, bg="#f4efe7", highlightthickness=0)
        self.scrollbar = tk.Scrollbar(self.root, orient="vertical", command=self.canvas.yview)
        self.scroll_frame = tk.Frame(self.canvas, bg="#f4efe7")

        self.scroll_frame.bind(
            "<Configure>",
            lambda event: self.canvas.configure(scrollregion=self.canvas.bbox("all")),
        )

        self.canvas.create_window((0, 0), window=self.scroll_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.scrollbar.grid(row=0, column=1, sticky="ns")
        self.canvas.bind_all("<MouseWheel>", self.on_mousewheel)

        self.build_ui()
        self.check_ros_ok()
        self.refresh_debug_panel()

    def build_ui(self):
        title_font = font.Font(family="Helvetica", size=18, weight="bold")
        section_font = font.Font(family="Helvetica", size=12, weight="bold")
        btn_font = font.Font(family="Helvetica", size=12)

        header = tk.Frame(self.scroll_frame, bg="#f4efe7")
        header.pack(fill="x", padx=18, pady=(18, 8))

        title = tk.Label(
            header,
            text="任务控制与调试面板",
            font=title_font,
            fg="#2e2a27",
            bg="#f4efe7",
        )
        title.pack(anchor="w")

        subtitle = tk.Label(
            header,
            text="采点按钮保留下方实时显示区域和两组坐标",
            font=("Helvetica", 10),
            fg="#6c625a",
            bg="#f4efe7",
        )
        subtitle.pack(anchor="w", pady=(4, 0))

        control_frame = tk.Frame(
            self.scroll_frame,
            bg="#fffaf3",
            highlightbackground="#d4c6b6",
            highlightthickness=1,
            bd=0,
        )
        control_frame.pack(fill="x", padx=18, pady=(4, 10))

        control_title = tk.Label(
            control_frame,
            text="采点控制",
            font=section_font,
            fg="#7d4f2a",
            bg="#fffaf3",
        )
        control_title.pack(anchor="w", padx=14, pady=(14, 8))

        button_grid = tk.Frame(control_frame, bg="#fffaf3")
        button_grid.pack(fill="x", padx=12, pady=(0, 14))

        for index, (task_name, task_value) in enumerate(self.tasks):
            button = tk.Button(
                button_grid,
                text=task_name,
                font=btn_font,
                width=14,
                bg="#dcc5a3",
                fg="#2d2117",
                activebackground="#cda97a",
                activeforeground="#2d2117",
                relief="flat",
                padx=8,
                pady=10,
                command=lambda value=task_value, name=task_name: self.ros_node.publish_cmd(value, name),
            )
            row = index // 2
            column = index % 2
            button.grid(row=row, column=column, padx=8, pady=6, sticky="ew")

        reset_button = tk.Button(
            control_frame,
            text="手动清零 Index",
            font=btn_font,
            bg="#c8553d",
            fg="#fffaf3",
            activebackground="#a63d28",
            activeforeground="#fffaf3",
            relief="flat",
            padx=8,
            pady=10,
            command=self.ros_node.publish_reset_index,
        )
        reset_button.pack(fill="x", padx=20, pady=(0, 14))

        button_grid.grid_columnconfigure(0, weight=1)
        button_grid.grid_columnconfigure(1, weight=1)

        debug_frame = tk.Frame(
            self.scroll_frame,
            bg="#2f4858",
            highlightbackground="#213743",
            highlightthickness=1,
            bd=0,
        )
        debug_frame.pack(fill="both", expand=True, padx=18, pady=(0, 18))

        debug_title = tk.Label(
            debug_frame,
            text="实时调试信息",
            font=section_font,
            fg="#f7efe5",
            bg="#2f4858",
        )
        debug_title.pack(anchor="w", padx=16, pady=(16, 10))

        self.create_status_card(debug_frame, "现在区域", self.zone_var, "#f7b267")
        self.create_status_card(debug_frame, "pos_trans 转换坐标", self.global_coord_var, "#f4845f")
        self.create_status_card(debug_frame, "障碍相对起点坐标", self.local_coord_var, "#84a59d")

    def create_status_card(self, parent, label_text, value_var, accent_color):
        card = tk.Frame(parent, bg="#f7efe5", bd=0)
        card.pack(fill="x", padx=16, pady=8)

        accent = tk.Frame(card, bg=accent_color, width=10, height=90)
        accent.pack(side="left", fill="y")

        content = tk.Frame(card, bg="#f7efe5")
        content.pack(fill="both", expand=True, side="left", padx=14, pady=12)

        label = tk.Label(
            content,
            text=label_text,
            font=("Helvetica", 11, "bold"),
            fg="#6a5d52",
            bg="#f7efe5",
        )
        label.pack(anchor="w")

        value = tk.Label(
            content,
            textvariable=value_var,
            font=("Helvetica", 16, "bold"),
            fg="#1e2a32",
            bg="#f7efe5",
            wraplength=330,
            justify="left",
        )
        value.pack(anchor="w", pady=(8, 0))

    def on_mousewheel(self, event):
        self.canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def refresh_debug_panel(self):
        state = self.ros_node.get_debug_state()
        self.zone_var.set(state["zone"])
        self.global_coord_var.set(state["global_coord"])
        self.local_coord_var.set(state["local_coord"])
        self.root.after(100, self.refresh_debug_panel)

    def check_ros_ok(self):
        """周期性检查 ROS 2 是否被关闭，以响应终端的 Ctrl+C"""
        if not rclpy.ok():
            self.root.destroy()
        else:
            self.root.after(100, self.check_ros_ok)

    def on_closing(self):
        """处理直接关闭 GUI 窗口的事件"""
        self.ros_node.get_logger().info("GUI 面板已关闭，正在退出节点...")
        self.root.destroy()
        if rclpy.ok():
            rclpy.shutdown()

    def run(self):
        """运行界面主循环"""
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)

    node = TaskNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    gui_app = TaskGUIApp(node)
    try:
        gui_app.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
