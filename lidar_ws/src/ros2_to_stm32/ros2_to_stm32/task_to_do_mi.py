import time
import math
import json
import rclpy
import matplotlib
from rclpy.node import Node
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import matplotlib.transforms as transforms
from .zone_manager import ZoneManager, zone_config_path 
COMMAND_MAP = {
    "左转": 1, "前进": 2, "右转": 3, "后退": 4, "站立": 5,
    "小跳1": 6, "小跳2": 7,"大斜坡左":4,
    "正常模式": 9, "小斜坡模式": 10, "木桥B模式": 11, "木桥A模式": 12,
    "大斜坡模式": 13, "匍匐模式": 14, "沙坑模式": 15, "绕杆模式": 16,
    "高墙模式": 17, "台阶模式": 18, "左平移": 19, "右平移": 20, "下台阶":8,
}


class Task_To_Do(Node):
    def __init__(self):
        super().__init__('task_to_do')

        self.publisher = self.create_publisher(Float32MultiArray, '/serial_send', 20)
        self.debug_publisher = self.create_publisher(String, '/ui_debug_state', 20)
        self.global_coord_publisher = self.create_publisher(Float32MultiArray, '/ui_global_coord', 20)
        self.local_coord_publisher = self.create_publisher(Float32MultiArray, '/ui_local_coord', 20)

        self.last_kKey = "站立"
        self.current_zone = "等待定位数据"
        self.current_action = "等待动作数据"
        self.global_x = 0.0
        self.global_y = 0.0
        self.global_yaw = 0.0
        self.local_x = 0.0
        self.local_y = 0.0
        self.local_yaw = 0.0

        self.robot_mode = "正常模式"
        self.mode_change = 1

        self._is_reversing = False

        self.is_jumping = False
        self.jump_start_time = self.get_clock().now()
        self.jump_duration = 5.0 

        self.index = 0
        self._entered_handlers = set()
        # 1. 实例化管理器
        self.zm = ZoneManager(zone_config_path())
        
        # 2. 读取数据
        all_data = self.zm.load_data()
        
        # 3. 获取 zones 数据，并将 func_name 还原为真正的函数引用
        self.zones = self.zm.bind_functions(self, all_data["zones"])
        self.zone = all_data["zone"]
        self.zon = all_data["zon"]
        
        self.get_logger().info("成功加载并解析区域坐标数据！")

        self.stop = 0
        self.publish_global_coord(self.global_x, self.global_y)
        self.publish_local_coord(self.local_x, self.local_y)
        self.publish_debug_state()

    def publish_debug_state(self):
        msg = String()
        msg.data = json.dumps({
            "zone": self.current_zone,
            "action": self.current_action,
            "mode": self.robot_mode,
            "global_x": self.global_x,
            "global_y": self.global_y,
            "global_yaw":self.global_yaw,
            "local_x": self.local_x,
            "local_y": self.local_y,
            "local_yaw":self.local_yaw,
        }, ensure_ascii=False)
        self.debug_publisher.publish(msg)

    def publish_global_coord(self, x_pos, y_pos, yaw=0.0):
        self.global_x = x_pos
        self.global_y = y_pos
        self.global_yaw = yaw
        msg = Float32MultiArray()
        msg.data = [float(x_pos), float(y_pos), float(yaw)]
        self.global_coord_publisher.publish(msg)

    def publish_local_coord(self, x_pos, y_pos ,yaw=0.0):
        self.local_x = x_pos
        self.local_y = y_pos
        self.local_yaw = yaw
        msg = Float32MultiArray()
        msg.data = [float(x_pos), float(y_pos), float(yaw)]
        self.local_coord_publisher.publish(msg)

    def set_debug_coords(self, global_x, global_y, yaw, x_begin, y_begin):
        self.publish_global_coord(global_x, global_y ,yaw)
        self.publish_local_coord(global_x - x_begin, global_y - y_begin ,yaw)
        self.publish_debug_state()

    def set_normal_debug_coords(self, global_x, global_y, yaw):
        self.publish_global_coord(global_x, global_y, yaw)
        self.publish_local_coord(global_x, global_y, yaw)
        self.publish_debug_state()

    def Quadruped_Obstacle(self, x, y, yaw):
        for kind, value in self.zone.items():
            if value["x_min"] <= x < value["x_max"] and value["y_min"] <= y < value["y_max"]:
                self.current_zone = kind
                self.set_normal_debug_coords(x, y, yaw)
                self.robot_mode = "匍匐模式"
                self.navigate_to_pose(x, y, yaw, 3.4, 0, 0)
                self.navigate_to_pose(x, y, yaw, x+5, 0, 0)

                # self.Kkey_To_Number("下台阶")
        

                return

        for kind, value in self.zones.items():
            if value["x_min"] <= x < value["x_max"] and value["y_min"] <= y < value["y_max"]:
                self.current_zone = kind
                func_name_str = value["func_name"]
                actual_function = getattr(self, func_name_str)
                actual_function(
                    x,
                    y,
                    yaw,
                    value["begin_x"],
                    value["begin_y"]
                )
                return
            

        self.current_zone = "未进入已配置区域"
        self.publish_global_coord(x, y, yaw)
        self.publish_local_coord(x, y, yaw)
        self.publish_debug_state()

    def updata_kKey(self):
        self.last_kKey = 0
        self.stop = 1
        self.current_action = "等待执行"
        self.publish_debug_state()

    def reset_index_on_first_entry(self, handler_name):
        if handler_name in self._entered_handlers:
            return
        self.index = 0
        self._entered_handlers.add(handler_name)

    def reset_index_manually(self):
        self.index = 0
        self.current_action = "手动清零 index"
        self.publish_debug_state()

    def handle_bridge_A(self, x, y, yaw, x_begin, y_begin):
        self.reset_index_on_first_entry("handle_bridge_B")
        self.get_logger().info("bridge_B")
        self.set_debug_coords(x, y, yaw, x_begin, y_begin)
        x = x - x_begin
        y = y - y_begin
        Waypoints = [
                    # (0.05, y, 0, "正常模式", 0),
                     (1.8, 0, None, "小斜坡模式", 0),
                     (x, y, 90, "正常模式", 0),
                     (x, 0.45, 90, "木桥B模式", 1, -2, 2), 
                    #  (x, 0.89, 90, "木桥B模式", 1, 0, 4),
                     (x, 1.43, 90, "木桥B模式", 1, -2, 2),
                    #  (x, 1.89, 90, "木桥B模式", 1, 0, 4), 
                     (x, 2.34, 90, "木桥B模式", 1, -2, 2), 
                    #  (x, 3.05 , 90, "木桥B模式", 1, 0, 4), 
                     (x, 3.22, 90, "木桥B模式", 2, -2, 2),
                     (x, y,180, "正常模式", 0, -2 ,2),
                     (self.zones["木桥B"]["begin_x"] - x_begin + 1.0, self.zones["木桥B"]["begin_y"] - y_begin, 180, "正常模式", 0, -4, 4),
                     (self.zones["木桥B"]["begin_x"] - x_begin, self.zones["木桥B"]["begin_y"] - y_begin, 180, "木桥A模式", 0, -2, 2)

        ]
                    #  (self.zones["木桥A"]["begin_x"] - x_begin, self.zones["木桥A"]["begin_y"] - y_begin, 180, "正常模式", 0)]
        if self.index >= len(Waypoints):
            return
        x_target, y_target, yaw_target, self.robot_mode, is_jump = Waypoints[self.index][0:5]
        left = -4
        right = 4
        self.get_logger().info(f"{x}：{y}:{x_target}：{y_target}")
        if len(Waypoints[self.index]) > 5:
            left = Waypoints[self.index][5]
            right = Waypoints[self.index][6]
        if self.navigate_to_pose(x, y, yaw, x_target, y_target, yaw_target, left, right) == True:
            self.index += 1
            # self.get_logger().info(f"{Waypoints[self.index]}：{self.index}")
            self.get_logger().info(f"{x}：{y}:{x_target}：{y_target}")
            # self.get_logger().info(f"{x_target}：{y_target}")
            if is_jump == 1:
                self.change_robot_mode("木桥B模式")
                msg = Float32MultiArray()
                time.sleep(1)
                msg.data = [float(COMMAND_MAP["木桥B模式"]), 1.0]
                self.publisher.publish(msg)
                msg.data = [float(COMMAND_MAP["小跳1"]), 1.0]
                self.publisher.publish(msg)
                self.get_logger().info("{11111111111111111111111111111111111111111111")
                # msg.data = [float(COMMAND_MAP["站立"]), 1.0]
                # self.publisher.publish(msg)
                time.sleep(5)
                # # msg.data = [float(COMMAND_MAP["站立"]), 1.0]
                # # self.publisher.publish(msg)
                self.is_jumping = True
                self.jump_start_time = self.get_clock().now()
            if is_jump == 2:
                self.change_robot_mode("木桥B模式")
                msg = Float32MultiArray()
                time.sleep(1)
                msg.data = [float(COMMAND_MAP["木桥B模式"]), 1.0]
                self.publisher.publish(msg)
                msg.data = [float(COMMAND_MAP["小跳2"]), 1.0]
                self.publisher.publish(msg)
                self.get_logger().info("{11111111111111111111111111111111111111111111")
                msg.data = [float(COMMAND_MAP["站立"]), 1.0]
                self.publisher.publish(msg)
                time.sleep(5)
                self.is_jumping = True
                self.jump_start_time = self.get_clock().now()
            

        self.get_logger().info(f"{self.robot_mode}：{self.index}")


    def handle_bridge_B(self, x, y, yaw, x_begin, y_begin):
        self.reset_index_on_first_entry("handle_bridge_A")
        self.get_logger().info("bridge_A")
        self.set_debug_coords(x, y, yaw, x_begin, y_begin)
        x = x - x_begin
        y = y - y_begin
        self.get_logger().info(f"{x},{y}")
        Waypoints = [(-0.5, 0, 180, "木桥A模式",0,-1,1), 
        (-1.0, 0, 180, "木桥A模式",0,-1,1), 
        (-1.5, 0, 180, "木桥A模式",0,-1,1), 
        (-2.0, 0, 180, "木桥A模式",0,-1,1),
        (-2.5, -0.15, 180, "木桥A模式",0,-1,1)
        ]
        if self.index >= len(Waypoints):
            return
        x_target, y_target, yaw_target, self.robot_mode ,is_jump = Waypoints[self.index][0:5]
        self.get_logger().info(f"{x}：{y}:{x_target}：{y_target}")
        left = -4
        right = 4
        if len(Waypoints[self.index]) > 5:
            left = Waypoints[self.index][5]
            right = Waypoints[self.index][6]
        if self.navigate_to_pose(x, y, yaw, x_target,y_target, yaw_target, left, right) == True:
            self.index += 1
            # if is_jump == 1:
            #         self.change_robot_mode("木桥B模式")
            #         msg = Float32MultiArray()
            #         time.sleep(1)
            #         msg.data = [float(COMMAND_MAP["大斜坡模式"]), 1.0]
            #         self.publisher.publish(msg)
            #         msg.data = [float(COMMAND_MAP["小跳1"]), 1.0]
            #         self.publisher.publish(msg)
            #         self.get_logger().info("{11111111111111111111111111111111111111111111")
            #         # msg.data = [float(COMMAND_MAP["站立"]), 1.0]
            #         # self.publisher.publish(msg)
            #         time.sleep(5)
            #         # # msg.data = [float(COMMAND_MAP["站立"]), 1.0]
            #         # # self.publisher.publish(msg)
            #         self.is_jumping = True
            #         self.jump_start_time = self.get_clock().now()
        self.get_logger().info(f"{self.robot_mode}：{self.last_kKey}")

    def handle_big_slope(self, x, y, yaw, x_begin, y_begin):
        self.reset_index_on_first_entry("handle_big_slope")
        self.get_logger().info("big_slope")
        self.set_debug_coords(x, y, yaw, x_begin, y_begin)
        x = x - x_begin
        y = y - y_begin
        Waypoints = [(0.1, 0, 180, "木桥B模式",1,0),
                      (x, y, -90, "大斜坡模式",0,"右转"), 
                      (x, -0.9, yaw, "大斜坡模式",0,"后退"),
                     (x, y, 180, "大斜坡模式",0,"左转"),
                     (-2.9, y, yaw, "大斜坡模式",1,"前进"),
                     (-4.0, 0, 180, "正常模式",0,0)
                     ]
        if self.index >= len(Waypoints):
            return
        x_target, y_target, yaw_target, self.robot_mode ,is_jump ,limit= Waypoints[self.index]
        self.get_logger().info(f"{x}：{y}:{x_target}：{y_target}")
        left = -4
        right = 4
        if self.navigate_to_pose(x, y, yaw, x_target,
                              y_target, yaw_target,left,right,limit):
            self.index += 1
            if is_jump == 1:
                self.change_robot_mode("木桥B模式")
                msg = Float32MultiArray()
                time.sleep(1)
                msg.data = [float(COMMAND_MAP["木桥B模式"]), 1.0]
                self.publisher.publish(msg)
                msg.data = [float(COMMAND_MAP["小跳1"]), 1.0]
                self.publisher.publish(msg)
                self.get_logger().info("{11111111111111111111111111111111111111111111")
                time.sleep(5)
                self.is_jumping = True
                self.jump_start_time = self.get_clock().now()
        # self.get_logger().info(f"{self.robot_mode}：{self.last_kKey}")

    # def handle_limit_pole_seek(self, x, y, yaw, x_begin, y_begin):
    #     self.reset_index_on_first_entry("handle_limit_pole_seek")
    #     self.get_logger().info("limit_pole_seek")
    #     self.set_debug_coords(x, y, yaw, x_begin, y_begin)
    #     x = x - x_begin
    #     y = y - y_begin
    #     Waypoints = [(-0.1, 0.16, 180, "正常模式",0), (-1.7, y, 180, "正常模式",1)]
    #     if self.index >= len(Waypoints):
    #         return
    #     x_target, y_target, yaw_target, self.robot_mode, is_jump = Waypoints[self.index]
    #     if self.navigate_to_pose(x, y, yaw, x_target,
    #                           y_target, yaw_target):
    #         self.index += 1
    #         if is_jump == 1:
    #                 self.change_robot_mode("沙坑模式")
    #                 msg = Float32MultiArray()
    #                 time.sleep(1)
    #                 msg.data = [float(COMMAND_MAP["沙坑模式"]), 1.0]
    #                 self.publisher.publish(msg)
    #                 msg.data = [float(COMMAND_MAP["小跳1"]), 1.0]
    #                 self.publisher.publish(msg)
    #                 self.get_logger().info("{11111111111111111111111111111111111111111111")
    #                 # msg.data = [float(COMMAND_MAP["站立"]), 1.0]
    #                 # self.publisher.publish(msg)
    #                 time.sleep(5)
    #                 # # msg.data = [float(COMMAND_MAP["站立"]), 1.0]
    #                 # # self.publisher.publish(msg)
    #                 self.is_jumping = True
    #                 self.jump_start_time = self.get_clock().now()
    #     self.get_logger().info(f"{self.robot_mode}：{self.last_kKey}")

    def handle_limit_pole(self, x, y, yaw, x_begin, y_begin):
        self.reset_index_on_first_entry("handle_limit_pole")
        self.get_logger().info("limit_pole")
        self.set_debug_coords(x, y, yaw, x_begin, y_begin)
        x = x - x_begin
        y = y - y_begin
        Waypoints = [(0.5, 0, 180, "正常模式"), (-4.0, 0, 180, "匍匐模式")]
        if self.index >= len(Waypoints):
            return
        x_target, y_target, yaw_target, self.robot_mode  = Waypoints[self.index][0:4]
        if self.navigate_to_pose(x, y, yaw, x_target,
                                 y_target, yaw_target):
            self.index +=nasdfa
            # if is_jump == 1:
            #         self.change_robot_mode("沙坑模式")
            #         msg = Float32MultiArray()
            #         time.sleep(1)
            #         msg.data = [float(COMMAND_MAP["沙坑模式"]), 1.0]
            #         self.publisher.publish(msg)
            #         msg.data = [float(COMMAND_MAP["小跳1"]), 1.0]
            #         self.publisher.publish(msg)
            #         self.get_logger().info("{11111111111111111111111111111111111111111111")
            #         # msg.data = [float(COMMAND_MAP["站立"]), 1.0]
            #         # self.publisher.publish(msg)
            #         time.sleep(5)
            #         # # msg.data = [float(COMMAND_MAP["站立"]), 1.0]
            #         # # self.publisher.publish(msg)
            #         self.is_jumping = True
            #         self.jump_start_time = self.get_clock().now()
        self.get_logger().info(f"{self.robot_mode}：{self.last_kKey}")

    def handle_bunker(self, x, y, yaw, x_begin, y_begin):
        x = x - x_begin
        y = y - y_begin
        self.reset_index_on_first_entry("handle_bunker")
        self.get_logger().info("bunker")
        self.set_debug_coords(x, y, yaw, x_begin, y_begin)

        # Waypoints = [(0.2, y, 180, "正常模式", 1),
        #             (-1.88, -0.04, 135, "沙坑模式", 2),
        #             (-2.2, -0.3, 135, "沙坑模式", 2),
        #             (-2.36, -1, 95, "沙坑模式", 1)]
        Waypoints = [(0.2, y, 180, "正常模式", 1),
            (-1.1, -0.04, -135, "沙坑模式", 1),
            (x, -1.30, -90, "沙坑模式", 3),
            # (-1.5, -1.30, -90, "沙坑模式", 3)
            ]


        if self.index >= len(Waypoints):
            return
        x_target, y_target, yaw_target, self.robot_mode, is_jump = Waypoints[self.index]
        self.get_logger().info(f"{x},{y},{yaw},{x_target},{y_target},{yaw_target}")
        if self.navigate_to_pose(x, y, yaw, x_target,
                                 y_target, yaw_target):
            self.index += 1
            if is_jump == 1:
                self.change_robot_mode("沙坑模式")
                msg = Float32MultiArray()
                time.sleep(1)
                msg.data = [float(COMMAND_MAP["沙坑模式"]), 1.0]
                self.publisher.publish(msg)
                msg.data = [float(COMMAND_MAP["小跳1"]), 1.0]
                self.publisher.publish(msg)
                self.get_logger().info("{11111111111111111111111111111111111111111111")
                msg.data = [float(COMMAND_MAP["站立"]), 1.0]
                self.publisher.publish(msg)
                time.sleep(5)
                self.is_jumping = True
                self.jump_start_time = self.get_clock().now()
            if is_jump == 2:
                self.change_robot_mode("沙坑模式")
                msg = Float32MultiArray()
                time.sleep(1)
                msg.data = [float(COMMAND_MAP["沙坑模式"]), 1.0]
                self.publisher.publish(msg)
                msg.data = [float(COMMAND_MAP["小跳2"]), 1.0]
                self.publisher.publish(msg)
                self.get_logger().info("{11111111111111111111111111111111111111111111")
                msg.data = [float(COMMAND_MAP["站立"]), 1.0]
                self.publisher.publish(msg)
                time.sleep(5)
                self.is_jumping = True
                self.jump_start_time = self.get_clock().now()
            if is_jump == 3:
                self.change_robot_mode("沙坑模式")
                msg = Float32MultiArray()
                msg.data = [float(COMMAND_MAP["前进"]), 1.0]
                self.publisher.publish(msg)
                time.sleep(3)
                msg.data = [float(COMMAND_MAP["沙坑模式"]), 1.0]
                self.publisher.publish(msg)
                msg.data = [float(COMMAND_MAP["小跳1"]), 1.0]
                self.publisher.publish(msg)
                self.get_logger().info("{11111111111111111111111111111111111111111111")
                msg.data = [float(COMMAND_MAP["站立"]), 1.0]
                self.publisher.publish(msg)
                time.sleep(5)
                self.is_jumping = True
                self.jump_start_time = self.get_clock().now()
        self.get_logger().info(f"{self.robot_mode}：{self.last_kKey}")

    def handle_pole(self, x, y, yaw, x_begin, y_begin):
        x = x - x_begin
        y = y - y_begin
        self.reset_index_on_first_entry("handle_pole")
        self.get_logger().info("pole")
        self.set_debug_coords(x, y, yaw, x_begin, y_begin)

        Waypoints = [(0, 0.3, 0, "正常模式"), (0.58, 0.3, -90, "正常模式"), (0.58, -0.95 ,-180, "正常模式")
                     ,(-0.5, -0.95, -90, "正常模式"), (-0.5, -1.8, 0, "正常模式"), (0.6, -1.8, 90, "正常模式")
                     ,(0.6, -0.8, 0, "正常模式"), (1.6, -0.8, -90, "正常模式"), (1.6, -2.0, 0, "正常模式")
                     ,(2.6, -2.0, 90, "正常模式"), (2.6, -1.1, 0, "正常模式"), (3.5, 0, 0, "正常模式")]
        if self.index >= len(Waypoints):
            return
        x_target, y_target, yaw_target, self.robot_mode = Waypoints[self.index]
        left = -5
        right = 5
        if self.navigate_to_pose(x, y, yaw, x_target,
                                 y_target, yaw_target,left,right):
            self.index += 1
        self.get_logger().info(f"{self.robot_mode}：{self.last_kKey}")

    def handle_hill_wall(self, x, y, yaw, x_begin, y_begin):
        x = x - x_begin
        y = y - y_begin
        self.reset_index_on_first_entry("handle_hill_wall")
        self.get_logger().info("hill_wall")
        self.set_debug_coords(x, y, yaw, x_begin, y_begin)
        Waypoints = [(-0.3, y, 0, "木桥B模式", 1), (1, y, 0, "正常模式", 0)]

        if self.index >= len(Waypoints):
            return
        x_target, y_target, yaw_target, self.robot_mode, is_jump = Waypoints[self.index]
        if self.navigate_to_pose(x, y, yaw, x_target,
                                 y_target, yaw_target):
            self.index += 1
            if is_jump == 1:
                self.change_robot_mode("高墙模式")
                msg = Float32MultiArray()
                time.sleep(1)
                msg.data = [float(COMMAND_MAP["高墙模式"]), 1.0]
                self.publisher.publish(msg)
                msg.data = [float(COMMAND_MAP["小跳1"]), 1.0]
                self.publisher.publish(msg)
                self.get_logger().info("{11111111111111111111111111111111111111111111")
                msg.data = [float(COMMAND_MAP["站立"]), 1.0]
                self.publisher.publish(msg)
                time.sleep(5)
                self.is_jumping = True
                self.jump_start_time = self.get_clock().now()
        self.get_logger().info(f"{self.robot_mode}：{self.last_kKey}")

    def handle_step(self, x, y, yaw, x_begin, y_begin):
        self.reset_index_on_first_entry("handle_step")
        self.get_logger().info("step")
        self.set_debug_coords(x, y, yaw, x_begin, y_begin)
        Waypoints = [(-0.1, 0, 0, "正常模式", 1), (x, y, yaw, "正常模式", 2), (x, y, yaw, "正常模式", 2),(1.0, 0, 0, "正常模式", 0)]
        x = x - x_begin
        y = y - y_begin
        if self.index >= len(Waypoints):
            return
        x_target, y_target, yaw_target, self.robot_mode, is_jump = Waypoints[self.index]
        if self.navigate_to_pose(x, y, yaw, x_target,
                                 y_target, yaw_target):
            self.index += 1
            if is_jump == 1:
                self.change_robot_mode("台阶模式")
                msg = Float32MultiArray()
                time.sleep(1)
                msg.data = [float(COMMAND_MAP["台阶模式"]), 1.0]
                self.publisher.publish(msg)
                msg.data = [float(COMMAND_MAP["小跳1"]), 1.0]
                self.publisher.publish(msg)
                self.get_logger().info("{11111111111111111111111111111111111111111111")
                msg.data = [float(COMMAND_MAP["站立"]), 1.0]
                self.publisher.publish(msg)
                time.sleep(5)
                self.is_jumping = True
                self.jump_start_time = self.get_clock().now()
            if is_jump == 2:
                self.change_robot_mode("台阶模式")
                msg = Float32MultiArray()
                time.sleep(1)
                msg.data = [float(COMMAND_MAP["台阶模式"]), 1.0]
                self.publisher.publish(msg)
                msg.data = [float(COMMAND_MAP["小跳2"]), 1.0]
                self.publisher.publish(msg)
                self.get_logger().info("{11111111111111111111111111111111111111111111")
                msg.data = [float(COMMAND_MAP["站立"]), 1.0]
                self.publisher.publish(msg)
                time.sleep(5)
                self.is_jumping = True
                self.jump_start_time = self.get_clock().now()
            if is_jump == 3:
                self.change_robot_mode("台阶模式")
                self.Kkey_To_Number("小跳3")
        self.get_logger().info(f"{self.robot_mode}：{self.last_kKey}")

    def change_robot_mode(self, new_mode):
        if self.robot_mode!= new_mode:
            self.mode_change = 1
            self.robot_mode = new_mode
            self.publish_debug_state()

    def decode_point(self, x_target, y_target, x, y, yaw_now):
        L = math.sqrt((x_target - x) ** 2 + (y_target - y) ** 2)
        yaw = 0
        if L != 0:
            yaw = -math.asin((y - y_target) / L) * 180 / math.pi
        if x > x_target:
            yaw = 180 - yaw
        if yaw > 180:
            yaw = -(360 - yaw)
        if yaw >= 0 and yaw_now >= 0 or yaw <= 0 and yaw_now <= 0:
            if abs(yaw - yaw_now) <= 90:
                L = -L
        else:
            answer = 0
            if abs(yaw - yaw_now) >= 180:
                answer = 360 - abs(yaw - yaw_now)
            else:
                answer = abs(yaw - yaw_now)
            if answer <= 90:
                L = -L
        return L, yaw
    def angle(self,yaw_target, yaw):
        yaw_difference = yaw_target - yaw
        if abs(yaw_difference) > 180:
            if yaw_difference < 0:
                yaw_difference = 360 - abs(yaw_difference)
            else:
                yaw_difference = -(360 - abs(yaw_difference))
        return yaw_difference
    def navigate_to_pose(self, x, y, yaw, x_target, y_target, yaw_end, left_yaw_limit = -4, right_yaw_limit = 4, limit = 0):
        distance, yaw_target = self.decode_point(x_target, y_target, x, y, yaw)
        yaw_difference = self.angle(yaw_target, yaw)
        if distance < -0.05:
            if abs(yaw_difference) < 8:
                if limit != 0:
                    self.Kkey_To_Number(limit)
                else:
                    self.Kkey_To_Number("前进")
                return False
            else:
                if yaw_difference < 0:
                    if limit != 0:
                        self.Kkey_To_Number(limit)
                    else:
                        self.Kkey_To_Number("左转")
                    return False
                else:
                    if limit != 0:
                        self.Kkey_To_Number(limit)
                    else:
                        self.Kkey_To_Number("右转")
                    return False
        # elif distance > 0.07:
        #     if yaw_target <= 0:
        #         yaw_target += 180
        #     else:
        #         yaw_target -= 180
        #     yaw_difference = self.angle(yaw_target, yaw)
        #     if abs(yaw_difference) < 5:
        #         self.Kkey_To_Number("后退")
        #         return False
        #     else:
        #         if yaw_difference < 0:
        #             self.Kkey_To_Number("左转")
        #             return False
        #         else:
        #             self.Kkey_To_Number("右转")
        #             return False
        else:
            if yaw_end == None:
                return True
            yaw_difference = self.angle(yaw_end, yaw)
            if left_yaw_limit < yaw_difference < right_yaw_limit:
                self.Kkey_To_Number("站立")
                return True
            else:
                
                if yaw_difference < 0:
                    if limit != 0:
                        self.Kkey_To_Number(limit)
                    else:
                        self.Kkey_To_Number("左转")
                    return False
                else:
                    if limit != 0:
                        self.Kkey_To_Number(limit)
                    else:
                        self.Kkey_To_Number("右转")
                    return False

    

    def Quadruped_Racing_Param_Trans(self, x1, y1, x2, y2, x_p, y_p, L):

        x_trans = ((x_p - x1)*(x2 - x1) + (y_p - y1)*(y2 - y1))/L
        y_trans = ((x_p - x1)*(y2 - y1) - (y_p - y1)*(x2 - x1))/L
        yaw_offset = abs(math.degrees(math.atan((x2-x1)/(y2-y1))))
        
        return x_trans,y_trans,yaw_offset
    def Kkey_To_Number(self, Kkey):
        if self.stop == 0:
            self.mode_change = 0
            msg = Float32MultiArray()
            msg.data = [float(COMMAND_MAP[self.robot_mode]), 2.0]
            self.publisher.publish(msg)
            self.current_action = Kkey
            self.publish_debug_state()
 
            jump_keys = ["小跳1", "小跳2"]
            
            if Kkey in jump_keys:
                msg = Float32MultiArray()
                msg.data = [float(COMMAND_MAP[Kkey]), 4.0]
                self.publisher.publish(msg)
                self.last_kKey = Kkey
                self.publish_debug_state()
                return 
            if self.last_kKey != Kkey:
                if Kkey == "前进":
                    self.get_logger().info(f"{Kkey}！！！\n")
                    msg = Float32MultiArray()
                    msg.data = [float(COMMAND_MAP[Kkey]), 4.0]
                    self.publisher.publish(msg)
                    self.last_kKey = Kkey
                    self.publish_debug_state()
                elif Kkey == "后退":
                    self.get_logger().info(f"{Kkey}！！！\n")
                    msg = Float32MultiArray()
                    msg.data = [float(COMMAND_MAP[Kkey]), 4.0]
                    self.publisher.publish(msg)
                    self.last_kKey = Kkey
                    self.publish_debug_state()
                elif Kkey == "左转":
                    self.get_logger().info(f"{Kkey}！！！\n")
                    msg = Float32MultiArray()
                    msg.data = [float(COMMAND_MAP[Kkey]),4.0]
                    self.publisher.publish(msg)
                    self.last_kKey = Kkey
                    self.publish_debug_state()
                elif Kkey == "右转":
                    self.get_logger().info(f"{Kkey}！！！\n")
                    msg = Float32MultiArray()
                    msg.data = [float(COMMAND_MAP[Kkey]), 4.0]
                    self.publisher.publish(msg)
                    self.last_kKey = Kkey
                    self.publish_debug_state()
                elif Kkey == "站立":
                    self.get_logger().info(f"{Kkey}！！！\n")
                    msg = Float32MultiArray()
                    msg.data = [float(COMMAND_MAP[Kkey]), 4.0]
                    self.publisher.publish(msg)
                    self.last_kKey = Kkey
                    self.publish_debug_state()
                elif Kkey == "左平移":
                    self.get_logger().info(f"{Kkey}！！！\n")
                    msg = Float32MultiArray()
                    msg.data = [float(COMMAND_MAP[Kkey]), 4.0]
                    self.publisher.publish(msg)
                    self.last_kKey = Kkey
                    self.publish_debug_state()
                elif Kkey == "右平移":
                    self.get_logger().info(f"{Kkey}！！！\n")
                    msg = Float32MultiArray()
                    msg.data = [float(COMMAND_MAP[Kkey]), 4.0]
                    self.publisher.publish(msg)
                    self.last_kKey = Kkey
                    self.publish_debug_state()
                elif Kkey == "下台阶":
                    self.get_logger().info(f"{Kkey}！！！\n")
                    msg = Float32MultiArray()
                    msg.data = [float(COMMAND_MAP[Kkey]), 4.0]
                    self.publisher.publish(msg)
                    self.last_kKey = Kkey
                    self.publish_debug_state()
               
            else:
                self.last_kKey = 0
             
        else:
            self.get_logger().info(f"no key！！！\n")
            self.last_kKey = 0
            self.stop = 0
            self.current_action = "等待执行"
            self.publish_debug_state()
