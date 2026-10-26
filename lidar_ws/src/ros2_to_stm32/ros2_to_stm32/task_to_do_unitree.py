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
    "小跳1": 6, "小跳2": 7,"大斜坡左":1,
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

        self.route_profile = self.declare_parameter('route_profile', 'demo').value
        if self.route_profile not in ('demo', 'zones'):
            raise ValueError('route_profile 必须为 demo 或 zones')
        self.motor_state = {}
        self.command = None
        self.request_id = 0
        self.command_time = 0.0
        self.pending_jump = None
        self.jump_sent = False
        self.demo_index = 0
        self.zone_indices = {}
        self.active_handler = None
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
        self.set_normal_debug_coords(x, y, yaw)
        if self.pending_jump is not None:
            mode, key = self.pending_jump
            if not self.jump_sent:
                self.jump_sent = self.send_command(mode, key)
            elif not self.motor_state.get('busy', True):
                self.pending_jump = None
                self.jump_sent = False
                self.command = None
                self.index += 1
            return
        if self.route_profile == 'demo':
            self.current_zone = '两点顺序测试，不是完整赛道'
            self.robot_mode = '正常模式'
            targets = [(1.0, 0.0, 0.0), (1.0, -1.0, 0.0)]
            if self.demo_index < len(targets):
                if self.navigate_to_pose(x, y, yaw, *targets[self.demo_index]):
                    self.demo_index += 1
            else:
                self.Kkey_To_Number('站立')
            return
        for kind, value in self.zones.items():
            if value['x_min'] <= x < value['x_max'] and value['y_min'] <= y < value['y_max']:
                self.current_zone = kind
                handler = getattr(self, value['func_name'])
                handler(x, y, yaw, value['begin_x'], value['begin_y'])
                return
        for kind, value in self.zone.items():
            if value['x_min'] <= x < value['x_max'] and value['y_min'] <= y < value['y_max']:
                target = self.zones[value['func_name']]
                self.current_zone = kind
                handler = getattr(self, target['func_name'])
                handler(x, y, yaw, target['begin_x'], target['begin_y'])
                return
        self.current_zone = '未进入障碍区域，站立等待'
        self.Kkey_To_Number('站立')

    def updata_kKey(self):   # update
        self.last_kKey = 0
        self.stop = 1
        self.current_action = "等待执行"
        self.publish_debug_state()

    def reset_index_on_first_entry(self, handler_name):
        if self.active_handler != handler_name:
            if self.active_handler is not None:
                self.zone_indices[self.active_handler] = self.index
            self.index = self.zone_indices.get(handler_name, 0)
            self.active_handler = handler_name

    def reset_index_manually(self):
        self.index = 0
        self.demo_index = 0
        self.zone_indices.clear()
        self.active_handler = None
        self.pending_jump = None
        self.jump_sent = False
        self.command = None
        self.current_action = '路线进度已清零'
        self.publish_debug_state()

    def handle_bridge_A(self, x, y, yaw, x_begin, y_begin):
        self.reset_index_on_first_entry("handle_bridge_A")
        self.get_logger().info("bridge_B")
        self.set_debug_coords(x, y, yaw, x_begin, y_begin)
        x = x - x_begin
        y = y - y_begin
        Waypoints = [
                     (1.8, 0, None, "小斜坡模式", 0),
                     (x, y, -90, "正常模式", 0),
                     (x, -0.35, -90, "木桥B模式", 1, -2, 2), 
                     (x, -1.32, -90, "木桥B模式", 1, -2, 2),
                     (x, -2.47, -90, "木桥B模式", 1, -2, 2), 
                     (x, -3.10, -90, "木桥B模式", 2, -2, 2),
                     (x, y,-180, "正常模式", 0, -2 ,2),
                     (self.zones["木桥B"]["begin_x"] - x_begin + 0.5, self.zones["木桥B"]["begin_y"] - y_begin + 0.05, 180, "正常模式", 0, -4, 4),
                     (self.zones["木桥B"]["begin_x"] - x_begin, self.zones["木桥B"]["begin_y"] - y_begin+0.05, 180, "木桥B模式", 0, -1, 1)

        ]
        if self.index >= len(Waypoints):
            self.Kkey_To_Number("站立")
            return
        x_target, y_target, yaw_target, self.robot_mode, is_jump = Waypoints[self.index][0:5]
        left = -4
        right = 4
        self.get_logger().info(f"{x}：{y}:{x_target}：{y_target}")
        if len(Waypoints[self.index]) > 5:
            left = Waypoints[self.index][5]
            right = Waypoints[self.index][6]
        if self.navigate_to_pose(x, y, yaw, x_target, y_target, yaw_target, left, right):
            if is_jump:
                mode = "木桥B模式"
                key = 7 if is_jump == 2 else 6
                self.begin_action(COMMAND_MAP[mode], key)
            else:
                self.index += 1

    def handle_bridge_B(self, x, y, yaw, x_begin, y_begin):
        self.reset_index_on_first_entry("handle_bridge_B")
        self.get_logger().info("bridge_A")
        self.set_debug_coords(x, y, yaw, x_begin, y_begin)
        x = x - x_begin
        y = y - y_begin
        self.get_logger().info(f"{x},{y}")
        Waypoints = [
        (-2.0, 0.05, -180, "木桥A模式",0,-2,2),
        (-2.5, 0.1, -180, "木桥A模式",0,-1,1)
        ]
        if self.index >= len(Waypoints):
            self.Kkey_To_Number("站立")
            return
        x_target, y_target, yaw_target, self.robot_mode ,is_jump = Waypoints[self.index][0:5]
        self.get_logger().info(f"{x}：{y}:{x_target}：{y_target}")
        left = -4
        right = 4
        if len(Waypoints[self.index]) > 5:
            left = Waypoints[self.index][5]
            right = Waypoints[self.index][6]
        if self.navigate_to_pose(x, y, yaw, x_target, y_target, yaw_target, left, right):
            self.index += 1

    def handle_big_slope(self, x, y, yaw, x_begin, y_begin):
        self.reset_index_on_first_entry("handle_big_slope")
        self.get_logger().info("big_slope")
        self.set_debug_coords(x, y, yaw, x_begin, y_begin)
        x = x - x_begin
        y = y - y_begin
        Waypoints = [(0.2, 0, -180, "木桥B模式",1,0),
                      (x, y, 90, "大斜坡模式",0,"左转"), 
                      (x, 1.0, yaw, "大斜坡模式",0,"后退"),
                     (x, y, -180, "大斜坡模式",0,"右转"),
                     (-2.9, y, yaw, "大斜坡模式",1,"前进"),
                     (-3.0, 0, -180, "正常模式",0,0)
                     ]
        if self.index >= len(Waypoints):
            self.Kkey_To_Number("站立")
            return
        x_target, y_target, yaw_target, self.robot_mode ,is_jump ,limit= Waypoints[self.index]
        self.get_logger().info(f"{x}：{y}:{x_target}：{y_target}")
        left = -4
        right = 4
        if self.navigate_to_pose(x, y, yaw, x_target, y_target, yaw_target, left, right, limit):
            if is_jump:
                mode = "木桥B模式"
                key = 7 if is_jump == 2 else 6
                self.begin_action(COMMAND_MAP[mode], key)
            else:
                self.index += 1

    def handle_limit_pole(self, x, y, yaw, x_begin, y_begin):
        self.reset_index_on_first_entry("handle_limit_pole")
        self.get_logger().info("limit_pole")
        self.set_debug_coords(x, y, yaw, x_begin, y_begin)
        x = x - x_begin
        y = y - y_begin
        Waypoints = [(0.5, 0, -180, "正常模式"), (-4.0, 0, -180, "匍匐模式")]
        if self.index >= len(Waypoints):
            self.Kkey_To_Number("站立")
            return
        x_target, y_target, yaw_target, self.robot_mode  = Waypoints[self.index][0:4]
        if self.navigate_to_pose(x, y, yaw, x_target, y_target, yaw_target):
            self.index += 1

    def handle_bunker(self, x, y, yaw, x_begin, y_begin):
        x = x - x_begin
        y = y - y_begin
        self.reset_index_on_first_entry("handle_bunker")
        self.get_logger().info("bunker")
        self.set_debug_coords(x, y, yaw, x_begin, y_begin)

        Waypoints = [(0.2, y, -180, "正常模式", 1),
            (-1.1, 0.04, 135, "沙坑模式", 1),
            (x, 1.25, 90, "沙坑模式", 3),
            ]


        if self.index >= len(Waypoints):
            self.Kkey_To_Number("站立")
            return
        x_target, y_target, yaw_target, self.robot_mode, is_jump = Waypoints[self.index]
        self.get_logger().info(f"{x},{y},{yaw},{x_target},{y_target},{yaw_target}")
        if self.navigate_to_pose(x, y, yaw, x_target, y_target, yaw_target):
            if is_jump:
                mode = "沙坑模式"
                key = 7 if is_jump == 2 else 6
                self.begin_action(COMMAND_MAP[mode], key)
            else:
                self.index += 1

    def handle_pole(self, x, y, yaw, x_begin, y_begin):
        x = x - x_begin
        y = y - y_begin
        self.reset_index_on_first_entry("handle_pole")
        self.get_logger().info("pole")
        self.set_debug_coords(x, y, yaw, x_begin, y_begin)

        Waypoints = [(0, -0.4, 0, "正常模式"), (0.5, -0.5, 90, "正常模式"), (0.4, 0.7 ,180, "正常模式")
                     ,(-0.5, 0.6, 90, "正常模式"), (-0.4, 1.6, 0, "正常模式"), (0.5,1.5, -90, "正常模式")
                     ,(0.4,0.5, 0, "正常模式"), (1.5, 0.6, 90, "正常模式"), (1.4, 1.5, 0, "正常模式")
                     ,(2.4, 1.4, -90, "正常模式"), (2.5, 0.5, 0, "正常模式"), (3.5, 1.0, 0, "正常模式")]
        if self.index >= len(Waypoints):
            self.Kkey_To_Number("站立")
            return
        x_target, y_target, yaw_target, self.robot_mode = Waypoints[self.index]
        left = -7
        right = 7
        if self.navigate_to_pose(x, y, yaw, x_target, y_target, yaw_target, left, right):
            self.index += 1

    def handle_hill_wall(self, x, y, yaw, x_begin, y_begin):
        x = x - x_begin
        y = y - y_begin
        self.reset_index_on_first_entry("handle_hill_wall")
        self.get_logger().info("hill_wall")
        self.set_debug_coords(x, y, yaw, x_begin, y_begin)
        Waypoints = [(-0.2, y, 0, "木桥B模式", 1), (1, y, 0, "正常模式", 0)]

        if self.index >= len(Waypoints):
            self.Kkey_To_Number("站立")
            return
        x_target, y_target, yaw_target, self.robot_mode, is_jump = Waypoints[self.index]
        if self.navigate_to_pose(x, y, yaw, x_target, y_target, yaw_target):
            if is_jump:
                mode = "高墙模式"
                key = 7 if is_jump == 2 else 6
                self.begin_action(COMMAND_MAP[mode], key)
            else:
                self.index += 1

    def handle_step(self, x, y, yaw, x_begin, y_begin):
        self.reset_index_on_first_entry("handle_step")
        self.get_logger().info("step")
        self.set_debug_coords(x, y, yaw, x_begin, y_begin)
        x = x - x_begin
        y = y - y_begin
        Waypoints = [(-0.15, 0, 0, "正常模式", 1), (0.32, y, yaw, "正常模式", 2), (0.62, y, yaw, "正常模式", 2),(1.0, 0, 0, "正常模式", 0)]
        if self.index >= len(Waypoints):
            self.Kkey_To_Number("站立")
            return
        x_target, y_target, yaw_target, self.robot_mode, is_jump = Waypoints[self.index]
        if self.navigate_to_pose(x, y, yaw, x_target, y_target, yaw_target):
            if is_jump:
                mode = "正常模式" if is_jump == 3 else "台阶模式"
                key = 7 if is_jump == 2 else 6
                self.begin_action(COMMAND_MAP[mode], key)
            else:
                self.index += 1

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
    def navigate_to_pose(self, x, y, yaw, x_target, y_target, yaw_end, left_yaw_limit=-4, right_yaw_limit=4, limit=0):
        distance = math.hypot(x_target - x, y_target - y)
        if distance > 0.05:
            target_heading = math.degrees(math.atan2(y_target - y, x_target - x))
            difference = self.angle(target_heading, yaw)
            if limit:
                action = limit
            elif abs(difference) < 8:
                action = '前进'
            else:
                action = '左转' if difference < 0 else '右转'
            self.Kkey_To_Number(action)
            return False
        if yaw_end is not None:
            difference = self.angle(yaw_end, yaw)
            if not left_yaw_limit < difference < right_yaw_limit:
                self.Kkey_To_Number(limit or ('左转' if difference < 0 else '右转'))
                return False
        return self.Kkey_To_Number('站立')

    def Quadruped_Racing_Param_Trans(self, x1, y1, x2, y2, x_p, y_p, L):

        x_trans = ((x_p - x1)*(x2 - x1) + (y_p - y1)*(y2 - y1))/L
        y_trans = ((x_p - x1)*(y2 - y1) - (y_p - y1)*(x2 - x1))/L
        yaw_offset = abs(math.degrees(math.atan((x2-x1)/(y2-y1))))
        
        return x_trans,y_trans,yaw_offset
    def Kkey_To_Number(self, Kkey):
        self.current_action = Kkey
        self.publish_debug_state()
        return self.send_command(COMMAND_MAP[self.robot_mode], COMMAND_MAP[Kkey])

    def send_command(self, mode, key):
        if not self.motor_state.get('armed', False):
            return False
        if self.command is not None:
            last_tx = self.motor_state.get('last_tx') or {}
            if last_tx.get('request_id') == self.command['request_id']:
                if (self.command['mode'], self.command['key']) == (mode, key):
                    return True
                self.command = None
            elif self.motor_state.get('pending_id') == self.command['request_id']:
                return False  # 串口节点已接收，正在等模式切换确认后发送
            elif time.monotonic() - self.command_time < 1.0:
                return False
            else:
                # 1 秒内串口节点既没发出也没接收（例如刚恢复、或当时正忙），重新发；
                # 只有确认没被接收才重发，所以跳跃不会被发两次
                self.command = None
        if self.motor_state.get('busy', False):
            return False
        self.request_id += 1
        self.command = dict(mode=mode, key=key, request_id=self.request_id)
        message = Float32MultiArray()
        message.data = [float(key), 1.0, float(mode), float(self.request_id)]
        self.publisher.publish(message)
        self.command_time = time.monotonic()
        return False

    def begin_action(self, mode, key):
        self.pending_jump = (mode, key)
        self.jump_sent = False
        self.command = None
        self.send_command(mode, key)
