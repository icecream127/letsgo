"""两点实机路线；复用原来的走点判断、发送去重和控制器回调。"""
import json
import math
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import Trigger

from .info_transmiting import Info_Trans
from .task_to_do_unitree import Task_To_Do


class TaskTodoMini(Task_To_Do):
    def __init__(self):
        # 启动脚本通过 LEGACY_ZONE_CONFIG 指向空区域的 mini 地图。
        # 保留原类的参数、发布器、发送状态及重置逻辑。
        super().__init__()
        with Path(__file__).with_name('map_mini.json').open(encoding='utf-8') as stream:
            self.waypoints = json.load(stream)['waypoints']
        self.get_logger().info('MINI 路线：(1, 0) → (2, 0.2)，身体中心坐标，普通模式')

    def Quadruped_Obstacle(self, x, y, yaw):
        # 与原路线入口相同，只补偿一次；不调用父类的区域路线。
        theta = math.radians(yaw)
        x -= self.sensor_forward_offset * math.cos(theta)
        y -= self.sensor_forward_offset * math.sin(theta)
        self.robot_mode = '正常模式'
        if self.index >= len(self.waypoints):
            self.current_zone = 'MINI 两点路线完成'
            self.set_normal_debug_coords(x, y, yaw)
            self.Kkey_To_Number('站立')
            return

        target = self.waypoints[self.index]
        self.current_zone = f"MINI 第 {self.index + 1} 点 ({target['x']}, {target['y']})"
        self.set_normal_debug_coords(x, y, yaw)
        # 直接复用原版：5cm到点、8度前进门限、原来的左右转方向。
        if self.navigate_to_pose(x, y, yaw, target['x'], target['y'], target['yaw_end']):
            self.get_logger().info(f'已到达 MINI 第 {self.index + 1} 点')
            self.index += 1


class MiniController(Info_Trans):
    def __init__(self):
        # 只替换任务实例；回调、超时检查、心跳、路线重置都继承原控制器。
        Node.__init__(self, 'info_trans')
        self.task_to_do = TaskTodoMini()
        self.state_time = float('-inf')
        self.pose_time = float('-inf')
        self.latest_pose = None
        self.error_until = 0.0
        self.heartbeat = self.create_publisher(Bool, '/controller_alive', 10)
        self.create_subscription(String, '/motor_state', self.state_callback, 10)
        self.create_subscription(Float32MultiArray, '/pos_trans', self.pos_trans_callback, 1)
        self.create_subscription(Float32MultiArray, '/task_params', self.calibration_callback, 10)
        self.create_service(Trigger, '/automation/reset_route', self.reset_route)
        self.create_timer(0.1, self.update)


def main(args=None):
    rclpy.init(args=args)
    node = MiniController()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.task_to_do)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.task_to_do.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
