import json
import math
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import Trigger

from .task_to_do_unitree import Task_To_Do


class Info_Trans(Node):
    """任务控制器：拿定位结果跑路线，把命令交给串口节点。

    串口节点报告可以控制（armed）时才跑路线；暂停后恢复，从当前路线进度继续。
    """

    def __init__(self):
        super().__init__('info_trans')
        self.task_to_do = Task_To_Do()
        self.state_time = float('-inf')
        self.pose_time = float('-inf')
        self.latest_pose = None
        self.error_until = 0.0  # 路线代码出错后短暂停发心跳，串口节点会请马站住
        self.heartbeat = self.create_publisher(Bool, '/controller_alive', 10)
        self.create_subscription(String, '/motor_state', self.state_callback, 10)
        self.create_subscription(Float32MultiArray, '/pos_trans', self.pos_trans_callback, 1)
        self.create_subscription(Float32MultiArray, '/task_params', self.calibration_callback, 10)
        self.create_service(Trigger, '/automation/reset_route', self.reset_route)
        self.create_timer(0.1, self.update)

    def calibration_callback(self, message):
        if message.data and message.data[0] in (1.0, 2.0):
            if not self.task_to_do.motor_state.get('armed', False):
                self.task_to_do.reset_index_manually()

    def state_callback(self, message):
        try:
            state = json.loads(message.data)
        except (ValueError, TypeError):
            return
        previously_armed = self.task_to_do.motor_state.get('armed', False)
        self.task_to_do.motor_state = state
        self.state_time = time.monotonic()
        if previously_armed and not state.get('armed', False):
            # 暂停时串口节点已丢掉未发的命令；恢复后重新发送当前命令
            self.task_to_do.command = None

    def pos_trans_callback(self, message):
        if len(message.data) == 3 and all(math.isfinite(value) for value in message.data):
            self.latest_pose = (message.data[0], message.data[1], -message.data[2])
            self.pose_time = time.monotonic()

    def reset_route(self, request, response):
        response.success = not self.task_to_do.motor_state.get('busy', False)
        if response.success:
            self.task_to_do.reset_index_manually()
            response.message = '已清零路线进度；继续运行前确认机器人位置和姿态'
        else:
            response.message = '一次性动作执行中，等它做完再清零'
        return response

    def update(self):
        task = self.task_to_do
        now = time.monotonic()
        self.heartbeat.publish(Bool(data=now >= self.error_until))
        if now - self.state_time > 1.0:
            task.current_action = '等待串口节点'
            task.publish_debug_state()
            return
        if not task.motor_state.get('armed', False):
            task.current_action = task.motor_state.get('reason', '等待马回传状态')
            task.publish_debug_state()
            return
        if self.latest_pose is None or now - self.pose_time > 1.0:
            return  # 串口节点也会判定定位超时并请马站住
        try:
            task.Quadruped_Obstacle(*self.latest_pose)
        except (ValueError, KeyError, IndexError, RuntimeError) as error:
            self.get_logger().error(f'路线代码出错，暂停 1.5 秒：{error}')
            self.error_until = now + 1.5
            task.command = None


def main(args=None):
    rclpy.init(args=args)
    node = Info_Trans()
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
