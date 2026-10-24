import json
import math
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import SetBool, Trigger

from .task_to_do_unitree import Task_To_Do


class Info_Trans(Node):
    def __init__(self):
        super().__init__('info_trans')
        self.task_to_do = Task_To_Do()
        self.state_time = float('-inf')
        self.pose_time = float('-inf')
        self.latest_pose = None
        self.startup_pause = None
        self.pause_client = self.create_client(SetBool, '/automation/enable')
        self.heartbeat = self.create_publisher(Bool, '/controller_alive', 10)
        self.route_status = self.create_publisher(String, '/route_state', 10)
        self.create_subscription(String, '/motor_state', self.state_callback, 10)
        self.create_subscription(Float32MultiArray, '/pos_trans', self.pos_trans_callback, 1)
        self.create_subscription(Float32MultiArray, '/task_params', self.calibration_callback, 10)
        self.create_service(Trigger, '/automation/reset_route', self.reset_route)
        self.create_timer(0.1, self.update)

    def calibration_callback(self, message):
        if message.data and message.data[0] in range(1, 11):
            if not self.task_to_do.motor_state.get('armed', False):
                self.task_to_do.reset_index_manually()

    def state_callback(self, message):
        try:
            state = json.loads(message.data)
            previously_armed = self.task_to_do.motor_state.get('armed', False)
            self.task_to_do.motor_state = state
            self.state_time = time.monotonic()
            if previously_armed and not state.get('armed', False):
                if self.task_to_do.pending_jump is not None:
                    self.task_to_do.recovery_required = True
                    self.get_logger().error('一次性动作期间暂停，结果需人工检查；重置路线后才能继续')
                self.task_to_do.command = None
                self.task_to_do.navigation_state = None
        except (ValueError, TypeError):
            self.request_pause('电控状态消息无效')

    def pos_trans_callback(self, message):
        if len(message.data) == 3 and all(math.isfinite(value) for value in message.data):
            self.latest_pose = (message.data[0], message.data[1], -message.data[2])
            self.pose_time = time.monotonic()

    def request_pause(self, reason):
        self.get_logger().error(reason)
        self.task_to_do.recovery_required = True
        if self.pause_client.service_is_ready():
            self.pause_client.call_async(SetBool.Request(data=False))

    def reset_route(self, request, response):
        state = self.task_to_do.motor_state
        response.success = not state.get('armed', False) and not state.get('busy', False)
        if response.success:
            self.task_to_do.reset_index_manually()
            response.message = '已清零路线进度；重新运行前确认机器人位置和姿态'
        else:
            response.message = '请先暂停并等待动作保护时间结束'
        return response

    def update(self):
        task = self.task_to_do
        now = time.monotonic()
        if self.startup_pause is None:
            if self.pause_client.service_is_ready():
                self.startup_pause = self.pause_client.call_async(SetBool.Request(data=False))
            return
        if not self.startup_pause.done():
            return
        try:
            task.refresh_route()
            route_reason = task.route_review_reason()
        except (OSError, ValueError, KeyError, TypeError, RuntimeError) as error:
            route_reason = f'路线配置异常：{error}'
        self.route_status.publish(String(data=json.dumps(dict(
            ready=not route_reason, reason=route_reason, profile=task.route_profile,
            map_path=task.zm.filepath, map_sha256=task.map_hash,
            issues=task.map_issues), ensure_ascii=False)))
        if route_reason and task.motor_state.get('armed', False):
            self.request_pause(route_reason)
        healthy = not task.recovery_required
        self.heartbeat.publish(Bool(data=healthy))
        if now - self.state_time > 1.0:
            if task.motor_state.get('armed', False):
                self.request_pause('电控桥接节点超时；请确认机器人已安全停止')
                task.motor_state['armed'] = False
            return
        if route_reason or not healthy or not task.motor_state.get('armed', False):
            task.current_action = route_reason or task.motor_state.get('reason', '等待人工允许运行')
            task.publish_debug_state()
            return
        if self.latest_pose is None or now - self.pose_time > 1.0:
            self.request_pause('定位超时')
            return
        try:
            task.Quadruped_Obstacle(*self.latest_pose)
        except (ValueError, KeyError, IndexError, RuntimeError) as error:
            self.request_pause(f'任务暂停：{error}')


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
