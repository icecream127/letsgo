"""Start GICP and Nav2, then run the map's saved ordinary navigation goals."""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import yaml

from nav_goals import load_map_goals, resolve_goal_timeout, send_goals
from navigation_gicp import add_navigation_arguments, prepare_navigation
from robot_params import script_params


class ParallelHorseState:
    def __init__(self, node, timeout):
        from std_msgs.msg import String

        self.state = {}
        self.received_at = -math.inf
        self.timeout = timeout
        self.subscription = node.create_subscription(String, '/parallel_horse/status', self.receive, 10)

    def receive(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(state, dict):
            self.state = state
            self.received_at = time.monotonic()

    def reason(self):
        if time.monotonic() - self.received_at > self.timeout:
            return '等待并马串口节点状态'
        if self.state.get('ready') is not True:
            return f"并马电控未就绪（{self.state.get('reason')}）"
        return None

    def stood_since(self, started_at):
        return self.received_at >= started_at and self.state.get('last_key') == 5


def wait_for_navigation(node, action, state, gate, timeout, start_launch, check_running, motor):
    import rclpy
    from go2_obstacle_interfaces.msg import LocalizationEvidence
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformException, TransformListener

    def stamp_age(stamp):
        clock = node.get_clock()
        return (clock.now() - Time.from_msg(stamp, clock_type=clock.clock_type)).nanoseconds * 1e-9

    def fresh_sensor(stamp):
        return -gate['sensor_future_tolerance'] <= stamp_age(stamp) <= gate['sensor_stamp_max_age']

    def on_odom(msg):
        if fresh_sensor(msg.header.stamp):
            last_sensor['odom'] = time.monotonic()

    def on_scan(msg):
        if fresh_sensor(msg.header.stamp) and sum(
                math.isfinite(r) and msg.range_min <= r <= msg.range_max for r in msg.ranges
        ) >= gate['min_valid_scan_ranges']:
            last_sensor['scan'] = time.monotonic()

    def on_evidence(msg):
        nonlocal evidence
        if msg.source_id and msg.header.frame_id == 'map' and msg.body_frame == 'livox_frame':
            evidence = msg

    evidence = None
    last_sensor = {'odom': -math.inf, 'scan': -math.inf}
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    subscriptions = [
        node.create_subscription(LocalizationEvidence, '/gicp/evidence', on_evidence, 10),
        node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data),
        node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data),
    ]
    try:
        start_launch()
        deadline = time.monotonic() + timeout
        state_request = None
        active = False
        while True:
            check_running()
            if state_request is None and state.service_is_ready():
                state_request = state.call_async(GetState.Request())
            if state_request is not None and state_request.done():
                reply = state_request.result()
                active = reply is not None and reply.current_state.id == State.PRIMARY_STATE_ACTIVE
                state_request = None

            pending = []
            if not active or not action.server_is_ready():
                pending.append('Nav2 未激活')
            if evidence is None or not evidence.accepted:
                pending.append('等待首次 GICP 定位成功')
            try:
                transform = buffer.lookup_transform('map', 'odom_nav', Time())
                fresh_tf = (-gate['tf_future_tolerance'] <= stamp_age(transform.header.stamp)
                            <= gate['tf_max_age'])
                if not fresh_tf or not buffer.can_transform('map', 'base_footprint', Time()):
                    pending.append('全局底盘 TF 未就绪')
            except TransformException:
                pending.append('全局底盘 TF 未就绪')
            now = time.monotonic()
            for key, received in last_sensor.items():
                if now - received > gate[f'{key}_timeout']:
                    pending.append(f'等待有效 /{key}')
            if motor is not None:
                reason = motor.reason()
                if reason:
                    pending.append(reason)
            elif node.count_subscribers('/cmd_vel') == 0:
                pending.append('未发现 /cmd_vel 订阅端')
            if not pending:
                return
            if now >= deadline:
                raise TimeoutError('启动等待超时：' + '，'.join(pending))
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        listener.unregister()
        for subscription in subscriptions:
            node.destroy_subscription(subscription)


def wait_for_zero_velocity(node, process, gate, motor):
    """Let the gate publish its final stop before tearing down its publisher."""
    import rclpy
    from geometry_msgs.msg import Twist

    stopped = False

    def on_velocity(msg):
        nonlocal stopped
        stopped = msg.linear.x == 0.0 and msg.angular.z == 0.0

    sub = node.create_subscription(Twist, '/cmd_vel', on_velocity, 10)
    started_at = time.monotonic()
    deadline = started_at + gate['cmd_timeout'] + 4 * gate['output_period']
    try:
        while time.monotonic() < deadline and process.poll() is None:
            if stopped and (motor is None or motor.stood_since(started_at)):
                return True
            rclpy.spin_once(node, timeout_sec=.05)
        return False
    finally:
        node.destroy_subscription(sub)


def stop_launch(process):
    if process is None or process.poll() is not None:
        return
    for signum, grace in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            if signum == signal.SIGINT:
                process.send_signal(signum)
            else:
                os.killpg(process.pid, signum)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=grace)
        except subprocess.TimeoutExpired:
            continue
        break


def run_navigation(command, goals, args, gate, startup_timeout, goal_timeout, motor_settings):
    import rclpy
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions

    def on_terminate(_signum, _frame):
        raise KeyboardInterrupt

    def check_launch():
        status = process.poll()
        if status is not None:
            raise RuntimeError(f'导航启动进程提前退出（退出码 {status}）')

    def check_execution():
        check_launch()
        if motor is not None:
            reason = motor.reason()
            if reason is not None:
                raise RuntimeError(reason)

    def start_launch():
        nonlocal process
        process = subprocess.Popen(command, start_new_session=True)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('auto_nav', parameter_overrides=[
        Parameter('use_sim_time', value=args.use_sim_time),
    ])
    process = None
    action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    state = node.create_client(GetState, '/bt_navigator/get_state')
    motor = ParallelHorseState(node, motor_settings['nav_command_timeout']) if motor_settings else None
    previous_handler = signal.signal(signal.SIGTERM, on_terminate)
    try:
        if state.wait_for_service(timeout_sec=1.) or action.server_is_ready():
            raise RuntimeError('当前 ROS 域已发现 Nav2，请先停止已有导航后再启动自动任务')
        if motor is not None and ('motor_serial', '/') in node.get_node_names_and_namespaces():
            raise RuntimeError('当前 ROS 域已有串口控制节点，请先停止旧控制程序')
        print('正在启动 GICP 和 Nav2，等待定位、里程计、扫描与执行端。', flush=True)
        wait_for_navigation(node, action, state, gate, startup_timeout, start_launch, check_launch, motor)
        print('导航已就绪，开始执行任务。', flush=True)
        try:
            succeeded = send_goals(node, action, goals, goal_timeout, check_execution)
        finally:
            stopped = wait_for_zero_velocity(node, process, gate, motor)
            if not stopped:
                confirmation = '并马站立帧写入状态' if motor is not None else '/cmd_vel 零速度'
                print(f'关闭导航前未确认{confirmation}，请检查底盘是否停车。', file=sys.stderr, flush=True)
        return 0 if succeeded and stopped else 1
    except KeyboardInterrupt:
        print('任务已中断，正在关闭本次启动的导航。', flush=True)
        return 1
    finally:
        try:
            stop_launch(process)
        finally:
            signal.signal(signal.SIGTERM, previous_handler)
            action.destroy()
            node.destroy_node()
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_navigation_arguments(parser)
    parser.add_argument('--parallel', action='store_true', help='用原并马串口执行普通导航')
    args = parser.parse_args()
    try:
        command, startup, map_yaml = prepare_navigation(args)
        goals = load_map_goals(map_yaml, args.project)
        if len(goals) < 2:
            raise ValueError('普通多点导航需要至少两个已保存的目标')
        startup_timeout = float(script_params('auto_nav', args.params_file)['startup_timeout'])
        if not math.isfinite(startup_timeout) or startup_timeout <= 0:
            raise ValueError('自动导航启动超时必须为正数，单位秒')
        goal_timeout = resolve_goal_timeout(params_file=args.params_file)
        gate = script_params('nav_cmd_gate', args.params_file)
        motor_settings = script_params('motor_serial', args.params_file) if args.parallel else None
        if args.parallel:
            command.append('parallel_horse:=true')
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    if args.check_only:
        print(json.dumps({**startup, 'frame_id': 'map', 'goals': goals}, ensure_ascii=False, indent=2))
        return 0
    try:
        return run_navigation(command, goals, args, gate, startup_timeout, goal_timeout, motor_settings)
    except (OSError, RuntimeError) as exc:
        print(f'任务停止：{exc}', flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
