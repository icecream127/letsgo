"""Run saved two-point navigation with real Nav2/GICP and a simulated base.

Run after sourcing run/environment.sh, in an isolated ROS domain (>= 190)
with ROS_LOCALHOST_ONLY=1. This test does not start the real sensor drivers.
Use --parallel to test the old parallel horse UART interface over a PTY.
Use --parallel-wait or --parallel-drop to exercise unavailable feedback.
Use --preview to exercise the same route with planning only.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import pty
import signal
import subprocess
import tempfile
import time
import tty

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ros2_to_stm32.motor_protocol import crc8
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


ROOT = Path(__file__).resolve().parents[1]


def write_fixture(directory, points, targets):
    size = 160
    grid = np.full((size, size), 254, dtype=np.uint8)
    grid[:2, :] = grid[-2:, :] = grid[:, :2] = grid[:, -2:] = 0
    pgm = b'P5\n160 160\n255\n' + grid.tobytes()
    map_yaml = (b'image: map.pgm\nresolution: 0.05\norigin: [-4.0, -4.0, 0.0]\n'
                b'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    count = len(points)
    pcd = (f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
           f'WIDTH {count}\nHEIGHT 1\nPOINTS {count}\nDATA binary\n').encode()
    pcd += points.astype('<f4').tobytes()
    (directory / 'map.pgm').write_bytes(pgm)
    (directory / 'map.yaml').write_bytes(map_yaml)
    (directory / 'cloud.pcd').write_bytes(pcd)
    (directory / 'projection.json').write_text(json.dumps({
        'source': 'cloud.pcd',
        'source_sha256': hashlib.sha256(pcd).hexdigest(),
        'image_sha256': hashlib.sha256(pgm).hexdigest(),
        'yaml_sha256': hashlib.sha256(map_yaml).hexdigest(),
    }))
    binding = {'frame_id': 'map', 'map_fingerprint': hashlib.sha256(map_yaml + pgm).hexdigest()}
    goals = [{'x': x, 'y': 0.0, 'yaw': yaw} for x, yaw in targets]
    project = json.dumps({
        **binding, 'version': 1, 'start_area': {'pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}},
        'tasks': [{
            'id': 'test-segment', 'name': 'test-segment', 'mode': 'nav',
            'entry': goals[0], 'exit': goals[1],
        }],
    }).encode()
    (directory / 'competition_tasks.json').write_bytes(project)
    (directory / 'nav2_goals.json').write_text(json.dumps({
        **binding, 'project_sha256': hashlib.sha256(project).hexdigest(), 'goals': goals,
    }))


class ParallelHorse:
    """UART peer with latched discrete motion, independent of /cmd_vel."""
    def __init__(self, master):
        self.master = master
        self.payload = bytearray()
        self.keys = []
        self.mode = 'b'
        self.walk_key = 5
        self.feedback = True

    def read_keys(self):
        try:
            self.payload.extend(os.read(self.master, 4096))
        except BlockingIOError:
            pass
        while len(self.payload) >= 3:
            header, key, trailer = self.payload[:3]
            del self.payload[:3]
            assert (header, trailer) == (0x53, 0x45)
            assert key in (1, 2, 3, 4, 5, 9), self.keys + [key]
            self.keys.append(key)
            if key == 9:
                self.mode = 'a'
            else:
                assert self.mode == 'a' or key == 5, self.keys
                self.walk_key = key

    def velocity(self):
        self.read_keys()
        if self.feedback:
            response = bytes((0xA5, 0x5A, ord(self.mode), 0, 0, 0))
            os.write(self.master, response + bytes((crc8(response), 0x0D)))
        return {1: (0.0, 0.5), 2: (0.2, 0.0), 3: (0.0, -0.5),
                4: (-0.15, 0.0), 5: (0.0, 0.0)}[self.walk_key]


class SimulatedBase(Node):
    def __init__(self, points, horse):
        super().__init__('auto_navigation_test_base')
        self.points = points
        self.horse = horse
        self.odom = self.create_publisher(Odometry, '/Odometry', 10)
        self.cloud = self.create_publisher(PointCloud2, '/cloud_registered_body', 10)
        self.tf = TransformBroadcaster(self)
        self.velocity = Twist()
        self.velocity_time = time.monotonic()
        self.last_tick = self.velocity_time
        self.x = self.y = self.yaw = 0.0
        self.commands = 0
        self.arrivals = {}
        self.goal_statuses = {}
        self.preview = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Twist, '/cmd_vel', self.receive_command, 10)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self.record_goal_status, latched,
        )
        self.create_subscription(NavPath, '/auto_nav/preview_path', self.record_preview, latched)
        self.create_timer(0.08, self.tick)

    def receive_command(self, msg):
        self.velocity = msg
        self.velocity_time = time.monotonic()
        if abs(msg.linear.x) + abs(msg.angular.z) > 0.01:
            self.commands += 1

    def record_goal_status(self, msg):
        for goal in msg.status_list:
            goal_id = bytes(goal.goal_info.goal_id.uuid)
            self.goal_statuses[goal_id] = goal.status
            if goal.status == GoalStatus.STATUS_SUCCEEDED:
                self.arrivals.setdefault(goal_id, (self.x, self.y, self.yaw))

    def record_preview(self, msg):
        self.preview = msg

    def tick(self):
        now = time.monotonic()
        elapsed = now - self.last_tick
        self.last_tick = now
        if self.horse is not None:
            v, w = self.horse.velocity()
        elif now - self.velocity_time <= 0.5:
            v, w = self.velocity.linear.x, self.velocity.angular.z
        else:
            v = w = 0.0
        self.x += elapsed * v * math.cos(self.yaw)
        self.y += elapsed * v * math.sin(self.yaw)
        self.yaw += elapsed * w

        stamp = self.get_clock().now().to_msg()
        odom = Odometry(header=Header(stamp=stamp, frame_id='odom'), child_frame_id='livox_frame')
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
        self.odom.publish(odom)

        tf = TransformStamped(header=odom.header, child_frame_id=odom.child_frame_id)
        tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(tf)

        # Fixed walls in map coordinates, observed relative to the moving sensor.
        dx, dy = self.points[:, 0] - self.x, self.points[:, 1] - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        body_points = np.column_stack((c * dx + s * dy, -s * dx + c * dy, self.points[:, 2]))
        self.cloud.publish(create_cloud_xyz32(Header(stamp=stamp, frame_id='livox_frame'), body_points))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--parallel', action='store_true')
    modes.add_argument('--parallel-wait', action='store_true')
    modes.add_argument('--parallel-drop', action='store_true')
    modes.add_argument('--preview', action='store_true')
    args = parser.parse_args()
    if (int(os.environ.get('ROS_DOMAIN_ID', '0')) < 190
            or os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        raise RuntimeError('Set an isolated test domain and ROS_LOCALHOST_ONLY=1')

    angle = np.arange(720) * 2 * math.pi / 720
    c, s = np.cos(angle), np.sin(angle)
    distance = 3.9 / np.maximum(np.abs(c), np.abs(s))
    heights = (-0.2, 0.0, 0.25, 0.5)
    points = np.column_stack((np.repeat(distance * c, len(heights)),
                              np.repeat(distance * s, len(heights)), np.tile(heights, len(angle))))
    with tempfile.TemporaryDirectory(prefix='test_auto_nav_', dir=ROOT / 'maps') as path, \
            tempfile.TemporaryFile(mode='w+t') as log:
        fixture = Path(path)
        parallel = args.parallel or args.parallel_wait or args.parallel_drop
        targets = ((0.75, 0.0), (1.5, 0.6 if parallel else 0.0))
        write_fixture(fixture, points, targets)
        command = ['bash', 'run/auto_nav.sh', fixture.name, '--imu-z', '0', '--no-sensors', '--no-rviz']
        horse = None
        master = slave = None
        try:
            if args.preview:
                command.append('--preview')
            if parallel:
                master, slave = pty.openpty()
                tty.setraw(slave)
                os.set_blocking(master, False)
                horse = ParallelHorse(master)
                horse.feedback = not args.parallel_wait
                command.append('--parallel')
            if parallel or args.preview:
                profile = fixture / 'test_robot.yaml'
                port = os.ttyname(slave) if parallel else fixture / 'unused_serial'
                startup_timeout = 1.0 if args.preview or args.parallel_wait else 15.0
                profile.write_text(
                    f'motor_serial:\n  ros__parameters:\n    port: {port}\n'
                    f'auto_nav:\n  ros__parameters:\n    startup_timeout: {startup_timeout}\n'
                )
                command.extend(['--params-file', str(profile)])
            run_mission(command, points, targets, horse, log, args)
        finally:
            if master is not None:
                os.close(master)
                os.close(slave)


def spin_until(robot, process, predicate, timeout):
    deadline = time.monotonic() + timeout
    while not predicate() and process.poll() is None and time.monotonic() < deadline:
        rclpy.spin_once(robot, timeout_sec=0.08)
    return predicate()


def captured_log(log):
    return os.pread(log.fileno(), os.fstat(log.fileno()).st_size, 0).decode(errors='replace')


def assert_navigation_running(robot, process, log):
    assert process.poll() is None, captured_log(log)
    assert ('bt_navigator', '/') in robot.get_node_names_and_namespaces(), captured_log(log)


def check_preview(robot, process, targets, horse, log):
    def full_preview():
        path = robot.preview
        if path is None or len(path.poses) < 3:
            return False
        final = path.poses[-1].pose.position
        return math.hypot(final.x - targets[-1][0], final.y) <= 0.25

    assert spin_until(robot, process, full_preview, 45.0), captured_log(log)
    preview = robot.preview
    assert preview.header.frame_id == 'map', preview.header
    start = preview.poses[0].pose.position
    assert math.hypot(start.x, start.y) <= 0.25, start
    assert any(
        math.hypot(p.pose.position.x - targets[0][0], p.pose.position.y) <= 0.15
        for p in preview.poses[:-1]
    ), 'Preview misses the first goal'
    spin_until(robot, process, lambda: False, 2.0)
    assert_navigation_running(robot, process, log)
    assert not robot.goal_statuses and robot.commands == 0, captured_log(log)
    if horse is None:
        assert ('motor_serial', '/') not in robot.get_node_names_and_namespaces(), captured_log(log)
    else:
        assert not any(1 <= key <= 4 for key in horse.keys), horse.keys
    label = 'preview only' if horse is None else 'waiting for parallel horse feedback'
    print(f'PASS: {label}: both legs visible, navigation alive, no motion goals')


def check_feedback_loss(robot, process, horse, log):
    assert spin_until(robot, process, lambda: horse.walk_key == 2, 45.0), captured_log(log)
    horse.feedback = False

    def canceled_and_standing():
        return (horse.walk_key == 5
                and GoalStatus.STATUS_CANCELED in robot.goal_statuses.values())

    assert spin_until(robot, process, canceled_and_standing, 8.0), captured_log(log)
    assert_navigation_running(robot, process, log)
    assert len(robot.goal_statuses) == 1 and not robot.arrivals, robot.goal_statuses
    horse.feedback = True
    spin_until(robot, process, lambda: False, 2.0)
    assert_navigation_running(robot, process, log)
    assert horse.walk_key == 5 and len(robot.goal_statuses) == 1, captured_log(log)
    print('PASS: feedback loss requests stand, cancels first goal and holds navigation')


def check_arrivals(robot, targets, horse, log):
    if horse is None:
        assert robot.commands > 0, captured_log(log)
    else:
        horse.read_keys()
        assert horse.keys[:2] == [5, 9], horse.keys
        assert 2 in horse.keys and 1 in horse.keys, horse.keys
        assert horse.keys[-1] == 5, horse.keys
    assert len(robot.arrivals) == len(targets), (robot.arrivals, captured_log(log))
    for (target_x, target_yaw), (x, y, yaw) in zip(targets, robot.arrivals.values()):
        assert math.hypot(x - target_x, y) <= 0.30 and abs(yaw - target_yaw) <= 0.30, (x, y, yaw)
    backend = 'parallel horse PTY keys' if horse is not None else 'gated velocity'
    print(f'PASS: saved start -> GICP -> Nav2 -> {backend} -> ordered two-goal completion',
          list(robot.arrivals.values()))


def stop_test_process(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def run_mission(command, points, targets, horse, log, args):
    rclpy.init()
    robot = SimulatedBase(points, horse)
    try:
        process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            if args.preview or args.parallel_wait:
                check_preview(robot, process, targets, horse, log)
            elif args.parallel_drop:
                check_feedback_loss(robot, process, horse, log)
            elif horse is not None:
                def arrived_and_standing():
                    return len(robot.arrivals) == len(targets) and horse.walk_key == 5

                assert spin_until(robot, process, arrived_and_standing, 60.0), captured_log(log)
                spin_until(robot, process, lambda: False, 1.0)
                assert_navigation_running(robot, process, log)
                check_arrivals(robot, targets, horse, log)
            else:
                spin_until(robot, process, lambda: process.poll() is not None, 60.0)
                assert process.returncode == 0, captured_log(log)
                check_arrivals(robot, targets, horse, log)
        finally:
            stop_test_process(process)
        if args.parallel:
            assert process.returncode == 0, captured_log(log)
        elif args.parallel_drop:
            assert process.returncode == 1, captured_log(log)
        elif args.preview or args.parallel_wait:
            assert process.returncode in (0, 1), captured_log(log)
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
