import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('--duration', type=float, default=42)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    config = root / 'lidar_ws/src/ros2_to_stm32/config/fast_lio_mid360_real.yaml'
    os.environ['ROS_DOMAIN_ID'] = '183'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    output = root / 'validation'
    output.mkdir(exist_ok=True)
    rclpy.init()
    node = Node('standalone_sensor_validation')
    result = {'odometry_count': 0, 'scan_count': 0, 'map_points': [], 'frames': {},
              'nonfinite_poses': 0}
    subscriptions = []

    def odometry(message):
        result['odometry_count'] += 1
        result['frames']['odometry'] = [message.header.frame_id, message.child_frame_id]
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        if not all(math.isfinite(value) for value in
                   [position.x, position.y, position.z, orientation.x,
                    orientation.y, orientation.z, orientation.w]):
            result['nonfinite_poses'] += 1

    def scan(message):
        result['scan_count'] += 1
        result['frames']['scan'] = message.header.frame_id

    def cloud_map(message):
        result['map_points'].append(message.width * message.height)
        result['frames']['map'] = message.header.frame_id

    subscriptions.append(node.create_subscription(Odometry, '/Odometry', odometry, 20))
    subscriptions.append(node.create_subscription(PointCloud2, '/cloud_registered', scan, 5))
    subscriptions.append(node.create_subscription(PointCloud2, '/Laser_map', cloud_map, 1))
    processes = []
    logs = []
    before = set(root.rglob('*.pcd'))
    try:
        for name, command in [
            ('fastlio', ['ros2', 'run', 'fast_lio', 'fastlio_mapping', '--ros-args',
                         '--params-file', str(config)]),
            ('replay', ['ros2', 'bag', 'play', args.bag, '--topics',
                        '/livox/lidar', '/livox/imu', '--delay', '3']),
        ]:
            log = (output / (name + '.log')).open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True, cwd=root))
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if processes[0].poll() is not None:
                raise RuntimeError('FAST-LIO exited during replay')
        print(json.dumps(result, indent=2), flush=True)
        (output / 'replay_partial.json').write_text(json.dumps(result, indent=2))
        client = node.create_client(Trigger, '/map_save')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('map_save service unavailable')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=15)
        if future.result() is None:
            raise RuntimeError('map_save timed out')
        result['save_response'] = {'success': future.result().success,
                                   'message': future.result().message}
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
    result['new_pcd_files'] = [str(path) for path in set(root.rglob('*.pcd')) - before]
    positive_maps = [count for count in result['map_points'] if count > 0]
    result['passed'] = (
        result['odometry_count'] > 10 and result['scan_count'] > 10
        and len(positive_maps) > 2 and positive_maps[-1] > positive_maps[0]
        and result['frames'].get('odometry') == ['odom', 'livox_frame']
        and result['frames'].get('map') == 'odom'
        and result['nonfinite_poses'] == 0
        and result['save_response']['success'] is False
        and not result['new_pcd_files']
    )
    (output / 'replay_result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
