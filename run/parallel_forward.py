"""Start H743 TROT and use FAST-LIO to test a short forward move."""
import argparse
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
import serial
import yaml

from horse_pc_link import HorsePcLink, LinkError
from robot_params import ROBOT_PARAMS, script_params
from ros2_to_stm32.nav_geometry import rotate


class TravelOdometry(Node):
    def __init__(self, settings):
        offset = np.array(settings['livox_to_base_xyz'], dtype=float)
        if offset.shape != (3,) or not np.isfinite(offset).all():
            raise ValueError('livox_to_base_xyz 必须包含三个有限数值')
        super().__init__('parallel_forward_test')
        self.offset = offset
        self.settings = settings
        self.sample = None
        self.error = None
        self.create_subscription(Odometry, '/Odometry', self.on_odom, qos_profile_sensor_data)

    def stamp_is_fresh(self, stamp):
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        return -self.settings['odom_future_tolerance'] <= age <= self.settings['odom_max_age']

    def on_odom(self, msg):
        if (msg.header.frame_id, msg.child_frame_id) != ('odom', 'livox_frame'):
            self.error = '/Odometry 坐标系不是 odom → livox_frame'
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self.stamp_is_fresh(stamp):
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        position, rotation = np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])
        norm = np.linalg.norm(rotation)
        if not np.isfinite(position).all() or not math.isfinite(norm) or norm < 1e-9:
            self.error = '/Odometry 位姿无效'
            return
        xy = (position + rotate(rotation / norm, self.offset))[:2]
        if self.sample is not None:
            last_stamp, last_xy = self.sample
            dt = stamp - last_stamp
            if dt <= 0:
                if dt < -self.settings['odom_reset_threshold']:
                    self.error = '/Odometry 时间戳重置'
                return
            jump_limit = max(self.settings['odom_jump_distance'], self.settings['odom_jump_speed'] * dt)
            if dt < self.settings['odom_jump_check_window'] and np.linalg.norm(xy - last_xy) > jump_limit:
                self.error = '/Odometry 位置跳变'
                return
        self.sample = stamp, xy

    def position(self):
        if self.error is not None:
            raise RuntimeError(self.error)
        if self.sample is not None and self.stamp_is_fresh(self.sample[0]):
            return self.sample[1]
        return None


# A fresh K1 request renews the firmware's 500 ms control lease.
STATUS_PERIOD = 0.05


def require_live_operation(status):
    if status['fault'] or status['state'] == 'FAULT' or status['phase'] in {
        'PREEMPTED', 'TIMEOUT', 'FAULT', 'DENIED',
    }:
        raise LinkError(
            f"电控结束请求：{status['state']} / {status['phase']}，"
            f"故障码 {status['fault']}，在线掩码 0x{status['online_mask']:02X}"
        )


def wait_for_distance(link, odom, settings):
    start_deadline = time.monotonic() + settings['forward_test_start_timeout']
    target = settings['forward_test_distance']
    previous = None
    origin = None
    next_status_at = 0.0

    while True:
        rclpy.spin_once(odom, timeout_sec=0.02)
        position = odom.position()
        if position is None:
            raise TimeoutError('/Odometry 过期')
        now = time.monotonic()
        if origin is None:
            if now >= start_deadline:
                raise TimeoutError('电控未在限时内进入 TROT/WALKING')
        else:
            distance = float(np.linalg.norm(position - origin))
            if distance >= target:
                return distance
            if now >= move_deadline:
                raise TimeoutError(f'前进超时，测得位移 {distance:.2f} 米')

        if now >= next_status_at:
            status = link.status()
            next_status_at = time.monotonic() + STATUS_PERIOD
            require_live_operation(status)
            current = (status['state'], status['gait'], status['phase'])
            if current != previous:
                print('电控状态 ' + ' / '.join(current), flush=True)
                previous = current
            if status['phase'] not in {'STARTING', 'WALKING'} or status['gait'] != 'trot':
                raise LinkError(f'电控退出 TROT 启动或行走，状态为 {current}')
            if current == ('RUN', 'trot', 'WALKING') and origin is None:
                origin = odom.position()
                if origin is None:
                    raise TimeoutError('开始行走时 /Odometry 过期')
                move_deadline = time.monotonic() + settings['forward_test_timeout']
                print(f'已记录行走起点，目标位移 {target:.2f} 米。', flush=True)
            elif origin is not None and current != ('RUN', 'trot', 'WALKING'):
                raise LinkError(f'电控退出行走，状态为 {current}')


def stop_and_confirm(link, timeout):
    link.command('Z')
    print('电控已收到收步请求，等待 STOPPED。', flush=True)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = link.status()
        if status['phase'] == 'STOPPED':
            print(f"电控已确认停止（{status['state']} / STOPPED）。", flush=True)
            return status
        require_live_operation(status)
        time.sleep(STATUS_PERIOD)
    raise TimeoutError('未收到 STOPPED，请核对电控和实车状态')


def run_forward_test(link, odom, settings):
    ready_deadline = time.monotonic() + settings['forward_test_ready_timeout']
    while odom.position() is None:
        if time.monotonic() >= ready_deadline:
            raise TimeoutError('未收到有效 /Odometry，未请求电控启动')
        rclpy.spin_once(odom, timeout_sec=0.02)

    link.connect()
    status = link.status()
    require_live_operation(status)
    if status['state'] not in {'READY', 'RUN'} or status['phase'] != 'IDLE':
        raise LinkError(
            f"当前为 {status['state']} / {status['phase']}，请检查电机与起立准备"
        )
    rclpy.spin_once(odom, timeout_sec=0)
    if odom.position() is None:
        raise TimeoutError('/Odometry 已过期，未请求电控启动')

    distance = None
    try:
        link.command('T')
        print('电控已受理 TROT 请求，等待预检、起立和行走。', flush=True)
        distance = wait_for_distance(link, odom, settings)
    finally:
        try:
            stopped = stop_and_confirm(link, settings['forward_test_stop_timeout'])
        except (OSError, LinkError) as exc:
            if distance is not None:
                raise
            print(f'收步未确认：{exc}', file=sys.stderr, flush=True)
    if stopped['state'] != 'RUN' or stopped['fault']:
        raise LinkError(f"电控已停止，但未保持健康站立（{stopped['state']}，故障码 {stopped['fault']}）")
    print(f'触发收步时的里程计位移为 {distance:.2f} 米。', flush=True)


def main():
    parser = argparse.ArgumentParser(description='通过 H743 USART3 请求自启动，测量前进 0.5 米后收步')
    parser.add_argument('--params-file', type=Path, default=ROBOT_PARAMS, help='试验参数覆盖文件')
    args = parser.parse_args()
    try:
        settings = script_params('motor_serial', args.params_file)
        for name in ('forward_test_distance', 'forward_test_timeout',
                     'forward_test_ready_timeout', 'forward_test_start_timeout',
                     'forward_test_stop_timeout'):
            value = settings[name] = float(settings[name])
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} 必须为有限正数')
        odom_settings = script_params('nav_adapter', args.params_file)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        odom = None
        try:
            odom = TravelOdometry(odom_settings)
            with serial.Serial(
                settings['port'], settings['baudrate'], timeout=0.02,
                write_timeout=0.1, exclusive=True,
            ) as port:
                port.reset_input_buffer()
                run_forward_test(HorsePcLink(port), odom, settings)
        finally:
            if odom is not None:
                odom.destroy_node()
            rclpy.shutdown()
        return 0
    except KeyboardInterrupt:
        print('已终止测试，请用遥控器确认站立。', file=sys.stderr, flush=True)
        return 130
    except (OSError, RuntimeError, TypeError, ValueError, KeyError, yaml.YAMLError) as exc:
        print(f'前进通信测试未完成：{exc}', file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
