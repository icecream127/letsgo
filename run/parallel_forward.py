"""Test the parallel horse's USART3 forward and stand keys after remote startup."""
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

from robot_params import ROBOT_PARAMS, script_params
from ros2_to_stm32.motor_protocol import MODE_STATUS, StatusParser, encode_key
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


def send_key(port, key):
    frame = encode_key(key)
    if port.write(frame) != len(frame):
        raise OSError(f'按键 {key} 的串口帧未完整写入')


def run_forward_test(port, odom, settings):
    status_parser = StatusParser()
    status_at = -math.inf

    def poll():
        nonlocal status_at
        rclpy.spin_once(odom, timeout_sec=0.02)
        now = time.monotonic()
        data = port.read(min(port.in_waiting, 4096))
        for status in status_parser.feed(data):
            if status['fault']:
                raise RuntimeError(f"电控故障 {status['fault']}，电机 {status['motor']}")
            if status['mode'] != MODE_STATUS[9]:
                raise RuntimeError(f"当前电控模式为 {status['mode']}，请用遥控器完成初始化并选普通模式")
            status_at = now
        return now, now - status_at < settings['status_timeout'], odom.position()

    ready_deadline = time.monotonic() + settings['forward_test_ready_timeout']
    while True:
        now, status_fresh, origin = poll()
        if status_fresh and origin is not None:
            break
        if now >= ready_deadline:
            missing = []
            if not status_fresh:
                missing.append('电控自动挡回传')
            if origin is None:
                missing.append('有效 /Odometry')
            raise TimeoutError('未收到' + '及'.join(missing) + '，未发送前进键')

    target = settings['forward_test_distance']
    deadline = time.monotonic() + settings['forward_test_timeout']
    try:
        send_key(port, 2)
        next_send = time.monotonic() + settings['nav_send_period']
        print(f'已发送前进键 2，目标位移 {target:.2f} 米。', flush=True)
        while True:
            now, status_fresh, position = poll()
            if not status_fresh:
                raise TimeoutError('电控回传中断')
            if position is None:
                raise TimeoutError('/Odometry 过期')
            distance = float(np.linalg.norm(position - origin))
            if distance >= target:
                break
            if now >= deadline:
                raise TimeoutError(f'前进超时，测得位移 {distance:.2f} 米')
            if now >= next_send:
                send_key(port, 2)
                next_send = now + settings['nav_send_period']
    finally:
        send_key(port, 5)
        print('已写入站立键 5，请现场确认已站稳。', flush=True)
    print(f'触发停止时的里程计位移为 {distance:.2f} 米。', flush=True)


def main():
    parser = argparse.ArgumentParser(description='遥控器初始化后，测试 USART3 前进和站立')
    parser.add_argument('--params-file', type=Path, default=ROBOT_PARAMS, help='试验参数覆盖文件')
    args = parser.parse_args()
    try:
        settings = script_params('motor_serial', args.params_file)
        for name in ('status_timeout', 'nav_send_period', 'forward_test_distance',
                     'forward_test_timeout', 'forward_test_ready_timeout'):
            value = settings[name] = float(settings[name])
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} 必须为有限正数')
        odom_settings = script_params('nav_adapter', args.params_file)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        odom = None
        try:
            odom = TravelOdometry(odom_settings)
            with serial.Serial(
                settings['port'], settings['baudrate'], timeout=0,
                write_timeout=0.2, exclusive=True,
            ) as port:
                port.reset_input_buffer()
                run_forward_test(port, odom, settings)
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
