"""Request full parallel horse initialization over the authorized USART3 link."""
import argparse
import math
from pathlib import Path
import sys
import time

import serial
import yaml

from robot_params import ROBOT_PARAMS, script_params
from ros2_to_stm32.motor_protocol import StatusParser, encode_key


INITIALIZE_KEY = 22
# In a healthy USART3 status frame, bit 7 of detail advertises host initialization.
HOST_INIT_CAPABILITY = 0x80


def wait_for_status(port, parser, deadline):
    while time.monotonic() < deadline:
        frames = parser.feed(port.read(min(max(port.in_waiting, 1), 4096)))
        if frames:
            return frames[-1]
    return None


def require_host_init_status(status):
    if status['fault']:
        raise RuntimeError(
            f"电控故障 {status['fault']}，电机 {status['motor']}，详情 {status['detail']}"
        )
    if not status['detail'] & HOST_INIT_CAPABILITY:
        raise RuntimeError('当前电控固件未声明支持小电脑初始化，请使用配套 USART3 固件')


def initialize(port, feedback_timeout, initialization_timeout):
    parser = StatusParser()
    status = wait_for_status(port, parser, time.monotonic() + feedback_timeout)
    if status is None:
        raise TimeoutError('未收到有效电控回传，请确认遥控器已授权自动，并核对 USART3 接线')
    require_host_init_status(status)
    if status['mode'] != 'z':
        print(f"电控已处于运行态（模式 {status['mode']}）。", flush=True)
        return

    frame = encode_key(INITIALIZE_KEY)
    if port.write(frame) != len(frame):
        raise OSError('初始化帧未完整写入，请核对电控状态')
    print('已发送键 22，等待电控进入运行态。', flush=True)

    deadline = time.monotonic() + initialization_timeout
    while time.monotonic() < deadline:
        feedback_deadline = min(deadline, time.monotonic() + feedback_timeout)
        status = wait_for_status(port, parser, feedback_deadline)
        if status is None:
            if time.monotonic() >= deadline:
                break
            raise TimeoutError('初始化期间电控回传中断，请检查现场电控和遥控器状态')
        require_host_init_status(status)
        if status['mode'] != 'z':
            print(f"初始化完成，电控已进入运行态（模式 {status['mode']}）。", flush=True)
            return
    raise TimeoutError('初始化未在限时内完成，请检查电控状态')


def main():
    parser = argparse.ArgumentParser(description='通过 USART3 向已授权的并马发送完整初始化键')
    parser.add_argument('--params-file', type=Path, default=ROBOT_PARAMS, help='试验参数覆盖文件')
    args = parser.parse_args()

    try:
        settings = script_params('motor_serial', args.params_file)
        feedback_timeout = float(settings['status_timeout'])
        initialization_timeout = float(settings['initialization_timeout'])
        if not all(math.isfinite(value) and value > 0.0
                   for value in (feedback_timeout, initialization_timeout)):
            raise ValueError('电控回传与初始化超时须为有限正数，单位秒')
        with serial.Serial(
            settings['port'], settings['baudrate'], bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=min(0.1, feedback_timeout, initialization_timeout),
            write_timeout=0.2, exclusive=True,
        ) as port:
            port.reset_input_buffer()
            initialize(port, feedback_timeout, initialization_timeout)
        return 0
    except KeyboardInterrupt:
        print('已停止等待，请现场核对电控初始化状态。', file=sys.stderr, flush=True)
        return 130
    except (OSError, RuntimeError, TypeError, ValueError, KeyError, yaml.YAMLError) as exc:
        print(f'初始化未确认：{exc}', file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
