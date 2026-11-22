"""Check parallel-horse ROS commands and USART3 frames with a virtual serial port.

Run with ROS_LOCALHOST_ONLY=1 and an isolated ROS_DOMAIN_ID of at least 190.
The child process opens only the PTY allocated by this script.
"""
import argparse
import json
import os
from pathlib import Path
import pty
import signal
import subprocess
import tempfile
import time
import tty

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from ros2_to_stm32.motor_protocol import crc8


ROOT = Path(__file__).resolve().parents[1]
MOTION_KEYS = {1, 2, 3, 4}


class VirtualBoard:
    def __init__(self, master):
        self.master = master
        self.mode = None
        self.fault = 0
        self.next_status = 0.0
        self.keys = []
        self.buffer = bytearray()

    def exchange(self, now):
        if self.mode is not None and now >= self.next_status:
            payload = bytes((0xA5, 0x5A, ord(self.mode), self.fault, 0, 0))
            os.write(self.master, payload + bytes((crc8(payload), 0x0D)))
            self.next_status = now + .08
        try:
            self.buffer.extend(os.read(self.master, 4096))
        except BlockingIOError:
            pass
        while len(self.buffer) >= 3:
            first, key, last = self.buffer[:3]
            assert (first, last) == (0x53, 0x45), self.buffer
            assert key in MOTION_KEYS | {5, 9}, key
            self.keys.append(key)
            del self.buffer[:3]


def exercise_bridge(probe, motor, process, require_authorization):
    commands = probe.create_publisher(Twist, '/cmd_vel', 10)
    authorizations = probe.create_publisher(Bool, '/parallel_horse/motion_authorized', 1)
    permission = None
    state = {}

    def receive_status(msg):
        state.update(json.loads(msg.data))

    subscription = probe.create_subscription(String, '/parallel_horse/status', receive_status, 10)
    next_command = next_authorization = 0.0

    def pump(predicate, command=None, seconds=3.0):
        nonlocal next_command, next_authorization
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            now = time.monotonic()
            assert process.poll() is None, f'Bridge exited ({process.returncode})'
            if command is not None and now >= next_command:
                message = Twist()
                message.linear.x, message.angular.z = command
                commands.publish(message)
                next_command = now + .04
            if permission is not None and now >= next_authorization:
                authorizations.publish(Bool(data=permission))
                next_authorization = now + .04
            motor.exchange(now)
            rclpy.spin_once(probe, timeout_sec=.01)
            if predicate():
                return True
        return False

    def wait_for(predicate, command=None, seconds=3.0):
        assert pump(predicate, command, seconds), (state, motor.keys, probe.get_node_names(),
                                                   probe.get_topic_names_and_types())

    def pump_for(seconds, command=None):
        pump(lambda: False, command, seconds)

    try:
        wait_for(lambda: bool(state), seconds=10.0)
        assert state['authorization_required'] == require_authorization, state
        pump_for(.35, (.18, 0.0))
        assert not state['ready'] and not motor.keys, (state, motor.keys)

        motor.mode = 'z'
        wait_for(lambda: state['mode'] == 'z', (.18, 0.0))
        motor.mode, motor.fault = 'c', 1
        wait_for(lambda: state['fault'] == 1, (.18, 0.0))
        assert not state['ready'] and not motor.keys, (state, motor.keys)

        motor.fault = 0
        wait_for(lambda: 9 in motor.keys, (.18, 0.0))
        assert motor.keys[:2] == [5, 9], motor.keys
        pump_for(.25)
        assert not state['ready'] and not MOTION_KEYS.intersection(motor.keys), (state, motor.keys)

        motor.mode = 'a'
        wait_for(lambda: state['ready'])
        pump_for(.15)
        assert not MOTION_KEYS.intersection(motor.keys), motor.keys

        if require_authorization:
            pump_for(.4, (.18, 0.0))
            assert state['ready'] and not state['motion_authorized'], state
            assert not MOTION_KEYS.intersection(motor.keys), motor.keys
            permission = True
            wait_for(lambda: state['motion_authorized'])
            assert not MOTION_KEYS.intersection(motor.keys), motor.keys

        count = len(motor.keys)
        wait_for(lambda: motor.keys[count:].count(2) >= 2, (.18, 0.0))
        wait_for(lambda: motor.keys[-1] == 1, (.18, .50))
        wait_for(lambda: motor.keys[-1] == 3, (0.0, -.50))
        wait_for(lambda: motor.keys[-1] == 4, (-.12, 0.0))
        wait_for(lambda: motor.keys[-1] == 5, (0.0, 0.0))

        wait_for(lambda: motor.keys[-1] == 2, (.18, 0.0))
        wait_for(lambda: motor.keys[-1] == 5)
        assert state['ready'], state

        if require_authorization:
            wait_for(lambda: motor.keys[-1] == 2, (.18, 0.0))
            permission = False
            wait_for(lambda: not state['motion_authorized'] and motor.keys[-1] == 5, (.18, 0.0))
            assert state['ready'], state

            permission = True
            wait_for(lambda: motor.keys[-1] == 2, (.18, 0.0))
            permission = None
            wait_for(lambda: not state['motion_authorized'] and motor.keys[-1] == 5, (.18, 0.0))
            assert state['ready'], state
            permission = True

        wait_for(lambda: motor.keys[-1] == 2, (.18, 0.0))
        motor.mode = None
        wait_for(lambda: not state['ready'] and motor.keys[-1] == 5, (.18, 0.0))
        count = len(motor.keys)
        motor.mode = 'a'
        pump_for(.4, (.18, 0.0))
        assert not state['ready'] and not MOTION_KEYS.intersection(motor.keys[count:]), (state, motor.keys)

        if require_authorization:
            print('PASS: no pre-authorization motion; revocation and timeout request stand', flush=True)
        print('PASS: ready handshake, four walk keys, zero/stale stop, feedback loss and no automatic resume',
              flush=True)
    finally:
        probe.destroy_subscription(subscription)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--authorization', action='store_true')
    args = parser.parse_args()
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) < 190 or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Set an isolated ROS_DOMAIN_ID (>=190) and ROS_LOCALHOST_ONLY=1')
    master, slave = pty.openpty()
    try:
        tty.setraw(slave)
        os.set_blocking(master, False)
        rclpy.init()
        probe = Node('parallel_horse_pty_probe')
        try:
            with tempfile.TemporaryDirectory(prefix='letsgo_parallel_horse_') as directory:
                parameters = Path(directory) / 'serial.yaml'
                parameters.write_text(
                    'motor_serial:\n  ros__parameters:\n'
                    f'    port: {os.ttyname(slave)}\n'
                    '    status_timeout: 0.4\n'
                    f'    nav_require_authorization: {str(args.authorization).lower()}\n',
                    encoding='utf-8',
                )
                prefix = Path(get_package_prefix('ros2_to_stm32'))
                executable = prefix / 'lib/ros2_to_stm32/parallel_horse_serial'
                process = subprocess.Popen(
                    [str(executable), '--ros-args', '--params-file', str(parameters)],
                    cwd=ROOT, start_new_session=True,
                )
                try:
                    exercise_bridge(probe, VirtualBoard(master), process, args.authorization)
                finally:
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait()
        finally:
            probe.destroy_node()
            rclpy.shutdown()
    finally:
        os.close(master)
        os.close(slave)


if __name__ == '__main__':
    main()
