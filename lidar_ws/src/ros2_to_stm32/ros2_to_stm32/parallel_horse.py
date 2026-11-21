"""Drive the parallel horse through the normal-walk keys of its USART3 firmware."""
import json
import math
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from .motor_protocol import MODE_STATUS, StatusParser, encode_key


NORMAL_MODE = 9
NORMAL_STATUS = MODE_STATUS[NORMAL_MODE]
LEFT, FORWARD, RIGHT, BACKWARD, STAND = range(1, 6)


def walk_key(command, previous, settings):
    vx, wz = command.linear.x, command.angular.z
    if not all(math.isfinite(value) for value in (vx, command.linear.y, wz)):
        return STAND
    near_zero_speed = abs(vx) <= settings['nav_linear_deadband']
    turning = previous in (LEFT, RIGHT)
    turn_threshold = settings['nav_turn_exit'] if turning else settings['nav_turn_enter']
    if abs(wz) > settings['nav_angular_deadband'] and (near_zero_speed or abs(wz) >= turn_threshold):
        return LEFT if wz > 0.0 else RIGHT
    if near_zero_speed:
        return STAND
    return FORWARD if vx > 0.0 else BACKWARD


class ParallelHorseSerial(Node):
    def __init__(self):
        super().__init__('motor_serial')
        defaults = {
            'port': '/dev/ttyUSB0', 'baudrate': 115200,
            'status_timeout': 1.0, 'mode_timeout': 2.0,
            'nav_command_timeout': 0.30, 'nav_send_period': 0.10,
            'nav_linear_deadband': 0.015, 'nav_angular_deadband': 0.05,
            'nav_turn_enter': 0.25, 'nav_turn_exit': 0.15, 'nav_min_switch_interval': 0.32,
        }
        self.settings = {name: self.declare_parameter(name, value).value for name, value in defaults.items()}
        periods = ('status_timeout', 'mode_timeout', 'nav_command_timeout',
                   'nav_send_period', 'nav_min_switch_interval')
        limits = ('nav_linear_deadband', 'nav_angular_deadband', 'nav_turn_enter', 'nav_turn_exit')
        if (not all(math.isfinite(self.settings[name]) and self.settings[name] > 0.0 for name in periods)
                or not all(math.isfinite(self.settings[name]) and self.settings[name] >= 0.0 for name in limits)
                or not self.settings['nav_angular_deadband'] <= self.settings['nav_turn_exit'] <= self.settings['nav_turn_enter']
                or self.settings['nav_send_period'] >= self.settings['nav_command_timeout']):
            raise ValueError('Invalid parallel horse timeouts or deadbands')

        self.port = None
        self.parser = StatusParser()
        self.next_connect = 0.0
        self.status = None
        self.status_at = -math.inf
        self.command = Twist()
        self.command_at = -math.inf
        self.mode_sent_at = None
        self.ready_at = None
        self.halted = None
        self.last_key = None
        self.last_tx_at = -math.inf
        self.key_changed_at = -math.inf

        self.state_pub = self.create_publisher(String, '/parallel_horse/status', 10)
        self.create_subscription(Twist, '/cmd_vel', self.receive_command, 10)
        self.create_timer(min(0.05, self.settings['nav_send_period']), self.tick)

    def receive_command(self, msg):
        self.command = msg
        self.command_at = time.monotonic()

    def receive_status(self, now):
        if self.port is None:
            if now < self.next_connect:
                return
            self.next_connect = now + 2.0
            self.port = serial.Serial(
                self.settings['port'], self.settings['baudrate'], bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0,
                write_timeout=0.2, exclusive=True,
            )
            self.parser = StatusParser()
        data = self.port.read(min(self.port.in_waiting, 4096))
        for status in self.parser.feed(data):
            self.status = status
            self.status_at = now

    def send_key(self, key, now):
        frame = encode_key(key)
        if self.port.write(frame) != len(frame):
            raise serial.SerialException('incomplete key frame')
        if key <= STAND:
            if self.last_key != key:
                self.key_changed_at = now
            self.last_key = key
            self.last_tx_at = now

    def control(self, now):
        if not self.status or now - self.status_at > self.settings['status_timeout']:
            reason = 'waiting_for_auto_feedback'
        elif self.status['mode'] == 'z':
            reason = 'waiting_for_stand_up'
        elif self.status['fault']:
            reason = 'board_fault'
        elif self.ready_at is not None and self.status['mode'] != NORMAL_STATUS:
            reason = 'normal_mode_lost'
        else:
            reason = None

        if reason:
            if self.ready_at is not None:
                self.halted = self.halted or reason
                if self.port is not None and self.last_key != STAND:
                    self.send_key(STAND, now)
            return self.halted or reason
        if self.halted:
            if self.last_key != STAND:
                self.send_key(STAND, now)
            return self.halted

        if self.ready_at is None:
            if self.mode_sent_at is None or now - self.mode_sent_at >= self.settings['mode_timeout']:
                self.send_key(STAND, now)
                self.send_key(NORMAL_MODE, now)
                self.mode_sent_at = now
            if self.status_at > self.mode_sent_at and self.status['mode'] == NORMAL_STATUS:
                self.ready_at = now
            else:
                return 'waiting_for_normal_mode'

        key = STAND
        if self.command_at >= self.ready_at and now - self.command_at <= self.settings['nav_command_timeout']:
            key = walk_key(self.command, self.last_key, self.settings)
        if (key != STAND and self.last_key not in (None, STAND)
                and now - self.key_changed_at < self.settings['nav_min_switch_interval']):
            key = self.last_key
        if key != self.last_key or now - self.last_tx_at >= self.settings['nav_send_period']:
            self.send_key(key, now)
        return 'ready'

    def tick(self):
        now = time.monotonic()
        try:
            self.receive_status(now)
            reason = self.control(now)
        except (serial.SerialException, OSError) as exc:
            message = f'serial_error: {exc}'
            if self.ready_at is not None:
                self.halted = self.halted or message
            reason = self.halted or message
            if self.port is not None:
                self.port.close()
                self.port = None
            self.status = None
            self.mode_sent_at = None
        state = {
            'ready': reason == 'ready', 'reason': reason, 'last_key': self.last_key,
            'mode': self.status['mode'] if self.status else None,
            'fault': self.status['fault'] if self.status else None,
        }
        self.state_pub.publish(String(data=json.dumps(state)))

    def destroy_node(self):
        if self.port is not None:
            try:
                if self.last_key in (LEFT, FORWARD, RIGHT, BACKWARD):
                    self.send_key(STAND, time.monotonic())
            except (serial.SerialException, OSError):
                pass
            finally:
                self.port.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ParallelHorseSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
