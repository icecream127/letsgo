import json
import math
import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import SetBool

from .motor_protocol import MODE_STATUS, TIMED_ACTIONS, StatusParser, encode_key, parse_durations, parse_request


class MotorSerial(Node):
    """把控制器的请求转成和遥控器一样的按键帧，经 USB-TTL 接板端串口6。

    要求电控保留状态协议：KEY_mode 拨到自动后，在连接的串口上回传状态帧，
    这里收到状态就开始执行；拨回手动（或遥控器按 5）后马停发状态，
    这里 1 秒内收不到就停下，开关再拨到自动就自动继续。
    以下任一条件不满足时只“暂时停住”，条件恢复后自动继续，不需要重新允许运行：
      收到马的状态、马已起身、定位不超时、控制器在运行、没有人工暂停（pause.sh）。
    """

    def __init__(self):
        super().__init__('motor_serial')
        defaults = {'port': '/dev/ttyUSB0', 'baudrate': 115200,
                    'status_timeout': 1.0, 'pose_timeout': 1.0,
                    'controller_timeout': 1.0, 'mode_timeout': 2.0,
                    'action_margin': 0.75,
                    'action_durations': ['9:6:1.50', '9:7:3.0', '18:6:1.42',
                                         '18:7:1.42', '11:6:2.78', '11:7:1.50',
                                         '13:6:1.42', '15:6:1.78', '15:7:1.50', '17:6:5.36']}
        self.settings = {name: self.declare_parameter(name, value).value for name, value in defaults.items()}
        self.durations = parse_durations(self.settings['action_durations'])
        missing = sorted(TIMED_ACTIONS - set(self.durations))
        if missing:
            self.get_logger().warn(f'这些一次性动作没有配置时长，发送后不等待：{missing}')
        self.port = None
        self.parser = StatusParser()
        self.status = dict(mode='z', fault=0, motor=0, detail=0)
        self.status_time = self.pose_time = self.controller_time = float('-inf')
        self.enabled = True          # 人工开关：pause.sh 关，arm.sh 开；默认开
        self.active = False          # 当前是否满足控制条件
        self.reason = '等待马回传状态：遥控器 KEY_mode 拨到自动'
        self.pending = None
        self.last_tx = None
        self.last_request_id = None
        self.busy_until = 0.0
        self.stop_at = None
        self.next_connect = 0.0
        self.state_pub = self.create_publisher(String, '/motor_state', 10)
        self.create_subscription(Float32MultiArray, '/serial_send', self.request, 10)
        self.create_subscription(Float32MultiArray, '/pos_trans', self.pose, 1)
        self.create_subscription(Bool, '/controller_alive', self.controller, 1)
        self.create_service(SetBool, '/automation/enable', self.enable)
        self.create_timer(0.05, self.tick)

    def pose(self, message):
        if len(message.data) == 3 and all(math.isfinite(value) for value in message.data):
            self.pose_time = time.monotonic()

    def controller(self, message):
        if message.data:
            self.controller_time = time.monotonic()

    def not_ready(self, now):
        """返回暂时不能控制的原因；可以控制时返回空字符串。"""
        if not self.enabled:
            return '人工暂停（bash run/arm.sh 恢复）'
        if self.port is None or now - self.status_time > self.settings['status_timeout']:
            return '收不到马的状态：KEY_mode 没拨到自动，或串口断开'
        if self.status['mode'] == 'z':
            return '马还没初始化起身'
        if now - self.pose_time > self.settings['pose_timeout']:
            return '定位超时：检查踩点和雷达'
        if now - self.controller_time > self.settings['controller_timeout']:
            return '控制器未运行'
        return ''

    def enable(self, request, response):
        self.enabled = bool(request.data)
        response.success = True
        response.message = '已恢复自动' if self.enabled else '已人工暂停并请求站立（不是急停）'
        return response

    def write_key(self, key, repeat=1):
        if self.port is None:
            raise serial.SerialException('串口未连接')
        frame = encode_key(key) * repeat
        if self.port.write(frame) != len(frame):
            raise serial.SerialException('串口短写，动作结果未知')

    def request(self, message):
        command = parse_request(message.data)
        if command is None:
            self.get_logger().warn(f'忽略无效请求：{list(message.data)}')
            return
        if not self.active or command['request_id'] == self.last_request_id:
            return
        if self.pending is not None or time.monotonic() < self.busy_until:
            return
        self.last_request_id = command['request_id']
        command['mode_sent_at'] = None
        self.pending = command

    def tick(self):
        now = time.monotonic()
        try:
            if self.port is None and now >= self.next_connect:
                self.next_connect = now + 2.0
                self.port = serial.Serial(self.settings['port'], self.settings['baudrate'],
                                          bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                          stopbits=serial.STOPBITS_ONE,
                                          xonxoff=False, rtscts=False, dsrdtr=False,
                                          timeout=0, write_timeout=0.2, exclusive=True)
                self.parser = StatusParser()
                self.status_time = float('-inf')
            if self.port is not None:
                data = self.port.read(min(self.port.in_waiting, 4096))
                for status in self.parser.feed(data):
                    self.status = status
                    self.status_time = now
            reason = self.not_ready(now)
            if reason:
                if self.active:
                    # 刚失去控制条件：丢掉还没发的命令，请马站住（跳跃中就等跳完再发）
                    self.active = False
                    self.pending = None
                    self.stop_at = max(now, self.busy_until)
                    self.get_logger().warn(reason + '，请求站立')
                self.reason = reason
            elif not self.active:
                self.active = True
                self.stop_at = None
                self.reason = '自动运行中'
            if self.stop_at is not None and now >= self.stop_at and self.port is not None:
                self.write_key(5)
                self.stop_at = None
            if self.active and self.pending is not None:
                command = self.pending
                expected = MODE_STATUS[command['mode']]
                sent_at = command['mode_sent_at']
                if self.status['mode'] != expected or (sent_at is not None and self.status_time <= sent_at):
                    # 先切模式，等回传确认模式变了再发动作；超时就重发模式键
                    if sent_at is None or now - sent_at > self.settings['mode_timeout']:
                        repeat = 1 if (command['mode'], command['key']) in TIMED_ACTIONS else 2
                        self.write_key(command['mode'], repeat)
                        command['mode_sent_at'] = now
                else:
                    repeat = 1 if (command['mode'], command['key']) in TIMED_ACTIONS else 4
                    self.write_key(command['key'], repeat)
                    duration = self.durations.get((command['mode'], command['key']))
                    self.busy_until = now + duration + self.settings['action_margin'] if duration else now
                    self.last_tx = {name: command[name] for name in ('key', 'mode', 'request_id')}
                    self.pending = None
        except (serial.SerialException, OSError) as error:
            self.get_logger().error(f'串口异常：{error}，2 秒后重连')
            self.pending = None
            self.active = False
            self.reason = f'串口异常：{error}'
            if self.port is not None:
                self.port.close()
            self.port = None
            self.status_time = float('-inf')
        state = dict(self.status, armed=self.active, reason=self.reason,
                     connected=self.port is not None and now - self.status_time <= self.settings['status_timeout'],
                     busy=time.monotonic() < self.busy_until, last_tx=self.last_tx,
                     pending_id=self.pending['request_id'] if self.pending else None)
        self.state_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))

    def destroy_node(self):
        should_stop = self.active or self.stop_at is not None
        self.active = False
        if self.port is not None:
            try:
                if should_stop and time.monotonic() >= self.busy_until:
                    self.write_key(5)
            except (serial.SerialException, OSError):
                pass
            self.port.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
