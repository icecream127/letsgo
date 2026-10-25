import json
import math
import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import SetBool

from .motor_protocol import MODE_STATUS, StatusParser, encode_key, parse_durations, parse_request


class MotorSerial(Node):
    def __init__(self):
        super().__init__('motor_serial')
        defaults = {'port': '/dev/ttyUSB0', 'baudrate': 115200,
                    'tx_only': False,
                    'status_timeout': 1.0, 'pose_timeout': 1.0,
                    'controller_timeout': 1.0, 'mode_timeout': 2.0,
                    'action_margin': 0.75,
                    'action_durations': ['9:6:1.50', '9:7:3.0', '18:6:1.42',
                                         '18:7:1.42', '11:6:2.78', '11:7:1.50',
                                         '13:6:1.42', '15:6:1.78', '15:7:1.50', '17:6:5.36']}
        self.settings = {name: self.declare_parameter(name, value).value for name, value in defaults.items()}
        for name in ('status_timeout', 'pose_timeout', 'controller_timeout', 'mode_timeout', 'action_margin'):
            if not math.isfinite(self.settings[name]) or self.settings[name] <= 0:
                raise ValueError(f'{name} 必须为有限正数')
        if self.settings['baudrate'] != 115200:
            raise ValueError('当前电控要求 115200 波特率')
        self.durations = parse_durations(self.settings['action_durations'])
        self.port = None
        self.parser = StatusParser()
        self.status = dict(mode='z', fault=0, motor=0, detail=0)
        self.status_time = self.pose_time = self.controller_time = float('-inf')
        self.calibrated = False
        self.route_ready = False
        self.route_time = float('-inf')
        self.route_reason = '等待路线审核状态'
        self.armed = False
        self.reason = '等待遥控器初始化、踩点并人工允许运行'
        self.pending = None
        self.last_tx = None
        self.last_request_id = 0
        self.busy_until = 0.0
        self.expected_mode = None
        self.stop_at = None
        self.next_connect = 0.0
        self.state_pub = self.create_publisher(String, '/motor_state', 10)
        self.create_subscription(Float32MultiArray, '/serial_send', self.request, 10)
        self.create_subscription(Float32MultiArray, '/pos_trans', self.pose, 1)
        self.create_subscription(Bool, '/calibration_ready', self.calibration, 1)
        self.create_subscription(Bool, '/controller_alive', self.controller, 1)
        self.create_subscription(String, '/route_state', self.route, 1)
        self.create_subscription(Float32MultiArray, '/task_params', self.calibration_request, 10)
        self.create_service(SetBool, '/automation/enable', self.enable)
        self.create_timer(0.05, self.tick)

    def pose(self, message):
        if len(message.data) == 3 and all(math.isfinite(value) for value in message.data):
            self.pose_time = time.monotonic()
        else:
            self.pause('定位数据无效')

    def calibration(self, message):
        self.calibrated = message.data
        if not self.calibrated and self.armed:
            self.pause('尚未完成有效的两次踩点')

    def calibration_request(self, message):
        if message.data and message.data[0] in range(1, 11):
            if message.data[0] in (1.0, 2.0):
                self.calibrated = False
            else:
                self.route_ready = False
                self.route_reason = '障碍参考点变更，需要重新审核地图'
            self.pause('重新踩点，需重新审核并允许运行')

    def route(self, message):
        try:
            state = json.loads(message.data)
            if not isinstance(state, dict) or type(state.get('ready')) is not bool:
                raise ValueError('路线状态格式错误')
            self.route_ready = state['ready']
            self.route_reason = state.get('reason', '')
            self.route_time = time.monotonic()
        except (ValueError, TypeError):
            self.route_ready = False
            self.route_reason = '路线状态无效'
        if not self.route_ready and self.armed:
            self.pause(self.route_reason or '路线尚未审核')

    def controller(self, message):
        if message.data:
            self.controller_time = time.monotonic()

    def unavailable(self, now):
        if self.port is None:
            return '串口未打开；断线不代表机器人已停止'
        if not self.settings['tx_only'] and now - self.status_time > self.settings['status_timeout']:
            return '串口或电控状态超时；断线不代表机器人已停止'
        if not self.settings['tx_only'] and self.status['mode'] == 'z':
            return '电控未起身、已趴下或重启'
        if not self.settings['tx_only'] and self.status['fault']:
            return f"电控故障 {self.status['fault']}，电机 {self.status['motor']}"
        if not self.calibrated or now - self.pose_time > self.settings['pose_timeout']:
            return '踩点未完成或定位超时'
        if now - self.controller_time > self.settings['controller_timeout']:
            return '控制器未运行或心跳超时'
        if not self.route_ready or now - self.route_time > self.settings['controller_timeout']:
            return self.route_reason or '路线状态未就绪或超时'
        return ''

    def enable(self, request, response):
        if not request.data:
            self.pause('人工暂停；站立请求不是急停')
            response.success = True
        else:
            reason = self.unavailable(time.monotonic())
            if self.stop_at is not None or time.monotonic() < self.busy_until:
                reason = '等待一次性动作保护时间及暂停站立请求结束'
            response.success = not reason
            if response.success:
                if not self.armed:
                    self.last_request_id = 0
                    self.expected_mode = None
                self.armed = True
                self.reason = '自动运行已允许；初始化完成由操作员确认'
                if self.settings['tx_only']:
                    self.reason = '单向发送已允许；电控状态未知，初始化及遥控器23授权由操作员确认'
            else:
                self.reason = reason
        response.message = self.reason
        return response

    def pause(self, reason):
        was_active = self.armed
        self.armed = False
        self.pending = None
        self.reason = reason
        if was_active:
            self.stop_at = max(time.monotonic(), self.busy_until)
            self.get_logger().error(reason + '；必要时使用独立急停')

    def write_key(self, key):
        if self.port is None:
            raise serial.SerialException('串口未连接')
        frame = encode_key(key)
        if self.port.write(frame) != len(frame):
            raise serial.SerialException('串口短写，动作结果未知')

    def request(self, message):
        try:
            command = parse_request(message.data)
            if not self.armed:
                return
            if command['request_id'] <= self.last_request_id:
                return
            if self.pending is not None or time.monotonic() < self.busy_until:
                return
            self.last_request_id = command['request_id']
            self.pending = command
            self.pending['mode_sent_at'] = None
        except ValueError as error:
            self.pause(str(error))

    def tick(self):
        now = time.monotonic()
        try:
            if self.port is None and now >= self.next_connect:
                self.next_connect = now + 2.0
                self.port = serial.Serial(self.settings['port'], self.settings['baudrate'],
                                          timeout=0, write_timeout=0.2, exclusive=True)
                self.parser = StatusParser()
                self.status_time = float('-inf')
                self.reason = ('单向模式：本地串口已打开，电控状态未知；等待人工允许运行'
                               if self.settings['tx_only'] else '串口已连接；等待反馈并人工允许运行')
            if self.port is not None and not self.settings['tx_only']:
                data = self.port.read(min(self.port.in_waiting, 4096))
                for status in self.parser.feed(data):
                    self.status = status
                    self.status_time = now
                    if self.armed and (status['fault'] or status['mode'] == 'z'):
                        self.pause('电控故障或未起身状态')
            if self.armed:
                reason = self.unavailable(now)
                if reason:
                    self.pause(reason)
                elif not self.settings['tx_only'] and self.pending is None and self.expected_mode is not None and self.status['mode'] != self.expected_mode:
                    self.pause('电控模式被外部改变，暂停自动任务')
            if self.stop_at is not None and now >= self.stop_at and self.port is not None:
                self.write_key(5)
                self.stop_at = None
            if self.armed and self.pending is not None:
                command = self.pending
                expected = MODE_STATUS[command['mode']]
                sent_at = command['mode_sent_at']
                if self.settings['tx_only']:
                    if self.expected_mode != expected:
                        self.write_key(5)
                        self.write_key(command['mode'])
                    duration = self.durations.get((command['mode'], command['key']))
                    self.busy_until = now + duration + self.settings['action_margin'] if duration else now
                    self.write_key(command['key'])
                    self.expected_mode = expected
                    self.last_tx = {name: command[name] for name in ('key', 'mode', 'request_id')}
                    self.pending = None
                elif self.status['mode'] != expected or (sent_at is not None and self.status_time <= sent_at):
                    if sent_at is None:
                        self.write_key(5)
                        self.write_key(command['mode'])
                        command['mode_sent_at'] = now
                    elif now - sent_at > self.settings['mode_timeout']:
                        self.pause('模式确认超时，未发送后续动作')
                else:
                    duration = self.durations.get((command['mode'], command['key']))
                    self.busy_until = now + duration + self.settings['action_margin'] if duration else now
                    self.write_key(command['key'])
                    self.expected_mode = expected
                    self.last_tx = {name: command[name] for name in ('key', 'mode', 'request_id')}
                    self.pending = None
        except (serial.SerialException, OSError) as error:
            self.pause(f'串口异常：{error}')
            if self.port is not None:
                self.port.close()
            self.port = None
            self.status_time = float('-inf')
        state = dict(self.status, armed=self.armed, reason=self.reason,
                     connected=None if self.settings['tx_only'] else self.port is not None and now - self.status_time <= self.settings['status_timeout'],
                     serial_open=self.port is not None, tx_only=self.settings['tx_only'],
                      busy=time.monotonic() < self.busy_until, last_tx=self.last_tx,
                      busy_source='host_timer', action_completion_feedback=False,
                      route_ready=self.route_ready)
        if self.settings['tx_only']:
            state.update(mode=None, fault=None, motor=None, detail=None)
        self.state_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))

    def destroy_node(self):
        should_stop = self.armed or self.stop_at is not None
        self.armed = False
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
