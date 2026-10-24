import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import rclpy
import yaml
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry

from ros2_to_stm32.motor_protocol import (
    MODE_STATUS, StatusParser, crc8, encode_key, parse_durations, parse_request,
    validate_action,
)
from ros2_to_stm32.serial import MotorSerial
from ros2_to_stm32.task_to_do_unitree import Task_To_Do
from ros2_to_stm32.trans import TopicRenamer


def status_frame(mode='a', fault=0):
    frame = bytes([0xA5, 0x5A, ord(mode), fault, 0, 0])
    return frame + bytes([crc8(frame), 0x0D])


class FakePort:
    def __init__(self):
        self.received = bytearray()
        self.sent = []
        self.short_write = False

    @property
    def in_waiting(self):
        return len(self.received)

    def read(self, count):
        data = bytes(self.received[:count])
        del self.received[:count]
        return data

    def write(self, frame):
        self.sent.append(frame)
        return 1 if self.short_write else len(frame)

    def close(self):
        pass


@pytest.fixture
def bridge():
    rclpy.init()
    clock = SimpleNamespace(now=100.0)
    port = FakePort()
    with patch('ros2_to_stm32.serial.serial.Serial', return_value=port), patch(
            'ros2_to_stm32.serial.time.monotonic', side_effect=lambda: clock.now):
        node = MotorSerial()
        port.received.extend(status_frame())
        node.tick()
        node.pose(Float32MultiArray(data=[0.0, 0.0, 0.0]))
        node.calibration(Bool(data=True))
        node.controller(Bool(data=True))
        node.route(String(data='{"ready": true, "reason": "offline test"}'))
        yield node, port, clock
        node.destroy_node()
    rclpy.shutdown()


def arm(node):
    response = node.enable(SetBool.Request(data=True), SetBool.Response())
    assert response.success, response.message


def request(node, key, mode=9, request_id=1):
    node.request(Float32MultiArray(data=[float(key), 1.0, float(mode), float(request_id)]))


def test_crc_and_stream_parser():
    assert crc8(b'123456789') == 0xF4
    parser = StatusParser()
    frame = status_frame('b', 10)
    assert parser.feed(b'noise' + frame[:3]) == []
    assert parser.feed(frame[3:])[0]['fault'] == 10
    corrupted = bytearray(frame)
    corrupted[6] ^= 1
    assert parser.feed(corrupted + frame + frame) == [
        dict(mode='b', fault=10, motor=0, detail=0)] * 2


def test_raw_protocol_and_action_validation():
    assert encode_key(2) == b'\x53\x02\x45'
    assert encode_key(2) != b'S2E'
    for mode in MODE_STATUS:
        for key in range(1, 6):
            validate_action(mode, key)
    for mode, key in [(10, 6), (17, 7), (12, 19), (9, 21), (9, 22), (9, 23)]:
        with pytest.raises(ValueError):
            validate_action(mode, key)
    for data in ([2.0, 4.0], [2.0, 4.0, 9.0, 1.0], [2.5, 1.0, 9.0, 1.0],
                 [float('nan'), 1.0, 9.0, 1.0]):
        with pytest.raises(ValueError):
            parse_request(data)


def test_config_duration_table():
    config = yaml.safe_load((Path(__file__).parents[1] / 'config/control.yaml').read_text())
    durations = parse_durations(config['motor_serial']['ros__parameters']['action_durations'])
    assert durations[17, 6] == 5.36
    assert (13, 7) not in durations


def test_default_disarmed_and_no_shutdown_motion(bridge):
    node, port, clock = bridge
    request(node, 2)
    node.tick()
    assert not port.sent
    node.calibrated = False
    assert not node.enable(SetBool.Request(data=True), SetBool.Response()).success


def test_forward_once_and_duplicate_request(bridge):
    node, port, clock = bridge
    arm(node)
    request(node, 2)
    node.tick()
    request(node, 2)
    node.tick()
    assert port.sent == [encode_key(2)]
    assert node.last_tx['request_id'] == 1


def test_mode_feedback_before_jump_and_busy_guard(bridge):
    node, port, clock = bridge
    arm(node)
    request(node, 6, 17)
    node.tick()
    assert port.sent == [encode_key(5), encode_key(17)]
    clock.now += 0.1
    port.received.extend(status_frame('k'))
    node.tick()
    assert port.sent == [encode_key(5), encode_key(17), encode_key(6)]
    assert node.busy_until == pytest.approx(clock.now + 5.36 + 0.75)
    request(node, 6, 17, 2)
    node.tick()
    assert port.sent.count(encode_key(6)) == 1
    node.enable(SetBool.Request(data=False), SetBool.Response())
    node.tick()
    assert port.sent.count(encode_key(5)) == 1
    clock.now = node.busy_until + 0.1
    node.tick()
    assert port.sent[-1] == encode_key(5)
    assert not node.armed


@pytest.mark.parametrize('trigger', ['pose', 'controller', 'status', 'fault', 'z', 'calibration'])
def test_watchdogs_latch_pause(bridge, trigger):
    node, port, clock = bridge
    arm(node)
    request(node, 2)
    node.tick()
    if trigger in ('pose', 'controller', 'status'):
        setattr(node, trigger + '_time', clock.now - 2)
    elif trigger in ('fault', 'z'):
        port.received.extend(status_frame('z' if trigger == 'z' else 'a', 1 if trigger == 'fault' else 0))
    else:
        node.calibration_request(Float32MultiArray(data=[1.0]))
    node.tick()
    assert not node.armed
    assert node.pending is None
    assert port.sent[-1] == encode_key(5)
    port.received.extend(status_frame())
    node.tick()
    assert not node.armed


def test_short_write_disarms_without_replaying(bridge):
    node, port, clock = bridge
    arm(node)
    port.short_write = True
    request(node, 6)
    node.tick()
    assert not node.armed
    assert node.last_tx is None
    assert node.pending is None


def test_mode_confirmation_timeout(bridge):
    node, port, clock = bridge
    arm(node)
    request(node, 6, 17)
    node.tick()
    clock.now += 2.1
    port.received.extend(status_frame())
    node.pose_time = node.controller_time = clock.now
    node.tick()
    assert not node.armed
    assert encode_key(6) not in port.sent


@pytest.fixture
def task():
    rclpy.init()
    node = Task_To_Do()
    node.route_profile = 'demo'
    node.motor_state = dict(armed=True, busy=False)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_target_behind_is_not_reached(task):
    commands = []
    task.Kkey_To_Number = lambda key: commands.append(key) or True
    assert not task.navigate_to_pose(0, 0, 0, -1, 0, 0)
    assert commands[-1] in ('左转', '右转')
    assert task.navigate_to_pose(0.98, 0, 0, 1, 0, 0)
    assert commands[-1] == '站立'


def test_demo_advances_one_target_per_update(task):
    targets = []
    task.navigate_to_pose = lambda *args: targets.append(args[3:]) or True
    task.Quadruped_Obstacle(0, 0, 0)
    assert targets == [(1.0, 0.0, 0.0)]
    task.Quadruped_Obstacle(1, 0, 0)
    assert targets[-1] == (1.0, -1.0, 0.0)


def test_jump_waits_for_local_send_and_timer(task):
    task.begin_action(18, 6)
    task.Quadruped_Obstacle(0, 0, 0)
    assert task.command is None
    task.action_ready_at = 0.0
    task.Quadruped_Obstacle(0, 0, 0)
    command = dict(task.command)
    task.Quadruped_Obstacle(0, 0, 0)
    assert task.index == 0
    task.motor_state.update(last_tx=command, busy=True)
    task.Quadruped_Obstacle(0, 0, 0)
    assert task.jump_sent
    task.Quadruped_Obstacle(0, 0, 0)
    assert task.index == 0
    task.motor_state['busy'] = False
    task.Quadruped_Obstacle(0, 0, 0)
    assert task.index == 1
    assert task.pending_jump is None


def test_course_handlers_exhaust_safely(task):
    commands = []
    task.Kkey_To_Number = lambda key: commands.append(key) or True
    for name in ('handle_bridge_A', 'handle_bridge_B', 'handle_big_slope',
                 'handle_limit_pole', 'handle_bunker', 'handle_pole',
                 'handle_hill_wall', 'handle_step'):
        task.active_handler = name
        task.index = 999
        getattr(task, name)(0, 0, 0, 0, 0)
        assert commands[-1] == '站立'


def test_third_step_uses_normal_mode_six(task):
    task.active_handler = 'handle_step'
    task.index = 2
    task.navigate_to_pose = lambda *args: True
    task.handle_step(0.62, 0, 0, 0, 0)
    assert task.pending_jump == (9, 6)


def test_calibration_requires_two_valid_points():
    rclpy.init()
    node = TopicRenamer()
    positions = []
    ready = []
    node.publisher.publish = lambda message: positions.append(message.data)
    node.calibration_publisher.publish = lambda message: ready.append(message.data)
    odometry = Odometry()
    odometry.pose.pose.orientation.w = 1.0
    try:
        node.task_param_callback(Float32MultiArray(data=[1.0]))
        assert not node.first_captured
        node.Odometry_callback(odometry)
        assert not positions
        node.task_param_callback(Float32MultiArray(data=[2.0]))
        assert not node.second_captured
        node.task_param_callback(Float32MultiArray(data=[1.0]))
        node.task_param_callback(Float32MultiArray(data=[2.0]))
        assert not node.second_captured
        odometry.pose.pose.position.x = 1.0
        node.Odometry_callback(odometry)
        node.task_param_callback(Float32MultiArray(data=[2.0]))
        node.Odometry_callback(odometry)
        assert ready[-1]
        assert positions[-1] == pytest.approx([1.0, 0.0, 0.0])
        odometry.pose.pose.orientation.z = math.sin(math.pi / 4)
        odometry.pose.pose.orientation.w = math.cos(math.pi / 4)
        node.Odometry_callback(odometry)
        assert positions[-1][2] == pytest.approx(90.0)
        node.task_param_callback(Float32MultiArray(data=[1.0]))
        count = len(positions)
        node.Odometry_callback(odometry)
        assert not ready[-1]
        assert len(positions) == count
    finally:
        node.destroy_node()
        rclpy.shutdown()
