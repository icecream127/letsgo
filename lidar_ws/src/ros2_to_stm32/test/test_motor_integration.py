import os
import pty
import time
import tty

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import SetBool

from ros2_to_stm32.info_transmiting import Info_Trans
from ros2_to_stm32.motor_protocol import crc8
from ros2_to_stm32.serial import MotorSerial


def test_ros_graph_through_virtual_ttl():
    master, slave = pty.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    rclpy.init()
    bridge = MotorSerial()
    bridge.settings['port'] = os.ttyname(slave)
    controller = Info_Trans()
    controller.task_to_do.route_profile = 'demo'
    probe = Node('offline_probe')
    poses = probe.create_publisher(Float32MultiArray, '/pos_trans', 1)
    calibration = probe.create_publisher(Bool, '/calibration_ready', 1)
    client = probe.create_client(SetBool, '/automation/enable')
    executor = SingleThreadedExecutor()
    for node in (bridge, controller, controller.task_to_do, probe):
        executor.add_node(node)
    frame = bytes([0xA5, 0x5A, ord('a'), 0, 0, 0])
    frame += bytes([crc8(frame), 0x0D])
    received = bytearray()
    commands = []
    position = [0.0, 0.0, 0.0]

    def pump_until(predicate, timeout=4.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            os.write(master, frame)
            poses.publish(Float32MultiArray(data=position))
            calibration.publish(Bool(data=True))
            executor.spin_once(timeout_sec=0.01)
            try:
                received.extend(os.read(master, 4096))
            except BlockingIOError:
                pass
            while len(received) >= 3:
                assert received[0] == 0x53 and received[2] == 0x45
                commands.append(received[1])
                del received[:3]
            if predicate():
                return
            time.sleep(0.005)
        raise AssertionError(f'离线链路超时：{bridge.reason}; commands={commands}')

    try:
        pump_until(lambda: not bridge.unavailable(time.monotonic()) and client.service_is_ready())
        assert not commands
        future = client.call_async(SetBool.Request(data=True))
        pump_until(future.done)
        assert future.result().success
        pump_until(lambda: 2 in commands)
        position[:] = [1.0, 0.0, 0.0]
        pump_until(lambda: controller.task_to_do.demo_index == 1)
        position[:] = [1.0, -1.0, 0.0]
        pump_until(lambda: controller.task_to_do.demo_index == 2)
        assert 5 in commands
        future = client.call_async(SetBool.Request(data=False))
        pump_until(future.done)
        pump_until(lambda: not controller.task_to_do.motor_state.get('armed', True))
        assert not bridge.armed
        assert not any(key in (6, 7, 21, 22, 23) for key in commands)
    finally:
        executor.shutdown()
        bridge.destroy_node()
        controller.task_to_do.destroy_node()
        controller.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)
