"""No hardware/serial: test the map-to-course adapter with injected TF."""
import math
import time
from types import SimpleNamespace
import unittest

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ros2_to_stm32.gicp_trans import GicpTrans


class Sink:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class BridgeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = GicpTrans()
        self.node.publisher = Sink()
        self.node.calibration_publisher = Sink()
        self.node.x1, self.node.y1 = 10.0, 20.0
        self.node.x2, self.node.y2 = 11.0, 20.0
        self.node.first_captured = self.node.second_captured = True
        self.tf = TransformStamped()
        self.tf.header.frame_id = 'map'
        self.tf.header.stamp = self.node.get_clock().now().to_msg()
        self.tf.transform.translation.x = 12.0
        self.tf.transform.translation.y = 21.0
        self.tf.transform.rotation.z = math.sin(math.pi / 4)
        self.tf.transform.rotation.w = math.cos(math.pi / 4)
        self.node.buffer = SimpleNamespace(lookup_transform=lambda *a, **k: self.tf)
        odom = Odometry()
        odom.header.stamp = self.tf.header.stamp
        self.node.on_odom(odom)
        self.node.on_valid(Bool(data=True))

    def tearDown(self):
        self.node.destroy_node()

    def test_map_to_course_and_yaw(self):
        self.node.update_map_pose()
        xyz = self.node.publisher.messages[-1].data
        for actual, expected in zip(xyz, [2.0, -1.0, 90.0]):
            self.assertAlmostEqual(actual, expected)

    def test_invalid(self):
        self.node.on_valid(Bool(data=False))
        self.node.update_map_pose()
        self.assertEqual(self.node.publisher.messages, [])
        self.assertFalse(self.node.calibration_publisher.messages[-1].data)

    def test_stale_valid(self):
        self.node.valid_at = time.monotonic() - 2
        self.node.update_map_pose()
        self.assertEqual(self.node.publisher.messages, [])

    def test_stale_odometry(self):
        self.node.odom_at = time.monotonic() - 1
        self.node.update_map_pose()
        self.assertEqual(self.node.publisher.messages, [])

    def test_stale_tf(self):
        self.tf.header.stamp.sec -= 2
        self.node.update_map_pose()
        self.assertEqual(self.node.publisher.messages, [])

    def test_no_calibration(self):
        self.node.first_captured = False
        self.node.update_map_pose()
        self.assertEqual(self.node.publisher.messages, [])


if __name__ == '__main__':
    unittest.main()
