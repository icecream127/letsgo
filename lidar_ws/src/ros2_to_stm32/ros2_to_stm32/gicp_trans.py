"""Map-corrected position input for the legacy A/B course controller.

Do not run alongside pos_trans. Body-center correction remains in Task_To_Do.
"""
import time

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException

from .trans import TopicRenamer


class GicpTrans(TopicRenamer):
    def __init__(self):
        super().__init__()
        self.destroy_subscription(self.subscription)
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.body_frame = self.declare_parameter('body_frame', 'livox_frame').value
        self.max_age = float(self.declare_parameter('max_pose_age', 0.5).value)
        self.valid_timeout = float(self.declare_parameter('valid_timeout', 1.5).value)
        if self.max_age <= 0 or self.valid_timeout <= 0:
            raise ValueError('Timeouts must be positive')
        self.valid = False
        self.valid_at = float('-inf')
        self.odom_at = float('-inf')
        self.odom_stamp = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(Bool, '/gicp/valid', self.on_valid, 1)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, 1)
        self.create_timer(0.05, self.update_map_pose)

    def on_valid(self, msg):
        self.valid = msg.data
        self.valid_at = time.monotonic()
        if not self.valid:
            self.block()

    def on_odom(self, msg):
        self.odom_at = time.monotonic()
        self.odom_stamp = Time.from_msg(msg.header.stamp)

    def block(self):
        # Reject A/B capture too. No /pos_trans means the serial watchdog stops.
        self.last_odometry_time = float('-inf')
        self.calibration_publisher.publish(Bool(data=False))

    def fresh(self, stamp):
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return -0.05 <= age <= self.max_age

    def update_map_pose(self):
        now = time.monotonic()
        if (not self.valid or now - self.valid_at > self.valid_timeout
                or now - self.odom_at > self.max_age or self.odom_stamp is None
                or not self.fresh(self.odom_stamp)):
            self.block()
            return
        try:
            # Latest common TF time: high-rate local odometry with map correction.
            tf = self.buffer.lookup_transform(self.map_frame, self.body_frame, Time(),
                                              timeout=Duration(seconds=0.0))
        except TransformException:
            self.block()
            return
        if not self.fresh(Time.from_msg(tf.header.stamp)):
            self.block()
            return
        msg = Odometry()
        msg.header = tf.header
        msg.child_frame_id = self.body_frame
        msg.pose.pose.position.x = tf.transform.translation.x
        msg.pose.pose.position.y = tf.transform.translation.y
        msg.pose.pose.position.z = tf.transform.translation.z
        msg.pose.pose.orientation = tf.transform.rotation
        # Preserve legacy reflected course-Y and yaw convention exactly once.
        super().Odometry_callback(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GicpTrans()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.block()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
