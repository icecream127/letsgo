"""Output-only command watchdog. No hardware protocol or serial access."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class CommandGate(Node):
    def __init__(self):
        super().__init__('nav_cmd_gate')
        self.input_timeouts = {
            key: float(self.declare_parameter(f'{key}_timeout', 0.5).value)
            for key in ('cmd', 'odom', 'scan')
        }
        self.speed_limits = (
            float(self.declare_parameter('max_forward_speed', 0.25).value),
            float(self.declare_parameter('max_reverse_speed', 0.15).value),
            float(self.declare_parameter('max_angular_speed', 0.6).value),
        )
        self.sensor_stamp_max_age = float(self.declare_parameter('sensor_stamp_max_age', 0.5).value)
        self.sensor_future_tolerance = float(self.declare_parameter('sensor_future_tolerance', 0.1).value)
        self.tf_max_age = float(self.declare_parameter('tf_max_age', 1.5).value)
        self.tf_future_tolerance = float(self.declare_parameter('tf_future_tolerance', 2.0).value)
        self.min_valid_scan_ranges = int(self.declare_parameter('min_valid_scan_ranges', 10).value)
        output_period = float(self.declare_parameter('output_period', 0.05).value)
        positive = (*self.input_timeouts.values(), self.sensor_stamp_max_age, self.tf_max_age, output_period)
        nonnegative = (*self.speed_limits, self.sensor_future_tolerance, self.tf_future_tolerance)
        if (not all(math.isfinite(v) and v > 0 for v in positive)
                or not all(math.isfinite(v) and v >= 0 for v in nonnegative)
                or self.min_valid_scan_ranges < 1):
            raise ValueError('Invalid nav_cmd_gate speed limits or watchdog parameters')
        self.times = dict(cmd=-math.inf,odom=-math.inf,scan=-math.inf)
        self.localized=False
        self.command=Twist()
        self.buffer=Buffer()
        self.listener=TransformListener(self.buffer,self)
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.status=self.create_publisher(String,'/navigation/status',10)
        self.create_subscription(Twist,'/cmd_vel_nav',self.on_cmd,10)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.use_gicp = self.declare_parameter('use_gicp', False).value
        if self.use_gicp:
            from go2_obstacle_interfaces.msg import LocalizationEvidence
            self.evidence = None
            self.create_subscription(LocalizationEvidence, '/gicp/evidence', self.on_evidence, 10)
            self.create_subscription(PoseWithCovarianceStamped, '/gicp/initialpose', self.on_initialpose, 10)
        else:
            self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_pose,10)
            self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.on_initialpose,10)
        self.create_timer(output_period, self.tick)

    def fresh_stamp(self,stamp):
        age = (self.get_clock().now() - Time.from_msg(stamp, clock_type=self.get_clock().clock_type)).nanoseconds * 1e-9
        return -self.sensor_future_tolerance <= age <= self.sensor_stamp_max_age

    def on_initialpose(self,msg):
        self.localized=False
        self.times['cmd']=-math.inf

    def on_evidence(self, msg):
        previous = self.evidence
        if previous is not None:
            if msg.source_id != previous.source_id:
                self.localized = False
                self.times['cmd'] = -math.inf
                return
            if msg.sequence <= previous.sequence or msg.epoch < previous.epoch:
                return
            if msg.epoch != previous.epoch:
                self.times['cmd'] = -math.inf
        self.evidence = msg
        self.localized = bool(msg.accepted and msg.source_id and msg.header.frame_id == 'map'
                              and msg.body_frame == 'livox_frame')
        if not self.localized:
            self.times['cmd'] = -math.inf

    def on_pose(self,msg):
        p=msg.pose.pose
        if msg.header.frame_id=='map' and all(math.isfinite(v) for v in
                (p.position.x,p.position.y,p.orientation.z,p.orientation.w)):
            self.localized=True

    def on_cmd(self,msg):
        if all(math.isfinite(v) for v in (msg.linear.x,msg.linear.y,msg.angular.z)):
            self.command=msg
            self.times['cmd']=time.monotonic()
        else:self.times['cmd']=-math.inf

    def on_odom(self,msg):
        if self.fresh_stamp(msg.header.stamp):self.times['odom']=time.monotonic()

    def on_scan(self,msg):
        if self.fresh_stamp(msg.header.stamp) and sum(
                math.isfinite(v) and msg.range_min<=v<=msg.range_max for v in msg.ranges)>=self.min_valid_scan_ranges:
            self.times['scan']=time.monotonic()

    def tick(self):
        now=time.monotonic()
        reason='ready'
        if not self.localized:reason='waiting_for_initial_pose_and_amcl'
        for key in ('odom','scan','cmd'):
            if now-self.times[key]>self.input_timeouts[key]:reason=f'stale_{key}'
        try:
            tf=self.buffer.lookup_transform('map','odom_nav',Time())
            age=(self.get_clock().now()-Time.from_msg(tf.header.stamp, clock_type=self.get_clock().clock_type)).nanoseconds*1e-9
            if age>self.tf_max_age or age< -self.tf_future_tolerance:reason='stale_localization_tf'
        except TransformException:reason='waiting_for_localization_tf'
        if self.use_gicp and not self.localized:
            reason = 'waiting_for_fresh_gicp_evidence'
        out=Twist()
        if reason=='ready':
            forward, reverse, angular = self.speed_limits
            out.linear.x = max(-reverse, min(forward, self.command.linear.x))
            out.angular.z = max(-angular, min(angular, self.command.angular.z))
        self.pub.publish(out)
        self.status.publish(String(data=reason))


def main(args=None):
    rclpy.init(args=args)
    node=CommandGate()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok(): node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
