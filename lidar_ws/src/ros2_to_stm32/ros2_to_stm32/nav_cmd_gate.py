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
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_pose,10)
        self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.on_initialpose,10)
        self.create_timer(.05,self.tick)

    def fresh_stamp(self,stamp):
        return -.1 <= (self.get_clock().now()-Time.from_msg(stamp)).nanoseconds*1e-9 <= .5

    def on_initialpose(self,msg):
        self.localized=False
        self.times['cmd']=-math.inf

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
                math.isfinite(v) and msg.range_min<=v<=msg.range_max for v in msg.ranges)>=10:
            self.times['scan']=time.monotonic()

    def tick(self):
        now=time.monotonic()
        reason='ready'
        if not self.localized:reason='waiting_for_initial_pose_and_amcl'
        for key in ('odom','scan','cmd'):
            if now-self.times[key]>.5:reason=f'stale_{key}'
        try:
            tf=self.buffer.lookup_transform('map','odom_nav',Time())
            age=(self.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds*1e-9
            if age>1.5 or age< -2.:reason='stale_localization_tf'
        except TransformException:reason='waiting_for_localization_tf'
        out=Twist()
        if reason=='ready':
            out.linear.x=max(-.15,min(.25,self.command.linear.x))
            out.angular.z=max(-.6,min(.6,self.command.angular.z))
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
