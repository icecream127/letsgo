"""FAST-LIO to planar Nav2 odometry, retaining the original raw TF branch."""
from collections import deque
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from .nav_geometry import multiply, rotate, yaw, yaw_q, planar_velocity


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class NavAdapter(Node):
    def __init__(self):
        super().__init__('nav_adapter')
        self.offset = np.array(self.declare_parameter('livox_to_base_xyz', [0., 0., 0.]).value)
        self.yaw_offset = float(self.declare_parameter('livox_to_base_yaw', 0.).value)
        if self.offset.shape != (3,) or not np.isfinite(self.offset).all() or not math.isfinite(self.yaw_offset):
            raise ValueError('Invalid mounting parameters')
        self.static_sent = False
        self.history = deque(maxlen=100)
        self.last_stamp = None
        self.reset_required = False
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, qos_profile_sensor_data)
        self.get_logger().info('Navigation adapter started')

    def transform(self, parent, child, position, rotation, stamp):
        result = TransformStamped()
        result.header.frame_id, result.child_frame_id = parent, child
        result.header.stamp = stamp
        result.transform.translation.x, result.transform.translation.y, result.transform.translation.z = map(float, position)
        result.transform.rotation.x, result.transform.rotation.y, result.transform.rotation.z, result.transform.rotation.w = map(float, rotation)
        return result

    def on_odom(self, msg):
        if self.reset_required:
            return
        if (msg.header.frame_id, msg.child_frame_id) != ('odom', 'livox_frame'):
            self.get_logger().error('Expected FAST-LIO frames odom -> livox_frame', throttle_duration_sec=5.)
            return
        t = stamp_seconds(msg.header.stamp)
        now = self.get_clock().now().nanoseconds * 1e-9
        if not -.1 <= now-t <= .5:
            return
        if self.last_stamp is not None and t <= self.last_stamp:
            if t < self.last_stamp-.1:
                self.reset_required = True
                self.get_logger().error('Clock/odometry reset: restart navigation and reinitialize localization')
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        position, rotation = np.array([p.x,p.y,p.z]), np.array([q.x,q.y,q.z,q.w])
        norm = np.linalg.norm(rotation)
        if not np.isfinite(position).all() or not np.isfinite(rotation).all() or norm < 1e-9:
            return
        rotation /= norm
        if not self.static_sent:
            # Identity link preserves the FAST-LIO coordinate system without correction.
            self.static.sendTransform(self.transform('odom_nav','odom',[0.,0.,0.],[0.,0.,0.,1.],msg.header.stamp))
            self.static_sent = True
        raw_p = position
        raw_q = rotation
        base_p = raw_p + rotate(raw_q,self.offset)
        heading = yaw(multiply(raw_q,yaw_q(self.yaw_offset)))
        if self.history:
            dt = t-self.history[-1][0]
            displacement = np.linalg.norm(base_p[:2]-np.array(self.history[-1][1:3]))
            if displacement > max(.5,3.*dt) and dt < 1.:
                self.reset_required = True
                self.get_logger().error('Odometry discontinuity: restart navigation and reinitialize localization')
                return
            if dt>.5:
                self.history.clear()
        self.last_stamp=t
        self.history.append((t,float(base_p[0]),float(base_p[1]),heading))
        while len(self.history)>3 and t-self.history[1][0]>.2:
            self.history.popleft()
        result=Odometry()
        result.header.stamp=msg.header.stamp
        result.header.frame_id='odom_nav'
        result.child_frame_id='base_footprint'
        result.pose.pose.position.x,result.pose.pose.position.y=map(float,base_p[:2])
        result.pose.pose.orientation.z=math.sin(heading/2)
        result.pose.pose.orientation.w=math.cos(heading/2)
        vx,vy,wz=planar_velocity(self.history)
        result.twist.twist.linear.x,result.twist.twist.linear.y,result.twist.twist.angular.z=map(float,(vx,vy,wz))
        for i in (0,7,14,21,28,35):
            result.pose.covariance[i]=.1
            result.twist.covariance[i]=.2
        self.pub.publish(result)
        self.tf.sendTransform([
            self.transform('odom_nav','base_footprint',[*base_p[:2],0.],yaw_q(heading),msg.header.stamp),
            self.transform('base_footprint','nav_scan',
                           rotate(yaw_q(-heading),raw_p-np.array([*base_p[:2],0.])),
                           yaw_q(0.),msg.header.stamp)])


def main(args=None):
    rclpy.init(args=args)
    node=NavAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
