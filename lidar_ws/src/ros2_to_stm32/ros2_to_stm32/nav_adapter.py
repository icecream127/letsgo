"""FAST-LIO to planar Nav2 odometry, retaining the original raw TF branch."""
from collections import deque
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener, TransformException
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
        self.use_gicp = self.declare_parameter('use_gicp', False).value
        if self.use_gicp:
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
        self.scan_rotations = {}
        self.scan_clouds = {}
        self.scan_cloud_pub = self.create_publisher(PointCloud2, '/cloud_registered_nav', 10)
        self.create_subscription(PointCloud2, '/cloud_registered_body', self.on_cloud, qos_profile_sensor_data)
        self.static_sent = False
        self.history = deque(maxlen=100)
        self.last_stamp = None
        self.reset_required = False
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, qos_profile_sensor_data)
        self.get_logger().info('Navigation adapter started')

    def on_cloud(self, msg):
        if msg.header.frame_id != 'livox_frame':
            return
        self.scan_clouds[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg
        self.publish_scan_cloud()

    def publish_scan_cloud(self):
        # Pair exact FAST-LIO measurement times. Rotate the body cloud into the
        # yaw-only nav_scan frame directly, without requiring a global map TF.
        for key in list(self.scan_clouds):
            if key not in self.scan_rotations:
                continue
            msg = self.scan_clouds.pop(key)
            rotation = self.scan_rotations.pop(key)
            points = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            xyz = np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float32).reshape(-1, 3)
            x, y, z, w = rotation
            matrix = np.array([
                [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
            msg.header.frame_id = 'nav_scan'
            self.scan_cloud_pub.publish(point_cloud2.create_cloud_xyz32(msg.header, xyz @ matrix.T))
        for cache in (self.scan_clouds, self.scan_rotations):
            while len(cache) > 20:
                del cache[next(iter(cache))]

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
        if not self.use_gicp and not self.static_sent:
            # Identity link preserves the FAST-LIO coordinate system without correction.
            self.static.sendTransform(self.transform('odom_nav','odom',[0.,0.,0.],[0.,0.,0.,1.],msg.header.stamp))
            self.static_sent = True
        raw_p = position
        raw_q = rotation
        base_p = raw_p + rotate(raw_q,self.offset)
        heading = yaw(multiply(raw_q,yaw_q(self.yaw_offset)))
        self.scan_rotations[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = multiply(yaw_q(-heading), raw_q)
        self.publish_scan_cloud()
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
        transforms = [
            self.transform('odom_nav','base_footprint',[*base_p[:2],0.],yaw_q(heading),msg.header.stamp),
            self.transform('base_footprint','nav_scan',
                           rotate(yaw_q(-heading),raw_p-np.array([*base_p[:2],0.])),
                           yaw_q(0.),msg.header.stamp)]
        if self.use_gicp:
            correction = self.global_transform(base_p, raw_q, heading, msg.header.stamp)
            if correction is not None:
                transforms.append(correction)
        self.tf.sendTransform(transforms)

    def global_transform(self, base_p, raw_q, heading, stamp):
        # GICP streams the latest accepted map correction. Apply it to this raw
        # odometry sample, then project the corrected base pose onto the plane.
        try:
            correction = self.buffer.lookup_transform('map', 'odom', Time())
        except TransformException:
            return None
        if not 0. <= self.get_clock().now().nanoseconds * 1e-9 - stamp_seconds(correction.header.stamp) <= .5:
            return None
        t, q = correction.transform.translation, correction.transform.rotation
        map_q = np.array([q.x, q.y, q.z, q.w])
        map_p = np.array([t.x, t.y, t.z]) + rotate(map_q, base_p)
        map_heading = yaw(multiply(multiply(map_q, raw_q), yaw_q(self.yaw_offset)))
        delta = yaw_q(map_heading - heading)
        translation = map_p[:2] - rotate(delta, [*base_p[:2], 0.])[:2]
        return self.transform('map', 'odom_nav', [*translation, 0.], delta, stamp)


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
