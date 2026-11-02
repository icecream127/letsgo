#!/usr/bin/env python3
"""Read-only navigation evidence; never publishes poses or velocity commands."""
import argparse
import json
import math
import time
from collections import Counter
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

p = argparse.ArgumentParser()
p.add_argument('--seconds', type=float, default=1800)
a = p.parse_args()
rclpy.init()
n = rclpy.create_node('navigation_evidence_observer')
counts = Counter()
latest = {}
subs = []
def cb(msg, topic):
    counts[topic] += 1
    now = n.get_clock().now().nanoseconds * 1e-9
    if hasattr(msg, 'header'):
        latest[topic + '_age_s'] = now - msg.header.stamp.sec - msg.header.stamp.nanosec * 1e-9
    if isinstance(msg, PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        v = msg.pose.pose.position
        pose = dict(topic=topic, frame=msg.header.frame_id, x=v.x, y=v.y,
                    yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                    stamp_sec=msg.header.stamp.sec, stamp_nanosec=msg.header.stamp.nanosec,
                    covariance=list(msg.pose.covariance))
        latest[topic] = pose
        print(json.dumps(dict(event='pose', **pose)), flush=True)
    elif isinstance(msg, Twist):
        latest[topic] = [msg.linear.x, msg.linear.y, msg.angular.z]
        if abs(msg.linear.x)+abs(msg.linear.y) > 1e-5: counts[topic+'_linear_nonzero'] += 1
        if abs(msg.angular.z)>1e-5: counts[topic+'_angular_nonzero'] += 1
        if abs(msg.linear.x)+abs(msg.linear.y)+abs(msg.angular.z)<1e-5: counts[topic+'_zero'] += 1
    elif isinstance(msg, LaserScan):
        latest['scan_finite'] = sum(math.isfinite(x) and msg.range_min<=x<=msg.range_max for x in msg.ranges)
    elif isinstance(msg, Path):
        latest[topic+'_poses'] = len(msg.poses)
        if msg.poses: counts[topic+'_nonempty'] += 1
    elif isinstance(msg, String):
        latest[topic] = msg.data
        counts['gate_'+msg.data] += 1
    elif isinstance(msg, TFMessage):
        for t in msg.transforms:
            if (t.header.frame_id,t.child_frame_id) == ('map','odom_nav'):
                latest['localization_tf_age_s'] = now-t.header.stamp.sec-t.header.stamp.nanosec*1e-9
                counts['map_to_odom_nav'] += 1
for topic, typ in [('/initialpose',PoseWithCovarianceStamped),('/amcl_pose',PoseWithCovarianceStamped),
                   ('/Odometry',Odometry),('/odom',Odometry),('/scan',LaserScan),('/plan',Path),
                   ('/cmd_vel_nav',Twist),('/cmd_vel',Twist),('/navigation/status',String),('/tf',TFMessage)]:
    subs.append(n.create_subscription(typ,topic,lambda msg,t=topic:cb(msg,t),qos_profile_sensor_data))
start=time.monotonic()
last=start
print(json.dumps({'event':'observer_started','wall_ns':time.time_ns(),'monotonic_ns':time.monotonic_ns()}),flush=True)
while time.monotonic()-start<a.seconds:
    rclpy.spin_once(n,timeout_sec=.1)
    if time.monotonic()-last>=5:
        print(json.dumps(dict(event='summary',elapsed_s=time.monotonic()-start,counts=counts,latest=latest)),flush=True)
        last=time.monotonic()
n.destroy_node()
rclpy.shutdown()
