"""Isolated ROS integration: fake sensor, real AMCL/planner/controller, no hardware.
Run with ROS_DOMAIN_ID=195 RMW_IMPLEMENTATION=rmw_fastrtps_cpp.
"""
import argparse
import math
import struct
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from rclpy.time import Time
from lifecycle_msgs.srv import GetState


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gicp', action='store_true')
    gicp = parser.parse_args().gicp
    assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 190, 'Use an isolated test domain'
    with tempfile.TemporaryDirectory(prefix='letsgo_nav_test_') as tmp:
        directory = Path(tmp)
        size = 160
        pixels = bytes(0 if x < 2 or y < 2 or x >= size-2 or y >= size-2 else 254
                       for y in range(size) for x in range(size))
        (directory/'map.pgm').write_bytes(b'P5\n160 160\n255\n'+pixels)
        (directory/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [-4.0, -4.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        points = []
        for i in range(720):
            angle = i*2*math.pi/720
            c, s = math.cos(angle), math.sin(angle)
            distance = 3.9/max(abs(c), abs(s))
            points.append((distance*c, distance*s, 0.0))
        extra_args = []
        if gicp:
            points = [(x, y, z) for x, y, _ in points for z in (-.2, 0., .25, .5)]
            reference = directory/'reference.pcd'
            count = len(points)
            header = f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {count}\nHEIGHT 1\nPOINTS {count}\nDATA binary\n'
            reference.write_bytes(header.encode() + b''.join(struct.pack('<fff', *p) for p in points))
            extra_args = ['localization:=gicp', f'pcd:={reference}', 'gicp_seed:=[0.1, 0.0, 0.0, 0.0, 0.0, 0.0]']
        log = open('/tmp/letsgo_navigation_gicp_test.log' if gicp else '/tmp/letsgo_navigation_test.log', 'w')
        process = subprocess.Popen(['ros2', 'launch', 'ros2_to_stm32', 'navigation.launch.py',
            'map:='+str(directory/'map.yaml'), 'sensors:=false', 'rviz:=false', *extra_args],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Node('navigation_test_sensor')
        broadcaster = TransformBroadcaster(node)
        if gicp:
            tf_buffer = Buffer()
            tf_listener = TransformListener(tf_buffer, node)
        odom_pub = node.create_publisher(Odometry, '/Odometry', 10)
        cloud_pub = node.create_publisher(PointCloud2, '/cloud_registered_body', 10 if gicp else qos_profile_sensor_data)
        pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        commands, plans, scans = [], [], []
        node.create_subscription(Twist, '/cmd_vel', lambda msg: commands.append((time.monotonic(), msg.linear.x, msg.angular.z)), 10)
        node.create_subscription(NavPath, '/plan', lambda msg: plans.append(len(msg.poses)), 10)
        node.create_subscription(LaserScan, '/scan', lambda msg: scans.append(len(msg.ranges)), qos_profile_sensor_data)
        send_sensor = True
        bad_cloud = False
        if gicp:
            from std_msgs.msg import String
            statuses = []
            node.create_subscription(String, '/navigation/status', lambda msg: statuses.append(msg.data), 10)
        last_send = 0.0

        def pump(seconds):
            nonlocal last_send
            end = time.monotonic()+seconds
            while time.monotonic()<end:
                now = time.monotonic()
                if send_sensor and now-last_send >= .08:
                    last_send = now
                    stamp = node.get_clock().now().to_msg()
                    odom = Odometry()
                    odom.header = Header(stamp=stamp, frame_id='odom')
                    odom.child_frame_id = 'livox_frame'
                    odom.pose.pose.position.x = .25 if gicp else 0.
                    odom.pose.pose.orientation.w = 1.0
                    odom_pub.publish(odom)
                    tf = TransformStamped()
                    tf.header = odom.header
                    tf.child_frame_id = 'livox_frame'
                    tf.transform.translation.x = odom.pose.pose.position.x
                    tf.transform.rotation.w = 1.0
                    broadcaster.sendTransform(tf)
                    cloud_pub.publish(create_cloud_xyz32(Header(stamp=stamp, frame_id='livox_frame'), [(x+50., y, z) for x,y,z in points] if bad_cloud else points))
                rclpy.spin_once(node, timeout_sec=.01)

        try:
            pump(8)
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = 'map'
            pose.pose.pose.orientation.w = 1.0
            pose.pose.covariance[0] = pose.pose.covariance[7] = .01
            pose.pose.covariance[35] = .01
            for _ in range(0 if gicp else 3):
                pose.header.stamp = node.get_clock().now().to_msg()
                pose_pub.publish(pose)
                pump(1)
            state_client = node.create_client(GetState, '/bt_navigator/get_state')
            deadline = time.monotonic()+30
            active = False
            while time.monotonic()<deadline and not active:
                if state_client.service_is_ready():
                    state = state_client.call_async(GetState.Request())
                    while not state.done() and time.monotonic()<deadline: pump(.1)
                    active = state.done() and state.result().current_state.id == 3
                pump(.1)
            assert active, 'Nav2 did not activate; see /tmp/letsgo_navigation_test.log'
            client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
            deadline = time.monotonic()+30
            while not client.server_is_ready() and time.monotonic()<deadline: pump(.2)
            assert client.server_is_ready(), 'NavigateToPose not active; see /tmp/letsgo_navigation_test.log'
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x = 1.0
            goal.pose.pose.orientation.w = 1.0
            future = client.send_goal_async(goal)
            deadline = time.monotonic()+15
            while not future.done() and time.monotonic()<deadline: pump(.1)
            assert future.done() and future.result().accepted, 'Goal rejected'
            deadline = time.monotonic()+8
            while not any(abs(x)+abs(w)>.01 for _,x,w in commands) and time.monotonic()<deadline: pump(.1)
            assert scans and plans and max(plans)>2, 'No scan or plan'
            assert any(abs(x)+abs(w)>.01 for _,x,w in commands), 'No nonzero gated cmd_vel'
            print(f'PASS: cloud -> scan -> {"GICP" if gicp else "AMCL"} -> plan -> nonzero cmd_vel', flush=True)
            if gicp:
                assert ('amcl', '/') not in node.get_node_names_and_namespaces()
                base = tf_buffer.lookup_transform('map', 'base_footprint', Time()).transform
                assert abs(base.translation.x) < .05 and abs(base.translation.y) < .05, base
                assert tf_buffer.can_transform('map', 'livox_frame', Time())
                scan_count_before_rejection = len(scans)
                bad_cloud = True
                raw_pub = node.create_publisher(Twist, '/cmd_vel_nav', 10)
                for _ in range(40):
                    raw = Twist()
                    raw.linear.x = .2
                    raw_pub.publish(raw)
                    pump(.1)
                assert statuses[-1] == 'waiting_for_fresh_gicp_evidence', statuses[-10:]
                assert len(scans) > scan_count_before_rejection + 10, 'Local scanning stopped when global localization was rejected'
                recent = [abs(x)+abs(w) for t,x,w in commands if t>time.monotonic()-.5]
                assert recent and max(recent)<1e-8, 'Rejected localization did not block velocity'
                print('PASS: rejected GICP evidence blocks nonzero input while sensors still publish', flush=True)
            publishers = node.get_publishers_info_by_topic('/cmd_vel')
            assert len(publishers)==1 and publishers[0].node_name=='nav_cmd_gate', publishers
            future.result().cancel_goal_async()
            pump(1.5)
            assert all(abs(x)+abs(w)<1e-8 for t,x,w in commands if t>time.monotonic()-.5), 'Cancel did not stop'
            # Inject raw commands after sensor dropout to verify gate, independently of controller behavior.
            send_sensor = False
            raw_pub = node.create_publisher(Twist, '/cmd_vel_nav', 10)
            for _ in range(20):
                raw = Twist()
                raw.linear.x = .2
                raw_pub.publish(raw)
                pump(.1)
            assert all(abs(x)+abs(w)<1e-8 for t,x,w in commands if t>time.monotonic()-.5), 'Stale sensors not blocked'
            print('PASS: single output publisher, cancellation and stale-sensor zero output', flush=True)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            log.close()


if __name__ == '__main__': main()
