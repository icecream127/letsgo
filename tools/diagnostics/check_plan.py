#!/usr/bin/env python3
"""Request a plan from the live robot pose. Does not command motion."""
import argparse
import json
import math
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
p=argparse.ArgumentParser()
p.add_argument('x',type=float);p.add_argument('y',type=float);p.add_argument('--yaw',type=float,default=0.)
a=p.parse_args()
rclpy.init();n=rclpy.create_node('live_plan_check')
c=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
if not c.wait_for_server(timeout_sec=15):
    print(json.dumps({'error':'planner_action_unavailable'}));raise SystemExit(1)
g=ComputePathToPose.Goal();g.goal.header.frame_id='map';g.goal.header.stamp=n.get_clock().now().to_msg()
g.goal.pose.position.x=a.x;g.goal.pose.position.y=a.y
g.goal.pose.orientation.z=math.sin(a.yaw/2);g.goal.pose.orientation.w=math.cos(a.yaw/2)
g.use_start=False;g.planner_id='GridBased'
f=c.send_goal_async(g);rclpy.spin_until_future_complete(n,f,timeout_sec=20)
if not f.done():raise RuntimeError('goal response timeout')
h=f.result();print(json.dumps({'accepted':h.accepted,'goal':[a.x,a.y,a.yaw],'use_live_start':True}),flush=True)
if h.accepted:
    f=h.get_result_async();rclpy.spin_until_future_complete(n,f,timeout_sec=20)
    if not f.done():
        h.cancel_goal_async();raise RuntimeError('planning timeout')
    r=f.result();print(json.dumps({'status':r.status,'path_poses':len(r.result.path.poses)}),flush=True)
n.destroy_node();rclpy.shutdown()
