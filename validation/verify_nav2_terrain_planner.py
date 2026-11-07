"""Run with sourced ROS Humble and an unused ROS_DOMAIN_ID; no controllers."""
import os,sys,time,signal,subprocess,json,tempfile
from pathlib import Path
import yaml,rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
assert os.environ.get('ROS_DOMAIN_ID') == '189' and os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Use isolated ROS_DOMAIN_ID=189 ROS_LOCALHOST_ONLY=1'
import argparse
parser=argparse.ArgumentParser();parser.add_argument('--c',nargs=2,type=float,default=[5.,-4.2]);args=parser.parse_args()
root=Path(__file__).resolve().parents[1]
report_dir=root/'validation/results/competition'
report_dir.mkdir(parents=True,exist_ok=True)
cfg=yaml.safe_load((root/'lidar_ws/src/ros2_to_stm32/config/nav2.yaml').read_text())
params={k:cfg[k] for k in ['planner_server','global_costmap']}
Path('/tmp/terrain_planner_params.yaml').write_text(yaml.safe_dump(params))
processes=[];logs=[]
def spawn(args):
 f=open('/tmp/terrain_planner_'+str(len(processes))+'.log','w');logs.append(f)
 processes.append(subprocess.Popen(args,cwd=root,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
def wait(future,timeout=30):
 rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
 if not future.done():raise RuntimeError('planner request timeout')
 return future.result()
try:
 spawn(['bash','run/nav2_map.sh','competition'])
 spawn(['ros2','run','tf2_ros','static_transform_publisher','--x','5.0','--y','-2.8','--z','0','--roll','0','--pitch','0','--yaw','0','--frame-id','map','--child-frame-id','base_footprint'])
 spawn(['ros2','run','nav2_planner','planner_server','--ros-args','--params-file','/tmp/terrain_planner_params.yaml'])
 spawn(['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args','-r','__node:=terrain_planner_manager','-p','autostart:=true','-p','node_names:=[planner_server]'])
 rclpy.init();node=rclpy.create_node('verify_terrain_planning');client=ActionClient(node,ComputePathToPose,'compute_path_to_pose')
 assert client.wait_for_server(timeout_sec=30),'planner action unavailable'
 from nav_msgs.msg import OccupancyGrid
 from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
 snapshots=[]
 sub=node.create_subscription(OccupancyGrid,'/global_costmap/costmap',snapshots.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
 deadline=time.monotonic()+20
 while sum(m.info.width==260 for m in snapshots)<2 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.2)
 assert snapshots and snapshots[-1].info.width==260,'full costmap unavailable'
 import numpy as np
 m=snapshots[-1];np.savez('/tmp/nav2_costmap_check.npz',data=np.array(m.data).reshape(m.info.height,m.info.width),origin=[m.info.origin.position.x,m.info.origin.position.y],resolution=m.info.resolution)
 for x,y in [(5.,-2.8),args.c,(1.2,-2.95)]:
  ix=int((x-m.info.origin.position.x)/m.info.resolution);iy=int((y-m.info.origin.position.y)/m.info.resolution);print('cost',x,y,m.data[iy*m.info.width+ix],flush=True)
 def pose(x,y):
  p=PoseStamped();p.header.frame_id='map';p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.;return p
 results=[]
 for label,start,end in [('field_to_C',[5.,-2.8],args.c),('C_to_A',args.c,[1.2,-2.95])]:
  goal=ComputePathToPose.Goal();goal.start=pose(*start);goal.goal=pose(*end);goal.use_start=True;goal.planner_id='GridBased'
  handle=wait(client.send_goal_async(goal));assert handle.accepted
  result=wait(handle.get_result_async());path=result.result.path
  assert result.status==4 and len(path.poses)>2,(label,result.status,result.result)
  actual=path.poses[-1].pose.position;offset=((actual.x-end[0])**2+(actual.y-end[1])**2)**.5
  assert offset<.1,(label,offset)
  results.append(dict(segment=label,status='PASS',poses=len(path.poses),goal_error_m=offset,path=[[p.pose.position.x,p.pose.position.y] for p in path.poses]))
  print(label,'PASS',len(path.poses),'poses, goal error',offset,flush=True)
 import hashlib
 report={'planner':'nav2_navfn_planner/NavfnPlanner','robot_radius':.3,'inflation_radius':.32,'map_sha256':hashlib.sha256((root/'maps/competition/map.pgm').read_bytes()).hexdigest(),'C_target':args.c,'config':'unchanged project global_costmap and planner_server parameters','segments':results}
 (report_dir/'nav2_planner_check.json').write_text(json.dumps(report,indent=2)+'\n')
finally:
 for p in processes:
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 for p in processes:
  try:p.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 for f in logs:f.close()
 if rclpy.ok():rclpy.shutdown()
