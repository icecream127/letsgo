"""Load the editor's map-frame base pose and initialize the current AMCL backend."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import yaml

ROOT=Path(__file__).resolve().parents[1]


def load_start(project, map_yaml):
    doc=json.loads(Path(project).read_text(encoding='utf-8-sig'))
    if doc.get('version')!=1 or doc.get('frame_id')!='map':
        raise ValueError('需要编辑器保存的 map 坐标任务工程，不是 nav2_goals.json')
    path=Path(map_yaml).resolve()
    meta=yaml.safe_load(path.read_text())
    fingerprint=hashlib.sha256(path.read_bytes()+(path.parent/meta['image']).read_bytes()).hexdigest()
    if doc.get('map_fingerprint')!=fingerprint:
        raise ValueError('工程与当前地图不匹配，请重新核对启动区')
    area=doc.get('start_area') or {}
    p=area.get('pose')
    if not isinstance(p,dict) or not all(isinstance(p.get(k),(int,float)) and not isinstance(p.get(k),bool) and math.isfinite(p[k]) for k in ('x','y','yaw')):
        raise ValueError('任务工程尚未设置有效启动位置和朝向')
    return {k:float(p[k]) for k in ('x','y','yaw')}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project',required=True)
    parser.add_argument('--map',default=str(ROOT/'maps/competition/map.yaml'))
    parser.add_argument('--check-only',action='store_true')
    parser.add_argument('--timeout',type=float,default=60.)
    parser.add_argument('--xy-std',type=float,default=.5,help='初始位置标准差，米')
    parser.add_argument('--yaw-std-deg',type=float,default=15.,help='初始角度标准差，度')
    args=parser.parse_args()
    if not all(math.isfinite(v) and v>0 for v in (args.timeout,args.xy_std,args.yaw_std_deg)):
        parser.error('等待时间和标准差必须是正数')
    try:pose=load_start(args.project,args.map)
    except (ValueError,KeyError,OSError,TypeError) as e:parser.error(str(e))
    print(json.dumps(dict(frame_id='map',reference_frame='base_footprint',pose=pose,backend='amcl'),ensure_ascii=False),flush=True)
    if args.check_only:return 0
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from lifecycle_msgs.srv import GetState
    from lifecycle_msgs.msg import State
    rclpy.init()
    node=rclpy.create_node('saved_start_initializer')
    pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
    client=node.create_client(GetState,'/amcl/get_state')
    deadline=time.monotonic()+args.timeout
    try:
        active=False
        while rclpy.ok() and time.monotonic()<deadline:
            if not client.wait_for_service(timeout_sec=.25):continue
            future=client.call_async(GetState.Request())
            while not future.done() and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.1)
            if not future.done():break
            response=future.result()
            connected=any(i.node_name=='amcl' and i.node_namespace=='/' for i in node.get_subscriptions_info_by_topic('/initialpose'))
            if response and response.current_state.id==State.PRIMARY_STATE_ACTIVE and connected:
                active=True;break
            rclpy.spin_once(node,timeout_sec=.2)
        if not active:
            print('AMCL 未激活或未订阅 /initialpose，未发送初始位姿。',flush=True)
            return 1
        msg=PoseWithCovarianceStamped()
        msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
        msg.pose.pose.position.x=pose['x'];msg.pose.pose.position.y=pose['y']
        msg.pose.pose.orientation.z=math.sin(pose['yaw']/2)
        msg.pose.pose.orientation.w=math.cos(pose['yaw']/2)
        msg.pose.covariance[0]=msg.pose.covariance[7]=args.xy_std**2
        msg.pose.covariance[35]=math.radians(args.yaw_std_deg)**2
        pub.publish(msg)
        end=time.monotonic()+1.
        while rclpy.ok() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        print('已向 AMCL 发送一次启动位姿猜测；未判定匹配成功，未发送导航目标。',flush=True)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    try:raise SystemExit(main())
    except KeyboardInterrupt:raise SystemExit(130)
