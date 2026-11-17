"""Isolated fake AMCL receiver: checks transport only, never starts robot nodes."""
import hashlib,importlib.util,json,math,os,subprocess,tempfile,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('initializer',ROOT/'run/initialize_start.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
assert int(os.environ.get('ROS_DOMAIN_ID','0'))>=190
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
rclpy.init();node=rclpy.create_node('amcl');messages=[]
sub=node.create_subscription(PoseWithCovarianceStamped,'/initialpose',messages.append,10)
state=[3]
def status(req,res):res.current_state.id=state[0];return res
service=node.create_service(GetState,'/amcl/get_state',status)
try:
    with tempfile.TemporaryDirectory() as d:
        p=Path(d);(p/'map.pgm').write_bytes(b'P5\n1 1\n255\n\xfe');(p/'map.yaml').write_text('image: map.pgm\n')
        fingerprint=hashlib.sha256((p/'map.yaml').read_bytes()+(p/'map.pgm').read_bytes()).hexdigest()
        doc=dict(version=1,frame_id='map',map_fingerprint=fingerprint,start_area=dict(pose=dict(x=1.2,y=-2.3,yaw=math.pi/2)))
        f=p/'project.json';f.write_text(json.dumps(doc))
        assert m.load_start(f,p/'map.yaml')['x']==1.2
        doc['map_fingerprint']='bad';f.write_text(json.dumps(doc))
        try:m.load_start(f,p/'map.yaml');raise AssertionError('mismatch accepted')
        except ValueError:pass
        doc['map_fingerprint']=fingerprint;doc['start_area']['pose']=None;f.write_text(json.dumps(doc))
        try:m.load_start(f,p/'map.yaml');raise AssertionError('missing pose accepted')
        except ValueError:pass
        doc['start_area']['pose']=dict(x=1.2,y=-2.3,yaw=math.pi/2);f.write_text(json.dumps(doc))
        for active in [True,False]:
            state[0]=3 if active else 2;messages.clear()
            proc=subprocess.Popen(['/usr/bin/python3',str(ROOT/'run/initialize_start.py'),'--project',str(f),'--map',str(p/'map.yaml'),'--timeout','4'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
            limit=time.monotonic()+12
            while proc.poll() is None and time.monotonic()<limit:rclpy.spin_once(node,timeout_sec=.1)
            if proc.poll() is None:proc.kill();raise AssertionError('initializer hung')
            output=proc.communicate()[0]
            assert proc.returncode==(0 if active else 1),output
            assert len(messages)==(1 if active else 0),(len(messages),output)
            if active:
                msg=messages[0];assert msg.header.frame_id=='map';assert msg.pose.pose.position.x==1.2
                assert abs(msg.pose.pose.orientation.z-math.sqrt(.5))<1e-10
                assert msg.pose.covariance[0]==.25
        print('PASS: map binding, missing start rejection, one-shot pose/quaternion/covariance, inactive AMCL timeout without publishing')
finally:node.destroy_node();rclpy.shutdown()
