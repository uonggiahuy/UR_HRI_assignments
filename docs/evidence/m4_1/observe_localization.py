import json,math,time,argparse
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus,HazardDetectionVector
from nav_msgs.msg import Odometry
from turtlebot4.initial_pose import InitialPoseNode,wait_until,seconds
from turtlebot4.teleop import cmd_vel_qos
from turtlebot4.navigation_diagnostics import cell_index
p=argparse.ArgumentParser();p.add_argument('--motion',action='store_true');p.add_argument('--output',required=True);a=p.parse_args()
rclpy.init();n=InitialPoseNode();extra={};hazards=[];samples=[]
for key,cls,topic in [('truth',Odometry,'/sim_ground_truth_pose'),('dock',DockStatus,'/dock_status')]:
 n.create_subscription(cls,topic,lambda m,k=key:extra.update({k:m}),qos_profile_sensor_data)
n.create_subscription(HazardDetectionVector,'/hazard_detection',lambda m:hazards.extend(h.type for h in m.detections),qos_profile_sensor_data)
pub=n.create_publisher(Twist,'/cmd_vel',cmd_vel_qos()) if a.motion else None
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def snapshot():
 tf=n.buffer.lookup_transform('map','base_link',Time());t=extra['truth'].pose.pose
 est=[tf.transform.translation.x,tf.transform.translation.y,yaw(tf.transform.rotation)]
 truth=[t.position.x,t.position.y,yaw(t.orientation)]
 return {'clock':n.clock_time,'estimated':est,'truth':truth,'error_m':math.dist(est[:2],truth[:2]),'yaw_error':abs(math.remainder(est[2]-truth[2],2*math.pi)),'tf_age':n.clock_time-seconds(tf.header.stamp),'linear_velocity':n.odom_msg.twist.twist.linear.x}
def spin_future(f,timeout):
 wait_until(n,f.done,time.monotonic()+timeout,'undock result');return f.result()
def stop():
 if pub:
  end=time.monotonic()+1
  while time.monotonic()<end:pub.publish(Twist());rclpy.spin_once(n,timeout_sec=.05)
try:
 wait_until(n,lambda: 'truth' in extra and 'dock' in extra and n.sensors_ready() and n.map_msg is not None and n.buffer.can_transform('map','base_link',Time()),time.monotonic()+10,'localization and ground truth')
 before=snapshot();assert before['error_m']<.3 and before['yaw_error']<.3
 result=None
 if a.motion:
  assert extra['dock'].is_docked
  assert any(e.node_name=='motion_control' for e in n.get_subscriptions_info_by_topic('/cmd_vel'))
  assert min(v for v in n.scan_msg.ranges if math.isfinite(v) and v>n.scan_msg.range_min)>.25
  client=ActionClient(n,Undock,'/undock');assert client.wait_for_server(timeout_sec=5)
  handle=spin_future(client.send_goal_async(Undock.Goal()),5);assert handle.accepted
  result=spin_future(handle.get_result_async(),35);assert result.status==4
  wait_until(n,lambda:not extra['dock'].is_docked,time.monotonic()+5,'undocked status')
  undocked=snapshot()
  assert undocked['error_m']<.3 and undocked['yaw_error']<.3
  for step in range(8):
   x=undocked['estimated'][0]+step*.05*math.cos(undocked['estimated'][2]);y=undocked['estimated'][1]+step*.05*math.sin(undocked['estimated'][2])
   assert n.map_msg.data[cell_index(x,y,n.map_msg)]==0
  cmd=Twist();cmd.linear.x=.08;end=time.monotonic()+4
 else:end=time.monotonic()+3
 next_sample=0.0;max_error=0.0;max_yaw=0.0
 while time.monotonic()<end:
  if pub:
   assert not any(h in (1,2,3,4) for h in hazards)
   assert min(v for v in n.scan_msg.ranges if math.isfinite(v) and v>n.scan_msg.range_min)>.25
   pub.publish(cmd)
  rclpy.spin_once(n,timeout_sec=.05);current=snapshot()
  max_error=max(max_error,current['error_m']);max_yaw=max(max_yaw,current['yaw_error'])
  assert current['error_m']<.3 and current['yaw_error']<.3 and abs(current['tf_age'])<.5
  if time.monotonic()>=next_sample:
   samples.append(current);next_sample=time.monotonic()+.05
 stop();after=snapshot()
 if a.motion:
  assert math.dist(after['truth'][:2],undocked['truth'][:2])>.2
  assert abs(after['linear_velocity'])<.01
 output={'status':'PASS','before':before,'after':after,'undock_status':result.status if result else None,'hazards':sorted(set(hazards)),'max_error_m':max_error,'max_yaw_error':max_yaw,'samples':samples}
 Path(a.output).write_text(json.dumps(output,indent=2)+'\n');print(json.dumps({k:v for k,v in output.items() if k!='samples'},indent=2))
finally:
 stop();n.destroy_node();rclpy.shutdown()
