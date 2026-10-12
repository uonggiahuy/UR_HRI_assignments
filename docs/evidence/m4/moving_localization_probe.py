import json,math,time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus,HazardDetectionVector
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from turtlebot4.navigation_diagnostics import NavigationProbe,wait_for_readiness,cell_index
from turtlebot4.teleop import cmd_vel_qos
rclpy.init();n=NavigationProbe();extra={};hazards=[];samples=[]
for key,cls,topic in [('truth',Odometry,'/sim_ground_truth_pose'),('dock',DockStatus,'/dock_status')]:
 n.create_subscription(cls,topic,lambda m,k=key:extra.update({k:m}),qos_profile_sensor_data)
n.create_subscription(HazardDetectionVector,'/hazard_detection',lambda m: hazards.extend(h.type for h in m.detections),qos_profile_sensor_data)
pub=n.create_publisher(Twist,'/cmd_vel',cmd_vel_qos())
def spin_until(f,timeout=30):
 rclpy.spin_until_future_complete(n,f,timeout_sec=timeout);assert f.done();return f.result()
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def snapshot():
 tf=n.buffer.lookup_transform('map','base_link',rclpy.time.Time()).transform
 t=extra['truth'].pose.pose
 return {'sim_time':n.clock_samples[-1],'estimated':[tf.translation.x,tf.translation.y,yaw(tf.rotation)],'truth':[t.position.x,t.position.y,yaw(t.orientation)],'error_m':math.hypot(tf.translation.x-t.position.x,tf.translation.y-t.position.y),'yaw_error':abs(math.atan2(math.sin(yaw(tf.rotation)-yaw(t.orientation)),math.cos(yaw(tf.rotation)-yaw(t.orientation))))}
def stop():
 end=time.monotonic()+1
 while time.monotonic()<end:pub.publish(Twist());rclpy.spin_once(n,timeout_sec=.05)
try:
 wait_for_readiness(n,Path('/root/ros2_ws/src/turtlebot4/maps/map.yaml'),20)
 before=snapshot()
 assert extra['dock'].is_docked
 scan=n.messages['scan'];assert min(v for v in scan.ranges if math.isfinite(v) and v>scan.range_min)>.25
 client=ActionClient(n,Undock,'/undock');assert client.wait_for_server(timeout_sec=5)
 handle=spin_until(client.send_goal_async(Undock.Goal()),5);assert handle.accepted
 result=spin_until(handle.get_result_async(),35);assert result.status==4
 end=time.monotonic()+2
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 assert not extra['dock'].is_docked
 undocked=snapshot();assert undocked['error_m']<.2 and undocked['yaw_error']<.25
 # Verified map-free cells and LiDAR clearance along a short forward segment.
 for step in range(8):
  x=undocked['estimated'][0]+step*.05*math.cos(undocked['estimated'][2]);y=undocked['estimated'][1]+step*.05*math.sin(undocked['estimated'][2])
  assert n.messages['map'].data[cell_index(x,y,n.messages['map'])]==0
 cmd=Twist();cmd.linear.x=.08;end=time.monotonic()+4
 while time.monotonic()<end:
  assert not any(h in (1,2,3,4) for h in hazards)
  pub.publish(cmd);rclpy.spin_once(n,timeout_sec=.05)
  if 'truth' in extra:samples.append(snapshot())
 stop();after=snapshot()
 assert after['error_m']<.2 and after['yaw_error']<.25
 assert math.dist(after['truth'][:2],undocked['truth'][:2])>.2
 assert abs(n.messages['odom'].twist.twist.linear.x)<.01
 out={'status':'PASS','before':before,'undocked':undocked,'after':after,'max_error_m':max(s['error_m'] for s in samples),'max_yaw_error':max(s['yaw_error'] for s in samples),'undock_action_status':result.status,'hazard_types':sorted(set(hazards)),'motion_samples':samples,'stopped_velocity':n.messages['odom'].twist.twist.linear.x}
 Path('/root/ros2_ws/src/turtlebot4/docs/evidence/m4/moving_localization.json').write_text(json.dumps(out,indent=2)+'\n')
 print(json.dumps({k:v for k,v in out.items() if k!='motion_samples'},indent=2))
finally:
 stop();n.destroy_node();rclpy.shutdown()
