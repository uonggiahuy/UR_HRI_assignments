"""Bounded simulation measurement; optional short motion with stop/hazard guards."""
import json,math,time,argparse
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import HazardDetectionVector,DockStatus
from ros_gz_interfaces.msg import Contacts
from turtlebot4.navigation_diagnostics import NavigationProbe,cell_index
from turtlebot4.localization_health import transform_pose,yaw,covariance_health
from turtlebot4.teleop import cmd_vel_qos
p=argparse.ArgumentParser();p.add_argument('--motion',action='store_true');p.add_argument('--output',required=True);a=p.parse_args()
rclpy.init();n=NavigationProbe();extra={};hazards=[];contacts=[];samples=[];reg=json.loads(Path('/tmp/m4_2/registration.json').read_text())
for key,cls,topic in [('truth',Odometry,'/sim_ground_truth_pose'),('dock',DockStatus,'/dock_status')]:n.create_subscription(cls,topic,lambda m,k=key:extra.update({k:m}),qos_profile_sensor_data)
n.create_subscription(HazardDetectionVector,'/hazard_detection',lambda m:hazards.extend(h.type for h in m.detections),qos_profile_sensor_data)
n.create_subscription(Contacts,'/bumper_contact',lambda m:contacts.extend(m.contacts),qos_profile_sensor_data)
pub=n.create_publisher(Twist,'/cmd_vel',cmd_vel_qos()) if a.motion else None
out={'status':'RUNNING','registration_verified':reg['verified'],'reference':'registration.json','physical_accuracy_status':'NOT VERIFIED','samples':samples}
def stop():
 if pub:
  end=time.monotonic()+1.5
  while time.monotonic()<end:pub.publish(Twist());rclpy.spin_once(n,timeout_sec=.02)
def snapshot():
 tf=n.buffer.lookup_transform('map','base_link',Time());t=extra['truth'].pose.pose
 est=[tf.transform.translation.x,tf.transform.translation.y,yaw(tf.transform.rotation)]
 truth=[t.position.x,t.position.y,yaw(t.orientation)];aligned=transform_pose(truth,(*reg['translation'],reg['yaw']))
 return {'clock':n.clock_samples[-1],'estimate':est,'truth_world':truth,'candidate_aligned_truth':aligned,'candidate_error_m':math.dist(est[:2],aligned[:2]),'candidate_yaw_error_rad':abs(math.remainder(est[2]-aligned[2],2*math.pi)),'tf_age_sec':n.clock_samples[-1]-(tf.header.stamp.sec+tf.header.stamp.nanosec/1e9),'sensor_ages_sec':{key:n.clock_samples[-1]-(n.messages[key].header.stamp.sec+n.messages[key].header.stamp.nanosec/1e9) for key in ('scan','odom')},'truth_stamp':extra['truth'].header.stamp.sec+extra['truth'].header.stamp.nanosec/1e9,'covariance':covariance_health(n.messages['pose']),'velocity':[n.messages['odom'].twist.twist.linear.x,n.messages['odom'].twist.twist.angular.z]}
try:
 end=time.monotonic()+6
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
 out['before']=snapshot()
 if a.motion:
  assert not extra['dock'].is_docked
  assert not any(h in (1,2,3,4) for h in hazards)
  grid=n.messages['map'];clearance=distance_transform_edt(np.array(grid.data).reshape(grid.info.height,grid.info.width)==0)*grid.info.resolution
  start=out['before']['estimate']
  for step in range(9):
   x=start[0]+step*.05*math.cos(start[2]);y=start[1]+step*.05*math.sin(start[2]);i=cell_index(x,y,grid)
   assert clearance.ravel()[i]>.40
  assert min(v for v in n.messages['scan'].ranges if math.isfinite(v) and v>n.messages['scan'].range_min)>.25
 cmd=Twist();cmd.linear.x=.08
 end=time.monotonic()+(4 if a.motion else 5);next_sample=0
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.02)
  current=snapshot()
  assert abs(current['tf_age_sec'])<1 and max(abs(v) for v in current['sensor_ages_sec'].values())<1
  assert not any(h in (1,2,3,4) for h in hazards) and not contacts
  if pub:
   assert min(v for v in n.messages['scan'].ranges if math.isfinite(v) and v>n.messages['scan'].range_min)>.25
   pub.publish(cmd)
  if time.monotonic()>=next_sample:samples.append(current);next_sample=time.monotonic()+.1
 stop();out['after']=snapshot();out['status']='PASS';out['hazard_types']=sorted(set(hazards));out['contacts']=len(contacts)
 if pub:assert abs(out['after']['velocity'][0])<.01 and abs(out['after']['velocity'][1])<.02
except BaseException as e:
 out['status']='FAIL';out['error']=str(e);raise
finally:
 stop();Path(a.output).write_text(json.dumps(out,indent=2));n.destroy_node();rclpy.shutdown()
