"""Simulation-only AMCL observer; motion is explicit, bounded and guarded.

Start one warehouse simulator and AMCL, initialize a known current map pose, and
undock safely before selecting a motion mode. Stationary is the default.
"""
import argparse,json,math,time
from collections import deque
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.msg import ParticleCloud
from irobot_create_msgs.msg import DockStatus,HazardDetectionVector
from ros_gz_interfaces.msg import Contacts
from turtlebot4.navigation_diagnostics import NavigationProbe,cell_index
from turtlebot4.localization_health import covariance_health,yaw,transform_pose
from turtlebot4.teleop import cmd_vel_qos
from turtlebot4.diagnostics import seconds


def main():
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['stationary','rotate','translate','combined'],default='stationary');p.add_argument('--linear-speed',type=float,default=.08);p.add_argument('--angular-speed',type=float,default=.20);p.add_argument('--registration',type=Path,help='Reviewed reference for stationary/moving assessment');p.add_argument('--duration',type=float,default=20);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    assert math.isfinite(a.duration) and 2<=a.duration<=30
    assert math.isfinite(a.linear_speed) and .02<=a.linear_speed<=.08
    assert math.isfinite(a.angular_speed) and .05<=a.angular_speed<=.20
    combined_angle=min(.6,a.angular_speed*a.duration/2)
    class ExperimentProbe(NavigationProbe):
     def __init__(self):
      self.scan_history=deque(maxlen=20);super().__init__()
     def record(self,key,msg):
      super().record(key,msg)
      if key=='scan':self.scan_history.append(msg)
    registration=json.loads(a.registration.read_text()) if a.registration else None
    if registration:
     from turtlebot4.localization_health import validate_reference_for_map
     from turtlebot4.localization_runner import resolve_saved_map_path
     validate_reference_for_map(registration,resolve_saved_map_path())
    rclpy.init();n=ExperimentProbe();extra={};hazards=[];contacts=[];samples=[];clouds=[];pose_updates=[]
    for key,cls,topic in [('truth',Odometry,'/sim_ground_truth_pose'),('dock',DockStatus,'/dock_status'),('cloud',ParticleCloud,'/particle_cloud')]:
     n.create_subscription(cls,topic,lambda m,k=key:extra.update({k:m}),qos_profile_sensor_data)
    n.create_subscription(HazardDetectionVector,'/hazard_detection',lambda m:hazards.extend(h.type for h in m.detections),qos_profile_sensor_data)
    n.create_subscription(Contacts,'/bumper_contact',lambda m:contacts.extend(m.contacts),qos_profile_sensor_data)
    pub=n.create_publisher(Twist,'/cmd_vel',cmd_vel_qos()) if a.mode!='stationary' else None
    out={'mode':a.mode,'status':'RUNNING','samples':samples,'physical_accuracy_status':'NOT VERIFIED','hazards':hazards,'bumper_contacts':0}
    def wait(pred,timeout,label):
     end=time.monotonic()+timeout
     while time.monotonic()<end:
      if pred():return
      rclpy.spin_once(n,timeout_sec=.01)
     raise TimeoutError(label)
    def stop():
     if pub:
      end=time.monotonic()+1.5
      while time.monotonic()<end:pub.publish(Twist());rclpy.spin_once(n,timeout_sec=.01)
     if 'odom' in n.messages:
      t=n.messages['odom'].twist.twist
      if abs(t.linear.x)>.01 or abs(t.angular.z)>.02:raise RuntimeError('Stop not verified')
    def scan_points(scan=None):
     scan=scan if scan is not None else n.messages['scan'];tf=n.buffer.lookup_transform('base_link',scan.header.frame_id,Time()).transform
     ranges=np.array(scan.ranges);ang=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
     mask=np.isfinite(ranges)&(ranges>scan.range_min)&(ranges<scan.range_max)
     pts=np.column_stack((ranges[mask]*np.cos(ang[mask]),ranges[mask]*np.sin(ang[mask])))
     theta=yaw(tf.rotation);c,s=math.cos(theta),math.sin(theta)
     pts=pts@np.array([[c,s],[-s,c]])+np.array([tf.translation.x,tf.translation.y])
     return pts

    def snapshot():
     tf=n.buffer.lookup_transform('map','base_link',Time());t=tf.transform
     estimate=[t.translation.x,t.translation.y,yaw(t.rotation)]
     truth=extra['truth'].pose.pose;odom=n.messages['odom'].pose.pose
     pose=n.messages['pose'];cloud=extra.get('cloud')
     cloud_data=None
     if cloud and cloud.particles:
      weights=np.array([p.weight for p in cloud.particles]);weights/=weights.sum()
      pts=np.array([[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in cloud.particles])
      mu=np.sum(pts[:,:2]*weights[:,None],axis=0);center=pts[:,:2]-mu
      cloud_data={'stamp':seconds(cloud.header.stamp),'count':len(weights),'post_resample_neff':float(1/np.sum(weights**2)), 'mean_xy':mu.tolist(),'position_sigma':float(np.sqrt(np.linalg.eigvalsh((center*weights[:,None]).T@center)[-1])), 'xy_bounds_95':np.percentile(pts[:,:2],[2.5,97.5],axis=0).tolist()}
     scan=next((msg for msg in reversed(n.scan_history) if n.clock_samples[-1]-seconds(msg.header.stamp)<=.1 and n.buffer.can_transform('map','base_link',Time.from_msg(msg.header.stamp))),None)
     assert scan is not None, 'No scan with time-aligned TF within 0.1s'
     scan_tf=n.buffer.lookup_transform('map','base_link',Time.from_msg(scan.header.stamp)).transform
     scan_est=[scan_tf.translation.x,scan_tf.translation.y,yaw(scan_tf.rotation)]
     points=scan_points(scan);theta=scan_est[2];c,s=math.cos(theta),math.sin(theta)
     hits=points@np.array([[c,s],[-s,c]])+scan_est[:2]
     ix=np.floor((hits[:,0]-grid.info.origin.position.x)/grid.info.resolution).astype(int);iy=np.floor((hits[:,1]-grid.info.origin.position.y)/grid.info.resolution).astype(int)
     inside=(ix>=0)&(iy>=0)&(ix<grid.info.width)&(iy<grid.info.height)
     distances=distance[iy[inside],ix[inside]]
     twist=n.messages['odom'].twist.twist
     return {'sim_time':n.clock_samples[-1],'map_pose':estimate,'world_pose':[truth.position.x,truth.position.y,yaw(truth.orientation)],'odom_pose':[odom.position.x,odom.position.y,yaw(odom.orientation)],'amcl_stamp':seconds(pose.header.stamp),'amcl_pose':[pose.pose.pose.position.x,pose.pose.pose.position.y,yaw(pose.pose.pose.orientation)],'covariance':covariance_health(pose),'variance':[float(pose.pose.covariance[i]) for i in (0,7,35)],'cloud':cloud_data,'scan_alignment':{'valid_endpoints':len(distances),'median_m':float(np.median(distances)), 'p90_m':float(np.percentile(distances,90)), 'p95_m':float(np.percentile(distances,95)),'fraction_within_0_15m':float(np.mean(distances<=.150001)), 'fraction_at_boundary':float(np.mean(abs(distances-.15)<1e-6)), 'scan_map_pose':scan_est,'geometry_scan_age_sec':n.clock_samples[-1]-seconds(scan.header.stamp)},'ages':{key:n.clock_samples[-1]-seconds(n.messages[key].header.stamp) for key in ('scan','odom')},'truth_age':n.clock_samples[-1]-seconds(extra['truth'].header.stamp),'tf_age':n.clock_samples[-1]-seconds(tf.header.stamp),'velocity':[twist.linear.x,twist.angular.z]}
    try:
     wait(lambda:all(k in n.messages for k in ('clock','scan','odom','pose','map')) and 'truth' in extra and 'dock' in extra and n.buffer.can_transform('map','base_link',Time()),10,'required graph')
     grid=n.messages['map'];cells=np.array(grid.data).reshape(grid.info.height,grid.info.width)
     distance=distance_transform_edt(cells!=100)*grid.info.resolution
     free_clearance=distance_transform_edt(cells==0)*grid.info.resolution
     names=n.get_node_names();assert not any(name in ('controller_server','bt_navigator','slam_toolbox') for name in names)
     wait(lambda:len(n.clock_samples)>10,3,'clock')
     wait(lambda:n.buffer.can_transform('map','base_link',Time.from_msg(n.messages['scan'].header.stamp)),5,'TF at scan acquisition')
     if pub:
      assert not extra['dock'].is_docked
      assert any(e.node_name=='motion_control' for e in n.get_subscriptions_info_by_topic('/cmd_vel'))
      assert not any(e.node_name=='turtlebot4_teleop' for e in n.get_publishers_info_by_topic('/cmd_vel'))
      assert min(v for v in n.messages['scan'].ranges if math.isfinite(v) and v>n.messages['scan'].range_min)>.5
     out['before']=snapshot();out['frames']={'scan':n.messages['scan'].header.frame_id,'odom':n.messages['odom'].header.frame_id,'odom_child':n.messages['odom'].child_frame_id,'truth':extra['truth'].header.frame_id}
     sensor=n.buffer.lookup_transform('base_link',n.messages['scan'].header.frame_id,Time()).transform
     out['laser_extrinsic']=[sensor.translation.x,sensor.translation.y,sensor.translation.z,yaw(sensor.rotation)]
     # Validate the full centerline sweep with a 0.45m margin before translation.
     start=out['before']['map_pose']
     if a.mode in ('translate','combined'):
      heading=start[2]+(combined_angle if a.mode=='combined' else 0)
      length=a.linear_speed*(a.duration if a.mode=='translate' else a.duration/2)
      for step in np.linspace(0,length,30):
       x=start[0]+step*math.cos(heading);y=start[1]+step*math.sin(heading);i=cell_index(x,y,grid)
       assert free_clearance.ravel()[i]>.45
     tstart=time.monotonic();end=tstart+a.duration;next_sample=0.;last_pose_stamp=None;previous=None
     while time.monotonic()<end:
      rclpy.spin_once(n,timeout_sec=.005)
      now=time.monotonic();sim=n.clock_samples[-1]
      assert max(abs(sim-seconds(n.messages[k].header.stamp)) for k in ('scan','odom'))<1
      assert abs(sim-seconds(extra['truth'].header.stamp))<1
      assert not any(h in (1,2,3,4) for h in hazards) and not contacts
      if pub:
       assert min(v for v in n.messages['scan'].ranges if math.isfinite(v) and v>n.messages['scan'].range_min)>.5
       cmd=Twist()
       if a.mode=='rotate':cmd.angular.z=a.angular_speed
       elif a.mode=='translate':cmd.linear.x=a.linear_speed
       elif now-tstart<a.duration/2:cmd.angular.z=combined_angle/(a.duration/2)
       else:cmd.linear.x=a.linear_speed
       pub.publish(cmd)
      if now>=next_sample:
       current=snapshot();assert abs(current['tf_age'])<1.5
       assert current['covariance']['position_major_std_m']<.75
       if previous:assert math.dist(current['map_pose'][:2],previous['map_pose'][:2])<.5
       if current['amcl_stamp']!=last_pose_stamp:pose_updates.append(current['amcl_stamp']);last_pose_stamp=current['amcl_stamp']
       samples.append(current);previous=current;next_sample=now+.20
     stop();wait(lambda:True,1,'stop');out['after']=snapshot()
     out['status']='PASS';out['unique_amcl_updates']=len(pose_updates);out['amcl_update_stamps']=pose_updates
     for key in ('world_pose','odom_pose'):
      out[key+'_displacement_m']=math.dist(out['before'][key][:2],out['after'][key][:2])
      out[key+'_yaw_change_rad']=math.remainder(out['after'][key][2]-out['before'][key][2],2*math.pi)
     cloud=extra.get('cloud')
     if cloud:out['final_particles']=[[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation),p.weight] for p in cloud.particles]
     out['final_scan_base_points']=scan_points().tolist()
     if registration:
      from turtlebot4.localization_analysis import assess_convergence
      out['convergence']=assess_convergence(samples,registration)
      out['physical_accuracy_status']='PASS' if out['convergence']['ground_truth_accuracy_verified'] else 'FAIL'
    except BaseException as exc:out['status']='FAIL';out['error']=str(exc);raise
    finally:
     try:stop()
     finally:
      out['bumper_contacts']=len(contacts);out['hazards']=sorted(set(hazards));a.output.write_text(json.dumps(out,indent=2));n.destroy_node();rclpy.shutdown()


if __name__ == "__main__":
    main()
