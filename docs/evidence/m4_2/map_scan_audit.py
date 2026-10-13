import json,time,math
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import label,distance_transform_edt
import rclpy
from turtlebot4.initial_pose import InitialPoseNode
from turtlebot4.map_saver import validate_saved_map_files
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
p=Path('/root/ros2_ws/src/turtlebot4/maps')
meta=validate_saved_map_files(p/'map.yaml',p/'map.pgm')
a=np.flipud(np.array(Image.open(p/'map.pgm')))
free=a==254; occupied=a==0
labs,n=label(free); sizes=np.bincount(labs.ravel())[1:]
out={'map':meta,'free_components':n,'largest_free_components':sorted(sizes.tolist(),reverse=True)[:10]}
rclpy.init(); node=InitialPoseNode();truth=[]
node.create_subscription(Odometry,'/sim_ground_truth_pose',lambda m:truth.append(m),qos_profile_sensor_data)
end=time.monotonic()+6
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
t=truth[-1];q=t.pose.pose.orientation
out['truth']={'frame':t.header.frame_id,'stamp':t.header.stamp.sec+t.header.stamp.nanosec/1e9,'position':[t.pose.pose.position.x,t.pose.pose.position.y],'yaw':2*math.atan2(q.z,q.w)}
scan=node.scan_msg
st=node.buffer.lookup_transform('base_link',scan.header.frame_id,Time()).transform
sensor_yaw=2*math.atan2(st.rotation.z,st.rotation.w)
angles=sensor_yaw+scan.angle_min+np.arange(len(scan.ranges))*scan.angle_increment
r=np.array(scan.ranges);v=np.isfinite(r)&(r>scan.range_min)&(r<scan.range_max)
pts=np.column_stack((r[v]*np.cos(angles[v])+st.translation.x,r[v]*np.sin(angles[v])+st.translation.y))
# Diagnostic candidate identity only; do not declare it registered from AMCL.
x=pts[:,0]+t.pose.pose.position.x;y=pts[:,1]+t.pose.pose.position.y
ix=np.floor((x-meta['origin'][0])/meta['resolution']).astype(int);iy=np.floor((y-meta['origin'][1])/meta['resolution']).astype(int)
inside=(ix>=0)&(ix<a.shape[1])&(iy>=0)&(iy<a.shape[0]);dist=distance_transform_edt(~occupied)*meta['resolution'];d=dist[iy[inside],ix[inside]]
out['candidate_identity_scan_residual']={'count':len(d),'median_m':float(np.median(d)),'p90_m':float(np.percentile(d,90)),'fraction_within_0_15m':float(np.mean(d<=.15)),'registration_verified':False}
np.savez('/tmp/m4_2/scan_map.npz',points=pts,truth=np.array(out['truth']['position']),map=a)
Path('/tmp/m4_2/map_audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2));node.destroy_node();rclpy.shutdown()
