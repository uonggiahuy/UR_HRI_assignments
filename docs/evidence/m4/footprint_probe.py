import json,time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer,TransformListener
from rclpy.qos import qos_profile_sensor_data
rclpy.init();n=Node('m4_footprint_check');b=Buffer();l=TransformListener(b,n);data={}
for key,cls,topic in [('local',PolygonStamped,'/local_costmap/published_footprint'),('global',PolygonStamped,'/global_costmap/published_footprint'),('truth',Odometry,'/sim_ground_truth_pose')]:n.create_subscription(cls,topic,lambda m,k=key:data.update({k:m}),qos_profile_sensor_data)
e=time.monotonic()+3
while time.monotonic()<e:rclpy.spin_once(n,timeout_sec=.01)
r={}
for key in ('local','global'):
 m=data[key];p=m.polygon.points;tf=b.lookup_transform(m.header.frame_id,'base_link',rclpy.time.Time());r[key]={'frame':m.header.frame_id,'footprint_center':[sum(v.x for v in p)/len(p),sum(v.y for v in p)/len(p)],'live_tf':[tf.transform.translation.x,tf.transform.translation.y],'footprint_stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'tf_stamp':tf.header.stamp.sec+tf.header.stamp.nanosec/1e9}
print(json.dumps(r,indent=2));n.destroy_node();rclpy.shutdown()
