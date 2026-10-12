import argparse,json,math,time,subprocess,uuid
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped,PoseWithCovarianceStamped
from nav2_msgs.msg import Costmap
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer,TransformListener
p=argparse.ArgumentParser();p.add_argument('--duration',type=float,default=90);p.add_argument('--output',required=True);p.add_argument('--nav',action='store_true');a=p.parse_args()
rclpy.init();n=Node('recovery_light_observer_'+uuid.uuid4().hex[:8]);n.set_parameters([Parameter('use_sim_time',value=True)])
b=Buffer();l=TransformListener(b,n);data={};last={};ranges={};tfstamps={};rows=[];services=[];pending={};clients={}
def sec(s):return s.sec+s.nanosec/1e9
def record(k,m):
 data[k]=m;last[k]=time.monotonic()
 s=sec(m.clock if k=='clock' else m.metadata.update_time if k in ('global','local') else m.header.stamp)
 if k not in ranges:ranges[k]=[s,s,0]
 ranges[k][1]=s;ranges[k][2]+=1
subs=[('clock',Clock,'/clock'),('scan',LaserScan,'/scan'),('odom',Odometry,'/odom'),('truth',Odometry,'/sim_ground_truth_pose'),('pose',PoseWithCovarianceStamped,'/amcl_pose')]
if a.nav:
 for k in ('global','local'):subs.extend([(k,Costmap,f'/{k}_costmap/costmap_raw'),(k+'_footprint',PolygonStamped,f'/{k}_costmap/published_footprint')])
for k,t,topic in subs:n.create_subscription(t,topic,lambda m,k=k:record(k,m),qos_profile_sensor_data)
def tfcb(m):
 for t in m.transforms:
  if (t.header.frame_id,t.child_frame_id) in [('map','odom'),('odom','base_link')]:tfstamps[t.header.frame_id+'->'+t.child_frame_id]=[sec(t.header.stamp),time.monotonic()]
n.create_subscription(TFMessage,'/tf',tfcb,qos_profile_sensor_data)
names=['map_server','amcl']
if a.nav:names+=['controller_server','smoother_server','planner_server','behavior_server','bt_navigator','waypoint_follower','velocity_smoother','global_costmap/global_costmap','local_costmap/local_costmap']
for name in names:
 for kind,t in [('get_state',GetState),('get_parameters',GetParameters)]:clients[name+'/'+kind]=n.create_client(t,'/'+name+'/'+kind)
if a.nav:clients['lifecycle_manager_navigation/get_parameters']=n.create_client(GetParameters,'/lifecycle_manager_navigation/get_parameters')
start=time.monotonic();nextrow=start;nextsvc=start+2
try:
 while time.monotonic()-start<a.duration:
  rclpy.spin_once(n,timeout_sec=.01);now=time.monotonic()
  for key,(f,s) in list(pending.items()):
   if f.done() or now-s>2:
    r={'wall_elapsed':now-start,'service':key,'latency':now-s,'status':'TIMEOUT'}
    if f.done() and f.result() is not None:
     response=f.result();r['status']='OK';r['value']=response.current_state.label if key.endswith('get_state') else [(v.type,v.bool_value,list(v.string_array_value)) for v in response.values]
    else:clients[key].remove_pending_request(f);f.cancel()
    services.append(r);del pending[key]
  if now>=nextsvc:
   for key,c in clients.items():
    if key in pending:continue
    if not c.service_is_ready():services.append({'wall_elapsed':now-start,'service':key,'status':'UNDISCOVERED'});continue
    req=GetState.Request() if key.endswith('get_state') else GetParameters.Request(names=['use_sim_time','node_names'] if key.startswith('lifecycle_manager') else ['use_sim_time'])
    pending[key]=(c.call_async(req),now)
   nextsvc=now+10
  if now>=nextrow:
   clock=sec(data['clock'].clock) if 'clock' in data else None
   row={'wall_elapsed':now-start,'monotonic':now,'clock':clock,'ages':{},'receipt_ages':{k:now-v for k,v in last.items()},'tf':{k:[v[0],clock-v[0] if clock is not None else None,now-v[1]] for k,v in tfstamps.items()}}
   for k,m in data.items():
    s=sec(m.clock if k=='clock' else m.metadata.update_time if k in ('global','local') else m.header.stamp);row['ages'][k]=clock-s if clock is not None else None
    if k in ('global','local'):row[k+'_header_stamp']=sec(m.header.stamp)
   try:
    tf=b.lookup_transform('map','base_link',rclpy.time.Time());row['robot']=[tf.transform.translation.x,tf.transform.translation.y];row['composed_tf_age']=clock-sec(tf.header.stamp)
    truth=data['truth'].pose.pose;row['truth']=[truth.position.x,truth.position.y];row['error_m']=math.dist(row['robot'],row['truth'])
   except Exception as ex:row['tf_error']=str(ex)
   rows.append(row);print(json.dumps(row),flush=True);nextrow=now+5
finally:
 Path(a.output).write_text(json.dumps({'duration':time.monotonic()-start,'ranges':ranges,'services':services,'samples':rows},indent=2)+'\n');n.destroy_node();rclpy.shutdown()
