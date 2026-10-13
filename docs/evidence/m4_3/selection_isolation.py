import importlib.util,json,time
from pathlib import Path
import rclpy
from turtlebot4.navigation_diagnostics import wait_for_readiness
from turtlebot4.localization_runner import resolve_saved_map_path
r=json.load(open('/tmp/m4_3/registration.json'));result={}
for name,path in [('original','/tmp/m4_3/verifier_before_selection_fix.py'),('bounded','/root/ros2_ws/src/turtlebot4/scripts/verify_navigation.py')]:
 spec=importlib.util.spec_from_file_location('verifier_'+name,path);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
 rclpy.init();n=mod.GoalEvidence(r)
 try:
  wait_for_readiness(n,resolve_saved_map_path(),25,navigation=True)
  before=n.snapshot();t=time.monotonic();goal,margin,plan=n.select_goal((-1.4,-.4));elapsed=time.monotonic()-t
  after=n.snapshot();result[name]={'selection_wall_sec':elapsed,'before_tf_age':before['tf_age'],'after_tf_age':after['tf_age'],'clock_receipt_age':time.monotonic()-n.received['clock'],'path_length':plan['length_m']}
  if name=='bounded':n.wait_current_observations();result[name]['after_drain_tf_age']=n.snapshot()['tf_age']
 finally:n.destroy_node();rclpy.shutdown()
Path('/tmp/m4_3/selection_isolation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
