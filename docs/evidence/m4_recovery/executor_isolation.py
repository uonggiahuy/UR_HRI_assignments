"""Stationary comparison of the old full-grid guard and constant-time guard."""
import json,time
import rclpy
from turtlebot4.navigation_diagnostics import (NavigationProbe,wait_for_readiness,
                                               validate_costmap,validate_costmap_freshness)
from turtlebot4.localization_runner import resolve_saved_map_path
rclpy.init();n=NavigationProbe();results=[]
try:
 wait_for_readiness(n,resolve_saved_map_path(),20,navigation=True)
 for guard in (validate_costmap,validate_costmap_freshness):
  end=time.monotonic()+3;count=0
  while time.monotonic()<end:
   rclpy.spin_once(n,timeout_sec=.01)
   guard(n.messages['global'],n.clock_samples[-1]);count+=1
  tf=n.buffer.lookup_transform('map','base_link',rclpy.time.Time())
  stamp=tf.header.stamp.sec+tf.header.stamp.nanosec/1e9
  results.append({'guard':guard.__name__,'callbacks':count,'clock':n.clock_samples[-1],
                  'tf_stamp':stamp,'tf_age':n.clock_samples[-1]-stamp})
 print(json.dumps(results,indent=2))
finally:n.destroy_node();rclpy.shutdown()
