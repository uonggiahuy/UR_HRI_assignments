# RViz resources

`simulation.rviz` displays the robot model, LiDAR scans, TF frames, and live OccupancyGrid map (`/map`).
The fixed frame is set to `odom` so that RViz operates cleanly both before SLAM starts and after SLAM begins publishing `map -> odom`.
