// Exercise the callback lock inversion in the loaded native TF2 library.
#include <tf2/buffer_core.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>

int main()
{
  tf2::BufferCore buffer;
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.transform.rotation.w = 1.0;
  transform.header.stamp.sec = 1;
  buffer.setTransform(transform, "regression");
  bool completed = false;
  buffer.addTransformableRequest(
    [&](auto, const auto &, const auto &, auto, auto) {
      // A callback must be able to queue a new request without taking a mutex
      // still held by testTransformableRequests (broken in TF2 0.25.23).
      buffer.addTransformableRequest(
        [](auto, const auto &, const auto &, auto, auto) {},
        "odom", "base_link", tf2::TimePoint(std::chrono::seconds(3)));
      completed = true;
    }, "odom", "base_link", tf2::TimePoint(std::chrono::seconds(2)));
  transform.header.stamp.sec = 2;
  buffer.setTransform(transform, "regression");
  if (!completed) {return 1;}
  std::cout << "TF2 callback reentry completed\n";
  return 0;
}
