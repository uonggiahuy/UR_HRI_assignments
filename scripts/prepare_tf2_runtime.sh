#!/usr/bin/env bash
# Build unmodified upstream Humble TF2 outside the assignment and /opt/ros.
set -eo pipefail
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'Run this script inside the Ubuntu 22.04 / ROS 2 Humble environment.' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
set -u
readonly tf2_revision=404b7224d623d614f18fa9738dbf1716403d857e
readonly runtime_dir="${XDG_CACHE_HOME:-$HOME/.cache}/turtlebot4/tf2-0.25.24"
mkdir -p "$runtime_dir"
if [[ ! -d "$runtime_dir/source/.git" ]]; then
  timeout 120 git clone --depth 1 --branch 0.25.24 \
    https://github.com/ros2/geometry2.git "$runtime_dir/source"
fi
if [[ "$(git -C "$runtime_dir/source" rev-parse HEAD)" != "$tf2_revision" ]]; then
  echo 'Unexpected TF2 source revision; refusing to build.' >&2
  exit 1
fi
git -C "$runtime_dir/source" diff --exit-code HEAD --
git -C "$runtime_dir/source" diff --cached --exit-code
# No vendor code is patched. Build only tf2; installed tf2_ros/Nav2 remain intact.
timeout 180 colcon --log-base "$runtime_dir/log" build \
  --base-paths "$runtime_dir/source/tf2" --packages-select tf2 \
  --build-base "$runtime_dir/build" --install-base "$runtime_dir/install" \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo
printf '\nSource this dependency overlay after ROS and the assignment overlay:\nsource "%s/install/setup.bash"\n' "$runtime_dir"
