#!/usr/bin/env bash
# Install vision dependencies outside the ROS/system Python environment.
set -euo pipefail
runtime="${XDG_CACHE_HOME:-$HOME/.cache}/turtlebot4/yolo-venv"
python3 -m venv --system-site-packages "$runtime"
"$runtime/bin/python" -m pip install 'torch==2.5.1+cpu' 'torchvision==0.20.1+cpu' \
  --index-url https://download.pytorch.org/whl/cpu
"$runtime/bin/python" -m pip install 'numpy==1.26.4' 'opencv-python==4.10.0.84' \
  'ultralytics==8.3.40'
set +u  # Humble's generated setup scripts reference optional unset variables.
source /opt/ros/humble/setup.bash
set -u
"$runtime/bin/python" - <<'PY'
from pathlib import Path
import os
import cv_bridge
import rclpy
from ultralytics import YOLO
cache = Path(os.environ.get('XDG_CACHE_HOME', Path.home() / '.cache')) / 'turtlebot4/models'
cache.mkdir(parents=True, exist_ok=True)
YOLO(str(cache / 'yolo11n.pt'))
print('YOLO runtime ready; ROS and cv_bridge import successfully')
PY
