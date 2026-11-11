#!/usr/bin/env bash
# Isolated CPU FER runtime; preserve ROS, Whisper and MediaPipe environments.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
runtime="${XDG_CACHE_HOME:-$HOME/.cache}/turtlebot4/emotion-venv"
[[ "$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')" == 3.10 ]]
[[ -r /opt/ros/humble/setup.bash ]]
set +u
source /opt/ros/humble/setup.bash
set -u
[[ "$ROS_DISTRO" == humble ]]
python3 -m venv --system-site-packages "$runtime"
"$runtime/bin/python" -m pip install -r "$script_dir/../config/assignment8-requirements.txt"
"$runtime/bin/python" -m pip install --no-deps fer==22.5.1
CUDA_VISIBLE_DEVICES=-1 TF_NUM_INTRAOP_THREADS=2 TF_NUM_INTEROP_THREADS=1 "$runtime/bin/python" - <<'PY'
import os
import time
import cv2
import numpy as np
import rclpy
import cv_bridge
from fer import FER
model = FER(mtcnn=False)
scores = np.asarray(model._classify_emotions(np.zeros((1, 64, 64, 1), dtype=np.float32)))
assert scores.shape == (1, 7) and np.isfinite(scores).all() and abs(scores.sum()-1) < .01
camera = os.environ.get('ASSIGNMENT8_CAMERA_DEVICE', '/dev/video0')
cap = cv2.VideoCapture(camera, cv2.CAP_V4L2)
try:
    if not cap.isOpened():
        raise RuntimeError('Cannot open real webcam '+camera)
    start = time.monotonic()
    for _ in range(10):
        ok, frame = cap.read()
        if not ok or frame is None:
            raise RuntimeError('Real webcam frame acquisition failed')
    print('PASS: real webcam', camera, frame.shape, 'capture FPS', 10/(time.monotonic()-start))
    start = time.monotonic()
    print('Actual FER webcam output:', model.detect_emotions(frame))
    print('Inference ms:', 1000*(time.monotonic()-start))
finally:
    cap.release()
print('PASS: CPU FER inference, OpenCV, ROS and cv_bridge; NumPy', np.__version__)
PY
