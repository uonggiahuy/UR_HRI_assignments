#!/usr/bin/env bash
# Keep MediaPipe separate from ROS system Python and the Assignment 5 YOLO venv.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
runtime="${XDG_CACHE_HOME:-$HOME/.cache}/turtlebot4/gesture-venv"
python3 -m venv --system-site-packages "$runtime"
"$runtime/bin/python" -m pip install -r "$script_dir/../config/gesture-requirements.txt"
set +u
source /opt/ros/humble/setup.bash
set -u
"$runtime/bin/python" - <<'PY'
import hashlib
import os
from pathlib import Path
import urllib.request
import cv2
import mediapipe as mp
import numpy
import rclpy
import cv_bridge

cache = Path(os.environ.get('XDG_CACHE_HOME', Path.home()/'.cache'))/'turtlebot4/models'
cache.mkdir(parents=True, exist_ok=True)
target = cache/'gesture_recognizer.task'
# Versioned official float16 model, never a mutable "latest" URL.
url = 'https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task'
expected = '97952348cf6a6a4915c2ea1496b4b37ebabc50cbbf80571435643c455f2b0482'
if not target.is_file() or hashlib.sha256(target.read_bytes()).hexdigest() != expected:
    with urllib.request.urlopen(url, timeout=60) as response:
        data = response.read(32*1024*1024)
    if hashlib.sha256(data).hexdigest() != expected:
        raise RuntimeError('Official model checksum mismatch')
    temporary = target.with_suffix('.download')
    temporary.write_bytes(data)
    temporary.replace(target)
options = mp.tasks.vision.GestureRecognizerOptions(
    base_options=mp.tasks.BaseOptions(model_asset_path=str(target)),
    running_mode=mp.tasks.vision.RunningMode.VIDEO)
with mp.tasks.vision.GestureRecognizer.create_from_options(options):
    print('Official model loaded:', target)
print('MediaPipe', mp.__version__, 'NumPy', numpy.__version__, 'OpenCV', cv2.__version__)
print('Model SHA256:', expected)
print('ROS and cv_bridge imports PASS. Live webcam acceptance still requires live frames.')
PY
