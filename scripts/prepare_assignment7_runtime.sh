#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
runtime="${XDG_CACHE_HOME:-$HOME/.cache}/turtlebot4/speech-venv"
python3 -c 'import sys; assert sys.version_info[:2] == (3,10), "Use ROS Humble Python 3.10"'
python3 -m venv --system-site-packages "$runtime"
"$runtime/bin/python" -m pip install -r "$script_dir/../config/assignment7-requirements.txt"
# Keep the PortAudio shared library local when it is absent in the image.
native="$runtime/native/usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)"
if ! python3 -c 'import ctypes.util, sys; sys.exit(0 if ctypes.util.find_library("portaudio") else 1)' && [ ! -f "$native/libportaudio.so.2" ]; then
  mkdir -p "$runtime/native"
  (
    cd "$runtime/native"
    apt-get download libportaudio2=19.6.0-1.1
    dpkg-deb -x libportaudio2_19.6.0-1.1_*.deb .
  )
  native="$runtime/native/usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)"
  ln -sf libportaudio.so.2 "$native/libportaudio.so"
fi
native="$runtime/native/usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)"
export LIBRARY_PATH="$native${LIBRARY_PATH:+:$LIBRARY_PATH}"
export LD_LIBRARY_PATH="$native${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
set +u
source /opt/ros/humble/setup.bash
set -u
"$runtime/bin/python" - <<'PY'
import rclpy
import numpy
import faster_whisper
import sounddevice
print('ROS import PASS; faster-whisper', faster_whisper.__version__, 'NumPy', numpy.__version__)
print('Audio inputs:', [d for d in sounddevice.query_devices() if d['max_input_channels'] > 0])
print('Empty input list means live microphone acceptance is BLOCKED.')
PY
