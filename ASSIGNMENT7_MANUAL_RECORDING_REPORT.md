# Assignment 7 — Manual recording verification

Date: 2026-11-07 (Asia/Bangkok). Overall acceptance: **PARTIAL**.

## Scope and diagnosis

Workspace `/root/ros2_ws`, host bind `/home/choconadyne/ros2_ws`;
actual container `ros_humble_env`, ROS 2 Humble, Python 3.10,
branch `assignments_7`, baseline `141d2d202370202bf3d7ca42c25697f2566114cd`.
Initial package checkout was clean. Source and historical M0/M1/upstream
reports and platform/debugging/demo skills were inspected before changes.
No running simulation/Nav2 graph was present. The initial RMW environment
variable was unset; no global DDS or discovery settings were changed.

The old runtime opened the input immediately and waited for amplitude >=0.012,
15 voiced frames and 35 quiet frames before inference. Overlong noise was
silently dropped. This explains how an accessed microphone could produce no
inference; it does not establish that every past failure had that cause.

Current PortAudio inputs are `pulse` (0) and `default` (1), using ALSA through
the existing host Pulse socket. Default source is the host analog input,
not a monitor; host mute was false and volume 58% when inspected.
The isolated runtime's native PortAudio loader paths are still required.
A direct venv Python invocation without those paths reported missing PortAudio;
the existing executable re-exec and setup script correctly supply them.
No system Python, audio routing, gain, vendor code or 9Router settings changed.

## Implementation and terminal ownership

- `turtlebot4/speech_recognition_node.py`: integrate manual controller, retain
  multilingual Vietnamese CPU INT8 and isolated runtime, publish completed
  results once. Legacy `UtteranceBuffer` remains as a compatibility API only.
- `turtlebot4/manual_recording.py`: WAITING -> RECORDING -> TRANSCRIBING ->
  WAITING; bounded samples and wall duration; input validation, failure recovery,
  diagnostic summaries and shutdown. Only STOP submits complete audio.
- `launch/assignment7.launch.py`, `config/assignment7.yaml`: expose manual mode,
  Vietnamese language, model and maximum recording duration (30 seconds).
- `tests/test_manual_recording.py`: 28 new tests, including real offline launch.
- `README.md`: five concise Vietnamese installation/operator sections.

Installed Humble `launch/actions/execute_local.py` and
`osrf_pycommon/process_utils/async_execute_process.py` were inspected:
child stdin is a pipe handled by launch events, not automatically forwarded
keyboard input. The speech component instead opens its inherited controlling
terminal `/dev/tty` with O_NONBLOCK and polls it every 50 ms using a ROS wall
timer. It retains canonical input and terminal signal handling, and never
blocks launch or the ROS executor. There is one terminal reader; navigation
remains independent. No additional executable, process or control topic is
needed. No controlling terminal produces an explicit error; use
`docker exec -it ros_humble_env bash`, not a detached/headless live launch.
Offline launch never constructs the keyboard or microphone component.

Enter during inference is consumed and discarded. Queued input is flushed
before returning to idle; repeated Enter within 250 ms is debounced.
Buffers reset for each session. Timeouts discard the whole recording rather
than submitting a partial command. Digital silence RMS <1e-5 is rejected;
there is no voiced-frame or end-silence requirement. Recoverable recording or
inference failures return to idle rather than interrupt an existing mission.
Speech process exit and launch shutdown retain the existing navigation cleanup.
The native inference call cannot be interrupted inside its daemon thread;
shutdown closes audio, prevents any late publication, waits at most one second
for the worker, then process exit releases inference resources. No detached
inference processes are created.

## Commands and automated validation

Run inside `ros_humble_env`:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
bash src/turtlebot4/scripts/prepare_assignment7_runtime.sh
source /root/.cache/turtlebot4/tf2-0.25.24/install/setup.bash
colcon build --symlink-install --packages-select turtlebot4
source install/setup.bash
colcon test --packages-select turtlebot4 --event-handlers console_direct+
colcon test-result --test-result-base build/turtlebot4 --verbose
```

The setup script passed; existing pinned dependencies were already satisfied.
The first test run without the existing TF2 overlay had 434 passes and one
native TF2 callback-reentry timeout. Sourcing the documented pinned 0.25.24
overlay fixed this environment issue; no upstream code was edited.
Final build/test results are recorded below after the model selection check.

## Genuine terminal and microphone evidence

The exact default command was run through `docker exec -it` in a PTY:

```bash
ros2 launch turtlebot4 assignment7.launch.py
```

Startup reached WAITING without opening capture. Newlines were written to that
same launch terminal, not sent through an alternate ROS topic or terminal.
First Enter started real PortAudio input; second Enter stopped/closed it and
only then invoked the actual faster-whisper model. Three consecutive recordings
completed without restarting launch. These were not synthetic WAVs or injected
transcripts. The user agreed to speak and was prompted during capture.

| Model | Samples | Audio duration | Peak | RMS | Inference | Actual result |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| base | 341760 | 21.360 s | 1.955540 | 0.413286 | 0.881 s | empty; no publication |
| base | 342720 | 21.420 s | 1.922573 | 0.391903 | 0.634 s | Đi đến khu mọi, Đi đến khu mọi, Đi đến khu mọi. |
| base | 280960 | 17.560 s | 2.024267 | 0.426411 | 0.571 s | Đi về đến khu phở et. |

All recordings were mono float32, 16000 Hz, blocksize 320. The two nonempty
transcripts were each published once and independently observed by the actual
voice navigation subscriber, which rejected them as unknown/invalid commands.
`ros2 topic info /recognized_speech --verbose` showed one speech publisher
and one navigation subscriber. The recognizer is responsive, but these base
transcripts did **not** satisfy accurate destination recognition. Above-nominal
sample magnitudes warrant checking source gain/noise/distortion; no specific
hardware defect was proved and no automatic gain change was made.

A fourth recording was interrupted with Ctrl+C while RECORDING: speech,
semantic-map and navigation processes all finished cleanly. A separate real
launch with `max_recording_duration:=2.0` captured 31360 samples (1.960 s),
reported peak 1.171695 / RMS 0.291450, rejected the whole timed-out utterance,
released capture and returned to READY without invoking Whisper.

No Nav2 action server or simulation was running. Therefore no robot movement,
voice-to-navigation success, or new simulator demo is claimed. Existing parser,
region, cancellation and navigation tests remain the regression evidence.

## Model selection follow-up

The user subsequently requested a stronger model. The original default was
multilingual `base` (74M parameters); multilingual `small` has 244M parameters.
References: [Whisper models](https://github.com/openai/whisper#available-models-and-languages),
[faster-whisper CPU INT8 usage](https://github.com/SYSTRAN/faster-whisper#usage).
The separate live small-model test and final default decision are recorded
below once measured. Larger model size alone is not proof of better accuracy
for this microphone and command vocabulary.

`ros2 launch turtlebot4 assignment7.launch.py whisper_model:=small` loaded the
actual multilingual model successfully. The same launch-terminal Enter cycle
captured **260800 samples, 16.300 seconds, peak 2.323683, RMS 0.309173**.
Whisper returned exactly **“Đi đến khu vực F”** in **1.644 seconds**,
with one segment and zero rejected low-confidence segments. It was published
once; the real voice navigation node reported `RECOGNIZED`, parsed `["F"]`,
and entered `VALIDATING`. No navigation success is claimed. Ctrl+C canceled
validation and all three processes finished cleanly.

At the user's explicit follow-up request, defaults in speech node, launch and
YAML were changed from `base` to **`small`**. Language `vi`, CPU INT8, pinned
runtime versions and manual recording remain unchanged. README reflects the
new default and documents `whisper_model:=base` as a lighter alternative.
This later instruction supersedes the original request to retain `base` as
the default. One correct small-model utterance is real acceptance evidence,
not a controlled accuracy benchmark or a promise for every voice/noise level.

Final **build PASS; 438 tests PASS, 0 errors/failures/skips** after model-default
changes (410 preserved tests plus 28 new tests). Coverage includes initial
WAITING, start/stop, no early inference, completion/idle, consecutive buffers,
empty/silent capture, wall timeout, sample-memory limit, microphone/start
failure, invalid format/overflow, callback loss, Enter during inference,
debounce, shutdown during capture and blocked inference, exactly-once
publication, terminal input/flush and no-TTY errors. The offline integration
test launches real Assignment 7 processes without any TTY and confirms IDLE,
absence of the speech process and clean SIGINT shutdown.

Original parser/regions/navigation/cancel tests and Assignment 4–6 regression
suites pass unchanged. No occupancy map, region, goal coordinate, Nav2 config,
localization source, UR package, setup.py or vendor dependency was modified.
`git diff --exit-code -- maps` and `git diff --check` passed.

Original map SHA256, unchanged:

- `map.yaml`: `b1be9283ae2c966e54ce6000133263b62fcf7daa2a84cce61c94b210622bf8c9`
- `map.pgm`: `915c88f67974aa4e160f2679cc17c538efceddc1e2ed2f9eb0cf05000b3a4955`

Implementation, Enter interaction, real microphone capture, small-model
transcription and ROS delivery: **PASS**. Full voice-to-robot navigation and
rerunning the earlier Gazebo/YOLO/gesture demonstrations: **MANUAL VERIFICATION
REQUIRED**. No robot or simulator was running during this change. Samples above
the nominal range remain an audio-quality observation to investigate if errors
recur. Raw recordings, logs, caches and virtual environments are not committed.

The final no-argument command was relaunched after rebuilding:
`ros2 launch turtlebot4 assignment7.launch.py`. It explicitly reported model
`small`, MANUAL mode and WAITING FOR INPUT, without automatic capture;
Ctrl+C again terminated all three processes cleanly.

## Git delivery

Implementation commit: **`a00919cd4e286b669f3266bcfa55e2b16d9d6b5f`**,
`fix(assignment7): record speech with terminal Enter and use Whisper small`.
The configured container SSH push failed with `Permission denied (publickey)`.
The already-authenticated host GitHub CLI credential helper was then used for
an HTTPS fast-forward push to the same repository/`assignments_7` branch,
without changing the configured remote or exporting any token. Push succeeded;
`git ls-remote origin refs/heads/assignments_7` independently verified that
remote SHA equals the implementation SHA above.
This report is delivered in a subsequent documentation commit; the final
remote report-commit SHA is reported in the final response. No force push,
branch switch, older assignment branch edit, or generated artifact commit
was performed.
