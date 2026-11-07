"""Synthetic audio tests; live terminal/microphone acceptance is separate."""

import threading
from types import SimpleNamespace

import numpy as np
import pytest

from turtlebot4.manual_recording import ManualRecorder


class Stream:
    samplerate = 16000
    active = False

    def __init__(self, callback):
        self.callback = callback
        self.stopped = self.closed = False

    def start(self):
        self.active = True

    def stop(self):
        self.active = False
        self.stopped = True

    def close(self):
        self.closed = True

    def feed(self, value=0.01, count=320):
        self.callback(np.full((count, 1), value, np.float32), count, None, None)


@pytest.fixture
def rig():
    now = [0.0]
    streams, inputs, published, logs = [], [], [], []

    def factory(callback):
        streams.append(Stream(callback))
        return streams[-1]

    def transcribe(audio):
        inputs.append(audio.copy())
        return 'Đi đến khu vực F'

    recorder = ManualRecorder(factory, transcribe, published.append,
                              report=logs.append, clock=lambda: now[0])
    yield SimpleNamespace(r=recorder, now=now, streams=streams, inputs=inputs,
                          published=published, logs=logs)
    recorder.close()


def stop(rig):
    rig.now[0] += 1
    rig.r.enter()
    if rig.r.worker:
        rig.r.worker.join(timeout=2)


def test_initial_waiting_no_microphone(rig):
    assert rig.r.state == 'WAITING'
    assert rig.streams == [] and rig.inputs == []


def test_enter_capture_stop_infer_and_once(rig):
    rig.r.enter()
    assert rig.r.state == 'RECORDING'
    stream = rig.streams[-1]
    stream.feed(count=640)
    assert not rig.inputs and not rig.published
    stop(rig)
    assert rig.r.state == 'TRANSCRIBING'
    assert stream.stopped and stream.closed
    assert len(rig.inputs) == 1 and rig.inputs[0].shape == (640,)
    assert rig.inputs[0].dtype == np.float32
    assert not rig.published
    rig.r.poll()
    assert rig.r.state == 'WAITING'
    for _ in range(3):
        rig.r.poll()
    assert rig.published == ['Đi đến khu vực F']
    assert any('samples=640' in line and 'RMS=' in line for line in rig.logs)


def test_consecutive_sessions_clear_audio(rig):
    for count in [320, 960, 640]:
        rig.now[0] += 1
        rig.r.enter()
        rig.streams[-1].feed(count=count)
        stop(rig)
        rig.r.poll()
        assert rig.r.frames == []
    assert [len(a) for a in rig.inputs] == [320, 960, 640]
    assert len(rig.published) == 3


@pytest.mark.parametrize('silent', [True, False])
def test_empty_or_silent(rig, silent):
    rig.r.enter()
    if silent:
        rig.streams[-1].feed(value=0)
    stop(rig)
    assert rig.r.state == 'WAITING'
    assert not rig.inputs and not rig.published
    assert any('Empty or silent' in s for s in rig.logs)


def test_timeout_discards_whole_recording(rig):
    rig.r.enter()
    rig.streams[-1].feed()
    rig.now[0] = 31
    rig.r.poll()
    assert rig.r.state == 'WAITING' and rig.streams[-1].closed
    assert not rig.inputs and rig.r.frames == []
    assert any('Maximum recording duration' in s for s in rig.logs)


def test_callback_memory_bound(rig):
    rig.r.max_duration = 0.5
    rig.r.enter()
    for _ in range(100):
        rig.streams[-1].feed(count=1000)
    assert rig.r.samples <= 8000
    rig.r.poll()
    assert rig.r.state == 'WAITING' and not rig.inputs


def test_microphone_failure_and_retry(rig):
    factory = rig.r.stream_factory

    def broken(callback):
        raise OSError('device unplugged')

    rig.r.stream_factory = broken
    rig.r.enter()
    assert rig.r.state == 'WAITING'
    assert any('Microphone failure' in s for s in rig.logs)
    rig.r.stream_factory = factory
    rig.now[0] = 1
    rig.r.enter()
    assert rig.r.state == 'RECORDING'


def test_stream_start_failure_releases(rig):
    def broken(callback):
        stream = Stream(callback)
        rig.streams.append(stream)
        stream.start = lambda: (_ for _ in ()).throw(OSError('start failed'))
        return stream

    rig.r.stream_factory = broken
    rig.r.enter()
    assert rig.streams[-1].closed and rig.r.state == 'WAITING'


def test_enter_during_inference_is_ignored(rig):
    release = threading.Event()
    original = rig.r.transcribe
    rig.r.transcribe = lambda audio: (release.wait(timeout=2), original(audio))[1]
    rig.r.enter()
    rig.streams[-1].feed()
    rig.now[0] = 1
    rig.r.enter()
    try:
        for _ in range(5):
            rig.now[0] += 1
            rig.r.enter()
        assert len(rig.streams) == 1 and rig.r.state == 'TRANSCRIBING'
    finally:
        release.set()
        rig.r.worker.join(timeout=3)
    rig.r.poll()
    assert len(rig.published) == 1 and rig.r.state == 'WAITING'


def test_shutdown_recording(rig):
    rig.r.enter()
    rig.streams[-1].feed()
    rig.r.close()
    rig.r.enter()
    rig.r.poll()
    assert rig.streams[-1].closed and not rig.published
    assert rig.r.frames == []


def test_shutdown_inference_never_publishes(rig):
    release = threading.Event()
    entered = threading.Event()

    def transcribe(audio):
        entered.set()
        release.wait(timeout=5)
        return 'Đi đến khu vực F'

    rig.r.transcribe = transcribe
    rig.r.enter()
    rig.streams[-1].feed()
    rig.now[0] = 1
    rig.r.enter()
    assert entered.wait(timeout=2)
    try:
        rig.r.close()
        assert rig.r.worker.is_alive()
        rig.r.poll()
        assert not rig.published
    finally:
        release.set()
        rig.r.worker.join(timeout=2)
    rig.r.poll()
    assert not rig.published


@pytest.mark.parametrize('result', ['', 'error'])
def test_inference_empty_or_failed_returns_idle(rig, result):
    def transcribe(audio):
        if result == 'error':
            raise RuntimeError('inference failed')
        return ''

    rig.r.transcribe = transcribe
    rig.r.enter()
    rig.streams[-1].feed()
    stop(rig)
    rig.r.poll()
    assert rig.r.state == 'WAITING' and not rig.published
    assert any('Failed:' in s or 'No speech recognized' in s for s in rig.logs)


@pytest.mark.parametrize('problem', ['status', 'dtype', 'nan', 'channels'])
def test_invalid_capture_rejected(rig, problem):
    rig.r.enter()
    audio = np.ones((320, 1), np.float32)
    if problem == 'dtype':
        audio = audio.astype(np.int16)
    if problem == 'nan':
        audio[:] = np.nan
    if problem == 'channels':
        audio = np.ones((320, 2), np.float32)
    rig.r.capture(audio, 320, None, 'overflow' if problem == 'status' else None)
    rig.r.poll()
    assert rig.r.state == 'WAITING' and not rig.inputs


def test_callback_timeout(rig):
    rig.r.enter()
    rig.now[0] = 4
    rig.r.poll()
    assert rig.r.state == 'WAITING' and rig.streams[-1].closed


def test_duplicate_enter_debounced(rig):
    rig.r.enter()
    rig.r.enter()
    assert rig.r.state == 'RECORDING' and len(rig.streams) == 1


@pytest.mark.parametrize('maximum', [0, -1, 301, float('inf'), float('nan')])
def test_invalid_maximum(maximum):
    with pytest.raises(ValueError):
        ManualRecorder(None, None, None, max_duration=maximum)


def test_terminal_canonical_enter_and_stale_flush(monkeypatch):
    import os
    import pty
    from turtlebot4.manual_recording import TerminalKeyboard

    master, slave = pty.openpty()
    os.set_blocking(slave, False)
    monkeypatch.setattr('turtlebot4.manual_recording.os.open', lambda *a: os.dup(slave))
    keyboard = TerminalKeyboard()
    try:
        assert not keyboard.enter()
        os.write(master, b'\n')
        assert keyboard.enter()
        os.write(master, b'stale\n')
        keyboard.discard()
        assert not keyboard.enter()
    finally:
        keyboard.close()
        os.close(master)
        os.close(slave)


def test_terminal_missing_fails_explicitly(monkeypatch):
    from turtlebot4.manual_recording import TerminalKeyboard

    def unavailable(*args):
        raise OSError('no controlling terminal')

    monkeypatch.setattr('turtlebot4.manual_recording.os.open', unavailable)
    with pytest.raises(RuntimeError, match='docker exec -it'):
        TerminalKeyboard()


def test_offline_launch_without_terminal_or_microphone():
    """Launch real ROS processes without a TTY; check readiness and clean shutdown."""
    import os
    import selectors
    import signal
    import subprocess
    import time

    process = subprocess.Popen(
        ['ros2', 'launch', 'turtlebot4', 'assignment7.launch.py',
         'test_mode:=offline', 'speech_enabled:=false'],
        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    output = b''
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    try:
        deadline = time.monotonic() + 15
        while b'"state": "IDLE"' not in output and time.monotonic() < deadline:
            for key, _ in selector.select(timeout=0.2):
                chunk = os.read(key.fd, 65536)
                output += chunk
                if not chunk:
                    raise AssertionError(output.decode())
        assert b'"state": "IDLE"' in output, output.decode()
        assert b'speech_recognition_node' not in output
        assert b'WAITING FOR INPUT' not in output
    finally:
        selector.close()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            remaining, _ = process.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate()
            raise
    assert process.returncode == 0, (output + remaining).decode()
