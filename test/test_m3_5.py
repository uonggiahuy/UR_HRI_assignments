"""Map semantics regression tests, including installed Nav2 executables.

Integration uses private namespaces, real lifecycle transitions and retained
OccupancyGrid messages. No simulator, fake map or upstream modification needed.
"""
from collections import Counter
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import uuid

from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.node import Node
import yaml

from turtlebot4 import map_saver as saver
from turtlebot4.localization_diagnostics import validate_map_compatibility
from turtlebot4.localization_runner import resolve_saved_map_path

ROOT = Path(__file__).resolve().parents[1]
EXPECTED = Counter({0: 80090, 100: 1938, -1: 137449})
PGM_SHA = '9bdc7c7095f17db56a5112e9c0bae857d9d8f29e9db757e9b4313410761b2dbc'


def map_pair(directory, free=0.196):
    directory.mkdir(parents=True, exist_ok=True)
    pgm = directory / 'map.pgm'
    shutil.copy2(ROOT / 'maps/map.pgm', pgm)
    meta = yaml.safe_load((ROOT / 'maps/map.yaml').read_text())
    meta['free_thresh'] = free
    path = directory / 'map.yaml'
    path.write_text(yaml.safe_dump(meta))
    return path, pgm


@pytest.mark.parametrize('pixel,expected', [(254, 0), (0, 100), (205, -1)])
def test_canonical_pixel_classification(pixel, expected):
    assert saver.classify_pgm_pixel(pixel, saver.DEFAULT_FREE_THRESH, saver.DEFAULT_OCC_THRESH) == expected


def test_real_pgm_preserved_and_yaml_classification():
    path = ROOT / 'maps/map.pgm'
    assert hashlib.sha256(path.read_bytes()).hexdigest() == PGM_SHA
    _, _, _, offset = saver.parse_pgm_header(path)
    assert Counter(path.read_bytes()[offset:]) == Counter({254: 80090, 0: 1938, 205: 137449})
    meta = saver.validate_saved_map_files(ROOT / 'maps/map.yaml', path)
    assert meta['free_cells'] == EXPECTED[0]
    assert meta['occupied_cells'] == EXPECTED[100]
    assert meta['unknown_cells'] == EXPECTED[-1]
    assert 1 / 255 < meta['free_thresh'] < 50 / 255 < meta['occupied_thresh'] < 1


@pytest.mark.parametrize('free,occupied', [(0.25, 0.65), (0.196, 0.1),
                                          (float('nan'), 0.65), (0.196, float('inf'))])
def test_unsafe_yaml_rejected_before_localization(tmp_path, free, occupied):
    path, pgm = map_pair(tmp_path, free)
    meta = yaml.safe_load(path.read_text())
    meta['occupied_thresh'] = occupied
    path.write_text(yaml.safe_dump(meta))
    with pytest.raises(ValueError):
        saver.validate_saved_map_files(path, pgm)
    with pytest.raises(ValueError):
        resolve_saved_map_path(path)


def test_unsafe_save_options_rejected_before_capture(tmp_path, monkeypatch):
    def unexpected_capture(**kwargs):
        pytest.fail('Unsafe thresholds reached ROS capture')
    monkeypatch.setattr(saver, 'capture_live_map', unexpected_capture)
    with pytest.raises(ValueError, match='preserve'):
        saver.save_map_atomic(tmp_path, free_thresh=0.25)



def test_saver_timeout_preserves_previous_map_and_removes_staging(tmp_path, monkeypatch):
    target = tmp_path / 'target'
    map_pair(target)
    before = {p.name: p.read_bytes() for p in target.iterdir()}
    staging = tmp_path / 'temporary'
    staging.mkdir()
    msg = OccupancyGrid()
    msg.info.resolution = 0.05
    msg.info.width, msg.info.height = 3, 1
    msg.data = [0, 100, -1]
    monkeypatch.setattr(saver, 'capture_live_map', lambda **kwargs: msg)
    monkeypatch.setattr(saver.tempfile, 'mkdtemp', lambda **kwargs: str(staging))
    def timeout(*args, **kwargs):
        raise subprocess.TimeoutExpired(args[0], 1)
    monkeypatch.setattr(saver.subprocess, 'run', timeout)
    with pytest.raises(subprocess.TimeoutExpired):
        saver.save_map_atomic(target, timeout=1)
    assert not staging.exists()
    assert {p.name: p.read_bytes() for p in target.iterdir()} == before


def test_diagnostics_reject_reclassified_cells(tmp_path):
    path, pgm = map_pair(tmp_path)
    meta = saver.validate_saved_map_files(path, pgm)
    grid = OccupancyGrid()
    grid.info.width, grid.info.height = meta['width'], meta['height']
    grid.info.resolution = meta['resolution']
    grid.info.origin.position.x, grid.info.origin.position.y = meta['origin'][:2]
    grid.data = [0] * (grid.info.width * grid.info.height)
    with pytest.raises(ValueError, match='free_cells'):
        validate_map_compatibility(grid, meta)


@pytest.mark.parametrize('failure', ['staging', 'second_replace', 'post_validation'])
@pytest.mark.parametrize('existing', [True, False])
def test_transaction_rollback_at_each_boundary(tmp_path, monkeypatch, failure, existing):
    source_yaml, source_pgm = map_pair(tmp_path / 'source')
    target = tmp_path / 'target'
    target.mkdir()
    dest_yaml, dest_pgm = target / 'map.yaml', target / 'map.pgm'
    if existing:
        map_pair(target)
    before = {p.name: p.read_bytes() for p in target.iterdir()}
    original_copy, original_replace = shutil.copy2, os.replace
    original_validate = saver.validate_saved_map_files

    def copy(src, dst):
        if failure == 'staging' and Path(dst).name == '.map.pgm.tmp':
            raise OSError('injected staging failure')
        return original_copy(src, dst)

    def replace(src, dst):
        # Fail once during commit; restoration remains possible.
        if failure == 'second_replace' and Path(src).name == '.map.pgm.tmp':
            raise OSError('injected commit failure')
        return original_replace(src, dst)

    def validate(yaml_path, pgm_path):
        if failure == 'post_validation' and yaml_path == dest_yaml:
            raise ValueError('injected post-validation failure')
        return original_validate(yaml_path, pgm_path)

    monkeypatch.setattr(saver.shutil, 'copy2', copy)
    monkeypatch.setattr(saver.os, 'replace', replace)
    monkeypatch.setattr(saver, 'validate_saved_map_files', validate)
    with pytest.raises(RuntimeError, match='rolled back'):
        saver.transactional_map_replace(source_yaml, source_pgm, dest_yaml, dest_pgm)
    assert {p.name: p.read_bytes() for p in target.iterdir()} == before


@contextmanager
def actual_map_server(yaml_path, log_path):
    namespace = '/m35_' + uuid.uuid4().hex[:10]
    initialized_here = not rclpy.ok()
    if initialized_here:
        rclpy.init()
    node = Node('map_correctness_test', namespace=namespace)
    received = []
    node.create_subscription(OccupancyGrid, namespace + '/map', received.append, saver.map_qos())
    with log_path.open('w') as log:
        process = subprocess.Popen([
            'ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args',
            '-r', '__ns:=' + namespace, '-p', 'yaml_filename:=' + str(yaml_path),
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            client = node.create_client(ChangeState, namespace + '/map_server/change_state')
            assert client.wait_for_service(timeout_sec=15), log_path.read_text()
            for transition in (1, 3):
                request = ChangeState.Request()
                request.transition.id = transition
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=15)
                assert future.done() and future.result().success, log_path.read_text()
            state_client = node.create_client(GetState, namespace + '/map_server/get_state')
            assert state_client.wait_for_service(timeout_sec=5)
            future = state_client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            assert future.result().current_state.id == 3
            deadline = time.monotonic() + 15
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            assert received, log_path.read_text()
            yield received[0], namespace + '/map'
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            node.destroy_node()
            if initialized_here:
                rclpy.shutdown()


def test_actual_nav2_before_and_after(tmp_path):
    faulty, _ = map_pair(tmp_path / 'faulty', 0.25)
    with actual_map_server(faulty, tmp_path / 'before.log') as (grid, _):
        assert Counter(grid.data) == Counter({0: 217539, 100: 1938})
        print('NAV2_BEFORE ' + json.dumps(dict(Counter(grid.data)), sort_keys=True))
    with actual_map_server(ROOT / 'maps/map.yaml', tmp_path / 'after.log') as (grid, _):
        assert Counter(grid.data) == EXPECTED
        print('NAV2_AFTER ' + json.dumps(dict(Counter(grid.data)), sort_keys=True))
        meta = saver.validate_saved_map_files(ROOT / 'maps/map.yaml', ROOT / 'maps/map.pgm')
        assert validate_map_compatibility(grid, meta)['unknown_cells'] == 137449
        # Counts alone cannot detect swapped cell positions.
        changed = list(grid.data)
        a, b = changed.index(0), changed.index(-1)
        changed[a], changed[b] = changed[b], changed[a]
        grid.data = changed
        with pytest.raises(ValueError, match='layout'):
            validate_map_compatibility(grid, meta)


def test_actual_nav2_saver_and_reload_real_warehouse(tmp_path):
    with actual_map_server(ROOT / 'maps/map.yaml', tmp_path / 'source.log') as (original, topic):
        info = saver.save_map_atomic(tmp_path / 'saved', map_topic=topic, timeout=20)
        assert info['unknown_cells'] == 137449
        assert yaml.safe_load(Path(info['target_yaml']).read_text())['free_thresh'] == 0.196
        with actual_map_server(Path(info['target_yaml']), tmp_path / 'reload.log') as (reloaded, _):
            assert Counter(reloaded.data) == EXPECTED
            assert list(reloaded.data) == list(original.data)
            print('NAV2_ROUNDTRIP ' + json.dumps(dict(Counter(reloaded.data)), sort_keys=True))
            assert reloaded.info.width == original.info.width
            assert reloaded.info.height == original.info.height
            assert reloaded.info.origin == original.info.origin
