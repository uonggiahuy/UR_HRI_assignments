"""Automated offline tests for Milestone M3.

Covers AMCL localization runner pre-checks, argument parsing, upstream launch
command construction, map resolution and transactional updates, pose and map
diagnostics validation, and RViz localization resource configuration.
No active simulator or ROS daemon is required.
"""

import argparse
from pathlib import Path
import tempfile
from unittest.mock import patch

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import MapMetaData, OccupancyGrid
import pytest
import yaml

from turtlebot4 import localization_diagnostics as diag
from turtlebot4 import localization_runner as runner
from turtlebot4 import map_saver
from turtlebot4 import simulation_launch as sim


# ---------------------------------------------------------------------------
# Runner argument parsing and validation
# ---------------------------------------------------------------------------

def test_localization_runner_timeout_validation():
    with pytest.raises(argparse.ArgumentTypeError):
        runner.bounded_timeout('0')
    with pytest.raises(argparse.ArgumentTypeError):
        runner.bounded_timeout('-5')
    with pytest.raises(argparse.ArgumentTypeError):
        runner.bounded_timeout('nan')
    with pytest.raises(argparse.ArgumentTypeError):
        runner.bounded_timeout('inf')
    with pytest.raises(argparse.ArgumentTypeError):
        runner.bounded_timeout('150')
    assert runner.bounded_timeout('15.5') == 15.5
    assert runner.bounded_timeout('120') == 120.0


def test_localization_runner_rejects_active_slam():
    with pytest.raises(RuntimeError, match='Active SLAM'):
        runner.check_no_active_slam(['/robot_state_publisher', '/slam_toolbox'])
    with pytest.raises(RuntimeError, match='Active SLAM'):
        runner.check_no_active_slam(['/sync_slam_toolbox_node'])
    with pytest.raises(RuntimeError, match='Active SLAM'):
        runner.check_no_active_slam(['/tb4/async_slam_toolbox_node'])
    assert runner.check_no_active_slam(['/robot_state_publisher', '/diffdrive_controller'])


def test_localization_runner_rejects_duplicate_localization():
    with pytest.raises(RuntimeError, match='Duplicate localization'):
        runner.check_no_duplicate_localization(['/amcl', '/robot_state_publisher'])
    with pytest.raises(RuntimeError, match='Duplicate localization'):
        runner.check_no_duplicate_localization(['/map_server'])
    with pytest.raises(RuntimeError, match='Duplicate localization'):
        runner.check_no_duplicate_localization(['/amcl_node'])
    assert runner.check_no_duplicate_localization(['/robot_state_publisher', '/scan'])


def test_resolve_saved_map_path_valid_and_missing(tmp_path):
    # Valid map YAML and PGM
    valid_pgm = tmp_path / 'test_map.pgm'
    valid_pgm.write_bytes(b"P5\n2 2\n255\n\x00\x00\x00\x00")
    valid_yaml = tmp_path / 'test_map.yaml'
    valid_yaml.write_text(yaml.safe_dump({
        'image': 'test_map.pgm',
        'resolution': 0.05,
        'origin': [0.0, 0.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }))

    resolved = runner.resolve_saved_map_path(str(valid_yaml))
    assert resolved == valid_yaml.resolve()

    # Missing file
    missing_yaml = tmp_path / 'nonexistent.yaml'
    with pytest.raises(FileNotFoundError, match='does not exist'):
        runner.resolve_saved_map_path(str(missing_yaml))

    # Invalid YAML content
    bad_yaml = tmp_path / 'bad.yaml'
    bad_yaml.write_text("invalid: yaml: content: [")
    with pytest.raises(ValueError, match='Invalid map YAML'):
        runner.resolve_saved_map_path(str(bad_yaml))


def test_build_localization_launch_cmd(tmp_path):
    map_path = Path('/root/ros2_ws/src/turtlebot4/maps/map.yaml')
    cmd = runner.build_localization_launch_cmd(
        map_path=map_path,
        use_sim_time=True,
        namespace='',
    )
    assert cmd[:4] == ['ros2', 'launch', 'turtlebot4_navigation', 'localization.launch.py']
    assert f'map:={map_path}' in cmd
    assert 'use_sim_time:=true' in cmd
    assert not any(arg.startswith('namespace:=') for arg in cmd)

    custom_params = tmp_path / 'custom_params.yaml'
    custom_params.write_text("amcl:\n  ros__parameters:\n    alpha1: 0.2\n")

    cmd_ns = runner.build_localization_launch_cmd(
        map_path=map_path,
        params_path=custom_params,
        use_sim_time=True,
        namespace='tb4',
    )
    assert 'namespace:=tb4' in cmd_ns
    assert f'params:={custom_params}' in cmd_ns


def test_descendant_pids_resolution():
    parents = {100: 1, 101: 100, 102: 101, 200: 1}
    assert set(sim.descendant_pids(100, parents)) == {101, 102}
    assert sim.descendant_pids(999, parents) == []


# ---------------------------------------------------------------------------
# Diagnostics pure logic: pose and map metadata validation
# ---------------------------------------------------------------------------

def test_validate_amcl_pose_msg_valid():
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = -2.23
    msg.pose.pose.position.y = -1.51
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.z = 0.9537
    msg.pose.pose.orientation.w = 0.3007
    # 6x6 covariance matrix (36 elements)
    cov = [0.0] * 36
    cov[0] = 0.05   # x var
    cov[7] = 0.05   # y var
    cov[35] = 0.02  # yaw var
    msg.pose.covariance = cov

    info = diag.validate_amcl_pose_msg(msg)
    assert info['frame_id'] == 'map'
    assert info['position'] == [-2.23, -1.51, 0.0]
    assert info['variance']['x'] == 0.05
    assert info['variance']['y'] == 0.05
    assert info['variance']['yaw'] == 0.02


def test_validate_amcl_pose_msg_unnormalized_and_invalid():
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'

    # Zero quaternion
    msg.pose.pose.orientation.w = 0.0
    with pytest.raises(ValueError, match='not normalized'):
        diag.validate_amcl_pose_msg(msg)

    # Non-finite position
    msg.pose.pose.orientation.w = 1.0
    msg.pose.pose.position.x = float('nan')
    with pytest.raises(ValueError, match='non-finite'):
        diag.validate_amcl_pose_msg(msg)

    # Negative covariance variance
    msg.pose.pose.position.x = 0.0
    cov = [0.0] * 36
    cov[0] = -0.5
    msg.pose.covariance = cov
    with pytest.raises(ValueError, match='negative variances'):
        diag.validate_amcl_pose_msg(msg)


def test_validate_map_compatibility_matching_and_mismatch():
    expected_yaml = {
        'resolution': 0.05,
        'width': 491,
        'height': 447,
        'origin': [-13.4, -12.6, 0.0],
    }
    grid = OccupancyGrid()
    grid.info.resolution = 0.05
    grid.info.width = 491
    grid.info.height = 447
    grid.info.origin.position.x = -13.4
    grid.info.origin.position.y = -12.6
    grid.data = [0] * (491 * 447)

    result = diag.validate_map_compatibility(grid, expected_yaml)
    assert result['width'] == 491
    assert result['height'] == 447
    assert result['resolution'] == 0.05

    # Resolution mismatch
    grid.info.resolution = 0.10
    with pytest.raises(ValueError, match='resolution'):
        diag.validate_map_compatibility(grid, expected_yaml)

    # Dimension mismatch
    grid.info.resolution = 0.05
    grid.info.width = 100
    with pytest.raises(ValueError, match='dimensions'):
        diag.validate_map_compatibility(grid, expected_yaml)

    # Origin mismatch
    grid.info.width = 491
    grid.info.origin.position.x = 0.0
    with pytest.raises(ValueError, match='origin'):
        diag.validate_map_compatibility(grid, expected_yaml)


# ---------------------------------------------------------------------------
# Map saver transactional update and rollback tests
# ---------------------------------------------------------------------------

def test_transactional_map_replace_success(tmp_path):
    dest_dir = tmp_path / 'maps'
    dest_dir.mkdir()
    dest_yaml = dest_dir / 'map.yaml'
    dest_pgm = dest_dir / 'map.pgm'

    # Initial valid map files
    dest_pgm.write_bytes(b"P5\n2 2\n255\n\x00\x00\x00\x00")
    dest_yaml.write_text(yaml.safe_dump({
        'image': 'map.pgm',
        'resolution': 0.05,
        'origin': [0.0, 0.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }))

    # Staged new map files with mutual consistency
    stage_dir = tmp_path / 'stage'
    stage_dir.mkdir()
    new_pgm = stage_dir / 'map.pgm'
    new_pgm.write_bytes(b"P5\n3 3\n255\n" + b"\xff" * 9)
    new_yaml = stage_dir / 'map.yaml'
    new_yaml.write_text(yaml.safe_dump({
        'image': 'map.pgm',
        'resolution': 0.05,
        'origin': [-1.0, -1.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }))

    meta = map_saver.transactional_map_replace(new_yaml, new_pgm, dest_yaml, dest_pgm)
    assert meta['width'] == 3
    assert meta['height'] == 3
    assert dest_pgm.read_bytes() == b"P5\n3 3\n255\n" + b"\xff" * 9
    assert yaml.safe_load(dest_yaml.read_text())['origin'] == [-1.0, -1.0, 0.0]
    # Verify no dangling backups or temp files
    assert not (dest_dir / 'map.yaml.bak').exists()
    assert not (dest_dir / 'map.pgm.bak').exists()


def test_transactional_map_replace_rollback_on_failure(tmp_path):
    dest_dir = tmp_path / 'maps'
    dest_dir.mkdir()
    dest_yaml = dest_dir / 'map.yaml'
    dest_pgm = dest_dir / 'map.pgm'

    original_pgm_bytes = b"P5\n2 2\n255\n\x00\x00\x00\x00"
    original_yaml_text = yaml.safe_dump({
        'image': 'map.pgm',
        'resolution': 0.05,
        'origin': [0.0, 0.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    })
    dest_pgm.write_bytes(original_pgm_bytes)
    dest_yaml.write_text(original_yaml_text)

    # Staged new map files
    stage_dir = tmp_path / 'stage'
    stage_dir.mkdir()
    new_pgm = stage_dir / 'map.pgm'
    new_pgm.write_bytes(b"P5\n3 3\n255\n" + b"\xff" * 9)
    new_yaml = stage_dir / 'map.yaml'
    new_yaml.write_text(yaml.safe_dump({
        'image': 'map.pgm',
        'resolution': 0.05,
        'origin': [-1.0, -1.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }))

    # Simulate an error during atomic replace of PGM
    original_replace = map_saver.os.replace

    def mock_replace(src, dst):
        if str(dst).endswith('map.pgm'):
            raise OSError("Simulated disk write failure")
        return original_replace(src, dst)

    with patch('turtlebot4.map_saver.os.replace', side_effect=mock_replace):
        with pytest.raises(OSError, match='Simulated disk write failure'):
            map_saver.transactional_map_replace(new_yaml, new_pgm, dest_yaml, dest_pgm)

    # Assert rollback restored the original contents exactly
    assert dest_pgm.read_bytes() == original_pgm_bytes
    assert dest_yaml.read_text() == original_yaml_text
    assert not (dest_dir / 'map.yaml.bak').exists()
    assert not (dest_dir / 'map.pgm.bak').exists()


# ---------------------------------------------------------------------------
# RViz simulation configuration verification
# ---------------------------------------------------------------------------

def test_rviz_simulation_config_has_amcl_displays_and_initial_pose_tool():
    path = Path(get_package_share_directory('turtlebot4')) / 'rviz/simulation.rviz'
    assert path.is_file()
    text = path.read_text()

    # Preserved M1/M2 baseline
    assert 'Fixed Frame: odom' in text
    assert 'rviz_default_plugins/RobotModel' in text
    assert 'rviz_default_plugins/LaserScan' in text
    assert 'rviz_default_plugins/TF' in text
    assert 'rviz_default_plugins/Map' in text

    # M3 AMCL visualization displays
    assert 'rviz_default_plugins/PoseWithCovariance' in text
    assert 'Value: amcl_pose' in text
    assert 'nav2_rviz_plugins/ParticleCloud' in text
    assert 'Value: particle_cloud' in text

    # M3 2D Pose Estimate tool
    assert 'rviz_default_plugins/SetInitialPose' in text
    assert 'Value: initialpose' in text
