"""Automated offline tests for Milestone M2.

Covers launch composition, SLAM runner validation, teleoperation calculations,
map saver parsing and validation, failure rollback, and visualization resources.
No active simulator or ROS daemon is required.
"""

import argparse
import ast
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Point, Quaternion
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import MapMetaData, OccupancyGrid
import pytest
import yaml

from turtlebot4 import map_saver
from turtlebot4 import slam_runner
from turtlebot4 import teleop
from turtlebot4 import simulation_launch as sim


def test_persistent_base_launch_syntax_and_loadability():
    path = Path(get_package_share_directory('turtlebot4')) / 'launch/turtlebot4.launch.py'
    assert path.is_file()
    ast.parse(path.read_text())
    description = PythonLaunchDescriptionSource(str(path)).get_launch_description(LaunchContext())
    assert description.entities
    declarations = {item.name: item for item in description.entities if isinstance(item, DeclareLaunchArgument)}
    assert set(declarations) == {'world', 'model', 'namespace', 'rviz'}


def test_slam_runner_timeout_validation():
    with pytest.raises(argparse.ArgumentTypeError):
        slam_runner.bounded_timeout('0')
    with pytest.raises(argparse.ArgumentTypeError):
        slam_runner.bounded_timeout('-5')
    with pytest.raises(argparse.ArgumentTypeError):
        slam_runner.bounded_timeout('nan')
    with pytest.raises(argparse.ArgumentTypeError):
        slam_runner.bounded_timeout('inf')
    with pytest.raises(argparse.ArgumentTypeError):
        slam_runner.bounded_timeout('150')
    assert slam_runner.bounded_timeout('10.5') == 10.5


def test_slam_runner_rejects_duplicate_slam():
    with pytest.raises(RuntimeError, match='Duplicate SLAM'):
        slam_runner.check_no_duplicate_slam(['/robot_state_publisher', '/slam_toolbox'])
    with pytest.raises(RuntimeError, match='Duplicate SLAM'):
        slam_runner.check_no_duplicate_slam(['/sync_slam_toolbox_node'])
    with pytest.raises(RuntimeError, match='Duplicate SLAM'):
        slam_runner.check_no_duplicate_slam(['/tb4/async_slam_toolbox_node'])
    assert slam_runner.check_no_duplicate_slam(['/robot_state_publisher', '/diffdrive_controller'])


def test_slam_runner_rejects_conflicting_amcl_localization():
    with pytest.raises(RuntimeError, match='Incompatible stack'):
        slam_runner.check_no_conflicting_localization(['/amcl'])
    with pytest.raises(RuntimeError, match='Incompatible stack'):
        slam_runner.check_no_conflicting_localization(['/tb4/amcl_node'])
    assert slam_runner.check_no_conflicting_localization(['/robot_state_publisher', '/diffdrive_controller'])


def test_slam_runner_build_launch_cmd():
    cmd = slam_runner.build_slam_launch_cmd(namespace='', sync=True)
    assert 'ros2' in cmd[0]
    assert 'slam.launch.py' in cmd[3]
    assert 'use_sim_time:=true' in cmd
    assert 'sync:=true' in cmd
    assert not any(arg.startswith('namespace:=') for arg in cmd)

    cmd_async = slam_runner.build_slam_launch_cmd(namespace='robot1', sync=False)
    assert 'sync:=false' in cmd_async
    assert 'namespace:=robot1' in cmd_async

    with pytest.raises(ValueError):
        slam_runner.build_slam_launch_cmd(namespace='invalid//ns')


def test_teleop_twist_calculation():
    linear, angular = 0.25, 0.50

    # Linear motion
    assert teleop.calculate_twist('i', linear, angular) == (0.25, 0.0)
    assert teleop.calculate_twist(',', linear, angular) == (-0.25, 0.0)

    # Angular turning
    assert teleop.calculate_twist('j', linear, angular) == (0.0, 0.50)
    assert teleop.calculate_twist('l', linear, angular) == (0.0, -0.50)

    # Diagonal curves
    assert teleop.calculate_twist('u', linear, angular) == (0.25, 0.50)
    assert teleop.calculate_twist('o', linear, angular) == (0.25, -0.50)
    assert teleop.calculate_twist('m', linear, angular) == (-0.25, 0.50)
    assert teleop.calculate_twist('.', linear, angular) == (-0.25, -0.50)

    # Safe stop
    assert teleop.calculate_twist('k', linear, angular) == (0.0, 0.0)
    assert teleop.calculate_twist(' ', linear, angular) == (0.0, 0.0)
    assert teleop.calculate_twist('unknown', linear, angular) == (0.0, 0.0)


def test_teleop_speed_adjustment_and_clamping():
    linear, angular = 0.25, 0.50

    # Increase linear
    l_up, a_same = teleop.adjust_speed('w', linear, angular)
    assert l_up > linear and a_same == angular

    # Decrease linear
    l_down, a_same = teleop.adjust_speed('x', linear, angular)
    assert l_down < linear and a_same == angular

    # Increase angular
    l_same, a_up = teleop.adjust_speed('e', linear, angular)
    assert a_up > angular and l_same == linear

    # Decrease angular
    l_same, a_down = teleop.adjust_speed('c', linear, angular)
    assert a_down < angular and l_same == linear

    # Clamping limits
    assert teleop.clamp_speed(10.0, teleop.MIN_LINEAR, teleop.MAX_LINEAR) == teleop.MAX_LINEAR
    assert teleop.clamp_speed(0.001, teleop.MIN_LINEAR, teleop.MAX_LINEAR) == teleop.MIN_LINEAR
    assert teleop.clamp_speed(10.0, teleop.MIN_ANGULAR, teleop.MAX_ANGULAR) == teleop.MAX_ANGULAR
    assert teleop.clamp_speed(0.001, teleop.MIN_ANGULAR, teleop.MAX_ANGULAR) == teleop.MIN_ANGULAR


def test_teleop_cmd_vel_qos_matches_robot_subscriber():
    qos = teleop.cmd_vel_qos()
    assert qos.reliability.name == 'BEST_EFFORT'
    assert qos.durability.name == 'VOLATILE'
    assert qos.depth == 5


def make_valid_occupancy_grid():
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info = MapMetaData()
    msg.info.resolution = 0.05
    msg.info.width = 4
    msg.info.height = 4
    msg.info.origin = Pose(position=Point(x=-2.0, y=-2.0, z=0.0),
                           orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    # 4x4 = 16 cells: 8 free (0), 2 occupied (100), 6 unknown (-1)
    msg.data = [0, 0, 0, 0,
                0, 100, 100, 0,
                -1, -1, 0, 0,
                -1, -1, -1, -1]
    return msg


def test_occupancy_grid_validation_accepts_valid_grid():
    msg = make_valid_occupancy_grid()
    stats = map_saver.validate_occupancy_grid_msg(msg)
    assert stats['width'] == 4
    assert stats['height'] == 4
    assert stats['total_cells'] == 16
    assert stats['free_cells'] == 8
    assert stats['occupied_cells'] == 2
    assert stats['unknown_cells'] == 6
    assert stats['resolution'] == 0.05


def test_occupancy_grid_validation_rejects_empty_or_fabricated_grid():
    # 100% unknown cells
    msg = make_valid_occupancy_grid()
    msg.data = [-1] * 16
    with pytest.raises(ValueError, match='100% of cells are unknown'):
        map_saver.validate_occupancy_grid_msg(msg)

    # 0 free cells
    msg = make_valid_occupancy_grid()
    msg.data = [100] * 8 + [-1] * 8
    with pytest.raises(ValueError, match='0 free cells'):
        map_saver.validate_occupancy_grid_msg(msg)

    # Out of range cell values (< -1)
    msg = make_valid_occupancy_grid()
    msg.data[0] = -2
    with pytest.raises(ValueError, match='out-of-range'):
        map_saver.validate_occupancy_grid_msg(msg)

    # Mismatched dimensions
    msg = make_valid_occupancy_grid()
    msg.info.width = 5
    with pytest.raises(ValueError, match='does not match'):
        map_saver.validate_occupancy_grid_msg(msg)


def test_pgm_header_parsing(tmp_path):
    pgm_path = tmp_path / 'test.pgm'
    # P5 header with width=3, height=2, maxval=255, 6 bytes data
    content = b"P5\n# test comment\n3 2\n255\n\x00\x64\xc8\xff\x7f\x32"
    pgm_path.write_bytes(content)

    width, height, maxval, offset = map_saver.parse_pgm_header(pgm_path)
    assert width == 3
    assert height == 2
    assert maxval == 255
    assert offset < len(content)
    assert len(content) - offset == 6


def test_pgm_header_parsing_rejects_invalid_format(tmp_path):
    bad_pgm = tmp_path / 'bad.pgm'
    bad_pgm.write_bytes(b"P3\n3 2\n255\n0 1 2 3 4 5")
    with pytest.raises(ValueError, match='Invalid PGM magic number'):
        map_saver.parse_pgm_header(bad_pgm)


def test_saved_map_files_validation(tmp_path):
    pgm_file = tmp_path / 'map.pgm'
    pgm_file.write_bytes(b"P5\n2 2\n255\n\x00\x00\xff\xff")

    yaml_file = tmp_path / 'map.yaml'
    yaml_content = {
        'image': 'map.pgm',
        'resolution': 0.05,
        'origin': [-5.0, -5.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }
    with open(yaml_file, 'w', encoding='utf-8') as f:
        yaml.safe_dump(yaml_content, f)

    res = map_saver.validate_saved_map_files(yaml_file, pgm_file)
    assert res['resolution'] == 0.05
    assert res['width'] == 2
    assert res['height'] == 2
    assert res['image'] == 'map.pgm'


def test_saved_map_files_validation_rejects_missing_image(tmp_path):
    yaml_file = tmp_path / 'map.yaml'
    yaml_content = {
        'image': 'non_existent.pgm',
        'resolution': 0.05,
        'origin': [0.0, 0.0, 0.0],
    }
    with open(yaml_file, 'w', encoding='utf-8') as f:
        yaml.safe_dump(yaml_content, f)

    fake_pgm = tmp_path / 'other.pgm'
    fake_pgm.write_bytes(b"P5\n1 1\n255\n\x00")

    with pytest.raises(FileNotFoundError, match='does not exist'):
        map_saver.validate_saved_map_files(yaml_file, fake_pgm)


def test_rviz_simulation_config_has_all_required_displays():
    path = Path(get_package_share_directory('turtlebot4')) / 'rviz/simulation.rviz'
    assert path.is_file()
    text = path.read_text()
    assert 'Fixed Frame: odom' in text
    assert 'rviz_default_plugins/RobotModel' in text
    assert 'rviz_default_plugins/LaserScan' in text
    assert 'rviz_default_plugins/TF' in text
    assert 'rviz_default_plugins/Map' in text
    assert 'Value: map' in text
