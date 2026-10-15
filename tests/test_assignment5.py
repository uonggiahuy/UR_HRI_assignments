"""Offline checks for mission geometry, immutable maps and bounded image handling."""
from pathlib import Path
from types import SimpleNamespace as NS
import threading

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap
import pytest
import yaml

from turtlebot4.assignment5_scene import scene_models
from turtlebot4.waypoint_navigator import load_goals, moving_sample, validate_goal_cells, WaypointNavigator
from turtlebot4.yolo_detector import LatestFrame

ROOT = Path(__file__).resolve().parents[1]


def test_original_map_bound_route():
    config = load_goals(ROOT / 'config/goals.yaml', ROOT / 'maps/map.yaml')
    assert len(config['goals']) == 3


@pytest.mark.parametrize('change', ['frame', 'count', 'nan', 'duplicate', 'close', 'hash', 'clearance'])
def test_rejects_invalid_mission(tmp_path, change):
    config = yaml.safe_load((ROOT / 'config/goals.yaml').read_text())
    if change == 'frame': config['frame_id'] = 'odom'
    if change == 'count': config['goals'].pop()
    if change == 'nan': config['goals'][0]['x'] = float('nan')
    if change == 'duplicate': config['goals'][1]['name'] = config['goals'][0]['name']
    if change == 'close':
        config['goals'][1].update(x=config['goals'][0]['x'], y=config['goals'][0]['y'])
    if change == 'hash': config['map_sha256']['pgm'] = 'invalid'
    if change == 'clearance': config['minimum_clearance'] = .1
    path = tmp_path / 'goals.yaml'; path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError): load_goals(path, ROOT / 'maps/map.yaml')


def grids():
    grid = OccupancyGrid()
    grid.info.width = grid.info.height = 100
    grid.info.resolution = .05
    grid.info.origin.orientation.w = 1.
    grid.data = [0] * 10000
    grid.data[0] = 100
    costmap = Costmap()
    costmap.metadata.size_x = costmap.metadata.size_y = 100
    costmap.metadata.resolution = .05
    costmap.metadata.origin.orientation.w = 1.
    costmap.data = [0] * 10000
    config = dict(minimum_clearance=.55, goals=[dict(name=str(i), x=float(i), y=2.) for i in (1,2,3)])
    return config, grid, costmap


def test_all_goals_checked_against_live_map_and_costmap():
    config, grid, costmap = grids()
    assert min(validate_goal_cells(config, grid, costmap)) > .55


@pytest.mark.parametrize('value', [-1, 100])
def test_unknown_or_occupied_goal_rejected(value):
    config, grid, costmap = grids()
    grid.data[40 * 100 + 40] = value
    with pytest.raises(ValueError): validate_goal_cells(config, grid, costmap)


@pytest.mark.parametrize('value', [253, 254, 255])
def test_inscribed_lethal_or_unknown_costmap_goal_rejected(value):
    config, grid, costmap = grids()
    costmap.data[40 * 100 + 40] = value
    with pytest.raises(ValueError): validate_goal_cells(config, grid, costmap)


def test_clearance_rejects_goal_near_obstacle():
    config, grid, costmap = grids()
    grid.data[42 * 100 + 40] = 100
    with pytest.raises(ValueError): validate_goal_cells(config, grid, costmap)


def test_latest_frame_drops_backlog():
    slot = LatestFrame()
    for i in range(100): slot.put(i)
    assert slot.take() == 99
    assert slot.take(timeout=0) is None
    slot.close()
    assert slot.take() is None


def test_shutdown_wakes_worker():
    slot = LatestFrame(); output = []
    thread = threading.Thread(target=lambda: output.append(slot.take(timeout=5)))
    thread.start(); slot.close(); thread.join(timeout=.5)
    assert not thread.is_alive() and output == [None]


@pytest.mark.parametrize('sample,expected', [
    ((10., .2, 0.), True), ((10., 0., .2), True), ((10., 0., 0.), False),
    ((8., .2, 0.), False),
])
def test_motion_evidence_requires_matching_acquisition_stamp(sample, expected):
    assert (moving_sample(dict(stamp=10.), [sample]) is not None) is expected


def test_packaged_models_have_no_external_mesh_dependency():
    models = list(scene_models(ROOT))
    assert len(models) == 2
    assert {item['name'] for _, item, _ in models} == {'assignment5_chair', 'assignment5_person'}
    for world, item, sdf in models:
        assert world == 'warehouse'
        assert 'https://' not in sdf and 'model://' not in sdf
        assert '<static>true</static>' in sdf


def test_models_resolve_through_colcon_symlink_install():
    assert len(list(scene_models(get_package_share_directory('turtlebot4')))) == 2


def test_result_timeout_never_returns_fake_success(monkeypatch):
    class Pending:
        def done(self): return False
    fake = NS()
    with pytest.raises(TimeoutError): WaypointNavigator.await_future(fake, Pending(), 0.)


def test_competing_teleop_rejected():
    fake = NS(get_name=lambda: 'waypoint_navigator', get_node_names=lambda: [],
              get_publishers_info_by_topic=lambda _: [NS(node_name='teleop_twist_keyboard')])
    with pytest.raises(RuntimeError, match='Competing'): WaypointNavigator.check_publishers(fake)
