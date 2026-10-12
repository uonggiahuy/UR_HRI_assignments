"""Offline M1 assertions; no simulator or ROS graph is required."""

import argparse
import ast
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
import pytest
from sensor_msgs.msg import LaserScan

from turtlebot4 import diagnostics as diag
from turtlebot4 import simulation_launch as sim


@pytest.mark.parametrize('value', ['nan', 'inf', '-1', '0', '0.5', '121'])
def test_timeout_rejects_unbounded_values(value):
    with pytest.raises(argparse.ArgumentTypeError):
        diag.bounded_timeout(value)


@pytest.mark.parametrize('value', ['1', '20', '120'])
def test_timeout_accepts_bounded_values(value):
    assert diag.bounded_timeout(value) == float(value)


@pytest.mark.parametrize('value', ['../robot', 'robot//one', '2robot', 'robot-name', '/'])
def test_namespace_rejects_invalid_names(value):
    with pytest.raises(ValueError):
        sim.validate_namespace(value)


@pytest.mark.parametrize('value,expected', [('', ''), ('robot', 'robot'), ('/lab/robot', 'lab/robot')])
def test_namespace_normalization(value, expected):
    assert sim.validate_namespace(value) == expected


def test_launch_syntax_and_installed_loadability():
    path = Path(get_package_share_directory('turtlebot4')) / 'launch/simulation.launch.py'
    ast.parse(path.read_text())
    description = PythonLaunchDescriptionSource(str(path)).get_launch_description(LaunchContext())
    assert description.entities


def test_upstream_resolution_and_world_defaults():
    assert (sim.upstream_share() / 'launch/ignition.launch.py').is_file()
    assert (sim.upstream_share() / 'launch/turtlebot4_spawn.launch.py').is_file()
    assert 'warehouse' in sim.supported_worlds()
    declarations = {item.name: item for item in sim.generate_launch_description().entities
                    if isinstance(item, DeclareLaunchArgument)}
    assert set(declarations) == {'world', 'model', 'namespace', 'rviz'}
    context = LaunchContext()
    defaults = {name: ''.join(x.perform(context) for x in item.default_value)
                for name, item in declarations.items()}
    assert defaults == {'world': 'warehouse', 'model': 'standard', 'namespace': '', 'rviz': 'false'}
    assert declarations['world'].choices == sim.supported_worlds()


@pytest.mark.parametrize('model,namespace,world', [
    ('standard', '', 'warehouse'), ('lite', 'robot', 'depot'), ('standard', '/lab/robot', 'maze')])
def test_composition_has_one_world_one_spawn_and_explicit_arguments(model, namespace, world):
    context = LaunchContext()
    context.launch_configurations.update(world=world, model=model, namespace=namespace, rviz='true')
    actions = sim.compose(context)[0].get_sub_entities()
    groups = [item for item in actions if isinstance(item, GroupAction)]
    includes = [item for group in groups for item in group.get_sub_entities()
                if isinstance(item, IncludeLaunchDescription)]
    assert len(includes) == 2
    for item in includes:
        item.launch_description_source.get_launch_description(context)
    paths = [item.launch_description_source.location for item in includes]
    assert sum('ignition.launch.py' in path for path in paths) == 1
    assert sum('turtlebot4_spawn.launch.py' in path for path in paths) == 1
    for item in includes:
        args = dict(item.launch_arguments)
        assert args['model'] == model
        assert args['world'] == world
        assert args['use_sim_time'] == 'true'
    spawn_args = dict(includes[1].launch_arguments)
    assert spawn_args['namespace'] == namespace.strip('/')
    assert all(spawn_args[key] == 'false' for key in ('slam', 'localization', 'nav2', 'description'))


def test_missing_upstream_launch_fails_visibly(tmp_path, monkeypatch):
    monkeypatch.setattr(sim, 'upstream_share', lambda: tmp_path)
    monkeypatch.setattr(sim, 'supported_worlds', lambda: ['warehouse'])
    context = LaunchContext()
    context.launch_configurations.update(world='warehouse', model='standard', namespace='', rviz='false')
    with pytest.raises(FileNotFoundError):
        sim.compose(context)


def test_child_failure_requests_shutdown():
    context = LaunchContext()
    assert sim.child_exit(SimpleNamespace(returncode=0, action='spawn'), context) == []
    assert len(sim.child_exit(SimpleNamespace(returncode=1, action='bridge'), context)) == 1


def make_scan():
    scan = LaserScan()
    scan.header.frame_id = 'lidar'
    scan.range_min, scan.range_max = 0.1, 10.0
    scan.angle_min, scan.angle_max, scan.angle_increment = -1.0, 1.0, 0.1
    scan.ranges = [1.0, float('inf')]
    return scan


def test_scan_allows_infinity_but_requires_valid_returns():
    scan = make_scan()
    assert diag.scan_valid(scan)
    scan.ranges = [float('inf'), float('nan')]
    assert not diag.scan_valid(scan)
    scan.ranges = []
    assert not diag.scan_valid(scan)


def test_scan_rejects_invalid_metadata():
    scan = make_scan()
    scan.range_max = float('nan')
    assert not diag.scan_valid(scan)


def test_odometry_finite_values_and_orientation():
    odom = Odometry()
    odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
    odom.pose.pose.orientation.w = 1.0
    assert diag.odometry_valid(odom)
    odom.twist.twist.linear.x = float('nan')
    assert not diag.odometry_valid(odom)
    odom.twist.twist.linear.x = 0.0
    odom.pose.pose.orientation.w = 0.0
    assert not diag.odometry_valid(odom)


def test_discovery_rejects_ambiguous_and_wrong_types():
    kind = 'sensor_msgs/msg/LaserScan'
    assert diag.discover_topic({'/laser': [kind]}, '/scan', kind) == '/laser'
    assert diag.discover_topic({'/a': [kind], '/b': [kind]}, '/scan', kind) is None
    assert diag.discover_topic({'/scan': ['std_msgs/msg/String'], '/a': [kind]}, '/scan', kind) is None
    assert diag.discover_topic({'/robot/laser': [kind], '/other/scan': [kind]}, '/robot/scan', kind, 'robot') == '/robot/laser'
    assert diag.discover_topic({'/_internal/ir/scan': [kind]}, '/scan', kind) is None


def test_tf_missing_chain_and_cycles():
    parents = {'base_link': 'odom', 'lidar': 'base_link'}
    assert diag.frame_chain(parents, 'odom', 'lidar') == ['odom', 'base_link', 'lidar']
    assert diag.frame_chain(parents, 'odom', 'missing') is None
    cycle = {'a': 'b', 'b': 'a'}
    assert diag.frame_chain(cycle, 'odom', 'a') is None
    assert diag.SimulationProbe.has_cycle(cycle, 'a')


def test_observation_counters_and_clock_regression():
    observed = diag.Observations()
    assert observed.summary()['received'] == 0
    observed.add(None, 1.0)
    observed.add(None, 0.0, valid=False)
    summary = observed.summary()
    assert summary['received'] == 2 and summary['invalid'] == 1
    assert summary['stamp_regression']


def test_absent_graph_is_failure_not_fabricated_pass():
    probe = SimpleNamespace(namespace='', topics={},
                            samples={key: diag.Observations() for key in ('clock', 'scan', 'odom')},
                            parents=defaultdict(set), authorities=defaultdict(set),
                            transform_conflicts=set(), duplicate_transforms=set(),
                            conflict_evidence=[],
                            tf_counts=defaultdict(int), invalid_tf=0,
                            endpoints=lambda *args, **kwargs: [],
                            expected=lambda key: '/' + key,
                            get_topic_names_and_types=lambda: [],
                            get_node_names_and_namespaces=lambda: [],
                            has_cycle=diag.SimulationProbe.has_cycle)
    checks = diag.SimulationProbe.report(probe, True)
    for key in ('clock', 'scan', 'odom', 'cmd_vel', 'tf', 'rviz_node'):
        assert checks[key]['status'] == 'FAIL'
    assert checks['rviz_visualization']['status'] == 'MANUAL VERIFICATION REQUIRED'


def test_static_tf_qos_requests_retained_history():
    qos = diag.observation_qos('tf_static')
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert diag.observation_qos('scan').reliability == ReliabilityPolicy.BEST_EFFORT


def test_transform_conflicts_and_duplicates():
    probe = SimpleNamespace(tf_counts=defaultdict(int), parents=defaultdict(set),
                            authorities=defaultdict(set), invalid_tf=0,
                            transform_samples={}, transform_conflicts=set(), duplicate_transforms=set())
    probe.conflict_evidence = []
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'odom', 'base_link'
    transform.transform.rotation.w = 1.0
    message = TFMessage(transforms=[transform, transform])
    diag.SimulationProbe.record_tf(probe, 'tf', message)
    assert ('odom', 'base_link') in probe.duplicate_transforms
    transform.transform.translation.x = 1.0
    diag.SimulationProbe.record_tf(probe, 'tf', TFMessage(transforms=[transform]))
    assert ('odom', 'base_link') in probe.transform_conflicts


def test_installed_visualization_resource_uses_simulator_frames():
    path = Path(get_package_share_directory('turtlebot4')) / 'rviz/simulation.rviz'
    assert path.is_file()
    text = path.read_text()
    assert 'Fixed Frame: odom' in text
    assert 'rviz_default_plugins/RobotModel' in text
    assert 'rviz_default_plugins/LaserScan' in text
    assert (path.parents[1] / 'config/simulation.yaml').is_file()


def test_shutdown_targets_only_owned_descendants():
    parents = {10: 1, 11: 10, 12: 11, 20: 1, 21: 20}
    assert set(sim.descendant_pids(10, parents)) == {11, 12}
    assert sim.descendant_pids(99, parents) == []


def test_tf_quaternion_sign_equivalence_is_not_a_conflict():
    assert diag.same_transform([1, 2, 3, 0, 0, 0.6, 0.8], [1, 2, 3, 0, 0, -0.6, -0.8])
    assert not diag.same_transform([1, 2, 3, 0, 0, 0, 1], [2, 2, 3, 0, 0, 0, 1])


def test_discovery_waits_for_canonical_topic_and_replaces_a_late_fallback():
    graph = {'/laser': ['sensor_msgs/msg/LaserScan']}
    removed = []
    probe = SimpleNamespace(namespace='', topics={}, subscriptions_owned=[], subscriptions_by_key={},
                            samples={'scan': diag.Observations()},
                            expected=lambda name: '/' + name,
                            get_topic_names_and_types=lambda: list(graph.items()),
                            create_subscription=lambda kind, topic, callback, qos: topic,
                            destroy_subscription=removed.append,
                            record_sample=lambda *args: None, record_tf=lambda *args: None)
    diag.SimulationProbe.discover(probe)
    assert probe.topics == {}
    diag.SimulationProbe.discover(probe, allow_fallback=True)
    assert probe.topics['scan'] == '/laser'
    graph['/scan'] = ['sensor_msgs/msg/LaserScan']
    diag.SimulationProbe.discover(probe)
    assert probe.topics['scan'] == '/scan'
    assert removed == ['/laser']
