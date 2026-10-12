"""Compose the installed Ignition world and one TurtleBot 4 spawn."""

from pathlib import Path
import os
import re
import signal

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetParametersFromFile, SetRemap


def upstream_share():
    return Path(get_package_share_directory('turtlebot4_ignition_bringup'))


def supported_worlds():
    return sorted(path.stem for path in (upstream_share() / 'worlds').glob('*.sdf'))


def validate_namespace(value):
    if value and not re.fullmatch(r'/?[A-Za-z_][A-Za-z_0-9]*(/[A-Za-z_][A-Za-z_0-9]*)*', value):
        raise ValueError('namespace must contain valid ROS name components')
    return value.strip('/')


def child_exit(event, context):
    if event.returncode and not context.is_shutdown:
        reason = f'Simulation child failed with exit code {event.returncode}: {event.action}'
        get_logger('turtlebot4.simulation').error(reason)
        return [EmitEvent(event=Shutdown(reason=reason))]
    return []


def descendant_pids(parent, parents):
    result = []
    pending = [parent]
    seen = {parent}
    while pending:
        current = pending.pop()
        children = [pid for pid, ppid in parents.items() if ppid == current and pid not in seen]
        result.extend(children)
        seen.update(children)
        pending.extend(children)
    return result


def process_snapshot():
    result = {}
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            text = path.read_text()
            fields = text[text.rfind(')') + 2:].split()
            result[int(path.parent.name)] = (int(fields[1]), fields[19])
        except (OSError, ValueError, IndexError):
            continue  # Processes may exit during a snapshot.
    return result


class GazeboLifecycle:
    """Forward shutdown to children of this launch's upstream shell process only."""

    def __init__(self):
        self.launchers = {}

    def started(self, event, context):
        if 'ign gazebo' in ' '.join(event.cmd):
            snapshot = process_snapshot()
            if event.pid in snapshot:
                self.launchers[event.pid] = snapshot[event.pid][1]

    def shutdown(self, event, context):
        snapshot = process_snapshot()
        parents = {pid: values[0] for pid, values in snapshot.items()}
        for launcher, started in self.launchers.items():
            if snapshot.get(launcher, (None, None))[1] != started:
                continue
            # Upstream shell=True only receives launch's signal itself. Its server/GUI
            # create separate process groups, so signal the owned descendants directly.
            for pid in reversed(descendant_pids(launcher, parents)):
                if process_snapshot().get(pid) != snapshot.get(pid):
                    continue
                try:
                    os.kill(pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        return []


def compose(context):
    share = upstream_share()
    world = LaunchConfiguration('world').perform(context)
    model = LaunchConfiguration('model').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)
    namespace = validate_namespace(LaunchConfiguration('namespace').perform(context))
    if world not in supported_worlds() or model not in ('standard', 'lite'):
        raise ValueError('Unsupported installed world or robot model')
    if rviz not in ('true', 'false'):
        raise ValueError('rviz must be true or false')

    def include(filename, arguments):
        path = share / 'launch' / filename
        if not path.is_file():
            raise FileNotFoundError(path)
        return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(path)),
            launch_arguments=arguments.items(),
        )])

    # Separate scopes prevent the child's generic argument names leaking to siblings.
    # SetParameter also covers upstream nodes that omit a use_sim_time argument.
    actions = [
        SetParameter(name='use_sim_time', value=True),
        # Merge measured wheels with defaults for passive wheel-drop joints.
        # A separate input alias prevents the publisher's output remap feeding itself.
        SetParametersFromFile(str(Path(get_package_share_directory('turtlebot4'))
                                  / 'config/simulation.yaml')),
        SetRemap(src='joint_state_publisher:joint_states', dst='joint_states_visualization'),
        SetRemap(src='joint_state_publisher:joint_states_input', dst='joint_states'),
        SetRemap(src='robot_state_publisher:joint_states', dst='joint_states_visualization'),
        include('ignition.launch.py', {
            'world': world, 'model': model, 'use_sim_time': 'true',
            'on_exit_shutdown': 'true',
        }),
        include('turtlebot4_spawn.launch.py', {
            'world': world, 'model': model, 'namespace': namespace, 'rviz': 'false',
            'use_sim_time': 'true', 'gazebo': 'ignition', 'description': 'false',
            'slam': 'false', 'localization': 'false', 'nav2': 'false',
            'x': '0.0', 'y': '0.0', 'z': '0.0', 'yaw': '0.0',
        }),
    ]
    if rviz == 'true':
        # The upstream RViz launch hardcodes a map-frame navigation config.
        # Reuse upstream rviz2 with an assignment-owned simulator-only config.
        config = Path(get_package_share_directory('turtlebot4')) / 'rviz/simulation.rviz'
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                            namespace=namespace, arguments=['-d', str(config)],
                            parameters=[{'use_sim_time': True}],
                            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                            output='screen'))
    return [GroupAction(scoped=True, actions=actions)]


def generate_launch_description():
    lifecycle = GazeboLifecycle()
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='warehouse', choices=supported_worlds()),
        DeclareLaunchArgument('model', default_value='standard', choices=['standard', 'lite']),
        DeclareLaunchArgument('namespace', default_value='', description='Robot ROS namespace'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        RegisterEventHandler(OnProcessStart(on_start=lifecycle.started)),
        RegisterEventHandler(OnShutdown(on_shutdown=lifecycle.shutdown)),
        RegisterEventHandler(OnProcessExit(on_exit=child_exit)),
        OpaqueFunction(function=compose),
    ])
