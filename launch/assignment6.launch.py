"""One owned simulation, real webcam recognizer and independent safe controller."""
import os
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def guard_simulator(context):
    if LaunchConfiguration('start_simulation').perform(context) != 'true':
        return []
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    ros_context = rclpy.Context()
    rclpy.init(context=ros_context)
    node = rclpy.create_node('assignment6_launch_guard', context=ros_context)
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)
    try:
        deadline = time.monotonic()+1.
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
            if node.count_publishers('/clock'):
                raise RuntimeError('A simulator already publishes /clock; stop it or use start_simulation:=false')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=ros_context)
    return []


def preflight_webcam(context):
    from turtlebot4.gesture_recognition_node import runtime_python
    index = int(LaunchConfiguration('webcam_device').perform(context))
    device = Path(f'/dev/video{index}')
    if index < 0 or not device.exists() or not os.access(device, os.R_OK | os.W_OK):
        raise RuntimeError(f'Real webcam unavailable: {device}; configure Docker --device and permissions before launching')
    if not runtime_python().is_file():
        raise RuntimeError('Run scripts/prepare_gesture_runtime.sh first')
    model = Path(LaunchConfiguration('model_path').perform(context)).expanduser()
    if not model.is_file():
        raise RuntimeError(f'MediaPipe model missing: {model}')
    return []


def generate_launch_description():
    share = Path(get_package_share_directory('turtlebot4'))
    cache = Path(os.environ.get('XDG_CACHE_HOME', Path.home()/'.cache'))
    config = str(share/'config/assignment6.yaml')
    value = lambda name, kind: ParameterValue(LaunchConfiguration(name), value_type=kind)
    recognizer = Node(package='turtlebot4', executable='gesture_recognition_node', output='screen',
                      parameters=[config, {'webcam_device': value('webcam_device', int),
                                          'show_camera': value('show_camera', bool),
                                          'model_path': LaunchConfiguration('model_path')}])
    behavior = Node(package='turtlebot4', executable='robot_behavior_node', output='screen',
                    parameters=[config, {k: value(k, float) for k in ('speed', 'distance', 'radius', 'angular_speed', 'signal_timeout')}])
    base = GroupAction(condition=IfCondition(LaunchConfiguration('start_simulation')), actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/turtlebot4.launch.py')),
                                 launch_arguments={'world': LaunchConfiguration('world'), 'rviz': 'false',
                                                   'model': 'standard', 'namespace': '',
                                                   'assignment5_objects': 'false',
                                                   'rviz_fixed_frame': 'odom'}.items())])
    args = dict(webcam_device='0', show_camera='true', world='depot', start_simulation='true',
                model_path=str(cache/'turtlebot4/models/gesture_recognizer.task'),
                speed='0.06', distance='0.12', radius='0.35', angular_speed='0.25', signal_timeout='0.7')
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in args.items()],
        DeclareLaunchArgument('namespace', default_value='', choices=['']),
        OpaqueFunction(function=guard_simulator),
        OpaqueFunction(function=preflight_webcam),
        *[RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
            EmitEvent(event=Shutdown(reason='Assignment 6 node exited; stop owned processes'))]))
          for node in (recognizer, behavior)],
        base, behavior, recognizer,
    ])
