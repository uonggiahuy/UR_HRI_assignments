"""Only webcam expression recognition and behavior; simulator is independently owned."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('turtlebot4'))/'config/assignment8.yaml')
    defaults = dict(camera_device='/dev/video0', emotion_model='fer', confidence_threshold=PythonExpression(["'0.40' if '", LaunchConfiguration('phone_mode'), "' == 'true' else '0.65'"]),
                    phone_mode='false', minimum_margin=PythonExpression(["'0.10' if '", LaunchConfiguration('phone_mode'), "' == 'true' else '0.0'"]),
                    stable_frames=PythonExpression(["'6' if '", LaunchConfiguration('phone_mode'), "' == 'true' else '4'"]),
                    show_camera='true', use_sim_time='true')
    value = lambda name, kind: ParameterValue(LaunchConfiguration(name), value_type=kind)
    common = dict(use_sim_time=value('use_sim_time', bool), confidence_threshold=value('confidence_threshold', float),
                  phone_mode=value('phone_mode', bool), minimum_margin=value('minimum_margin', float))
    recognizer = Node(package='turtlebot4', executable='emotion_recognition_node', output='screen',
                      parameters=[config, dict(**common, camera_device=LaunchConfiguration('camera_device'),
                        emotion_model=LaunchConfiguration('emotion_model'), show_camera=value('show_camera', bool), stable_frames=value('stable_frames', int))])
    behavior = Node(package='turtlebot4', executable='emotion_behavior_node', output='screen', parameters=[config, common])
    return LaunchDescription([
        DeclareLaunchArgument('phone_mode', default_value='false', choices=['true', 'false']),
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items() if k != 'phone_mode'],
        *[RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[EmitEvent(
            event=Shutdown(reason='Assignment 8 component exited; stopping controller'))])) for node in (recognizer, behavior)],
        behavior, recognizer,
    ])
