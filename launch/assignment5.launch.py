"""Mission and detection share a foreground owner; failures cancel the mission."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('turtlebot4'))
    detector = Node(package='turtlebot4', executable='yolo_detector', name='yolo_detector',
                    parameters=[str(share / 'config/yolo.yaml'),
                                {'image_topic': LaunchConfiguration('image_topic')}], output='screen')
    navigator = Node(package='turtlebot4', executable='waypoint_navigator', name='waypoint_navigator',
                     sigterm_timeout='12', sigkill_timeout='5',
                     parameters=[{'use_sim_time': True,
                                  'goals_file': LaunchConfiguration('goals_file'),
                                  'ready_timeout': 60., 'goal_timeout': 120.}], output='screen')
    viewer = Node(package='turtlebot4', executable='yolo_viewer', name='yolo_viewer',
                  condition=IfCondition(LaunchConfiguration('show_yolo_window')),
                  parameters=[{'use_sim_time': True}], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('show_yolo_window', default_value='true',
                              description='Open optional annotated-camera HighGUI window'),
        DeclareLaunchArgument('image_topic', default_value='/oakd/rgb/preview/image_raw'),
        DeclareLaunchArgument('goals_file', default_value=str(share / 'config/goals.yaml')),
        RegisterEventHandler(OnProcessExit(target_action=detector, on_exit=[
            EmitEvent(event=Shutdown(reason='YOLO exited; cancel mission'))])),
        RegisterEventHandler(OnProcessExit(target_action=navigator, on_exit=[
            EmitEvent(event=Shutdown(reason='Mission ended; stop detector'))])),
        detector, navigator, viewer,
    ])
