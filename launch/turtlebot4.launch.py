"""Persistent base with an explicit RViz frame for retained-map sessions."""
from launch.actions import DeclareLaunchArgument
from turtlebot4.simulation_launch import generate_launch_description as base_description


def generate_launch_description():
    description = base_description()
    description.entities.insert(0, DeclareLaunchArgument(
        'assignment5_objects', default_value='false', choices=['true', 'false'],
        description='Spawn the packaged Assignment 5 chair and person beside the dock route'))
    description.entities.insert(0, DeclareLaunchArgument(
        'rviz_fixed_frame', default_value='odom', choices=['odom', 'map'],
        description='Use map from startup for SLAM/AMCL retained-map visualization'))
    return description
