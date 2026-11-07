"""Only Assignment 7 nodes; the existing base/localization/Nav2 own the robot."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def validate_mode(context):
    mode = LaunchConfiguration("test_mode").perform(context)
    speech = LaunchConfiguration("speech_enabled").perform(context)
    if mode not in ("live", "offline") or speech not in ("true", "false"):
        raise ValueError("Invalid test_mode or speech_enabled")
    if mode == "offline" and speech != "false":
        raise ValueError("Offline tests require speech_enabled:=false")
    return []


def generate_launch_description():
    share = Path(get_package_share_directory("turtlebot4"))
    defaults = dict(
        regions_file=str(share / "maps/regions.yaml"),
        map_file=str(share / "maps/map.yaml"),
        whisper_model="small",
        microphone_device="",
        recording_mode="manual",
        language="vi",
        max_recording_duration="30.0",
        show_markers="true",
        speech_enabled="true",
        navigation_timeout="180.0",
        test_mode="live",
    )
    common = {
        "regions_file": LaunchConfiguration("regions_file"),
        "map_file": LaunchConfiguration("map_file"),
        "use_sim_time": True,
    }
    config = str(share / "config/assignment7.yaml")
    speech = Node(
        package="turtlebot4",
        executable="speech_recognition_node",
        name="speech_recognition_node",
        condition=IfCondition(LaunchConfiguration("speech_enabled")),
        parameters=[
            config,
            {
                "whisper_model": LaunchConfiguration("whisper_model"),
                "recording_mode": LaunchConfiguration("recording_mode"),
                "language": LaunchConfiguration("language"),
                "max_recording_duration": ParameterValue(
                    LaunchConfiguration("max_recording_duration"), value_type=float
                ),
                "microphone_device": ParameterValue(
                    LaunchConfiguration("microphone_device"), value_type=str
                ),
            },
        ],
        output="screen",
    )
    navigation = Node(
        package="turtlebot4",
        executable="voice_navigation_node",
        sigterm_timeout="55",
        sigkill_timeout="5",
        parameters=[
            config,
            common,
            {
                "navigation_timeout": ParameterValue(
                    LaunchConfiguration("navigation_timeout"), value_type=float
                )
            },
        ],
        output="screen",
    )
    markers = Node(
        package="turtlebot4",
        executable="semantic_map_node",
        name="semantic_map_node",
        condition=IfCondition(LaunchConfiguration("show_markers")),
        parameters=[config, common],
        output="screen",
    )
    return LaunchDescription(
        [
            *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
            RegisterEventHandler(
                OnProcessExit(
                    target_action=speech,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="Speech process exited; shutting down Assignment 7"
                            )
                        )
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=navigation,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="Voice navigation exited"))
                    ],
                )
            ),
            OpaqueFunction(function=validate_mode),
            speech,
            markers,
            navigation,
        ]
    )
