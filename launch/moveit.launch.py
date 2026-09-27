"""Launch MoveIt with the assignment-local URDF and stock UR semantics."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_yaml(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _launch_setup(context):
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    launch_rviz = LaunchConfiguration("launch_rviz")

    description_file = PathJoinSubstitution(
        [FindPackageShare("ur3_llm_control"), "urdf", "mounted_ur.urdf.xacro"]
    )
    vendor_description = FindPackageShare("ur_description")
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            description_file,
            " name:=ur ur_type:=",
            ur_type,
            " safety_limits:=",
            safety_limits,
            " safety_pos_margin:=",
            safety_pos_margin,
            " safety_k_position:=",
            safety_k_position,
            " joint_limit_params:=",
            PathJoinSubstitution(
                [vendor_description, "config", ur_type, "joint_limits.yaml"]
            ),
            " kinematics_params:=",
            PathJoinSubstitution(
                [vendor_description, "config", ur_type, "default_kinematics.yaml"]
            ),
            " physical_params:=",
            PathJoinSubstitution(
                [vendor_description, "config", ur_type, "physical_parameters.yaml"]
            ),
            " visual_params:=",
            PathJoinSubstitution(
                [vendor_description, "config", ur_type, "visual_parameters.yaml"]
            ),
            " sim_ignition:=false",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    semantic_file = PathJoinSubstitution(
        [
            FindPackageShare("ur3_llm_control"),
            "srdf",
            "ur_with_gripper.srdf.xacro",
        ]
    )
    semantic_content = Command(
        [FindExecutable(name="xacro"), " ", semantic_file, " name:=ur prefix:=\"\""]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(semantic_content, value_type=str)
    }
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
    )
    robot_description_planning = {
        "robot_description_planning": _load_yaml(
            "ur_moveit_config", "config/joint_limits.yaml"
        )
    }

    planning_pipeline = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    planning_pipeline["move_group"].update(
        _load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    )

    controllers = _load_yaml("ur_moveit_config", "config/controllers.yaml")
    controllers["scaled_joint_trajectory_controller"]["default"] = False
    controllers["joint_trajectory_controller"]["default"] = True
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers,
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
    }
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }
    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    common_parameters = [
        robot_description,
        robot_description_semantic,
        {"publish_robot_description_semantic": True},
        robot_description_kinematics,
        robot_description_planning,
        planning_pipeline,
        trajectory_execution,
        moveit_controllers,
        planning_scene_monitor,
        {"use_sim_time": True},
    ]
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=common_parameters,
    )
    planning_scene = Node(
        package="ur3_llm_control",
        executable="planning_scene",
        name="workcell_planning_scene",
        output="screen",
        parameters=[{"scene_config": LaunchConfiguration("scene_config")}],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "rviz", "view_robot.rviz"]
            ),
        ],
        parameters=common_parameters,
    )
    return [move_group, planning_scene, rviz]


def generate_launch_description():
    scene_config = LaunchConfiguration("scene_config")
    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur3e", choices=["ur3", "ur3e"]),
            DeclareLaunchArgument("safety_limits", default_value="true"),
            DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
            DeclareLaunchArgument("safety_k_position", default_value="20"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument(
                "scene_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ur3_llm_control"), "config", "scene.yaml"]
                ),
            ),
            SetEnvironmentVariable("UR3_LLM_SCENE_CONFIG", scene_config),
            OpaqueFunction(function=_launch_setup),
        ]
    )
