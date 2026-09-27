"""Launch the UR simulation, assignment workcell, and assignment MoveIt."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ur3_llm_control.workcell_scene import box_sdf, iter_models, load_scene


def _spawn_node(model):
    x, y, z, roll, pitch, yaw = model.pose
    return Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{model.name}",
        output="screen",
        arguments=[
            "-string",
            box_sdf(model),
            "-name",
            model.name,
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            str(z),
            "-R",
            str(roll),
            "-P",
            str(pitch),
            "-Y",
            str(yaw),
        ],
    )


def _launch_setup(context):
    scene_path = LaunchConfiguration("scene_config").perform(context)
    scene = load_scene(scene_path)
    models = list(iter_models(scene))
    spawners = [_spawn_node(model) for model in models]

    # ros_gz_sim/create waits for the Gazebo world/create service. Chain each
    # process exit so the pedestal and table exist before dynamic cubes and
    # short-lived create nodes do not exhaust DDS participant slots.
    spawn_sequence = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=current_spawner,
                on_exit=[next_spawner],
            )
        )
        for current_spawner, next_spawner in zip(spawners, spawners[1:])
    ]

    ur_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_simulation_gz"),
                    "launch",
                    "ur_sim_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
            "launch_rviz": "false",
            "runtime_config_package": "ur3_llm_control",
            "controllers_file": "ur3_controllers.yaml",
            "description_package": "ur3_llm_control",
            "description_file": "mounted_ur.urdf.xacro",
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur3_llm_control"), "launch", "moveit.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("launch_moveit")),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "scene_config": LaunchConfiguration("scene_config"),
            # Resolve before entering the include scope. The upstream UR
            # simulation declares an argument with the same name and forces it
            # false, so deferring this substitution can suppress M4's RViz.
            "launch_rviz": LaunchConfiguration("launch_rviz").perform(context),
        }.items(),
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_gripper_controller",
        output="screen",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    # The assignment-local description reads this same canonical scene file
    # for the UR mounting transform. Register every exit handler before the
    # first spawn so no fast process exit can be missed.
    scene_environment = SetEnvironmentVariable(
        "UR3_LLM_SCENE_CONFIG", scene_path
    )
    return [
        scene_environment,
        ur_simulation,
        moveit,
        # RViz plus the M5 scene manager can briefly consume the remaining
        # CycloneDDS participant slots. Start this short-lived spawner after
        # the upstream controller spawners have normally exited.
        TimerAction(period=3.0, actions=[gripper_controller_spawner]),
        *spawn_sequence,
        spawners[0],
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur3e",
                choices=["ur3", "ur3e"],
                description="UR model passed to the official simulation launch.",
            ),
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                choices=["true", "false"],
                description="Start the upstream Gazebo GUI; false is headless.",
            ),
            DeclareLaunchArgument(
                "launch_moveit",
                default_value="true",
                choices=["true", "false"],
                description="Start assignment-local MoveIt using the gripper URDF.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                choices=["true", "false"],
                description="Start RViz with the MoveIt MotionPlanning display.",
            ),
            DeclareLaunchArgument(
                "scene_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ur3_llm_control"), "config", "scene.yaml"]
                ),
                description="Canonical M2.5 workcell YAML file.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
