"""Launch the M1 command node without simulation or motion components."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Build the package-only M1 launch description."""
    ur_type = LaunchConfiguration("ur_type")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur3e",
                choices=["ur3", "ur3e"],
                description="UR model selected for future assignment milestones.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                choices=["true", "false"],
                description="Use a simulation clock when one is available.",
            ),
            Node(
                package="ur3_llm_control",
                executable="command_node",
                name="ur3_llm_command",
                output="screen",
                parameters=[
                    {
                        "ur_type": ur_type,
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
            ),
        ]
    )
