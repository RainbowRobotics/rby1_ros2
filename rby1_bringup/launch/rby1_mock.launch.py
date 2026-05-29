from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value="auto",
                description="RBY1 model: 'a', 'm', 'ub', or 'auto' (mock mode treats 'auto' as 'a')",
            ),
            DeclareLaunchArgument(
                "version",
                default_value="auto",
                description="Model version (mock mode treats 'auto' as default 1.2; ignored for ub)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("rby1_bringup"), "launch", "rby1_control.launch.py"]
                    )
                ),
                launch_arguments={
                    "model": LaunchConfiguration("model"),
                    "version": LaunchConfiguration("version"),
                    "use_mock_hardware": "true",
                    "launch_robot_manager": "false",
                    "publish_status": "false",
                    "publish_battery_state": "false",
                    "publish_tool_flange_state": "false",
                }.items(),
            ),
        ]
    )
