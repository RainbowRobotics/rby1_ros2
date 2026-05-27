from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration("model").perform(context).lower()
    if model not in ("a", "m"):
        raise RuntimeError("model must be 'a' or 'm'")

    robot_description_file = PathJoinSubstitution(
        [FindPackageShare("rby1_description"), "urdf", f"rby1{model}.urdf.xacro"]
    )
    robot_description = Command(
        [
            "xacro ",
            robot_description_file,
            " robot_ip:=",
            LaunchConfiguration("robot_ip"),
            " use_mock_hardware:=true",
        ]
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value="a", description="RBY1 model: a or m"),
            DeclareLaunchArgument("robot_ip", default_value="127.0.0.1:50051"),
            OpaqueFunction(function=launch_setup),
        ]
    )
