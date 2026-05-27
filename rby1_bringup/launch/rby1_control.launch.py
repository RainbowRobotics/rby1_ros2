from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration("model").perform(context).lower()
    if model not in ("a", "m"):
        raise RuntimeError("model must be 'a' or 'm'")

    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    auto_reconnect = LaunchConfiguration("auto_reconnect")
    read_timeout_sec = LaunchConfiguration("read_timeout_sec")
    connect_timeout_sec = LaunchConfiguration("connect_timeout_sec")

    robot_description_file = PathJoinSubstitution(
        [FindPackageShare("rby1_description"), "urdf", f"rby1{model}.urdf.xacro"]
    )
    robot_description = Command(
        [
            "xacro ",
            robot_description_file,
            " robot_ip:=",
            robot_ip,
            " use_mock_hardware:=",
            use_mock_hardware,
            " auto_reconnect:=",
            auto_reconnect,
            " read_timeout_sec:=",
            read_timeout_sec,
            " connect_timeout_sec:=",
            connect_timeout_sec,
        ]
    )

    controllers_file = PathJoinSubstitution(
        [FindPackageShare("rby1_bringup"), "config", "rby1_controllers.yaml"]
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[{"robot_description": robot_description}, controllers_file],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="rby1_robot_manager",
            executable="rby1_robot_manager_node",
            namespace=LaunchConfiguration("robot_namespace"),
            output="screen",
            parameters=[
                {
                    "robot_ip": robot_ip,
                    "model": model,
                    "auto_reconnect": auto_reconnect,
                    "connect_timeout_sec": connect_timeout_sec,
                    "read_timeout_sec": read_timeout_sec,
                    "status_period_sec": LaunchConfiguration("status_period_sec"),
                    "publish_status": LaunchConfiguration("publish_status"),
                    "publish_battery_state": LaunchConfiguration("publish_battery_state"),
                    "publish_tool_flange_state": LaunchConfiguration("publish_tool_flange_state"),
                }
            ],
            condition=IfCondition(LaunchConfiguration("launch_robot_manager")),
        ),
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value="a", description="RBY1 model: a or m"),
            DeclareLaunchArgument("robot_ip", default_value="127.0.0.1:50051"),
            DeclareLaunchArgument("robot_namespace", default_value="rby1"),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument("auto_reconnect", default_value="true"),
            DeclareLaunchArgument("connect_timeout_sec", default_value="3.0"),
            DeclareLaunchArgument("read_timeout_sec", default_value="0.5"),
            DeclareLaunchArgument("status_period_sec", default_value="0.02"),
            DeclareLaunchArgument("publish_status", default_value="true"),
            DeclareLaunchArgument("publish_battery_state", default_value="true"),
            DeclareLaunchArgument("publish_tool_flange_state", default_value="true"),
            DeclareLaunchArgument("launch_robot_manager", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
