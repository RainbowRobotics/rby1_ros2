import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    arg_model = DeclareLaunchArgument(
        "model",
        default_value="a",
        description="Robot model type ('a' or 'm')"
    )
    arg_version = DeclareLaunchArgument(
        "version",
        default_value="1.0",
        description="Model version string (e.g., 1.0)"
    )
    arg_robot_ip = DeclareLaunchArgument(
        "robot_ip",
        default_value="127.0.0.1:50051",
        description="IP address of the robot or simulator gRPC server"
    )
    arg_collision_check_enable = DeclareLaunchArgument(
        "collision_check_enable",
        default_value="false",
        description="Enable predictive collision checking in hardware interface"
    )
    arg_collision_threshold = DeclareLaunchArgument(
        "collision_threshold",
        default_value="0.01",
        description="Collision threshold in meters"
    )

    # Process Xacro description file
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([FindPackageShare("rby1_description"), "urdf", "rby1_control.urdf.xacro"]),
        " model:=", LaunchConfiguration("model"),
        " version:=", LaunchConfiguration("version"),
        " robot_ip:=", LaunchConfiguration("robot_ip"),
        " collision_check_enable:=", LaunchConfiguration("collision_check_enable"),
        " collision_threshold:=", LaunchConfiguration("collision_threshold"),
    ])

    robot_description = {"robot_description": robot_description_content}

    # Controller configurations yaml
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("rby1_description"),
        "config",
        "rby1_controllers.yaml"
    ])

    # Nodes
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_yaml],
    )

    # Spawners for controllers
    spawner_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
        output="screen",
    )

    spawner_default_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["both_arms_torso_head_controller", "--controller-manager", "controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        arg_model,
        arg_version,
        arg_robot_ip,
        arg_collision_check_enable,
        arg_collision_threshold,
        node_robot_state_publisher,
        node_controller_manager,
        spawner_joint_state_broadcaster,
        spawner_default_controller,
    ])
