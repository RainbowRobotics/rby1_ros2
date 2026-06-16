from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # Declare launch arguments
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Start robot with fake hardware/simulation (mock_components/GenericSystem) or real SDK hardware."
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="127.0.0.1:50051",
        description="IP address of the RBY1 robot SDK server."
    )
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="m",
        description="Robot model type (a or m)."
    )
    model_name_arg = DeclareLaunchArgument(
        "model_name",
        default_value="rby1m",
        description="Folder name of the robot description model (rby1a or rby1m)."
    )
    model_version_arg = DeclareLaunchArgument(
        "model_version",
        default_value="1_3",
        description="Version string of the model (e.g., 1.0, 1_1, 1_2, 1_3)."
    )
    collision_check_enable_arg = DeclareLaunchArgument(
        "collision_check_enable",
        default_value="false",
        description="Enable/disable hardware-level predictive collision checking."
    )
    collision_threshold_arg = DeclareLaunchArgument(
        "collision_threshold",
        default_value="0.01",
        description="Collision distance threshold in meters."
    )

    # Build MoveIt configuration with mappings
    moveit_config = (
        MoveItConfigsBuilder("rby1", package_name="rby1_moveit_test")
        .robot_description(
            file_path="config/rby1.urdf.xacro",
            mappings={
                "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "model": LaunchConfiguration("model"),
                "model_name": LaunchConfiguration("model_name"),
                "model_version": LaunchConfiguration("model_version"),
                "collision_check_enable": LaunchConfiguration("collision_check_enable"),
                "collision_threshold": LaunchConfiguration("collision_threshold"),
            }
        )
        .to_moveit_configs()
    )

    # Generate the demo launch using moveit_configs_utils
    demo_launch = generate_demo_launch(moveit_config)

    # Return LaunchDescription with custom arguments appended
    return LaunchDescription([
        use_fake_hardware_arg,
        robot_ip_arg,
        model_arg,
        model_name_arg,
        model_version_arg,
        collision_check_enable_arg,
        collision_threshold_arg,
        *demo_launch.entities
    ])
