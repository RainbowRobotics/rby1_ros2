from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


SUPPORTED_VERSIONS = {
    "a": ("1.0", "1.1", "1.2"),
    "m": ("1.0", "1.1", "1.2", "1.3"),
}
DEFAULT_VERSION = "1.2"


def _truthy(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes", "on")


def _resolve_model_and_version(
    context, robot_ip: str, use_mock_hardware: bool, connect_timeout_sec: float
):
    model_arg = LaunchConfiguration("model").perform(context).strip().lower()
    version_arg = LaunchConfiguration("version").perform(context).strip().lower()

    if model_arg not in ("a", "m", "ub", "auto"):
        raise RuntimeError("model must be 'a', 'm', 'ub', or 'auto'")

    needs_probe = (model_arg == "auto" or version_arg == "auto") and not use_mock_hardware

    probed_model = None
    probed_version = None
    if needs_probe:
        from rby1_bringup.model_probe import detect_model_and_version

        probed_model, probed_version = detect_model_and_version(robot_ip, connect_timeout_sec)
    elif use_mock_hardware and (model_arg == "auto" or version_arg == "auto"):
        print("[rby1_bringup] use_mock_hardware is true; auto values resolve to defaults (probe skipped).")

    if model_arg == "auto":
        model = probed_model if probed_model is not None else "a"
    else:
        model = model_arg

    if model == "ub":
        version = ""  # not used by ub xacro
    elif version_arg == "auto":
        supported = SUPPORTED_VERSIONS.get(model, ())
        if probed_version is not None and probed_version in supported:
            version = probed_version
        else:
            if probed_version is not None:
                print(
                    f"[rby1_bringup] WARNING: probed version '{probed_version}' is not in "
                    f"supported versions {supported} for model '{model}'; "
                    f"falling back to default '{DEFAULT_VERSION}'."
                )
            version = DEFAULT_VERSION
    else:
        version = version_arg

    return model, version


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    use_mock_hardware_str = LaunchConfiguration("use_mock_hardware").perform(context)
    use_mock_hardware_bool = _truthy(use_mock_hardware_str)
    connect_timeout_sec_str = LaunchConfiguration("connect_timeout_sec").perform(context)
    try:
        connect_timeout_sec_val = float(connect_timeout_sec_str)
    except ValueError:
        connect_timeout_sec_val = 3.0

    model, version = _resolve_model_and_version(
        context, robot_ip, use_mock_hardware_bool, connect_timeout_sec_val
    )

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    auto_reconnect = LaunchConfiguration("auto_reconnect")
    read_timeout_sec = LaunchConfiguration("read_timeout_sec")
    connect_timeout_sec = LaunchConfiguration("connect_timeout_sec")

    robot_description_file = PathJoinSubstitution(
        [FindPackageShare("rby1_description"), "urdf", f"rby1{model}.urdf.xacro"]
    )
    xacro_cmd = [
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
    if model != "ub":
        xacro_cmd += [" version:=", version]
    robot_description = Command(xacro_cmd)

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
            DeclareLaunchArgument(
                "model",
                default_value="auto",
                description="RBY1 model: 'a', 'm', 'ub', or 'auto' to detect via rby1-sdk",
            ),
            DeclareLaunchArgument(
                "version",
                default_value="auto",
                description=(
                    "Model version: 'auto' to detect via rby1-sdk, or one of "
                    "a:1.0/1.1/1.2 m:1.0/1.1/1.2/1.3 (ignored for ub)"
                ),
            ),
            DeclareLaunchArgument("robot_ip", default_value="192.168.30.1:50051"),
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
