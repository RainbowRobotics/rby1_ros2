from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("RBY1_A_v1_0", package_name="rby1_moveit_a_1_0").to_moveit_configs()
    return generate_static_virtual_joint_tfs_launch(moveit_config)
