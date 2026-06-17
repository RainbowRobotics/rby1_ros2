from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("RBY1_M_v1_2", package_name="rby1_moveit_m_1_2").to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)
