from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("panda_pba_robots", package_name="test_ws").to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
