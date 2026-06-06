"""Factory function to instantiate a configured X2 RobotModel from URDF."""

from gear_sonic.data.robot_model.robot_model import RobotModel
from gear_sonic.data.robot_model.supplemental_info.x2.x2_supplemental_info import X2SupplementalInfo
from gear_sonic.utils.robot_paths import get_robot_paths


def instantiate_x2_robot_model():
    """
    Instantiate an X2 robot model (x2_ultra variant).

    Returns:
        RobotModel: Configured X2 robot model
    """
    paths = get_robot_paths("x2")
    return RobotModel(
        str(paths["urdf"]),
        str(paths["asset_path"]),
        set_floating_base=True,
        supplemental_info=X2SupplementalInfo(),
    )
