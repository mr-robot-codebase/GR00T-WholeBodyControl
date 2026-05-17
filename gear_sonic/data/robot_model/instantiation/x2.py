"""Factory function to instantiate a configured X2 RobotModel from URDF."""

from pathlib import Path

from gear_sonic.data.robot_model.robot_model import RobotModel
from gear_sonic.data.robot_model.supplemental_info.x2.x2_supplemental_info import X2SupplementalInfo


def instantiate_x2_robot_model():
    """
    Instantiate an X2 robot model (x2_ultra variant).

    Returns:
        RobotModel: Configured X2 robot model
    """
    assets_dir = Path(__file__).resolve().parent.parent.parent / "assets"
    urdf_path = assets_dir / "robot_description" / "urdf" / "x2" / "main.urdf"
    asset_path = assets_dir  # parent of robot_description/ for package:// resolution

    return RobotModel(
        str(urdf_path),
        str(asset_path),
        set_floating_base=True,
        supplemental_info=X2SupplementalInfo(),
    )
