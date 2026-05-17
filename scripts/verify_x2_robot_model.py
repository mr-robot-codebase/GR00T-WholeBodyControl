"""Smoke test: instantiate RobotModel with X2SupplementalInfo and print key info."""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from gear_sonic.data.robot_model.robot_model import RobotModel
from gear_sonic.data.robot_model.supplemental_info.x2.x2_supplemental_info import X2SupplementalInfo

URDF_PATH = "gear_sonic/data/assets/robot_description/urdf/x2/main.urdf"
ASSET_PATH = "gear_sonic/data/assets"  # parent of robot_description/ for package:// resolution

print("=" * 60)
print("Loading X2 RobotModel with X2SupplementalInfo ...")
print("=" * 60)

info = X2SupplementalInfo()
model = RobotModel(
    urdf_path=URDF_PATH,
    asset_path=ASSET_PATH,
    set_floating_base=True,
    supplemental_info=info,
)

print(f"\n[OK] RobotModel built successfully")
print(f"     Total DOFs (incl. floating base): {model.num_dofs}")
print(f"     Total joints:                     {model.num_joints}")

print(f"\n--- All joints recognised by Pinocchio ---")
for name in model.joint_names:
    print(f"  {name}")

print(f"\n--- Body actuated joint indices ({len(model.get_body_actuated_joint_indices())} joints) ---")
for name, idx in zip(info.body_actuated_joints, model.get_body_actuated_joint_indices()):
    lo = model.lower_joint_limits[idx - 7]
    hi = model.upper_joint_limits[idx - 7]
    print(f"  [{idx:3d}] {name:40s}  limits: [{lo:.3f}, {hi:.3f}]")

print(f"\n--- Joint groups ---")
for group in ["legs", "waist", "arms", "head", "lower_body", "upper_body", "body"]:
    indices = model.get_joint_group_indices(group)
    print(f"  {group:25s}: {len(indices)} joints")

print(f"\n--- Default body pose (non-zero joints) ---")
q0 = model.get_default_body_pose()
for name in info.body_actuated_joints:
    idx = model.dof_index(name)
    val = q0[idx]
    if abs(val) > 1e-6:
        print(f"  {name:40s} = {val:.4f}")

print(f"\n--- Forward kinematics at default pose ---")
model.cache_forward_kinematics(q0)
for side in ["left", "right"]:
    frame = info.hand_frame_names[side]
    placement = model.frame_placement(frame)
    print(f"  {side} hand frame ({frame}): translation = {placement.translation.round(4)}")

print("\n[PASS] X2 RobotModel instantiation complete.\n")
