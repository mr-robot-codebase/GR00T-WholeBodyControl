"""X2-specific supplemental info: actuated joints, limits, and default poses."""

from dataclasses import dataclass

import numpy as np

from gear_sonic.data.robot_model.supplemental_info.robot_supplemental_info import (
    RobotSupplementalInfo,
)


@dataclass
class X2SupplementalInfo(RobotSupplementalInfo):
    """Supplemental information for the X2 robot (x2_ultra variant).

    The X2 body joint layout is nearly identical to G1 with three differences:
    - No dexterous hand joints (wrists are the terminal joints)
    - Adds head_yaw_joint and head_pitch_joint
    - Different joint limits (pulled directly from x2_ultra.urdf)
    - Wrist chain order: wrist_yaw → wrist_pitch → wrist_roll
      so wrist_roll_link is the distal end-effector frame
    """

    def __init__(self):
        name = "X2_Ultra"

        body_actuated_joints = [
            # Left leg
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            # Right leg
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            # Waist
            "waist_yaw_joint",
            "waist_pitch_joint",
            "waist_roll_joint",
            # Left arm
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_yaw_joint",
            "left_wrist_pitch_joint",
            "left_wrist_roll_joint",
            # Right arm
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_yaw_joint",
            "right_wrist_pitch_joint",
            "right_wrist_roll_joint",
            # Head
            "head_yaw_joint",
            "head_pitch_joint",
        ]

        # X2 has no dexterous hand joints
        left_hand_actuated_joints = []
        right_hand_actuated_joints = []

        # Limits extracted from x2_ultra.urdf
        joint_limits = {
            # Left leg
            "left_hip_pitch_joint":   [-2.704,  2.556],
            "left_hip_roll_joint":    [-0.235,  2.906],
            "left_hip_yaw_joint":     [-1.684,  3.430],
            "left_knee_joint":        [ 0.000,  2.407],
            "left_ankle_pitch_joint": [-0.803,  0.453],
            "left_ankle_roll_joint":  [-0.262,  0.262],
            # Right leg
            "right_hip_pitch_joint":   [-2.704,  2.556],
            "right_hip_roll_joint":    [-2.906,  0.235],
            "right_hip_yaw_joint":     [-3.430,  1.684],
            "right_knee_joint":        [ 0.000,  2.407],
            "right_ankle_pitch_joint": [-0.803,  0.453],
            "right_ankle_roll_joint":  [-0.263,  0.263],
            # Waist
            "waist_yaw_joint":   [-3.430,  2.382],
            "waist_pitch_joint": [-0.314,  0.314],
            "waist_roll_joint":  [-0.488,  0.488],
            # Left arm
            "left_shoulder_pitch_joint": [-3.080,  2.040],
            "left_shoulder_roll_joint":  [-0.061,  2.993],
            "left_shoulder_yaw_joint":   [-2.556,  2.556],
            "left_elbow_joint":          [-2.356,  0.000],
            "left_wrist_yaw_joint":      [-2.556,  2.556],
            "left_wrist_pitch_joint":    [-0.558,  0.558],
            "left_wrist_roll_joint":     [-1.571,  0.724],
            # Right arm
            "right_shoulder_pitch_joint": [-3.080,  2.040],
            "right_shoulder_roll_joint":  [-2.993,  0.061],
            "right_shoulder_yaw_joint":   [-2.556,  2.556],
            "right_elbow_joint":          [-2.356,  0.000],
            "right_wrist_yaw_joint":      [-2.556,  2.556],
            "right_wrist_pitch_joint":    [-0.558,  0.558],
            "right_wrist_roll_joint":     [-0.724,  1.571],
            # Head
            "head_yaw_joint":   [-0.366,  0.366],
            "head_pitch_joint": [-0.384,  0.384],
        }

        joint_groups = {
            "waist": {
                "joints": ["waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint"],
                "groups": [],
            },
            "head": {
                "joints": ["head_yaw_joint", "head_pitch_joint"],
                "groups": [],
            },
            "left_leg": {
                "joints": [
                    "left_hip_pitch_joint",
                    "left_hip_roll_joint",
                    "left_hip_yaw_joint",
                    "left_knee_joint",
                    "left_ankle_pitch_joint",
                    "left_ankle_roll_joint",
                ],
                "groups": [],
            },
            "right_leg": {
                "joints": [
                    "right_hip_pitch_joint",
                    "right_hip_roll_joint",
                    "right_hip_yaw_joint",
                    "right_knee_joint",
                    "right_ankle_pitch_joint",
                    "right_ankle_roll_joint",
                ],
                "groups": [],
            },
            "legs": {"joints": [], "groups": ["left_leg", "right_leg"]},
            "left_arm": {
                "joints": [
                    "left_shoulder_pitch_joint",
                    "left_shoulder_roll_joint",
                    "left_shoulder_yaw_joint",
                    "left_elbow_joint",
                    "left_wrist_yaw_joint",
                    "left_wrist_pitch_joint",
                    "left_wrist_roll_joint",
                ],
                "groups": [],
            },
            "right_arm": {
                "joints": [
                    "right_shoulder_pitch_joint",
                    "right_shoulder_roll_joint",
                    "right_shoulder_yaw_joint",
                    "right_elbow_joint",
                    "right_wrist_yaw_joint",
                    "right_wrist_pitch_joint",
                    "right_wrist_roll_joint",
                ],
                "groups": [],
            },
            "arms": {"joints": [], "groups": ["left_arm", "right_arm"]},
            "lower_body": {"joints": [], "groups": ["waist", "legs"]},
            "upper_body_no_hands": {"joints": [], "groups": ["arms", "head"]},
            "body": {"joints": [], "groups": ["lower_body", "upper_body_no_hands"]},
            # X2 has no hands — upper_body == upper_body_no_hands
            "upper_body": {"joints": [], "groups": ["upper_body_no_hands"]},
        }

        # Generic name → robot-specific joint name mapping (used by teleop/IK)
        joint_name_mapping = {
            "waist_pitch": "waist_pitch_joint",
            "waist_roll":  "waist_roll_joint",
            "waist_yaw":   "waist_yaw_joint",
            "shoulder_pitch": {
                "left":  "left_shoulder_pitch_joint",
                "right": "right_shoulder_pitch_joint",
            },
            "shoulder_roll": {
                "left":  "left_shoulder_roll_joint",
                "right": "right_shoulder_roll_joint",
            },
            "shoulder_yaw": {
                "left":  "left_shoulder_yaw_joint",
                "right": "right_shoulder_yaw_joint",
            },
            "elbow_pitch": {
                "left":  "left_elbow_joint",
                "right": "right_elbow_joint",
            },
            "wrist_yaw": {
                "left":  "left_wrist_yaw_joint",
                "right": "right_wrist_yaw_joint",
            },
            "wrist_pitch": {
                "left":  "left_wrist_pitch_joint",
                "right": "right_wrist_pitch_joint",
            },
            "wrist_roll": {
                "left":  "left_wrist_roll_joint",
                "right": "right_wrist_roll_joint",
            },
        }

        root_frame_name = "pelvis"

        # wrist_roll_link is the terminal link in X2's wrist chain
        hand_frame_names = {
            "left":  "left_wrist_roll_link",
            "right": "right_wrist_roll_link",
        }

        calibration_joint_q = {"elbow_pitch": {"left": 0.0, "right": 0.0}}

        # May need tuning once the actual wrist frame orientation is verified
        hand_rotation_correction = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])

        default_joint_q = {
            "shoulder_roll": {"left": 0.2, "right": -0.2},
        }

        teleop_upper_body_motion_scale = 1.0

        super().__init__(
            name=name,
            body_actuated_joints=body_actuated_joints,
            left_hand_actuated_joints=left_hand_actuated_joints,
            right_hand_actuated_joints=right_hand_actuated_joints,
            joint_limits=joint_limits,
            joint_groups=joint_groups,
            root_frame_name=root_frame_name,
            hand_frame_names=hand_frame_names,
            calibration_joint_q=calibration_joint_q,
            joint_name_mapping=joint_name_mapping,
            hand_rotation_correction=hand_rotation_correction,
            default_joint_q=default_joint_q,
            teleop_upper_body_motion_scale=teleop_upper_body_motion_scale,
        )
