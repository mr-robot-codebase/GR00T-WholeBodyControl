"""DDS bridge between MuJoCo sim and the WBC policy for Agibot X2.

The WBC policy speaks Unitree hg DDS protocol (trained on G1).  This bridge
publishes MuJoCo ground truth over the same DDS topics so the policy receives
the three inputs it needs:

  1. Joint state   — rt/lowstate       (q, dq, tau_est from motor encoders)
  2. IMU state     — rt/lowstate.imu_state + rt/secondary_imu  (pelvis + torso)
  3. Odometry      — rt/odostate       (world-frame position/velocity, computed
                                        from MuJoCo ground truth in sim; on real
                                        X2 hardware this would come from a state
                                        estimator feeding into a ROS2-to-DDS adapter)

joint_remap[g1_dds_slot] = x2_joint_index translates commands and state
between G1's 29-slot DDS layout and X2's 31-DOF MuJoCo model.  The two X2
head joints (indices 29-30) have no G1 slot and receive zero torque.

Keyboard-only: no joystick/wireless-controller code.
"""

import threading
from typing import Dict, Tuple

import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__HandCmd_ as HandCmd_default,
    unitree_hg_msg_dds__HandState_ as HandState_default,
    unitree_hg_msg_dds__IMUState_ as IMUState_default,
    unitree_hg_msg_dds__LowCmd_,
    unitree_hg_msg_dds__LowState_ as LowState_default,
    unitree_hg_msg_dds__OdoState_ as OdoState_default,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    HandCmd_,
    HandState_,
    IMUState_,
    LowCmd_,
    LowState_,
    OdoState_,
)


class X2Bridge:
    """Sim DDS bridge for Agibot X2.

    Reads MuJoCo ground truth from the obs dict and publishes over Unitree hg
    DDS topics so the G1 WBC policy can drive X2 without modification.
    """

    # G1[slot] → X2[joint_index]
    JOINT_REMAP = np.array([
         0,  1,  2,  3,  4,  5,   # G1[0-5]  → X2[0-5]  left leg
         6,  7,  8,  9, 10, 11,   # G1[6-11] → X2[6-11] right leg
        12,                        # G1[12] waist_yaw   → X2[12]
        14,                        # G1[13] waist_roll  → X2[14]
        13,                        # G1[14] waist_pitch → X2[13]
        15, 16, 17, 18,            # G1[15-18] left shoulder + elbow → X2[15-18]
        21,                        # G1[19] left_wrist_roll  → X2[21]
        20,                        # G1[20] left_wrist_pitch → X2[20]
        19,                        # G1[21] left_wrist_yaw   → X2[19]
        22, 23, 24, 25,            # G1[22-25] right shoulder + elbow → X2[22-25]
        28,                        # G1[26] right_wrist_roll  → X2[28]
        27,                        # G1[27] right_wrist_pitch → X2[27]
        26,                        # G1[28] right_wrist_yaw   → X2[26]
    ])

    def __init__(self, config):
        self.num_body_motor = 29    # WBC speaks 29-slot G1 protocol
        self.num_hand_motor = config.get("NUM_HAND_MOTORS", 0)
        self.use_sensor = config["USE_SENSOR"]

        self.joint_remap = self.JOINT_REMAP.copy()

        # G1 wrist PD gains cause oscillation on X2's lighter actuators.
        # Zero wrist slots so they float passively until gains are tuned.
        self.gain_scale = np.ones(29)
        self.gain_scale[19:22] = 0.0   # left wrist  (G1 slots 19-21)
        self.gain_scale[26:29] = 0.0   # right wrist (G1 slots 26-28)

        self.joystick = None  # keyboard-only

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()

        # --- publishers ---
        self.low_state = LowState_default()
        self.low_state_puber = ChannelPublisher("rt/lowstate", LowState_)
        self.low_state_puber.Init()

        self.odo_state = OdoState_default()
        self.odo_state_puber = ChannelPublisher("rt/odostate", OdoState_)
        self.odo_state_puber.Init()

        self.torso_imu_state = IMUState_default()
        self.torso_imu_puber = ChannelPublisher("rt/secondary_imu", IMUState_)
        self.torso_imu_puber.Init()

        self.left_hand_state = HandState_default()
        self.left_hand_state_puber = ChannelPublisher("rt/dex3/left/state", HandState_)
        self.left_hand_state_puber.Init()

        self.right_hand_state = HandState_default()
        self.right_hand_state_puber = ChannelPublisher("rt/dex3/right/state", HandState_)
        self.right_hand_state_puber.Init()

        # --- subscribers ---
        self.low_cmd_suber = ChannelSubscriber("rt/lowcmd", LowCmd_)
        self.low_cmd_suber.Init(self.LowCmdHandler, 1)

        self.left_hand_cmd = HandCmd_default()
        self.left_hand_cmd_suber = ChannelSubscriber("rt/dex3/left/cmd", HandCmd_)
        self.left_hand_cmd_suber.Init(self.LeftHandCmdHandler, 1)

        self.right_hand_cmd = HandCmd_default()
        self.right_hand_cmd_suber = ChannelSubscriber("rt/dex3/right/cmd", HandCmd_)
        self.right_hand_cmd_suber.Init(self.RightHandCmdHandler, 1)

        self.low_cmd_lock = threading.Lock()
        self.left_hand_cmd_lock = threading.Lock()
        self.right_hand_cmd_lock = threading.Lock()

        self.reset()

    def reset(self):
        with self.low_cmd_lock:
            self.low_cmd_received = False
            self.new_low_cmd = False
        with self.left_hand_cmd_lock:
            self.left_hand_cmd_received = False
            self.new_left_hand_cmd = False
        with self.right_hand_cmd_lock:
            self.right_hand_cmd_received = False
            self.new_right_hand_cmd = False

    def LowCmdHandler(self, msg):
        with self.low_cmd_lock:
            self.low_cmd = msg
            self.low_cmd_received = True
            self.new_low_cmd = True

    def LeftHandCmdHandler(self, msg):
        with self.left_hand_cmd_lock:
            self.left_hand_cmd = msg
            self.left_hand_cmd_received = True
            self.new_left_hand_cmd = True

    def RightHandCmdHandler(self, msg):
        with self.right_hand_cmd_lock:
            self.right_hand_cmd = msg
            self.right_hand_cmd_received = True
            self.new_right_hand_cmd = True

    def cmd_received(self) -> bool:
        with self.low_cmd_lock:
            low = self.low_cmd_received
        with self.left_hand_cmd_lock:
            left = self.left_hand_cmd_received
        with self.right_hand_cmd_lock:
            right = self.right_hand_cmd_received
        return low or left or right

    def PublishLowState(self, obs: Dict[str, any]):
        if self.use_sensor:
            raise NotImplementedError("USE_SENSOR not implemented for X2.")

        # Joint state — from MuJoCo motor encoders (ground truth)
        for i in range(self.num_body_motor):
            self.low_state.motor_state[i].q = obs["body_q"][i]
            self.low_state.motor_state[i].dq = obs["body_dq"][i]
            self.low_state.motor_state[i].ddq = obs["body_ddq"][i]
            self.low_state.motor_state[i].tau_est = obs["body_tau_est"][i]

        # Odometry — world-frame position/velocity from MuJoCo ground truth
        self.odo_state.position[:] = obs["floating_base_pose"][:3]
        self.odo_state.linear_velocity[:] = obs["floating_base_vel"][:3]
        self.odo_state.orientation[:] = obs["floating_base_pose"][3:7]
        self.odo_state.angular_velocity[:] = obs["floating_base_vel"][3:6]

        # Pelvis IMU — embedded in lowstate
        self.low_state.imu_state.quaternion[:] = obs["floating_base_pose"][3:7]
        self.low_state.imu_state.gyroscope[:] = obs["floating_base_vel"][3:6]
        self.low_state.imu_state.accelerometer[:] = obs["floating_base_acc"][:3]

        # Torso (secondary) IMU
        self.torso_imu_state.quaternion[:] = obs["secondary_imu_quat"]
        self.torso_imu_state.gyroscope[:] = obs["secondary_imu_vel"][3:6]

        self.low_state.tick = int(obs["time"] * 1e3)
        self.low_state_puber.Write(self.low_state)

        self.odo_state.tick = int(obs["time"] * 1e3)
        self.odo_state_puber.Write(self.odo_state)

        self.torso_imu_puber.Write(self.torso_imu_state)

        # Hand state
        for i in range(self.num_hand_motor):
            self.left_hand_state.motor_state[i].q = obs["left_hand_q"][i]
            self.left_hand_state.motor_state[i].dq = obs["left_hand_dq"][i]
        self.left_hand_state_puber.Write(self.left_hand_state)

        for i in range(self.num_hand_motor):
            self.right_hand_state.motor_state[i].q = obs["right_hand_q"][i]
            self.right_hand_state.motor_state[i].dq = obs["right_hand_dq"][i]
        self.right_hand_state_puber.Write(self.right_hand_state)

    def GetAction(self) -> Tuple[np.ndarray, bool, bool]:
        with self.low_cmd_lock:
            body_q = [self.low_cmd.motor_cmd[i].q for i in range(self.num_body_motor)]
        with self.left_hand_cmd_lock:
            left_hand_q = [self.left_hand_cmd.motor_cmd[i].q for i in range(self.num_hand_motor)]
        with self.right_hand_cmd_lock:
            right_hand_q = [self.right_hand_cmd.motor_cmd[i].q for i in range(self.num_hand_motor)]
        with self.low_cmd_lock and self.left_hand_cmd_lock and self.right_hand_cmd_lock:
            is_new_action = self.new_low_cmd and self.new_left_hand_cmd and self.new_right_hand_cmd
            if is_new_action:
                self.new_low_cmd = False
                self.new_left_hand_cmd = False
                self.new_right_hand_cmd = False

        return (
            np.concatenate([body_q[:-7], left_hand_q, body_q[-7:], right_hand_q]),
            self.cmd_received(),
            is_new_action,
        )
