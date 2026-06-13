# Adding a New Robot to the Simulation (No Retraining)

This tutorial walks through everything required to drive a **new robot** in
the MuJoCo sim loop (`run_sim_loop.py`) using an **existing G1-trained SONIC
policy** — the same approach used to bring up Agibot X2. No new policy
training is involved: the policy continues to speak the 29-slot Unitree hg
DDS protocol over local DDS, and a per-robot **bridge** translates between
that protocol and your robot's actual joint layout inside the sim.

```{admonition} Not training a new policy?
:class: important
This guide is for reusing a **G1-trained policy** on a different robot body
via DDS remapping in `base_sim.py` — useful for quickly bringing up a new
robot in sim. If you instead want to **train a new SONIC policy from scratch**
for your robot's own morphology, see
[Training on New Embodiments](../user_guide/new_embodiments.md) — that is an
entirely separate (Isaac Lab) pipeline and does not share files with this guide.
```

## Overview

`base_sim.py` talks to the WBC policy entirely over local DDS, using G1's
29-slot `LowCmd`/`LowState` convention regardless of which robot is actually
being simulated. To add a new robot you need to:

1. Add the robot's URDF/MJCF/mesh assets and register their paths.
2. Create a WBC config YAML describing the robot's joints, gains, and limits.
3. Write a DDS bridge that maps G1's 29 slots onto your robot's joints.
4. Register the bridge so `base_sim.py` can find it.
5. Add a `RobotSupplementalInfo` + instantiation factory for the MuJoCo model.

Everything else (sim stepping, elastic band, image publishing, joystick,
etc.) is shared and config-driven — no changes needed there.

## Step 1: Robot Assets

Place URDF, meshes, and MJCF/scene XML under `resources/<robot>/`, following
the layout of `resources/x2/` or `resources/g1/`:

```
resources/<robot>/
├── main.urdf            # URDF used by RobotModel
├── meshes/              # STL/OBJ mesh files
├── <robot>.xml          # MuJoCo MJCF for the robot
└── scene.xml            # MuJoCo scene (ground plane, lighting, includes <robot>.xml)
```

Register the paths in [robot_paths.yaml](../../../robot_paths.yaml) so both
the simulator and the robot model factory can find them without hardcoded
paths:

```yaml
robots:
  <robot>:
    urdf: "resources/<robot>/main.urdf"
    asset_path: "resources/<robot>"
    scene: "resources/<robot>/scene.xml"
    meshes: "resources/<robot>/meshes"
    mjcf: "resources/<robot>/<robot>.xml"
```

## Step 2: WBC Config YAML

Create `gear_sonic/utils/mujoco_sim/wbc_configs/<robot>_sonic_model12.yaml`,
using `x2_31dof_sonic_model12.yaml` as a template. Key fields:

```yaml
ROBOT_TYPE: '<robot>_<n>dof'
ROBOT_SCENE: "resources/<robot>/scene.xml"

BRIDGE_CLASS: "<Robot>Bridge"   # name of the bridge class, see Step 3
BAND_ATTACHED_LINK: "pelvis"    # body the elastic band attaches to
# Only needed if the robot can run with enable_waist=False:
BAND_ATTACHED_LINK_NO_WAIST: "torso_link"

ENABLE_ELASTIC_BAND: True
USE_SENSOR: False
...
```

Also fill in:

- `MOTOR2JOINT` / `JOINT2MOTOR` — identity arrays sized to your robot's DOF count, unless your MuJoCo joint order differs from your motor order.
- `JOINT_KP` / `JOINT_KD` and `MOTOR_KP` / `MOTOR_KD` — PD gains per joint. Start conservative for any joints with lighter actuators than G1.
- `DEFAULT_DOF_ANGLES` / `DEFAULT_MOTOR_ANGLES` — the pose the robot starts in.
- `motor_pos_lower_limit_list` / `motor_pos_upper_limit_list` / `motor_vel_limit_list` / `motor_effort_limit_list` — pulled directly from your URDF.
- `NUM_MOTORS`, `NUM_JOINTS`, `NUM_HAND_MOTORS`, `NUM_HAND_JOINTS`, `NUM_UPPER_BODY_JOINTS`.
- `history_config` / `obs_dims` / etc. — copy from the X2 config and adjust dimensions (`dof_pos`, `dof_vel`, `actions`, `ref_upper_dof_pos`) to match your robot's joint count.

## Step 3: Write the DDS Bridge

Create `gear_sonic/utils/mujoco_sim/<robot>_bridge.py`. The bridge's job is
to publish MuJoCo ground truth as `rt/lowstate` (and odometry/IMU topics) and
consume `rt/lowcmd`, translating between G1's 29-slot convention and your
robot's joint indices. Use `x2_bridge.py` as a template — it implements all
of the DDS plumbing (publishers/subscribers, hand state, odometry, IMU).

The three translation mechanisms a bridge needs:

### `JOINT_REMAP` — index mapping

`JOINT_REMAP[g1_slot] = <robot>_joint_index` maps each of G1's 29 DDS slots
to the corresponding joint index in your robot's MuJoCo model. If a body
joint exists in the same anatomical position for both robots but at a
different DOF index, list it here. Joints your robot has that G1 doesn't
(e.g. a head) simply have no G1 slot and aren't driven by the policy.

```python
JOINT_REMAP = np.array([
     0,  1,  2,  3,  4,  5,   # G1[0-5]  → left leg
     6,  7,  8,  9, 10, 11,   # G1[6-11] → right leg
    ...
])
```

### `gain_scale` — per-slot PD gain multiplier

If some joints on your robot use much lighter actuators than G1 (e.g. wrists)
and the G1-tuned PD gains cause oscillation, zero or scale those slots:

```python
self.gain_scale = np.ones(29)
self.gain_scale[19:22] = 0.0   # left wrist  (G1 slots 19-21)
self.gain_scale[26:29] = 0.0   # right wrist (G1 slots 26-28)
```

Joints with `gain_scale = 0` float passively in `compute_body_torques` (only
`tau_ff` from the policy is applied, with no PD correction).

### `JOINT_OFFSET` — zero-pose correction

Different robots can define "zero" for the same joint differently — e.g. X2's
mechanical zero for shoulder-roll/elbow corresponds to a visibly different arm
pose than G1's zero. Without correction, a G1-trained motion will look
correct on G1 but bent/twisted on your robot in sim.

`JOINT_OFFSET[g1_slot]` is a per-slot radian offset applied symmetrically by
`base_sim.py`:

- **State → policy** (`PublishLowState`): the bridge subtracts the offset
  from the robot's measured `q` so the policy sees angles in G1's convention.
- **Policy → robot** (`compute_body_torques` / `compute_body_qpos` in
  `base_sim.py`): the offset is added back to the policy's commanded `q_des`
  before computing PD torques / target qpos.

```python
JOINT_OFFSET = np.zeros(29)
JOINT_OFFSET[16] = -0.061      # left_shoulder_roll
JOINT_OFFSET[18] = -np.pi / 2  # left_elbow
JOINT_OFFSET[23] = 0.061       # right_shoulder_roll
JOINT_OFFSET[25] = -np.pi / 2  # right_elbow
```

```{admonition} Finding offset values empirically
:class: tip
1. Load both robots' MJCF in MuJoCo at `qpos = 0` (`mj_resetData` + `mj_forward`).
2. For each arm segment (e.g. upper arm, forearm), compute its direction
   vector in the torso-local frame for both robots.
3. Use `scipy.optimize.minimize` (L-BFGS-B) to find the per-joint offset that
   rotates your robot's segment direction to match G1's, **bounded by your
   robot's actual joint limits** (`motor_pos_lower/upper_limit_list`).
4. If the unconstrained optimum is infeasible (outside the joint range),
   clamp to the nearest feasible bound and accept the residual error — verify
   visually by rendering both poses (offscreen `Renderer`) with the offsets
   applied.
```

In `__init__`, copy both arrays so they can be mutated per-instance if needed:

```python
self.joint_remap = self.JOINT_REMAP.copy()
self.joint_offset = self.JOINT_OFFSET.copy()
```

If your robot's zero pose already matches G1's, you can skip `JOINT_OFFSET`
entirely — `base_sim.py` treats `joint_offset = None` as all-zero offsets
(see `unitree_sdk2py_bridge.py`, used for G1 itself).

## Step 4: Register the Bridge

Add your bridge class to `BRIDGE_REGISTRY` in
[base_sim.py](../../../gear_sonic/utils/mujoco_sim/base_sim.py):

```python
from gear_sonic.utils.mujoco_sim.<robot>_bridge import <Robot>Bridge

BRIDGE_REGISTRY = {
    "UnitreeSdk2Bridge": UnitreeSdk2Bridge,
    "X2Bridge": X2Bridge,
    "<Robot>Bridge": <Robot>Bridge,
}
```

`base_sim.py` selects the bridge class via `config["BRIDGE_CLASS"]` (set in
Step 2), so no other branching logic is needed — `init_unitree_bridge`,
`compute_body_torques`, `compute_body_qpos`, and the elastic band attachment
all read from `self.config` and `self.unitree_bridge.*` generically.

## Step 5: Robot Model for the Motion/FK Side

Create `gear_sonic/data/robot_model/supplemental_info/<robot>/<robot>_supplemental_info.py`
with a `RobotSupplementalInfo` subclass listing `body_actuated_joints`, joint
limits, and default poses — use `x2_supplemental_info.py` as a template.

Then add `gear_sonic/data/robot_model/instantiation/<robot>.py`:

```python
from gear_sonic.data.robot_model.robot_model import RobotModel
from gear_sonic.data.robot_model.supplemental_info.<robot>.<robot>_supplemental_info import (
    <Robot>SupplementalInfo,
)
from gear_sonic.utils.robot_paths import get_robot_paths


def instantiate_<robot>_robot_model():
    paths = get_robot_paths("<robot>")
    return RobotModel(
        str(paths["urdf"]),
        str(paths["asset_path"]),
        set_floating_base=True,
        supplemental_info=<Robot>SupplementalInfo(),
    )
```

```{admonition} run_sim_loop.py currently hardcodes X2
:class: warning
[run_sim_loop.py](../../../gear_sonic/scripts/run_sim_loop.py) currently calls
`instantiate_x2_robot_model()` unconditionally. To add another robot, either
branch on `wbc_config["ROBOT_TYPE"]` / `BRIDGE_CLASS` to pick the right
`instantiate_<robot>_robot_model()`, or extend `robot_paths.yaml` /
`SimLoopConfig` so the robot model factory can be selected generically.
```

## Step 6: Run It

```bash
source .venv_sim/bin/activate
python gear_sonic/scripts/run_sim_loop.py --wbc_version <robot>_sonic_model12
```

If the robot's limbs are twisted or oscillating relative to G1's playback,
revisit `gain_scale` (oscillation) and `JOINT_OFFSET` (twisted/rotated limbs)
in your bridge.

## Going Further: Training a Native Policy

The G1-trained policy with DDS remapping gets a new robot moving quickly, but
joint limits, link lengths, and mass distribution differences mean the
motion won't be optimal. To train SONIC natively for your robot's
morphology (a separate pipeline, not covered here), see
[Training on New Embodiments](../user_guide/new_embodiments.md).
