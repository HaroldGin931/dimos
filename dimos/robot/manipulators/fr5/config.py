# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The FAIRINO FR5 v6 model and joint position interface."""

import math
from pathlib import Path

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.contract.keys import Interface
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.gripper import GripperConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.manipulators._modeling import joint_names

_SOURCE = RobotDescriptionSource(
    url="https://github.com/FAIR-INNOVATION/frcobot_ros2",
    ref="eb6d7ce40071aeb1a0c06e40e95f3be46bd7594e",
)
_DESCRIPTION = _SOURCE / "fairino_description"
_JOINT_NAMES = joint_names(6, prefix="j")
FR5_JOINT_NAMES = tuple(f"arm/{joint}" for joint in _JOINT_NAMES)
# Home: folded over the base on the working side, in whole degrees. The elbow
# is 5 degrees inside the controller's 160 degree soft limit and j6 is off its
# 175 degree limit. The wrist (j5) is at +90: far from its singular zero, where
# a straight-line jog spins joints 4 and 6, and with the flange facing up, away
# from the base.
FR5_HOME_DEG = (155.0, -160.0, 155.0, -85.0, 90.0, 165.0)
# The controller's soft limits, j1 to j6, in degrees. It faults (3/n) as joint n
# reaches one, and then takes no command until the joint is dragged back.
FR5_SOFT_LIMITS_DEG = (
    (-175.0, 175.0),
    (-265.0, 85.0),
    (-160.0, 160.0),
    (-265.0, 85.0),
    (-175.0, 175.0),
    (-175.0, 175.0),
)
# How fast any joint may be told to turn, by a jog, a Viser joint slider or a
# planned trajectory; far below the URDF's 3.15 rad/s. At 1 rad/s a jog near a
# singular pose, turning several joints at once, set off the controller's
# collision detection with nothing touched.
FR5_MAX_JOINT_VELOCITY_RAD_S = 0.3
# How far inside the soft limits the model's limits are, so that no jog or
# planned trajectory is sent to one.
FR5_LIMIT_MARGIN_DEG = 2.0

# The gripper is DH-Robotics' PGI-140-80, which the controller drives over the
# tool-end RS-485. Its one driven joint: how far each finger is from closed, in metres.
# Its range and the gripper's shape are in pgi140.urdf.
FR5_GRIPPER_JOINT = "arm/gripper"
# Where the flange's face is along wrist3_link's z, in metres: the far end of
# FAIRINO's wrist3 mesh.
_FLANGE_Z_M = 0.099
# From the flange's face to the gripper's mounting face, in metres: the adapter
# plate between them.
FR5_GRIPPER_MOUNT_M = 0.007
# The gripper is not turned about the flange's z: its fingers open along the
# flange's x, with its cable socket on the +x side.
# Gripper parts that touch or overlap as built.
_GRIPPER_COLLISION_EXCLUSIONS = [
    ("wrist3_link", "gripper_base"),
    ("gripper_base", "gripper_finger1"),
    ("gripper_base", "gripper_finger2"),
    ("gripper_finger1", "gripper_finger2"),
]
# The gripper's joints that move: the one driven, then the finger that copies it.
_GRIPPER_JOINTS = ("gripper", "gripper_finger2_joint")
_GRIPPER_MODEL = RobotModel.from_file(Path(__file__).parent / "pgi140.urdf")


def fr5_model(gripper: bool = False, *, robot: str | None = None) -> RobotModel:
    """The FR5 v6 URDF, from FAIRINO's frcobot_ros2 description, with its joint
    limits ``FR5_LIMIT_MARGIN_DEG`` inside the controller's soft limits.

    Args:
        gripper: With the PGI-140-80 gripper on its flange: the links
            ``gripper_base``, ``gripper_finger1``, ``gripper_finger2`` and
            ``gripper_tcp``, and the joint ``gripper`` (0 m closed, 0.04 m open).
        robot: Name the joints under this robot, e.g. "arm" gives "arm/j1".
            Links keep their names.
    """
    model = RobotModel.from_file(
        _DESCRIPTION / "urdf/fairino5_v6.urdf",
        package_paths={"fairino_description": _DESCRIPTION},
    )
    for joint, (lower, upper) in zip(_JOINT_NAMES, FR5_SOFT_LIMITS_DEG, strict=True):
        model = model.with_joint_position_limits(
            joint,
            lower=math.radians(lower + FR5_LIMIT_MARGIN_DEG),
            upper=math.radians(upper - FR5_LIMIT_MARGIN_DEG),
        )
    moving = list(_JOINT_NAMES)
    if gripper:
        model = model.with_attached_model(
            _GRIPPER_MODEL,
            parent_link="wrist3_link",
            joint_name="gripper_mount",
            xyz=(0.0, 0.0, _FLANGE_Z_M + FR5_GRIPPER_MOUNT_M),
        )
        moving.extend(_GRIPPER_JOINTS)
    if not robot:
        return model
    return model.with_renamed_joints({joint: f"{robot}/{joint}" for joint in moving})


def fr5_coordinator_connection(gripper: bool = False) -> ConnectionConfig:
    """What a coordinator needs to know about the FR5 connection: its model, with
    the joints ``arm/j1`` ... ``arm/j6``, and ``arm/gripper`` with a gripper,
    taking positions."""
    names = [*FR5_JOINT_NAMES, *([FR5_GRIPPER_JOINT] if gripper else [])]
    return ConnectionConfig(
        source="arm",
        model=fr5_model(gripper, robot="arm"),
        joint_commands={name: (Interface.POSITION,) for name in names},
    )


def _gripper_config() -> GripperConfig:
    """Which gripper links may touch a grasped object, and its open and closed
    positions: the ends of the joint ``gripper``'s range in pgi140.urdf."""
    # Read from the gripper's own file: loading the arm's model fetches it.
    [joint] = [joint for joint in _GRIPPER_MODEL.load().joints if joint.name == "gripper"]
    assert joint.lower is not None and joint.upper is not None
    return GripperConfig(
        pad_links=("gripper_finger1", "gripper_finger2"),
        finger_joints={FR5_GRIPPER_JOINT: (joint.upper, joint.lower)},
    )


def make_fr5_model_config(gripper: bool = False) -> RobotModelConfig:
    """Configure the FR5 planning group with the coordinator's joint names, to
    plan no faster than ``FR5_MAX_JOINT_VELOCITY_RAD_S``.

    Args:
        gripper: With the gripper on its flange; see ``fr5_model``. The group's
            tip is then ``gripper_tcp``, between the fingertips.
    """
    model = fr5_model(gripper, robot="arm")
    return RobotModelConfig(
        # The vendor's MoveIt configuration supplies 0.7 rad/s² for each axis.
        model=model.with_joint_velocity_limits(
            dict.fromkeys(FR5_JOINT_NAMES, FR5_MAX_JOINT_VELOCITY_RAD_S)
        ).with_default_joint_acceleration_limit(0.7),
        srdf_path=_SOURCE / "fairino5_v6_moveit2_config/config/fairino5_v6_robot.srdf",
        joint_names=list(FR5_JOINT_NAMES),
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=FR5_JOINT_NAMES,
                base_link="base_link",
                tip_link="gripper_tcp" if gripper else "wrist3_link",
            )
        ],
        collision_exclusion_pairs=_GRIPPER_COLLISION_EXCLUSIONS if gripper else [],
        gripper=_gripper_config() if gripper else None,
        home_joints=[math.radians(angle) for angle in FR5_HOME_DEG],
    )
