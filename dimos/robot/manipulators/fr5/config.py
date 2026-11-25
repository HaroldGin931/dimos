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

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.contract.keys import Interface
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
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


def fr5_model(*, robot: str | None = None) -> RobotModel:
    """The FR5 v6 URDF, from FAIRINO's frcobot_ros2 description.

    Args:
        robot: Name the joints under this robot, e.g. "arm" gives "arm/j1".
            Links keep their names.
    """
    model = RobotModel.from_file(
        _DESCRIPTION / "urdf/fairino5_v6.urdf",
        package_paths={"fairino_description": _DESCRIPTION},
    )
    if not robot:
        return model
    return model.with_renamed_joints({joint: f"{robot}/{joint}" for joint in _JOINT_NAMES})


_MODEL = fr5_model(robot="arm")


def fr5_coordinator_connection() -> ConnectionConfig:
    """What a coordinator needs to know about the FR5 connection: its model, with
    the joints ``arm/j1`` ... ``arm/j6`` taking positions."""
    return ConnectionConfig(
        source="arm",
        model=_MODEL,
        joint_commands={name: (Interface.POSITION,) for name in FR5_JOINT_NAMES},
    )


def make_fr5_model_config() -> RobotModelConfig:
    """Configure the FR5 planning group with the coordinator's joint names."""
    return RobotModelConfig(
        # The vendor's MoveIt configuration supplies 0.7 rad/s² for each axis.
        model=_MODEL.with_default_joint_acceleration_limit(0.7),
        srdf_path=_SOURCE / "fairino5_v6_moveit2_config/config/fairino5_v6_robot.srdf",
        joint_names=list(FR5_JOINT_NAMES),
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=FR5_JOINT_NAMES,
                base_link="base_link",
                tip_link="wrist3_link",
            )
        ],
        home_joints=[math.radians(angle) for angle in FR5_HOME_DEG],
    )
