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

"""Keyboard teleoperation of one FAIRINO FR5 v6, in a pygame window.

`dimos run keyboard-teleop-fr5 --fr5connection.ip <IP>` jogs the wrist from the
keyboard, and plans and runs trajectories from the Viser UI; a jog takes over
from a running trajectory. Add `--simulation` to try it without the arm.

Nothing here stops a jog at the edge of the arm's reach or before the arm
meets itself: keep jogs short and watch the arm.
"""

from __future__ import annotations

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.robot.manipulators.fr5.blueprints.basic import (
    fr5_coordinator,
    fr5_remaps,
    fr5_trajectory_task,
)
from dimos.robot.manipulators.fr5.config import make_fr5_model_config
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

# How fast a jog may turn any joint, below the URDF's 3.15 rad/s.
FR5_JOG_MAX_JOINT_VELOCITY_RAD_S = 1.0

_fr5_model = make_fr5_model_config()


class FR5TwistCoordinator(ArmTwistCoordinator):
    """End-effector twists in, from the keyboard; one FR5 named "arm" out,
    driven through ``FR5Connection`` by joint targets."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


def _eef_twist_task() -> TaskConfig:
    """Move the FR5's wrist by the twists that come in on ``ee_twist_command``.

    The task goes idle 0.3 s after the last twist; the connection's deadman then
    ends the servo session, and the next twist starts another.
    """
    return TaskConfig(
        name=EEF_TWIST_TASK_NAME,
        type="eef_twist",
        joint_names=list(_fr5_model.joint_names),
        # Above the trajectory task, so a jog takes over from a running trajectory.
        priority=20,
        params={
            "robot_model": _fr5_model,
            "target_frame": "wrist3_link",
            "max_joint_velocity_rad_s": FR5_JOG_MAX_JOINT_VELOCITY_RAD_S,
        },
    )


# The blueprint registry finds a blueprint only by a line ending in a call like `.remappings`.
keyboard_teleop_fr5 = autoconnect(
    KeyboardTeleopModule.blueprint(),
    fr5_coordinator(FR5TwistCoordinator, [_eef_twist_task(), fr5_trajectory_task(priority=10)]),
    planner(model=_fr5_model, visualization={"backend": "viser"}),
).remappings(fr5_remaps(FR5TwistCoordinator))
