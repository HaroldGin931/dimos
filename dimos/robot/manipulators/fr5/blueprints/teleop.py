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

"""Keyboard teleoperation of one FAIRINO FR5 v6 with WebAPP 3.9.5, in a pygame window.

DimOS fetches the SDK and model on first run. Enable the arm locally, then
`dimos run keyboard-teleop-fr5` jogs the wrist from the
keyboard, and plans and runs trajectories from the Viser UI (the planning
extra); a jog takes over from a running trajectory. With `--simulation`, a mock
connection that echoes its targets stands in for the arm.

Nothing here stops a jog at the edge of the arm's reach or before the arm
meets itself: keep jogs short and watch the arm.
"""

from __future__ import annotations

import math

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.contract.limits import movable_joints
from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, MetaBlueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.module import ModuleBase
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.robot.manipulators.fr5.config import (
    FR5_HOME_DEG,
    FR5_JOINT_NAMES,
    FR5_MAX_JOINT_VELOCITY_RAD_S,
    fr5_coordinator_connection,
    fr5_model,
    make_fr5_model_config,
)
from dimos.robot.manipulators.fr5.connection import LINK_TIMEOUT_S, SERVO_PERIOD_S, FR5Connection
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

_fr5_model = make_fr5_model_config()


class FR5TwistCoordinator(ArmTwistCoordinator):
    """End-effector twists in, from the keyboard; one FR5 named "arm" out,
    driven through ``FR5Connection`` by joint targets."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


def fr5_connection() -> Blueprint:
    """``FR5Connection``, or a mock that echoes its targets when ``--simulation`` is on."""
    if global_config.simulation:
        # The mock names each model joint under its source, so it takes them bare.
        model = fr5_model()
        return MockJointConnection.blueprint(
            source="arm",
            model=model,
            state_rate_hz=1 / SERVO_PERIOD_S,
            # At zero the wrist is singular, where a straight jog spins j4 and j6.
            initial_positions={
                joint.name: math.radians(angle)
                for joint, angle in zip(movable_joints(model), FR5_HOME_DEG, strict=True)
            },
        )
    return FR5Connection.blueprint()


def fr5_remaps() -> list[tuple[type[ModuleBase], str, str]]:
    """The connection's ports, onto the coordinator's ports for the robot named "arm"."""
    connection: type[ModuleBase] = (
        MockJointConnection if global_config.simulation else FR5Connection
    )
    return [
        (connection, "position_command", "arm_position_command"),
        (FR5TwistCoordinator, "arm_joint_state", "joint_state"),
    ]


def _keyboard_teleop() -> MetaBlueprint:
    """Jog an FR5's wrist from this computer's keyboard, and plan its motions.

    The coordinator names the joints ``arm/j1`` ... ``arm/j6``. Wire it up with
    ``.remappings(fr5_remaps())``. An arm away from FAIRINO's default address is
    given at run time, after the blueprint name (``--fr5connection.ip <IP>``)
    or as ``FR5CONNECTION__IP``.
    """
    return autoconnect(
        KeyboardTeleopModule.blueprint(),
        FR5TwistCoordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[fr5_coordinator_connection()],
            tick_rate=1 / SERVO_PERIOD_S,
            reading_timeout_s=LINK_TIMEOUT_S,
            tasks=[
                # Move the wrist by the twists that come in on ``ee_twist_command``.
                # The task goes idle 0.3 s after the last twist; the connection's
                # deadman then ends the servo session, and the next twist starts another.
                TaskConfig(
                    name=EEF_TWIST_TASK_NAME,
                    type="eef_twist",
                    joint_names=list(_fr5_model.joint_names),
                    # Above the trajectory task, so a jog takes over from a running trajectory.
                    priority=20,
                    params={
                        "robot_model": _fr5_model,
                        "target_frame": "wrist3_link",
                        "max_joint_velocity_rad_s": FR5_MAX_JOINT_VELOCITY_RAD_S,
                    },
                ),
                # Planned trajectories and the Viser joint sliders.
                joint_trajectory_task(
                    FR5_JOINT_NAMES,
                    priority=10,
                    velocity_limits=dict.fromkeys(FR5_JOINT_NAMES, FR5_MAX_JOINT_VELOCITY_RAD_S),
                ),
            ],
        ),
        fr5_connection(),
        planner(model=_fr5_model, visualization={"backend": "viser"}),
    )


# The blueprint registry finds a blueprint only by a line ending in a call like `.remappings`.
keyboard_teleop_fr5 = _keyboard_teleop().remappings(fr5_remaps())
