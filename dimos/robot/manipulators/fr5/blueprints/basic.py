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

"""Coordinator and planner blueprints for one FAIRINO FR5 v6 with WebAPP 3.9.5.

DimOS fetches the SDK and model on first run. Enable the arm locally, then run
`dimos run coordinator-fr5 --fr5connection.ip <IP>` for joint trajectories only,
or `fr5-planner-coordinator` to add planning and the Viser UI (the planning extra).
With `--simulation`, a mock connection that echoes its targets stands in for the arm.
"""

from __future__ import annotations

from collections.abc import Sequence

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, MetaBlueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.module import ModuleBase
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.fr5.config import (
    FR5_JOINT_NAMES,
    fr5_coordinator_connection,
    fr5_model,
    make_fr5_model_config,
)
from dimos.robot.manipulators.fr5.connection import LINK_TIMEOUT_S, SERVO_PERIOD_S, FR5Connection


class FR5Coordinator(ControlCoordinator):
    """Joint targets out to one FR5 named "arm", and its joints in, through
    ``FR5Connection``."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


def fr5_trajectory_task(*, priority: int = 10) -> TaskConfig:
    return joint_trajectory_task(FR5_JOINT_NAMES, priority=priority)


def fr5_connection() -> Blueprint:
    """``FR5Connection``, or a mock that echoes its targets when ``--simulation`` is on."""
    if global_config.simulation:
        # The mock names each model joint under its source, so it takes them bare.
        return MockJointConnection.blueprint(
            source="arm", model=fr5_model(), state_rate_hz=1 / SERVO_PERIOD_S
        )
    return FR5Connection.blueprint()


def fr5_remaps(coordinator: type[ModuleBase]) -> list[tuple[type[ModuleBase], str, str]]:
    """The connection's ports, onto ``coordinator``'s ports for the robot named "arm"."""
    connection: type[ModuleBase] = (
        MockJointConnection if global_config.simulation else FR5Connection
    )
    return [
        (connection, "position_command", "arm_position_command"),
        (coordinator, "arm_joint_state", "joint_state"),
    ]


def fr5_coordinator(
    coordinator: type[ControlCoordinator] = FR5Coordinator,
    tasks: Sequence[TaskConfig] | None = None,
) -> MetaBlueprint:
    """Run ``tasks`` (joint trajectories by default) on an FR5 named "arm".

    The coordinator names the joints ``arm/j1`` ... ``arm/j6``; a planner
    driving it uses ``make_fr5_model_config()``. Wire it up with
    ``.remappings(fr5_remaps(coordinator))``. The arm's address is given at run
    time, after the blueprint name (``--fr5connection.ip 192.168.58.2``) or as
    ``FR5CONNECTION__IP``.

    Args:
        coordinator: The coordinator class, with the input streams its tasks read.
        tasks: The coordinator's tasks.
    """
    return autoconnect(
        coordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[fr5_coordinator_connection()],
            tasks=list(tasks) if tasks is not None else [fr5_trajectory_task()],
            tick_rate=1 / SERVO_PERIOD_S,
            reading_timeout_s=LINK_TIMEOUT_S,
        ),
        fr5_connection(),
    )


# The blueprint registry finds a blueprint only by a line ending in a call like `.remappings`.
coordinator_fr5 = fr5_coordinator().remappings(fr5_remaps(FR5Coordinator))
fr5_planner_coordinator = autoconnect(
    planner(model=make_fr5_model_config(), visualization={"backend": "viser"}),
    fr5_coordinator(),
).remappings(fr5_remaps(FR5Coordinator))
