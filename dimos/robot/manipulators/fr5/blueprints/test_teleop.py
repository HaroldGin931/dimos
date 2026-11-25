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

import math
from typing import Any
from unittest.mock import Mock

import pytest

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.contract.limits import movable_joints
from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, MetaBlueprint
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.manipulators.fr5.blueprints.basic import (
    FR5Coordinator,
    fr5_coordinator,
    fr5_remaps,
)
from dimos.robot.manipulators.fr5.blueprints.teleop import (
    FR5_JOG_MAX_JOINT_VELOCITY_RAD_S,
    FR5TwistCoordinator,
    keyboard_teleop_fr5,
)
from dimos.robot.manipulators.fr5.config import (
    FR5_HOME_DEG,
    FR5_JOINT_NAMES,
    fr5_coordinator_connection,
    make_fr5_model_config,
)
from dimos.robot.manipulators.fr5.connection import FR5Connection
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule
from dimos.utils.testing.waiting import wait_until


def _modules(blueprint: MetaBlueprint) -> tuple[Blueprint, dict[type, dict[str, Any]]]:
    solid = blueprint.solidify()
    return solid, {atom.module: atom.kwargs for atom in solid.blueprints}


def _tasks(modules: dict[type, dict[str, Any]]) -> dict[str, TaskConfig]:
    return {task.type: task for task in modules[FR5TwistCoordinator]["tasks"]}


def test_keyboard_jogs_the_wrist_through_the_connection() -> None:
    blueprint, modules = _modules(keyboard_teleop_fr5)
    tasks = _tasks(modules)

    assert set(modules) == {
        KeyboardTeleopModule,
        FR5TwistCoordinator,
        FR5Connection,
        ManipulationModule,
    }
    assert tasks["eef_twist"].joint_names == list(FR5_JOINT_NAMES)
    assert tasks["trajectory"].joint_names == list(FR5_JOINT_NAMES)
    # A jog takes over from a planned trajectory.
    assert tasks["eef_twist"].priority > tasks["trajectory"].priority
    assert tasks["eef_twist"].params["target_frame"] == "wrist3_link"
    assert modules[FR5TwistCoordinator]["tick_rate"] == 125.0
    assert blueprint.remapping_map == {
        ("fr5connection", "position_command"): "arm_position_command",
        ("ControlCoordinator", "arm_joint_state"): "joint_state",
    }


def test_jog_speed_stays_below_the_speed_the_connection_refuses() -> None:
    model = fr5_coordinator_connection().model
    assert model is not None
    _, modules = _modules(keyboard_teleop_fr5)

    assert _tasks(modules)["eef_twist"].params["max_joint_velocity_rad_s"] == (
        FR5_JOG_MAX_JOINT_VELOCITY_RAD_S
    )
    assert all(
        joint.velocity is not None and FR5_JOG_MAX_JOINT_VELOCITY_RAD_S < joint.velocity
        for joint in movable_joints(model)
    )


def test_simulation_composes_a_mock_connection(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    blueprint = fr5_coordinator().remappings(fr5_remaps(FR5Coordinator)).solidify()

    assert not any(atom.module is FR5Connection for atom in blueprint.blueprints)
    [mock] = [atom for atom in blueprint.blueprints if atom.module is MockJointConnection]
    assert mock.kwargs["source"] == "arm"
    assert mock.kwargs["state_rate_hz"] == 125.0
    assert blueprint.remapping_map[(mock.name, "position_command")] == "arm_position_command"


class _LocalRPC(Mock, RPCSpec):
    """Keep module lifecycle calls in process."""


def test_simulated_arm_reports_the_joints_the_coordinator_expects(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    [mock] = [
        atom
        for atom in fr5_coordinator().solidify().blueprints
        if atom.module is MockJointConnection
    ]
    arm = MockJointConnection(**mock.kwargs, rpc_transport=_LocalRPC)
    arm.set_transport("position_command", Mock())
    readings: list[JointState] = []
    arm.joint_state.subscribe(readings.append)
    arm.start()
    try:
        wait_until(lambda: readings, timeout=1, interval=0.01)
    finally:
        arm.stop()

    assert readings[0].name == list(FR5_JOINT_NAMES)


def test_home_is_the_folded_pose_and_the_planner_accepts_it() -> None:
    pytest.importorskip("roboplan.core")
    config = make_fr5_model_config()
    assert config.home_joints is not None
    assert config.home_joints == pytest.approx([math.radians(angle) for angle in FR5_HOME_DEG])
    world = RoboPlanWorld()
    world.load_model(prepare_robot_model(config))
    world.finalize()

    assert world.check_config_collision_free(
        JointState(name=list(FR5_JOINT_NAMES), position=list(config.home_joints))
    )
