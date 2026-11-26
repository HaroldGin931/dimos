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
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.registry import control_task_registry
from dimos.core.coordination.blueprints import Blueprint, MetaBlueprint
from dimos.core.coordination.module_coordinator import resolve_module_refs
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.manipulators.fr5.blueprints.teleop import (
    FR5TwistCoordinator,
    _keyboard_teleop,
    fr5_remaps,
    keyboard_teleop_fr5,
)
from dimos.robot.manipulators.fr5.config import (
    FR5_HOME_DEG,
    FR5_JOINT_NAMES,
    FR5_MAX_JOINT_VELOCITY_RAD_S,
    FR5_SOFT_LIMITS_DEG,
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


def _simulated_teleop() -> MetaBlueprint:
    # Composed here, after --simulation is on: keyboard_teleop_fr5 was composed on import.
    return _keyboard_teleop().remappings(fr5_remaps())


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


def test_jogs_sliders_and_plans_all_keep_to_one_speed_below_what_the_connection_refuses() -> None:
    model = fr5_coordinator_connection().model
    assert model is not None
    _, modules = _modules(keyboard_teleop_fr5)
    tasks = _tasks(modules)
    slow = dict.fromkeys(FR5_JOINT_NAMES, FR5_MAX_JOINT_VELOCITY_RAD_S)

    assert tasks["eef_twist"].params["max_joint_velocity_rad_s"] == FR5_MAX_JOINT_VELOCITY_RAD_S
    # The Viser joint sliders and planned trajectories both run on this task.
    assert tasks["trajectory"].params["velocity_limits"] == slow
    assert {
        joint.name: joint.velocity for joint in movable_joints(make_fr5_model_config().model)
    } == slow
    assert all(
        joint.velocity is not None and FR5_MAX_JOINT_VELOCITY_RAD_S < joint.velocity
        for joint in movable_joints(model)
    )


def test_clear_errors_reaches_the_coordinator_to_drop_its_tasks() -> None:
    blueprint, _ = _modules(keyboard_teleop_fr5)
    [connection] = [atom for atom in blueprint.blueprints if atom.module is FR5Connection]

    assert resolve_module_refs(blueprint)[connection.name, "_commander"] == "ControlCoordinator"


def _state(positions: list[float], t_now: float) -> CoordinatorState:
    still = dict.fromkeys(FR5_JOINT_NAMES, 0.0)
    joints = JointStateSnapshot(
        joint_positions=dict(zip(FR5_JOINT_NAMES, positions, strict=True)),
        joint_velocities=still,
        joint_efforts=still,
        timestamp=t_now,
    )
    return CoordinatorState(joints=joints, t_now=t_now, dt=0.008)


@pytest.mark.parametrize("task_type", ["trajectory", "eef_twist"])
def test_a_target_from_before_clear_errors_is_not_followed_after_it(task_type: str) -> None:
    _, modules = _modules(keyboard_teleop_fr5)
    config = _tasks(modules)[task_type]
    task = control_task_registry.create(config.type, config, hardware={}, connections={})
    home = [math.radians(angle) for angle in FR5_HOME_DEG]
    # The arm was dragged 10 degrees from where it was told to go.
    dragged = [angle - math.radians(10.0) for angle in home]
    if task_type == "trajectory":
        # As the Viser joint sliders command it.
        assert task.on_joint_command(JointState(name=list(FR5_JOINT_NAMES), position=home), 1.0)
    else:
        twist = TwistStamped()
        twist.linear.x = 0.05
        assert task.on_ee_twist_command(twist, 1.0)
    moving = task.compute(_state(dragged, 1.008))
    assert moving is not None and moving.positions != pytest.approx(dragged)

    # What clear_errors() does to the coordinator's tasks.
    task.set_estop(True)
    task.set_estop(False)

    # Nothing is commanded, or the arm is held where it is.
    after = task.compute(_state(dragged, 1.016)) if task.is_active() else None
    assert after is None or after.positions == pytest.approx(dragged)


def test_no_command_reaches_a_soft_limit_where_the_controller_faults() -> None:
    model = fr5_coordinator_connection().model
    assert model is not None

    for joint, (lower, upper) in zip(movable_joints(model), FR5_SOFT_LIMITS_DEG, strict=True):
        assert joint.lower is not None and joint.upper is not None
        assert math.degrees(joint.lower) == pytest.approx(lower + 2.0)
        assert math.degrees(joint.upper) == pytest.approx(upper - 2.0)


def test_simulation_composes_a_mock_connection(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    blueprint = _simulated_teleop().solidify()

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
        for atom in _simulated_teleop().solidify().blueprints
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
    assert readings[0].position == pytest.approx([math.radians(angle) for angle in FR5_HOME_DEG])


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
