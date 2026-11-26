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

"""Exercise FR5 joint streams against an in-process fake of the vendor SDK."""

from collections.abc import Iterator
from dataclasses import dataclass
from hashlib import sha256
from itertools import count
import math
from pathlib import Path
import threading
from types import SimpleNamespace
from unittest.mock import Mock, call
import xmlrpc.client

import pytest
from pytest_mock import MockerFixture
import requests
from requests_mock import Mocker

from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.assets.model import JointDescription, RobotModel
from dimos.robot.manipulators.fr5 import connection
from dimos.robot.manipulators.fr5.connection import FR5Connection
from dimos.utils.testing.waiting import wait_until

_IP = "192.0.2.1"
_JOINTS = [f"arm/j{i}" for i in range(1, 7)]
_MEASURED_DEG = [10.0, -15.0, 20.0, -25.0, 30.0, -35.0]
_TARGET_OFFSET_DEG = 0.001
_SPEED_DEG_S = [1.0, -2.0, 3.0, -4.0, 5.0, -6.0]
_TORQUE_NM = [0.5, -1.5, 2.5, -3.5, 4.5, -5.5]


class LocalRPC(Mock, RPCSpec):
    """Keep module lifecycle calls in process."""


@dataclass
class Rig:
    arm: FR5Connection
    sdk: Mock
    targets: Out[JointState]

    def move(self) -> None:
        # Reversed stream order must still reach the SDK in controller joint order.
        angles = [math.radians(angle + _TARGET_OFFSET_DEG) for angle in reversed(_MEASURED_DEG)]
        self.targets.publish(JointState(name=_JOINTS[::-1], position=angles))

    def ack(self, frame: int, command: int) -> None:
        callback = self.sdk.SetUDPCmdRpyCallback.call_args.args[0]
        callback(0, frame, command, 1, "1")

    def begin_motion(self) -> None:
        self.arm.start()
        self.move()
        self.ack(0, 689)
        self.move()


@pytest.fixture
def rig(monkeypatch: pytest.MonkeyPatch) -> Iterator[Rig]:
    model = RobotModel.from_root_link("base").with_added_joints(
        *(
            JointDescription(
                name, "revolute", "base", f"link{i}", lower=-math.pi, upper=math.pi, velocity=2.0
            )
            for i, name in enumerate(_JOINTS)
        )
    )
    sdk = Mock(is_connect=True, feedback_age=0.0)
    sdk.ResetAllError.return_value = sdk.RobotEnable.return_value = 0
    sdk.feedback = SimpleNamespace(
        jt_cur_pos=_MEASURED_DEG,
        actual_qd=_SPEED_DEG_S,
        jt_cur_tor=_TORQUE_NM,
        robot_mode=0,
        rbtEnableState=1,
        main_code=0,
        sub_code=0,
        EmergencyStop=0,
        collisionState=0,
        safety_stop0_state=0,
        safety_stop1_state=0,
    )
    frame = SimpleNamespace(_dimos_sample=None)
    ticks = count(59_984, 8)

    def next_frame() -> tuple[int, SimpleNamespace]:
        second, millisecond = divmod(next(ticks) % 60_000, 1000)
        sample = SimpleNamespace(second=second, millisecond=millisecond, **vars(sdk.feedback))
        frame._dimos_sample = (sample, connection.time.monotonic() - sdk.feedback_age)
        return 0, frame

    sdk.GetRobotRealTimeState.side_effect = next_frame
    sdk.GetRobotRealTimeState.return_value = (0, frame)
    sdk._udp_count = 0

    def send_udp(*_: object, **__: object) -> int:
        # Every UDP command takes the SDK's next frame number, as the vendor's does.
        sdk._udp_count += 1
        return 0

    sdk.ServoMoveStart.side_effect = sdk.ServoJ.side_effect = sdk.ServoMoveEnd.side_effect = (
        send_udp
    )
    monkeypatch.setattr(connection, "_open_sdk", Mock(return_value=sdk))
    monkeypatch.setattr(connection, "_STEADY_S", 0.1)
    monkeypatch.setattr(connection, "_SETTLE_S", 0.4)
    targets = Out(JointState, "targets")
    arm = FR5Connection(ip=_IP, model=model, deadman_timeout_s=2.0, rpc_transport=LocalRPC)
    arm.set_transport(
        "position_command", Mock(subscribe=lambda callback, _: targets.subscribe(callback))
    )
    yield Rig(arm, sdk, targets)
    arm.stop()


@pytest.mark.parametrize(
    ("model", "webapp"),
    [("FR5-WML", "v3.9.5"), ("FR5-V1-002(V6.0)", "v3.9.8")],
)
def test_rejects_an_unsupported_robot_before_opening_the_sdk(
    mocker: MockerFixture, requests_mock: Mocker, model: str, webapp: str
) -> None:
    load_sdk = mocker.patch.object(connection, "_load_sdk")
    xml = xmlrpc.client.dumps(([0, model, webapp, "V3.9.25-QX"],), methodresponse=True)
    requests_mock.post(f"http://{_IP}:{connection._SDK_RPC_PORT}/RPC2", text=xml)

    with pytest.raises(ValueError, match="expected FR5 v6 with WebAPP 3.9.5"):
        connection._open_sdk(_IP)

    load_sdk.assert_not_called()


def test_an_arm_that_does_not_answer_is_reported_with_what_to_check(
    mocker: MockerFixture, requests_mock: Mocker
) -> None:
    load_sdk = mocker.patch.object(connection, "_load_sdk")
    requests_mock.post(f"http://{_IP}:{connection._SDK_RPC_PORT}/RPC2", exc=requests.ConnectTimeout)

    with pytest.raises(ConnectionError, match=f"no FR5 answers at {_IP}: check the cable"):
        connection._open_sdk(_IP)

    load_sdk.assert_not_called()


def test_sdk_download_is_verified_and_reused_from_cache(
    mocker: MockerFixture, requests_mock: Mocker, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    source = tmp_path / "fairino" / connection._SDK_TAG / "Robot.py"
    content = (
        b"from dataclasses import dataclass\n@dataclass\nclass Version:\n    tag: str\n"
        b"print('vendor', 'status')\n"
    )
    mocker.patch.object(connection, "CACHE_DIR", tmp_path)
    mocker.patch.object(connection, "_SDK_SOURCE_SHA256", sha256(content).hexdigest())
    download = requests_mock.get(
        connection._SDK_URL,
        [{"content": b"raise AssertionError('Unverified SDK code ran')\n"}, {"content": content}],
    )
    debug = mocker.patch.object(connection.logger, "debug")

    with pytest.raises(ImportError, match="Unexpected FAIRINO SDK source"):
        connection._load_sdk()
    assert not source.exists()

    assert connection._load_sdk().Version("FR5").tag == "FR5"
    assert source.read_bytes() == content
    assert connection._load_sdk().Version("FR5").tag == "FR5"
    assert download.call_count == 2
    debug.assert_called_with("FAIRINO SDK", output="vendor status")
    assert capsys.readouterr().out == ""


def test_feedback_is_in_radians_and_targets_start_with_a_hold_in_degrees(rig: Rig) -> None:
    readings: list[JointState] = []
    rig.arm.joint_state.subscribe(readings.append)
    rig.begin_motion()
    rig.ack(1, 376)
    rig.move()
    wait_until(lambda: readings, timeout=1, interval=0.01)
    rig.arm.stop()

    assert readings[0].name == _JOINTS
    assert readings[0].position == pytest.approx(list(map(math.radians, _MEASURED_DEG)))
    assert readings[0].velocity == pytest.approx(list(map(math.radians, _SPEED_DEG_S)))
    assert readings[0].effort == pytest.approx(_TORQUE_NM)
    rig.sdk.ServoMoveStart.assert_called_once_with(cmdType=1)
    assert rig.sdk.ServoJ.call_args_list == [
        call(pytest.approx(angles), [0.0] * 4, cmdT=0.008, cmdType=1)
        for angles in (_MEASURED_DEG, [angle + _TARGET_OFFSET_DEG for angle in _MEASURED_DEG])
    ]
    rig.sdk.ServoMoveEnd.assert_called_once_with(cmdType=1)
    rig.sdk.RobotEnable.assert_not_called()


def test_a_target_ahead_of_the_joint_speed_is_approached_one_step_per_servoj(rig: Rig) -> None:
    # One 8 ms step at the rig's 2 rad/s; the target is three steps ahead, as
    # after commands dropped while a reply was pending.
    step = 2.0 * 0.008
    ahead = [math.radians(angle) + 3 * step for angle in _MEASURED_DEG]
    rig.begin_motion()
    for frame in (1, 2):
        rig.ack(frame, 376)
        rig.targets.publish(JointState(name=_JOINTS, position=ahead))

    assert [servo.args[0] for servo in rig.sdk.ServoJ.call_args_list[1:]] == [
        pytest.approx([angle + math.degrees(steps * step) for angle in _MEASURED_DEG])
        for steps in (1, 2)
    ]
    assert rig.arm.status().last_error is None


def _clear_fault_code(rig: Rig) -> int:
    rig.sdk.feedback.main_code = rig.sdk.feedback.sub_code = 0
    return 0


def test_an_arm_with_a_fault_starts_reporting_its_joints_and_moves_after_clear_errors(
    rig: Rig,
) -> None:
    rig.sdk.feedback.main_code, rig.sdk.feedback.sub_code = 4, 1
    rig.sdk.ResetAllError.side_effect = lambda: _clear_fault_code(rig)
    readings: list[JointState] = []
    rig.arm.joint_state.subscribe(readings.append)
    rig.arm.start()
    rig.move()

    wait_until(lambda: readings, timeout=1, interval=0.01)
    assert "FR5 fault 4/1: collision" in str(rig.arm.status().last_error)
    assert "clear_errors()" in str(rig.arm.status().last_error)
    rig.sdk.ServoMoveStart.assert_not_called()

    rig.arm.clear_errors()
    rig.move()

    rig.sdk.RobotEnable.assert_called_once_with(1)
    rig.sdk.ServoMoveStart.assert_called_once_with(cmdType=1)


def test_a_fault_while_moving_keeps_the_joints_coming_and_clear_errors_resumes(rig: Rig) -> None:
    rig.begin_motion()
    rig.ack(1, 376)
    rig.sdk.feedback.main_code, rig.sdk.feedback.sub_code = 3, 2
    wait_until(lambda: rig.arm.status().last_error is not None, timeout=1, interval=0.01)
    readings: list[JointState] = []
    rig.arm.joint_state.subscribe(readings.append)

    wait_until(lambda: readings, timeout=1, interval=0.01)
    assert "a joint is past its soft limit" in str(rig.arm.status().last_error)

    rig.sdk.ResetAllError.side_effect = lambda: _clear_fault_code(rig)
    rig.arm.clear_errors()
    rig.move()

    assert rig.sdk.ServoMoveStart.call_count == 2


def test_clear_errors_drops_what_the_commander_had_under_way(rig: Rig) -> None:
    rig.sdk.feedback.main_code, rig.sdk.feedback.sub_code = 4, 1
    calls = Mock()
    calls.attach_mock(rig.sdk.ResetAllError, "reset")
    rig.sdk.ResetAllError.side_effect = lambda: _clear_fault_code(rig)
    rig.arm._commander = calls.commander
    rig.arm.start()

    rig.arm.clear_errors()

    assert calls.mock_calls == [
        call.commander.set_estop(True),
        call.reset(),
        call.commander.set_estop(False),
    ]


def test_clear_errors_that_fails_still_releases_the_commander(rig: Rig) -> None:
    rig.sdk.feedback.EmergencyStop = 1
    commander = Mock()
    rig.arm._commander = commander
    rig.arm.start()

    with pytest.raises(RuntimeError, match="emergency stop is pressed"):
        rig.arm.clear_errors()

    assert commander.set_estop.call_args_list == [call(True), call(False)]


def test_clear_errors_fails_while_the_fault_still_holds(rig: Rig) -> None:
    rig.sdk.feedback.main_code, rig.sdk.feedback.sub_code = 3, 2
    rig.arm.start()
    with pytest.raises(RuntimeError, match="FR5 fault 3/2"):
        rig.arm.clear_errors()
    rig.move()

    rig.sdk.ServoMoveStart.assert_not_called()


def test_clear_errors_fails_if_the_fault_comes_back_before_the_arm_has_settled(rig: Rig) -> None:
    rig.sdk.feedback.main_code, rig.sdk.feedback.sub_code = 4, 1
    rig.arm.start()

    def clear_briefly() -> int:
        _clear_fault_code(rig)
        threading.Timer(0.03, setattr, (rig.sdk.feedback, "main_code", 4)).start()
        return 0

    rig.sdk.ResetAllError.side_effect = clear_briefly
    with pytest.raises(RuntimeError, match="FR5 fault 4/0"):
        rig.arm.clear_errors()
    rig.move()

    rig.sdk.ServoMoveStart.assert_not_called()


@pytest.mark.parametrize(
    ("field", "error"),
    [
        ("EmergencyStop", "emergency stop is pressed"),
        ("safety_stop0_state", "safety stop input is active"),
    ],
)
def test_clear_errors_refuses_at_once_under_a_stop_only_a_person_can_release(
    rig: Rig, field: str, error: str
) -> None:
    setattr(rig.sdk.feedback, field, 1)
    rig.arm.start()
    with pytest.raises(RuntimeError, match=error):
        rig.arm.clear_errors()

    rig.sdk.ResetAllError.assert_not_called()
    rig.sdk.RobotEnable.assert_not_called()


def test_an_arm_in_manual_mode_takes_no_command_and_says_to_switch_it(rig: Rig) -> None:
    rig.sdk.feedback.robot_mode = 1
    rig.arm.start()
    rig.move()

    assert "manual mode; switch it to automatic" in str(rig.arm.status().last_error)
    rig.sdk.ServoMoveStart.assert_not_called()

    rig.sdk.feedback.robot_mode = 0
    rig.arm.clear_errors()
    rig.move()

    rig.sdk.ServoMoveStart.assert_called_once_with(cmdType=1)


def test_a_command_takes_feedback_the_reader_has_not_got_to_yet(rig: Rig) -> None:
    rig.arm.start()
    with rig.arm._hardware_lock:  # The reader is held up for longer than the link timeout.
        connection.time.sleep(connection.LINK_TIMEOUT_S * 1.5)
        rig.arm.write({})

    assert rig.arm.status().last_error is None
    rig.sdk.ServoMoveStart.assert_called_once_with(cmdType=1)


def test_a_disabled_arm_takes_no_command_until_clear_errors_enables_it(rig: Rig) -> None:
    rig.sdk.feedback.rbtEnableState = 0

    def enable(_: int) -> int:
        rig.sdk.feedback.rbtEnableState = 1
        return 0

    rig.sdk.RobotEnable.side_effect = enable
    rig.arm.start()
    rig.move()
    rig.sdk.ServoMoveStart.assert_not_called()

    rig.arm.clear_errors()
    rig.move()

    rig.sdk.ServoMoveStart.assert_called_once_with(cmdType=1)


def test_deadman_ends_the_servo_session_and_the_next_command_starts_another(rig: Rig) -> None:
    rig.arm.config.deadman_timeout_s = 0.1
    rig.begin_motion()
    rig.ack(1, 376)

    wait_until(lambda: rig.arm.status().deadman_fired, timeout=1, interval=0.01)
    rig.sdk.ServoMoveEnd.assert_called_once_with(cmdType=1)

    # Frames 0 and 1 were the first session; ServoMoveEnd took frame 2.
    rig.move()
    rig.ack(3, 689)
    rig.move()

    assert rig.sdk.ServoMoveStart.call_count == 2
    assert rig.sdk.ServoJ.call_args_list[-1] == call(
        pytest.approx(_MEASURED_DEG), [0.0] * 4, cmdT=0.008, cmdType=1
    )
    assert rig.arm.status().last_error is None


@pytest.mark.parametrize(
    ("reply_frame", "feedback_age", "error"),
    [(1, 1.0, "feedback timed out"), (0, 0.0, "UDP reply timed out")],
    ids=["stale_feedback", "previous_reply_frame"],
)
def test_feedback_or_reply_timeout_ends_the_servo_session(
    rig: Rig, reply_frame: int, feedback_age: float, error: str
) -> None:
    rig.begin_motion()
    rig.ack(reply_frame, 376)
    rig.sdk.feedback_age = feedback_age

    wait_until(lambda: rig.arm.status().last_error is not None, timeout=1, interval=0.01)
    assert error in str(rig.arm.status().last_error)
    rig.sdk.ServoMoveEnd.assert_called_once_with(cmdType=1)

    rig.sdk.feedback_age = 0.0
    rig.ack(1, 376)
    rig.move()
    rig.sdk.ServoJ.assert_called_once()


def test_the_singular_pose_notice_does_not_end_the_servo_session(rig: Rig) -> None:
    rig.sdk.feedback.main_code = 10
    rig.begin_motion()
    rig.ack(1, 376)
    rig.move()

    assert rig.sdk.ServoJ.call_count == 2
    assert rig.arm.status().last_error is None
    rig.sdk.ServoMoveEnd.assert_not_called()


def test_a_controller_fault_ends_the_servo_session(rig: Rig) -> None:
    rig.begin_motion()
    rig.ack(1, 376)
    rig.sdk.feedback.main_code, rig.sdk.feedback.sub_code = 4, 4

    wait_until(lambda: rig.arm.status().last_error is not None, timeout=1, interval=0.01)

    assert "FR5 fault 4/4" in str(rig.arm.status().last_error)
    rig.sdk.ServoMoveEnd.assert_called_once_with(cmdType=1)
