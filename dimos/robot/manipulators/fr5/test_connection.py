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
from types import SimpleNamespace
from unittest.mock import Mock, call
import xmlrpc.client

import pytest
from pytest_mock import MockerFixture
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
    sdk.feedback = SimpleNamespace(
        jt_cur_pos=_MEASURED_DEG,
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


def test_refuses_to_start_if_the_arm_is_disabled(rig: Rig) -> None:
    rig.sdk.feedback.rbtEnableState = 0
    with pytest.raises(RuntimeError, match="not enabled"):
        rig.arm.start()

    rig.sdk.ServoMoveStart.assert_not_called()


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
