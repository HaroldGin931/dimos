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

"""FR5 v6 with the matching FAIRINO Linux Python SDK cached by DimOS."""

from __future__ import annotations

from hashlib import sha256
import math
from pathlib import Path
from queue import Empty, SimpleQueue
import re
from tempfile import TemporaryDirectory
import threading
import time
from types import ModuleType
from typing import Any, Literal
import xmlrpc.client

from pydantic import model_validator
from reactivex.disposable import Disposable
import requests
from typing_extensions import Self

from dimos.constants import CACHE_DIR, DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule, Values
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import model_limit, movable_joints
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.fr5.cnde import install_cnde_reader
from dimos.robot.manipulators.fr5.config import FR5_JOINT_NAMES, fr5_coordinator_connection
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

SERVO_PERIOD_S = 0.008
LINK_TIMEOUT_S = 0.1
_SERVO_START = 689
_SERVO_J = 376
_SDK_RPC_PORT = 20003
_WEBAPP_VERSION = "3.9.5"
_SDK_TAG = "v2.2.5_robot_v3.9.5"
# The driver patches this SDK's private CNDE and UDP clients.
_SDK_URL = (
    "https://raw.githubusercontent.com/FAIR-INNOVATION/fairino-python-sdk/"
    "be272581c2ab65f6577f13ee7d0f676c89ab7a25/linux/fairino/Robot.py"
)
_SDK_SOURCE_SHA256 = "8dd4ec225a6d1cf149a149c6fdf09e112a5d6fd989426702997bd9cc38252da0"


class FR5ConnectionConfig(ConnectionConfig):
    """Where the FR5 is.

    Attributes:
        ip: The controller's IP address.
        source: Always "arm": the driver names the joints ``arm/j1`` ... ``arm/j6``.
        jump_limit_s: How far a joint target may be from where the joint is,
            in seconds at the joint's top speed (its URDF velocity limit).
    """

    ip: str
    source: Literal["arm"] = "arm"
    joint_commands: dict[str, tuple[Interface, ...]] = {}
    jump_limit_s: float = LINK_TIMEOUT_S

    @model_validator(mode="after")
    def _defaults(self) -> Self:
        declaration = fr5_coordinator_connection()
        if self.model is None:
            self.model = declaration.model
        if not self.joint_commands:
            self.joint_commands = declaration.joint_commands
        return self


class FR5Connection(ConnectionModule):
    """A six-joint FR5 streamed joint targets with ServoJ every 8 ms.

    A servo session starts with the first command and ends on ``halt``, e.g.
    when commands stop; the next command starts a new one. Stale feedback, a
    fault, a safety stop, a target too fast or too far from the measured
    position, or a refused or unanswered UDP command ends the session for good:
    restart the blueprint to resume.
    """

    config: FR5ConnectionConfig
    position_command: In[JointState]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._sdk: Any = None
        # Per joint, in rad: how far one ServoJ step, and a target from the
        # measured position, may go.
        self._step_limits: list[float] = []
        self._reach: list[float] = []
        self._measured: list[float] = []
        self._measured_at = 0.0
        self._feedback_stamp: tuple[int, int] | None = None
        self._last_target: list[float] = []
        self._started_servo = False
        # The UDP frame and command awaiting its reply, and the reply deadline.
        self._pending: tuple[int, int, float] | None = None
        self._replies: SimpleQueue[tuple[int, int, str, float]] = SimpleQueue()
        self._fault: str | None = None
        self._stop_reader = threading.Event()
        self._reader: threading.Thread | None = None

    def connect(self) -> None:
        assert self.config.model is not None
        joints = movable_joints(self.config.model)
        if tuple(joint.name for joint in joints) != FR5_JOINT_NAMES:
            raise ValueError(f"FR5Connection drives the joints {', '.join(FR5_JOINT_NAMES)}")
        speeds = [model_limit(joint, Interface.VELOCITY).hi or 0.0 for joint in joints]
        self._step_limits = [speed * SERVO_PERIOD_S for speed in speeds]
        self._reach = [speed * self.config.jump_limit_s for speed in speeds]
        sdk = self._sdk = _open_sdk(self.config.ip)
        try:
            if not sdk.is_connect:
                raise ConnectionError(f"FR5 SDK could not connect to {self.config.ip}")
            sdk.SetUDPCmdRpyCallback(self._reply)
            started = time.monotonic()
            while not (self._read_state() and self._measured_at > started):
                if time.monotonic() - started >= 1.0:
                    raise ConnectionError("FR5 has no fresh joint feedback")
                time.sleep(SERVO_PERIOD_S)
        except Exception:
            self._close_sdk()
            raise

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(
                self.position_command.subscribe(
                    lambda msg: self.command(joint_state_to_values(msg, (Interface.POSITION,)))
                )
            )
        )
        self._reader = threading.Thread(target=self._read_loop, name="FR5 feedback", daemon=True)
        self._reader.start()

    @rpc
    def stop(self) -> None:
        self._stop_reader.set()
        if self._reader:
            self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        try:
            super().stop()
        finally:
            self._close_sdk()

    def write(self, values: Values) -> None:
        if self._fault is not None:
            raise RuntimeError(self._fault)
        try:
            self._check_link()
            if self._pending is not None:
                return  # Never queue an old target behind an unacknowledged frame.
            frame = self._sdk._udp_count  # The frame number the SDK sends next.
            if not self._started_servo:
                self._pending = (frame, _SERVO_START, time.monotonic() + LINK_TIMEOUT_S)
                self._started_servo = True
                _check("ServoMoveStart", self._sdk.ServoMoveStart(cmdType=1))
                return
            # A session's first ServoJ holds the measured position.
            target = self._measured
            if self._last_target:
                target = [
                    values.get(f"{joint}/{Interface.POSITION}", measured)
                    for joint, measured in zip(FR5_JOINT_NAMES, self._measured, strict=True)
                ]
                self._check_target(target)
            self._pending = (frame, _SERVO_J, time.monotonic() + LINK_TIMEOUT_S)
            _check(
                "ServoJ",
                self._sdk.ServoJ(
                    list(map(math.degrees, target)), [0.0] * 4, cmdT=SERVO_PERIOD_S, cmdType=1
                ),
            )
            self._last_target = target
        except Exception as error:
            self._fault = f"{error}; restart the FR5 blueprint to resume"
            raise

    def halt(self) -> None:
        self._pending = None
        self._last_target = []
        if self._started_servo:
            self._started_servo = False
            _check("ServoMoveEnd", self._sdk.ServoMoveEnd(cmdType=1))

    def _check_target(self, target: list[float]) -> None:
        # ServoJ reaches each target within one period, so the step from the
        # last target is the commanded speed.
        for joint, angle, previous, measured, step, reach in zip(
            FR5_JOINT_NAMES,
            target,
            self._last_target,
            self._measured,
            self._step_limits,
            self._reach,
            strict=True,
        ):
            if abs(angle - previous) > step:
                raise ValueError(f"FR5 {joint} target step exceeds the model joint speed")
            if abs(angle - measured) > reach:
                raise ValueError(f"FR5 {joint} target is too far from its measured position")

    def _close_sdk(self) -> None:
        sdk, self._sdk = self._sdk, None
        if sdk is not None:
            try:
                if sdk.is_connect:
                    sdk.CloseRPC()
                else:
                    # CloseRPC itself returns early on a partially connected SDK.
                    sdk.CNDEDisconnect()
            finally:
                # The SDK omits the UDP client from CloseRPC.
                sdk._udp_client.close()

    def _read_state(self) -> bool:
        """Take the newest CNDE sample, if there is one; raise on a fault or safety stop."""
        code, state = self._sdk.GetRobotRealTimeState()
        _check("GetRobotRealTimeState", code)
        sample = getattr(state, "_dimos_sample", None)
        if sample is None:
            return False
        state, received_at = sample
        if time.monotonic() - received_at >= LINK_TIMEOUT_S:
            return False
        stamp = (state.second, state.millisecond)
        if self._feedback_stamp == stamp:
            return False
        if state.main_code or state.sub_code:
            raise RuntimeError(f"FR5 fault {state.main_code}/{state.sub_code}")
        if (
            state.EmergencyStop
            or state.collisionState
            or state.safety_stop0_state
            or state.safety_stop1_state
        ):
            raise RuntimeError("FR5 reports emergency stop, collision or safety stop")
        if state.rbtEnableState != 1:
            raise RuntimeError("FR5 is not enabled; enable it locally before starting")
        positions = [math.radians(angle) for angle in state.jt_cur_pos]
        if len(positions) != 6 or not all(map(math.isfinite, positions)):
            raise ValueError("FR5 reported invalid joint positions")
        self._feedback_stamp = stamp
        self._measured = positions
        self._measured_at = received_at
        if self._connected:
            wall_time = time.time() - (time.monotonic() - received_at)
            self.joint_state.publish(
                JointState(ts=wall_time, name=list(FR5_JOINT_NAMES), position=positions.copy())
            )
        return True

    def _read_loop(self) -> None:
        while not self._stop_reader.wait(SERVO_PERIOD_S):
            with self._hardware_lock:
                if not self._connected or self._fault is not None:
                    continue
                try:
                    self._check_link()
                    self._read_state()
                except Exception as error:
                    self._fault = f"{error}; restart the FR5 blueprint to resume"
                    self._report(self._fault)
                    try:
                        self.halt()
                    except Exception as halt_error:
                        self._report(f"halt failed: {halt_error}")

    def _reply(self, _source: int, count: int, command: int, size: int, content: str) -> int:
        if command in (_SERVO_START, _SERVO_J) and size == len(content):
            self._replies.put((count, command, content, time.monotonic()))
        return 0

    def _check_link(self) -> None:
        """Raise if feedback is stale or the pending UDP command was refused or went unanswered."""
        now = time.monotonic()
        if now - self._measured_at > LINK_TIMEOUT_S:
            raise ConnectionError("FR5 joint feedback timed out")
        while self._pending is not None:
            frame, command, deadline = self._pending
            try:
                received, kind, content, stamp = self._replies.get_nowait()
            except Empty:
                if now > deadline:
                    raise ConnectionError(
                        f"FR5 UDP reply timed out for command {command} frame={frame} "
                        f"after {LINK_TIMEOUT_S * 1000:.0f}ms"
                    ) from None
                return
            if (received, kind) != (frame, command):
                continue
            if stamp > deadline:
                raise ConnectionError(
                    f"FR5 UDP reply arrived late for command {command} frame={frame}"
                )
            if content.strip() not in {"1", "errcode:0"}:
                raise RuntimeError(f"FR5 rejected UDP command {command}: {content!a}")
            self._pending = None


def _check(call: str, code: int) -> None:
    if code != 0:
        raise ConnectionError(f"FR5 {call} returned {code}")


def _sdk_print(*values: object) -> None:
    logger.debug("FAIRINO SDK", output=" ".join(map(str, values)))


def _load_sdk() -> ModuleType:
    path = CACHE_DIR / "fairino" / _SDK_TAG / "Robot.py"
    try:
        source = path.read_bytes()
    except FileNotFoundError:
        source = b""
    if sha256(source).hexdigest() != _SDK_SOURCE_SHA256:
        response = requests.get(_SDK_URL, timeout=15.0)
        response.raise_for_status()
        source = response.content
        if sha256(source).hexdigest() != _SDK_SOURCE_SHA256:
            raise ImportError(f"Unexpected FAIRINO SDK source from {_SDK_URL}")
        path.parent.mkdir(parents=True, exist_ok=True)
        with TemporaryDirectory(dir=path.parent) as staging:
            temporary = Path(staging) / path.name
            temporary.write_bytes(source)
            temporary.replace(path)
    sdk = ModuleType("_dimos_fairino_robot")
    sdk.__file__ = str(path)
    vars(sdk)["print"] = _sdk_print
    # Execute the exact bytes checked above, bypassing Python's bytecode cache.
    exec(compile(source, str(path), "exec", dont_inherit=True), vars(sdk))
    return sdk


def _open_sdk(ip: str) -> Any:
    # The vendor's GetSoftwareVersion retries socket errors forever. Make this
    # read-only identification request once, with a timeout, before opening a session.
    with requests.Session() as http:
        http.trust_env = False
        response = http.post(
            f"http://{ip}:{_SDK_RPC_PORT}/RPC2",
            data=xmlrpc.client.dumps((), "GetSoftwareVersion"),
            headers={"Content-Type": "text/xml"},
            timeout=2.0,
            allow_redirects=False,
        )
        response.raise_for_status()
        result, _ = xmlrpc.client.loads(response.content)
    match result:
        case ([int(code), str(model), str(web_version), str(controller_version)],):
            _check("GetSoftwareVersion", code)
        case _:
            raise ValueError("FR5 returned an invalid software version response")
    version = re.fullmatch(r"[vV]?(\d+\.\d+\.\d+)(?:-[\w.-]+)?", web_version.strip())
    if (
        model not in {"FR5", "FR5 V6.0", "FR5-V1-002(V6.0)"}
        or version is None
        or version[1] != _WEBAPP_VERSION
    ):
        raise ValueError(
            f"expected FR5 v6 with WebAPP {_WEBAPP_VERSION}, "
            f"found {model}, WebAPP {web_version}, controller {controller_version}"
        )
    Robot = _load_sdk()
    # FR5 v3.9.5 supplies robot_time but rejects frame_cnt. The SDK decodes
    # fields in subscription order, so put the timestamp after the state it dates.
    state = Robot.RobotState
    fields = [
        state.JointCurPos,
        state.RbtEnableState,
        state.MainCode,
        state.SubCode,
        state.EmergencyStop,
        state.CollisionState,
        state.SafetyStop0State,
        state.SafetyStop1State,
        state.RobotTime,
    ]
    _check(
        "SetRobotRealtimeStateConfig",
        Robot.SetRobotRealtimeStateConfig(fields, round(SERVO_PERIOD_S * 1000)),
    )
    install_cnde_reader(Robot, LINK_TIMEOUT_S)
    # Disable the SDK's reconnect loop before it starts its CNDE reader. UDP
    # sends, including ServoMoveEnd, can then only wait on the socket timeout.
    Robot.RPC._reconnect_enable = False
    sdk = Robot.RPC(ip)
    # The SDK treats send_data's nonzero error codes as truthy successes
    # in ServoJ. A successful datagram send returns the boolean True.
    send_data = sdk._udp_client.send_data
    sdk.SendUDPFrame = lambda packet: send_data(packet) is True
    return sdk
