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

"""Exercise CNDE framing with deterministic TCP reads and no robot connection."""

import ctypes
from threading import Event
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from dimos.robot.manipulators.fr5 import cnde


class _State(ctypes.Structure):
    _fields_ = [("second", ctypes.c_uint8)]


class _Client(SimpleNamespace):
    def _parse_cnde_state_data(self, data: bytes) -> None:
        self._robot_state_pkg.second = data[0]


def _client(*chunks: bytes | Exception) -> _Client:
    cnde.install_cnde_reader(SimpleNamespace(FRCNDEClient=_Client), 0.1)
    return _Client(
        _tcp_socket=Mock(recv=Mock(side_effect=chunks)),
        _stop_event=Event(),
        _robot_state_pkg=_State(),
    )


def _frame(second: int) -> bytes:
    # Delimiters inside a payload are legal and must not split the frame.
    return b"\x5a\x5a\x01\x04\x05\x00" + bytes([second]) + b"\xa5\xa5\x5a\x5a\xa5\xa5"


@pytest.fixture(autouse=True)
def clock(monkeypatch: pytest.MonkeyPatch) -> Mock:
    monotonic = Mock(return_value=3.0)
    monkeypatch.setattr(cnde, "time", SimpleNamespace(monotonic=monotonic))
    return monotonic


@pytest.mark.parametrize("data", [b"", _frame(0)[:-1] + b"!"], ids=["eof", "bad_trailer"])
def test_eof_and_corruption_fail_closed(data: bytes) -> None:
    client = _client(data)
    assert client._recv_data(bytearray(64)) == -1


def test_fragmented_and_queued_frames_keep_arrival_time(clock: Mock) -> None:
    first, second, third = map(_frame, range(3))
    client = _client(first[:4], TimeoutError(), first[4:] + second + third)
    output = bytearray(64)

    assert client._recv_data(output) == 0
    assert client._recv_data(output) == len(first)
    assert output[: len(first)] == first
    client._parse_cnde_state_data(output[6 : len(first) - 2])
    original, original_arrival = client._robot_state_pkg._dimos_sample
    clock.return_value = 3.08
    assert client._recv_data(output) == len(second)
    assert output[: len(second)] == second
    client._parse_cnde_state_data(output[6 : len(second) - 2])
    updated, updated_arrival = client._robot_state_pkg._dimos_sample
    assert (original.second, updated.second) == (0, 1)
    assert original_arrival == updated_arrival == 3.0
    clock.return_value = 3.11
    assert client._recv_data(output) == -1
