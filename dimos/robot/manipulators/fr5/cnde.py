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

"""Read complete CNDE frames from the FAIRINO SDK's TCP stream."""

import socket
import time
from typing import Any

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class _FrameReader:
    def __init__(self, sock: socket.socket) -> None:
        self.sock = sock
        self.pending = bytearray()
        self.buffered_at = 0.0
        self.received_at = 0.0

    def read(self, output: bytearray, timeout: float, max_age: float) -> int:
        deadline = time.monotonic() + timeout
        latest = self.buffered_at
        while True:
            now = time.monotonic()
            if self.pending and now - self.buffered_at >= max_age:
                raise ConnectionError("buffered CNDE data expired")
            if len(self.pending) >= 6:
                size = 8 + int.from_bytes(self.pending[4:6], "little")
                if self.pending[:2] != b"\x5a\x5a" or size > len(output):
                    raise ConnectionError("invalid CNDE frame header or length")
                if len(self.pending) >= size:
                    if self.pending[size - 2 : size] != b"\xa5\xa5":
                        raise ConnectionError("invalid CNDE frame trailer")
                    output[:size] = self.pending[:size]
                    del self.pending[:size]
                    # Any surplus belongs to the same TCP read as the frame's tail.
                    self.received_at = self.buffered_at = latest
                    return size
            remaining = deadline - now
            if self.pending:
                remaining = min(remaining, max_age - (now - self.buffered_at))
            if remaining <= 0:
                return 0
            self.sock.settimeout(remaining)
            data = self.sock.recv(len(output) - len(self.pending))
            if not data:
                raise ConnectionError("EOF on CNDE socket")
            latest = time.monotonic()
            if not self.pending:
                self.buffered_at = latest
            self.pending.extend(data)


def install_cnde_reader(sdk: Any, max_age: float) -> None:
    # The SDK assumes one recv() equals one frame during setup and streaming.
    def recv(client: Any, output: bytearray, timeout: float = 1.0) -> int:
        sock = client._tcp_socket
        if sock is None:
            return -1
        reader: _FrameReader | None = getattr(client, "_dimos_cnde_reader", None)
        if reader is None:
            reader = client._dimos_cnde_reader = _FrameReader(sock)
            # The SDK mutates a shared ctypes structure field by field. Publish an
            # immutable snapshot and its actual TCP arrival together after decoding.
            state = client._robot_state_pkg
            parse = client._parse_cnde_state_data

            def parse_sample(data: bytes) -> None:
                parse(data)
                state._dimos_sample = (
                    type(state).from_buffer_copy(state),
                    client._dimos_cnde_reader.received_at,
                )

            client._parse_cnde_state_data = parse_sample
        try:
            return reader.read(output, timeout, max_age)
        except TimeoutError:
            return 0  # Keep partial bytes for the next read.
        except OSError as error:
            if not client._stop_event.is_set():
                logger.warning("FR5 CNDE receive failed", reason=str(error))
            return -1

    sdk.FRCNDEClient._recv_data = recv
