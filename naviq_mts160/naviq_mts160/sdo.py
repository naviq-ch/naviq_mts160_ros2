"""Expedited CANopen SDO client (CiA 301) for the MTS160.

Only expedited transfers (1..4 data bytes) are implemented: every object the
driver touches fits.  No ROS imports.

Frame building/parsing are pure functions (unit-testable); :class:`SdoClient`
runs a transaction over a :class:`naviq_mts160.canbus.CanBus`.
"""

from __future__ import annotations

import queue
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional

from . import decoder

# Command specifiers (bits 7..5 of byte 0)
CCS_DOWNLOAD_INIT = 0x20   # client -> server, write
CCS_UPLOAD_INIT = 0x40     # client -> server, read
SCS_UPLOAD_RESP = 0x40     # server -> client, read response
SCS_DOWNLOAD_RESP = 0x60   # server -> client, write acknowledge
CS_ABORT = 0x80

ABORT_CODES = {
    0x05030000: "Toggle bit not alternated",
    0x05040000: "SDO protocol timed out",
    0x05040001: "Client/server command specifier not valid or unknown",
    0x05040005: "Out of memory",
    0x06010000: "Unsupported access to an object",
    0x06010001: "Attempt to read a write-only object",
    0x06010002: "Attempt to write a read-only object",
    0x06020000: "Object does not exist in the object dictionary",
    0x06040041: "Object cannot be mapped to the PDO",
    0x06040042: "PDO length exceeded",
    0x06040043: "General parameter incompatibility",
    0x06040047: "General internal incompatibility in the device",
    0x06060000: "Access failed due to a hardware error",
    0x06070010: "Data type / length of service parameter does not match",
    0x06070012: "Length of service parameter too high",
    0x06070013: "Length of service parameter too low",
    0x06090011: "Sub-index does not exist",
    0x06090030: "Value range of parameter exceeded",
    0x06090031: "Value of parameter written too high",
    0x06090032: "Value of parameter written too low",
    0x06090036: "Maximum value is less than minimum value",
    0x08000000: "General error",
    0x08000020: "Data cannot be transferred or stored to the application",
    0x08000021: "Data cannot be transferred or stored because of local control",
    0x08000022: "Data cannot be transferred or stored because of the present device state",
}

# Objects used by this project
OD_TPDO1_EVENT_TIMER = (0x1800, 5)
OD_TPDO2_EVENT_TIMER = (0x1801, 5)
OD_TPDO3_EVENT_TIMER = (0x1802, 5)
OD_ZERO = (0x2000, 0)
OD_SELFTEST = (0x2001, 0)
OD_TAPE_POLARITY = (0x2002, 1)
OD_TAPE_THRESHOLD = (0x2002, 2)
OD_MARKER_THRESHOLD = (0x2002, 3)
OD_SELFTEST_RESULT = (0x2003, 1)
OD_SELFTEST_MIN_DELTA = (0x2003, 2)
OD_SELFTEST_MAX_DELTA = (0x2003, 3)


class SdoError(Exception):
    """Base class for SDO failures."""


class SdoTimeout(SdoError):
    pass


class SdoAbort(SdoError):
    def __init__(self, index: int, sub: int, code: int):
        desc = ABORT_CODES.get(code, "unknown abort code")
        super().__init__(f"SDO abort 0x{index:04X}:{sub} code 0x{code:08X} ({desc})")
        self.index = index
        self.sub = sub
        self.code = code
        self.description = desc


class SdoProtocolError(SdoError):
    pass


@dataclass(frozen=True)
class SdoResponse:
    kind: str            # "download", "upload" or "abort"
    index: int
    sub: int
    data: bytes          # payload for "upload" (1..4 bytes), empty otherwise
    abort_code: Optional[int] = None


# ----------------------------------------------------------- pure frame codecs
def build_download(index: int, sub: int, data: bytes) -> bytes:
    """Expedited download (write) request, 8 bytes."""
    n = len(data)
    if not 1 <= n <= 4:
        raise ValueError("expedited download carries 1..4 bytes")
    cmd = CCS_DOWNLOAD_INIT | ((4 - n) << 2) | 0x02 | 0x01   # e=1, s=1
    return bytes([cmd, index & 0xFF, (index >> 8) & 0xFF, sub & 0xFF]) + bytes(data) + bytes(4 - n)


def build_upload(index: int, sub: int) -> bytes:
    """Upload (read) request, 8 bytes."""
    return bytes([CCS_UPLOAD_INIT, index & 0xFF, (index >> 8) & 0xFF, sub & 0xFF, 0, 0, 0, 0])


def build_abort(index: int, sub: int, code: int) -> bytes:
    return bytes([CS_ABORT, index & 0xFF, (index >> 8) & 0xFF, sub & 0xFF]) + struct.pack("<I", code)


def build_upload_response(index: int, sub: int, data: bytes) -> bytes:
    """Server-side encoder (used by tests / simulators)."""
    n = len(data)
    if not 1 <= n <= 4:
        raise ValueError("expedited upload carries 1..4 bytes")
    cmd = SCS_UPLOAD_RESP | ((4 - n) << 2) | 0x02 | 0x01
    return bytes([cmd, index & 0xFF, (index >> 8) & 0xFF, sub & 0xFF]) + bytes(data) + bytes(4 - n)


def build_download_response(index: int, sub: int) -> bytes:
    return bytes([SCS_DOWNLOAD_RESP, index & 0xFF, (index >> 8) & 0xFF, sub & 0xFF, 0, 0, 0, 0])


def parse_request(data: bytes) -> SdoResponse:
    """Parse a client request (used by simulators). kind is 'download' or 'upload'."""
    if len(data) != decoder.SDO_LEN:
        raise SdoProtocolError(f"SDO request must be 8 bytes, got {len(data)}")
    cmd = data[0]
    index = data[1] | (data[2] << 8)
    sub = data[3]
    ccs = cmd & 0xE0
    if ccs == CCS_DOWNLOAD_INIT:
        e = (cmd >> 1) & 1
        s = cmd & 1
        if not e:
            raise SdoProtocolError("segmented download not supported")
        n = (cmd >> 2) & 3 if s else 0
        return SdoResponse("download", index, sub, bytes(data[4:8 - n]))
    if ccs == CCS_UPLOAD_INIT:
        return SdoResponse("upload", index, sub, b"")
    if ccs == CS_ABORT:
        return SdoResponse("abort", index, sub, b"", struct.unpack("<I", data[4:8])[0])
    raise SdoProtocolError(f"unknown command specifier 0x{cmd:02X}")


def parse_response(data: bytes) -> SdoResponse:
    """Parse a server response frame (payload of ``0x580 + node``)."""
    if len(data) != decoder.SDO_LEN:
        raise SdoProtocolError(f"SDO response must be 8 bytes, got {len(data)}")
    cmd = data[0]
    index = data[1] | (data[2] << 8)
    sub = data[3]
    scs = cmd & 0xE0
    if scs == SCS_DOWNLOAD_RESP:
        return SdoResponse("download", index, sub, b"")
    if scs == SCS_UPLOAD_RESP:
        e = (cmd >> 1) & 1
        s = cmd & 1
        if not e:
            raise SdoProtocolError("segmented upload not supported (only expedited)")
        n = (cmd >> 2) & 3 if s else 0
        return SdoResponse("upload", index, sub, bytes(data[4:8 - n]))
    if scs == CS_ABORT:
        code = struct.unpack("<I", data[4:8])[0]
        return SdoResponse("abort", index, sub, b"", code)
    raise SdoProtocolError(f"unexpected SDO command specifier 0x{cmd:02X}")


# ------------------------------------------------------------------- the client
class SdoClient:
    """Blocking expedited SDO client on top of :class:`CanBus`.

    One transaction at a time (guarded by a lock).  Responses are matched on
    index/sub-index; stale frames are discarded.
    """

    def __init__(self, bus, node_id: int, timeout_s: float = 0.5, retries: int = 1):
        if not decoder.MIN_NODE_ID <= node_id <= decoder.MAX_NODE_ID:
            raise ValueError("node_id must be 1..127")
        self._bus = bus
        self.node_id = node_id
        self.timeout_s = float(timeout_s)
        self.retries = int(retries)
        ids = decoder.cob_ids(node_id)
        self.request_id = ids["sdo_rx"]
        self.response_id = ids["sdo_tx"]
        self._lock = threading.Lock()
        self._queue: "queue.Queue" = queue.Queue()
        self.transactions = 0
        self.timeouts = 0
        self.aborts = 0
        bus.add_listener(self._on_frame)

    def close(self) -> None:
        try:
            self._bus.remove_listener(self._on_frame)
        except Exception:
            pass

    # listener called from the bus reader thread
    def _on_frame(self, frame) -> None:
        if frame.can_id == self.response_id:
            self._queue.put(frame)

    def _drain(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                return

    def _transact(self, request: bytes, index: int, sub: int) -> SdoResponse:
        with self._lock:
            last_exc: Optional[Exception] = None
            for _attempt in range(self.retries + 1):
                self._drain()
                self.transactions += 1
                self._bus.send(self.request_id, request)
                deadline = time.monotonic() + self.timeout_s
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        self.timeouts += 1
                        last_exc = SdoTimeout(
                            f"no SDO response for 0x{index:04X}:{sub} within {self.timeout_s:.3f} s"
                        )
                        break
                    try:
                        frame = self._queue.get(timeout=remaining)
                    except queue.Empty:
                        continue
                    try:
                        resp = parse_response(frame.data)
                    except SdoProtocolError as exc:
                        last_exc = exc
                        continue
                    if resp.index != index or resp.sub != sub:
                        continue   # stale response from an earlier transaction
                    if resp.kind == "abort":
                        self.aborts += 1
                        raise SdoAbort(index, sub, int(resp.abort_code or 0))
                    return resp
            assert last_exc is not None
            raise last_exc

    # ---------------------------------------------------------------- raw API
    def upload(self, index: int, sub: int = 0) -> bytes:
        resp = self._transact(build_upload(index, sub), index, sub)
        if resp.kind != "upload":
            raise SdoProtocolError(f"expected upload response, got {resp.kind}")
        return resp.data

    def download(self, index: int, sub: int, data: bytes) -> None:
        resp = self._transact(build_download(index, sub, data), index, sub)
        if resp.kind != "download":
            raise SdoProtocolError(f"expected download acknowledge, got {resp.kind}")

    # -------------------------------------------------------------- typed API
    def read_u8(self, index: int, sub: int = 0) -> int:
        return struct.unpack("<B", self.upload(index, sub)[:1])[0]

    def read_u16(self, index: int, sub: int = 0) -> int:
        data = self.upload(index, sub)
        return struct.unpack("<H", (data + b"\0\0")[:2])[0]

    def read_u32(self, index: int, sub: int = 0) -> int:
        data = self.upload(index, sub)
        return struct.unpack("<I", (data + b"\0\0\0\0")[:4])[0]

    def write_u8(self, index: int, sub: int, value: int) -> None:
        self.download(index, sub, struct.pack("<B", int(value) & 0xFF))

    def write_u16(self, index: int, sub: int, value: int) -> None:
        self.download(index, sub, struct.pack("<H", int(value) & 0xFFFF))

    def write_u32(self, index: int, sub: int, value: int) -> None:
        self.download(index, sub, struct.pack("<I", int(value) & 0xFFFFFFFF))

    # ------------------------------------------------------- MTS160 helpers
    def start_zero(self) -> None:
        """SDO 0x2000: start zero-level calibration (firmware saves it to flash)."""
        self.write_u8(*OD_ZERO, 1)

    def start_selftest(self) -> None:
        """SDO 0x2001: start the internal self-test (takes ~30 ms)."""
        self.write_u8(*OD_SELFTEST, 1)

    def read_selftest(self):
        """Read 0x2003:1..3 -> (passed, min_delta_ut, max_delta_ut)."""
        result = self.read_u8(*OD_SELFTEST_RESULT)
        min_delta = self.read_u16(*OD_SELFTEST_MIN_DELTA)
        max_delta = self.read_u16(*OD_SELFTEST_MAX_DELTA)
        return (result == 1, min_delta, max_delta)

    def run_selftest(self, settle_s: float = 0.05):
        """Start a self-test, wait, read the result registers."""
        self.start_selftest()
        time.sleep(settle_s)
        return self.read_selftest()

    def set_tpdo_period(self, tpdo: int, period_ms: int) -> None:
        """Write event timer of TPDO 1..3 (0x1800+n-1 sub 5). 0 disables the TPDO."""
        if tpdo not in (1, 2, 3):
            raise ValueError("tpdo must be 1, 2 or 3")
        if not 0 <= period_ms <= 0xFFFF:
            raise ValueError("period_ms must be 0..65535")
        self.write_u16(0x1800 + tpdo - 1, 5, period_ms)

    def get_tpdo_period(self, tpdo: int) -> int:
        if tpdo not in (1, 2, 3):
            raise ValueError("tpdo must be 1, 2 or 3")
        return self.read_u16(0x1800 + tpdo - 1, 5)
