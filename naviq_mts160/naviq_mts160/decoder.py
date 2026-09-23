"""Pure-Python decoding and encoding of Naviq MTS160 CAN frames.

This module has **no ROS imports** so it can be unit-tested and reused
anywhere (the fixture tool imports it directly as its non-ROS fallback).

Frame layouts follow the MTS160 manual
(https://docs.naviq.com/manual/can-communication.html) and were cross-checked
against the sensor firmware (``application/src/canopen.c``):

* TPDO1 "Sense"    ``0x180 + node``  5 bytes  ``int8 lpos, int8 rpos, int8 lang, int8 rang, u8 status``
* TPDO2 "Marker"   ``0x280 + node``  8 bytes  ``int16 lx, ly, rx, ry`` little-endian, 0.1 mm/LSB
* TPDO3 "Navicode" ``0x380 + node``  3 bytes  ``u16 code`` little-endian, ``u8 counter``
* Heartbeat        ``0x700 + node``  1 byte   NMT state

Status byte (bit 7 .. bit 0)::

    merge | fork | intersection | right_marker | left_marker | strength[1] | strength[0] | fault(0)
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Dict, Optional, Union

# --------------------------------------------------------------------------- IDs
NODE_ID_MASK = 0x7F
FUNCTION_CODE_MASK = 0x780

FC_TPDO1 = 0x180        # "Sense"
FC_TPDO2 = 0x280        # "Marker"
FC_TPDO3 = 0x380        # "Navicode"
FC_SDO_TX = 0x580       # sensor -> host (SDO responses)
FC_SDO_RX = 0x600       # host -> sensor (SDO requests)
FC_HEARTBEAT = 0x700

MIN_NODE_ID = 1
MAX_NODE_ID = 127

# ----------------------------------------------------------------- payload sizes
TPDO1_LEN = 5
TPDO2_LEN = 8
TPDO3_LEN = 3
HEARTBEAT_LEN = 1
SDO_LEN = 8

EXPECTED_LENGTHS: Dict[int, int] = {
    FC_TPDO1: TPDO1_LEN,
    FC_TPDO2: TPDO2_LEN,
    FC_TPDO3: TPDO3_LEN,
    FC_HEARTBEAT: HEARTBEAT_LEN,
}

# ---------------------------------------------------------------------- scaling
MARKER_LSB_MM = 0.1          # TPDO2 units
POSITION_LSB_MM = 1.0        # TPDO1 units
ANGLE_LSB_DEG = 1.0          # TPDO1 units

# ------------------------------------------------------------------ status byte
STATUS_FAULT = 0x01          # "unused (0)" in current firmware
STATUS_STRENGTH_SHIFT = 1
STATUS_STRENGTH_MASK = 0x06
STATUS_LEFT_MARKER = 0x08
STATUS_RIGHT_MARKER = 0x10
STATUS_INTERSECTION = 0x20
STATUS_FORK = 0x40
STATUS_MERGE = 0x80

STRENGTH_NONE = 0
STRENGTH_WEAK = 1
STRENGTH_MEDIUM = 2
STRENGTH_STRONG = 3
STRENGTH_NAMES = {0: "none", 1: "weak", 2: "medium", 3: "strong"}

# ------------------------------------------------------------------- NMT states
NMT_BOOTUP = 0x00
NMT_STOPPED = 0x04
NMT_OPERATIONAL = 0x05
NMT_PREOPERATIONAL = 0x7F
NMT_STATE_NAMES = {
    NMT_BOOTUP: "boot-up",
    NMT_STOPPED: "stopped",
    NMT_OPERATIONAL: "operational",
    NMT_PREOPERATIONAL: "pre-operational",
}

_TPDO1_STRUCT = struct.Struct("<bbbbB")
_TPDO2_STRUCT = struct.Struct("<hhhh")
_TPDO3_STRUCT = struct.Struct("<HB")
_HB_STRUCT = struct.Struct("<B")


# ------------------------------------------------------------------- exceptions
class DecodeError(ValueError):
    """Base class for decoding problems."""


class WrongLengthError(DecodeError):
    """Payload length does not match the TPDO definition."""

    def __init__(self, can_id: int, expected: int, actual: int):
        super().__init__(
            f"CAN id 0x{can_id:03X}: expected {expected} data bytes, got {actual}"
        )
        self.can_id = can_id
        self.expected = expected
        self.actual = actual


# ------------------------------------------------------------------ data classes
@dataclass(frozen=True)
class TrackFrame:
    """Decoded TPDO1 ("Sense")."""

    left_position_mm: int
    right_position_mm: int
    left_angle_deg: int
    right_angle_deg: int
    strength: int
    left_marker: bool
    right_marker: bool
    intersection: bool
    fork: bool
    merge: bool
    fault: bool
    status: int

    @property
    def tape_detected(self) -> bool:
        return self.strength != STRENGTH_NONE

    @property
    def single_track(self) -> bool:
        return (
            self.left_position_mm == self.right_position_mm
            and self.left_angle_deg == self.right_angle_deg
        )

    @property
    def strength_name(self) -> str:
        return STRENGTH_NAMES.get(self.strength, "?")


@dataclass(frozen=True)
class MarkerFrame:
    """Decoded TPDO2 ("Marker"); raw values are int16 in 0.1 mm."""

    left_x_raw: int
    left_y_raw: int
    right_x_raw: int
    right_y_raw: int

    @property
    def left_x_mm(self) -> float:
        return self.left_x_raw * MARKER_LSB_MM

    @property
    def left_y_mm(self) -> float:
        return self.left_y_raw * MARKER_LSB_MM

    @property
    def right_x_mm(self) -> float:
        return self.right_x_raw * MARKER_LSB_MM

    @property
    def right_y_mm(self) -> float:
        return self.right_y_raw * MARKER_LSB_MM


@dataclass(frozen=True)
class NavicodeFrame:
    """Decoded TPDO3 ("Navicode")."""

    code: int
    counter: int


@dataclass(frozen=True)
class HeartbeatFrame:
    """Decoded CANopen heartbeat (0x700 + node)."""

    state: int

    @property
    def state_name(self) -> str:
        return NMT_STATE_NAMES.get(self.state, f"unknown(0x{self.state:02X})")


Decoded = Union[TrackFrame, MarkerFrame, NavicodeFrame, HeartbeatFrame]


# --------------------------------------------------------------------- helpers
def function_code(can_id: int) -> int:
    """Upper part of an 11-bit COB-ID (``id & 0x780``)."""
    return can_id & FUNCTION_CODE_MASK


def node_id_of(can_id: int) -> int:
    """Node ID part of an 11-bit COB-ID (``id & 0x7F``)."""
    return can_id & NODE_ID_MASK


def is_node_frame(can_id: int, node_id: int) -> bool:
    return node_id_of(can_id) == node_id and can_id <= 0x7FF


def cob_ids(node_id: int) -> Dict[str, int]:
    """All COB-IDs used by a sensor with the given node ID."""
    if not MIN_NODE_ID <= node_id <= MAX_NODE_ID:
        raise ValueError(f"node_id must be in {MIN_NODE_ID}..{MAX_NODE_ID}, got {node_id}")
    return {
        "tpdo1": FC_TPDO1 + node_id,
        "tpdo2": FC_TPDO2 + node_id,
        "tpdo3": FC_TPDO3 + node_id,
        "heartbeat": FC_HEARTBEAT + node_id,
        "sdo_tx": FC_SDO_TX + node_id,
        "sdo_rx": FC_SDO_RX + node_id,
    }


def _check_len(can_id: int, data: bytes, expected: int) -> None:
    if len(data) != expected:
        raise WrongLengthError(can_id, expected, len(data))


# --------------------------------------------------------------------- decoders
def decode_status(status: int) -> Dict[str, Union[int, bool]]:
    """Split the TPDO1 status byte into named fields."""
    return {
        "strength": (status & STATUS_STRENGTH_MASK) >> STATUS_STRENGTH_SHIFT,
        "left_marker": bool(status & STATUS_LEFT_MARKER),
        "right_marker": bool(status & STATUS_RIGHT_MARKER),
        "intersection": bool(status & STATUS_INTERSECTION),
        "fork": bool(status & STATUS_FORK),
        "merge": bool(status & STATUS_MERGE),
        "fault": bool(status & STATUS_FAULT),
    }


def encode_status(
    strength: int,
    left_marker: bool = False,
    right_marker: bool = False,
    intersection: bool = False,
    fork: bool = False,
    merge: bool = False,
    fault: bool = False,
) -> int:
    if not 0 <= strength <= 3:
        raise ValueError("strength must be 0..3")
    status = (strength << STATUS_STRENGTH_SHIFT) & STATUS_STRENGTH_MASK
    status |= STATUS_LEFT_MARKER if left_marker else 0
    status |= STATUS_RIGHT_MARKER if right_marker else 0
    status |= STATUS_INTERSECTION if intersection else 0
    status |= STATUS_FORK if fork else 0
    status |= STATUS_MERGE if merge else 0
    status |= STATUS_FAULT if fault else 0
    return status


def decode_tpdo1(data: bytes, can_id: int = FC_TPDO1) -> TrackFrame:
    _check_len(can_id, data, TPDO1_LEN)
    lpos, rpos, lang, rang, status = _TPDO1_STRUCT.unpack(bytes(data))
    s = decode_status(status)
    return TrackFrame(
        left_position_mm=lpos,
        right_position_mm=rpos,
        left_angle_deg=lang,
        right_angle_deg=rang,
        strength=int(s["strength"]),
        left_marker=bool(s["left_marker"]),
        right_marker=bool(s["right_marker"]),
        intersection=bool(s["intersection"]),
        fork=bool(s["fork"]),
        merge=bool(s["merge"]),
        fault=bool(s["fault"]),
        status=status,
    )


def encode_tpdo1(
    left_position_mm: int,
    right_position_mm: int,
    left_angle_deg: int,
    right_angle_deg: int,
    strength: int,
    left_marker: bool = False,
    right_marker: bool = False,
    intersection: bool = False,
    fork: bool = False,
    merge: bool = False,
    fault: bool = False,
) -> bytes:
    status = encode_status(strength, left_marker, right_marker, intersection, fork, merge, fault)
    return _TPDO1_STRUCT.pack(
        int(left_position_mm), int(right_position_mm), int(left_angle_deg), int(right_angle_deg), status
    )


def decode_tpdo2(data: bytes, can_id: int = FC_TPDO2) -> MarkerFrame:
    _check_len(can_id, data, TPDO2_LEN)
    lx, ly, rx, ry = _TPDO2_STRUCT.unpack(bytes(data))
    return MarkerFrame(lx, ly, rx, ry)


def encode_tpdo2(left_x_raw: int, left_y_raw: int, right_x_raw: int, right_y_raw: int) -> bytes:
    return _TPDO2_STRUCT.pack(int(left_x_raw), int(left_y_raw), int(right_x_raw), int(right_y_raw))


def encode_tpdo2_mm(left_x_mm: float, left_y_mm: float, right_x_mm: float, right_y_mm: float) -> bytes:
    """Convenience encoder taking millimetres (rounded to 0.1 mm)."""
    return encode_tpdo2(
        round(left_x_mm / MARKER_LSB_MM),
        round(left_y_mm / MARKER_LSB_MM),
        round(right_x_mm / MARKER_LSB_MM),
        round(right_y_mm / MARKER_LSB_MM),
    )


def decode_tpdo3(data: bytes, can_id: int = FC_TPDO3) -> NavicodeFrame:
    _check_len(can_id, data, TPDO3_LEN)
    code, counter = _TPDO3_STRUCT.unpack(bytes(data))
    return NavicodeFrame(code, counter)


def encode_tpdo3(code: int, counter: int) -> bytes:
    return _TPDO3_STRUCT.pack(int(code) & 0xFFFF, int(counter) & 0xFF)


def decode_heartbeat(data: bytes, can_id: int = FC_HEARTBEAT) -> HeartbeatFrame:
    _check_len(can_id, data, HEARTBEAT_LEN)
    (state,) = _HB_STRUCT.unpack(bytes(data))
    return HeartbeatFrame(state)


def encode_heartbeat(state: int) -> bytes:
    return _HB_STRUCT.pack(int(state) & 0xFF)


def decode_frame(can_id: int, data: bytes, node_id: int) -> Optional[Decoded]:
    """Decode one CAN frame if it is a TPDO/heartbeat from ``node_id``.

    Returns ``None`` for frames from other nodes, for SDO traffic and for
    unknown function codes.  Raises :class:`WrongLengthError` when a frame
    from the sensor has the wrong payload length.
    """
    if not is_node_frame(can_id, node_id):
        return None
    fc = function_code(can_id)
    if fc == FC_TPDO1:
        return decode_tpdo1(data, can_id)
    if fc == FC_TPDO2:
        return decode_tpdo2(data, can_id)
    if fc == FC_TPDO3:
        return decode_tpdo3(data, can_id)
    if fc == FC_HEARTBEAT:
        return decode_heartbeat(data, can_id)
    return None


def counter_advanced(previous: Optional[int], current: int) -> bool:
    """``is_new`` logic: True when the navicode counter changed (mod 256).

    The first observed frame (``previous is None``) is *not* new: the driver
    cannot know whether the stored code pre-dates its own start.
    """
    if previous is None:
        return False
    return (int(current) & 0xFF) != (int(previous) & 0xFF)


def apply_inversions(
    frame: TrackFrame, invert_position: bool = False, invert_angle: bool = False
) -> TrackFrame:
    """Return a copy with the mounting-flip conventions applied."""
    if not (invert_position or invert_angle):
        return frame
    sp = -1 if invert_position else 1
    sa = -1 if invert_angle else 1
    return TrackFrame(
        left_position_mm=sp * frame.left_position_mm,
        right_position_mm=sp * frame.right_position_mm,
        left_angle_deg=sa * frame.left_angle_deg,
        right_angle_deg=sa * frame.right_angle_deg,
        strength=frame.strength,
        left_marker=frame.left_marker,
        right_marker=frame.right_marker,
        intersection=frame.intersection,
        fork=frame.fork,
        merge=frame.merge,
        fault=frame.fault,
        status=frame.status,
    )
