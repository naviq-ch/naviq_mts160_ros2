"""Decoder unit tests: every field, sign, scale, flag bit, wrong length,
node-ID filter, negative int16 markers and the navicode ``is_new`` rule.
No ROS, no hardware."""

import struct

import pytest

from naviq_mts160 import decoder as d

NODE = 10


# ----------------------------------------------------------------- COB-IDs
def test_cob_ids_node_10():
    ids = d.cob_ids(NODE)
    assert ids == {
        "tpdo1": 0x18A, "tpdo2": 0x28A, "tpdo3": 0x38A,
        "heartbeat": 0x70A, "sdo_tx": 0x58A, "sdo_rx": 0x60A,
    }


@pytest.mark.parametrize("bad", [0, 128, -1, 300])
def test_cob_ids_rejects_bad_node(bad):
    with pytest.raises(ValueError):
        d.cob_ids(bad)


def test_function_code_and_node_id():
    assert d.function_code(0x18A) == 0x180
    assert d.node_id_of(0x18A) == 10
    assert d.function_code(0x70A) == 0x700
    assert d.node_id_of(0x7FF) == 127
    assert d.is_node_frame(0x18A, 10)
    assert not d.is_node_frame(0x18B, 10)
    assert not d.is_node_frame(0x18A | 0x800, 10)   # not an 11-bit id


# ------------------------------------------------------------------- TPDO1
def test_tpdo1_fields_and_signs():
    # left -10 mm, right +10 mm, left -5 deg, right +7 deg, status: strong, both markers
    data = struct.pack("<bbbbB", -10, 10, -5, 7, 0b0001_1110)
    t = d.decode_tpdo1(data)
    assert t.left_position_mm == -10
    assert t.right_position_mm == 10
    assert t.left_angle_deg == -5
    assert t.right_angle_deg == 7
    assert t.strength == d.STRENGTH_STRONG
    assert t.tape_detected
    assert t.left_marker and t.right_marker
    assert not (t.intersection or t.fork or t.merge or t.fault)
    assert not t.single_track
    assert t.status == 0b0001_1110


def test_tpdo1_extremes():
    t = d.decode_tpdo1(bytes([0xB0, 0x50, 0xA6, 0x5A, 0x00]))   # -80, 80, -90, 90
    assert (t.left_position_mm, t.right_position_mm) == (-80, 80)
    assert (t.left_angle_deg, t.right_angle_deg) == (-90, 90)
    assert t.strength == 0 and not t.tape_detected


@pytest.mark.parametrize("strength", [0, 1, 2, 3])
def test_tpdo1_strength_bits(strength):
    t = d.decode_tpdo1(bytes([0, 0, 0, 0, strength << 1]))
    assert t.strength == strength
    assert t.tape_detected == (strength != 0)
    assert t.strength_name == ["none", "weak", "medium", "strong"][strength]


@pytest.mark.parametrize("bit,field", [
    (0x01, "fault"), (0x08, "left_marker"), (0x10, "right_marker"),
    (0x20, "intersection"), (0x40, "fork"), (0x80, "merge"),
])
def test_tpdo1_each_flag_bit(bit, field):
    t = d.decode_tpdo1(bytes([0, 0, 0, 0, bit]))
    for name in ("fault", "left_marker", "right_marker", "intersection", "fork", "merge"):
        assert getattr(t, name) == (name == field), name
    assert t.strength == 0


def test_tpdo1_single_track_property():
    assert d.decode_tpdo1(bytes([5, 5, 3, 3, 0x04])).single_track
    assert not d.decode_tpdo1(bytes([5, 6, 3, 3, 0x04])).single_track
    assert not d.decode_tpdo1(bytes([5, 5, 3, 4, 0x04])).single_track


def test_tpdo1_roundtrip():
    for lp, rp, la, ra, s in [(-80, 80, -90, 90, 3), (0, 0, 0, 0, 0), (12, -34, 45, -67, 2)]:
        for flags in range(0, 0x100, 0x08 + 0x01):
            kw = dict(left_marker=bool(flags & 0x08), right_marker=bool(flags & 0x10),
                      intersection=bool(flags & 0x20), fork=bool(flags & 0x40),
                      merge=bool(flags & 0x80), fault=bool(flags & 0x01))
            raw = d.encode_tpdo1(lp, rp, la, ra, s, **kw)
            t = d.decode_tpdo1(raw)
            assert (t.left_position_mm, t.right_position_mm, t.left_angle_deg, t.right_angle_deg, t.strength) == (lp, rp, la, ra, s)
            for k, v in kw.items():
                assert getattr(t, k) == v


@pytest.mark.parametrize("n", [0, 1, 4, 6, 8])
def test_tpdo1_wrong_length(n):
    with pytest.raises(d.WrongLengthError) as ei:
        d.decode_tpdo1(bytes(n), can_id=0x18A)
    assert ei.value.expected == 5 and ei.value.actual == n and ei.value.can_id == 0x18A
    assert isinstance(ei.value, d.DecodeError)


# ------------------------------------------------------------------- TPDO2
def test_tpdo2_negative_int16_little_endian():
    # manual example: -235 -> -23.5 mm; bytes 0x15 0xFF (LSB first)
    data = bytes([0x15, 0xFF, 0x00, 0x00, 0xE8, 0x03, 0x00, 0x80])   # -235, 0, 1000, -32768
    m = d.decode_tpdo2(data)
    assert m.left_x_raw == -235 and m.left_x_mm == pytest.approx(-23.5)
    assert m.left_y_raw == 0 and m.left_y_mm == 0.0
    assert m.right_x_raw == 1000 and m.right_x_mm == pytest.approx(100.0)
    assert m.right_y_raw == -32768 and m.right_y_mm == pytest.approx(-3276.8)


def test_tpdo2_roundtrip_and_mm_encoder():
    raw = d.encode_tpdo2_mm(-23.5, 12.3, 0.0, -0.1)
    m = d.decode_tpdo2(raw)
    assert (m.left_x_raw, m.left_y_raw, m.right_x_raw, m.right_y_raw) == (-235, 123, 0, -1)
    assert d.decode_tpdo2(d.encode_tpdo2(32767, -32768, 1, -1)) == d.MarkerFrame(32767, -32768, 1, -1)


@pytest.mark.parametrize("n", [0, 7, 9])
def test_tpdo2_wrong_length(n):
    with pytest.raises(d.WrongLengthError):
        d.decode_tpdo2(bytes(n))


# ------------------------------------------------------------------- TPDO3
def test_tpdo3_little_endian_and_counter():
    n = d.decode_tpdo3(bytes([0x01, 0x00, 0x07]))
    assert n.code == 1 and n.counter == 7
    n = d.decode_tpdo3(bytes([0x34, 0x12, 0xFF]))
    assert n.code == 0x1234 and n.counter == 255
    assert d.decode_tpdo3(d.encode_tpdo3(0xFFFF, 0)) == d.NavicodeFrame(0xFFFF, 0)


@pytest.mark.parametrize("n", [0, 2, 4, 8])
def test_tpdo3_wrong_length(n):
    with pytest.raises(d.WrongLengthError):
        d.decode_tpdo3(bytes(n))


def test_counter_advanced_is_new_rule():
    assert d.counter_advanced(None, 5) is False        # first frame: unknown history
    assert d.counter_advanced(5, 5) is False
    assert d.counter_advanced(5, 6) is True
    assert d.counter_advanced(255, 0) is True          # wrap
    assert d.counter_advanced(0, 255) is True


# --------------------------------------------------------------- heartbeat
@pytest.mark.parametrize("state,name", [(0x00, "boot-up"), (0x04, "stopped"), (0x05, "operational"), (0x7F, "pre-operational")])
def test_heartbeat(state, name):
    h = d.decode_heartbeat(bytes([state]))
    assert h.state == state and h.state_name == name


def test_heartbeat_unknown_state_and_length():
    assert "unknown" in d.decode_heartbeat(bytes([0x42])).state_name
    with pytest.raises(d.WrongLengthError):
        d.decode_heartbeat(b"")
    with pytest.raises(d.WrongLengthError):
        d.decode_heartbeat(b"\x05\x00")


# ------------------------------------------------------------ decode_frame
def test_decode_frame_dispatch_and_node_filter():
    tp1 = d.encode_tpdo1(1, 1, 2, 2, 2)
    assert isinstance(d.decode_frame(0x18A, tp1, NODE), d.TrackFrame)
    assert isinstance(d.decode_frame(0x28A, bytes(8), NODE), d.MarkerFrame)
    assert isinstance(d.decode_frame(0x38A, bytes(3), NODE), d.NavicodeFrame)
    assert isinstance(d.decode_frame(0x70A, b"\x05", NODE), d.HeartbeatFrame)
    # other node IDs are ignored, whatever the length
    assert d.decode_frame(0x18B, tp1, NODE) is None
    assert d.decode_frame(0x181, bytes(2), NODE) is None
    # SDO traffic and other function codes from our node are not decoded here
    assert d.decode_frame(0x58A, bytes(8), NODE) is None
    assert d.decode_frame(0x60A, bytes(8), NODE) is None
    assert d.decode_frame(0x08A, bytes(8), NODE) is None   # EMCY
    assert d.decode_frame(0x00A, bytes(2), NODE) is None
    # wrong length from our node raises (driver counts and drops)
    with pytest.raises(d.WrongLengthError):
        d.decode_frame(0x18A, bytes(4), NODE)


# -------------------------------------------------------------- inversions
def test_apply_inversions():
    t = d.decode_tpdo1(d.encode_tpdo1(-10, 12, -5, 7, 3, fork=True))
    assert d.apply_inversions(t) is t
    p = d.apply_inversions(t, invert_position=True)
    assert (p.left_position_mm, p.right_position_mm, p.left_angle_deg, p.right_angle_deg) == (10, -12, -5, 7)
    a = d.apply_inversions(t, invert_angle=True)
    assert (a.left_position_mm, a.right_position_mm, a.left_angle_deg, a.right_angle_deg) == (-10, 12, 5, -7)
    b = d.apply_inversions(t, True, True)
    assert (b.left_position_mm, b.right_position_mm, b.left_angle_deg, b.right_angle_deg) == (10, -12, 5, -7)
    assert b.fork and b.strength == 3 and b.status == t.status
