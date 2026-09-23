"""Expedited SDO codec tests and an SdoClient test against a fake sensor."""

import struct
import threading
import time

import pytest

from naviq_mts160 import sdo
from naviq_mts160.canbus import Frame


# ----------------------------------------------------------------- codecs
def test_build_download_command_bytes():
    assert sdo.build_download(0x2001, 0, b"\x01") == bytes([0x2F, 0x01, 0x20, 0x00, 0x01, 0, 0, 0])
    assert sdo.build_download(0x1800, 5, struct.pack("<H", 10)) == bytes([0x2B, 0x00, 0x18, 0x05, 10, 0, 0, 0])
    assert sdo.build_download(0x1017, 0, b"\x01\x02\x03") == bytes([0x27, 0x17, 0x10, 0x00, 1, 2, 3, 0])
    assert sdo.build_download(0x1005, 0, b"\x80\x00\x00\x00") == bytes([0x23, 0x05, 0x10, 0x00, 0x80, 0, 0, 0])
    with pytest.raises(ValueError):
        sdo.build_download(0x2000, 0, b"")
    with pytest.raises(ValueError):
        sdo.build_download(0x2000, 0, bytes(5))


def test_build_upload():
    assert sdo.build_upload(0x2003, 2) == bytes([0x40, 0x03, 0x20, 0x02, 0, 0, 0, 0])


def test_parse_response_upload_sizes():
    r = sdo.parse_response(bytes([0x4F, 0x03, 0x20, 0x01, 0x01, 0, 0, 0]))
    assert r.kind == "upload" and (r.index, r.sub) == (0x2003, 1) and r.data == b"\x01"
    r = sdo.parse_response(bytes([0x4B, 0x03, 0x20, 0x02, 0x34, 0x12, 0, 0]))
    assert r.data == b"\x34\x12" and struct.unpack("<H", r.data)[0] == 0x1234
    r = sdo.parse_response(bytes([0x47, 0x00, 0x10, 0x00, 1, 2, 3, 0]))
    assert r.data == b"\x01\x02\x03"
    r = sdo.parse_response(bytes([0x43, 0x00, 0x10, 0x00, 1, 2, 3, 4]))
    assert r.data == b"\x01\x02\x03\x04"
    r = sdo.parse_response(bytes([0x42, 0x00, 0x10, 0x00, 1, 2, 3, 4]))   # size not indicated
    assert r.data == b"\x01\x02\x03\x04"


def test_parse_response_download_ack_and_abort():
    r = sdo.parse_response(bytes([0x60, 0x00, 0x20, 0x00, 0, 0, 0, 0]))
    assert r.kind == "download" and r.index == 0x2000
    r = sdo.parse_response(bytes([0x80, 0x00, 0x20, 0x00]) + struct.pack("<I", 0x06010002))
    assert r.kind == "abort" and r.abort_code == 0x06010002
    with pytest.raises(sdo.SdoProtocolError):
        sdo.parse_response(bytes(7))
    with pytest.raises(sdo.SdoProtocolError):
        sdo.parse_response(bytes([0x00] * 8))       # segmented download response: unsupported
    with pytest.raises(sdo.SdoProtocolError):
        sdo.parse_response(bytes([0x41, 0, 0x10, 0, 4, 0, 0, 0]))   # segmented upload


def test_parse_request_roundtrip():
    req = sdo.parse_request(sdo.build_download(0x1800, 5, struct.pack("<H", 20)))
    assert req.kind == "download" and req.index == 0x1800 and req.sub == 5 and req.data == struct.pack("<H", 20)
    req = sdo.parse_request(sdo.build_upload(0x2003, 3))
    assert req.kind == "upload" and req.index == 0x2003 and req.sub == 3


def test_server_encoders_roundtrip():
    assert sdo.parse_response(sdo.build_upload_response(0x2003, 2, b"\x10\x27")).data == b"\x10\x27"
    assert sdo.parse_response(sdo.build_download_response(0x2000, 0)).kind == "download"


# ------------------------------------------------------------- fake sensor
class FakeBus:
    """Minimal stand-in for CanBus with a simulated MTS160 SDO server."""

    def __init__(self, node_id=10, delay_s=0.0, answer=True):
        self.node_id = node_id
        self.listeners = []
        self.sent = []
        self.delay_s = delay_s
        self.answer = answer
        self.od = {(0x2003, 1): b"\x01", (0x2003, 2): struct.pack("<H", 420), (0x2003, 3): struct.pack("<H", 1234),
                   (0x1800, 5): struct.pack("<H", 10), (0x2002, 1): b"\x00"}
        self.write_only = {(0x2000, 0), (0x2001, 0)}
        self.read_only = {(0x2003, 1), (0x2003, 2), (0x2003, 3)}
        self.writes = []

    def add_listener(self, cb):
        self.listeners.append(cb)

    def remove_listener(self, cb):
        self.listeners = [c for c in self.listeners if c is not cb]

    def _deliver(self, data):
        f = Frame(can_id=0x580 + self.node_id, data=bytes(data), t_wall=time.time(), t_mono=time.monotonic())
        for cb in list(self.listeners):
            cb(f)

    def send(self, can_id, data):
        self.sent.append((can_id, bytes(data)))
        assert can_id == 0x600 + self.node_id
        if not self.answer:
            return
        req = sdo.parse_request(data)
        key = (req.index, req.sub)
        if req.kind == "upload":
            if key in self.write_only:
                resp = sdo.build_abort(req.index, req.sub, 0x06010001)
            elif key in self.od:
                resp = sdo.build_upload_response(req.index, req.sub, self.od[key])
            else:
                resp = sdo.build_abort(req.index, req.sub, 0x06020000)
        else:
            if key in self.read_only:
                resp = sdo.build_abort(req.index, req.sub, 0x06010002)
            elif key in self.write_only or key in self.od:
                self.writes.append((req.index, req.sub, req.data))
                if key in self.od:
                    self.od[key] = req.data
                resp = sdo.build_download_response(req.index, req.sub)
            else:
                resp = sdo.build_abort(req.index, req.sub, 0x06020000)

        def go():
            if self.delay_s:
                time.sleep(self.delay_s)
            self._deliver(resp)
        threading.Thread(target=go, daemon=True).start()


def test_client_reads_selftest_registers():
    bus = FakeBus(delay_s=0.005)
    c = sdo.SdoClient(bus, 10, timeout_s=0.5)
    assert c.read_u8(0x2003, 1) == 1
    assert c.read_u16(0x2003, 2) == 420
    assert c.read_u16(0x2003, 3) == 1234
    assert c.read_selftest() == (True, 420, 1234)
    assert c.get_tpdo_period(1) == 10
    assert c.transactions == 7 and c.timeouts == 0


def test_client_writes_and_helpers():
    bus = FakeBus()
    c = sdo.SdoClient(bus, 10, timeout_s=0.5)
    c.start_zero()
    c.start_selftest()
    c.set_tpdo_period(1, 5)
    assert bus.writes == [(0x2000, 0, b"\x01"), (0x2001, 0, b"\x01"), (0x1800, 5, struct.pack("<H", 5))]
    assert c.get_tpdo_period(1) == 5
    passed, mn, mx = c.run_selftest(settle_s=0.001)
    assert passed and (mn, mx) == (420, 1234)
    with pytest.raises(ValueError):
        c.set_tpdo_period(4, 10)
    with pytest.raises(ValueError):
        c.set_tpdo_period(1, 70000)


def test_client_abort_and_timeout():
    bus = FakeBus()
    c = sdo.SdoClient(bus, 10, timeout_s=0.5)
    with pytest.raises(sdo.SdoAbort) as ei:
        c.read_u8(0x2001, 0)          # write-only
    assert ei.value.code == 0x06010001 and "write-only" in str(ei.value)
    with pytest.raises(sdo.SdoAbort):
        c.write_u8(0x2003, 1, 0)      # read-only
    with pytest.raises(sdo.SdoAbort):
        c.read_u8(0x5555, 0)          # nonexistent
    assert c.aborts == 3

    silent = FakeBus(answer=False)
    c2 = sdo.SdoClient(silent, 10, timeout_s=0.05, retries=1)
    t0 = time.monotonic()
    with pytest.raises(sdo.SdoTimeout):
        c2.read_u8(0x2003, 1)
    assert 0.09 <= time.monotonic() - t0 < 1.0    # two attempts of 50 ms
    assert c2.timeouts == 2 and len(silent.sent) == 2


def test_client_ignores_stale_and_foreign_frames():
    bus = FakeBus(delay_s=0.01)
    c = sdo.SdoClient(bus, 10, timeout_s=0.5)
    # a stale response for another object arrives before ours
    bus._deliver(sdo.build_upload_response(0x1234, 0, b"\xAA"))
    # and a frame for another node must never be queued
    other = Frame(can_id=0x58B, data=sdo.build_upload_response(0x2003, 1, b"\x00"), t_wall=0, t_mono=0)
    for cb in bus.listeners:
        cb(other)
    assert c.read_u8(0x2003, 1) == 1


def test_client_rejects_bad_node():
    with pytest.raises(ValueError):
        sdo.SdoClient(FakeBus(), 0)
