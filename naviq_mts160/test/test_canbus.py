"""CanBus wrapper tests on python-can's in-process ``virtual`` interface."""

import threading
import time
import uuid

import can
import pytest

from naviq_mts160.canbus import CanBus, CanBusError, Frame, _gs_usb_kwargs


def test_gs_usb_kwargs_parsing():
    assert _gs_usb_kwargs("0", 500000) == {"interface": "gs_usb", "bitrate": 500000, "channel": "0", "index": 0}
    assert _gs_usb_kwargs("1:4", 250000) == {"interface": "gs_usb", "bitrate": 250000, "channel": "1:4", "bus": 1, "address": 4}


def test_virtual_bus_roundtrip():
    chan = f"naviq-test-{uuid.uuid4().hex[:8]}"
    got = []
    ev = threading.Event()

    def on_frame(f: Frame):
        got.append(f)
        ev.set()

    bus = CanBus("virtual", chan, 500000)
    bus.add_listener(on_frame)
    states = []
    bus.add_state_listener(lambda c, m: states.append((c, m)))
    assert bus.start() is True
    assert bus.connected and states == [(True, "connected")]

    peer = can.Bus(interface="virtual", channel=chan)
    try:
        peer.send(can.Message(arbitration_id=0x18A, data=b"\x01\x02\x03\x04\x06", is_extended_id=False))
        assert ev.wait(2.0)
        f = got[0]
        assert f.can_id == 0x18A and f.data == b"\x01\x02\x03\x04\x06" and not f.is_extended
        assert abs(f.t_wall - time.time()) < 5 and f.t_mono > 0
        assert bus.rx_count == 1

        bus.send(0x60A, bytes(8))
        m = peer.recv(2.0)
        assert m is not None and m.arbitration_id == 0x60A and bytes(m.data) == bytes(8)
        assert bus.tx_count == 1
    finally:
        peer.shutdown()
        bus.stop()
    assert not bus.connected
    assert states[-1] == (False, "stopped")


def test_send_when_not_connected_raises():
    bus = CanBus("virtual", "unused-" + uuid.uuid4().hex[:6])
    with pytest.raises(CanBusError):
        bus.send(0x60A, b"\x00")


def test_open_failure_is_reported_not_raised():
    bus = CanBus("socketcan", "definitely-not-a-can-interface-" + uuid.uuid4().hex[:4], reconnect_delay_s=0.05)
    assert bus.start() is False
    assert not bus.connected and bus.open_failures >= 1 and bus.last_error
    time.sleep(0.2)
    assert bus.open_failures >= 2      # keeps retrying in the background
    bus.stop()
