"""python-can bus wrapper: reader thread, listeners, automatic reconnect.

No ROS imports.  Backends are selected by ``interface_type`` exactly as
python-can names them (``socketcan``, ``gs_usb``, ``virtual``, ...).

* ``socketcan`` (default): ``channel`` is the interface name (``can0``);
  the bitrate is configured by the OS and ignored here.
* ``gs_usb``: user-space candleLight/CANable driver over libusb (the WSL2
  bench).  ``channel`` may be a device index (``"0"``), a USB ``bus:address``
  pair (``"1:4"``) or a device serial number; the bitrate is set here.
* ``virtual``: in-process bus used by the tests.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable, List, Optional

import can

log = logging.getLogger("naviq_mts160.canbus")


@dataclass(frozen=True)
class Frame:
    """One received CAN frame with receive-time stamps."""

    can_id: int
    data: bytes
    t_wall: float                 # time.time() at receipt (for ROS stamps)
    t_mono: float                 # time.monotonic() at receipt (for latency/jitter)
    t_bus: Optional[float] = None  # backend/hardware timestamp if provided
    is_extended: bool = False


class CanBusError(Exception):
    pass


FrameListener = Callable[[Frame], None]
StateListener = Callable[[bool, str], None]


def _gs_usb_kwargs(channel: str, bitrate: int) -> dict:
    """Translate ``channel`` into python-can gs_usb constructor arguments."""
    kwargs = {"interface": "gs_usb", "bitrate": int(bitrate)}
    text = str(channel).strip()
    if text.isdigit():
        kwargs["channel"] = text
        kwargs["index"] = int(text)
        return kwargs
    if ":" in text and all(p.isdigit() for p in text.split(":", 1)):
        bus_no, addr = text.split(":", 1)
        kwargs["channel"] = text
        kwargs["bus"] = int(bus_no)
        kwargs["address"] = int(addr)
        return kwargs
    # otherwise: a USB serial number -> resolve to bus/address
    try:
        from gs_usb.gs_usb import GsUsb  # type: ignore
    except Exception as exc:  # pragma: no cover - depends on optional package
        raise CanBusError(f"gs_usb package not available to resolve serial '{text}': {exc}")
    for dev in GsUsb.scan():
        usb_dev = getattr(dev, "gs_usb", None)
        serial = None
        try:
            serial = usb_dev.serial_number if usb_dev is not None else None
        except Exception:
            serial = None
        if serial == text:
            kwargs["channel"] = text
            kwargs["bus"] = usb_dev.bus
            kwargs["address"] = usb_dev.address
            return kwargs
    raise CanBusError(f"no gs_usb device with serial '{text}' found")


def _patch_gs_usb_start() -> None:
    """Make the ``gs_usb`` package's ``GsUsb.start()`` leave the device usable over libusb.

    ``GsUsb.start()`` issues a USB port reset and then immediately sends the
    MODE control request.  Over libusb (native Windows/WinUSB as well as
    usbip into WSL2) the device comes back from that reset *unconfigured*:
    control transfers and the CAN peripheral work (the sensor's frames are
    acknowledged) but the bulk endpoints are not enabled, so no frame ever
    reaches the host.  Measured on the bench 2026-09-23 (candleLight sw 2 /
    hw 1, CANable-MKS).  The Linux kernel driver never resets the device.
    This replacement re-selects the configuration, claims interface 0,
    clears the endpoint halts and sends HOST_FORMAT before starting.
    """
    try:
        import struct
        import usb.core
        import usb.util
        from gs_usb import gs_usb as gsmod
        from gs_usb.gs_usb import GsUsb
    except Exception:  # pragma: no cover - gs_usb not installed
        return
    if getattr(GsUsb, "_naviq_patched", False):
        return
    mode_cls = getattr(gsmod, "DeviceMode", None)
    breq_mode = getattr(gsmod, "_GS_USB_BREQ_MODE", 2)
    breq_host_format = getattr(gsmod, "_GS_USB_BREQ_HOST_FORMAT", 0)
    start_val = getattr(gsmod, "GS_CAN_MODE_START", 1)
    supported = (getattr(gsmod, "GS_CAN_MODE_LISTEN_ONLY", 1) | getattr(gsmod, "GS_CAN_MODE_LOOP_BACK", 2)
                 | getattr(gsmod, "GS_CAN_MODE_ONE_SHOT", 8) | getattr(gsmod, "GS_CAN_MODE_HW_TIMESTAMP", 16))
    default_flags = getattr(gsmod, "GS_CAN_MODE_NORMAL", 0) | getattr(gsmod, "GS_CAN_MODE_HW_TIMESTAMP", 16)

    def start(self, flags=default_flags):
        dev = self.gs_usb
        dev.reset()
        time.sleep(0.2)
        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
        except Exception:
            pass
        try:
            dev.set_configuration()
        except usb.core.USBError as exc:
            log.debug("gs_usb set_configuration: %s", exc)
        try:
            usb.util.claim_interface(dev, 0)
        except usb.core.USBError as exc:
            log.debug("gs_usb claim_interface: %s", exc)
        for ep in (0x81, 0x02):
            try:
                dev.clear_halt(ep)
            except usb.core.USBError:
                pass
        try:
            dev.ctrl_transfer(0x41, breq_host_format, 0, 0, struct.pack("<I", 0x0000BEEF))
        except usb.core.USBError as exc:
            log.debug("gs_usb host format: %s", exc)
        flags &= self.device_capability.feature
        flags &= supported
        self.device_flags = flags
        if mode_cls is not None:
            payload = mode_cls(start_val, flags).pack()
        else:
            payload = struct.pack("<II", start_val, flags)
        dev.ctrl_transfer(0x41, breq_mode, 0, 0, payload)

    GsUsb.start = start
    GsUsb._naviq_patched = True
    log.info("gs_usb: GsUsb.start() patched (configure + claim + clear halts after reset)")


class CanBus:
    """Owns a python-can ``Bus`` and a reader thread.

    Listeners are called from the reader thread for every received data
    frame (error and remote frames are counted and dropped).  If the bus
    raises, it is closed and re-opened with a back-off; state listeners are
    told about connect/disconnect transitions.
    """

    def __init__(
        self,
        interface_type: str = "socketcan",
        channel: str = "can0",
        bitrate: int = 500000,
        reconnect_delay_s: float = 1.0,
        recv_timeout_s: float = 0.2,
        extra_kwargs: Optional[dict] = None,
    ):
        self.interface_type = str(interface_type)
        self.channel = str(channel)
        self.bitrate = int(bitrate)
        self.reconnect_delay_s = float(reconnect_delay_s)
        self.recv_timeout_s = float(recv_timeout_s)
        self.extra_kwargs = dict(extra_kwargs or {})

        self._bus: Optional[can.BusABC] = None
        self._bus_lock = threading.Lock()
        self._listeners: List[FrameListener] = []
        self._state_listeners: List[StateListener] = []
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self.connected = False
        self.connected_since: Optional[float] = None
        self.rx_count = 0
        self.tx_count = 0
        self.error_frames = 0
        self.reconnects = 0
        self.open_failures = 0
        self.last_error: str = ""

    # ------------------------------------------------------------ listeners
    def add_listener(self, cb: FrameListener) -> None:
        self._listeners.append(cb)

    def remove_listener(self, cb: FrameListener) -> None:
        self._listeners = [c for c in self._listeners if c is not cb]

    def add_state_listener(self, cb: StateListener) -> None:
        self._state_listeners.append(cb)

    def _notify_state(self, connected: bool, message: str) -> None:
        for cb in list(self._state_listeners):
            try:
                cb(connected, message)
            except Exception:  # pragma: no cover
                log.exception("state listener raised")

    # ------------------------------------------------------------ lifecycle
    def _build_kwargs(self) -> dict:
        if self.interface_type == "gs_usb":
            _patch_gs_usb_start()
            kwargs = _gs_usb_kwargs(self.channel, self.bitrate)
        elif self.interface_type == "socketcan":
            kwargs = {"interface": "socketcan", "channel": self.channel}
        else:
            kwargs = {"interface": self.interface_type, "channel": self.channel}
            if self.bitrate:
                kwargs["bitrate"] = self.bitrate
        kwargs.update(self.extra_kwargs)
        return kwargs

    def _open(self) -> bool:
        try:
            kwargs = self._build_kwargs()
            bus = can.Bus(**kwargs)
        except Exception as exc:
            self.open_failures += 1
            self.last_error = f"open failed: {exc}"
            log.warning("CAN open failed (%s/%s): %s", self.interface_type, self.channel, exc)
            return False
        with self._bus_lock:
            self._bus = bus
        self.connected = True
        self.connected_since = time.time()
        self.last_error = ""
        log.info("CAN bus open: %s/%s", self.interface_type, self.channel)
        self._notify_state(True, "connected")
        return True

    def _close(self, reason: str) -> None:
        with self._bus_lock:
            bus, self._bus = self._bus, None
        if bus is not None:
            try:
                bus.shutdown()
            except Exception:
                pass
        if self.connected:
            self.connected = False
            self._notify_state(False, reason)

    def start(self, open_now: bool = True) -> bool:
        """Start the reader thread.  Returns whether the first open succeeded."""
        ok = self._open() if open_now else False
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="mts160-can-rx", daemon=True)
        self._thread.start()
        return ok

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None
        self._close("stopped")

    # ---------------------------------------------------------------- I/O
    def send(self, can_id: int, data: bytes) -> None:
        with self._bus_lock:
            bus = self._bus
        if bus is None:
            raise CanBusError("CAN bus not connected")
        msg = can.Message(arbitration_id=int(can_id), data=bytes(data), is_extended_id=False)
        try:
            bus.send(msg)
        except Exception as exc:
            self.last_error = f"send failed: {exc}"
            log.warning("CAN send failed: %s", exc)
            self._close(self.last_error)
            raise CanBusError(self.last_error) from exc
        self.tx_count += 1

    def _run(self) -> None:
        while not self._stop.is_set():
            with self._bus_lock:
                bus = self._bus
            if bus is None:
                if self._open():
                    continue
                self._stop.wait(self.reconnect_delay_s)
                continue
            try:
                msg = bus.recv(timeout=self.recv_timeout_s)
            except Exception as exc:
                self.last_error = f"recv failed: {exc}"
                log.warning("CAN recv failed, reconnecting: %s", exc)
                self.reconnects += 1
                self._close(self.last_error)
                self._stop.wait(self.reconnect_delay_s)
                continue
            if msg is None:
                continue
            if msg.is_error_frame or msg.is_remote_frame:
                self.error_frames += 1
                continue
            if not getattr(msg, "is_rx", True):
                continue   # TX echo (gs_usb)
            t_wall = time.time()
            t_mono = time.monotonic()
            t_bus = float(msg.timestamp) if msg.timestamp else None
            frame = Frame(
                can_id=int(msg.arbitration_id),
                data=bytes(msg.data),
                t_wall=t_wall,
                t_mono=t_mono,
                t_bus=t_bus,
                is_extended=bool(msg.is_extended_id),
            )
            self.rx_count += 1
            for cb in list(self._listeners):
                try:
                    cb(frame)
                except Exception:
                    log.exception("frame listener raised")
