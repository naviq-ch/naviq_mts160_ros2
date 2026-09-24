"""diagnostic_updater tasks for the MTS160 driver.

The driver keeps a :class:`Stats` object that the CAN reader thread updates;
the tasks here read it from the executor thread once per second and produce
``/diagnostics`` entries: bus state, heartbeat/NMT state, per-TPDO rates,
data timeout, frame-length errors, SDO errors and receive-to-publish
latency.
"""

from __future__ import annotations

import collections
import statistics
import threading
import time
from dataclasses import dataclass, field
from typing import Deque, Dict, Optional

from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater

from . import decoder


@dataclass
class Stats:
    """Counters shared between the CAN thread and the diagnostics timer."""

    lock: threading.Lock = field(default_factory=threading.Lock)
    heartbeat_count: int = 0
    last_heartbeat_mono: Optional[float] = None
    nmt_state: Optional[int] = None
    tpdo_counts: Dict[int, int] = field(default_factory=lambda: {1: 0, 2: 0, 3: 0})
    last_tpdo1_mono: Optional[float] = None
    last_track: Optional[decoder.TrackFrame] = None
    bad_length_count: int = 0
    unknown_frame_count: int = 0
    other_node_count: int = 0
    sdo_response_count: int = 0
    sdo_errors: int = 0
    last_sdo_error: str = ""
    latency_s: Deque[float] = field(default_factory=lambda: collections.deque(maxlen=2000))
    tpdo1_intervals_s: Deque[float] = field(default_factory=lambda: collections.deque(maxlen=2000))


class Mts160Diagnostics:
    """Registers the diagnostic tasks on an Updater owned by the driver node."""

    def __init__(self, node, stats: Stats, bus, node_id: int, timeout_ms: int,
                 heartbeat_timeout_s: float = 3.0, period_s: float = 1.0):
        self._stats = stats
        self._bus = bus
        self._timeout_s = timeout_ms / 1000.0
        self._hb_timeout_s = heartbeat_timeout_s
        self._prev_counts = {1: 0, 2: 0, 3: 0}
        self._prev_time = time.monotonic()
        self._prev_bad_length = 0
        self.updater = diagnostic_updater.Updater(node, period=period_s)
        self.updater.setHardwareID(f"mts160 node {node_id} ({bus.interface_type}/{bus.channel})")
        self.updater.add("CAN bus", self._bus_task)
        self.updater.add("Heartbeat", self._heartbeat_task)
        self.updater.add("Sensor data", self._data_task)
        self.updater.add("Frame errors", self._frame_errors_task)
        self.updater.add("SDO", self._sdo_task)
        self.updater.add("Latency", self._latency_task)

    # ------------------------------------------------------------------ tasks
    def _bus_task(self, stat):
        bus = self._bus
        if bus.connected:
            stat.summary(DiagnosticStatus.OK, "connected")
        else:
            stat.summary(DiagnosticStatus.ERROR, f"disconnected ({bus.last_error or 'not opened'})")
        stat.add("interface_type", bus.interface_type)
        stat.add("channel", bus.channel)
        stat.add("bitrate", str(bus.bitrate))
        stat.add("rx_frames", str(bus.rx_count))
        stat.add("tx_frames", str(bus.tx_count))
        stat.add("error_frames", str(bus.error_frames))
        stat.add("reconnects", str(bus.reconnects))
        stat.add("open_failures", str(bus.open_failures))
        stat.add("last_error", bus.last_error)
        return stat

    def _heartbeat_task(self, stat):
        s = self._stats
        now = time.monotonic()
        with s.lock:
            last = s.last_heartbeat_mono
            count = s.heartbeat_count
            state = s.nmt_state
        if last is None:
            stat.summary(DiagnosticStatus.WARN, "no heartbeat seen yet")
            age = float("nan")
        else:
            age = now - last
            name = decoder.NMT_STATE_NAMES.get(state, f"0x{state:02X}") if state is not None else "?"
            if age > self._hb_timeout_s:
                stat.summary(DiagnosticStatus.ERROR, f"heartbeat lost ({age:.1f} s since last)")
            elif state != decoder.NMT_OPERATIONAL:
                stat.summary(DiagnosticStatus.WARN, f"NMT state {name}: TPDOs only sent when operational")
            else:
                stat.summary(DiagnosticStatus.OK, f"NMT {name}")
        stat.add("nmt_state", decoder.NMT_STATE_NAMES.get(state, str(state)) if state is not None else "unknown")
        stat.add("age_s", f"{age:.3f}")
        stat.add("count", str(count))
        return stat

    def _data_task(self, stat):
        s = self._stats
        now = time.monotonic()
        with s.lock:
            counts = dict(s.tpdo_counts)
            last1 = s.last_tpdo1_mono
            track = s.last_track
            intervals = list(s.tpdo1_intervals_s)
        dt = max(now - self._prev_time, 1e-6)
        rates = {k: (counts[k] - self._prev_counts[k]) / dt for k in counts}
        self._prev_counts = counts
        self._prev_time = now

        if last1 is None:
            stat.summary(DiagnosticStatus.ERROR, "no TPDO1 received")
        else:
            age = now - last1
            if age > self._timeout_s:
                stat.summary(DiagnosticStatus.ERROR, f"TPDO1 timeout: {age * 1000:.0f} ms since last frame")
            else:
                stat.summary(DiagnosticStatus.OK, f"TPDO1 {rates[1]:.1f} Hz")
        for k in (1, 2, 3):
            stat.add(f"tpdo{k}_rate_hz", f"{rates[k]:.2f}")
            stat.add(f"tpdo{k}_count", str(counts[k]))
        if intervals:
            stat.add("tpdo1_interval_mean_ms", f"{statistics.fmean(intervals) * 1000:.3f}")
            stat.add("tpdo1_interval_max_ms", f"{max(intervals) * 1000:.3f}")
        if track is not None:
            stat.add("strength", track.strength_name)
            stat.add("left_position_mm", str(track.left_position_mm))
            stat.add("right_position_mm", str(track.right_position_mm))
            stat.add("left_angle_deg", str(track.left_angle_deg))
            stat.add("right_angle_deg", str(track.right_angle_deg))
            stat.add("flags", ",".join(
                n for n, v in (("Lmark", track.left_marker), ("Rmark", track.right_marker),
                               ("intersection", track.intersection), ("fork", track.fork),
                               ("merge", track.merge)) if v) or "-")
        return stat

    def _frame_errors_task(self, stat):
        s = self._stats
        with s.lock:
            bad = s.bad_length_count
            unknown = s.unknown_frame_count
            other = s.other_node_count
        if bad > self._prev_bad_length:
            stat.summary(DiagnosticStatus.WARN, f"{bad - self._prev_bad_length} wrong-length frames dropped")
        else:
            stat.summary(DiagnosticStatus.OK, "no frame errors")
        self._prev_bad_length = bad
        stat.add("bad_length_total", str(bad))
        stat.add("unknown_function_code_total", str(unknown))
        stat.add("other_node_frames_total", str(other))
        return stat

    def _sdo_task(self, stat):
        s = self._stats
        with s.lock:
            errs = s.sdo_errors
            last_err = s.last_sdo_error
            responses = s.sdo_response_count
        if errs:
            stat.summary(DiagnosticStatus.WARN, f"{errs} SDO error(s): {last_err}")
        else:
            stat.summary(DiagnosticStatus.OK, "no SDO errors")
        stat.add("sdo_responses_total", str(responses))
        stat.add("sdo_errors", str(errs))
        stat.add("last_sdo_error", last_err)
        return stat

    def _latency_task(self, stat):
        s = self._stats
        with s.lock:
            lat = list(s.latency_s)
        if not lat:
            stat.summary(DiagnosticStatus.OK, "no samples")
            return stat
        lat_us = [v * 1e6 for v in lat]
        stat.summary(DiagnosticStatus.OK, f"recv->publish mean {statistics.fmean(lat_us):.0f} us")
        stat.add("samples", str(len(lat_us)))
        stat.add("mean_us", f"{statistics.fmean(lat_us):.1f}")
        stat.add("max_us", f"{max(lat_us):.1f}")
        if len(lat_us) > 1:
            stat.add("stdev_us", f"{statistics.pstdev(lat_us):.1f}")
        return stat
