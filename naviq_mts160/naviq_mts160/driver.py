"""ROS 2 driver node for the Naviq MTS160 magnetic guide sensor.

Node name ``mts160``.  Reads TPDO1..3 and the heartbeat from the sensor's
CANopen node over python-can (``socketcan`` on a Linux robot, ``gs_usb`` on
the WSL2 bench), publishes ``~/track``, ``~/markers``, ``~/navicode`` and
optionally ``~/raw``, offers the ``~/zero`` service and reports on
``/diagnostics``.

The CAN reader runs in its own thread (see :mod:`naviq_mts160.canbus`);
messages are published from that thread with receive-time stamps.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.msg import IntegerRange, ParameterDescriptor

from naviq_msgs.msg import Markers, Navicode, RawTpdo, Track, TrackDetection
from naviq_msgs.srv import Zero

from . import decoder, sdo
from .canbus import CanBus, CanBusError, Frame
from .diagnostics import Mts160Diagnostics, Stats


def _int_desc(description: str, lo: int, hi: int, read_only: bool = False) -> ParameterDescriptor:
    return ParameterDescriptor(
        description=description,
        integer_range=[IntegerRange(from_value=lo, to_value=hi, step=1)],
        read_only=read_only,
    )


def _desc(description: str, read_only: bool = False) -> ParameterDescriptor:
    return ParameterDescriptor(description=description, read_only=read_only)


class Mts160Driver(Node):
    """The ``mts160`` node."""

    def __init__(self, node_name: str = "mts160", *, start_bus: bool = True, **kwargs):
        super().__init__(node_name, **kwargs)

        # ------------------------------------------------------- parameters
        self.declare_parameter("can_interface_type", "socketcan",
                               _desc("python-can backend: socketcan, gs_usb, virtual, ...", True))
        self.declare_parameter("can_channel", "can0",
                               _desc("interface name (socketcan) or device index/bus:address/serial (gs_usb)", True))
        self.declare_parameter("can_bitrate", 500000,
                               _int_desc("bit/s; used only by backends that set it themselves (gs_usb)", 10000, 1000000, True))
        self.declare_parameter("node_id", 10, _int_desc("CANopen node ID of the sensor", 1, 127, True))
        self.declare_parameter("frame_id", "mts160_link", _desc("frame_id for all stamped messages"))
        self.declare_parameter("timeout_ms", 100, _int_desc("no TPDO1 for this long -> diagnostics ERROR", 1, 60000))
        self.declare_parameter("publish_raw", False, _desc("publish every frame from the node on ~/raw"))
        self.declare_parameter("invert_position", False,
                               _desc("mounting flip: negate track positions and the marker lateral (X) axis"))
        self.declare_parameter("invert_angle", False, _desc("mounting flip: negate track angles"))
        self.declare_parameter("tpdo1_period_ms", 0,
                               _int_desc("0 = leave the sensor alone; >0 = write SDO 0x1800:5 at startup (RAM only)", 0, 65535, True))
        self.declare_parameter("tpdo2_period_ms", 0, _int_desc("same for 0x1801:5", 0, 65535, True))
        self.declare_parameter("tpdo3_period_ms", 0, _int_desc("same for 0x1802:5", 0, 65535, True))
        self.declare_parameter("sdo_timeout_ms", 500, _int_desc("SDO response timeout", 10, 10000, True))
        self.declare_parameter("heartbeat_timeout_s", 3.0, _desc("heartbeat older than this -> diagnostics ERROR"))

        self.interface_type = self.get_parameter("can_interface_type").value
        self.channel = self.get_parameter("can_channel").value
        self.bitrate = int(self.get_parameter("can_bitrate").value)
        self.node_id = int(self.get_parameter("node_id").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.timeout_ms = int(self.get_parameter("timeout_ms").value)
        self.publish_raw = bool(self.get_parameter("publish_raw").value)
        self.invert_position = bool(self.get_parameter("invert_position").value)
        self.invert_angle = bool(self.get_parameter("invert_angle").value)
        self.tpdo_periods = {
            1: int(self.get_parameter("tpdo1_period_ms").value),
            2: int(self.get_parameter("tpdo2_period_ms").value),
            3: int(self.get_parameter("tpdo3_period_ms").value),
        }
        self.sdo_timeout_s = int(self.get_parameter("sdo_timeout_ms").value) / 1000.0
        hb_timeout = float(self.get_parameter("heartbeat_timeout_s").value)
        if not self.interface_type:
            raise ValueError("can_interface_type must not be empty")
        if not self.channel:
            raise ValueError("can_channel must not be empty")
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.cob = decoder.cob_ids(self.node_id)

        # ------------------------------------------------------- publishers
        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_track = self.create_publisher(TrackDetection, "~/track", qos_profile_sensor_data)
        self.pub_markers = self.create_publisher(Markers, "~/markers", qos_profile_sensor_data)
        self.pub_navicode = self.create_publisher(Navicode, "~/navicode", reliable)
        self.pub_raw = self.create_publisher(RawTpdo, "~/raw", qos_profile_sensor_data) if self.publish_raw else None

        # ---------------------------------------------------------- state
        self.stats = Stats()
        self._last_track: Optional[decoder.TrackFrame] = None
        self._last_navicode_counter: Optional[int] = None
        self._closing = False

        # ------------------------------------------------------------- bus
        self.bus = CanBus(self.interface_type, self.channel, self.bitrate)
        self.bus.add_listener(self._on_frame)
        self.bus.add_state_listener(self._on_bus_state)
        self.sdo = sdo.SdoClient(self.bus, self.node_id, timeout_s=self.sdo_timeout_s)

        # -------------------------------------------------------- services
        self._srv_group = MutuallyExclusiveCallbackGroup()
        self.srv_zero = self.create_service(Zero, "~/zero", self._handle_zero,
                                            callback_group=self._srv_group)

        # ----------------------------------------------------- diagnostics
        self.diag = Mts160Diagnostics(self, self.stats, self.bus, self.node_id, self.timeout_ms,
                                      heartbeat_timeout_s=hb_timeout)

        self.get_logger().info(
            f"MTS160 driver: {self.interface_type}/{self.channel} @ {self.bitrate} bit/s, node {self.node_id} "
            f"(TPDO1 0x{self.cob['tpdo1']:03X}, TPDO2 0x{self.cob['tpdo2']:03X}, TPDO3 0x{self.cob['tpdo3']:03X}, "
            f"HB 0x{self.cob['heartbeat']:03X})")

        if start_bus:
            if not self.bus.start():
                self.get_logger().error(
                    f"could not open CAN bus {self.interface_type}/{self.channel}: {self.bus.last_error}; retrying in background")

    # -------------------------------------------------------------- params
    def _on_set_parameters(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == "frame_id":
                self.frame_id = str(p.value)
            elif p.name == "invert_position":
                self.invert_position = bool(p.value)
            elif p.name == "invert_angle":
                self.invert_angle = bool(p.value)
            elif p.name == "timeout_ms":
                self.timeout_ms = int(p.value)
                self.diag._timeout_s = self.timeout_ms / 1000.0
            elif p.name == "heartbeat_timeout_s":
                self.diag._hb_timeout_s = float(p.value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ bus state
    def _on_bus_state(self, connected: bool, message: str) -> None:
        if connected:
            self.get_logger().info(f"CAN bus connected ({message})")
            if any(v > 0 for v in self.tpdo_periods.values()):
                # SDO transactions need the reader thread -> run them on a helper thread
                threading.Thread(target=self._apply_tpdo_periods, name="mts160-sdo-init", daemon=True).start()
        else:
            self.get_logger().warning(f"CAN bus disconnected: {message}")

    def _apply_tpdo_periods(self) -> None:
        time.sleep(0.05)
        for tpdo, period in sorted(self.tpdo_periods.items()):
            if period <= 0:
                continue
            try:
                self.sdo.set_tpdo_period(tpdo, period)
                self.get_logger().info(f"TPDO{tpdo} event timer set to {period} ms (SDO 0x{0x1800 + tpdo - 1:04X}:5, RAM only)")
            except (sdo.SdoError, CanBusError) as exc:
                self._record_sdo_error(f"set TPDO{tpdo} period: {exc}")

    def _record_sdo_error(self, text: str) -> None:
        with self.stats.lock:
            self.stats.sdo_errors += 1
            self.stats.last_sdo_error = text
        self.get_logger().error(text)

    # ---------------------------------------------------------- CAN frames
    @staticmethod
    def _stamp(t_wall: float):
        return Time(nanoseconds=int(t_wall * 1e9)).to_msg()

    def _on_frame(self, frame: Frame) -> None:
        """Called from the CAN reader thread for every received frame."""
        if self._closing:
            return
        if frame.is_extended or not decoder.is_node_frame(frame.can_id, self.node_id):
            with self.stats.lock:
                self.stats.other_node_count += 1
            return

        if self.pub_raw is not None:
            raw = RawTpdo()
            raw.header.stamp = self._stamp(frame.t_wall)
            raw.header.frame_id = self.frame_id
            raw.can_id = frame.can_id
            raw.data = list(frame.data)
            self._publish(self.pub_raw, raw)

        fc = decoder.function_code(frame.can_id)
        if fc == decoder.FC_SDO_TX:
            with self.stats.lock:
                self.stats.sdo_response_count += 1
            return   # handled by SdoClient's own listener

        try:
            decoded = decoder.decode_frame(frame.can_id, frame.data, self.node_id)
        except decoder.WrongLengthError as exc:
            with self.stats.lock:
                self.stats.bad_length_count += 1
            self.get_logger().warning(f"dropped frame: {exc}", throttle_duration_sec=5.0)
            return
        if decoded is None:
            with self.stats.lock:
                self.stats.unknown_frame_count += 1
            return

        if isinstance(decoded, decoder.TrackFrame):
            self._handle_track(decoded, frame)
        elif isinstance(decoded, decoder.MarkerFrame):
            self._handle_markers(decoded, frame)
        elif isinstance(decoded, decoder.NavicodeFrame):
            self._handle_navicode(decoded, frame)
        elif isinstance(decoded, decoder.HeartbeatFrame):
            with self.stats.lock:
                self.stats.heartbeat_count += 1
                self.stats.last_heartbeat_mono = frame.t_mono
                self.stats.nmt_state = decoded.state

    def _publish(self, publisher, msg) -> None:
        if self._closing:
            return
        try:
            publisher.publish(msg)
        except Exception as exc:   # context torn down while the thread is still running
            if not self._closing:
                self.get_logger().warning(f"publish failed: {exc}", throttle_duration_sec=5.0)

    def _handle_track(self, tf: decoder.TrackFrame, frame: Frame) -> None:
        self._last_track = tf
        out = decoder.apply_inversions(tf, self.invert_position, self.invert_angle)
        msg = TrackDetection()
        msg.header.stamp = self._stamp(frame.t_wall)
        msg.header.frame_id = self.frame_id
        msg.left = Track(position_mm=float(out.left_position_mm), angle_deg=float(out.left_angle_deg))
        msg.right = Track(position_mm=float(out.right_position_mm), angle_deg=float(out.right_angle_deg))
        msg.strength = int(out.strength)
        msg.tape_detected = out.tape_detected
        msg.single_track = out.single_track
        msg.left_marker = out.left_marker
        msg.right_marker = out.right_marker
        msg.intersection = out.intersection
        msg.fork = out.fork
        msg.merge = out.merge
        self._publish(self.pub_track, msg)
        now = time.monotonic()
        with self.stats.lock:
            self.stats.tpdo_counts[1] += 1
            if self.stats.last_tpdo1_mono is not None:
                self.stats.tpdo1_intervals_s.append(frame.t_mono - self.stats.last_tpdo1_mono)
            self.stats.last_tpdo1_mono = frame.t_mono
            self.stats.last_track = tf
            self.stats.latency_s.append(now - frame.t_mono)

    def _handle_markers(self, mf: decoder.MarkerFrame, frame: Frame) -> None:
        sx = -1.0 if self.invert_position else 1.0
        last = self._last_track
        msg = Markers()
        msg.header.stamp = self._stamp(frame.t_wall)
        msg.header.frame_id = self.frame_id
        msg.left_detected = bool(last.left_marker) if last is not None else False
        msg.right_detected = bool(last.right_marker) if last is not None else False
        msg.left_x_mm = float(sx * mf.left_x_mm)
        msg.left_y_mm = float(mf.left_y_mm)
        msg.right_x_mm = float(sx * mf.right_x_mm)
        msg.right_y_mm = float(mf.right_y_mm)
        self._publish(self.pub_markers, msg)
        with self.stats.lock:
            self.stats.tpdo_counts[2] += 1

    def _handle_navicode(self, nf: decoder.NavicodeFrame, frame: Frame) -> None:
        msg = Navicode()
        msg.header.stamp = self._stamp(frame.t_wall)
        msg.header.frame_id = self.frame_id
        msg.code = int(nf.code)
        msg.counter = int(nf.counter)
        msg.is_new = decoder.counter_advanced(self._last_navicode_counter, nf.counter)
        self._last_navicode_counter = nf.counter
        self._publish(self.pub_navicode, msg)
        with self.stats.lock:
            self.stats.tpdo_counts[3] += 1

    # ------------------------------------------------------------- services
    def _handle_zero(self, request, response):
        try:
            self.sdo.start_zero()
        except (sdo.SdoError, CanBusError) as exc:
            self._record_sdo_error(f"zero failed: {exc}")
            response.ok = False
            return response
        response.ok = True
        self.get_logger().info("zero-level calibration started (SDO 0x2000); firmware stores it in flash")
        return response

    # -------------------------------------------------------------- teardown
    def destroy_node(self):
        self._closing = True
        try:
            self.sdo.close()
            self.bus.stop()
        finally:
            return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Mts160Driver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
