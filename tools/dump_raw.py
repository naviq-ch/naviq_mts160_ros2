#!/usr/bin/env python3
"""Print every CAN frame the sensor sends, one line each, as the driver receives it (candump style)
plus the decoded meaning.  Needs the driver running with publish_raw:=true.

    python3 tools/dump_raw.py            # Ctrl-C to stop
    python3 tools/dump_raw.py --seconds 5
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from naviq_msgs.msg import RawTpdo
from naviq_mts160 import decoder


def describe(can_id: int, data: bytes, node_id: int) -> str:
    try:
        d = decoder.decode_frame(can_id, data, node_id)
    except decoder.WrongLengthError as exc:
        return f"wrong length: {exc}"
    if isinstance(d, decoder.TrackFrame):
        flags = [n for n in ("left_marker", "right_marker", "intersection", "fork", "merge") if getattr(d, n)]
        return (f"TPDO1 track: L {d.left_position_mm:+.0f} mm {d.left_angle_deg:+.0f} deg | "
                f"R {d.right_position_mm:+.0f} mm {d.right_angle_deg:+.0f} deg | strength {d.strength} {' '.join(flags)}")
    if isinstance(d, decoder.MarkerFrame):
        return f"TPDO2 markers: L ({d.left_x_mm:+.1f}, {d.left_y_mm:+.1f}) mm  R ({d.right_x_mm:+.1f}, {d.right_y_mm:+.1f}) mm"
    if isinstance(d, decoder.NavicodeFrame):
        return f"TPDO3 navicode: code {d.code} counter {d.counter}"
    if isinstance(d, decoder.HeartbeatFrame):
        return f"heartbeat: NMT state 0x{d.state:02X}"
    return "other"


class Dump(Node):
    def __init__(self, node_id: int, namespace: str):
        super().__init__("mts160_dump")
        self.node_id = node_id
        self.create_subscription(RawTpdo, f"{namespace}/raw", self.on_raw, qos_profile_sensor_data)

    def on_raw(self, m: RawTpdo):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        data = bytes(m.data)
        print(f"({t:.6f}) {m.can_id:03X}#{data.hex().upper():<16} {describe(m.can_id, data, self.node_id)}", flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--node-id", type=int, default=10)
    ap.add_argument("--namespace", default="/mts160")
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0 = until Ctrl-C)")
    args = ap.parse_args(argv)
    rclpy.init()
    node = Dump(args.node_id, args.namespace)
    end = time.monotonic() + args.seconds if args.seconds > 0 else None
    try:
        while rclpy.ok() and (end is None or time.monotonic() < end):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
