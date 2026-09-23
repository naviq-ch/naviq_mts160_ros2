"""Minimal tape-following controller: ``mts160/track`` -> ``/cmd_vel``.

A proportional steering law on lateral offset and incidence angle::

    omega = -(k_pos * position_m + k_angle * angle_rad) * steer_sign

Stops when no tape is detected or the track message is stale.  This is an
example, not a navigation stack: tune the gains and the sign for your base
(see README, "Sign conventions").
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

from naviq_msgs.msg import TrackDetection


class LineFollower(Node):
    def __init__(self):
        super().__init__("mts160_line_follower")
        self.declare_parameter("track_topic", "/mts160/track")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("side", "left")          # which reported track to follow: left / right
        self.declare_parameter("linear_speed", 0.15)    # m/s while tape is detected
        self.declare_parameter("k_pos", 4.0)            # rad/s per metre of lateral offset
        self.declare_parameter("k_angle", 1.5)          # rad/s per radian of incidence angle
        self.declare_parameter("steer_sign", 1.0)       # flip if the robot steers away from the tape
        self.declare_parameter("max_angular", 1.0)      # rad/s
        self.declare_parameter("stale_s", 0.2)          # stop if no track message for this long
        self.declare_parameter("rate_hz", 50.0)

        self.side = str(self.get_parameter("side").value)
        self.pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.sub = self.create_subscription(TrackDetection, str(self.get_parameter("track_topic").value),
                                            self._on_track, qos_profile_sensor_data)
        self._last = None
        self._last_t = 0.0
        self.timer = self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)

    def _on_track(self, msg: TrackDetection):
        self._last = msg
        self._last_t = time.monotonic()

    def _tick(self):
        cmd = Twist()
        msg = self._last
        stale = (time.monotonic() - self._last_t) > float(self.get_parameter("stale_s").value)
        if msg is None or stale or not msg.tape_detected:
            self.pub.publish(cmd)   # zero velocity
            return
        track = msg.right if self.side == "right" else msg.left
        pos_m = track.position_mm / 1000.0
        ang = math.radians(track.angle_deg)
        k_pos = float(self.get_parameter("k_pos").value)
        k_ang = float(self.get_parameter("k_angle").value)
        sign = float(self.get_parameter("steer_sign").value)
        omega = -(k_pos * pos_m + k_ang * ang) * sign
        lim = float(self.get_parameter("max_angular").value)
        omega = max(-lim, min(lim, omega))
        cmd.linear.x = float(self.get_parameter("linear_speed").value)
        cmd.angular.z = omega
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
