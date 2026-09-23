"""launch_testing: the node starts from driver.launch.py, validates its
parameters, and /diagnostics transitions from ERROR (no data) to OK once
frames arrive (over python-can's cross-process ``udp_multicast`` backend
when msgpack is available, otherwise the ERROR state alone is checked)."""

import os
import time
import unittest
import uuid

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

try:
    import msgpack  # noqa: F401
    HAVE_MSGPACK = True
except Exception:
    HAVE_MSGPACK = False

MCAST_GROUP = "239.74.163.%d" % (10 + os.getpid() % 200)
BACKEND = ("udp_multicast", MCAST_GROUP) if HAVE_MSGPACK else ("virtual", "launch-" + uuid.uuid4().hex[:6])


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    driver = launch_ros.actions.Node(
        package="naviq_mts160", executable="mts160", name="mts160", output="screen",
        parameters=[{
            "can_interface_type": BACKEND[0],
            "can_channel": BACKEND[1],
            "node_id": 10,
            "timeout_ms": 100,
        }],
    )
    bad = launch_ros.actions.Node(
        package="naviq_mts160", executable="mts160", name="mts160_bad", output="screen",
        parameters=[{"can_interface_type": "virtual", "can_channel": "x", "node_id": 300}],
    )
    return launch.LaunchDescription([
        driver, bad,
        launch.actions.TimerAction(period=1.0, actions=[launch_testing.actions.ReadyToTest()]),
    ]), {"driver": driver, "bad": bad}


class TestDriverRunning(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("launch_test_listener")
        cls.msgs = []
        cls.node.create_subscription(DiagnosticArray, "/diagnostics", cls.msgs.append,
                                     QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50))

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.try_shutdown()

    def _spin_until(self, pred, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if pred():
                return True
        return False

    @staticmethod
    def _status(msg, name_suffix):
        for s in msg.status:
            if s.name.endswith(name_suffix):
                return s
        return None

    def test_invalid_node_id_exits(self, proc_info, bad):
        proc_info.assertWaitForShutdown(process=bad, timeout=15)
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[1, 2, -6, 134], process=bad)

    def test_diagnostics_error_without_data(self, proc_info, driver):
        self.assertTrue(self._spin_until(lambda: any(self._status(m, "Sensor data") for m in self.msgs), 8.0),
                        "no diagnostics from driver")
        msg = [m for m in self.msgs if self._status(m, "Sensor data")][-1]
        data = self._status(msg, "Sensor data")
        self.assertEqual(data.level, DiagnosticStatus.ERROR, data.message)
        bus = self._status(msg, "CAN bus")
        self.assertIsNotNone(bus)
        self.assertEqual(bus.level, DiagnosticStatus.OK, bus.message)
        self.assertIn("mts160", msg.status[0].hardware_id)

    @unittest.skipUnless(HAVE_MSGPACK, "udp_multicast backend needs msgpack")
    def test_diagnostics_ok_with_data(self, proc_info, driver):
        import can
        from naviq_mts160 import decoder
        bus = can.Bus(interface="udp_multicast", channel=MCAST_GROUP)
        try:
            ids = decoder.cob_ids(10)
            end = time.monotonic() + 6.0
            ok = False
            n = 0
            while time.monotonic() < end and not ok:
                bus.send(can.Message(arbitration_id=ids["tpdo1"], data=decoder.encode_tpdo1(0, 0, 0, 0, 3), is_extended_id=False))
                if n % 100 == 0:
                    bus.send(can.Message(arbitration_id=ids["heartbeat"], data=decoder.encode_heartbeat(0x05), is_extended_id=False))
                n += 1
                rclpy.spin_once(self.node, timeout_sec=0.01)
                for m in self.msgs[-5:]:
                    d = self._status(m, "Sensor data")
                    h = self._status(m, "Heartbeat")
                    if d is not None and d.level == DiagnosticStatus.OK and h is not None and h.level == DiagnosticStatus.OK:
                        ok = True
        finally:
            bus.shutdown()
        self.assertTrue(ok, "diagnostics never transitioned to OK")


@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):
    def test_driver_exit_code(self, proc_info, driver):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15, 130, 143], process=driver)
