"""Replay tests.

1. Every recorded fixture pair in ``test/fixtures`` is run through the pure
   decoder and checked against its ground-truth JSON.
2. The same frames are pushed through a live ``Mts160Driver`` node on
   python-can's in-process ``virtual`` bus; the published messages must match
   the decoder output frame for frame.
3. If ``vcan0`` exists (CI), the node is started as a separate process on
   socketcan and the log is replayed onto ``vcan0`` (what ``canplayer`` does).
"""

import os
import subprocess
import sys
import threading
import time
import uuid

import pytest

import can

from naviq_mts160 import decoder

sys.path.insert(0, os.path.dirname(__file__))
import fixture_utils as fu  # noqa: E402

FIXTURE_DIR = os.path.join(os.path.dirname(__file__), "fixtures")
NODE = 10


# ------------------------------------------------------------------ helpers
def _all_fixture_pairs(tmp_path_factory):
    """Recorded fixtures plus one synthetic session (so the machinery is always exercised)."""
    pairs = fu.fixture_pairs(FIXTURE_DIR)
    tmp = tmp_path_factory.mktemp("synthetic")
    log = os.path.join(str(tmp), "synthetic_centred.log")
    js = os.path.join(str(tmp), "synthetic_centred.json")
    session = fu.synthetic_session(NODE, seconds=1.0, lpos=-3, rpos=-3, lang=2, rang=2, strength=2,
                                   markers=(-235, 120, 0, 0), code=1, counter=3, left_marker=True)
    fu.write_candump(log, session)
    import json
    with open(js, "w") as fh:
        json.dump({
            "description": "synthetic centred track",
            "node_id": NODE,
            "segments": [{
                "name": "all", "t_start": 0.0, "t_end": 1.0,
                "printer": {"x": 220, "y": 150, "z": 20, "e": 0, "yaw_deg": 0},
                "expect": {
                    "strength": {"min": 2},
                    "left_position_mm": {"mean": -3.0, "tol": 0.01},
                    "right_position_mm": {"mean": -3.0, "tol": 0.01},
                    "left_angle_deg": {"mean": 2.0, "tol": 0.01},
                    "single_track": {"fraction_true_min": 1.0},
                    "left_marker": {"fraction_true_min": 1.0},
                    "fork": {"fraction_true_max": 0.0},
                    "navicode_counter_changes": 0,
                    "tpdo1_rate_hz": {"min": 95, "max": 105},
                },
            }],
        }, fh)
    pairs.append((log, js))
    return pairs


@pytest.fixture(scope="module")
def fixture_pairs(tmp_path_factory):
    return _all_fixture_pairs(tmp_path_factory)


# ------------------------------------------------------ 1. decoder vs labels
def test_fixtures_against_labels(fixture_pairs):
    assert fixture_pairs, "no fixtures"
    failures = []
    for log, js in fixture_pairs:
        frames = fu.read_candump(log)
        fx = fu.load_fixture(js)
        failures.extend(f"{os.path.basename(log)}: {f}" for f in fu.evaluate_fixture(frames, fx))
    assert not failures, "\n".join(failures)


def test_synthetic_expectation_engine_detects_errors(tmp_path):
    log = os.path.join(str(tmp_path), "bad.log")
    fu.write_candump(log, fu.synthetic_session(NODE, seconds=0.5, lpos=10, rpos=12, strength=1))
    frames = fu.read_candump(log)
    fx = {"node_id": NODE, "segments": [{"name": "s", "t_start": 0, "t_end": 1,
                                        "expect": {"strength": {"min": 2}, "single_track": {"fraction_true_min": 0.5},
                                                   "left_position_mm": {"mean": 0, "tol": 1}}}]}
    fails = fu.evaluate_fixture(frames, fx)
    assert len(fails) == 3


# ------------------------------------------- 2. driver node on a virtual bus
@pytest.fixture(scope="module")
def rclpy_ctx():
    rclpy = pytest.importorskip("rclpy")
    rclpy.init()
    yield rclpy
    rclpy.try_shutdown()


def _play(log_frames, chan, speed=20.0):
    """Send recorded frames on the virtual channel with compressed timing."""
    peer = can.Bus(interface="virtual", channel=chan)
    try:
        t_first = log_frames[0].t
        start = time.monotonic()
        for f in log_frames:
            due = start + (f.t - t_first) / speed
            delay = due - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            peer.send(can.Message(arbitration_id=f.can_id, data=f.data, is_extended_id=False))
    finally:
        peer.shutdown()


def test_driver_publishes_recorded_frames(rclpy_ctx, fixture_pairs):
    rclpy = rclpy_ctx
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from naviq_msgs.msg import Markers, Navicode, TrackDetection
    from naviq_mts160.driver import Mts160Driver
    from rclpy.parameter import Parameter

    # best-effort like the driver, but deep enough for a 20x-speed burst (the default sensor-data depth is 5)
    deep = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5000)
    # navicode is reliable; a deep reader history so a slow CI runner cannot overflow it during the burst
    deep_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5000)

    for log, js in fixture_pairs:
        frames = fu.read_candump(log)
        fx = fu.load_fixture(js)
        node_id = int(fx.get("node_id", NODE))
        expected = fu.decode_log(frames, node_id)
        exp_tracks = [s.value for s in expected if s.kind == "track"]
        exp_markers = [s.value for s in expected if s.kind == "markers"]
        exp_navi = [s.value for s in expected if s.kind == "navicode"]

        chan = "replay-" + uuid.uuid4().hex[:8]
        ns = "/replay_" + uuid.uuid4().hex[:6]        # own namespace (~/track -> <ns>/mts160/track): a live driver on the host must not feed the listener
        driver = Mts160Driver(namespace=ns, parameter_overrides=[
            Parameter("can_interface_type", value="virtual"),
            Parameter("can_channel", value=chan),
            Parameter("node_id", value=node_id),
            Parameter("publish_raw", value=True),
        ])
        listener = rclpy.create_node("replay_listener_" + uuid.uuid4().hex[:6])
        got = {"track": [], "markers": [], "navicode": [], "raw": []}
        listener.create_subscription(TrackDetection, f"{ns}/mts160/track", lambda m: got["track"].append(m), deep)
        listener.create_subscription(Markers, f"{ns}/mts160/markers", lambda m: got["markers"].append(m), deep)
        listener.create_subscription(Navicode, f"{ns}/mts160/navicode", lambda m: got["navicode"].append(m), deep_reliable)
        from naviq_msgs.msg import RawTpdo
        listener.create_subscription(RawTpdo, f"{ns}/mts160/raw", lambda m: got["raw"].append(m), deep)

        ex = SingleThreadedExecutor()
        ex.add_node(driver)
        ex.add_node(listener)
        spin = threading.Thread(target=ex.spin, daemon=True)
        spin.start()
        try:
            # wait for discovery: every driver publisher must see the listener's subscription
            pubs = [driver.pub_track, driver.pub_markers, driver.pub_navicode, driver.pub_raw]
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline and any(pb.get_subscription_count() < 1 for pb in pubs):
                time.sleep(0.02)
            assert all(pb.get_subscription_count() >= 1 for pb in pubs), "subscriptions did not match"
            time.sleep(0.1)
            assert driver.bus.connected
            _play(frames, chan, speed=20.0)
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and (len(got["track"]) < len(exp_tracks) or len(got["navicode"]) < len(exp_navi)
                                                   or len(got["raw"]) < len(frames) or len(got["markers"]) < len(exp_markers)):
                time.sleep(0.05)
        finally:
            ex.shutdown(timeout_sec=1.0)
            driver.destroy_node()
            listener.destroy_node()

        # Reliable navicode: exact. Best-effort track/markers over loopback: allow tiny loss, require order.
        assert len(got["navicode"]) == len(exp_navi), os.path.basename(log)
        assert len(got["track"]) >= 0.98 * len(exp_tracks), f"{os.path.basename(log)}: {len(got['track'])}/{len(exp_tracks)} tracks"
        assert len(got["markers"]) >= 0.98 * len(exp_markers)
        assert len(got["raw"]) >= 0.98 * len(frames)

        # Field-level comparison (published stream is a subsequence of the expected one)
        it = iter(exp_tracks)
        for m in got["track"]:
            for e in it:
                if (m.left.position_mm, m.right.position_mm, m.left.angle_deg, m.right.angle_deg, m.strength) == \
                        (e.left_position_mm, e.right_position_mm, e.left_angle_deg, e.right_angle_deg, e.strength):
                    assert m.tape_detected == e.tape_detected and m.single_track == e.single_track
                    assert (m.left_marker, m.right_marker, m.intersection, m.fork, m.merge) == \
                           (e.left_marker, e.right_marker, e.intersection, e.fork, e.merge)
                    assert m.header.frame_id == "mts160_link"
                    break
            else:
                pytest.fail(f"published track not in expected sequence: {m}")

        prev = None
        for m, e in zip(got["navicode"], exp_navi):
            assert m.code == e.code and m.counter == e.counter
            assert m.is_new == decoder.counter_advanced(prev, e.counter)
            prev = e.counter

        for m in got["markers"][:50]:
            # float32 fields: 0.1 mm/LSB values above 16 mm carry ~2e-6 of rounding, so compare at 1e-3
            assert any(abs(m.left_x_mm - e.left_x_mm) < 1e-3 and abs(m.left_y_mm - e.left_y_mm) < 1e-3
                       and abs(m.right_x_mm - e.right_x_mm) < 1e-3 and abs(m.right_y_mm - e.right_y_mm) < 1e-3
                       for e in exp_markers)


def test_driver_counts_bad_lengths_and_filters_nodes(rclpy_ctx):
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from naviq_mts160.driver import Mts160Driver

    chan = "badlen-" + uuid.uuid4().hex[:8]
    driver = Mts160Driver(parameter_overrides=[
        Parameter("can_interface_type", value="virtual"),
        Parameter("can_channel", value=chan),
        Parameter("node_id", value=NODE),
    ])
    ex = SingleThreadedExecutor()
    ex.add_node(driver)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    peer = can.Bus(interface="virtual", channel=chan)
    try:
        time.sleep(0.2)
        peer.send(can.Message(arbitration_id=0x18A, data=bytes(4), is_extended_id=False))      # wrong length
        peer.send(can.Message(arbitration_id=0x18B, data=bytes(5), is_extended_id=False))      # other node
        peer.send(can.Message(arbitration_id=0x08A, data=bytes(8), is_extended_id=False))      # EMCY: unknown fc
        peer.send(can.Message(arbitration_id=0x18A, data=bytes(5), is_extended_id=False))      # good
        peer.send(can.Message(arbitration_id=0x70A, data=b"\x05", is_extended_id=False))       # heartbeat
        time.sleep(0.3)
        with driver.stats.lock:
            assert driver.stats.bad_length_count == 1
            assert driver.stats.other_node_count == 1
            assert driver.stats.unknown_frame_count == 1
            assert driver.stats.tpdo_counts[1] == 1
            assert driver.stats.heartbeat_count == 1 and driver.stats.nmt_state == 0x05
    finally:
        peer.shutdown()
        ex.shutdown(timeout_sec=1.0)
        driver.destroy_node()


def test_driver_rejects_invalid_node_id(rclpy_ctx):
    from rclpy.parameter import Parameter
    from rclpy.exceptions import InvalidParameterValueException
    from naviq_mts160.driver import Mts160Driver
    with pytest.raises(InvalidParameterValueException):
        Mts160Driver(parameter_overrides=[Parameter("can_interface_type", value="virtual"),
                                          Parameter("can_channel", value="x"),
                                          Parameter("node_id", value=200)], start_bus=False)


# -------------------------------------------------- 3. vcan0 + socketcan
def _vcan_available():
    return sys.platform.startswith("linux") and os.path.exists("/sys/class/net/vcan0")


@pytest.mark.skipif(not _vcan_available(), reason="vcan0 not present (no SocketCAN in WSL2)")
def test_replay_on_vcan0(rclpy_ctx, tmp_path):
    """The node as a separate process on SocketCAN: a synthetic 3 s stream is replayed onto vcan0 with
    its original timing (what canplayer would do) and the published tracks are compared with the decoder."""
    import signal
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import SingleThreadedExecutor
    from diagnostic_msgs.msg import DiagnosticArray
    from naviq_msgs.msg import TrackDetection

    node_id = NODE
    session = fu.synthetic_session(node_id, seconds=3.0, lpos=-3, rpos=-3, lang=2, rang=2, strength=2,
                                   markers=(120, -40, 0, 0), left_marker=True)
    frames = [fu.LogFrame(t, cid, data) for t, cid, data in session]
    expected = [s.value for s in fu.decode_log(frames, node_id) if s.kind == "track"]

    # own process group: "ros2 run" does not forward SIGTERM to the node, so signal the whole group
    proc = subprocess.Popen(
        ["ros2", "run", "naviq_mts160", "mts160", "--ros-args",
         "-r", "__ns:=/replay_vcan",
         "-p", "can_interface_type:=socketcan", "-p", "can_channel:=vcan0", "-p", f"node_id:={node_id}"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    listener = rclpy_ctx.create_node("vcan_listener")
    got = []
    diag = {}
    sub = listener.create_subscription(TrackDetection, "/replay_vcan/mts160/track", got.append, qos_profile_sensor_data)

    def on_diag(msg):
        for st in msg.status:
            diag[st.name] = (st.message, {kv.key: kv.value for kv in st.values})
    listener.create_subscription(DiagnosticArray, "/diagnostics", on_diag, 10)
    ex = SingleThreadedExecutor()
    ex.add_node(listener)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    try:
        # wait until the driver process is up and its publisher has MATCHED our subscription (slow CI runners)
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and sub.get_publisher_count() < 1:
            assert proc.poll() is None, "driver process exited before publishing"
            time.sleep(0.1)
        assert sub.get_publisher_count() >= 1, "driver publisher on vcan0 not matched"
        time.sleep(0.5)
        # canplayer-equivalent: replay with original timing onto vcan0
        bus = can.Bus(interface="socketcan", channel="vcan0")
        try:
            t_first = frames[0].t
            start = time.monotonic()
            for f in frames:
                delay = start + (f.t - t_first) - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                bus.send(can.Message(arbitration_id=f.can_id, data=f.data, is_extended_id=False))
        finally:
            bus.shutdown()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and len(got) < len(expected):
            time.sleep(0.05)
        time.sleep(1.2)                                       # one more diagnostics period for the report below
    finally:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)          # graceful: rclpy handles it, node exits 0
        try:
            out, _ = proc.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            out, _ = proc.communicate()
        ex.shutdown(timeout_sec=1.0)
        listener.destroy_node()
    report = f"driver diagnostics: {diag}" + chr(10) + f"driver output: {out[-2000:]}"
    assert len(got) >= 0.98 * len(expected), f"{len(got)}/{len(expected)} tracks" + chr(10) + report
    m, e = got[0], expected[0]
    assert (m.left.position_mm, m.right.position_mm, m.strength) == (e.left_position_mm, e.right_position_mm, e.strength)
    assert m.left_marker and not m.right_marker
