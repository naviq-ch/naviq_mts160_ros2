# naviq_mts160_ros2

ROS 2 driver for the **Naviq MTS160** magnetic guide sensor
(CANopen TPDOs parsed directly, no CANopen stack), plus the bench tooling used
to validate it against physical ground truth on a motorised 3D-printer
fixture.

| package | what |
|---|---|
| `naviq_msgs` | `TrackDetection`, `Markers`, `Navicode`, `RawTpdo` messages; `Zero` service |
| `naviq_mts160` | the `mts160` node (`rclpy` + `python-can`), launch file, example config, tests |
| `tools/naviq_mts160_fixture` | development-only: printer fixture control, calibration, survey, characterisation, report |

Target: ROS 2 **Jazzy** / Ubuntu 24.04 / Python 3.12. Licence: Apache-2.0.

## Five-minute quickstart

Manuals: [docs/INSTALL.md](docs/INSTALL.md) (install and test) and [docs/USER_MANUAL.md](docs/USER_MANUAL.md) (topics, data formats, parameters, diagnostics, usage).

Prerequisites: a Jazzy install, the sensor on a CAN bus at 500 kbit/s with
node ID 10, auto-run enabled and TPDO1 enabled (factory tool or serial
`!CNCF`), and a CAN adapter.

```bash
# 1. dependencies
sudo apt install ros-jazzy-diagnostic-updater python3-pip can-utils
python3 -m pip install --user --break-system-packages "python-can>=4.4"   # + gs_usb if you use that backend

# 2. build
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/naviq-ch/naviq_mts160_ros2.git
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

# 3a. Linux robot with SocketCAN (kernel gs_usb / any CAN interface)
sudo ip link set can0 up type can bitrate 500000
ros2 launch naviq_mts160 driver.launch.py can_interface_type:=socketcan can_channel:=can0

# 3b. WSL2 / no SocketCAN: candleLight / CANable adapter from user space
ros2 launch naviq_mts160 driver.launch.py can_interface_type:=gs_usb can_channel:=0

# 4. look
ros2 topic echo /mts160/track
ros2 topic echo /diagnostics
```

## Node `mts160`

**Parameters**

| name | default | meaning |
|---|---|---|
| `can_interface_type` | `socketcan` | python-can backend: `socketcan`, `gs_usb`, `virtual`, ... |
| `can_channel` | `can0` | interface name (socketcan) or device index / `bus:address` / serial (gs_usb) |
| `can_bitrate` | `500000` | only used by backends that set the bitrate themselves (gs_usb) |
| `node_id` | `10` | CANopen node ID (1..127) |
| `frame_id` | `mts160_link` | header frame of all messages |
| `timeout_ms` | `100` | no TPDO1 for this long → `/diagnostics` ERROR |
| `publish_raw` | `false` | publish every frame from the node on `~/raw` |
| `invert_position` | `false` | mounting flip: negate positions and the marker lateral (X) axis |
| `invert_angle` | `false` | mounting flip: negate angles |
| `tpdo1_period_ms` / `tpdo2_period_ms` / `tpdo3_period_ms` | `0` | `0` leaves the sensor alone; `>0` writes the CANopen event timer (SDO `0x1800..0x1802:5`) at startup, RAM only |
| `sdo_timeout_ms` | `500` | SDO response timeout |
| `heartbeat_timeout_s` | `3.0` | heartbeat older than this → ERROR |

The driver never changes bitrate, node ID, heartbeat, auto-run, termination,
polarity or thresholds, and never sends `!SAVE`.

**Topics** (all stamped with the CAN receive time)

| topic | type | source |
|---|---|---|
| `~/track` | `naviq_msgs/TrackDetection` | TPDO1 `0x180+id` every 10 ms: left/right position (mm) and angle (deg), strength, flags |
| `~/markers` | `naviq_msgs/Markers` | TPDO2 `0x280+id`: marker X/Y in 0.1 mm, detected flags copied from the latest TPDO1 |
| `~/navicode` | `naviq_msgs/Navicode` | TPDO3 `0x380+id`: code, counter, `is_new` on counter change |
| `~/raw` | `naviq_msgs/RawTpdo` | every frame from the node (optional) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | bus state, heartbeat/NMT state, TPDO rates, data timeout, frame-length errors, SDO errors, receive→publish latency |

**Service**: `~/zero` (SDO write `0x2000`; the firmware stores the zero
reference in flash itself — see `application/src/sensing.c` in the firmware —
so it persists). The sensor's self-test (SDO `0x2001`) is deliberately not
exposed: run it from the Naviq utility or the serial console when needed.

**Extra**: `ros2 run naviq_mts160 mts160_line_follower` — a minimal
proportional `/cmd_vel` follower on `~/track` (example only).

**Seeing what the sensor sends**: with the driver running (`publish_raw:=true`),
`ros2 topic echo /mts160/track` shows the decoded values and
`python3 tools/dump_raw.py` prints every CAN frame candump-style with its
decoded meaning on the same line.

## Frame layouts (from the manual, verified against the firmware)

```
TPDO1 0x18A (5 B):  int8 left_pos | int8 right_pos | int8 left_angle | int8 right_angle | u8 status
      status bits 7..0: merge | fork | intersection | right_marker | left_marker | strength[1] | strength[0] | unused(0)
TPDO2 0x28A (8 B):  int16 LE left_x | left_y | right_x | right_y   (0.1 mm/LSB)
TPDO3 0x38A (3 B):  u16 LE navicode | u8 counter
HB    0x70A (1 B):  NMT state (0x05 operational, 0x7F pre-operational, 0x04 stopped)
SDO   0x60A request / 0x58A response, expedited only
```

Frames are filtered with `(id & 0x7F) == node_id` and dispatched on
`id & 0x780`; wrong payload lengths are counted in diagnostics and dropped.

## Sign conventions

Measured on the bench (`report/summary.md`, `tools/calibration.yaml`):

* **Lateral position** (`Track.position_mm`): mm from the sensor centre, left negative / right positive.
  Measured: moving the sensor toward printer +Y at yaw 0 makes the reported position more *positive*
  (the tape then lies on the sensor's right), slope +1.001 mm/mm, residual 0.29 mm rms, noise 0.03 mm.
  The reported value is linear over about ±58 mm; the left-reported track clamps at −59 mm and the
  right-reported track at +58 mm (the manual quotes ±80 mm).
* **Track angle** (`Track.angle_deg`): the manual's convention (incidence of the tape, ±90°). The bench's
  yaw calibration could not be completed (the sensor slips on the fixture's yaw-motor coupling), so the
  angle sign is reported as the manual states it and is *not* independently verified; `invert_angle` is
  available for a mirrored mounting.
* **Marker X/Y** (`Markers`): X is lateral (same axis and sign as the track position: reported X rises
  with carriage +Y at +1.03 mm/mm), Y is longitudinal. Measured footprint of a 20 mm point marker at 20 mm
  height: about ±20 mm lateral, ±10 mm longitudinal; the longitudinal estimate is compressed (~0.74 mm/mm)
  and saturates near ±8 mm.
* `invert_position` / `invert_angle` flip the driver's output for a mirrored mounting; with both false the
  driver reports exactly what the bench measured.

## Validation status (2026-09-24)

Verified on the bench sensor: the node runs at 99.6 Hz on TPDO1 with TPDO2/TPDO3/heartbeat decoded, `/diagnostics`
OK, receive→publish latency 0.17 ms mean; the recorded-frame fixtures in
`naviq_mts160/test/fixtures/` come from these sessions and `colcon test` passes (64 tests, vcan0 case skipped
on WSL2). The physical characterisation was cut short by the operator: the sensor slips on the fixture's
yaw-motor coupling, so every yaw ≠ 0 test (angle sign and scale, cross-coupling, fork at ±15°, marker at yaw 90)
is **not done**, and the height / repeatability sweeps were abandoned. The navicode section of the bench bed was
never decoded by the sensor (counter unchanged in every pass), so `~/navicode` is verified for framing and
`is_new` only. Details: `report/summary.md`, `tools/bench_notes.md`.

## Development bench (WSL2)

The driver was validated on a Windows PC running the whole stack in **WSL2
Ubuntu 24.04**; the CANable-MKS (candle firmware), the printer's CH340 serial
and the sensor's USB console are attached with `usbipd-win`. There is no
SocketCAN in the stock WSL2 kernel, so the driver uses python-can's `gs_usb`
backend there. See `tools/bench_setup.md` (cold-boot procedure),
`tools/bench_notes.md` (log), `tools/calibration.yaml`, `tools/bed_map.yaml`
and `report/summary.md`.

Fixture tooling (never part of the released driver):

```bash
cd tools
python3 -m naviq_mts160_fixture.hold_e                                # keep the yaw (E) stepper energised, no motion
python3 -m naviq_mts160_fixture.calibrate baseline --assume-homed     # 7.2
python3 -m naviq_mts160_fixture.calibrate rotation --assume-homed     # 7.3
python3 -m naviq_mts160_fixture.survey --assume-homed                 # 7.4
python3 -m naviq_mts160_fixture.characterize --assume-homed all       # 8.x
python3 -m naviq_mts160_fixture.export_fixtures                       # data/ -> test/fixtures
python3 -m naviq_mts160_fixture.report                                # summary.md
```

Operator rules encoded in the tooling: the bed is 220 × 200 mm with switches only at the minima (envelope
X 0–190, Y 0–195); the sensor is rotated only with the carriage at X 100–120 and its tip must stay left of the
right frame (X 205); the E driver stays energised (an unpowered E lets the yaw slip; its noise on the X min switch
is why endstop checking is off for ordinary moves and X/Y are homed by hand).

## Tests

```bash
cd ~/ros2_ws && colcon test --packages-select naviq_mts160 --event-handlers console_direct+
colcon test-result --verbose
```

* `test_decoder.py`, `test_sdo.py`, `test_canbus.py`: pure Python (no ROS,
  no hardware): every field, sign, scale, flag bit, wrong length, node-ID
  filter, negative int16 markers, `is_new`, SDO codec + client, bus wrapper.
* `test_replay.py`: recorded `candump -l` fixtures with ground-truth JSON
  (`test/fixtures/`) through the decoder and through the live node on
  python-can's in-process `virtual` bus; on a host with `vcan0` the node is
  also run as a separate process over SocketCAN with the log replayed. The
  test node runs in its own namespace and `test/conftest.py` picks a private
  `ROS_DOMAIN_ID` unless one is set, so a driver running on the same host
  (the bench) cannot feed the tests.
* `test_launch.py` (`launch_testing`): the node starts from the launch file,
  invalid parameters are rejected, `/diagnostics` goes ERROR→OK when frames
  arrive (cross-process `udp_multicast` backend, needs `python3-msgpack`).
* CI: `.github/workflows/ci.yml` (`action-ros-ci` in the `rostooling/setup-ros-docker` Jazzy image; the
  vcan0 replay case is skipped there and runs on a Linux host with `vcan0`).
