# MTS160 ROS 2 driver: user manual

Everything needed to use the `mts160` node once it is installed
([INSTALL.md](INSTALL.md) covers installation and testing).

Contents

1. [What the node does](#1-what-the-node-does)
2. [Starting it](#2-starting-it)
3. [Parameters](#3-parameters)
4. [Topics and data formats](#4-topics-and-data-formats)
5. [Service](#5-service)
6. [Diagnostics](#6-diagnostics)
7. [Coordinate frame, signs and mounting](#7-coordinate-frame-signs-and-mounting)
8. [Timing, stamps and QoS](#8-timing-stamps-and-qos)
9. [Behaviour on faults](#9-behaviour-on-faults)
10. [Using the data](#10-using-the-data)
11. [Command cheat sheet](#11-command-cheat-sheet)
12. [Appendix: CAN frames](#12-appendix-can-frames)

---

## 1. What the node does

The Naviq MTS160 is a magnetic guide sensor with a 160 mm sensing width. On
its CAN interface it is a CANopen node that broadcasts three process-data
objects (TPDOs) and a heartbeat; the driver listens to them and republishes
their content as ROS messages:

| sensor frame | content | ROS topic |
|---|---|---|
| TPDO1 "Sense" | tape position and angle for a left and a right track, signal strength, marker / intersection / fork / merge flags | `~/track` |
| TPDO2 "Marker" | position of a marker on the left and on the right, 0.1 mm | `~/markers` |
| TPDO3 "Navicode" | last decoded navicode value and a capture counter | `~/navicode` |
| heartbeat | NMT state | `/diagnostics` |

The node is a pure listener with one exception: the `~/zero` service
starts the sensor's zero-level calibration over an SDO write. It never
changes the sensor's bitrate, node ID, heartbeat, auto-run, termination,
polarity or thresholds, and it does not expose the sensor's self-test.
Sensor provisioning is done with the Naviq utility or the serial console.

Node name: `mts160`. All topics below are under the node's namespace and
name, so with no namespace they are `/mts160/track`, `/mts160/markers`,
`/mts160/navicode`, `/mts160/raw` and `/mts160/zero`; with
`namespace:=/robot1` they become `/robot1/mts160/track` and so on.
`/diagnostics` is global.

## 2. Starting it

```bash
ros2 launch naviq_mts160 driver.launch.py can_interface_type:=socketcan can_channel:=can0
```

Launch arguments:

| argument | default | meaning |
|---|---|---|
| `can_interface_type` | `socketcan` | python-can backend (`socketcan`, `gs_usb`, `virtual`, ...) |
| `can_channel` | `can0` | interface name; for `gs_usb` a device index, `bus:address` or serial |
| `can_bitrate` | `500000` | only used by backends that set the bitrate themselves (`gs_usb`) |
| `node_id` | `10` | the sensor's CANopen node ID |
| `frame_id` | `mts160_link` | `header.frame_id` of every message |
| `publish_raw` | `false` | also publish every raw frame on `~/raw` |
| `params_file` | (none) | YAML applied on top of the arguments above |
| `publish_static_tf` | `true` | start an example `base_link → mts160_link` static transform |
| `namespace` | (none) | namespace for the node |

Or run the executable directly:

```bash
ros2 run naviq_mts160 mts160 --ros-args -p can_interface_type:=socketcan -p can_channel:=can0
ros2 run naviq_mts160 mts160 --ros-args --params-file my_mts160.yaml
```

`naviq_mts160/config/example.yaml` is a complete, commented parameter file.

## 3. Parameters

| name | type | default | range | changeable at runtime | meaning |
|---|---|---|---|---|---|
| `can_interface_type` | string | `socketcan` | | no | python-can backend |
| `can_channel` | string | `can0` | | no | interface / device |
| `can_bitrate` | int | `500000` | 10000..1000000 | no | bit/s, used only by backends that set it (`gs_usb`) |
| `node_id` | int | `10` | 1..127 | no | frames from other node IDs are ignored and counted |
| `frame_id` | string | `mts160_link` | | yes | `header.frame_id` of all messages |
| `timeout_ms` | int | `100` | 1..60000 | yes | no TPDO1 for this long → `Sensor data` ERROR |
| `heartbeat_timeout_s` | double | `3.0` | | yes | heartbeat older than this → `Heartbeat` ERROR |
| `publish_raw` | bool | `false` | | no | publish `~/raw` |
| `invert_position` | bool | `false` | | yes | negate both track positions and the marker lateral (X) axis |
| `invert_angle` | bool | `false` | | yes | negate both track angles |
| `tpdo1_period_ms` | int | `0` | 0..65535 | no | `0` leaves the sensor alone; `>0` writes the TPDO1 event timer (SDO `0x1800:5`) once at startup, RAM only |
| `tpdo2_period_ms` | int | `0` | 0..65535 | no | same for TPDO2 (`0x1801:5`); `0` written to the sensor would disable that TPDO, so the driver only writes values `> 0` |
| `tpdo3_period_ms` | int | `0` | 0..65535 | no | same for TPDO3 (`0x1802:5`) |
| `sdo_timeout_ms` | int | `500` | 10..10000 | no | wait for an SDO response |

"Changeable at runtime" parameters take effect immediately with
`ros2 param set /mts160 invert_angle true`; the others are read once at
startup (the descriptors mark them read-only). An out-of-range value at
startup makes the node exit with a non-zero code and a clear log line.

The TPDO period parameters are the only way the driver changes the sensor's
output rate. They live in the sensor's RAM: after a sensor power cycle the
sensor is back at its saved configuration until the driver restarts.

## 4. Topics and data formats

All messages carry a `std_msgs/Header` whose `stamp` is the time the CAN
frame was received on the host (see section 8) and whose `frame_id` is the
`frame_id` parameter.

### 4.1 `~/track` — `naviq_msgs/TrackDetection`

One message per TPDO1 frame, 100 Hz with the sensor's default 10 ms period.
QoS: sensor data (best effort, keep last 5).

```
std_msgs/Header header
naviq_msgs/Track left
naviq_msgs/Track right
uint8 strength            # 0 none, 1 weak, 2 medium, 3 strong
bool  tape_detected       # strength != 0
bool  single_track        # left == right (same position and same angle)
bool  left_marker
bool  right_marker
bool  intersection
bool  fork
bool  merge
```

with

```
# naviq_msgs/Track
float32 position_mm       # lateral offset of the tape centreline from the sensor centre
float32 angle_deg         # incidence angle of the tape
```

| field | unit / resolution | range | meaning |
|---|---|---|---|
| `left.position_mm`, `right.position_mm` | mm, 1 mm steps | ±80 nominal; linear over about ±58 mm on the bench | where the tape centreline crosses the sensor's lateral axis: 0 = under the centre, negative = on the sensor's left, positive = on its right. Reads 0 when no tape is detected. |
| `left.angle_deg`, `right.angle_deg` | degrees, 1° steps | ±90 | angle between the tape and the sensor's longitudinal axis, 0 = tape running straight through |
| `strength` | 0..3 | | field strength class; constants `STRENGTH_NONE/WEAK/MEDIUM/STRONG` are defined in the message. Drops to 1 and then 0 as the sensor is raised or leaves the tape. |
| `tape_detected` | bool | | convenience: `strength != 0` |
| `single_track` | bool | | the two reported tracks are identical, i.e. one tape under the sensor |
| `left_marker`, `right_marker` | bool | | a marker (point source) is seen on that side of the tape |
| `intersection` | bool | | a broad crossing is under the sensor: the sensor forces both angles to 0 and clears the marker flags while it is set |
| `fork` | bool | | advisory: the tape splits ahead (left/right angle difference about +10..+45°) |
| `merge` | bool | | advisory: two tapes join (angle difference about −10..−45°) |

The sensor always reports two tracks. On a single tape they are identical;
on a fork they separate, and each follows its own branch until the branches
are more than a sensing width apart. Which one is "left" is the sensor's
choice; follow one consistently (see section 10).

Two observations from the bench: the left-reported track clamps at −59 mm
and the right-reported one at +58 mm rather than at the ±80 of the manual,
and the `fork` flag was asserted only for part of a traverse through a
fork while `merge` was never asserted; treat both flags as hints, not as
geometry.

### 4.2 `~/markers` — `naviq_msgs/Markers`

One message per TPDO2 frame, 50 Hz by default. QoS: sensor data.

```
std_msgs/Header header
bool    left_detected     # copied from the latest ~/track left_marker
bool    right_detected    # copied from the latest ~/track right_marker
float32 left_x_mm         # lateral position of the left marker, 0.1 mm steps
float32 left_y_mm         # longitudinal position of the left marker
float32 right_x_mm
float32 right_y_mm
```

X is the lateral axis with the same sign as `position_mm` (positive toward
the sensor's right); Y is the longitudinal axis (positive toward the
sensor's front as the manual states; the sign of Y was not verified on the
bench). Positions read 0.0 when the corresponding marker is not detected.
A standalone point marker with no tape in view is reported as both left and
right marker at the same lateral position.

Measured with a 20 mm point marker at 20 mm sensor height: the marker is
seen over about ±20 mm laterally and ±10 mm longitudinally around its
centre, with the longitudinal estimate compressed (about 0.74 mm reported
per mm moved) and saturating near ±8 mm.

### 4.3 `~/navicode` — `naviq_msgs/Navicode`

One message per TPDO3 frame, 20 Hz by default. QoS: reliable, keep last 10.

```
std_msgs/Header header
uint16 code               # last decoded navicode value
uint8  counter            # incremented by the sensor each time a code is captured, wraps at 255
bool   is_new             # true on the first message whose counter differs from the previous one
```

The sensor decodes a navicode while a tape is detected: decoding starts when
a marker appears on either side of the tape and completes when no marker is
seen any more, after which `code` holds the value and `counter` increments.
`code` and `counter` keep their last value between captures. `is_new` is
never true on the driver's first message (it cannot know whether the stored
code pre-dates its start), so a code that was captured before the driver
started is available in `code` but is not flagged. On the bench the sensor
did not decode the test bed's navicode section, so this topic is verified
for framing and for `is_new`, not against a known code.

### 4.4 `~/raw` — `naviq_msgs/RawTpdo` (optional)

Only with `publish_raw:=true`. One message per CAN frame whose node ID
matches, including TPDO1..3, the heartbeat and SDO responses. QoS: sensor
data.

```
std_msgs/Header header    # stamp = CAN receive time
uint16  can_id            # 11-bit COB-ID, e.g. 0x18A (394) for TPDO1 of node 10
uint8[] data              # payload as received
```

Use it to record what the sensor sent (`tools/dump_raw.py` prints it in
`candump` style, `ros2 bag record /mts160/raw` keeps it) and to see frames
the decoder does not publish elsewhere.

### 4.5 `/diagnostics` — `diagnostic_msgs/DiagnosticArray`

Once a second, six statuses named `mts160: <entry>`, detailed in section 6.

## 5. Service

### `~/zero` — `naviq_msgs/srv/Zero`

Request: empty. Response: `bool ok` (the sensor acknowledged the SDO write).

Starts the sensor's zero-level calibration (SDO `0x2000`). Conditions: the
sensor mounted in its working position, stationary, over bare floor, away
from any tape, marker, steel or magnet. The sensor measures its offset and
stores it in its own non-volatile memory, so the result persists across
power cycles and does not need repeating at every start. Use it after
mounting or moving the sensor, or when `strength` is not 0 over bare floor.
A zero taken near a tape corrupts every reading until it is redone.

```bash
ros2 service call /mts160/zero naviq_msgs/srv/Zero
```

## 6. Diagnostics

`hardware_id` is `mts160 node <id> (<backend>/<channel>)`. Entries:

| entry | OK | WARN | ERROR | key/values |
|---|---|---|---|---|
| `CAN bus` | interface open | | not open / lost; message holds the last error | `interface_type`, `channel`, `bitrate`, `rx_frames`, `tx_frames`, `error_frames`, `reconnects`, `open_failures`, `last_error` |
| `Heartbeat` | NMT operational | no heartbeat seen yet; or NMT state not operational (the sensor sends TPDOs only when operational) | heartbeat older than `heartbeat_timeout_s` | `nmt_state`, `age_s`, `count` |
| `Sensor data` | TPDO1 rate | | no TPDO1 yet, or none for `timeout_ms` | `tpdo1..3_rate_hz`, `tpdo1..3_count`, `tpdo1_interval_mean_ms`, `tpdo1_interval_max_ms`, latest `strength`, `left/right_position_mm`, `left/right_angle_deg`, `flags` |
| `Frame errors` | none | wrong-length frames were dropped since the last report | | `bad_length_total`, `unknown_function_code_total`, `other_node_frames_total` |
| `SDO` | no SDO errors | an SDO transaction failed (timeout or abort) | | `sdo_responses_total`, `sdo_errors`, `last_sdo_error` |
| `Latency` | receive → publish mean | | | `samples`, `mean_us`, `max_us`, `stdev_us` |

A robot health monitor can key on `Sensor data` alone: it is ERROR whenever
the tape data is not fresh, whatever the cause.

## 7. Coordinate frame, signs and mounting

The sensor frame (`frame_id`, default `mts160_link`): origin at the centre of
the sensing width, lateral axis across the sensor with positive toward the
sensor's right, longitudinal axis along the direction of travel. The manual
defines positions as "left negative, right positive"; the bench confirmed
this for the position channel (moving the sensor so that the tape lies
under its right half makes `position_mm` positive, slope 1.0 mm/mm,
0.3 mm rms residual at 20 mm height). The sign of `angle_deg` and of the
marker Y axis is reported as the manual defines it and was not
independently verified, because the bench fixture could not rotate the
sensor reliably.

Mounting height (sensor underside to tape) affects the linear range:

| height | position slope | residual rms | usable span seen |
|---|---|---|---|
| 15 mm | 0.99 | 0.75 mm | tape lost beyond about −45 mm on the far side |
| 20 mm (nominal) | 0.99 | 0.38 mm | full |
| 30 mm | 0.99 | 0.58 mm | full |
| 40 mm | 0.64 | 3.7 mm | the tape is lost over most of the width |

Keep the sensor near 20 mm.

If the sensor is mounted rotated 180° about its vertical axis (cable
pointing the other way), or you want positions and angles in your own
convention, set `invert_position` and/or `invert_angle`; they negate the
respective fields of both tracks (and the marker X axis for
`invert_position`) before publishing. Everything in this manual describes
the values with both false.

The driver publishes no TF. The launch file's optional static transform
(`publish_static_tf`, default on) places `mts160_link` 0.30 m ahead of and
0.02 m above `base_link` with no rotation, as an example only: on a real
robot disable it and publish the true mounting transform yourself.

## 8. Timing, stamps and QoS

* The sensor measures every 5 ms and sends TPDO1 every 10 ms, so consecutive
  `~/track` messages are distinct samples.
* `header.stamp` is the host's wall-clock time when python-can handed the
  frame to the driver. It is not sensor time and it ignores `use_sim_time`.
  Measured on the bench (WSL2 with a USB adapter, a slow path): mean
  interval 10.1 ms, worst 22 ms, receive-to-publish 0.17 ms mean.
* `~/track`, `~/markers` and `~/raw` use the sensor-data QoS profile
  (best effort, volatile, keep last 5). **A subscriber with reliable QoS
  will not receive them**; subscribe with `qos_profile_sensor_data` or an
  explicit best-effort profile.
* `~/navicode` is reliable (keep last 10) so a captured code is not lost to
  a dropped sample.

## 9. Behaviour on faults

| situation | what happens |
|---|---|
| CAN interface missing or cannot be opened | node keeps running, `CAN bus` ERROR, retries every second |
| bus lost while running (adapter unplugged, bus-off) | reader reconnects every second, `CAN bus` ERROR meanwhile, `reconnects` counts |
| sensor silent (unpowered, not operational, TPDO1 off) | `Sensor data` ERROR after `timeout_ms`; `Heartbeat` ERROR after `heartbeat_timeout_s`; last values are **not** republished |
| sensor in pre-operational or stopped | `Heartbeat` WARN with the state name; no TPDOs arrive |
| frame with a wrong payload length | dropped and counted (`Frame errors` WARN) |
| frames from another node ID | ignored and counted (`other_node_frames_total`) |
| SDO timeout or abort (zero service, TPDO period at startup) | logged, `SDO` WARN with the text, service returns `ok: false` |
| invalid parameter at startup | node exits with a non-zero code |

The driver never publishes stale data: a topic simply stops when its frames
stop, which is why the diagnostics, not the topic rate, should drive a
safety reaction.

## 10. Using the data

Minimal subscriber:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from naviq_msgs.msg import TrackDetection

class Follower(Node):
    def __init__(self):
        super().__init__("tape_follower")
        self.create_subscription(TrackDetection, "/mts160/track", self.on_track, qos_profile_sensor_data)

    def on_track(self, m: TrackDetection):
        if not m.tape_detected:
            return                      # stop / search
        offset_m = m.left.position_mm / 1000.0
        angle_rad = m.left.angle_deg * 3.14159 / 180.0
        # steer to bring offset_m and angle_rad to zero

rclpy.init(); rclpy.spin(Follower())
```

Guidelines:

* Follow one track (`left` or `right`) consistently. At a fork the two
  separate; choose the branch by comparing them (the one whose position is
  more negative goes left) and keep following that one. On a single tape
  they are equal, so the choice does not matter.
* Treat `intersection` as "hold your heading": angles are forced to 0 while
  it is set.
* Watch `strength`: 1 means the tape is at the edge of detection (height or
  lateral limit); plan to stop before it reaches 0.
* Stop when `~/track` messages stop arriving (compare `header.stamp` with
  now, or watch `/diagnostics`), not only when `tape_detected` is false.
* Markers and navicodes are events: use `left_marker` / `right_marker` for
  stop points, `is_new` to consume a navicode exactly once.

A ready-made example is included:

```bash
ros2 run naviq_mts160 mts160_line_follower
```

It steers `/cmd_vel` with `omega = -(k_pos * position_m + k_angle * angle_rad) * steer_sign`
and stops when the tape is lost or the data is stale. Parameters:
`track_topic` (`/mts160/track`), `cmd_vel_topic` (`/cmd_vel`), `side`
(`left`), `linear_speed` (0.15 m/s), `k_pos` (4.0 rad/s per m), `k_angle`
(1.5 rad/s per rad), `steer_sign` (1.0; flip if the base steers away from
the tape), `max_angular` (1.0 rad/s), `stale_s` (0.2), `rate_hz` (50). It is
a starting point, not a navigation stack.

Recording and replay: `ros2 bag record /mts160/track /mts160/markers /mts160/navicode /diagnostics`
for ROS-level logs; `publish_raw:=true` plus `tools/dump_raw.py` or
`ros2 bag record /mts160/raw` keeps the actual CAN frames, which can be
replayed into the driver through `vcan0` with `canplayer` (INSTALL.md 7.2).

## 11. Command cheat sheet

```bash
ros2 launch naviq_mts160 driver.launch.py can_interface_type:=socketcan can_channel:=can0
ros2 topic echo /mts160/track            # decoded values, 100 Hz
ros2 topic hz /mts160/track
ros2 topic echo /mts160/markers
ros2 topic echo /mts160/navicode
ros2 topic echo /diagnostics
ros2 param list /mts160
ros2 param set /mts160 invert_angle true
ros2 service call /mts160/zero naviq_msgs/srv/Zero
python3 tools/dump_raw.py                # every frame with its meaning (needs publish_raw:=true)
ros2 interface show naviq_msgs/msg/TrackDetection
```

## 12. Appendix: CAN frames

Node ID *n* (default 10, so the IDs below are `0x18A`, `0x28A`, `0x38A`,
`0x70A`, `0x60A`, `0x58A`). All multi-byte values little-endian.

```
TPDO1 0x180+n (5 B): int8 left_pos_mm | int8 right_pos_mm | int8 left_angle_deg | int8 right_angle_deg | u8 status
      status bit 7..0: merge | fork | intersection | right_marker | left_marker | strength[1] | strength[0] | unused(0)
TPDO2 0x280+n (8 B): int16 left_x | int16 left_y | int16 right_x | int16 right_y      (0.1 mm/LSB)
TPDO3 0x380+n (3 B): u16 navicode | u8 counter
HB    0x700+n (1 B): NMT state  0x00 boot-up, 0x04 stopped, 0x05 operational, 0x7F pre-operational
SDO   0x600+n request / 0x580+n response, expedited (8 B); used by the driver:
      0x2000:0 u8 = 1   start zero-level calibration          (the ~/zero service)
      0x1800..0x1802:5 u16   TPDO1..3 event timer, ms          (tpdoN_period_ms > 0, at startup)
```

Frames are accepted when `(id & 0x7F) == node_id` and dispatched on
`id & 0x780`; a payload of the wrong length is counted and dropped.
