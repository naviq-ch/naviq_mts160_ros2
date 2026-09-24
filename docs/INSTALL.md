# Installing and testing the MTS160 ROS 2 driver

This manual takes a robot from a bare Ubuntu 24.04 / ROS 2 Jazzy install to a
running `mts160` node, and then checks it in three stages: without hardware,
with a recorded sensor log, and with the sensor on the bus.

Contents

1. [What gets installed](#1-what-gets-installed)
2. [Requirements](#2-requirements)
3. [Sensor prerequisites](#3-sensor-prerequisites)
4. [Install](#4-install)
5. [Bring up the CAN interface](#5-bring-up-the-can-interface)
6. [Run the driver](#6-run-the-driver)
7. [Test it](#7-test-it)
8. [Run it as a service](#8-run-it-as-a-service)
9. [Troubleshooting](#9-troubleshooting)

---

## 1. What gets installed

| package | contents |
|---|---|
| `naviq_msgs` | `TrackDetection`, `Markers`, `Navicode`, `RawTpdo` messages and the `Zero` service |
| `naviq_mts160` | the `mts160` node (Python, `rclpy` + `python-can`), `driver.launch.py`, `config/example.yaml`, tests |

The node reads the sensor's CANopen TPDOs directly (no CANopen stack) and
publishes:

| topic | type | rate (sensor default) |
|---|---|---|
| `/mts160/track` | `naviq_msgs/TrackDetection` | 100 Hz (TPDO1, 10 ms) |
| `/mts160/markers` | `naviq_msgs/Markers` | 50 Hz (TPDO2, 20 ms) |
| `/mts160/navicode` | `naviq_msgs/Navicode` | 20 Hz (TPDO3, 50 ms) |
| `/mts160/raw` | `naviq_msgs/RawTpdo` | every frame, only with `publish_raw:=true` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz |

The only write the driver can make to the sensor is the `/mts160/zero`
service (zero-level calibration, stored by the sensor in flash). It never
changes bitrate, node ID, heartbeat, auto-run, termination, polarity or
thresholds, and it does not expose the sensor's self-test.

## 2. Requirements

* Ubuntu 24.04 with ROS 2 Jazzy (`ros-jazzy-ros-base` is enough) and `colcon`.
* A CAN interface the kernel supports as SocketCAN. Tested with a candleLight /
  CANable adapter running the candle firmware (kernel driver `gs_usb`, appears
  as `can0`). Any other SocketCAN adapter (PEAK, Kvaser, an SoC CAN controller)
  works the same way; only the `ip link` step differs.
* Python packages: `python3-can` (apt, pulled in by rosdep). Only if you must
  run without SocketCAN (for example WSL2) you also need `python-can >= 4.4`
  and `gs_usb` from pip, see section 9.
* `can-utils` (`candump`, `canplayer`) for the tests in section 7.

## 3. Sensor prerequisites

Configure the sensor once with the Naviq utility (WebUSB) or its serial
console; the driver does not do this. It expects:

| setting | value |
|---|---|
| interface | CAN (CANopen) |
| bitrate | 500 kbit/s |
| node ID | 10 (any 1..127 works, pass `node_id:=`) |
| auto-run | on (the sensor enters NMT *operational* by itself and starts sending TPDOs) |
| TPDO1 | on, 10 ms (track); TPDO2 20 ms (markers) and TPDO3 50 ms (navicode) as needed |
| heartbeat | on (1000 ms); the driver reports ERROR when it is older than `heartbeat_timeout_s` |
| termination | on if the sensor is at one end of the bus |
| tape polarity | matching the tape you use (north or south up) |

Save the configuration in the sensor (the utility's save / `!SAVE`), then
power-cycle it. Wire CAN_H, CAN_L and GND to the adapter, with a 120 Ω
termination at each end of the bus.

## 4. Install

```bash
# ROS 2 Jazzy already installed and sourced
sudo apt install python3-colcon-common-extensions python3-rosdep can-utils
sudo rosdep init 2>/dev/null; rosdep update

mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/naviq/naviq_mts160_ros2.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y     # rclpy, diagnostic_updater, python3-can, ...
colcon build --symlink-install --packages-select naviq_msgs naviq_mts160
source install/setup.bash
```

Add `source ~/ros2_ws/install/setup.bash` to `~/.bashrc` if you want it in
every terminal. Do not set `ROS_DOMAIN_ID` in one place only (see 9.4).

## 5. Bring up the CAN interface

```bash
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0          # state UP, bitrate 500000
candump can0                        # frames from the sensor, see 7.3
```

To make it persistent with systemd-networkd, create
`/etc/systemd/network/80-can.network`:

```ini
[Match]
Name=can0

[CAN]
BitRate=500K
RestartSec=100ms
```

then `sudo systemctl enable --now systemd-networkd`. (`RestartSec` lets the
interface recover from bus-off by itself.)

## 6. Run the driver

```bash
ros2 launch naviq_mts160 driver.launch.py can_interface_type:=socketcan can_channel:=can0
```

Launch arguments (all optional): `can_interface_type`, `can_channel`,
`can_bitrate` (used only by backends that set it themselves), `node_id`,
`frame_id`, `publish_raw`, `params_file`, `publish_static_tf` (an example
`base_link → mts160_link` transform, set `false` on a real robot and publish
your own), `namespace`.

For anything more, copy `naviq_mts160/config/example.yaml`, edit it and pass
`params_file:=/path/to/mts160.yaml`. The node's parameters are listed in the
README (`timeout_ms`, `heartbeat_timeout_s`, `invert_position`,
`invert_angle`, `tpdo*_period_ms`, `sdo_timeout_ms`).

The `mts160` node exits with a non-zero code on an invalid parameter and
keeps running through CAN errors: the bus wrapper reconnects by itself and
`/diagnostics` shows what is wrong.

## 7. Test it

### 7.1 Without hardware: the test suite

```bash
cd ~/ros2_ws
colcon test --packages-select naviq_mts160 --event-handlers console_direct+
colcon test-result --verbose
```

Expected: all tests pass; one is skipped when `vcan0` does not exist. The
suite covers

* `test_decoder.py`, `test_sdo.py`, `test_canbus.py`: every frame field, sign,
  scale and flag bit, wrong lengths, node-ID filtering, negative marker
  values, `is_new`, the SDO codec and client, the bus wrapper's reconnect;
* `test_replay.py`: the recorded sensor logs in `naviq_mts160/test/fixtures/`
  through the decoder and through the live node on python-can's in-process
  `virtual` bus, compared with the labels in the matching `.json` files;
* `test_launch.py`: the node starts from the launch file, rejects an invalid
  parameter, and `/diagnostics` goes ERROR → OK when frames arrive.

The tests run in their own ROS domain (`test/conftest.py`), so a driver
running on the same machine does not disturb them.

### 7.2 With a recorded sensor log (no sensor needed)

The fixtures are `candump -l` files; play one into a virtual CAN interface
and watch the node decode it:

```bash
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
ros2 launch naviq_mts160 driver.launch.py can_interface_type:=socketcan can_channel:=vcan0 publish_raw:=true
```

in a second terminal:

```bash
cd ~/ros2_ws/src/naviq_mts160_ros2
ros2 topic echo /mts160/track &
canplayer -I naviq_mts160/test/fixtures/lateral.log vcan0=can0     # vcan0 receives what was logged on can0
```

`lateral.log` is the sensor being carried sideways across a tape: the
position ramps from about −58 mm to +43 mm while `strength` stays 3.
`fork_yaw+0.log` / `merge_yaw+0.log` show the `fork` flag, and
`marker_alongside_yaw0.log` the marker fields.

### 7.3 With the sensor

Step 1: is the sensor talking? Before starting ROS:

```bash
candump can0
```

You should see, with node ID 10, `18A` every 10 ms (5 bytes), `28A` every
20 ms (8 bytes), `38A` every 50 ms (3 bytes) and `70A` once a second
(1 byte, `05` = operational). Nothing at all: see 9.1.

Step 2: start the driver and check the rates:

```bash
ros2 launch naviq_mts160 driver.launch.py can_interface_type:=socketcan can_channel:=can0 publish_raw:=true
```

```bash
ros2 topic hz /mts160/track          # ~100 Hz
ros2 topic echo /mts160/track
```

Step 3: read what the sensor sends, one line per frame with its meaning:

```bash
python3 ~/ros2_ws/src/naviq_mts160_ros2/tools/dump_raw.py
```

```
(1790249413.620815) 18A#D6D7FC0306       TPDO1 track: L -42 mm -4 deg | R -41 mm +3 deg | strength 3
(1790249413.640709) 28A#0000000000000000 TPDO2 markers: L (+0.0, +0.0) mm  R (+0.0, +0.0) mm
(1790249413.660974) 38A#000005           TPDO3 navicode: code 0 counter 5
```

Step 4: plausibility checks with the sensor in your hand or on the robot,
about 20 mm above the floor:

| do this | expect on `/mts160/track` |
|---|---|
| no tape under the sensor | `strength: 0`, `tape_detected: false`, positions 0 |
| tape under the centre | `strength` 2–3, `tape_detected: true`, `position_mm` near 0 |
| slide the sensor so the tape is under its right half | `position_mm` positive and growing, up to about +58 |
| tape under the left half | `position_mm` negative, down to about −59 |
| turn the sensor on the tape | `angle_deg` follows the incidence angle |
| lift the sensor | `strength` drops to 1, then 0 |
| a marker beside the tape | `left_marker` / `right_marker` and the marker X/Y on `/mts160/markers` |

Sign conventions and what was measured are in the README; use
`invert_position` / `invert_angle` if your mounting is mirrored.

Step 5: diagnostics.

```bash
ros2 topic echo /diagnostics
```

All six entries (`CAN bus`, `Heartbeat`, `Sensor data`, `Frame errors`,
`SDO`, `Latency`) should be level 0 (OK) with the TPDO rates listed under
`Sensor data`.

Step 6 (only when needed): zero-level calibration. With the sensor mounted,
stationary, and away from any tape, marker or steel:

```bash
ros2 service call /mts160/zero naviq_msgs/srv/Zero
```

The sensor stores the result itself. Do this when the sensor reports a
non-zero `strength` on bare floor after mounting, not routinely.

## 8. Run it as a service

`/etc/systemd/system/mts160-driver.service`:

```ini
[Unit]
Description=Naviq MTS160 ROS 2 driver
After=network-online.target sys-subsystem-net-devices-can0.device
Wants=sys-subsystem-net-devices-can0.device

[Service]
User=robot
Environment=HOME=/home/robot
ExecStart=/bin/bash -lc "source /opt/ros/jazzy/setup.bash && source /home/robot/ros2_ws/install/setup.bash && exec ros2 launch naviq_mts160 driver.launch.py can_interface_type:=socketcan can_channel:=can0 publish_static_tf:=false"
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload && sudo systemctl enable --now mts160-driver
journalctl -u mts160-driver -f
```

If your robot uses a `ROS_DOMAIN_ID`, put `Environment=ROS_DOMAIN_ID=<n>` in
the unit as well as in the shells that will talk to it.

## 9. Troubleshooting

**9.1 `candump` shows nothing.** In order: is `can0` UP at 500000
(`ip -details link show can0`)? Is the sensor powered and in CAN mode with
auto-run on (an LED pattern or the console's status tells you)? Bus wiring
and termination: a missing or doubled 120 Ω often gives `bus-off` in
`ip -details link` (`ip link set can0 down && up` clears it). Wrong node ID:
`candump` shows frames but with another low byte (e.g. `18B`); pass
`node_id:=11`.

**9.2 `/diagnostics` says ERROR.** `CAN bus: not connected` → the interface
does not exist or the backend cannot open it (device name, permissions).
`Sensor data: no TPDO1 for ...` → the bus is fine but the sensor is silent
(9.1) or TPDO1 is disabled. `Heartbeat: stale` → auto-run or the heartbeat
producer is off in the sensor. `Frame errors` counts frames of the wrong
length: a different firmware or a node on the same ID.

**9.3 Frames arrive but values are all zero.** The sensor sees no tape
(`strength 0`); check the tape polarity setting against your tape and the
mounting height (spec: about 20 mm).

**9.4 `ros2 topic echo` says the topic is not published.** The driver and
your terminal are on different ROS domains: compare `echo $ROS_DOMAIN_ID` in
the terminal with the service's environment. Both unset (domain 0) is the
simplest.

**9.5 No SocketCAN (WSL2, macOS, a locked-down kernel).** Use python-can's
user-space `gs_usb` backend (candleLight / CANable with candle firmware):

```bash
python3 -m pip install --user --break-system-packages "python-can>=4.4" gs_usb
# udev rule so the device is accessible without root
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", MODE="0666", TAG+="uaccess"' | sudo tee /etc/udev/rules.d/99-gs-usb.rules
sudo udevadm control --reload-rules
ros2 launch naviq_mts160 driver.launch.py can_interface_type:=gs_usb can_channel:=0
```

`can_channel` is the device index, `bus:address` or the adapter's serial
number. On WSL2 the adapter is attached with `usbipd-win`; the full bench
procedure is in `tools/bench_setup.md`. The driver contains a workaround for
the `gs_usb` package leaving the device unconfigured after its USB reset;
if the bus stays silent, unplug and re-plug the adapter once.

**9.6 Rebuilding after a message change.** `naviq_msgs` is generated code:
after pulling a change to `msg/` or `srv/`, rebuild both packages
(`colcon build --packages-select naviq_msgs naviq_mts160`) and restart every
node that uses them.
