# Bruno 2.0

Ground differential-drive robot: ESP32 base controller + L298N drivetrain, LD14P LiDAR, and a 3-sonar front array, bridged to ROS 2 Jazzy on an onboard x86 host.

## 1. System Architecture

- **Host:** Dell Latitude 5320 (x86_64), Ubuntu Server 24.04.5 LTS (Noble Numbat), ROS 2 **Jazzy** (`ros-jazzy-ros-base`).
- **Microcontroller & sensors:**
  - ESP32 base controller (`firmware/bruno_base/bruno_base.ino`) — drives the motors and reads the sonar array.
  - LDROBOT **LD14P** LiDAR — 360° scan, ~6.1 Hz native rotation rate on this unit.
  - 3x **HC-SR04** ultrasonic sensors mounted at **-30° (left)**, **0° (center)**, **+30° (right)**, wired directly to the ESP32.
- **Drivetrain:** Differential drive via an **L298N** H-bridge, open-loop (no wheel encoders). Hand-tuned calibration baked into firmware:
  - `LEFT_TRIM = 0.65` — left motor PWM is reduced 35% relative to the right to correct a straight-line drift.
  - Baseline test PWM of **160** (of 255) used for smoke-test driving.
  - Right motor wiring (`IN3`/`IN4`) is swapped in firmware relative to the left motor, compensating for the right motor being mounted mirror-image to the left.

## 2. Hardware & Serial Port Mapping

Two identical Silicon Labs CP2102 USB-UART adapters are attached (ESP32 and LD14P). Both report the **same USB VID:PID (`10c4:ea60`) and the same factory serial string (`0001`)**, so neither device can be distinguished by USB serial number or VID:PID — any udev rule has to key off something else.

`/etc/udev/rules.d/99-bruno-ports.rules` matches on **`ENV{ID_PATH}`**, which encodes the physical USB port topology (which physical port on the host each adapter is plugged into) rather than any identity reported by the device itself:

```
SUBSYSTEM=="tty", ENV{ID_PATH}=="pci-0000:00:14.0-usb-0:2:1.0", SYMLINK+="esp32", MODE="0666"
SUBSYSTEM=="tty", ENV{ID_PATH}=="pci-0000:00:14.0-usb-0:3:1.0", SYMLINK+="lidar", MODE="0666"
```

This produces two stable symlinks regardless of enumeration order (e.g. `ttyUSB0` vs `ttyUSB1` swapping across a reboot):

| Symlink | Target | Baud rate | Device |
|---|---|---|---|
| `/dev/esp32` | `ttyUSB*` | 115200 | ESP32 base controller |
| `/dev/lidar` | `ttyUSB*` | 230400 | LD14P LiDAR |

If these symlinks ever go missing, check `lsusb` for two `10c4:ea60` entries before assuming a udev regression — a loose cable will make the adapters disappear from the USB bus entirely, which looks similar but isn't a udev problem.

## 3. ROS 2 Workspace & Packages

Workspace: `ros2_ws/` (colcon). Two packages:

### `bruno_base` (`ros2_ws/src/bruno_base`, ament_python)

Bridges the ESP32's serial protocol to ROS 2:

- Subscribes to `/cmd_vel` (`geometry_msgs/msg/Twist`) and writes `M <left_pwm> <right_pwm>\n` to `/dev/esp32`. `linear.x`/`angular.z` are mixed into per-side PWM and scaled by a `max_pwm` parameter (default 160); since no wheel radius/track width exists yet, these are treated as normalized ~[-1, 1] inputs rather than calibrated m/s.
- **500ms safety watchdog:** if no `/cmd_vel` arrives within `cmd_vel_timeout_sec` (default 0.5s), sends `M 0 0\n` once to halt the robot, and re-arms on the next command.
- Reads the ESP32's `U <left_cm> <mid_cm> <right_cm>` telemetry stream and publishes three `sensor_msgs/msg/Range` topics: `/sonar/left`, `/sonar/center`, `/sonar/right`. A reading of `-1` (no echo / out of range) is reported as `range = max_range` (4.0 m).

### `ldlidar_stl_ros2` (`ros2_ws/src/ldlidar_stl_ros2`, vendored from [`ldrobotSensorTeam/ldlidar_stl_ros2`](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2))

- This driver version has no dedicated LD14P product type — it's run under the **`LDLiDAR_LD19`** profile, since the LD14P shares the LD19's packet protocol and baud rate. Confirmed empirically: real `/scan` output decodes correctly under this profile.
- Carries one local patch on top of upstream: `ldlidar_driver/src/logger/log_module.cpp` was missing `#include <pthread.h>`, which GCC 13 (Ubuntu 24.04's default) rejects outright where older compilers tolerated it implicitly.
- Launch file `launch/ld14p.launch.py` configures `port_name: /dev/lidar`, `port_baudrate: 230400`, `frame_id: laser_frame`, topic `scan`, and bundles a static transform `base_link` → `laser_frame` at translation `[0.08, 0.0, 0.12]` (no rotation).
- Verified live: full 360° scans (`angle_min: 0`, `angle_max: 2π`), stable at **~6.1 Hz** (lower than the LD19's nominal ~10 Hz, but confirmed expected for this specific LD14P hardware revision).

## 4. Quick Start

```bash
cd ~/bruno_robot/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# LiDAR driver + static TF (base_link -> laser_frame)
ros2 launch ldlidar_stl_ros2 ld14p.launch.py

# ESP32 base controller bridge (in another terminal, same sourcing)
ros2 run bruno_base bruno_base_node
```

Dry-run a drive command and check sonar telemetry (in another sourced terminal):

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
ros2 topic echo /sonar/left
ros2 topic hz /scan
```

### Power monitoring: the `battery` CLI

`~/.local/bin/battery` (on `PATH`) reports live power draw with no arguments:

```bash
battery
```

It combines three sources:
- **Battery telemetry** from `/sys/class/power_supply/BAT*` (capacity %, charge/discharge state, net wattage).
- **CPU package draw** sampled from Intel RAPL (`/sys/class/powercap/intel-rapl*`).
- **USB bus power breakdown**, with hardcoded, calibrated estimates for Bruno's two CP2102 adapters specifically (LD14P LiDAR ~1.5 W while spinning + lasing, ESP32 bridge ~0.35 W logic-only) and a rough `bMaxPower`-based estimate for any other attached USB device.

Example output while charging:
```
Battery: 77% (Charging)
Battery Charge Intake:   20.66 W
 ├─ CPU Package Draw:    6.91 W
 ├─ USB Bus Rail (Est):  2.08 W
 │   └─ Unknown USB Device: ~0.23 W
 │   └─ ESP32 Controller Bridge: ~0.35 W
 │   └─ LD14P LiDAR (Laser + Motor): ~1.50 W
 └─ Est. Adapter Draw:   32.14 W (Battery + Board)
```
