# Bruno Robot — Session Handover

**Purpose:** Full continuity brief for an external LLM architect picking up Bruno's development. Every fact below was directly verified against the live machine during this session (file reads, `udevadm`, `lsusb`, `dpkg -l`, `colcon build`, etc.) unless explicitly marked "unverified" or "assumption."

---

## 1. System & Environment Overview

- **Host:** x86_64, Ubuntu Server **24.04.5 LTS (Noble Numbat)**.
- **ROS 2 distro:** **Jazzy**, installed at `/opt/ros/jazzy` as package `ros-jazzy-ros-base` (base install — not `desktop`, so no Rviz2/demos bundled by default; `tf2_ros` and `urdf` are included since `ros-base` extends `ros_core`).
- **User account:** `bruno`, groups: `bruno adm dialout cdrom sudo dip plugdev lxd ollama`. Already in `dialout`, so no udev `MODE`/group fix is needed for raw `/dev/ttyUSB*` access.
- **sudo:** requires an interactive password for this user — there is **no NOPASSWD rule** configured (one was proposed earlier in the session but the user chose to run privileged installs manually instead). Any future automated step needing `sudo` will need to be handed to the user to run.
- **Toolchain installed via apt (manually, by user):**
  - `python3-colcon-common-extensions` 0.3.0-100
  - `python3-rosdep` / `python3-rosdep-modules` 0.27.0-1 (rosdep already `init`'d and `update`'d — sources cache present under `~/.ros/rosdep`)
  - `ros-jazzy-ros-base` 0.11.0-1noble.20261006.035407
- **Toolchain:** `build-essential` (specifically `g++`) was installed by the user mid-session — confirmed present at `/usr/bin/g++` (GCC 13.3.0). This unblocked the colcon build (see §3/§4 — build now succeeds).
- **arduino-cli:** v1.5.1, installed at `~/.local/bin/arduino-cli`.

### Serial hardware

Two identical-model USB-serial adapters are attached:

| Device node | Chipset | USB ID | Kernel devpath | Reported serial no. |
|---|---|---|---|---|
| `/dev/ttyUSB0` | Silicon Labs CP2102 USB-UART Bridge | `10c4:ea60` | `.../usb3/3-2/3-2:1.0/ttyUSB0` | `0001` (generic/shared) |
| `/dev/ttyUSB1` | Silicon Labs CP2102 USB-UART Bridge | `10c4:ea60` | `.../usb3/3-3/3-3:1.0/ttyUSB1` | `0001` (generic/shared) |

Both boards report the **same factory default USB serial number (`0001`)**, so they are **not distinguishable by serial number** — any udev rule must key off the physical USB port path, not `ATTRS{serial}`.

- `/dev/ttyUSB0` = **ESP32 base controller**, 115200 baud.
- `/dev/ttyUSB1` = **LD14P LiDAR**, 230400 baud.
- `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` → `../../ttyUSB1` (kernel auto-generated; not port-stable, don't rely on it to pick ESP32 vs LiDAR since both would produce the same by-id name path pattern).

### `/dev/esp32` and `/dev/lidar` symlinks — now working (fixed this session)

`/etc/udev/rules.d/99-bruno-ports.rules` went through two broken iterations before landing on a working rule:

1. Original: `DEVPATH=="*/usb-0:2:1.0/ttyUSB*", SYMLINK+="esp32", ...` — never matched; the real `DEVPATH` segment is `3-2:1.0`, not `usb-0:2:1.0`.
2. Second attempt: `KERNELS=="3-2:1.0", SUBSYSTEM=="tty", SYMLINK+="esp32", ...` — still didn't match at the `tty` device level (the `KERNELS` value `3-2:1.0` belongs to the parent USB-interface device, not the `ttyUSB*` child node itself).

**Working rule (current, verified live):**
```
SUBSYSTEM=="tty", ENV{ID_PATH}=="pci-0000:00:14.0-usb-0:2:1.0", SYMLINK+="esp32", MODE="0666"
SUBSYSTEM=="tty", ENV{ID_PATH}=="pci-0000:00:14.0-usb-0:3:1.0", SYMLINK+="lidar", MODE="0666"
```
`ENV{ID_PATH}` (not `ID_PATH_TAG`, which is the underscore-sanitized variant used for matching elsewhere) is set directly on the `tty`-subsystem device and was confirmed via:
```
udevadm info -q property -p $(udevadm info -q path -n /dev/ttyUSB0) | grep -E "ID_PATH|DEVPATH"
```
Applied via `sudo tee .../99-bruno-ports.rules`, then `sudo udevadm control --reload-rules && sudo udevadm trigger` (run manually by the user — no sudo NOPASSWD rule exists, see below). Confirmed live:
```
/dev/esp32 -> ttyUSB0
/dev/lidar -> ttyUSB1
```
**Caveat:** mid-session, both CP210x adapters briefly disappeared from the USB bus entirely (`lsusb` showed nothing, not just missing symlinks) — root cause was a physical cable/connection issue, resolved by the user replugging. If `/dev/esp32`/`/dev/lidar` ever go missing again, check `lsusb` for the `10c4:ea60` entries before assuming a udev regression.

---

## 2. Base Motor Control & ESP32 Firmware

File: `firmware/bruno_base/bruno_base.ino`. Board FQBN: `esp32:esp32:esp32`. Flashed successfully this session via `arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32:UploadSpeed=115200 ...` (the default 921600 upload speed produced "serial noise/corruption" errors on this board/cable — **115200 upload speed is the reliable setting here**).

### Pinout

| Function | Pins |
|---|---|
| Left motor driver (L298N) | `IN1=16, IN2=18, ENA=25` (ENA is PWM) |
| Right motor driver (L298N) | `IN3=19, IN4=23, ENB=26` (ENB is PWM) |
| Status LED — Blue (driving) | `15` |
| Status LED — Green (heartbeat) | `2` |
| Status LED — Yellow (warning) | `4` |
| Status LED — Red (halt) | `5` |
| Ultrasonic Left (−30°): TRIG / ECHO | `13 / 34` |
| Ultrasonic Center (0°): TRIG / ECHO | `14 / 35` |
| Ultrasonic Right (+30°): TRIG / ECHO | `27 / 39` |

Verified none of the ultrasonic pins fall on GPIO 0/1/2/3 or strapping pins 6–11 (note: `LED_GREEN` sits on GPIO2, a strapping pin, but that's a status LED, not a safety-relevant sensor input, so it was left as-is).

### Motor calibration

- `LEFT_TRIM = 0.65` applied to the **left** motor's commanded PWM only (`applyMotorOutputs()`: `outLeft = commandedLeft * LEFT_TRIM`, `outRight = commandedRight` unmodified). This is a **35% reduction** on the left side.
- In-code comment: *"mechanical trim test: 35% reduction gives straightest open-loop tracking"* — i.e. this value was arrived at empirically (open-loop drive test) to correct a left/right speed mismatch causing the robot to curve instead of drive straight. No closed-loop/encoder feedback exists; this is a static, hand-tuned compensation factor, not derived from a measured motor-current/RPM spec. **No other written record of the specific test data (angle drift measured, etc.) was found in the repo** — if the architect needs the underlying test methodology, that lives only in the user's memory, not in any file.
- **Baseline PWM = 160**: confirmed from the quick-test scripts (`~/scripts/forward.sh` / `backward.sh`, symlinked from `~/.local/bin/forward` and `~/.local/bin/backward`): both send `M 160 160` / `M -160 -160` respectively, wait 3.5s, then `M 0 0` to stop. 160/255 (~63% duty) is the working default used for straight-line smoke tests.
- **Right motor direction inversion:** `applyMotorOutputs()` calls `driveMotor(IN4, IN3, ENB, outRight)` — note `IN4`/`IN3` are swapped relative to the left motor's `driveMotor(IN1, IN2, ENA, outLeft)` call. In-code comment: *"right motor is mounted/wired mirrored; swap IN3/IN4 to invert direction"* — this compensates for the right motor being physically mounted mirror-image to the left, which would otherwise make both motors spin the same rotational direction drive the robot in opposite linear directions.

### Safety logic (ultrasonic obstacle stop)

- `readDistanceCm()` uses `pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US)` with **`ECHO_TIMEOUT_US = 25000UL` (25 ms)** — updated this session from the original 20000UL. This bounds max sensed range to ~4.3 m and guarantees `pulseIn()` can never block the main loop longer than 25ms per sensor (prevents serial-handling starvation).
- `duration == 0` (timeout/no echo) → function returns `-1` ("out of range"/no object detected).
- `withinRange(d, threshold)` explicitly guards `d > 0 && d < threshold` — so `-1` (or any non-positive value) **can never be treated as an obstacle**. Only genuine in-range positive readings below `SAFE_DISTANCE_CM` (15 cm) trigger `haltActive = true` (emergency stop + red LED); readings below `WARN_DISTANCE_CM` (30 cm) only light the yellow warning LED without halting.
- This logic was verified correct by code review this session — no changes were needed beyond the timeout constant above.

### Telemetry / command protocol

- **Outbound (ESP32 → host), every `STREAM_INTERVAL_MS` (150ms):** `U <left_cm> <mid_cm> <right_cm>\n` — e.g. `U -1 -1 -1` when no sensor is connected/in range.
- **Inbound (host → ESP32):** `M <left_pwm> <right_pwm>\n`, parsed via `sscanf(" %c %d %d", ...)`, both values `constrain()`-ed to `[-255, 255]`. `LEFT_TRIM` is applied only at output time, not at parse time — `commandedLeft`/`commandedRight` store the raw requested values.
- Baud rate: **115200**, confirmed live by `stty`+`cat` on `/dev/ttyUSB0` streaming `U -1 -1 -1` lines after flashing.

### Quick-test utility scripts

- `~/scripts/forward.sh`, `~/scripts/backward.sh` — Python3 (`pyserial`), hardcoded to `/dev/ttyUSB0` @ 115200, send `M 160 160` / `M -160 -160` for 3.5s then stop.
- Symlinked into `~/.local/bin/forward` and `~/.local/bin/backward` (both already on PATH alongside `arduino-cli`).
- **Caveat:** these scripts hardcode `/dev/ttyUSB0` directly — once the `/dev/esp32` udev symlink is fixed (§1), consider repointing these at `/dev/esp32` for robustness against USB enumeration order changes across reboots.

---

## 3. LiDAR (LD14P / LD19) Integration

- **Driver package:** [`ldlidar_stl_ros2`](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2) (official LDROBOT repo — note: a near-identical-named repo under a different GitHub org returned 404 and was a dead end; the correct org is `ldrobotSensorTeam`). Cloned into `ros2_ws/src/ldlidar_stl_ros2`, commit `bf668a8` ("Update README.md").
- **Model workaround:** This driver version's `ldlidar::LDType` enum only defines `NO_VERSION, LD_06, LD_19, STL_06P, STL_26, STL_27L` — **there is no dedicated LD14P entry.** Per explicit user confirmation this session, we use the documented community workaround: **`product_name: 'LDLiDAR_LD19'`**, since the LD14P shares the LD19's packet protocol and baud rate. This is an assumption carried over from the user's own research, not independently verified against an LDROBOT datasheet in this session.
- **Launch file:** `ros2_ws/src/ldlidar_stl_ros2/launch/ld14p.launch.py` (based on the stock `ld19.launch.py`), final config:
  - `product_name: 'LDLiDAR_LD19'`
  - `topic_name: 'scan'` → publishes `sensor_msgs/msg/LaserScan` on `/scan`
  - `frame_id: 'laser_frame'`
  - `port_name: '/dev/lidar'` (updated from `/dev/ttyUSB1` once the udev symlink above was confirmed stable)
  - `port_baudrate: 230400`
  - angle crop disabled, `laser_scan_dir: True` (unchanged defaults from `ld19.launch.py`)
- **Static TF node** (bundled into the same launch file, as a second `Node` action using `tf2_ros static_transform_publisher`):
  - `base_link` → `laser_frame`
  - translation: `x=0.08, y=0.0, z=0.12`
  - rotation: `yaw=0, pitch=0, roll=0`
  - invoked with the modern named-argument form: `--x --y --z --yaw --pitch --roll --frame-id --child-frame-id` (not the legacy 6-positional-arg form used in the stock `ld19.launch.py` sibling file).
  - **Validated live:** launch log shows `translation: ('0.080000', '0.000000', '0.120000')`, `rotation: ('0.000000', '0.000000', '0.000000', '1.000000')`, `from 'base_link' to 'laser_frame'` — matches spec exactly.

### Upstream source bug fixed during build

`colcon build` initially failed (after `g++` was installed) with:
```
log_module.cpp:172: error: 'pthread_mutex_init' was not declared in this scope
```
Root cause: `ldlidar_driver/src/logger/log_module.cpp` calls `pthread_mutex_init/lock/unlock` but never includes `<pthread.h>` — a pre-existing bug in the upstream `ldrobotSensorTeam/ldlidar_stl_ros2` source (commit `bf668a8`) that older/more lenient GCC versions tolerated implicitly; GCC 13.3.0 on this box rejects it. **Fix applied:** added `#include <pthread.h>` alongside the existing `<time.h>`/`<string.h>` includes near the top of that file. After this one-line fix, `colcon build --symlink-install` succeeds cleanly (`1 package finished`).

### Verification status: **CONFIRMED WORKING**

Build succeeded, node launched, and `/scan` verified live this session:

- **Launch:** `ros2 launch ldlidar_stl_ros2 ld14p.launch.py` → node log shows `ldlidar node start is success`, `ldlidar communication is normal.`, `Publish topic message:ldlidar scan data.`, and `Actual BaudRate reported:230400` (confirms the 230400 baud link to the physical LD14P is actually active, not just configured).
- **`ros2 topic hz /scan`:** steady **~6.0–6.16 Hz** (min 0.100s / max 0.200s / std dev ~0.048s). **This is lower than the LD19's nominal ~10 Hz spec, but confirmed expected for this specific physical LD14P hardware revision** — not a config or protocol bug. The message's own self-reported `scan_time` field (`~0.200s`, i.e. 5 Hz) is internally consistent with the measured rate.
- **`ros2 topic echo /scan --once`:** healthy full-circle data — `angle_min: 0.0`, `angle_max: 6.2831854820251465` (2π), `range_min/max: 0.02/25.0`, plausible in-room ranges (~1–6 m), intensities in a sane 0–255-ish band, `NaN` only on genuine no-return angles (expected/normal, not an error).
- **Conclusion:** the `LDLiDAR_LD19` product-type workaround (§ above) decodes real LD14P packets correctly — protocol compatibility is now empirically confirmed, not just assumed.
- **Shutdown:** verified clean — `SIGINT` to the `ros2 launch` parent process cascades correctly; log shows `ldlidar_published is end` and `process has finished cleanly` for both the lidar node and the static TF publisher. No orphaned processes.

---

## 4. ESP32 ROS 2 Base Controller Bridge (`bruno_base` package)

**Status: built and verified live this session.** This closes the gap flagged in earlier revisions of this doc — there is now a real ROS 2 node sitting on top of the `U`/`M` serial protocol from §2.

- **Package:** `ros2_ws/src/bruno_base/` — `ament_python`, scaffolded via `ros2 pkg create`.
  - `package.xml` depends on `rclcpp`, `rclpy`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, **`python3-serial`**. Note: the pyserial dependency was requested as the literal rosdep key `pyserial`, but that key doesn't exist in the rosdep database (`rosdep resolve pyserial` → `ERROR: no rosdep rule for 'pyserial'`); the correct key is `python3-serial`, which resolves to the already-installed apt package providing the `serial` module (confirmed importable, v3.5). Also note `rclcpp` is a C++ client library with no actual use in this pure-Python package — kept as a declared dependency per explicit request, but it's inert.
  - `rosdep install --from-paths src/bruno_base --ignore-src -y` → clean.
  - `colcon build --symlink-install --packages-select bruno_base` → **succeeds** (`1 package finished`).

- **Node:** `bruno_base/bruno_base_node.py`, console-script entry point `bruno_base_node`.
  - **Parameters:** `port` (default `/dev/esp32`), `baud` (115200), `max_pwm` (160), `cmd_vel_timeout_sec` (0.5) — all configurable at launch.
  - **`/cmd_vel` (`geometry_msgs/msg/Twist`) → `M <left> <right>\n`:** differential-drive mixing `left = linear.x - angular.z`, `right = linear.x + angular.z`, each scaled by `max_pwm` and clamped to the hardware's `[-255, 255]` bound.
    - **Important documented assumption:** there is no URDF, wheel radius, or track width defined anywhere in this project yet (open-loop motors, no encoders — see §2/§6), so true m/s → PWM conversion isn't possible. `linear.x`/`angular.z` are therefore treated as normalized ~[-1, 1] control inputs, not calibrated physical units. Revisit this mixing function once real robot geometry exists.
  - **Safety timeout:** a 0.1s-period timer checks time since the last `/cmd_vel`; if it exceeds `cmd_vel_timeout_sec` (500ms default), sends `M 0 0` once and sets a flag so it doesn't spam the serial port; the flag clears on the next received `/cmd_vel`. Also sends one `M 0 0` immediately at node startup as a defensive default.
  - **Sonar telemetry:** a dedicated daemon thread blocks on `self.ser.readline()`, parses `U <left_cm> <mid_cm> <right_cm>`, and publishes three `sensor_msgs/msg/Range` topics — `/sonar/left`, `/sonar/center`, `/sonar/right` (`frame_id`s `sonar_{left,center,right}_link` — **no static TF exists for these frames yet**, same caveat pattern as `laser_frame` before §3's TF was added). Malformed lines (wrong token count / non-numeric) are logged and skipped, not fatal.
    - `-1` (or any non-positive reading, matching the firmware's "no echo" convention from §2) is reported as `range = max_range` (4.0 m), per the `sensor_msgs/Range` convention for an out-of-range reading. `min_range`/`max_range`/`field_of_view` are set to HC-SR04-typical values (0.02 m / 4.0 m / 0.26 rad) — the 4.0 m max_range figure matches the firmware's `ECHO_TIMEOUT_US = 25000` (~4.3 m theoretical cap, §2), not independently re-measured.
  - A `threading.Lock` guards `self.ser.write()` calls, since both the `/cmd_vel` callback and the timeout timer can write, concurrently with the reader thread reading from the same `Serial` object.

### Bug found and fixed during live testing

Initial version's `main()` called `rclpy.shutdown()` unconditionally in a `finally` block. On `SIGINT`, rclpy's own default signal handler already shuts down the context before the `KeyboardInterrupt` is caught here, so the explicit `rclpy.shutdown()` raised `RCLError: failed to shutdown: rcl_shutdown already called` and the process exited with a traceback (no orphaned process was left, but the exit wasn't clean). **Fix:** guarded with `if rclpy.ok(): rclpy.shutdown()`. Re-tested after the fix — clean exit, no traceback, confirmed via `ps`/`pgrep` that no node or wrapper process remained.

### Verification performed live this session

- Node launched against the real `/dev/esp32` device: log showed `Opened /dev/esp32 @ 115200 baud`, followed immediately by the expected `No /cmd_vel for 0.5s - sending M 0 0` warning (confirms the timeout logic fires correctly on a cold start with no commands yet).
- `ros2 topic list` showed `/cmd_vel`, `/sonar/left`, `/sonar/center`, `/sonar/right` all present.
- `ros2 topic echo /sonar/left --once` → `range: 4.0`, `max_range: 4.0`, `frame_id: sonar_left_link` — correct out-of-range reporting (no physical sonar sensors connected during this test).
- Published one `/cmd_vel` message via `ros2 topic pub --once`; confirmed the timeout warning did **not** fire immediately after (proves the subscriber callback resets the timeout clock), then confirmed it **did** fire again ~0.5s after that single message stopped arriving (proves correct re-arming, not just a one-shot suppression).
- Did not independently sniff the actual `M <l> <r>` bytes on the wire (the node held the serial port exclusively) — the PWM math itself was verified by code review, not by an external byte-level capture.

### Dry-run test instructions

```bash
cd ~/bruno_robot/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run bruno_base bruno_base_node
# in a second terminal (also source install/setup.bash):
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
ros2 topic echo /sonar/left
```

---

## 5. Repository & Workspace Structure

`/home/bruno/bruno_robot` is **not a git repository** (`git rev-parse` fails there) — only `ros2_ws/src/ldlidar_stl_ros2` inside it is its own separate git clone.

```
bruno_robot/                          (plain directory, not a git repo)
├── CLAUDE.md                         (hardware pin map + flashing instructions)
├── SESSION_HANDOVER.md               (this file)
├── .claude/settings.local.json
├── firmware/
│   └── bruno_base/
│       └── bruno_base.ino            (ESP32 base controller firmware — §2)
└── ros2_ws/                          (colcon workspace, created this session)
    ├── build/                        (successful build artifacts — ldlidar_stl_ros2, bruno_base)
    ├── install/                      (populated — both packages symlink-installed)
    ├── log/
    └── src/
        ├── ldlidar_stl_ros2/         (git clone of ldrobotSensorTeam/ldlidar_stl_ros2, commit bf668a8, + 1-line local fix)
        │   ├── launch/
        │   │   ├── ld06.launch.py / viewer_ld06.launch.py
        │   │   ├── ld19.launch.py  / viewer_ld19.launch.py
        │   │   ├── stl27l.launch.py / viewer_stl27l.launch.py
        │   │   └── ld14p.launch.py           ← created this session, confirmed working (§3)
        │   ├── ldlidar_driver/src/logger/log_module.cpp  ← patched: added #include <pthread.h> (§3)
        │   ├── ldlidar_driver/ (core C++ driver, src + include)
        │   ├── include/, src/ (ROS2 node wrapper)
        │   ├── rviz2/, scripts/
        │   ├── CMakeLists.txt, package.xml
        │   └── README.md
        └── bruno_base/               ← NEW this session, ament_python, confirmed working (§4)
            ├── package.xml
            ├── setup.py / setup.cfg
            ├── resource/bruno_base
            ├── bruno_base/
            │   ├── __init__.py
            │   └── bruno_base_node.py
            └── test/ (default ament_python lint scaffolding: copyright/flake8/pep257)
~/scripts/forward.sh, backward.sh     (quick-test motor scripts, §2)
~/.local/bin/{forward,backward}       (symlinks to the above)
~/.local/bin/arduino-cli              (v1.5.1)
/etc/udev/rules.d/99-bruno-ports.rules  (fixed and verified working — §1)
```

### Dependency / build status

- `rosdep install --from-paths src --ignore-src -y` → **"All required rosdeps installed successfully"** for both packages (`ldlidar_stl_ros2` needs `rclcpp`, `sensor_msgs`, `ros2launch`; `bruno_base` needs `rclcpp`, `rclpy`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `python3-serial` — all satisfied by `ros-jazzy-ros-base` + already-installed `python3-serial`).
- `colcon build --symlink-install` → **SUCCEEDS** for both packages (after installing `g++` and patching the missing `<pthread.h>` include in `ldlidar_stl_ros2` — see §3; `bruno_base` built clean on the first attempt, no fixes needed in the build itself).

---

## 6. Current State & Next Immediate Milestones

### Exactly where we stopped

Two milestones are now **complete and verified end-to-end**:
1. **LiDAR integration (§3):** `g++` installed → udev symlinks fixed → `colcon build` succeeded (after a 1-line upstream patch) → node launched → `/scan` confirmed publishing clean, stable 360° data at ~6 Hz → static TF confirmed correct → shut down cleanly.
2. **ESP32 ROS 2 base controller bridge (§4):** `bruno_base` package built, launched against the real ESP32, `/cmd_vel` → motor command path and the 500ms safety timeout both confirmed via live behavioral tests, `/sonar/{left,center,right}` confirmed publishing correctly-formed `Range` messages, a real shutdown bug was found and fixed, re-verified clean after the fix.

Nothing is currently running — both test sessions were shut down cleanly (no orphaned processes) after verification.

### Immediate next action (resume here)

No blockers remain from either milestone. Reasonable next steps for the architect to choose from:
1. **Bringup / persistence:** nothing currently auto-starts any of the three pieces (ESP32 bridge, LiDAR driver) on boot, and there's no combined launch file bringing both up together. A `bruno_bringup` launch file (or systemd units) wrapping `bruno_base_node` + `ld14p.launch.py` is the natural next piece of plumbing before this is usable unattended.
2. **Real cmd_vel source:** nothing currently publishes to `/cmd_vel` except the manual `ros2 topic pub` test command — e.g. `teleop_twist_keyboard` for manual driving, or planning ahead to Nav2's velocity output once that exists.
3. **URDF / robot_state_publisher / odometry / Nav2 / SLAM** — none of this exists yet (see gaps below). The system currently has exactly one static TF link (`base_link` → `laser_frame`, §3) and zero dynamic ones; there's no `base_link`-publishing node (no URDF, no odom frame) for anything else to attach to, and no odometry source at all (open-loop motors, no encoders).

### Outstanding items / known gaps (not yet addressed)

- **No sudo NOPASSWD rule exists** — every future privileged step (apt installs, udev rule changes, etc.) needs to be manually run by the user; plan handovers accordingly.
- **No bringup/persistence layer.** Both working pieces (§3 LiDAR, §4 base bridge) have to be launched manually right now; nothing survives a reboot or crash automatically.
- **No odometry, Nav2, SLAM, or URDF/robot_state_publisher work has started.** This session covered: (a) firmware timeout/correctness check + reflash, (b) getting the LiDAR driver fully working, and (c) getting a ROS 2 bridge node talking to the ESP32. Wheel encoders don't appear to exist on this platform — motor control is open-loop per §2 — so any odometry source will need to come from elsewhere (e.g. wheel velocity estimation from commanded PWM, or purely LiDAR-based localization).
- **`bruno_base_node`'s `/cmd_vel` → PWM mixing is unitless/uncalibrated** (§4) — treats `linear.x`/`angular.z` as normalized ~[-1,1] inputs rather than real m/s or rad/s, because no wheel radius or track width has been established anywhere in the project. Revisit once real robot geometry (URDF) exists.
- **`/sonar/{left,center,right}` have no corresponding static TFs** (`sonar_left_link` etc. are referenced as `frame_id`s in the `Range` messages but nothing publishes a transform for them, same gap pattern `laser_frame` had before §3's fix) — add static (or URDF-embedded) transforms from `base_link` to each sonar mount point before anything tries to consume these in a costmap.
- **LD14P's ~6 Hz native scan rate** (vs. the LD19's ~10 Hz) should be kept in mind for any downstream consumer tuning (e.g. costmap update rates, SLAM scan-matching parameters) — it's confirmed-expected behavior for this hardware revision, not a bug, but it is slower than typical LD19/LD06 tutorials assume.
- `bruno_robot/` itself has no git history/version control — consider whether that's intentional before this grows further (only the vendored `ldlidar_stl_ros2` driver is a git repo, and that's just the upstream clone — it also now carries one uncommitted local patch, the `<pthread.h>` include fix, which would be lost if that clone is ever reset/re-cloned). `bruno_base` is brand new and also has no version control of its own yet.
