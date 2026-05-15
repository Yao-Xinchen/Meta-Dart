# Meta-Dart Dart Launcher

C++17 application for the complete Meta-Dart dart launcher. It detects darts with a camera + YOLO/ONNX pipeline, picks a stable dart from one of the configured slots, moves it with a Dynamixel OpenManipulator-style arm to the launcher loading position, releases it, then commands two DJI M3508 spring-stretcher motors to pull and retract the launcher hooks.

The trigger/release mechanism is not implemented here yet. The current shooting-side code only stretches the springs and retracts the motors after the dart has been loaded.

## System Overview

Main modules:

| Module | Files | Role |
|---|---|---|
| Camera | `include/camera.hpp`, `src/camera.cpp` | Captures OpenCV frames from `CAMERA_INDEX`. |
| Vision | `include/vision.hpp`, `src/vision.cpp` | Runs YOLO/ONNX dart detection and publishes `Detection`. Falls back to stub mode if the ONNX model is missing. |
| Arm | `include/arm.hpp`, `src/arm.cpp` | Controls Dynamixel joints/gripper, loads named poses from `positions.xml`. |
| Decision | `include/decision.hpp`, `src/decision.cpp` | Stable-detection pickup state machine, slot trajectories, disturbance handling. |
| Spring stretcher | `include/spring_stretcher.hpp`, `src/spring_stretcher.cpp` | Controls two DJI M3508 motors over SocketCAN, calibrates top stops, stretches springs, retracts hooks. |
| Runtime config | `include/runtime_config.hpp`, `src/runtime_config.cpp` | Loads JSON runtime parameters from `config/loader_config.json`. |

High-level flow:

1. Start camera, vision, arm, and spring stretcher.
2. Spring stretcher calibrates both hooks by moving them to the top hard stops.
3. Arm moves to the search pose.
4. Vision detects a dart in a configured slot.
5. Decision waits until the dart is stable.
6. Arm executes that slot's pickup trajectory.
7. At `loading`, the gripper opens and releases the dart into the launcher.
8. If spring stretcher hardware is online and calibrated, it stretches the springs, holds briefly, then retracts the motors.

## Dependencies

- CMake and a C++17 compiler
- OpenCV development package
- ONNX Runtime C++ at `~/Applications/onnxruntime` by default
- DynamixelSDK and dynamixel-workbench submodules under `thirdparty/`
- Linux SocketCAN for the DJI M3508 spring stretcher

ONNX Runtime default path:

```bash
~/Applications/onnxruntime
```

Override it at configure time:

```bash
cmake -S . -B build -DORT_ROOT=/your/path/to/onnxruntime
```

## Hardware Setup

Static hardware constants live in `include/config.hpp`.

Important defaults:

```cpp
constexpr const char* DXL_PORT = "/dev/ttyUSB0";
constexpr int DXL_BAUD = 1000000;
constexpr int CAMERA_INDEX = 2;

constexpr const char* SPRING_STRETCHER_CAN_IFACE = "can0";
constexpr uint8_t SPRING_STRETCHER_LEFT_ID = 1;
constexpr uint8_t SPRING_STRETCHER_RIGHT_ID = 2;
```

Add your user to the serial/video groups, then log out and back in:

```bash
sudo usermod -aG dialout,video $USER
```

Bring up CAN before using the spring stretcher. Example for 1 Mbps:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

If `can0` is already up with the wrong bitrate:

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
```

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

Build only the spring stretcher tuning tools:

```bash
cmake -S . -B build
cmake --build build --target \
  test_stretcher_monitor \
  test_stretcher_direction \
  test_stretcher_calibration \
  test_stretcher_stretch
```

## Run

```bash
# Run with the normal runtime config
./build/dart_launcher config/loader_config.json

# Force replay mode: execute the stored trajectory without monitoring dart motion
./build/dart_launcher config/loader_config.json --replay

# Force disturb mode: pause/replan behavior if the dart moves during pickup
./build/dart_launcher config/loader_config.json --disturb

# Check config parsing without starting hardware
./build/dart_launcher --check-config config/loader_config.json
```

Environment overrides:

```bash
META_DART_CONFIG=config/loader_config.json ./build/dart_launcher
META_DART_CONF_THRESHOLD=0.25 ./build/dart_launcher config/loader_config.json
NO_COLOR=1 ./build/dart_launcher config/loader_config.json
```

Shutdown controls:

- Press `q` in the terminal or OpenCV window to stop, return the arm home, and disconnect.
- Press `Ctrl-C` to request normal shutdown without the keyboard-triggered emergency home move.

## Runtime Configuration

The main runtime file is `config/loader_config.json`.

### Vision

| Field | Meaning |
|---|---|
| `vision.confidence_threshold` | YOLO confidence threshold for accepting detections. |

The model path itself is currently static in `include/config.hpp`:

```cpp
constexpr const char* ONNX_MODEL_PATH = "models/best.onnx";
```

### Decision

Important decision fields:

| Field | Meaning |
|---|---|
| `stable_dart_time_s` | Dart must remain stable this long before pickup starts. |
| `search_stability_grace_s` | Grace time for brief detection loss during search. |
| `search_stability_bbox_margin_m` | Extra margin around search zone during stability checks. |
| `pickup_mode` | `"disturb"` monitors dart motion during pickup; `"replay"` just runs the trajectory. |
| `joint_cruise_speed_rad_s` | Nominal arm joint speed for trajectory timing. |
| `fast_motion_speedup` | Speedup for non-guarded moves. |
| `move_timeout_s` | Max time to wait for a motion segment. |
| `gripper_settle_s` | Delay after open/close commands. |
| `loading_release_tolerance_rad` | How close the arm must be to `loading` before releasing the dart. |
| `dart_motion_hold_s` | Hold time after disturbance is detected. |
| `relocation_window_s` | Observation window for finding a moved dart. |
| `emergency_home_duration_s` | Duration used when `q` requests return home. |

### Zones

`decision.zones` defines camera-space dart slots in meters:

```json
{
  "name": "slot_0",
  "center_x": -0.112,
  "center_y": -0.020,
  "half_width": 0.020,
  "half_height": 0.020,
  "trajectory": "slot_0",
  "pickup_half_width": 0.035,
  "pickup_half_height": 0.050
}
```

- `half_width` / `half_height`: box used while searching for a stable dart.
- `pickup_half_width` / `pickup_half_height`: larger guard box while the arm approaches.
- `pickup_outside_grace_s`: optional per-zone override for how long a dart may leave the guard box before pickup is aborted.

### Trajectories

Each trajectory is a list of named arm poses plus gripper actions:

```json
"slot_0": [
  {"position": "s0_prep", "gripper": "open"},
  {"position": "s0_grasp", "gripper": "close"},
  {"position": "s0_pulled_out", "gripper": "none"},
  {"position": "backward_prep", "gripper": "none"},
  {"position": "forward_prep", "gripper": "none"},
  {"position": "loading", "gripper": "open"},
  {"position": "high_prep", "gripper": "none"},
  {"position": "backward_prep", "gripper": "none"}
]
```

Valid gripper actions are `"none"`, `"open"`, and `"close"`.

All `position` names must exist in `positions.xml`.

## Positions File

The arm loads `positions.xml` from the current working directory. This repo also has a copy under `data/positions.xml`; keep the runtime file you use consistent with the directory you run from.

Record positions by hand-dragging the arm:

```bash
./build/record_positions positions.xml
```

Inspect named positions without running the full dart launcher:

```bash
./build/test_named_positions positions.xml home forward_prep backward_prep loading
```

Run a trajectory sequence:

```bash
sudo ./build/test_trajectory positions.xml s0_prep s0_grasp s0_pulled_out loading
```

## Spring Stretcher

The spring stretcher uses two DJI M3508 motors over SocketCAN. The motors do not have an absolute zero, so startup calibration velocity-controls each hook toward the top hard stop. When the hook stops moving for long enough, that top-stop encoder position becomes zero/home.

Normal stretch sequence:

1. Calibrate top stops.
2. Wait for a dart to be released at `loading`.
3. Move both hooks to `SPRING_STRETCHER_STRETCH_RAD` from zero.
4. Hold for `SPRING_STRETCHER_HOLD_TIME_S`.
5. Retract motors back to `SPRING_STRETCHER_HOME_RAD`.

Static stretcher constants in `include/config.hpp`:

| Constant | Meaning |
|---|---|
| `SPRING_STRETCHER_CAN_IFACE` | SocketCAN interface, usually `can0`. |
| `SPRING_STRETCHER_LEFT_ID`, `SPRING_STRETCHER_RIGHT_ID` | DJI hardware IDs. |
| `SPRING_STRETCHER_LEFT_DIR`, `SPRING_STRETCHER_RIGHT_DIR` | Direction sign. Positive means stretch direction. |
| `SPRING_STRETCHER_STRETCH_RAD` | Pull distance from calibrated top/home. |
| `SPRING_STRETCHER_HOME_RAD` | Retracted position, usually `0`. |
| `SPRING_STRETCHER_POS_TOL_RAD` | Position tolerance for goal reached. |
| `SPRING_STRETCHER_VEL_TOL_RAD_S` | Velocity tolerance for settled goal reached. |
| `SPRING_STRETCHER_HOLD_TIME_S` | Time to hold stretched position before retracting. |
| `SPRING_STRETCHER_CALIB_VEL_RAD_S` | Target hook velocity toward the top stop during calibration. |
| `SPRING_STRETCHER_CALIB_MAX_CURRENT_A` | Current safety cap for calibration velocity control. |
| `SPRING_STRETCHER_CALIB_VEL_KP/KD` | Calibration velocity-to-current gains. |
| `SPRING_STRETCHER_CALIB_JAM_VEL_RAD_S` | Velocity threshold for “stuck at top”. |
| `SPRING_STRETCHER_CALIB_JAM_TIME_S` | Time below jam velocity before zero is accepted. |
| `SPRING_STRETCHER_CALIB_TIMEOUT_S` | Safety timeout for calibration. |
| `SPRING_STRETCHER_POS_KP/KI/KD` | Position-to-velocity gains. |
| `SPRING_STRETCHER_VEL_KP/KI/KD` | Velocity-to-current gains. |
| `SPRING_STRETCHER_MAX_VEL_RAD_S` | Velocity command limit. |
| `SPRING_STRETCHER_MAX_CURRENT_A` | Current command limit. |

### Spring Stretcher Tuning Tools

Run these in order. They live in `test/` and build as standalone targets.

| Target | Moves motors? | Purpose |
|---|---:|---|
| `test_stretcher_monitor` | No, sends zero current | Check CAN interface, motor IDs, feedback, encoder direction by hand. |
| `test_stretcher_direction` | Yes, small fixed current | Verify `LEFT_DIR` and `RIGHT_DIR` make positive motion stretch the springs. |
| `test_stretcher_calibration` | Yes, toward top stop | Tune top-stop zeroing velocity, current cap, gains, jam velocity, jam time, timeout. |
| `test_stretcher_stretch` | Yes, full cycle | Calibrate, stretch to target, hold, retract; tune stretch length and gains. |

Commands:

```bash
# 1. Passive monitor: both motors should show seen=1
sudo ./build/test_stretcher_monitor 10 can0

# 2. Direction check: small current for one second
sudo ./build/test_stretcher_direction 1.0 1.0 1 -1 can0

# 3. Calibration tuning
sudo ./build/test_stretcher_calibration 6.0 3.0 0.5 0.0 0.5 0.35 4.0 1 -1 can0

# 4. Stretch cycle tuning
sudo ./build/test_stretcher_stretch 6.0 10.0 35.0 0.6 60.0 0.25 can0
```

Tuning order:

1. `test_stretcher_monitor`: verify CAN and IDs.
2. `test_stretcher_direction`: set `SPRING_STRETCHER_LEFT_DIR` and `SPRING_STRETCHER_RIGHT_DIR`.
3. `test_stretcher_calibration`: tune calibration velocity control and jam detection.
4. `test_stretcher_stretch`: tune stretch distance, current limit, `POS_KP`, `VEL_KP`, and max velocity.

## Test Targets

| Target | Purpose |
|---|---|
| `test_vision_camera` | Camera + YOLO only, no arm movement. |
| `test_stretcher_monitor` | Passive CAN feedback monitor for M3508 stretcher motors. |
| `test_stretcher_direction` | Fixed-current direction test for stretcher motors. |
| `test_stretcher_calibration` | Top-stop calibration test for stretcher motors. |
| `test_stretcher_stretch` | Calibrate, stretch, hold, retract cycle. |
| `test_arm` | Arm hardware basics: init, home, gripper, joint sweeps. |
| `test_trajectory` | Move through named arm positions. |
| `test_gripper_trajectory` | Move through named arm positions with gripper actions. |
| `test_named_positions` | Inspect and move to named positions one by one. |
| `record_positions` | Hand-drag arm and record named positions to XML. |
| `scan_motors` | Scan Dynamixel bus. |
| `calibrate_gripper` | Manual gripper calibration helper. |
| `clear_errors` | Clear Dynamixel hardware errors. |

Useful examples:

```bash
# Safe vision test, no arm movement
./build/test_vision_camera

# Scan Dynamixel bus
./build/scan_motors

# Clear Dynamixel errors
./build/clear_errors

# Record positions
./build/record_positions positions.xml

# Inspect named positions
./build/test_named_positions positions.xml home forward_prep backward_prep loading
```

## Environment Check

Run the environment checker before first use:

```bash
./test/check_meta_dart_environment.sh
```

Optional checks:

```bash
RUN_BUILD_CHECK=1 ./test/check_meta_dart_environment.sh
RUN_BUILD_CHECK=1 RUN_MOTOR_SCAN=1 ./test/check_meta_dart_environment.sh
```

## Development Notes

- Prefer changing runtime behavior in `config/loader_config.json` when possible.
- Changes to `include/config.hpp` require recompilation.
- Vision can run without arm hardware; the main program continues in camera-only mode if `ArmModule::start()` fails.
- Spring stretching is skipped unless the stretcher reports both `hw_ok` and `calibrated`.
- The stretcher tests use direct SocketCAN helper code in `test/stretcher_test_utils.hpp`; they do not depend on the full dart launcher state machine.

## Troubleshooting

**Arm does not connect**

Check `DXL_PORT`, permissions, power, and IDs 11-15. Then try:

```bash
./build/scan_motors
./build/clear_errors
```

**Camera opens but no detections appear**

Run:

```bash
./build/test_vision_camera
```

Check `CAMERA_INDEX`, `models/best.onnx`, lighting, and `vision.confidence_threshold`.

**YOLO detects but pickup does not trigger**

Check the printed x/y detection against `decision.zones` in `config/loader_config.json`. Increase `half_width` / `half_height` if the slot box is too tight.

**Pickup aborts because dart moved**

This is expected in disturb mode. Tune `pickup_motion_bbox_margin_m`, `pickup_motion_outside_grace_s`, and relocation fields, or run with `--replay` to ignore dart motion during pickup.

**Arm shakes or hunts**

Reduce `joint_cruise_speed_rad_s`, increase segment durations, or reduce `JOINT_POSITION_P_GAIN` in `include/config.hpp`. Also verify recorded poses are not near joint limits.

**`loading` timeout in logs**

Increase `loading_release_tolerance_rad` or `move_timeout_s`, and verify `loading` physically:

```bash
./build/test_named_positions positions.xml loading
```

**Spring stretcher does not start**

Check CAN:

```bash
ip link show can0
sudo ./build/test_stretcher_monitor 10 can0
```

If no feedback appears, check CAN wiring, bitrate, motor power, and `SPRING_STRETCHER_LEFT_ID` / `RIGHT_ID`.

**Stretcher calibration declares zero too early**

Lower `SPRING_STRETCHER_CALIB_JAM_VEL_RAD_S` or increase `SPRING_STRETCHER_CALIB_JAM_TIME_S`.

**Stretcher calibration grinds at the top stop**

Lower `SPRING_STRETCHER_CALIB_VEL_RAD_S`, lower `SPRING_STRETCHER_CALIB_MAX_CURRENT_A`, or lower `SPRING_STRETCHER_CALIB_JAM_TIME_S` after confirming velocity noise is acceptable.

**Stretch motion oscillates**

Lower `SPRING_STRETCHER_POS_KP`, `SPRING_STRETCHER_VEL_KP`, or `SPRING_STRETCHER_MAX_CURRENT_A`. Use `test_stretcher_stretch` before running the full dart launcher.
