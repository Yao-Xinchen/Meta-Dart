# Meta-Dart

C++17 application that detects darts with a YOLO/ONNX camera pipeline and picks them up with a Dynamixel robot arm, sorted by slot.

## Dependencies

- CMake, g++ (C++17)
- OpenCV (`libopencv-dev`)
- ONNX Runtime C++ — expected at `~/Applications/onnxruntime` (symlink or directory)
- DynamixelSDK and dynamixel-workbench submodules

```bash
sudo apt-get install -y build-essential cmake pkg-config libopencv-dev v4l-utils
```

ONNX Runtime default path: `~/Applications/onnxruntime`. Override at configure time:

```bash
cmake -S . -B build -DORT_ROOT=/your/path/to/onnxruntime
```

## Hardware

Edit `include/config.hpp` for your setup:

```cpp
constexpr const char* DXL_PORT    = "/dev/ttyUSB0";  // Dynamixel U2D2 serial port
constexpr int         CAMERA_INDEX = 2;               // camera device index
```

Add your user to the required groups:

```bash
sudo usermod -aG dialout,video $USER  # then re-login
```

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

## Run

```bash
# Run with config file (disturb mode by default)
./build/loader config/loader_config.json

# Force replay mode (no dart monitoring during pickup)
./build/loader config/loader_config.json --replay

# Force disturb mode (pause arm if dart moves during approach)
./build/loader config/loader_config.json --disturb

# Check config without starting hardware
./build/loader --check-config config/loader_config.json
```

Environment variable overrides:

```bash
META_DART_CONFIG=config/loader_config.json ./build/loader
META_DART_CONF_THRESHOLD=0.25 ./build/loader config/loader_config.json
NO_COLOR=1 ./build/loader config/loader_config.json
```

**Press `q`** (in terminal or OpenCV window) to stop the arm, return it to home, and exit.  
**Press `Ctrl-C`** to stop immediately without returning home.

## Test Targets

| Target | Purpose |
|---|---|
| `test_vision_camera` | Camera + YOLO only, no arm |
| `test_arm` | Arm hardware basics |
| `test_trajectory` | Joint trajectory |
| `test_gripper_trajectory` | Trajectory with gripper open/close |
| `test_named_positions` | Move arm through named positions one by one |
| `record_positions` | Hand-drag arm and record named positions to XML |
| `scan_motors` | Scan Dynamixel bus |
| `calibrate_gripper` | Gripper calibration |
| `clear_errors` | Clear motor hardware errors |

```bash
# Safe vision test — no arm movement
./build/test_vision_camera

# Record positions
./build/record_positions positions.xml

# Inspect named positions
./build/test_named_positions positions.xml home forward_prep backward_prep
```

## Configuration

All runtime parameters live in `config/loader_config.json`. Key fields:

### Vision

| Field | Default | Notes |
|---|---|---|
| `vision.confidence_threshold` | `0.2` | YOLO detection threshold |

### Decision

| Field | Default | Notes |
|---|---|---|
| `stable_dart_time_s` | `1.0` | How long dart must stay in a slot to trigger pickup |
| `pickup_mode` | `"disturb"` | `"disturb"` monitors dart during approach; `"replay"` does not |
| `dart_motion_hold_s` | `5.0` | How long the arm holds position after detecting dart moved |
| `relocation_window_s` | `4.0` | Observation window after disturbance to find new slot |
| `joint_cruise_speed_rad_s` | `0.45` | Arm speed (rad/s) |
| `fast_motion_speedup` | `1.6` | Speedup factor for non-guarded moves |
| `emergency_home_duration_s` | `1.2` | Duration of emergency return home on `q` |

### Slots / Zones

Each zone in `decision.zones` defines a bounding box in camera space (meters):

```json
{
  "name": "slot_0",
  "center_x": -0.112, "center_y": -0.020,
  "half_width": 0.020, "half_height": 0.020,
  "trajectory": "slot_0",
  "pickup_half_width": 0.035, "pickup_half_height": 0.050
}
```

- `half_width/half_height` — search detection box
- `pickup_half_width/pickup_half_height` — larger guard box used during approach

### Trajectories

Each trajectory is a list of `{position, gripper}` steps in `decision.trajectories`:

```json
"slot_0": [
  {"position": "s0_prep",       "gripper": "open"},
  {"position": "s0_grasp",      "gripper": "close"},
  {"position": "s0_pulled_out", "gripper": "none"},
  {"position": "backward_prep", "gripper": "none"},
  {"position": "forward_prep",  "gripper": "none"},
  {"position": "loading",       "gripper": "open"},
  {"position": "high_prep",     "gripper": "none"},
  {"position": "backward_prep", "gripper": "none"}
]
```

Gripper actions: `"none"`, `"open"`, `"close"`.  
All position names must exist in `positions.xml`.

## Positions File

`positions.xml` at the repo root stores all named joint positions.  
Record new positions by hand-dragging the arm with torque off:

```bash
./build/record_positions positions.xml
```

Then verify each point physically before running the full program:

```bash
./build/test_named_positions positions.xml s0_prep s0_grasp
```

## Low-level Tuning

Hardware constants in `include/config.hpp` (requires recompile):

```cpp
constexpr int ARM_LOOP_HZ                = 100;
constexpr int32_t JOINT_PROFILE_VELOCITY = 80;
constexpr int32_t JOINT_PROFILE_ACCELERATION = 20;
constexpr int32_t JOINT_POSITION_P_GAIN  = 350;
constexpr float   JOINT_GOAL_WRITE_DEADBAND = 0.002f;
```

## Environment Check

Run this script before first use to verify all dependencies and devices:

```bash
./test/check_meta_dart_environment.sh

# Optional: also trigger a build check and motor scan
RUN_BUILD_CHECK=1 RUN_MOTOR_SCAN=1 ./test/check_meta_dart_environment.sh
```

## Troubleshooting

**Arm does not connect** — check `/dev/ttyUSB0` exists, your user is in `dialout`, motor IDs are 11–15.  
Run `./build/clear_errors` then `./build/scan_motors`.

**YOLO detects but no pickup triggers** — run `./build/test_vision_camera` and check that the dart x/y falls inside the slot bounding box in the config. Increase `half_width/half_height` if needed.

**Arm shakes** — reduce `joint_cruise_speed_rad_s` or `JOINT_POSITION_P_GAIN`. Check trajectory points are not too close to joint limits.

**`loading` timeout in logs** — increase `loading_release_tolerance_rad` or `move_timeout_s`. Use `test_named_positions` to confirm `loading` is reachable.
