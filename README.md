# Meta-Dart

Meta-Dart is a C++/Python dart pickup pipeline for an OpenManipulator-X style
Dynamixel arm.  The current main flow is:

1. Read frames continuously from the 2K USB camera.
2. Run YOLO OBB inference through ONNX Runtime.
3. Watch the detected dart position.
4. If the dart stays inside one configured bounding box for 2 seconds, trigger
   that box's pickup trajectory.
5. Ignore camera updates while the arm is moving.
6. Return home, wait 5 seconds, then resume searching for the next dart.

The main executable is `build/loader`.

## Current Hardware Configuration

Configured in `include/config.hpp`:

```cpp
constexpr const char* DXL_PORT = "/dev/ttyUSB0";
constexpr int CAMERA_INDEX = 2;
constexpr const char* ONNX_MODEL_PATH = "models/best.onnx";
constexpr const char* POSITIONS_XML_PATH = "positions.xml";
```

Expected devices:

```text
/dev/ttyUSB0    Dynamixel/U2D2 serial port
/dev/video2     2K USB Camera capture node
```

From the latest environment check:

```text
/dev/video2 = 2K USB Camera: 2K USB Camera, capture node
/dev/video0 = Integrated laptop camera
```

## Required System Packages

Install these on Ubuntu:

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  pkg-config \
  libopencv-dev \
  v4l-utils \
  usbutils \
  ffmpeg
```

The current verified versions were:

```text
g++ 11.4.0
cmake 3.22.1
pkg-config 0.29.2
OpenCV 4.5.4
```

## Conda Environment

The Python YOLO pipeline uses the `ece445` conda environment.

```bash
conda create -n ece445 python=3.10 -y
conda activate ece445

cd /home/xiaoman/Code/Meta-Dart
python -m pip install --upgrade pip setuptools wheel
python -m pip install --index-url https://download.pytorch.org/whl/cpu torch torchvision
python -m pip install -r python/yolo/requirements.txt
python -m pip install onnx onnxruntime
```

`python/yolo/requirements.txt` currently contains:

```text
ultralytics>=8,<9
```

Verify:

```bash
conda activate ece445
python - <<'PY'
import torch
import cv2
import ultralytics
import onnxruntime

print("torch:", torch.__version__)
print("cuda available:", torch.cuda.is_available())
print("cv2:", cv2.__version__)
print("ultralytics:", ultralytics.__version__)
print("onnxruntime:", onnxruntime.__version__)
PY
```

## ONNX Runtime C++

CMake expects ONNX Runtime here:

```text
~/Applications/onnxruntime
```

The current setup uses:

```text
~/Applications/onnxruntime -> ~/Applications/onnxruntime-linux-x64-1.20.1
```

Required files:

```text
~/Applications/onnxruntime/include/onnxruntime_cxx_api.h
~/Applications/onnxruntime/lib/libonnxruntime.so
```

## Permissions

The user should be in at least these groups:

```text
dialout
video
plugdev
```

Check current shell permissions:

```bash
id -nG
```

For the arm test, the current shell must include `dialout`.

If `groups xiaoman` shows `dialout` but `id -nG` does not, refresh the shell:

```bash
newgrp dialout
```

or log out and back in.

## Project Files

Required runtime files:

```text
models/best.onnx
positions.xml
```

`positions.xml` should exist at the repo root because `config.hpp` uses:

```cpp
constexpr const char* POSITIONS_XML_PATH = "positions.xml";
```

If needed:

```bash
cd /home/xiaoman/Code/Meta-Dart
cp data/positions.xml positions.xml
```

The current named positions are:

```text
home
forward_prep
backward_prep
s0_prep
s0_grasp
s0_pulled_out
loading
```

## Build

Configure and build:

```bash
cd /home/xiaoman/Code/Meta-Dart
cmake -S . -B build
cmake --build build -j
```

Build selected targets:

```bash
cmake --build build -j --target loader
cmake --build build -j --target test_vision_camera
cmake --build build -j --target test_gripper_trajectory
```

Main outputs:

```text
build/loader
build/test_vision_camera
build/test_trajectory
build/test_gripper_trajectory
build/test_arm
build/scan_motors
build/record_positions
build/calibrate_gripper
build/clear_errors
```

## Full Environment Check

Run:

```bash
cd /home/xiaoman/Code/Meta-Dart
./test/check_meta_dart_environment.sh
```

This checks:

```text
C++ tools
OpenCV
ONNX Runtime C++
models/best.onnx
positions.xml
Dynamixel source files
build outputs
user groups
/dev/ttyUSB0
/dev/video2
ece445 Python imports
OpenCV camera read
```

Optional build check:

```bash
RUN_BUILD_CHECK=1 ./test/check_meta_dart_environment.sh
```

Optional Dynamixel bus scan:

```bash
RUN_MOTOR_SCAN=1 ./test/check_meta_dart_environment.sh
```

The motor scan does not run a trajectory, but it does access the serial bus.

## Camera And YOLO Tests

### Safe C++ Camera + ONNX Test

This test does not start the arm and will not move the robot:

```bash
cd /home/xiaoman/Code/Meta-Dart
./build/test_vision_camera
```

Run for 10 seconds:

```bash
./build/test_vision_camera 10
```

The test:

```text
opens CAMERA_INDEX=2 (/dev/video2)
loads models/best.onnx
runs VisionModule
shows the OpenCV window "Vision"
prints valid detection frame counts once per second
```

If detections flicker at the default threshold, run with a lower threshold:

```bash
META_DART_CONF_THRESHOLD=0.25 ./build/test_vision_camera
META_DART_CONF_THRESHOLD=0.15 ./build/test_vision_camera
```

Good output looks like:

```text
[VisionTest] valid frames 25/26: last x=... y=... z=0.000 m
```

### Python YOLO Test

Using Ultralytics:

```bash
conda activate ece445
cd /home/xiaoman/Code/Meta-Dart/python/yolo

python play.py \
  --weights ../../models/best.onnx \
  --source 2 \
  --device cpu \
  --conf 0.25 \
  --name dart_camera_test
```

If Ultralytics cannot load the ONNX file directly, use a compatible `.pt`
checkpoint or use the C++ `test_vision_camera` path.

## Arm Tests

### Single Position Test

Start with one safe target:

```bash
cd /home/xiaoman/Code/Meta-Dart
./build/test_trajectory positions.xml home
```

### Short Trajectory

```bash
./build/test_trajectory positions.xml home forward_prep home
```

### Full Joint Trajectory

```bash
./build/test_trajectory
```

Default trajectory:

```text
home -> forward_prep -> backward_prep
-> s0_prep -> s0_grasp -> s0_pulled_out
-> backward_prep -> forward_prep -> loading -> home
```

### Gripper Trajectory Test

This follows the same fixed trajectory and includes gripper actions:

```bash
./build/test_gripper_trajectory positions.xml
```

Behavior:

```text
s0_prep reached: open gripper
s0_grasp reached: close gripper
after s0_grasp: keep gripper closed
```

## Main Autonomous Pipeline

Run:

```bash
cd /home/xiaoman/Code/Meta-Dart
META_DART_CONF_THRESHOLD=0.25 ./build/loader
```

Current main logic:

```text
1. Start camera and vision threads.
2. Start arm thread.
3. Decision thread homes the arm.
4. While searching, watch Detection from VisionModule.
5. If dart XY stays inside one configured bbox for 2 seconds, trigger that
   zone's pickup trajectory.
6. During pickup, ignore camera detections.
7. Execute the selected trajectory table. Normal intermediate waypoints are
   fly-through points; the arm only stops at gripper action points and the
   final home pose. The current tables are seeded with:
   home -> forward_prep -> backward_prep
   -> s0_prep: open gripper
   -> s0_grasp: close gripper
   -> s0_pulled_out
   -> backward_prep -> forward_prep -> loading -> home
8. Wait 5 seconds.
9. Search again for the next dart.
```

Current decision constants are in `src/decision.cpp`:

```cpp
STABLE_DART_TIME = 2s
DEFAULT_ZONE_HALF_W_M = 0.020f
DEFAULT_ZONE_HALF_H_M = 0.020f
JOINT_CRUISE_SPEED_RAD_S = 0.60f
MIN_SEGMENT_DURATION = 1.5f
MAX_SEGMENT_DURATION = 7.0f
FLYTHROUGH_JOINT_THRESHOLD = 0.12f
RESET_COOLDOWN = 5s
```

Arm smoothness constants are in `include/config.hpp`:

```cpp
JOINT_PROFILE_VELOCITY = 80
JOINT_PROFILE_ACCELERATION = 20
JOINT_POSITION_P_GAIN = 350
JOINT_GOAL_WRITE_DEADBAND = 0.005f
```

The arm uses quintic S-curve interpolation in `src/arm.cpp`.  If motion still
shakes, first try a slower trajectory duration, then reduce `JOINT_POSITION_P_GAIN`
slightly.  If motion becomes too sluggish or misses targets, increase
`JOINT_PROFILE_VELOCITY` before increasing acceleration.

Trajectory segments use distance-based timing: the maximum joint delta between
the current pose and the next pose is divided by `JOINT_CRUISE_SPEED_RAD_S`,
then clamped between `MIN_SEGMENT_DURATION` and `MAX_SEGMENT_DURATION`.  This
keeps long waypoint jumps from moving much faster than short jumps.

For continuous motion, intermediate waypoints without gripper actions are
treated as fly-through points.  The decision layer switches to the next segment
once the arm is within `FLYTHROUGH_JOINT_THRESHOLD` of that waypoint, instead
of waiting for a full stop.

The four configurable zone centers are also in `src/decision.cpp`:

```cpp
DART_ZONES = {
    {"slot_0", -0.058f,  0.027f, ... TrajectoryId::Slot0},
    {"slot_1", -0.145f, -0.008f, ... TrajectoryId::Slot1},
    {"slot_2", -0.111f, -0.010f, ... TrajectoryId::Slot2},
    {"slot_3",  0.109f, -0.004f, ... TrajectoryId::Slot3},
};
```

Each slot has its own editable trajectory table:

```cpp
TRAJECTORY_SLOT_0
TRAJECTORY_SLOT_1
TRAJECTORY_SLOT_2
TRAJECTORY_SLOT_3
```

Edit those tables to change the exact position sequence and gripper actions for
each fixed dart location.

## Safety Checklist

Before running anything that moves the arm:

```text
1. Confirm /dev/ttyUSB0 exists.
2. Confirm current shell has dialout.
3. Confirm positions.xml matches the physical workspace.
4. Clear the arm workspace.
5. Keep power cutoff within reach.
6. Test one position before full trajectory.
7. Test camera detection with test_vision_camera before running loader.
```

Suggested order:

```bash
./test/check_meta_dart_environment.sh
./build/test_vision_camera 10
./build/test_trajectory positions.xml home
./build/test_gripper_trajectory positions.xml
META_DART_CONF_THRESHOLD=0.25 ./build/loader
```
