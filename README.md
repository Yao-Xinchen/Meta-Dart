# Meta-Dart

Meta-Dart 是一个用于“视觉检测飞镖 + Dynamixel 机械臂抓取 + 按 slot 自动分拣/装载”的 C++17 主程序工程。当前主入口是 `build/loader`，它会同时启动相机、YOLO/ONNX 视觉、机械臂控制、决策状态机和键盘急停监听。

这份 README 按当前 repo 的实际代码整理，重点回答三个问题：

1. 这套系统怎么装、怎么检测、怎么运行。
2. 主程序现在到底怎么决策和抓取。
3. 想调 slot、轨迹、速度、扰动敏感度、窗口大小时应该改哪里。

## 目录

- [快速上手](#快速上手)
- [当前硬件假设](#当前硬件假设)
- [系统依赖](#系统依赖)
- [Conda 环境](#conda-环境)
- [ONNX Runtime C++](#onnx-runtime-c)
- [权限](#权限)
- [Repo 结构](#repo-结构)
- [编译目标](#编译目标)
- [全面环境检测](#全面环境检测)
- [视觉测试](#视觉测试)
- [主程序运行](#主程序运行)
- [主程序线程和模块](#主程序线程和模块)
- [决策状态机](#决策状态机)
- [Disturb 模式和 Replay 模式](#disturb-模式和-replay-模式)
- [Trajectory 配置](#trajectory-配置)
- [位姿文件 positions.xml](#位姿文件-positionsxml)
- [记录新位姿](#记录新位姿)
- [查看每个点现实中在哪里](#查看每个点现实中在哪里)
- [夹抓轨迹测试](#夹抓轨迹测试)
- [常用调参地图](#常用调参地图)
- [终端日志](#终端日志)
- [安全建议](#安全建议)
- [常见问题](#常见问题)
- [推荐开发流程](#推荐开发流程)
- [一句话版](#一句话版)

## 快速上手

从 repo 根目录开始：

```bash
cd /home/xiaoman/Code/Meta-Dart
```

检查环境，不会移动机械臂：

```bash
./test/check_meta_dart_environment.sh
```

编译：

```bash
cmake -S . -B build
cmake --build build -j
```

只测试相机和 YOLO，不启动机械臂：

```bash
./build/test_vision_camera
```

运行主程序，默认使用 `disturb` 模式：

```bash
./build/loader config/loader_config.json
```

运行主程序，但关闭扰动处理，改成纯轨迹 replay：

```bash
./build/loader config/loader_config.json --replay
```

启动后：

- 按终端里的 `q`：立刻停止当前动作，快速回到 `home`，然后断开程序。
- 在 OpenCV 视觉窗口里按 `q`：同样触发安全返回。
- 按 `Ctrl-C`：正常停止程序，不额外执行快速回 home。

## 当前硬件假设

硬件常量在 `include/config.hpp`：

```cpp
constexpr const char* DXL_PORT = "/dev/ttyUSB0";
constexpr int CAMERA_INDEX = 2;
constexpr const char* POSITIONS_XML_PATH = "positions.xml";
constexpr const char* ONNX_MODEL_PATH = "models/best.onnx";
```

当前默认设备：

```text
/dev/ttyUSB0    Dynamixel/U2D2 串口
/dev/video2     2K USB Camera capture node
```

OpenCV 视觉窗口：

```cpp
constexpr const char* VISION_WINDOW_NAME = "Meta-Dart Vision";
constexpr int VISION_WINDOW_W = 1800;
constexpr int VISION_WINDOW_H = 1000;
```

如果你想把视觉窗口改得更大或更小，改上面两个尺寸后重新编译。

## 系统依赖

Ubuntu 上建议安装：

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

需要的核心组件：

- `g++`，支持 C++17。
- `cmake`。
- `pkg-config`。
- OpenCV 开发包，例如 `libopencv-dev`。
- ONNX Runtime C++ 包。
- DynamixelSDK 和 dynamixel-workbench 子模块。
- `models/best.onnx`。
- repo 根目录下的 `positions.xml`。

## Conda 环境

Python/YOLO 侧推荐使用 `ece445`：

```bash
conda create -n ece445 python=3.10 -y
conda activate ece445

cd /home/xiaoman/Code/Meta-Dart
python -m pip install --upgrade pip setuptools wheel
python -m pip install --index-url https://download.pytorch.org/whl/cpu torch torchvision
python -m pip install -r python/yolo/requirements.txt
python -m pip install onnx onnxruntime opencv-python
```

快速验证：

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

C++ 主程序并不依赖 conda 运行，但 Python 训练、导出、YOLO 工具和环境检测脚本会检查 `ece445`。

## ONNX Runtime C++

CMake 默认在这里找 ONNX Runtime：

```text
~/Applications/onnxruntime
```

当前建议使用 v1.20.1，并让这个路径指向解压后的目录：

```text
~/Applications/onnxruntime -> ~/Applications/onnxruntime-linux-x64-1.20.1
```

必须存在：

```text
~/Applications/onnxruntime/include/onnxruntime_cxx_api.h
~/Applications/onnxruntime/lib/libonnxruntime.so
```

如果不想用默认路径，可以在 configure 时覆盖：

```bash
cmake -S . -B build -DORT_ROOT=/your/path/to/onnxruntime
```

## 权限

机械臂串口需要 `dialout`，相机通常需要 `video`：

```bash
groups xiaoman
id -nG
```

如果 `groups xiaoman` 里有 `dialout`，但当前 shell 的 `id -nG` 没有，说明当前 shell 还没刷新组权限。可以重新登录，或者临时：

```bash
newgrp dialout
```

检查设备：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
ls -l /dev/video*
v4l2-ctl --list-devices
```

## Repo 结构

```text
Meta-Dart/
  CMakeLists.txt
  README.md
  config/
    loader_config.json          # 主程序高参数和四个 slot/trajectory
  include/
    config.hpp                  # 硬件常量、相机编号、窗口大小、控制频率
    runtime_config.hpp          # JSON config 对应的数据结构
    decision.hpp                # 决策模块接口
    vision.hpp                  # YOLO/ONNX 视觉模块接口
    arm.hpp                     # Dynamixel 机械臂模块接口
    log.hpp                     # 彩色终端日志
  src/
    main.cpp                    # loader 入口、CLI、q 快速回 home
    camera.cpp                  # 相机采集线程
    vision.cpp                  # ONNX Runtime 推理和 OpenCV overlay
    arm.cpp                     # Dynamixel 控制线程
    decision.cpp                # 搜索、抓取、扰动、重规划状态机
    runtime_config.cpp          # JSON config 解析
  test/
    check_meta_dart_environment.sh
    test_vision_camera.cpp
    test_gripper_trajectory.cpp
    test_named_positions.cpp
    record_positions.cpp
    scan_motors.cpp
    calibrate_gripper.cpp
    clear_errors.cpp
  models/
    best.onnx
  positions.xml                 # 主程序运行时读取的位姿文件
  data/positions.xml            # 初始/备份位姿
  python/yolo/                  # Python 训练、预测、导出相关工具
  thirdparty/
    DynamixelSDK/
    dynamixel-workbench/
```

## 编译目标

`CMakeLists.txt` 当前会生成：

```text
loader                    主程序
test_vision_camera         相机 + YOLO/ONNX 测试，不动机械臂
test_arm                   机械臂基础测试
test_recall                记录/回放相关测试
test_trajectory            轨迹测试
test_gripper_trajectory    带夹抓动作的轨迹测试
test_named_positions       逐个查看 named position 在现实里的位置
record_positions           手动拖动机械臂并记录 named position
scan_motors                扫描 Dynamixel 电机
calibrate_gripper          夹抓标定
clear_errors               清除电机错误
```

常用编译命令：

```bash
cmake --build build -j --target loader
cmake --build build -j --target test_vision_camera
cmake --build build -j --target test_gripper_trajectory
cmake --build build -j --target record_positions
```

## 全面环境检测

推荐每次大改硬件或环境后先跑：

```bash
cd /home/xiaoman/Code/Meta-Dart
./test/check_meta_dart_environment.sh
```

它会检查：

- repo 根目录和 `include/config.hpp`。
- `positions.xml` 和 `models/best.onnx`。
- `g++`、C++17、CMake、pkg-config、OpenCV。
- ONNX Runtime C++ 路径。
- DynamixelSDK 和 dynamixel-workbench 源码。
- build 输出是否存在。
- 当前用户组和串口权限。
- `/dev/ttyUSB0`。
- `/dev/video2`。
- `ece445` conda 环境里的 Python 包。
- OpenCV 是否能打开并读取当前相机。

可选项：

```bash
RUN_BUILD_CHECK=1 ./test/check_meta_dart_environment.sh
RUN_MOTOR_SCAN=1 ./test/check_meta_dart_environment.sh
CONDA_ENV_NAME=ece445 ./test/check_meta_dart_environment.sh
```

`RUN_MOTOR_SCAN=1` 会访问 Dynamixel 总线，但不会执行轨迹。

## 视觉测试

安全测试，只开相机和 YOLO，不启动机械臂：

```bash
cd /home/xiaoman/Code/Meta-Dart
./build/test_vision_camera
```

限制运行秒数：

```bash
./build/test_vision_camera 10
```

降低检测阈值：

```bash
META_DART_CONF_THRESHOLD=0.15 ./build/test_vision_camera
```

正常输出类似：

```text
[VisionTest] valid frames 24/25: last x=-0.145 m  y=-0.008 m  z=0.000 m  total_updates=277
```

`x/y/z` 是视觉模块估算出来的飞镖位置。主程序里的 slot bounding box 就是用这个 `x/y` 来判断飞镖落在哪个固定点位。

## 主程序运行

默认运行：

```bash
./build/loader config/loader_config.json
```

检查 config 是否能解析，不启动硬件：

```bash
./build/loader --check-config config/loader_config.json
```

指定模式：

```bash
./build/loader config/loader_config.json --mode disturb
./build/loader config/loader_config.json --mode replay
./build/loader config/loader_config.json --disturb
./build/loader config/loader_config.json --replay
```

也可以用环境变量指定 config：

```bash
META_DART_CONFIG=config/loader_config.json ./build/loader
```

如果只是想临时改 YOLO 置信度，环境变量会覆盖 JSON 里的 `vision.confidence_threshold`：

```bash
META_DART_CONF_THRESHOLD=0.25 ./build/loader config/loader_config.json
```

如果终端颜色影响记录日志：

```bash
NO_COLOR=1 ./build/loader config/loader_config.json
```

## 主程序线程和模块

当前不是单纯两个 thread，而是模块化多线程：

```text
CameraModule
  从 /dev/video2 读取图像，写入 frame buffer

VisionModule
  从 frame buffer 取图，运行 ONNX/YOLO，写入 detection buffer

ArmModule
  连接 Dynamixel，持续读取当前 joint，按目标轨迹输出控制

DecisionModule
  读取 detection，决定 SEARCHING / CATCH / DISTURB / ASSEMBLING

Keyboard thread
  监听终端 q，一旦按下就请求停机并快速回 home
```

数据流：

```text
/dev/video2
  -> CameraModule
  -> VisionModule
  -> Detection(x, y, z, valid)
  -> DecisionModule
  -> ArmModule
  -> Dynamixel joints + gripper
```

## 决策状态机

当前主流程：

1. 启动后进入 homing/准备阶段。
2. 机械臂执行 `home -> forward_prep -> backward_prep`。
3. 停在 `backward_prep` 做 SEARCHING。
4. 视觉持续读取飞镖位置。
5. 如果飞镖在某个 slot 的 bounding box 内稳定超过 `stable_dart_time_s`，触发该 slot 对应 trajectory。
6. 执行 `sN_prep -> sN_grasp -> sN_pulled_out -> backward_prep -> forward_prep -> loading -> high_prep -> backward_prep`。
7. 在 `sN_prep` 打开夹抓，在 `sN_grasp` 闭合夹抓，在 `loading` 打开夹抓。
8. `loading` 后经过 `high_prep`，再丝滑回到 `backward_prep` 等下一次搜索。

当前四个 slot 在 `config/loader_config.json`：

```json
"zones": [
  {
    "name": "slot_0",
    "center_x": -0.112,
    "center_y": -0.020,
    "half_width": 0.020,
    "half_height": 0.020,
    "trajectory": "slot_0",
    "pickup_half_width": 0.035,
    "pickup_half_height": 0.050
  },
  {
    "name": "slot_1",
    "center_x": -0.062,
    "center_y": 0.029,
    "half_width": 0.030,
    "half_height": 0.030,
    "trajectory": "slot_1",
    "pickup_half_width": 0.035,
    "pickup_half_height": 0.050
  },
  {
    "name": "slot_2",
    "center_x": 0.068,
    "center_y": 0.026,
    "half_width": 0.020,
    "half_height": 0.020,
    "trajectory": "slot_2",
    "pickup_half_width": 0.035,
    "pickup_half_height": 0.050
  },
  {
    "name": "slot_3",
    "center_x": 0.105,
    "center_y": -0.008,
    "half_width": 0.020,
    "half_height": 0.020,
    "trajectory": "slot_3",
    "pickup_half_width": 0.055,
    "pickup_half_height": 0.075,
    "pickup_outside_grace_s": 0.40
  }
]
```

`center_x/center_y` 是 slot 中心坐标，`half_width/half_height` 是 SEARCHING 阶段判断飞镖属于该 slot 的框。`pickup_half_width/pickup_half_height` 是抓取过程中使用的固定中心守卫框，通常比 searching 框更宽松。

## Disturb 模式和 Replay 模式

当前有两种运行逻辑。

### Disturb 模式

默认模式：

```bash
./build/loader config/loader_config.json --disturb
```

或者 config 中：

```json
"pickup_mode": "disturb",
"monitor_dart_motion_during_pickup": true
```

行为：

- SEARCHING 阶段，飞镖在某个 slot 稳定后触发抓取。
- 从 `backward_prep` 移动到 `sN_prep` 这段会更谨慎，视觉仍然监控飞镖。
- 如果飞镖在到达 prep 前离开当前 slot 的固定中心守卫框超过 grace time，机械臂立即暂停在当前 joint。
- 进入 DISTURB relocation 观察。
- 如果 4 秒窗口内，飞镖在另一个 slot 内稳定超过 1 秒，则机械臂先过 `backward_prep`，然后直接抓新 slot，不再额外等待 SEARCHING 的稳定时间。
- 如果没有稳定 relocation，则回 `backward_prep`，重新 SEARCHING。
- 一旦成功到达 `sN_prep`，后续抓取不再受图像干扰，因为夹抓伸过去后 YOLO 框容易形变或短暂消失。

### Replay 模式

关闭扰动处理：

```bash
./build/loader config/loader_config.json --replay
```

或者 config 中：

```json
"pickup_mode": "replay",
"monitor_dart_motion_during_pickup": false
```

行为：

- SEARCHING 阶段仍然根据相机判断 slot。
- 一旦触发某个 slot，就直接 replay 对应 trajectory。
- 抓取过程中不因为视觉检测变化而暂停或重规划。

如果 YOLO 在机械臂靠近时经常形变、丢框，或者你想先验证纯机械轨迹，推荐用 replay 模式。

## Trajectory 配置

所有 slot 的轨迹都集中在 `config/loader_config.json` 的 `decision.trajectories`。例如 `slot_2`：

```json
"slot_2": [
  {"position": "s2_prep", "gripper": "open"},
  {"position": "s2_grasp", "gripper": "close"},
  {"position": "s2_pulled_out", "gripper": "none"},
  {"position": "backward_prep", "gripper": "none"},
  {"position": "forward_prep", "gripper": "none"},
  {"position": "loading", "gripper": "open"},
  {"position": "high_prep", "gripper": "none"},
  {"position": "backward_prep", "gripper": "none"}
]
```

可用的夹抓动作：

```text
none     不改变夹抓
open     打开夹抓
close    闭合夹抓
```

如果你想改某个 slot 的动作顺序，只改对应数组即可。比如想让 `slot_3` 多经过一个中间点，就在 `slot_3` 的数组里插入：

```json
{"position": "your_new_point", "gripper": "none"}
```

前提是 `positions.xml` 里已经有 `your_new_point`。

## 位姿文件 positions.xml

主程序默认读取 repo 根目录：

```text
positions.xml
```

当前 repo 根目录的 `positions.xml` 包含：

```text
home
forward_prep
backward_prep
loading
high_prep
s0_prep
s0_grasp
s0_pulled_out
s1_prep
s1_grasp
s1_pulled_out
s2_prep
s2_grasp
s2_pulled_out
s3_prep
s3_grasp
s3_pulled_out
```

`data/positions.xml` 是旧的/基础备份，不一定包含最新 slot。主程序实际看的是根目录 `positions.xml`，这点很重要。

## 记录新位姿

`record_positions` 是手动拖动机械臂记录 named position 的工具。它不会打开 torque，机械臂可以用手移动。

```bash
cd /home/xiaoman/Code/Meta-Dart
./build/record_positions positions.xml
```

使用方式：

1. 把机械臂手动移动到目标位置。
2. 按 Enter 捕获当前四个 joint。
3. 输入名字，例如 `s3_prep`。
4. 程序会立刻保存到 `positions.xml`。
5. 空名字表示丢弃本次捕获。
6. `Ctrl-D` 结束。

如果你发现“刚记录的位置不见了”，通常是因为你写到了 `data/positions.xml` 或在别的工作目录运行。主程序默认找 repo 根目录的 `positions.xml`。

## 查看每个点现实中在哪里

用 `test_named_positions`，它会一个点一个点移动，并在每个点暂停让你观察：

```bash
cd /home/xiaoman/Code/Meta-Dart
./build/test_named_positions positions.xml home forward_prep backward_prep s2_prep s2_grasp loading high_prep
```

如果不传具体点名，会按默认 s0 测试路径走一遍：

```bash
./build/test_named_positions
```

这是理解每个 named position 在现实中位置的最直接工具。

## 夹抓轨迹测试

带夹抓动作的单独测试：

```bash
cd /home/xiaoman/Code/Meta-Dart
./build/test_gripper_trajectory
```

这个目标主要用于验证机械轨迹和夹抓开合，不包含主程序里的视觉 slot 决策和 disturb/replay 逻辑。如果你已经把轨迹改到 JSON，主程序以 JSON 为准；测试文件里自己的默认轨迹可能需要单独同步。

## 常用调参地图

主调参文件：

```text
config/loader_config.json
```

硬件常量、相机编号、窗口大小、底层控制频率：

```text
include/config.hpp
```

### 视觉阈值

```json
"vision": {
  "confidence_threshold": 0.2
}
```

也可以运行时临时覆盖：

```bash
META_DART_CONF_THRESHOLD=0.25 ./build/loader config/loader_config.json
```

### 搜索稳定时间

```json
"stable_dart_time_s": 1.0
```

越大越稳，触发越慢。越小反应更快，但更容易误触发。

### SEARCHING 阶段 bbox

每个 slot：

```json
"center_x": -0.062,
"center_y": 0.029,
"half_width": 0.030,
"half_height": 0.030
```

如果飞镖明明在 slot 里却一直 reset，可以适当加大 `half_width/half_height`，或者加大：

```json
"search_stability_grace_s": 0.45,
"search_stability_bbox_margin_m": 0.005
```

### 抓取过程固定中心守卫

每个 slot 可以单独设置：

```json
"pickup_half_width": 0.035,
"pickup_half_height": 0.050,
"pickup_outside_grace_s": 0.40
```

含义：

- `pickup_half_width/pickup_half_height`：抓取过程中固定中心守卫框大小。
- `pickup_outside_grace_s`：允许 YOLO 短暂丢框或跳出框的时间。

slot_3 目前更宽松，因为夹抓靠近时它的 YOLO 框更容易压缩、前移或短暂消失：

```json
"pickup_half_width": 0.055,
"pickup_half_height": 0.075,
"pickup_outside_grace_s": 0.40
```

如果 slot_3 仍然太敏感，优先调大 `pickup_outside_grace_s`，再调大 `pickup_half_width/pickup_half_height`。

### Disturb relocation

```json
"dart_motion_hold_s": 5.0,
"relocation_window_s": 4.0,
"relocation_stable_time_s": 1.0,
"relocation_outside_grace_s": 0.45,
"relocation_bbox_margin_m": 0.010,
"relocation_accept_same_slot": false
```

含义：

- `dart_motion_hold_s`：检测到扰动后，机械臂暂停保持的时间。
- `relocation_window_s`：在这段时间内观察飞镖是否进入新 slot。
- `relocation_stable_time_s`：新 slot 内稳定多久才接受 relocation。
- `relocation_accept_same_slot`：是否接受“还是同一个 slot”的 relocation。

### 速度和丝滑度

```json
"min_segment_duration_s": 2.0,
"max_segment_duration_s": 9.0,
"joint_cruise_speed_rad_s": 0.45,
"fast_motion_speedup": 1.6,
"flythrough_joint_threshold_rad": 0.12,
"waypoint_settle_ms": 300.0
```

调速建议：

- 整体太慢：增大 `joint_cruise_speed_rad_s` 或 `fast_motion_speedup`。
- 普通轨迹太慢，但想保留从 `backward_prep` 到 `sN_prep` 的谨慎应变：优先增大 `fast_motion_speedup`。
- 中间停顿明显：减小 `waypoint_settle_ms`，并确认非关键点可以 fly-through。
- 抖动明显：不要只追求加速，先降低速度或检查 `include/config.hpp` 里的底层 profile/gain。

底层控制相关：

```cpp
constexpr int ARM_LOOP_HZ = 100;
constexpr int32_t JOINT_PROFILE_VELOCITY = 80;
constexpr int32_t JOINT_PROFILE_ACCELERATION = 20;
constexpr int32_t JOINT_POSITION_P_GAIN = 350;
constexpr float JOINT_GOAL_WRITE_DEADBAND = 0.002f;
```

这些在 `include/config.hpp`。改完需要重新编译。

### loading 到达判定和松爪

```json
"loading_release_tolerance_rad": 0.25,
"move_timeout_s": 15.0,
"gripper_settle_s": 1.0
```

当前逻辑是：到 `loading` 后打开夹抓。如果 `loading` 没有完全到达，但误差在 `loading_release_tolerance_rad` 内，也会打开夹抓，避免因为小误差一直卡死。

如果日志经常显示 loading timeout：

- 先用 `test_named_positions` 单独检查 `loading` 是否合理。
- 适当增大 `loading_release_tolerance_rad`。
- 适当增大 `move_timeout_s`。
- 检查该姿态是否太接近机械臂极限或电机负载过大。

### q 快速回 home

```json
"emergency_home_duration_s": 1.2
```

按 `q` 后会停止当前动作，发送快速回 `home`，等待到达或超时，然后断开。

如果你觉得回 home 太猛，调大它。如果你需要更快，谨慎调小，但不要小于机械臂能稳定执行的时间。

## 终端日志

当前日志是彩色高层状态标签：

```text
【SYSTEM】     启动、配置、完成
【SEARCHING】  等待飞镖、候选 slot、稳定检测
【ASSEMBLING】 机械臂移动、准备姿态、回安全姿态
【CATCH】      触发抓取、开合夹抓、抓取完成
【DISTURB】    飞镖移动、暂停、relocation、危险回退
【WARN】       非致命警告
【ERROR】      错误
```

典型成功流程：

```text
【SYSTEM】 Meta-Dart loader starting
【SYSTEM】 loaded runtime config: config/loader_config.json
【ASSEMBLING】 homing
【SEARCHING】 preparing search pose: home -> forward_prep -> backward_prep
【SEARCHING】 waiting at backward_prep
【SEARCHING】 slot_2 candidate; waiting 1.0s
【CATCH】 slot_2 stable; triggering slot_2
【CATCH】 start slot_2 for slot_2 (disturb mode)
【ASSEMBLING】 guarded move to s2_prep ...
【CATCH】 prep reached: s2_prep; guard off
【CATCH】 open gripper at s2_prep
【CATCH】 close gripper at s2_grasp
【CATCH】 loading reached; opening gripper
【CATCH】 pickup complete
【SEARCHING】 active
```

## 安全建议

这不是仿真。运行任何会启动 `ArmModule` 的程序前，请确认：

- 机械臂工作空间内没有手、线、工具或挡板。
- 飞镖、夹抓和装载位置不会互相卡住。
- `/dev/ttyUSB0` 对应的是正确的 Dynamixel/U2D2。
- `positions.xml` 是你想要的那份文件。
- 第一次测试新轨迹时，用 `test_named_positions` 单点确认，再跑完整轨迹。
- 不确定时先用 `--replay` 验证机械路径，再打开 `--disturb`。

紧急停止习惯：

- 想让程序快速回 home 并退出：按 `q`。
- 想直接结束程序：按 `Ctrl-C`。
- 如果硬件已经报错或卡死，先断电排查，不要连续强跑。

## 常见问题

### 找不到 config

错误：

```text
failed to load runtime config 'config/my_loader_config.json': cannot open ...
```

解决：

```bash
cd /home/xiaoman/Code/Meta-Dart
./build/loader config/loader_config.json
```

或者传绝对路径：

```bash
./build/loader /home/xiaoman/Code/Meta-Dart/config/loader_config.json
```

### 找不到 positions.xml

主程序默认找 repo 根目录：

```text
/home/xiaoman/Code/Meta-Dart/positions.xml
```

如果缺失：

```bash
cp data/positions.xml positions.xml
```

注意：`data/positions.xml` 不一定包含最新 slot。记录新点时建议直接写根目录：

```bash
./build/record_positions positions.xml
```

### 相机是 video 几

当前配置：

```text
CAMERA_INDEX = 2
/dev/video2 = 2K USB Camera
```

检查：

```bash
v4l2-ctl --list-devices
```

如果 USB 相机编号变了，改 `include/config.hpp` 的 `CAMERA_INDEX`，然后重新编译。

### YOLO 能检测但主程序不触发

先跑：

```bash
./build/test_vision_camera
```

看飞镖稳定时的 `x/y` 是否落入 `config/loader_config.json` 的 slot bbox。

如果总是 `Dart left zones; reset search stability`：

- 增大该 slot 的 `half_width/half_height`。
- 增大 `search_stability_grace_s`。
- 降低或提高 `vision.confidence_threshold`，看哪个更稳定。

### 抓取时误判 dart moved

如果是夹抓靠近导致 YOLO 框形变或短暂消失：

- 优先使用 `--replay` 验证机械路径。
- 对应 slot 增大 `pickup_half_width/pickup_half_height`。
- 对应 slot 增大 `pickup_outside_grace_s`。
- 如果问题主要发生在 `sN_prep` 之后，当前代码已经在 prep reached 后关闭 guard，理论上不会继续受视觉影响。

### 机械臂很抖

先检查是否是轨迹太快、关键帧太稀或目标点太接近极限。可调：

```json
"joint_cruise_speed_rad_s": 0.45,
"fast_motion_speedup": 1.6,
"waypoint_settle_ms": 300.0
```

底层再调：

```cpp
JOINT_PROFILE_VELOCITY
JOINT_PROFILE_ACCELERATION
JOINT_POSITION_P_GAIN
JOINT_GOAL_WRITE_DEADBAND
```

经验上，抖动不一定靠提高 control frequency 解决。很多时候是 setpoint 太密、速度/加速度太激进、P gain 太高或目标点附近来回 hunting。

### ID ping failed 或 Hardware error

可尝试：

```bash
./build/scan_motors
./build/clear_errors
```

同时检查：

- 电源是否足够。
- 电机 ID 是否是 11、12、13、14、15。
- 串口是不是 `/dev/ttyUSB0`。
- 当前 shell 是否有 `dialout`。
- 某个关节是否卡住或过载。

### OpenCV 窗口太小或太大

改 `include/config.hpp`：

```cpp
constexpr int VISION_WINDOW_W = 1800;
constexpr int VISION_WINDOW_H = 1000;
```

然后：

```bash
cmake --build build -j --target loader test_vision_camera
```

## 推荐开发流程

改视觉或 slot 坐标：

```bash
./build/test_vision_camera
./build/loader --check-config config/loader_config.json
./build/loader config/loader_config.json --replay
./build/loader config/loader_config.json --disturb
```

改机械位姿：

```bash
./build/record_positions positions.xml
./build/test_named_positions positions.xml your_new_point
./build/test_gripper_trajectory
./build/loader config/loader_config.json --replay
```

改主程序安全/扰动逻辑：

```bash
cmake --build build -j --target loader
./build/loader --check-config config/loader_config.json
./build/test_vision_camera
./build/loader config/loader_config.json --disturb
```

## 一句话版

先用 `test_vision_camera` 确认飞镖坐标稳定，再用 `record_positions` 和 `test_named_positions` 确认机械点位，最后用 `loader config/loader_config.json --replay` 验证纯轨迹，用 `--disturb` 打开移动飞镖时的暂停和重规划逻辑。调参主要改 `config/loader_config.json`，硬件和窗口大小改 `include/config.hpp`。
