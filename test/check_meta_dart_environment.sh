#!/usr/bin/env bash
#
# Comprehensive, non-moving environment check for Meta-Dart.
#
# This script checks:
#   - C++ build tools and OpenCV
#   - ONNX Runtime C++ install expected by CMake
#   - required project files and build outputs
#   - ece445 conda Python/YOLO pipeline
#   - camera devices and OpenCV camera access
#   - Dynamixel serial device and permissions
#
# It does not move the robot arm.
#
# Optional:
#   CONDA_ENV_NAME=ece445 ./test/check_meta_dart_environment.sh
#   RUN_BUILD_CHECK=1 ./test/check_meta_dart_environment.sh
#   RUN_MOTOR_SCAN=1 ./test/check_meta_dart_environment.sh

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONDA_ENV_NAME="${CONDA_ENV_NAME:-ece445}"
RUN_BUILD_CHECK="${RUN_BUILD_CHECK:-0}"
RUN_MOTOR_SCAN="${RUN_MOTOR_SCAN:-0}"

PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0

section() {
  printf '\n==== %s ====\n' "$1"
}

pass() {
  PASS_COUNT=$((PASS_COUNT + 1))
  printf '[PASS] %s\n' "$1"
}

warn() {
  WARN_COUNT=$((WARN_COUNT + 1))
  printf '[WARN] %s\n' "$1"
}

fail() {
  FAIL_COUNT=$((FAIL_COUNT + 1))
  printf '[FAIL] %s\n' "$1"
}

info() {
  printf '[INFO] %s\n' "$1"
}

have_cmd() {
  command -v "$1" >/dev/null 2>&1
}

check_cmd() {
  local cmd="$1"
  if have_cmd "$cmd"; then
    pass "command found: ${cmd} ($(command -v "$cmd"))"
  else
    fail "command missing: ${cmd}"
  fi
}

first_config_value() {
  local pattern="$1"
  sed -nE "$pattern" "${REPO_ROOT}/include/config.hpp" | head -1
}

CONFIG_DXL_PORT="$(first_config_value 's/.*DXL_PORT[[:space:]]*=[[:space:]]*"([^"]+)".*/\1/p')"
CONFIG_CAMERA_INDEX="$(first_config_value 's/.*CAMERA_INDEX[[:space:]]*=[[:space:]]*([0-9]+).*/\1/p')"
CONFIG_POSITIONS_XML="$(first_config_value 's/.*POSITIONS_XML_PATH[[:space:]]*=[[:space:]]*"([^"]+)".*/\1/p')"
CONFIG_ONNX_MODEL="$(first_config_value 's/.*ONNX_MODEL_PATH[[:space:]]*=[[:space:]]*"([^"]+)".*/\1/p')"

CONFIG_DXL_PORT="${CONFIG_DXL_PORT:-/dev/ttyUSB0}"
CONFIG_CAMERA_INDEX="${CONFIG_CAMERA_INDEX:-2}"
CONFIG_POSITIONS_XML="${CONFIG_POSITIONS_XML:-positions.xml}"
CONFIG_ONNX_MODEL="${CONFIG_ONNX_MODEL:-models/best.onnx}"

resolve_repo_path() {
  local path="$1"
  if [[ "$path" = /* ]]; then
    printf '%s\n' "$path"
  else
    printf '%s/%s\n' "$REPO_ROOT" "$path"
  fi
}

POSITIONS_PATH="$(resolve_repo_path "$CONFIG_POSITIONS_XML")"
ONNX_MODEL_PATH="$(resolve_repo_path "$CONFIG_ONNX_MODEL")"

section "Repo And Config"
info "repo root: ${REPO_ROOT}"
info "configured DXL_PORT: ${CONFIG_DXL_PORT}"
info "configured CAMERA_INDEX: ${CONFIG_CAMERA_INDEX}"
info "configured POSITIONS_XML_PATH: ${CONFIG_POSITIONS_XML}"
info "configured ONNX_MODEL_PATH: ${CONFIG_ONNX_MODEL}"

if [[ -f "${REPO_ROOT}/include/config.hpp" ]]; then
  pass "include/config.hpp exists"
else
  fail "include/config.hpp missing"
fi

if [[ -f "${POSITIONS_PATH}" ]]; then
  pass "positions file exists: ${POSITIONS_PATH}"
else
  fail "positions file missing: ${POSITIONS_PATH}"
fi

if [[ -f "${ONNX_MODEL_PATH}" ]]; then
  pass "ONNX model exists: ${ONNX_MODEL_PATH} ($(du -h "${ONNX_MODEL_PATH}" | awk '{print $1}'))"
else
  fail "ONNX model missing: ${ONNX_MODEL_PATH}"
fi

if [[ -f "${POSITIONS_PATH}" ]]; then
  for name in home forward_prep backward_prep s0_prep s0_grasp s0_pulled_out loading; do
    if grep -q "name=\"${name}\"" "${POSITIONS_PATH}"; then
      pass "trajectory position exists: ${name}"
    else
      fail "trajectory position missing in ${POSITIONS_PATH}: ${name}"
    fi
  done
fi

section "System Tools"
check_cmd g++
if have_cmd g++; then
  g++ --version | head -1
  tmp_cpp="/tmp/meta_dart_cxx17_check.cpp"
  tmp_bin="/tmp/meta_dart_cxx17_check"
  cat >"${tmp_cpp}" <<'CPP'
#include <array>
#include <optional>
int main() {
    std::array<int, 2> a{1, 2};
    std::optional<int> v = a[0] + a[1];
    return *v == 3 ? 0 : 1;
}
CPP
  if g++ -std=c++17 "${tmp_cpp}" -o "${tmp_bin}" >/tmp/meta_dart_cxx17_check.log 2>&1 && "${tmp_bin}"; then
    pass "g++ can compile and run a C++17 probe"
  else
    fail "g++ C++17 probe failed; see /tmp/meta_dart_cxx17_check.log"
  fi
fi

check_cmd cmake
if have_cmd cmake; then
  cmake --version | head -1
fi

check_cmd pkg-config
if have_cmd pkg-config; then
  pkg-config --version
fi

if have_cmd pkg-config && pkg-config --exists opencv4; then
  pass "pkg-config can find opencv4 version $(pkg-config --modversion opencv4)"
else
  fail "pkg-config cannot find opencv4"
fi

if [[ -f /usr/include/opencv4/opencv2/opencv.hpp ]]; then
  pass "OpenCV development header exists at /usr/include/opencv4/opencv2/opencv.hpp"
else
  warn "OpenCV header not found at /usr/include/opencv4/opencv2/opencv.hpp"
fi

check_cmd v4l2-ctl
check_cmd lsusb
check_cmd udevadm

section "ONNX Runtime C++"
ORT_ROOT="${HOME}/Applications/onnxruntime"
info "expected ORT_ROOT: ${ORT_ROOT}"
if [[ -e "${ORT_ROOT}" ]]; then
  pass "ONNX Runtime path exists: ${ORT_ROOT}"
  ls -ld "${ORT_ROOT}"
else
  fail "ONNX Runtime path missing: ${ORT_ROOT}"
fi

if [[ -f "${ORT_ROOT}/include/onnxruntime_cxx_api.h" ]]; then
  pass "ONNX Runtime C++ header exists"
else
  fail "ONNX Runtime C++ header missing: ${ORT_ROOT}/include/onnxruntime_cxx_api.h"
fi

if [[ -f "${ORT_ROOT}/lib/libonnxruntime.so" ]]; then
  pass "ONNX Runtime shared library exists"
else
  fail "ONNX Runtime shared library missing: ${ORT_ROOT}/lib/libonnxruntime.so"
fi

section "Dynamixel Sources And Build Outputs"
if [[ -f "${REPO_ROOT}/thirdparty/DynamixelSDK/c++/src/dynamixel_sdk/packet_handler.cpp" ]]; then
  pass "DynamixelSDK C++ sources are present"
else
  fail "DynamixelSDK C++ sources missing"
fi

if [[ -d "${REPO_ROOT}/thirdparty/dynamixel-workbench/dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox" ]]; then
  pass "dynamixel-workbench toolbox sources are present"
else
  fail "dynamixel-workbench toolbox sources missing"
fi

if git -C "${REPO_ROOT}" submodule status >/tmp/meta_dart_submodules.log 2>&1; then
  info "git submodule status:"
  sed 's/^/  /' /tmp/meta_dart_submodules.log
  if grep -q '^-.*thirdparty/' /tmp/meta_dart_submodules.log; then
    warn "git reports one or more submodules with '-' prefix, even if source files may be present"
  else
    pass "git submodule status does not show uninitialized submodules"
  fi
else
  warn "git submodule status failed"
fi

for target in dart_launcher test_trajectory test_gripper_trajectory test_arm scan_motors; do
  if [[ -x "${REPO_ROOT}/build/${target}" ]]; then
    pass "build output exists: build/${target}"
  else
    warn "build output missing or not executable: build/${target}"
  fi
done

if [[ "${RUN_BUILD_CHECK}" == "1" ]]; then
  section "CMake Build Check"
  if cmake -S "${REPO_ROOT}" -B "${REPO_ROOT}/build"; then
    pass "cmake configure succeeded"
  else
    fail "cmake configure failed"
  fi
  if cmake --build "${REPO_ROOT}/build" -j --target dart_launcher test_trajectory test_gripper_trajectory; then
    pass "selected CMake targets built successfully"
  else
    fail "selected CMake targets failed to build"
  fi
else
  info "Skipping build check. Set RUN_BUILD_CHECK=1 to configure/build selected targets."
fi

section "User Groups And Device Permissions"
info "current user: ${USER:-unknown}"
info "current process groups: $(id -nG)"
if groups "${USER:-$(id -un)}" >/tmp/meta_dart_groups.log 2>&1; then
  info "account groups: $(cat /tmp/meta_dart_groups.log)"
fi

if id -nG | tr ' ' '\n' | grep -qx dialout; then
  pass "current shell has dialout group"
else
  warn "current shell does not have dialout group; reopen terminal/newgrp dialout/relogin may be needed"
fi

if id -nG | tr ' ' '\n' | grep -qx video; then
  pass "current shell has video group"
else
  warn "current shell does not have video group; camera ACL may still grant access"
fi

section "Dynamixel Serial Device"
if compgen -G "/dev/ttyUSB*" >/dev/null || compgen -G "/dev/ttyACM*" >/dev/null; then
  ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
else
  fail "no /dev/ttyUSB* or /dev/ttyACM* devices found"
fi

if [[ -e "${CONFIG_DXL_PORT}" ]]; then
  pass "configured DXL_PORT exists: ${CONFIG_DXL_PORT}"
  ls -l "${CONFIG_DXL_PORT}"
  if [[ -r "${CONFIG_DXL_PORT}" && -w "${CONFIG_DXL_PORT}" ]]; then
    pass "current process can read/write ${CONFIG_DXL_PORT}"
  else
    fail "current process cannot read/write ${CONFIG_DXL_PORT}"
  fi
else
  fail "configured DXL_PORT does not exist: ${CONFIG_DXL_PORT}"
fi

if [[ "${RUN_MOTOR_SCAN}" == "1" ]]; then
  section "Optional Motor Scan"
  if [[ -x "${REPO_ROOT}/build/scan_motors" ]]; then
    "${REPO_ROOT}/build/scan_motors" "${CONFIG_DXL_PORT}"
  else
    fail "build/scan_motors missing; cannot run motor scan"
  fi
else
  info "Skipping motor scan. Set RUN_MOTOR_SCAN=1 to run build/scan_motors."
fi

section "Camera Devices"
if compgen -G "/dev/video*" >/dev/null || compgen -G "/dev/media*" >/dev/null; then
  ls -l /dev/video* /dev/media* 2>/dev/null || true
else
  fail "no /dev/video* or /dev/media* devices found"
fi

if [[ -e "/dev/video${CONFIG_CAMERA_INDEX}" ]]; then
  pass "configured camera node exists: /dev/video${CONFIG_CAMERA_INDEX}"
  ls -l "/dev/video${CONFIG_CAMERA_INDEX}"
else
  warn "configured camera node is not present: /dev/video${CONFIG_CAMERA_INDEX}"
fi

if have_cmd v4l2-ctl; then
  info "v4l2-ctl --list-devices:"
  v4l2-ctl --list-devices 2>/tmp/meta_dart_v4l2_err.log || warn "v4l2-ctl --list-devices failed: $(cat /tmp/meta_dart_v4l2_err.log)"
fi

for node in /dev/video*; do
  [[ -e "$node" ]] || continue
  if have_cmd udevadm; then
    product="$(udevadm info --query=property --name="$node" 2>/dev/null | sed -nE 's/^ID_V4L_PRODUCT=(.*)$/\1/p' | head -1)"
    caps="$(udevadm info --query=property --name="$node" 2>/dev/null | sed -nE 's/^ID_V4L_CAPABILITIES=(.*)$/\1/p' | head -1)"
    info "${node}: product='${product:-unknown}' capabilities='${caps:-unknown}'"
  fi
done

section "Conda Python Pipeline"
if have_cmd conda; then
  pass "conda command found"
else
  fail "conda command missing"
fi

if have_cmd conda && conda env list | awk '{print $1}' | grep -qx "${CONDA_ENV_NAME}"; then
  pass "conda environment exists: ${CONDA_ENV_NAME}"
else
  fail "conda environment missing: ${CONDA_ENV_NAME}"
fi

if have_cmd conda; then
  conda run -n "${CONDA_ENV_NAME}" python - <<'PY'
import importlib
import sys

required = ["torch", "cv2", "ultralytics"]
optional = ["onnx", "onnxruntime"]
failed = []

print("python:", sys.version.split()[0])

for name in required + optional:
    try:
        mod = importlib.import_module(name)
        version = getattr(mod, "__version__", "unknown")
        level = "required" if name in required else "optional"
        print(f"{level}: {name} OK version={version}")
    except Exception as exc:
        level = "required" if name in required else "optional"
        print(f"{level}: {name} FAIL {type(exc).__name__}: {exc}")
        if name in required:
            failed.append(name)

try:
    import torch
    print("torch.cuda.is_available:", torch.cuda.is_available())
    print("torch.cuda.device_count:", torch.cuda.device_count())
except Exception as exc:
    print("torch cuda check failed:", exc)
    failed.append("torch")

raise SystemExit(1 if failed else 0)
PY
  py_status=$?
  if [[ "${py_status}" -eq 0 ]]; then
    pass "required Python pipeline imports succeeded in ${CONDA_ENV_NAME}"
  else
    fail "one or more required Python pipeline imports failed in ${CONDA_ENV_NAME}"
  fi
fi

section "OpenCV Camera Probe"
if have_cmd conda; then
  conda run -n "${CONDA_ENV_NAME}" python - "${CONFIG_CAMERA_INDEX}" <<'PY'
import sys

try:
    import cv2
except Exception as exc:
    print(f"cv2 import failed: {type(exc).__name__}: {exc}")
    raise SystemExit(1)

configured = int(sys.argv[1])
checked = sorted(set(list(range(6)) + [configured]))
opened_any = False
configured_ok = False

print("cv2:", cv2.__version__)
for index in checked:
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    opened = cap.isOpened()
    ok = False
    shape = None
    if opened:
        ok, frame = cap.read()
        if ok:
            shape = frame.shape
            opened_any = True
            if index == configured:
                configured_ok = True
    print(f"camera index {index}: opened={opened} read={ok} shape={shape}")
    cap.release()

if not opened_any:
    raise SystemExit(2)
if not configured_ok:
    raise SystemExit(3)
PY
  cam_status=$?
  if [[ "${cam_status}" -eq 0 ]]; then
    pass "configured CAMERA_INDEX=${CONFIG_CAMERA_INDEX} opens and reads through OpenCV"
  elif [[ "${cam_status}" -eq 3 ]]; then
    warn "at least one camera opened, but configured CAMERA_INDEX=${CONFIG_CAMERA_INDEX} did not read successfully"
  else
    fail "OpenCV camera probe could not read any camera"
  fi
fi

section "Summary"
printf 'PASS=%d WARN=%d FAIL=%d\n' "${PASS_COUNT}" "${WARN_COUNT}" "${FAIL_COUNT}"
if [[ "${FAIL_COUNT}" -gt 0 ]]; then
  printf 'Result: FAIL. Fix failed items above before running the full pipeline.\n'
  exit 1
fi

if [[ "${WARN_COUNT}" -gt 0 ]]; then
  printf 'Result: PASS WITH WARNINGS. Review warning items above.\n'
else
  printf 'Result: PASS.\n'
fi
