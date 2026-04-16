cmake_minimum_required(VERSION 3.16)
project(loader CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── OpenCV ────────────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED)

# ── ONNX Runtime ──────────────────────────────────────────────────────────────
set(ORT_ROOT "$ENV{HOME}/Applications/onnxruntime"
    CACHE PATH "ONNX Runtime install root (contains include/ and lib/)")
if(NOT EXISTS "${ORT_ROOT}/include/onnxruntime_cxx_api.h")
    message(FATAL_ERROR "ONNX Runtime not found at ${ORT_ROOT}.")
endif()

# ── DynamixelSDK sources ──────────────────────────────────────────────────────
set(DXL_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/DynamixelSDK/c++")

file(GLOB DXL_SDK_SRCS
    "${DXL_SDK_ROOT}/src/dynamixel_sdk/group_bulk_read.cpp"
    "${DXL_SDK_ROOT}/src/dynamixel_sdk/group_bulk_write.cpp"
    "${DXL_SDK_ROOT}/src/dynamixel_sdk/group_sync_read.cpp"
    "${DXL_SDK_ROOT}/src/dynamixel_sdk/group_sync_write.cpp"
    "${DXL_SDK_ROOT}/src/dynamixel_sdk/packet_handler.cpp"
    "${DXL_SDK_ROOT}/src/dynamixel_sdk/port_handler.cpp"
    "${DXL_SDK_ROOT}/src/dynamixel_sdk/port_handler_linux.cpp"
    "${DXL_SDK_ROOT}/src/dynamixel_sdk/protocol1_packet_handler.cpp"
    "${DXL_SDK_ROOT}/src/dynamixel_sdk/protocol2_packet_handler.cpp"
    "${DXL_SDK_ROOT}/src/dynamixel_sdk/group_handler.cpp"
)

# ── DynamixelWorkbench toolbox sources ────────────────────────────────────────
set(DXL_WB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/dynamixel-workbench/dynamixel_workbench_toolbox")

file(GLOB DXL_WB_SRCS
    "${DXL_WB_ROOT}/src/dynamixel_workbench_toolbox/*.cpp"
)

# ── Main executable ───────────────────────────────────────────────────────────
add_executable(loader
    src/main.cpp
    src/camera.cpp
    src/vision.cpp
    src/kinematics.cpp
    src/arm.cpp
    src/decision.cpp
    ${DXL_SDK_SRCS}
    ${DXL_WB_SRCS}
)

target_include_directories(loader PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${DXL_SDK_ROOT}/include/dynamixel_sdk
    ${DXL_WB_ROOT}/include
    ${DXL_WB_ROOT}/include/dynamixel_workbench_toolbox
    # DynamixelWorkbench.h shim lives in src/
    ${DXL_WB_ROOT}/src
    ${DXL_SDK_ROOT}/include
    ${ORT_ROOT}/include
)

target_link_libraries(loader PRIVATE
    ${OpenCV_LIBS}
    pthread
    ${ORT_ROOT}/lib/libonnxruntime.so
)

target_link_options(loader PRIVATE -Wl,-rpath,${ORT_ROOT}/lib)

target_compile_options(loader PRIVATE -Wall -Wextra -O2)

# ── Arm hardware test ─────────────────────────────────────────────────────────
add_executable(test_arm
    test/test_arm.cpp
    src/arm.cpp
    src/kinematics.cpp
    ${DXL_SDK_SRCS}
    ${DXL_WB_SRCS}
)

target_include_directories(test_arm PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${DXL_SDK_ROOT}/include/dynamixel_sdk
    ${DXL_WB_ROOT}/include
    ${DXL_WB_ROOT}/include/dynamixel_workbench_toolbox
    ${DXL_WB_ROOT}/src
    ${DXL_SDK_ROOT}/include
)

target_link_libraries(test_arm PRIVATE pthread)

target_compile_options(test_arm PRIVATE -Wall -Wextra -O2)

# ── Motor scan utility ────────────────────────────────────────────────────────
add_executable(scan_motors
    test/scan_motors.cpp
    ${DXL_SDK_SRCS}
)

target_include_directories(scan_motors PRIVATE
    ${DXL_SDK_ROOT}/include/dynamixel_sdk
    ${DXL_SDK_ROOT}/include
)

target_link_libraries(scan_motors PRIVATE pthread)

target_compile_options(scan_motors PRIVATE -Wall -Wextra -O2)

# ── Gripper calibration utility ───────────────────────────────────────────────
add_executable(calibrate_gripper
    test/calibrate_gripper.cpp
    ${DXL_SDK_SRCS}
    ${DXL_WB_SRCS}
)

target_include_directories(calibrate_gripper PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${DXL_SDK_ROOT}/include/dynamixel_sdk
    ${DXL_WB_ROOT}/include
    ${DXL_WB_ROOT}/include/dynamixel_workbench_toolbox
    ${DXL_WB_ROOT}/src
    ${DXL_SDK_ROOT}/include
)

target_link_libraries(calibrate_gripper PRIVATE pthread)

target_compile_options(calibrate_gripper PRIVATE -Wall -Wextra -O2)

# ── Motor error clear utility ─────────────────────────────────────────────────
add_executable(clear_errors
    test/clear_errors.cpp
    ${DXL_SDK_SRCS}
    ${DXL_WB_SRCS}
)

target_include_directories(clear_errors PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${DXL_SDK_ROOT}/include/dynamixel_sdk
    ${DXL_WB_ROOT}/include
    ${DXL_WB_ROOT}/include/dynamixel_workbench_toolbox
    ${DXL_WB_ROOT}/src
    ${DXL_SDK_ROOT}/include
)

target_link_libraries(clear_errors PRIVATE pthread)

target_compile_options(clear_errors PRIVATE -Wall -Wextra -O2)
