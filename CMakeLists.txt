cmake_minimum_required(VERSION 3.16)
project(dart_launcher CXX)

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

# ── Dynamixel sources ─────────────────────────────────────────────────────────
set(DXL_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/DynamixelSDK/c++")
set(DXL_WB_ROOT  "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/dynamixel-workbench/dynamixel_workbench_toolbox")

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

file(GLOB DXL_WB_SRCS
    "${DXL_WB_ROOT}/src/dynamixel_workbench_toolbox/*.cpp"
)

set(DXL_INCLUDE_DIRS
    ${DXL_SDK_ROOT}/include/dynamixel_sdk
    ${DXL_WB_ROOT}/include
    ${DXL_WB_ROOT}/include/dynamixel_workbench_toolbox
    ${DXL_WB_ROOT}/src
    ${DXL_SDK_ROOT}/include
)

# Shared sources for targets that drive the arm directly
set(ARM_SRCS src/arm.cpp ${DXL_SDK_SRCS} ${DXL_WB_SRCS})

# ── Main executable ───────────────────────────────────────────────────────────
add_executable(dart_launcher
    src/main.cpp
    src/camera.cpp
    src/vision.cpp
    src/arm.cpp
    src/spring_stretcher.cpp
    src/decision.cpp
    src/runtime_config.cpp
    ${DXL_SDK_SRCS}
    ${DXL_WB_SRCS}
)

target_include_directories(dart_launcher PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${DXL_INCLUDE_DIRS}
    ${ORT_ROOT}/include
)

target_link_libraries(dart_launcher PRIVATE
    ${OpenCV_LIBS}
    pthread
    ${ORT_ROOT}/lib/libonnxruntime.so
)

target_link_options(dart_launcher PRIVATE -Wl,-rpath,${ORT_ROOT}/lib)
target_compile_options(dart_launcher PRIVATE -Wall -Wextra -O2)

# ── Vision camera test (no arm, no decision) ──────────────────────────────────
add_executable(test_vision_camera
    test/test_vision_camera.cpp
    src/camera.cpp
    src/vision.cpp
)

target_include_directories(test_vision_camera PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${ORT_ROOT}/include
)

target_link_libraries(test_vision_camera PRIVATE
    ${OpenCV_LIBS}
    pthread
    ${ORT_ROOT}/lib/libonnxruntime.so
)

target_link_options(test_vision_camera PRIVATE -Wl,-rpath,${ORT_ROOT}/lib)
target_compile_options(test_vision_camera PRIVATE -Wall -Wextra -O2)

# ── Spring stretcher tuning utilities ─────────────────────────────────────────
add_executable(test_stretcher_monitor test/test_stretcher_monitor.cpp)
target_include_directories(test_stretcher_monitor PRIVATE include test)
target_link_libraries(test_stretcher_monitor PRIVATE pthread)
target_compile_options(test_stretcher_monitor PRIVATE -Wall -Wextra -O2)

add_executable(test_stretcher_direction test/test_stretcher_direction.cpp)
target_include_directories(test_stretcher_direction PRIVATE include test)
target_link_libraries(test_stretcher_direction PRIVATE pthread)
target_compile_options(test_stretcher_direction PRIVATE -Wall -Wextra -O2)

add_executable(test_stretcher_calibration test/test_stretcher_calibration.cpp)
target_include_directories(test_stretcher_calibration PRIVATE include test)
target_link_libraries(test_stretcher_calibration PRIVATE pthread)
target_compile_options(test_stretcher_calibration PRIVATE -Wall -Wextra -O2)

add_executable(test_stretcher_stretch test/test_stretcher_stretch.cpp)
target_include_directories(test_stretcher_stretch PRIVATE include test)
target_link_libraries(test_stretcher_stretch PRIVATE pthread)
target_compile_options(test_stretcher_stretch PRIVATE -Wall -Wextra -O2)

# ── Arm hardware test ─────────────────────────────────────────────────────────
add_executable(test_arm test/test_arm.cpp ${ARM_SRCS})
target_include_directories(test_arm PRIVATE include ${DXL_INCLUDE_DIRS})
target_link_libraries(test_arm PRIVATE pthread)
target_compile_options(test_arm PRIVATE -Wall -Wextra -O2)

# ── Record-and-recall test ────────────────────────────────────────────────────
add_executable(test_recall test/test_recall.cpp ${ARM_SRCS})
target_include_directories(test_recall PRIVATE include ${DXL_INCLUDE_DIRS})
target_link_libraries(test_recall PRIVATE pthread)
target_compile_options(test_recall PRIVATE -Wall -Wextra -O2)

# ── Trajectory test ───────────────────────────────────────────────────────────
add_executable(test_trajectory test/test_trajectory.cpp ${ARM_SRCS})
target_include_directories(test_trajectory PRIVATE include ${DXL_INCLUDE_DIRS})
target_link_libraries(test_trajectory PRIVATE pthread)
target_compile_options(test_trajectory PRIVATE -Wall -Wextra -O2)

# ── Gripper trajectory test ───────────────────────────────────────────────────
add_executable(test_gripper_trajectory test/test_gripper_trajectory.cpp ${ARM_SRCS})
target_include_directories(test_gripper_trajectory PRIVATE include ${DXL_INCLUDE_DIRS})
target_link_libraries(test_gripper_trajectory PRIVATE pthread)
target_compile_options(test_gripper_trajectory PRIVATE -Wall -Wextra -O2)

# ── Named position inspector ──────────────────────────────────────────────────
add_executable(test_named_positions test/test_named_positions.cpp ${ARM_SRCS})
target_include_directories(test_named_positions PRIVATE include ${DXL_INCLUDE_DIRS})
target_link_libraries(test_named_positions PRIVATE pthread)
target_compile_options(test_named_positions PRIVATE -Wall -Wextra -O2)

# ── Position recorder utility ─────────────────────────────────────────────────
add_executable(record_positions test/record_positions.cpp ${DXL_SDK_SRCS} ${DXL_WB_SRCS})
target_include_directories(record_positions PRIVATE include ${DXL_INCLUDE_DIRS})
target_link_libraries(record_positions PRIVATE pthread)
target_compile_options(record_positions PRIVATE -Wall -Wextra -O2)

# ── Motor scan utility ────────────────────────────────────────────────────────
add_executable(scan_motors test/scan_motors.cpp ${DXL_SDK_SRCS})
target_include_directories(scan_motors PRIVATE ${DXL_SDK_ROOT}/include/dynamixel_sdk ${DXL_SDK_ROOT}/include)
target_link_libraries(scan_motors PRIVATE pthread)
target_compile_options(scan_motors PRIVATE -Wall -Wextra -O2)

# ── Gripper calibration utility ───────────────────────────────────────────────
add_executable(calibrate_gripper test/calibrate_gripper.cpp ${ARM_SRCS})
target_include_directories(calibrate_gripper PRIVATE include ${DXL_INCLUDE_DIRS})
target_link_libraries(calibrate_gripper PRIVATE pthread)
target_compile_options(calibrate_gripper PRIVATE -Wall -Wextra -O2)

# ── Motor error clear utility ─────────────────────────────────────────────────
add_executable(clear_errors test/clear_errors.cpp ${ARM_SRCS})
target_include_directories(clear_errors PRIVATE include ${DXL_INCLUDE_DIRS})
target_link_libraries(clear_errors PRIVATE pthread)
target_compile_options(clear_errors PRIVATE -Wall -Wextra -O2)
