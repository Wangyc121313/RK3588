cmake_minimum_required(VERSION 3.16)

project(rk3588_perception LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(project_warnings INTERFACE)
target_compile_options(project_warnings INTERFACE
    -Wall
    -Wextra
    -Wpedantic
)

add_executable(camera_v4l2_demo
    src/camera_v4l2_demo.cpp
)

target_link_libraries(camera_v4l2_demo PRIVATE project_warnings)

set(RPLIDAR_REPO_ROOT "${CMAKE_SOURCE_DIR}/third_party/rplidar_sdk")
set(RPLIDAR_SDK_ROOT "${RPLIDAR_REPO_ROOT}/sdk")
set(RPLIDAR_SDK_INCLUDE_DIR "${RPLIDAR_SDK_ROOT}/include")

if(EXISTS "${RPLIDAR_SDK_INCLUDE_DIR}/sl_lidar.h")
    add_executable(rplidar_demo
        src/rplidar_demo.cpp
    )

    add_executable(rplidar_timed_demo
        src/rplidar_timed_demo.cpp
    )

    add_executable(rplidar_guard_demo
        src/rplidar_guard_demo.cpp
    )

    add_executable(rplidar_motor_ctl
        src/rplidar_motor_ctl.cpp
    )

    target_include_directories(rplidar_demo PRIVATE
        ${RPLIDAR_SDK_INCLUDE_DIR}
    )

    target_include_directories(rplidar_timed_demo PRIVATE
        ${RPLIDAR_SDK_INCLUDE_DIR}
    )

    target_include_directories(rplidar_guard_demo PRIVATE
        ${RPLIDAR_SDK_INCLUDE_DIR}
    )

    target_include_directories(rplidar_motor_ctl PRIVATE
        ${RPLIDAR_SDK_INCLUDE_DIR}
    )

    target_link_directories(rplidar_demo PRIVATE
        ${RPLIDAR_REPO_ROOT}/output/Linux/Release
    )

    target_link_directories(rplidar_timed_demo PRIVATE
        ${RPLIDAR_REPO_ROOT}/output/Linux/Release
    )

    target_link_directories(rplidar_guard_demo PRIVATE
        ${RPLIDAR_REPO_ROOT}/output/Linux/Release
    )

    target_link_directories(rplidar_motor_ctl PRIVATE
        ${RPLIDAR_REPO_ROOT}/output/Linux/Release
    )

    target_link_libraries(rplidar_demo PRIVATE
        project_warnings
        sl_lidar_sdk
        pthread
    )

    target_link_libraries(rplidar_timed_demo PRIVATE
        project_warnings
        sl_lidar_sdk
        pthread
    )

    target_link_libraries(rplidar_guard_demo PRIVATE
        project_warnings
        sl_lidar_sdk
        pthread
    )

    target_link_libraries(rplidar_motor_ctl PRIVATE
        project_warnings
        sl_lidar_sdk
        pthread
    )
else()
    message(STATUS "RPLIDAR SDK not found at ${RPLIDAR_SDK_ROOT}; skipping rplidar_demo target")
endif()