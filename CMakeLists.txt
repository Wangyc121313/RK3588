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

add_executable(camera_thread_demo
    src/camera_thread_demo.cpp
    src/modules/camera_capture.cpp
)

add_executable(pipeline_thread_demo
    src/pipeline_thread_demo.cpp
)

target_link_libraries(camera_v4l2_demo PRIVATE project_warnings)

target_include_directories(camera_thread_demo PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(camera_thread_demo PRIVATE project_warnings)

target_include_directories(pipeline_thread_demo PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(pipeline_thread_demo PRIVATE project_warnings)

find_path(MPP_INCLUDE_DIR
    NAMES rk_mpi.h
    PATHS
        /usr/include/rockchip
        /usr/local/include/rockchip
        /usr/include
        /usr/local/include
)

find_library(MPP_LIB
    NAMES
        rockchip_mpp
        rk_mpi
    PATHS
        /usr/lib
        /usr/local/lib
        /usr/lib/aarch64-linux-gnu
)

if(MPP_INCLUDE_DIR AND MPP_LIB)
    add_executable(mpp_encoder_demo
        src/mpp_encoder_demo.cpp
        src/modules/camera_capture.cpp
        src/modules/mpp_encoder.cpp
        src/modules/nv12_overlay.cpp
    )

    target_include_directories(mpp_encoder_demo PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${MPP_INCLUDE_DIR}
    )

    target_link_libraries(mpp_encoder_demo PRIVATE
        project_warnings
        ${MPP_LIB}
    )

    set_target_properties(mpp_encoder_demo PROPERTIES
        BUILD_RPATH "/usr/lib:/usr/local/lib"
    )
else()
    message(STATUS "MPP headers/libs not found; skipping mpp_encoder_demo target")
endif()

set(ZLM_ROOT "${CMAKE_SOURCE_DIR}/third_party/ZLMediaKit")
set(ZLM_API_INCLUDE_DIR "${ZLM_ROOT}/api/include")
set(ZLM_RELEASE_DIR "${ZLM_ROOT}/release/linux/Release")
set(ZLM_API_LIB "${ZLM_RELEASE_DIR}/libmk_api.so")

if(TARGET mpp_encoder_demo)
    if(EXISTS "${ZLM_API_INCLUDE_DIR}/mk_mediakit.h" AND EXISTS "${ZLM_API_LIB}")
        target_sources(mpp_encoder_demo PRIVATE
            src/modules/zlm_rtsp_publisher.cpp
        )

        target_include_directories(mpp_encoder_demo PRIVATE
            ${ZLM_API_INCLUDE_DIR}
        )

        target_link_libraries(mpp_encoder_demo PRIVATE
            ${ZLM_API_LIB}
        )

        set_target_properties(mpp_encoder_demo PROPERTIES
            BUILD_RPATH "${ZLM_RELEASE_DIR}:/usr/lib:/usr/local/lib"
        )
    else()
        message(FATAL_ERROR "ZLMediaKit C API not found; build third_party/ZLMediaKit and provide release/linux/Release/libmk_api.so")
    endif()
endif()

set(LIBRGA_ROOT "${CMAKE_SOURCE_DIR}/third_party/librga")
set(LIBRGA_INCLUDE_DIR "${LIBRGA_ROOT}/include")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(LIBRGA_LIB_DIR "${LIBRGA_ROOT}/libs/Linux/gcc-aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    set(LIBRGA_LIB_DIR "${LIBRGA_ROOT}/libs/Linux/gcc-armhf")
else()
    set(LIBRGA_LIB_DIR "")
endif()

set(LIBRGA_SHARED "${LIBRGA_LIB_DIR}/librga.so")

if(EXISTS "${LIBRGA_INCLUDE_DIR}/im2d.h" AND EXISTS "${LIBRGA_SHARED}")
    add_executable(rga_demo
        src/rga_demo.cpp
        src/modules/rga_processor.cpp
    )

    add_executable(camera_rga_demo
        src/camera_rga_demo.cpp
        src/modules/camera_capture.cpp
        src/modules/rga_processor.cpp
    )

    target_include_directories(rga_demo PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${LIBRGA_INCLUDE_DIR}
    )

    target_include_directories(camera_rga_demo PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${LIBRGA_INCLUDE_DIR}
    )

    if(TARGET mpp_encoder_demo)
        target_sources(mpp_encoder_demo PRIVATE
            src/modules/rga_processor.cpp
            src/modules/rknn_runner.cpp
        )

        target_include_directories(mpp_encoder_demo PRIVATE
            ${LIBRGA_INCLUDE_DIR}
        )

        target_link_libraries(mpp_encoder_demo PRIVATE
            ${LIBRGA_SHARED}
        )

        set_target_properties(mpp_encoder_demo PROPERTIES
            BUILD_RPATH "${ZLM_RELEASE_DIR}:${LIBRGA_LIB_DIR}:/usr/lib:/usr/local/lib"
        )
    endif()

    target_link_libraries(rga_demo PRIVATE
        project_warnings
        ${LIBRGA_SHARED}
    )

    target_link_libraries(camera_rga_demo PRIVATE
        project_warnings
        ${LIBRGA_SHARED}
    )

    set_target_properties(rga_demo PROPERTIES
        BUILD_RPATH "${LIBRGA_LIB_DIR}"
    )

    set_target_properties(camera_rga_demo PROPERTIES
        BUILD_RPATH "${LIBRGA_LIB_DIR}"
    )

    set(RKNN_API_LIB "/usr/lib/librknn_api.so")
    if(EXISTS "${RKNN_API_LIB}")
        if(TARGET mpp_encoder_demo)
            target_link_libraries(mpp_encoder_demo PRIVATE
                ${RKNN_API_LIB}
            )
        endif()

        add_executable(camera_rga_rknn_demo
            src/camera_rga_rknn_demo.cpp
            src/modules/camera_capture.cpp
            src/modules/rga_processor.cpp
            src/modules/rknn_runner.cpp
        )

        add_executable(camera_rga_rknn_rtsp_demo
            src/camera_rga_rknn_rtsp_demo.cpp
            src/modules/camera_capture.cpp
            src/modules/rga_processor.cpp
            src/modules/rknn_runner.cpp
            src/modules/frame_overlay.cpp
        )

        target_include_directories(camera_rga_rknn_demo PRIVATE
            ${CMAKE_SOURCE_DIR}/include
            ${LIBRGA_INCLUDE_DIR}
        )

        target_include_directories(camera_rga_rknn_rtsp_demo PRIVATE
            ${CMAKE_SOURCE_DIR}/include
            ${LIBRGA_INCLUDE_DIR}
        )

        target_link_libraries(camera_rga_rknn_demo PRIVATE
            project_warnings
            ${LIBRGA_SHARED}
            ${RKNN_API_LIB}
        )

        target_link_libraries(camera_rga_rknn_rtsp_demo PRIVATE
            project_warnings
            ${LIBRGA_SHARED}
            ${RKNN_API_LIB}
        )

        set_target_properties(camera_rga_rknn_demo PROPERTIES
            BUILD_RPATH "${LIBRGA_LIB_DIR}:/usr/lib"
        )

        set_target_properties(camera_rga_rknn_rtsp_demo PROPERTIES
            BUILD_RPATH "${LIBRGA_LIB_DIR}:/usr/lib"
        )
    else()
        message(STATUS "librknn_api.so not found under /usr/lib; skipping camera_rga_rknn_demo target")
    endif()
else()
    message(STATUS "librga not found under third_party/librga; skipping rga_demo target")
endif()

set(RPLIDAR_REPO_ROOT "${CMAKE_SOURCE_DIR}/third_party/rplidar_sdk")
set(RPLIDAR_SDK_ROOT "${RPLIDAR_REPO_ROOT}/sdk")
set(RPLIDAR_SDK_INCLUDE_DIR "${RPLIDAR_SDK_ROOT}/include")

if(EXISTS "${RPLIDAR_SDK_INCLUDE_DIR}/sl_lidar.h")
    if(TARGET mpp_encoder_demo)
        target_include_directories(mpp_encoder_demo PRIVATE
            ${RPLIDAR_SDK_INCLUDE_DIR}
        )

        target_link_directories(mpp_encoder_demo PRIVATE
            ${RPLIDAR_REPO_ROOT}/output/Linux/Release
        )

        target_link_libraries(mpp_encoder_demo PRIVATE
            sl_lidar_sdk
            pthread
        )
    endif()

    add_executable(rplidar_demo
        src/rplidar_demo.cpp
    )

    add_executable(rplidar_timed_demo
        src/rplidar_timed_demo.cpp
    )

    add_executable(rplidar_guard_demo
        src/rplidar_guard_demo.cpp
    )

    add_executable(rplidar_angle_calib
        src/rplidar_angle_calib.cpp
    )

    add_executable(rplidar_fov_filter_demo
        src/rplidar_fov_filter_demo.cpp
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

    target_include_directories(rplidar_angle_calib PRIVATE
        ${RPLIDAR_SDK_INCLUDE_DIR}
    )

    target_include_directories(rplidar_fov_filter_demo PRIVATE
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

    target_link_directories(rplidar_angle_calib PRIVATE
        ${RPLIDAR_REPO_ROOT}/output/Linux/Release
    )

    target_link_directories(rplidar_fov_filter_demo PRIVATE
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

    target_link_libraries(rplidar_angle_calib PRIVATE
        project_warnings
        sl_lidar_sdk
        pthread
    )

    target_link_libraries(rplidar_fov_filter_demo PRIVATE
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