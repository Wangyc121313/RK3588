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
        demos/lidar/rplidar_demo.cpp
    )

    add_executable(rplidar_timed_demo
        demos/lidar/rplidar_timed_demo.cpp
    )

    add_executable(rplidar_guard_demo
        demos/lidar/rplidar_guard_demo.cpp
    )

    add_executable(rplidar_fov_filter_demo
        demos/lidar/rplidar_fov_filter_demo.cpp
    )

    add_executable(rplidar_angle_calib
        tools/calib/rplidar_angle_calib.cpp
    )

    add_executable(rplidar_motor_ctl
        tools/motor_ctl/rplidar_motor_ctl.cpp
    )

    foreach(target_name
            rplidar_demo
            rplidar_timed_demo
            rplidar_guard_demo
            rplidar_fov_filter_demo
            rplidar_angle_calib
            rplidar_motor_ctl)
        target_include_directories(${target_name} PRIVATE
            ${RPLIDAR_SDK_INCLUDE_DIR}
        )

        target_link_directories(${target_name} PRIVATE
            ${RPLIDAR_REPO_ROOT}/output/Linux/Release
        )

        target_link_libraries(${target_name} PRIVATE
            project_warnings
            sl_lidar_sdk
            pthread
        )
    endforeach()
else()
    message(STATUS "RPLIDAR SDK not found at ${RPLIDAR_SDK_ROOT}; skipping lidar targets")
endif()
