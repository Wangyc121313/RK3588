if(RK_DEPS_RPLIDAR_FOUND)
    add_executable(rplidar_angle_calib tools/calib/rplidar_angle_calib.cpp)
    target_link_libraries(rplidar_angle_calib PRIVATE rk_lidar)
endif()
