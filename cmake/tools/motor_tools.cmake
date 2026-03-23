if(RK_DEPS_RPLIDAR_FOUND)
    add_executable(rplidar_motor_ctl tools/motor_ctl/rplidar_motor_ctl.cpp)
    target_link_libraries(rplidar_motor_ctl PRIVATE rk_lidar)
endif()
