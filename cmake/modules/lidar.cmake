add_library(rk_lidar STATIC
    src/lidar/lidar_reader.cpp
    src/lidar/lidar_adapter.cpp
)
target_include_directories(rk_lidar PUBLIC ${CMAKE_SOURCE_DIR}/include)
if(RK_DEPS_RPLIDAR_FOUND)
    target_compile_definitions(rk_lidar PUBLIC RK3588_HAS_RPLIDAR_SDK=1)
    target_include_directories(rk_lidar PUBLIC ${RPLIDAR_SDK_INCLUDE_DIR})
    target_link_directories(rk_lidar PUBLIC ${RPLIDAR_REPO_ROOT}/output/Linux/Release)
    target_link_libraries(rk_lidar PUBLIC sl_lidar_sdk pthread)
else()
    target_compile_definitions(rk_lidar PUBLIC RK3588_HAS_RPLIDAR_SDK=0)
endif()
