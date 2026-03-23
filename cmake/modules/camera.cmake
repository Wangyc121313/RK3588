add_library(rk_camera STATIC src/camera/camera_capture.cpp)
target_include_directories(rk_camera PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(rk_camera PUBLIC rk_core)
