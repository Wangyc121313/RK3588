add_library(rk_video_base INTERFACE)
target_include_directories(rk_video_base INTERFACE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(rk_video_base INTERFACE rk_core)
