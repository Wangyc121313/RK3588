add_library(rk_video_overlay STATIC
    src/video/frame_overlay.cpp
    src/video/nv12_overlay.cpp
)
target_include_directories(rk_video_overlay PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(rk_video_overlay PUBLIC rk_video_base rk_infer)
