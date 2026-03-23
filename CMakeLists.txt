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

include(cmake/deps/mpp.cmake)
include(cmake/deps/rga.cmake)
include(cmake/deps/rknn.cmake)
include(cmake/deps/rplidar.cmake)
include(cmake/deps/zlm.cmake)
include(cmake/deps/ffmpeg.cmake)

include(cmake/modules/core.cmake)
include(cmake/modules/camera.cmake)
include(cmake/modules/infer.cmake)
include(cmake/modules/fusion.cmake)
include(cmake/modules/lidar.cmake)
include(cmake/modules/video_base.cmake)
include(cmake/modules/video_overlay.cmake)
include(cmake/modules/video_rga.cmake)
include(cmake/modules/video_encode.cmake)
include(cmake/modules/video_publish_rtsp.cmake)
include(cmake/modules/video_publish_webrtc.cmake)
include(cmake/modules/video_publish_hub.cmake)
include(cmake/modules/pipeline.cmake)

include(cmake/apps/perception_app.cmake)
include(cmake/apps/stream_gateway_app.cmake)

include(cmake/demos/camera_demos.cmake)
include(cmake/demos/video_demos.cmake)
include(cmake/demos/lidar_demos.cmake)
include(cmake/demos/fusion_demos.cmake)

include(cmake/tools/calib_tools.cmake)
include(cmake/tools/motor_tools.cmake)
include(cmake/tools/diagnostics_tools.cmake)