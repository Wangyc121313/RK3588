add_library(rk_pipeline STATIC
	src/pipeline/perception_pipeline.cpp
	src/pipeline/runtime_telemetry.cpp
	src/pipeline/streaming_pipeline.cpp
	src/pipeline/pipeline_factory.cpp
)
target_include_directories(rk_pipeline PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(rk_pipeline PUBLIC
	rk_core
	rk_camera
	rk_infer
	rk_fusion
	rk_video_overlay
	rk_video_rga
	rk_video_encode
	rk_video_publish_hub
	rk_lidar
)
