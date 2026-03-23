add_library(project_warnings INTERFACE)
target_compile_options(project_warnings INTERFACE
    -Wall
    -Wextra
    -Wpedantic
)

add_executable(camera_v4l2_demo
    demos/camera/camera_v4l2_demo.cpp
)

target_link_libraries(camera_v4l2_demo PRIVATE project_warnings)

add_executable(camera_thread_demo
    demos/camera/camera_thread_demo.cpp
    src/camera/camera_capture.cpp
)

target_include_directories(camera_thread_demo PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(camera_thread_demo PRIVATE project_warnings)

add_executable(pipeline_thread_demo
    demos/fusion/pipeline_thread_demo.cpp
)

target_include_directories(pipeline_thread_demo PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(pipeline_thread_demo PRIVATE project_warnings)
