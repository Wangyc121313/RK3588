# Project Structure (Refactor Phase 1)

This repository was reorganized to improve readability while keeping core functionality unchanged.

## Layout

- `apps/`: production entrypoints
- `demos/`: focused demo programs by domain
- `tools/`: calibration / utility executables
- `src/modules/`: core reusable implementations (unchanged in phase 1)
- `include/`: public headers (phase 1 keeps existing module headers intact)
- `cmake/targets/`: split CMake target definitions
- `reports/perf/`: benchmark and stress-test outputs

## Current App Entrypoints

- `apps/perception_main.cpp` -> target: `mpp_encoder_demo`
- `apps/rtsp_pipeline_main.cpp` -> target: `camera_rga_rknn_rtsp_demo`

## Demo Mapping

- `demos/camera/camera_v4l2_demo.cpp` -> `camera_v4l2_demo`
- `demos/camera/camera_thread_demo.cpp` -> `camera_thread_demo`
- `demos/camera/camera_rga_demo.cpp` -> `camera_rga_demo`
- `demos/camera/camera_rga_rknn_demo.cpp` -> `camera_rga_rknn_demo`
- `demos/encode/rga_demo.cpp` -> `rga_demo`
- `demos/fusion/pipeline_thread_demo.cpp` -> `pipeline_thread_demo`
- `demos/lidar/rplidar_demo.cpp` -> `rplidar_demo`
- `demos/lidar/rplidar_timed_demo.cpp` -> `rplidar_timed_demo`
- `demos/lidar/rplidar_guard_demo.cpp` -> `rplidar_guard_demo`
- `demos/lidar/rplidar_fov_filter_demo.cpp` -> `rplidar_fov_filter_demo`

## Tool Mapping

- `tools/calib/rplidar_angle_calib.cpp` -> `rplidar_angle_calib`
- `tools/motor_ctl/rplidar_motor_ctl.cpp` -> `rplidar_motor_ctl`

## Build System

Top-level `CMakeLists.txt` now includes:

- `cmake/targets/base.cmake`
- `cmake/targets/mpp_rga_rknn.cmake`
- `cmake/targets/lidar.cmake`

## Performance Artifacts

Run:

```bash
./scripts/perf_stress_suite.sh 1800 3
```

Outputs are written under `reports/perf/run_YYYYMMDD_HHMMSS/` and `reports/perf/latest` points to the newest run.
