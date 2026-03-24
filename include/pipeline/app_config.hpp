#pragma once

#include <cstdint>
#include <string>

namespace rk3588::modules {

struct AppConfig {
    std::string camera_device = "/dev/video0";
    std::uint32_t camera_width = 640;
    std::uint32_t camera_height = 480;
    std::uint32_t camera_buffer_count = 4;
    std::uint32_t camera_queue_depth = 1;

    int run_seconds = 0;

    std::string model_path = "models/yolov8n.rknn";
    std::uint32_t model_width = 640;
    std::uint32_t model_height = 640;
    std::string labels_path = "third_party/rknn_model_zoo/examples/yolov8/model/coco_80_labels_list.txt";
    int infer_every_n_frames = 5;
    std::string calibration_profile_path;

    std::string rtsp_url = "rtsp://127.0.0.1:8554/live/camera";
    std::string webrtc_url = "rtc://127.0.0.1:8000/live/camera";
    std::string publish_mode = "rtsp";
    bool rtsp_listen_mode = false;
    int fps = 30;
    std::string dump_h264_path;
    bool debug_video_hud = false;
    std::string telemetry_path;
    std::uint32_t telemetry_interval_ms = 1000;

    std::string lidar_port = "/dev/ttyUSB0";
    int lidar_baud = 115200;
    float camera_fov_deg = 55.0F;
    float lidar_offset_deg = 11.7F;
    float lidar_fov_deg = 55.0F;
    float lidar_window_half_deg = 2.5F;
    float lidar_min_dist_m = 0.15F;
    float lidar_max_dist_m = 20.0F;
    std::uint64_t lidar_max_age_ms = 120U;

    bool swap_uv = false;
    std::uint32_t forced_422_fourcc = 0;

    [[nodiscard]] bool rtspEnabled() const {
        return publish_mode == "rtsp" || publish_mode == "both";
    }

    [[nodiscard]] bool webrtcEnabled() const {
        return publish_mode == "webrtc" || publish_mode == "both";
    }

    [[nodiscard]] bool publishModeValid() const {
        return rtspEnabled() || webrtcEnabled();
    }
};

}  // namespace rk3588::modules
