#pragma once

#include <string>

namespace rk3588::modules {

struct AppConfig {
    std::string model_path = "models/yolov8n.rknn";
    std::string camera_device = "/dev/video0";
    std::string rtsp_url = "rtsp://127.0.0.1:8554/live";
};

}  // namespace rk3588::modules
