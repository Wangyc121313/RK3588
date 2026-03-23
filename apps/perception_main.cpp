#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>

#include <linux/videodev2.h>

#include "pipeline/pipeline_factory.hpp"

namespace {

bool envEnabled(const char* name) {
    const char* env = std::getenv(name);
    return env != nullptr && env[0] != '\0' && env[0] != '0';
}

std::string envStringOrEmpty(const char* name) {
    const char* env = std::getenv(name);
    return env == nullptr ? std::string() : std::string(env);
}

std::uint32_t envUintOr(const char* name, std::uint32_t fallback) {
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return fallback;
    }
    const int value = std::atoi(env);
    return value > 0 ? static_cast<std::uint32_t>(value) : fallback;
}

std::uint32_t forced422FourccFromEnv() {
    const char* env = std::getenv("RK3588_FORCE_422");
    if (env == nullptr || env[0] == '\0') {
        return 0;
    }
    const std::string s(env);
    if (s == "YUYV" || s == "yuyv") {
        return V4L2_PIX_FMT_YUYV;
    }
    if (s == "YVYU" || s == "yvyu") {
        return V4L2_PIX_FMT_YVYU;
    }
    if (s == "UYVY" || s == "uyvy") {
        return V4L2_PIX_FMT_UYVY;
    }
    if (s == "VYUY" || s == "vyuy") {
        return V4L2_PIX_FMT_VYUY;
    }
    return 0;
}

}  // namespace

int main(int argc, char* argv[]) {
    rk3588::modules::AppConfig config;
    config.camera_device = argc > 1 ? argv[1] : config.camera_device;
    config.camera_width = argc > 2 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[2]))) : config.camera_width;
    config.camera_height = argc > 3 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[3]))) : config.camera_height;
    config.run_seconds = argc > 4 ? std::atoi(argv[4]) : config.run_seconds;
    config.model_path = argc > 5 ? argv[5] : config.model_path;
    config.model_width = argc > 6 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[6]))) : config.model_width;
    config.model_height = argc > 7 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[7]))) : config.model_height;
    config.labels_path = argc > 8 ? argv[8] : config.labels_path;
    config.rtsp_url = argc > 9 ? argv[9] : config.rtsp_url;
    config.fps = argc > 10 ? std::max(1, std::atoi(argv[10])) : config.fps;
    config.dump_h264_path = argc > 11 ? argv[11] : config.dump_h264_path;
    config.infer_every_n_frames = argc > 12 ? std::max(1, std::atoi(argv[12])) : config.infer_every_n_frames;
    config.lidar_port = argc > 13 ? argv[13] : config.lidar_port;
    config.lidar_baud = argc > 14 ? std::max(1, std::atoi(argv[14])) : config.lidar_baud;
    config.lidar_offset_deg = argc > 15 ? static_cast<float>(std::atof(argv[15])) : config.lidar_offset_deg;
    config.lidar_fov_deg = argc > 16 ? static_cast<float>(std::atof(argv[16])) : config.lidar_fov_deg;
    config.lidar_window_half_deg = argc > 17 ? std::max(0.5F, static_cast<float>(std::atof(argv[17]))) : config.lidar_window_half_deg;
    config.lidar_min_dist_m = argc > 18 ? std::max(0.01F, static_cast<float>(std::atof(argv[18]))) : config.lidar_min_dist_m;
    config.lidar_max_dist_m = argc > 19
        ? std::max(config.lidar_min_dist_m + 0.1F, static_cast<float>(std::atof(argv[19])))
        : config.lidar_max_dist_m;
    config.lidar_max_age_ms = argc > 20
        ? static_cast<std::uint64_t>(std::max(20, std::atoi(argv[20])))
        : config.lidar_max_age_ms;
    config.publish_mode = argc > 21 ? argv[21] : config.publish_mode;
    config.webrtc_url = argc > 22 ? argv[22] : config.webrtc_url;
    config.swap_uv = envEnabled("RK3588_YUV_SWAP_UV");
    config.forced_422_fourcc = forced422FourccFromEnv();
    config.debug_video_hud = envEnabled("RK3588_DEBUG_VIDEO_HUD");
    config.telemetry_path = envStringOrEmpty("RK3588_TELEMETRY_PATH");
    config.telemetry_interval_ms = envUintOr("RK3588_TELEMETRY_INTERVAL_MS", config.telemetry_interval_ms);
    config.calibration_profile_path = envStringOrEmpty("RK3588_CALIBRATION_PROFILE");

    auto pipeline = rk3588::modules::PipelineFactory::makePerception(config);
    return pipeline.run();
}
