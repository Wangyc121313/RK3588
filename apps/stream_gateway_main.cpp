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
    config.model_path = argc > 1 ? argv[1] : config.model_path;
    config.rtsp_url = argc > 2 ? argv[2] : "rtsp://127.0.0.1:8554/live";
    config.camera_device = argc > 3 ? argv[3] : config.camera_device;
    config.run_seconds = argc > 4 ? std::max(1, std::atoi(argv[4])) : 30;
    config.camera_width = argc > 5 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[5]))) : config.camera_width;
    config.camera_height = argc > 6 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[6]))) : config.camera_height;
    config.model_width = argc > 7 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[7]))) : config.model_width;
    config.model_height = argc > 8 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[8]))) : config.model_height;
    config.fps = argc > 9 ? std::max(1, std::atoi(argv[9])) : 25;
    config.labels_path = argc > 10 ? argv[10] : config.labels_path;
    config.publish_mode = argc > 11 ? argv[11] : "rtsp";
    config.webrtc_url = argc > 12 ? argv[12] : config.webrtc_url;
    config.dump_h264_path = argc > 13 ? argv[13] : config.dump_h264_path;
    config.infer_every_n_frames = argc > 14 ? std::max(1, std::atoi(argv[14])) : config.infer_every_n_frames;
    config.rtsp_listen_mode = envEnabled("RK3588_RTSP_LISTEN");
    config.swap_uv = envEnabled("RK3588_YUV_SWAP_UV");
    config.forced_422_fourcc = forced422FourccFromEnv();
    config.debug_video_hud = envEnabled("RK3588_DEBUG_VIDEO_HUD");
    config.telemetry_path = envStringOrEmpty("RK3588_TELEMETRY_PATH");
    config.telemetry_interval_ms = envUintOr("RK3588_TELEMETRY_INTERVAL_MS", config.telemetry_interval_ms);

    auto pipeline = rk3588::modules::PipelineFactory::makeStreaming(config);
    return pipeline.run();
}
