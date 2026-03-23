#pragma once

#include <string>

namespace rk3588::modules {

enum class VideoPublishMode {
    Rtsp,
    WebRtc,
    Both
};

struct VideoRuntimeConfig {
    VideoPublishMode mode = VideoPublishMode::Rtsp;
    std::string rtsp_url = "rtsp://127.0.0.1:8554/live";
    std::string webrtc_signaling_url;
};

}  // namespace rk3588::modules
