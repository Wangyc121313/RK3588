#include "video/webrtc_publisher.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

#ifdef RK_HAS_ZLM
extern "C" {
#include "mk_h264_splitter.h"
#include "mk_mediakit.h"
}
#endif

namespace rk3588::modules {

#ifdef RK_HAS_ZLM
namespace {

struct ParsedRtcUrl {
    std::string host = "127.0.0.1";
    std::uint16_t port = 8000;
    std::string app = "live";
    std::string stream = "camera";
};

std::string trimSlashes(std::string s) {
    while (!s.empty() && s.front() == '/') {
        s.erase(s.begin());
    }
    while (!s.empty() && s.back() == '/') {
        s.pop_back();
    }
    return s;
}

std::uint16_t envPortOr(const char* name, std::uint16_t fallback) {
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return fallback;
    }
    const int port = std::atoi(env);
    if (port <= 0 || port > 65535) {
        return fallback;
    }
    return static_cast<std::uint16_t>(port);
}

ParsedRtcUrl parseRtcUrl(const std::string& rtc_url) {
    ParsedRtcUrl out;
    if (rtc_url.empty()) {
        return out;
    }

    std::string rest = rtc_url;
    const std::string rtc_schema = "rtc://";
    const std::string webrtc_schema = "webrtc://";
    if (rest.rfind(rtc_schema, 0) == 0) {
        rest = rest.substr(rtc_schema.size());
    } else if (rest.rfind(webrtc_schema, 0) == 0) {
        rest = rest.substr(webrtc_schema.size());
    }

    std::string host_port;
    std::string path;
    const auto slash = rest.find('/');
    if (slash == std::string::npos) {
        host_port = rest;
    } else {
        host_port = rest.substr(0, slash);
        path = rest.substr(slash + 1);
    }

    if (!host_port.empty()) {
        const auto colon = host_port.rfind(':');
        if (colon != std::string::npos && colon + 1 < host_port.size()) {
            const std::string host = host_port.substr(0, colon);
            const std::string port_str = host_port.substr(colon + 1);
            if (!host.empty()) {
                out.host = host;
            }
            const int port = std::atoi(port_str.c_str());
            if (port > 0 && port <= 65535) {
                out.port = static_cast<std::uint16_t>(port);
            }
        } else {
            out.host = host_port;
        }
    }

    path = trimSlashes(path);
    if (!path.empty()) {
        const auto p = path.find('/');
        if (p == std::string::npos) {
            out.stream = path;
        } else {
            out.app = path.substr(0, p);
            out.stream = trimSlashes(path.substr(p + 1));
            if (out.app.empty()) {
                out.app = "live";
            }
            if (out.stream.empty()) {
                out.stream = "camera";
            }
        }
    }

    return out;
}

struct WebRtcPublisherImpl {
    mk_media media = nullptr;
    mk_h264_splitter splitter = nullptr;
    ParsedRtcUrl parsed;
    std::uint16_t http_port = 8080;
    std::uint16_t signaling_port = 10000;
    std::uint16_t rtc_port = 8000;
    std::uint16_t ice_port = 8000;
    std::uint64_t cur_dts_ms = 0;
    std::uint64_t cur_pts_ms = 0;
    bool started = false;
};

static void API_CALL onH264Frame(void* user_data, mk_h264_splitter /*splitter*/, const char* frame, int size) {
    if (user_data == nullptr || frame == nullptr || size <= 0) {
        return;
    }
    auto* impl = reinterpret_cast<WebRtcPublisherImpl*>(user_data);
    if (impl->media == nullptr) {
        return;
    }

    mk_frame out = mk_frame_create(
        MKCodecH264,
        impl->cur_dts_ms,
        impl->cur_pts_ms,
        frame,
        static_cast<size_t>(size),
        nullptr,
        nullptr);
    if (out == nullptr) {
        return;
    }

    (void)mk_media_input_frame(impl->media, out);
    mk_frame_unref(out);
}

}  // namespace
#endif

struct WebRtcPublisher::Impl {
#ifdef RK_HAS_ZLM
    WebRtcPublisherImpl zlm;
#endif
};

WebRtcPublisher::WebRtcPublisher(std::string rtc_url,
                                 std::uint32_t width,
                                 std::uint32_t height,
                                 std::uint32_t fps)
    : rtc_url_(std::move(rtc_url)), width_(width), height_(height), fps_(fps) {}

void WebRtcPublisher::setVideoConfig(std::uint32_t width, std::uint32_t height, std::uint32_t fps) {
    width_ = width;
    height_ = height;
    fps_ = fps;
}

bool WebRtcPublisher::start() {
#ifndef RK_HAS_ZLM
    std::cerr << "webrtc publisher unavailable: build without ZLMediaKit support\n";
    return false;
#else
    stop();
    if (width_ == 0 || height_ == 0 || fps_ == 0) {
        std::cerr << "webrtc publisher start failed: invalid video config\n";
        return false;
    }

    impl_ = new Impl();
    auto& z = impl_->zlm;
    z.parsed = parseRtcUrl(rtc_url_);
    z.http_port = envPortOr("RK3588_WEBRTC_HTTP_PORT", 8080);
    z.signaling_port = envPortOr("RK3588_WEBRTC_SIGNALING_PORT", 10000);
    z.rtc_port = envPortOr("RK3588_WEBRTC_RTC_PORT", z.parsed.port);
    z.ice_port = envPortOr("RK3588_WEBRTC_ICE_PORT", z.rtc_port);

    mk_config config = {};
    config.thread_num = 0;
    config.log_level = 0;
    config.log_mask = LOG_CONSOLE;
    config.log_file_path = nullptr;
    config.log_file_days = 0;
    config.ini_is_path = 1;
    config.ini = nullptr;
    config.ssl_is_path = 1;
    config.ssl = nullptr;
    config.ssl_pwd = nullptr;
    mk_env_init(&config);

    if (mk_rtc_server_start(z.rtc_port) == 0) {
        std::cerr << "webrtc rtc server start failed, port=" << z.rtc_port << '\n';
        stop();
        return false;
    }
    if (mk_ice_server_start(z.ice_port) == 0) {
        std::cerr << "webrtc ice server start failed, port=" << z.ice_port << '\n';
        stop();
        return false;
    }
    if (mk_signaling_server_start(z.signaling_port, 0) == 0) {
        std::cerr << "webrtc signaling server start failed, port=" << z.signaling_port << '\n';
        stop();
        return false;
    }
    if (mk_http_server_start(z.http_port, 0) == 0) {
        std::cerr << "webrtc http server start failed, port=" << z.http_port << '\n';
        stop();
        return false;
    }

    mk_ini media_option = mk_ini_create();
    if (media_option == nullptr) {
        stop();
        return false;
    }
    mk_ini_set_option_int(media_option, "enable_rtsp", 0);
    mk_ini_set_option_int(media_option, "enable_rtmp", 0);
    mk_ini_set_option_int(media_option, "enable_hls", 0);
    mk_ini_set_option_int(media_option, "enable_hls_fmp4", 0);
    mk_ini_set_option_int(media_option, "enable_ts", 0);
    mk_ini_set_option_int(media_option, "enable_fmp4", 0);
    mk_ini_set_option_int(media_option, "enable_mp4", 0);
    mk_ini_set_option_int(media_option, "enable_audio", 0);

    z.media = mk_media_create2(
        "__defaultVhost__",
        z.parsed.app.c_str(),
        z.parsed.stream.c_str(),
        0,
        media_option);
    mk_ini_release(media_option);
    if (z.media == nullptr) {
        stop();
        return false;
    }

    codec_args args = {};
    args.video.width = static_cast<int>(width_);
    args.video.height = static_cast<int>(height_);
    args.video.fps = static_cast<int>(fps_);
    mk_track track = mk_track_create(MKCodecH264, &args);
    if (track == nullptr) {
        stop();
        return false;
    }
    mk_media_init_track(z.media, track);
    mk_media_init_complete(z.media);
    mk_track_unref(track);

    z.splitter = mk_h264_splitter_create(onH264Frame, &z, 0);
    if (z.splitter == nullptr) {
        stop();
        return false;
    }

    z.started = true;
    return true;
#endif
}

bool WebRtcPublisher::publish(const EncodedFramePacket& packet) {
#ifndef RK_HAS_ZLM
    (void)packet;
    return false;
#else
    if (impl_ == nullptr || !impl_->zlm.started || impl_->zlm.splitter == nullptr) {
        return false;
    }
    if (packet.data == nullptr || packet.size == 0) {
        return false;
    }
    impl_->zlm.cur_dts_ms = packet.dts_ms;
    impl_->zlm.cur_pts_ms = packet.pts_ms;
    mk_h264_splitter_input_data(
        impl_->zlm.splitter,
        reinterpret_cast<const char*>(packet.data),
        static_cast<int>(packet.size));
    return true;
#endif
}

void WebRtcPublisher::stop() {
#ifdef RK_HAS_ZLM
    if (impl_ == nullptr) {
        return;
    }

    if (impl_->zlm.splitter != nullptr) {
        mk_h264_splitter_release(impl_->zlm.splitter);
        impl_->zlm.splitter = nullptr;
    }
    if (impl_->zlm.media != nullptr) {
        mk_media_release(impl_->zlm.media);
        impl_->zlm.media = nullptr;
    }
    mk_stop_all_server();

    delete impl_;
    impl_ = nullptr;
#endif
}

std::string WebRtcPublisher::rtcPlayUrl() const {
#ifndef RK_HAS_ZLM
    return "";
#else
    if (impl_ == nullptr) {
        const auto parsed = parseRtcUrl(rtc_url_);
        return std::string("rtc://") + parsed.host + ":" + std::to_string(parsed.port) + "/" + parsed.app + "/" +
               parsed.stream;
    }
    const auto& z = impl_->zlm;
    return std::string("rtc://") + z.parsed.host + ":" + std::to_string(z.rtc_port) + "/" + z.parsed.app + "/" +
           z.parsed.stream;
#endif
}

std::string WebRtcPublisher::sdpApiUrl() const {
#ifndef RK_HAS_ZLM
    return "";
#else
    if (impl_ == nullptr) {
        return "";
    }
    const auto& z = impl_->zlm;
    return std::string("http://") + z.parsed.host + ":" + std::to_string(z.http_port) +
           "/index/api/webrtc?type=play&app=" + z.parsed.app + "&stream=" + z.parsed.stream;
#endif
}

}  // namespace rk3588::modules
