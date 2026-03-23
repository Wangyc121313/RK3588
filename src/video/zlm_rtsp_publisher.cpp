#include "video/zlm_rtsp_publisher.hpp"

#include <cstdlib>
#include <string>

extern "C" {
#include "mk_h264_splitter.h"
#include "mk_mediakit.h"
}

namespace rk3588::modules {

namespace {

struct RtspParsedUrl {
    std::string host = "127.0.0.1";
    std::uint16_t port = 8554;
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

RtspParsedUrl parseRtspUrl(const std::string& rtsp_url) {
    RtspParsedUrl out;
    if (rtsp_url.empty()) {
        return out;
    }

    std::string rest = rtsp_url;
    const std::string schema = "rtsp://";
    if (rest.rfind(schema, 0) == 0) {
        rest = rest.substr(schema.size());
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
            out.app = "live";
            out.stream = path;
        } else {
            out.app = path.substr(0, p);
            out.stream = path.substr(p + 1);
            out.stream = trimSlashes(out.stream);
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

}  // namespace

struct ZlmRtspPublisherImpl {
    mk_media media = nullptr;
    mk_h264_splitter splitter = nullptr;
    RtspParsedUrl parsed;
    std::uint64_t cur_dts_ms = 0;
    std::uint64_t cur_pts_ms = 0;
    bool started = false;
};

static void API_CALL onH264Frame(void* user_data, mk_h264_splitter /*splitter*/, const char* frame, int size) {
    if (user_data == nullptr || frame == nullptr || size <= 0) {
        return;
    }
    auto* impl = reinterpret_cast<ZlmRtspPublisherImpl*>(user_data);
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

ZlmRtspPublisher::~ZlmRtspPublisher() {
    stop();
}

bool ZlmRtspPublisher::start(const std::string& rtsp_url,
                             std::uint32_t width,
                             std::uint32_t height,
                             std::uint32_t fps) {
    stop();

    if (width == 0 || height == 0 || fps == 0) {
        return false;
    }

    impl_ = new ZlmRtspPublisherImpl();
    impl_->parsed = parseRtspUrl(rtsp_url);

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

    if (mk_rtsp_server_start(impl_->parsed.port, 0) == 0) {
        stop();
        return false;
    }

    mk_ini media_option = mk_ini_create();
    if (media_option == nullptr) {
        stop();
        return false;
    }
    mk_ini_set_option_int(media_option, "enable_rtsp", 1);
    mk_ini_set_option_int(media_option, "enable_rtmp", 0);
    mk_ini_set_option_int(media_option, "enable_hls", 0);
    mk_ini_set_option_int(media_option, "enable_hls_fmp4", 0);
    mk_ini_set_option_int(media_option, "enable_ts", 0);
    mk_ini_set_option_int(media_option, "enable_fmp4", 0);
    mk_ini_set_option_int(media_option, "enable_mp4", 0);
    mk_ini_set_option_int(media_option, "enable_audio", 0);

    impl_->media = mk_media_create2(
        "__defaultVhost__",
        impl_->parsed.app.c_str(),
        impl_->parsed.stream.c_str(),
        0,
        media_option);
    mk_ini_release(media_option);
    if (impl_->media == nullptr) {
        stop();
        return false;
    }

    codec_args args = {};
    args.video.width = static_cast<int>(width);
    args.video.height = static_cast<int>(height);
    args.video.fps = static_cast<int>(fps);
    mk_track track = mk_track_create(MKCodecH264, &args);
    if (track == nullptr) {
        stop();
        return false;
    }
    mk_media_init_track(impl_->media, track);
    mk_media_init_complete(impl_->media);
    mk_track_unref(track);

    impl_->splitter = mk_h264_splitter_create(onH264Frame, impl_, 0);
    if (impl_->splitter == nullptr) {
        stop();
        return false;
    }

    impl_->started = true;
    return true;
}

bool ZlmRtspPublisher::pushPacket(const void* data,
                                  std::size_t bytes,
                                  std::uint64_t dts_ms,
                                  std::uint64_t pts_ms) {
    if (impl_ == nullptr || !impl_->started || impl_->splitter == nullptr || data == nullptr || bytes == 0) {
        return false;
    }

    impl_->cur_dts_ms = dts_ms;
    impl_->cur_pts_ms = pts_ms;
    mk_h264_splitter_input_data(impl_->splitter, reinterpret_cast<const char*>(data), static_cast<int>(bytes));
    return true;
}

void ZlmRtspPublisher::stop() {
    if (impl_ == nullptr) {
        return;
    }

    if (impl_->splitter != nullptr) {
        mk_h264_splitter_release(impl_->splitter);
        impl_->splitter = nullptr;
    }

    if (impl_->media != nullptr) {
        mk_media_release(impl_->media);
        impl_->media = nullptr;
    }

    mk_stop_all_server();

    delete impl_;
    impl_ = nullptr;
}

std::string ZlmRtspPublisher::publishUrl() const {
    if (impl_ == nullptr) {
        return "";
    }
    return std::string("rtsp://") + impl_->parsed.host + ":" + std::to_string(impl_->parsed.port) + "/" +
           impl_->parsed.app + "/" + impl_->parsed.stream;
}

}  // namespace rk3588::modules
