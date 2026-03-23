#pragma once

#include <cstdint>
#include <string>

#include "video/stream_publisher.hpp"

namespace rk3588::modules {

class WebRtcPublisher final : public StreamPublisher {
public:
    explicit WebRtcPublisher(std::string rtc_url,
                             std::uint32_t width = 0,
                             std::uint32_t height = 0,
                             std::uint32_t fps = 0);

    void setVideoConfig(std::uint32_t width, std::uint32_t height, std::uint32_t fps);

    bool start() override;
    bool publish(const EncodedFramePacket& packet) override;
    void stop() override;
    const char* name() const override { return "webrtc"; }

    std::string rtcPlayUrl() const;
    std::string sdpApiUrl() const;

private:
    struct Impl;

    std::string rtc_url_;
    std::uint32_t width_ = 0;
    std::uint32_t height_ = 0;
    std::uint32_t fps_ = 0;
    Impl* impl_ = nullptr;
};

}  // namespace rk3588::modules
