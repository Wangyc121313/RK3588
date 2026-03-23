#pragma once

#include <string>

#include "video/stream_publisher.hpp"

namespace rk3588::modules {

class WebRtcPublisher final : public StreamPublisher {
public:
    explicit WebRtcPublisher(std::string signaling_url);

    bool start() override;
    bool publish(const EncodedFramePacket& packet) override;
    void stop() override;
    const char* name() const override { return "webrtc"; }

private:
    std::string signaling_url_;
    bool started_ = false;
};

}  // namespace rk3588::modules
