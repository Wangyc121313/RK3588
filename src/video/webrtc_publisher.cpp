#include "video/webrtc_publisher.hpp"

#include <iostream>

namespace rk3588::modules {

WebRtcPublisher::WebRtcPublisher(std::string signaling_url)
    : signaling_url_(std::move(signaling_url)) {}

bool WebRtcPublisher::start() {
    started_ = true;
    std::cout << "webrtc publisher started (stub), signaling_url=" << signaling_url_ << '\n';
    return true;
}

bool WebRtcPublisher::publish(const EncodedFramePacket& packet) {
    (void)packet;
    return started_;
}

void WebRtcPublisher::stop() {
    started_ = false;
}

}  // namespace rk3588::modules
