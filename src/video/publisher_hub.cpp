#include "video/publisher_hub.hpp"

namespace rk3588::modules {

void PublisherHub::addPublisher(std::shared_ptr<StreamPublisher> publisher) {
    if (publisher) {
        publishers_.push_back(std::move(publisher));
    }
}

bool PublisherHub::startAll() {
    bool ok = true;
    for (const auto& publisher : publishers_) {
        ok = publisher->start() && ok;
    }
    return ok;
}

void PublisherHub::publishAll(const EncodedFramePacket& packet) {
    for (const auto& publisher : publishers_) {
        (void)publisher->publish(packet);
    }
}

void PublisherHub::stopAll() {
    for (const auto& publisher : publishers_) {
        publisher->stop();
    }
}

}  // namespace rk3588::modules
