#pragma once

#include <memory>
#include <vector>

#include "video/stream_publisher.hpp"

namespace rk3588::modules {

class PublisherHub {
public:
    void addPublisher(std::shared_ptr<StreamPublisher> publisher);
    bool startAll();
    void publishAll(const EncodedFramePacket& packet);
    void stopAll();

private:
    std::vector<std::shared_ptr<StreamPublisher>> publishers_;
};

}  // namespace rk3588::modules
