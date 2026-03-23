#pragma once

#include <string>

#include "video/encoded_packet.hpp"

namespace rk3588::modules {

class StreamPublisher {
public:
    virtual ~StreamPublisher() = default;

    virtual bool start() = 0;
    virtual bool publish(const EncodedFramePacket& packet) = 0;
    virtual void stop() = 0;
    virtual const char* name() const = 0;
};

}  // namespace rk3588::modules
