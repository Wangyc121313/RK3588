#pragma once

#include <cstdint>
#include <string>

namespace rk3588::modules {

struct ZlmRtspPublisherImpl;

class ZlmRtspPublisher {
public:
    ZlmRtspPublisher() = default;
    ~ZlmRtspPublisher();

    ZlmRtspPublisher(const ZlmRtspPublisher&) = delete;
    ZlmRtspPublisher& operator=(const ZlmRtspPublisher&) = delete;

    bool start(const std::string& rtsp_url,
               std::uint32_t width,
               std::uint32_t height,
               std::uint32_t fps);

    bool pushPacket(const void* data, std::size_t bytes, std::uint64_t dts_ms, std::uint64_t pts_ms);
    void stop();

    std::string publishUrl() const;

private:
    ZlmRtspPublisherImpl* impl_ = nullptr;
};

}  // namespace rk3588::modules
