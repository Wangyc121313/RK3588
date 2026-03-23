#pragma once

#include <cstddef>
#include <cstdint>

namespace rk3588::modules {

struct EncodedFramePacket {
    const void* data = nullptr;
    std::size_t size = 0;
    std::uint64_t dts_ms = 0;
    std::uint64_t pts_ms = 0;
    bool key_frame = false;
};

}  // namespace rk3588::modules
