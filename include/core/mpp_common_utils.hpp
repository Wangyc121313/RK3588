#pragma once

#include <cstddef>
#include <cstdint>

namespace rk3588::core::mpp {

inline constexpr const char* kSystemDmaHeapPath = "/dev/dma_heap/system";

inline std::uint32_t align16(std::uint32_t value) {
    return (value + 15u) & ~15u;
}

// Scan backward for JPEG EOI marker (FFD9) to trim trailing driver padding bytes.
inline std::size_t findJpegEffectiveSize(const std::uint8_t* data, std::size_t len) {
    if (data == nullptr || len < 4) {
        return 0;
    }
    for (std::size_t i = len; i >= 2; --i) {
        if (data[i - 2] == 0xFF && data[i - 1] == 0xD9) {
            return i;
        }
    }
    return 0;
}

}  // namespace rk3588::core::mpp
