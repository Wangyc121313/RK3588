#pragma once

#include <cstdint>
#include <vector>

namespace rk3588::core {

struct FramePacket {
    std::uint64_t frame_id = 0;
    std::uint64_t timestamp_ms = 0;
    std::int64_t pts_us = -1;
    std::int64_t dts_us = -1;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t hor_stride = 0;
    std::uint32_t ver_stride = 0;
    std::uint32_t pixel_format = 0;
    std::uint32_t buffer_index = 0;
    std::uint32_t buffer_size = 0;
    int dma_fd = -1;
    std::uintptr_t cpu_addr = 0;
};

struct LidarPoint {
    float angle_deg = 0.0f;
    float distance_m = 0.0f;
};

struct PointCloudPacket {
    std::uint64_t scan_id = 0;
    std::uint64_t timestamp_ms = 0;
    std::vector<LidarPoint> points;
};

}  // namespace rk3588::core
