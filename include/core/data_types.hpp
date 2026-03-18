#pragma once

#include <cstdint>
#include <vector>

namespace rk3588::core {

struct FramePacket {
    std::uint64_t frame_id = 0;
    std::uint64_t timestamp_ms = 0;
    std::vector<std::uint8_t> pixels;
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
