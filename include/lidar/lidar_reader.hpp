#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "lidar/lidar_types.hpp"

namespace rk3588::modules {

class LidarReader {
public:
    struct Config {
        const char* port = "/dev/ttyUSB0";
        int baud = 115200;
        float min_distance_m = 0.15F;
        float max_distance_m = 20.0F;
    };

    LidarReader();
    ~LidarReader();

    bool start(const Config& config);
    void stop();
    std::optional<PointCloudPacket> poll(std::uint32_t timeout_ms = 200U);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    Config config_;
    std::uint64_t next_scan_id_ = 0;
    bool running_ = false;
};

}  // namespace rk3588::modules
