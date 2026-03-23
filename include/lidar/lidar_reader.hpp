#pragma once

#include <optional>

#include "lidar/lidar_types.hpp"

namespace rk3588::modules {

class LidarReader {
public:
    bool start();
    void stop();
    std::optional<PointCloudPacket> poll();
};

}  // namespace rk3588::modules
