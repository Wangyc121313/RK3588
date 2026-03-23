#pragma once

#include "lidar/lidar_types.hpp"

namespace rk3588::modules {

class LidarAdapter {
public:
    PointCloudPacket adapt(const PointCloudPacket& in) const;
};

}  // namespace rk3588::modules
