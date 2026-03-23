#include "lidar/lidar_adapter.hpp"

namespace rk3588::modules {

PointCloudPacket LidarAdapter::adapt(const PointCloudPacket& in) const {
    return in;
}

}  // namespace rk3588::modules
