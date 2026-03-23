#include "lidar/lidar_reader.hpp"

namespace rk3588::modules {

bool LidarReader::start() { return true; }
void LidarReader::stop() {}
std::optional<PointCloudPacket> LidarReader::poll() { return std::nullopt; }

}  // namespace rk3588::modules
