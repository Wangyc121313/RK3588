#include "lidar/lidar_adapter.hpp"

#include <algorithm>

#include "fusion/sensor_fusion.hpp"

namespace rk3588::modules {

LidarAdapter::LidarAdapter(LidarAdapterConfig config) : config_(config) {}

void LidarAdapter::setConfig(const LidarAdapterConfig& config) {
    config_ = config;
}

const LidarAdapterConfig& LidarAdapter::config() const {
    return config_;
}

PointCloudPacket LidarAdapter::adapt(const PointCloudPacket& in) const {
    ++stats_.adapt_calls;
    stats_.input_points += static_cast<std::uint64_t>(in.points.size());

    if (!config_.normalize_angle_to_360 && !config_.enforce_distance_window) {
        stats_.output_points += static_cast<std::uint64_t>(in.points.size());
        return in;
    }

    PointCloudPacket out;
    out.scan_id = in.scan_id;
    out.timestamp_ms = in.timestamp_ms;
    out.points.reserve(in.points.size());

    for (const auto& point : in.points) {
        if (config_.enforce_distance_window) {
            if (point.distance_m < config_.min_distance_m || point.distance_m > config_.max_distance_m) {
                ++stats_.dropped_by_distance;
                continue;
            }
        }

        LidarPoint adapted = point;
        if (config_.normalize_angle_to_360) {
            adapted.angle_deg = SensorFusion::wrap360(adapted.angle_deg);
        }
        out.points.push_back(adapted);
    }

    stats_.output_points += static_cast<std::uint64_t>(out.points.size());
    return out;
}

void LidarAdapter::resetStats() {
    stats_ = {};
}

LidarAdapterStats LidarAdapter::stats() const {
    return stats_;
}

}  // namespace rk3588::modules
