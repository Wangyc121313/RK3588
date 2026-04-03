#pragma once

#include <cstdint>

#include "lidar/lidar_types.hpp"

namespace rk3588::modules {

struct LidarAdapterConfig {
    bool normalize_angle_to_360 = false;
    bool enforce_distance_window = false;
    float min_distance_m = 0.15F;
    float max_distance_m = 20.0F;
};

struct LidarAdapterStats {
    std::uint64_t adapt_calls = 0;
    std::uint64_t input_points = 0;
    std::uint64_t output_points = 0;
    std::uint64_t dropped_by_distance = 0;
};

class LidarAdapter {
public:
    explicit LidarAdapter(LidarAdapterConfig config = {});

    void setConfig(const LidarAdapterConfig& config);
    [[nodiscard]] const LidarAdapterConfig& config() const;

    PointCloudPacket adapt(const PointCloudPacket& in) const;

    void resetStats();
    [[nodiscard]] LidarAdapterStats stats() const;

private:
    LidarAdapterConfig config_;
    mutable LidarAdapterStats stats_;
};

}  // namespace rk3588::modules
