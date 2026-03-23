#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>

#include "core/data_types.hpp"

namespace rk3588::core {

struct LidarMatchInfo {
    bool matched = false;
    std::uint64_t delta_ms = 0;
    std::uint64_t estimated_scan_period_ms = 0;
};

class LidarRingBuffer {
public:
    static constexpr std::size_t kCapacity = 5;

    void write(PointCloudPacket cloud) {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_[write_index_] = std::move(cloud);
        valid_[write_index_] = true;
        write_index_ = (write_index_ + 1) % kCapacity;
    }

    bool readClosest(std::uint64_t ts_ms, PointCloudPacket& out, std::uint64_t& delta_ms) const {
        LidarMatchInfo info;
        return readClosestWithInfo(ts_ms, out, info) && (delta_ms = info.delta_ms, true);
    }

    bool readClosestWithInfo(std::uint64_t ts_ms, PointCloudPacket& out, LidarMatchInfo& info) const {
        std::lock_guard<std::mutex> lock(mutex_);

        info = {};

        bool found = false;
        std::uint64_t best_delta = std::numeric_limits<std::uint64_t>::max();
        std::size_t best_index = 0;
        std::array<std::uint64_t, kCapacity> timestamps {};
        std::size_t timestamp_count = 0;

        for (std::size_t i = 0; i < kCapacity; ++i) {
            if (!valid_[i]) {
                continue;
            }

            timestamps[timestamp_count++] = buffer_[i].timestamp_ms;

            const auto cloud_ts = buffer_[i].timestamp_ms;
            const auto diff = (cloud_ts > ts_ms) ? (cloud_ts - ts_ms) : (ts_ms - cloud_ts);
            if (diff < best_delta) {
                best_delta = diff;
                best_index = i;
                found = true;
            }
        }

        if (!found) {
            return false;
        }

        out = buffer_[best_index];
        info.matched = true;
        info.delta_ms = best_delta;
        if (timestamp_count >= 2) {
            std::sort(timestamps.begin(), timestamps.begin() + static_cast<std::ptrdiff_t>(timestamp_count));
            std::uint64_t period_sum = 0;
            std::size_t period_count = 0;
            for (std::size_t i = 1; i < timestamp_count; ++i) {
                const std::uint64_t period = timestamps[i] - timestamps[i - 1];
                if (period > 0) {
                    period_sum += period;
                    ++period_count;
                }
            }
            if (period_count > 0) {
                info.estimated_scan_period_ms = period_sum / period_count;
            }
        }
        return true;
    }

private:
    mutable std::mutex mutex_;
    std::array<PointCloudPacket, kCapacity> buffer_ {};
    std::array<bool, kCapacity> valid_ {};
    std::size_t write_index_ = 0;
};

}  // namespace rk3588::core
