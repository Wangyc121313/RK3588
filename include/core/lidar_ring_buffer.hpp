#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>

#include "core/data_types.hpp"

namespace rk3588::core {

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
        std::lock_guard<std::mutex> lock(mutex_);

        bool found = false;
        std::uint64_t best_delta = std::numeric_limits<std::uint64_t>::max();
        std::size_t best_index = 0;

        for (std::size_t i = 0; i < kCapacity; ++i) {
            if (!valid_[i]) {
                continue;
            }

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
        delta_ms = best_delta;
        return true;
    }

private:
    mutable std::mutex mutex_;
    std::array<PointCloudPacket, kCapacity> buffer_ {};
    std::array<bool, kCapacity> valid_ {};
    std::size_t write_index_ = 0;
};

}  // namespace rk3588::core
