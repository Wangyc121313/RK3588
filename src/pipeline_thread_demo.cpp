#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>
#include <thread>

#include "core/bounded_queue.hpp"
#include "core/data_types.hpp"
#include "core/lidar_ring_buffer.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using namespace rk3588::core;

std::uint64_t nowMs(const Clock::time_point start) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count());
}

float minDistance(const PointCloudPacket& cloud) {
    float min_dist = 9999.0f;
    for (const auto& point : cloud.points) {
        if (point.distance_m > 0.05f && point.distance_m < min_dist) {
            min_dist = point.distance_m;
        }
    }
    return min_dist < 9999.0f ? min_dist : -1.0f;
}

}  // namespace

int main(int argc, char* argv[]) {
    const int run_seconds = argc > 1 ? std::max(1, std::atoi(argv[1])) : 10;
    constexpr std::uint64_t sync_threshold_ms = 150;

    BoundedQueue<FramePacket> frame_queue(4);
    LidarRingBuffer lidar_buffer;
    std::atomic<bool> stop {false};

    const auto t0 = Clock::now();

    std::thread camera_thread([&] {
        std::uint64_t frame_id = 0;
        while (!stop.load()) {
            FramePacket frame;
            frame.frame_id = frame_id++;
            frame.timestamp_ms = nowMs(t0);
            frame.pixels.resize(640 * 480);  // Mock payload keeps threading behavior realistic.

            frame_queue.push(std::move(frame));
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
    });

    std::thread lidar_thread([&] {
        std::uint64_t scan_id = 0;
        std::mt19937 rng(42);
        std::uniform_real_distribution<float> noise(-0.02f, 0.02f);

        while (!stop.load()) {
            PointCloudPacket cloud;
            cloud.scan_id = scan_id++;
            cloud.timestamp_ms = nowMs(t0);
            cloud.points.reserve(360);

            for (int angle = 0; angle < 360; ++angle) {
                LidarPoint point;
                point.angle_deg = static_cast<float>(angle);
                point.distance_m = 1.0f + noise(rng);
                cloud.points.push_back(point);
            }

            lidar_buffer.write(std::move(cloud));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    std::thread worker_thread([&] {
        while (!stop.load()) {
            FramePacket frame;
            if (!frame_queue.pop_for(frame, std::chrono::milliseconds(100))) {
                continue;
            }

            PointCloudPacket matched;
            std::uint64_t delta_ms = 0;
            if (!lidar_buffer.readClosest(frame.timestamp_ms, matched, delta_ms)) {
                std::cout << "[worker] frame=" << frame.frame_id << " ts=" << frame.timestamp_ms
                          << "ms no-lidar-data" << '\n';
                continue;
            }

            const bool synced = delta_ms <= sync_threshold_ms;
            const float min_dist = minDistance(matched);
            std::cout << "[worker] frame=" << std::setw(4) << frame.frame_id
                      << " frame_ts=" << std::setw(5) << frame.timestamp_ms << "ms"
                      << " matched_scan=" << std::setw(4) << matched.scan_id
                      << " scan_ts=" << std::setw(5) << matched.timestamp_ms << "ms"
                      << " delta=" << std::setw(3) << delta_ms << "ms"
                      << " sync=" << (synced ? "ok" : "bad")
                      << " min_dist=" << std::fixed << std::setprecision(2) << min_dist << "m"
                      << '\n';
        }
    });

    std::this_thread::sleep_for(std::chrono::seconds(run_seconds));
    stop = true;
    frame_queue.close();

    camera_thread.join();
    lidar_thread.join();
    worker_thread.join();

    std::cout << "pipeline_thread_demo finished after " << run_seconds << "s" << '\n';
    return 0;
}
