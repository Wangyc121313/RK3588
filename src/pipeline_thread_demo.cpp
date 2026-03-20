#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <random>
#include <thread>

#include "core/bounded_queue.hpp"
#include "core/data_types.hpp"
#include "core/lidar_ring_buffer.hpp"
#include "modules/sensor_fusion.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using namespace rk3588::core;

std::uint64_t nowMs(const Clock::time_point start) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count());
}

}  // namespace

int main(int argc, char* argv[]) {
    const int run_seconds = argc > 1 ? std::max(1, std::atoi(argv[1])) : 10;
    constexpr std::uint64_t sync_threshold_ms = 150;
    constexpr float window_half_deg = 3.0F;

    BoundedQueue<FramePacket> frame_queue(4);
    LidarRingBuffer lidar_buffer;
    std::atomic<bool> stop {false};

    rk3588::modules::FusionConfig fusion_cfg;
    fusion_cfg.image_width = 640;
    fusion_cfg.camera_fov_deg = 60.0F;
    fusion_cfg.lidar_angle_offset_deg = 350.0F;
    fusion_cfg.default_window_half_deg = window_half_deg;
    fusion_cfg.min_valid_distance_m = 0.15F;
    rk3588::modules::SensorFusion fusion(fusion_cfg);
    const auto fov_range = fusion.cameraFovToLidarRange();

    std::cout << std::fixed << std::setprecision(2)
              << "[fusion] lidar FOV range left=" << fov_range.left_deg
              << " right=" << fov_range.right_deg
              << " crosses_zero=" << (fov_range.crosses_zero ? "yes" : "no")
              << " offset_deg=" << fusion_cfg.lidar_angle_offset_deg
              << '\n';

    const auto t0 = Clock::now();

    std::thread camera_thread([&] {
        std::uint64_t frame_id = 0;
        while (!stop.load()) {
            FramePacket frame;
            frame.frame_id = frame_id++;
            frame.timestamp_ms = nowMs(t0);
            frame.width = 640;
            frame.height = 480;
            frame.pixel_format = 0;
            frame.buffer_index = 0;
            frame.dma_fd = -1;

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

            const float center_px = static_cast<float>(frame.width) * 0.5F;
            const float left_px = static_cast<float>(frame.width) * 0.2F;
            const float right_px = static_cast<float>(frame.width) * 0.8F;

            const float center_angle = fusion.pixelToLidarAngle(center_px);
            const float left_angle = fusion.pixelToLidarAngle(left_px);
            const float right_angle = fusion.pixelToLidarAngle(right_px);

            const std::optional<float> center_dist = fusion.medianDistanceInAngleWindow(matched, center_angle, window_half_deg);
            const std::optional<float> left_dist = fusion.medianDistanceInAngleWindow(matched, left_angle, window_half_deg);
            const std::optional<float> right_dist = fusion.medianDistanceInAngleWindow(matched, right_angle, window_half_deg);

            std::cout << "[worker] frame=" << std::setw(4) << frame.frame_id
                      << " frame_ts=" << std::setw(5) << frame.timestamp_ms << "ms"
                      << " matched_scan=" << std::setw(4) << matched.scan_id
                      << " scan_ts=" << std::setw(5) << matched.timestamp_ms << "ms"
                      << " delta=" << std::setw(3) << delta_ms << "ms"
                      << " sync=" << (synced ? "ok" : "bad")
                      << " center_ang=" << std::fixed << std::setprecision(1) << center_angle
                      << " left_ang=" << left_angle
                      << " right_ang=" << right_angle
                      << " center_dist=" << std::setprecision(2) << (center_dist.has_value() ? *center_dist : -1.0F) << "m"
                      << " left_dist=" << (left_dist.has_value() ? *left_dist : -1.0F) << "m"
                      << " right_dist=" << (right_dist.has_value() ? *right_dist : -1.0F) << "m"
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
