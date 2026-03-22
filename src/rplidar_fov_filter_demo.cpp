#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

namespace {

class DriverDeleter {
public:
    void operator()(sl::ILidarDriver* driver) const {
        delete driver;
    }
};

class ChannelDeleter {
public:
    void operator()(sl::IChannel* channel) const {
        delete channel;
    }
};

float wrapDeg360(float deg) {
    float out = std::fmod(deg, 360.0f);
    if (out < 0.0f) {
        out += 360.0f;
    }
    return out;
}

float wrapDeg180(float deg) {
    float out = std::fmod(deg + 180.0f, 360.0f);
    if (out < 0.0f) {
        out += 360.0f;
    }
    return out - 180.0f;
}

float lidarAngleDeg(const sl_lidar_response_measurement_node_hq_t& node) {
    return static_cast<float>(node.angle_z_q14) * 90.0f / 16384.0f;
}

float distanceMeters(const sl_lidar_response_measurement_node_hq_t& node) {
    return static_cast<float>(node.dist_mm_q2) / 4.0f / 1000.0f;
}

bool isValidPoint(const sl_lidar_response_measurement_node_hq_t& node) {
    return node.dist_mm_q2 != 0;
}

struct FilteredPoint {
    float lidar_deg;
    float cam_deg;
    float dist_m;
    int quality;
};

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog
              << " [port] [baud] [offset_deg] [fov_deg] [duration_s] [min_quality] [min_dist_m] [max_dist_m]\n"
              << "Defaults: /dev/ttyUSB0 115200 191.7 60 10 8 0.15 8.0\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string port = argc > 1 ? argv[1] : "/dev/ttyUSB0";
    const int baudrate = argc > 2 ? std::atoi(argv[2]) : 115200;
    const float offset_deg = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 191.7f;
    const float fov_deg = argc > 4 ? std::max(5.0f, static_cast<float>(std::atof(argv[4]))) : 60.0f;
    const int duration_s = argc > 5 ? std::max(1, std::atoi(argv[5])) : 10;
    const int min_quality = argc > 6 ? std::max(0, std::atoi(argv[6])) : 8;
    const float min_dist_m = argc > 7 ? std::max(0.01f, static_cast<float>(std::atof(argv[7]))) : 0.15f;
    const float max_dist_m = argc > 8 ? std::max(min_dist_m, static_cast<float>(std::atof(argv[8]))) : 8.0f;

    if (argc > 9) {
        printUsage(argv[0]);
        return 1;
    }

    auto driver_result = sl::createLidarDriver();
    if (!driver_result || *driver_result == nullptr) {
        std::cerr << "failed to create lidar driver\n";
        return 1;
    }
    std::unique_ptr<sl::ILidarDriver, DriverDeleter> driver(*driver_result);

    auto channel_result = sl::createSerialPortChannel(port, baudrate);
    if (!channel_result || *channel_result == nullptr) {
        std::cerr << "failed to create serial channel for " << port << '\n';
        return 1;
    }
    std::unique_ptr<sl::IChannel, ChannelDeleter> channel(*channel_result);

    sl_result result = driver->connect(channel.get());
    if (SL_IS_FAIL(result)) {
        std::cerr << "connect failed on " << port << ", code=" << result << '\n';
        return 1;
    }

    sl_lidar_response_device_info_t device_info {};
    result = driver->getDeviceInfo(device_info);
    if (SL_IS_FAIL(result)) {
        std::cerr << "getDeviceInfo failed, code=" << result << '\n';
        driver->disconnect();
        return 1;
    }

    std::cout << "RPLIDAR connected on " << port << '\n';
    std::cout << std::fixed << std::setprecision(2)
              << "offset_deg=" << wrapDeg360(offset_deg)
              << " fov_deg=" << fov_deg
              << " duration_s=" << duration_s
              << " quality>=" << min_quality
              << " dist_range=[" << min_dist_m << ", " << max_dist_m << "] m\n";

    result = driver->startScan(0, 1);
    if (SL_IS_FAIL(result)) {
        std::cerr << "startScan failed, code=" << result << '\n';
        driver->disconnect();
        return 1;
    }

    const float half_fov = fov_deg * 0.5f;
    const auto start = std::chrono::steady_clock::now();
    std::uint64_t scans = 0;
    std::uint64_t points_total = 0;
    std::uint64_t points_in_fov = 0;

    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        if (elapsed_s >= duration_s) {
            break;
        }

        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t node_count = sizeof(nodes) / sizeof(nodes[0]);

        result = driver->grabScanDataHq(nodes, node_count, 1000);
        if (SL_IS_FAIL(result)) {
            if (result == SL_RESULT_OPERATION_TIMEOUT) {
                continue;
            }
            std::cerr << "grabScanDataHq failed, code=" << result << '\n';
            break;
        }

        driver->ascendScanData(nodes, node_count);

        std::vector<FilteredPoint> current;
        current.reserve(node_count / 4);

        for (size_t i = 0; i < node_count; ++i) {
            if (!isValidPoint(nodes[i])) {
                continue;
            }

            const float dist = distanceMeters(nodes[i]);
            if (dist < min_dist_m || dist > max_dist_m) {
                continue;
            }

            const int quality = static_cast<int>(nodes[i].quality);
            if (quality < min_quality) {
                continue;
            }

            const float lidar_deg = wrapDeg360(lidarAngleDeg(nodes[i]));
            const float cam_deg = wrapDeg180(lidar_deg - offset_deg);

            ++points_total;
            if (std::fabs(cam_deg) > half_fov) {
                continue;
            }

            current.push_back(FilteredPoint {lidar_deg, cam_deg, dist, quality});
            ++points_in_fov;
        }

        ++scans;

        std::cout << "scan=" << scans
                  << " elapsed=" << elapsed_s << "s"
                  << " in_fov=" << current.size();

        if (!current.empty()) {
            auto nearest_it = std::min_element(current.begin(), current.end(),
                [](const FilteredPoint& a, const FilteredPoint& b) {
                    return a.dist_m < b.dist_m;
                });

            std::cout << " nearest={dist=" << nearest_it->dist_m
                      << "m cam_deg=" << nearest_it->cam_deg
                      << " lidar_deg=" << nearest_it->lidar_deg << "}";

            std::cout << " samples:";
            const size_t preview_count = std::min<std::size_t>(5, current.size());
            for (size_t j = 0; j < preview_count; ++j) {
                const auto& p = current[j];
                std::cout << " [d=" << p.dist_m
                          << " cam=" << p.cam_deg
                          << " lidar=" << p.lidar_deg << "]";
            }
        }
        std::cout << '\n';
    }

    driver->stop();
    driver->disconnect();

    const double ratio = points_total > 0
        ? static_cast<double>(points_in_fov) / static_cast<double>(points_total)
        : 0.0;

    std::cout << "\n=== FOV Filter Summary ===\n";
    std::cout << "scans=" << scans
              << " total_kept_after_quality_dist=" << points_total
              << " in_fov=" << points_in_fov
              << " ratio=" << std::setprecision(3) << ratio << '\n';
    std::cout << "camera_fov_deg=" << fov_deg
              << " => cam_angle_range=[" << -half_fov << ", " << half_fov << "]\n";

    return 0;
}
