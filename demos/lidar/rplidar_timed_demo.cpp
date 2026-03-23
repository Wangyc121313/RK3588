#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
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

float distanceMeters(const sl_lidar_response_measurement_node_hq_t& node) {
    return static_cast<float>(node.dist_mm_q2) / 4.0f / 1000.0f;
}

bool isValidPoint(const sl_lidar_response_measurement_node_hq_t& node) {
    return node.dist_mm_q2 != 0;
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string port = argc > 1 ? argv[1] : "/dev/ttyUSB0";
    const int baudrate = argc > 2 ? std::atoi(argv[2]) : 115200;
    const int duration_seconds = argc > 3 ? std::max(1, std::atoi(argv[3])) : 10;
    const int hold_seconds_after_stop = argc > 4 ? std::max(0, std::atoi(argv[4])) : 2;

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
        driver.reset();
        return 1;
    }

    std::cout << "RPLIDAR connected on " << port << '\n';
    std::cout << "firmware: " << static_cast<int>(device_info.firmware_version >> 8)
              << '.' << static_cast<int>(device_info.firmware_version & 0xFF) << '\n';
    std::cout << "hardware: " << static_cast<int>(device_info.hardware_version) << '\n';

    result = driver->startScan(0, 1);
    if (SL_IS_FAIL(result)) {
        std::cerr << "startScan failed, code=" << result << '\n';
        driver->disconnect();
        driver.reset();
        return 1;
    }

    std::cout << "running for " << duration_seconds << " s; lidar will auto-stop" << '\n';

    const auto start = std::chrono::steady_clock::now();
    auto stats_window_start = start;
    int sample_idx = 0;
    std::uint64_t window_scans = 0;
    std::uint64_t window_valid_points = 0;

    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        if (elapsed_s >= duration_seconds) {
            break;
        }

        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t node_count = sizeof(nodes) / sizeof(nodes[0]);

        result = driver->grabScanDataHq(nodes, node_count, 1000);
        if (SL_IS_FAIL(result) && result != SL_RESULT_OPERATION_TIMEOUT) {
            std::cerr << "grabScanDataHq failed, code=" << result << '\n';
            break;
        }

        if (SL_IS_OK(result)) {
            driver->ascendScanData(nodes, node_count);
        }

        int valid_points = 0;
        float min_distance = std::numeric_limits<float>::max();
        for (size_t i = 0; i < node_count; ++i) {
            if (!isValidPoint(nodes[i])) {
                continue;
            }
            ++valid_points;
            min_distance = std::min(min_distance, distanceMeters(nodes[i]));
        }

        std::cout << "sample " << sample_idx++
                  << " | elapsed=" << elapsed_s << "s"
                  << " | valid_points=" << valid_points;
        if (valid_points > 0) {
            std::cout << " | min_distance=" << min_distance << "m";
        }
        std::cout << '\n';

        ++window_scans;
        window_valid_points += static_cast<std::uint64_t>(valid_points);
        const auto stats_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - stats_window_start).count();
        if (stats_elapsed >= 1000) {
            const double window_sec = static_cast<double>(stats_elapsed) / 1000.0;
            const double scan_hz = static_cast<double>(window_scans) / window_sec;
            const double points_per_sec = static_cast<double>(window_valid_points) / window_sec;
            const double points_per_scan = (window_scans > 0)
                ? static_cast<double>(window_valid_points) / static_cast<double>(window_scans)
                : 0.0;
            const double rpm_est = scan_hz * 60.0;

            std::cout << "[rate] scan_hz=" << scan_hz
                      << " | rpm_est=" << rpm_est
                      << " | points_per_scan=" << points_per_scan
                      << " | points_per_sec=" << points_per_sec
                      << '\n';

            stats_window_start = now;
            window_scans = 0;
            window_valid_points = 0;
        }
    }

    driver->stop();
    result = driver->setMotorSpeed(0);
    if (SL_IS_FAIL(result)) {
        std::cerr << "warning: setMotorSpeed(0) failed, code=" << result << '\n';
    }

    auto* serial_channel = dynamic_cast<sl::ISerialPortChannel*>(channel.get());
    if (serial_channel != nullptr) {
        // For A1-class devices, DTR high usually maps to motor-off on USB serial adapters.
        serial_channel->setDTR(true);
    }

    if (hold_seconds_after_stop > 0) {
        std::cout << "holding serial channel for " << hold_seconds_after_stop
                  << " s to keep motor-off control line stable" << '\n';
        std::this_thread::sleep_for(std::chrono::seconds(hold_seconds_after_stop));
    }

    driver->disconnect();
    driver.reset();

    std::cout << "lidar scan stopped, motor stop requested, and disconnected" << '\n';
    return 0;
}
