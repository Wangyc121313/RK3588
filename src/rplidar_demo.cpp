#include <cstdlib>
#include <limits>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

namespace {

class DriverDeleter {
public:
    void operator()(sl::ILidarDriver* driver) const {
        if (driver != nullptr) {
            delete driver;
        }
    }
};

class ChannelDeleter {
public:
    void operator()(sl::IChannel* channel) const {
        delete channel;
    }
};

float angleDegrees(const sl_lidar_response_measurement_node_hq_t& node) {
    return static_cast<float>(node.angle_z_q14) * 90.0f / 16384.0f;
}

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
        return 1;
    }

    std::cout << "RPLIDAR connected on " << port << '\n';
    std::cout << "firmware: " << static_cast<int>(device_info.firmware_version >> 8)
              << '.' << static_cast<int>(device_info.firmware_version & 0xFF) << '\n';
    std::cout << "hardware: " << static_cast<int>(device_info.hardware_version) << '\n';

    result = driver->startScan(0, 1);
    if (SL_IS_FAIL(result)) {
        std::cerr << "startScan failed, code=" << result << '\n';
        return 1;
    }

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t node_count = sizeof(nodes) / sizeof(nodes[0]);

    result = driver->grabScanDataHq(nodes, node_count);
    if (SL_IS_FAIL(result)) {
        std::cerr << "grabScanDataHq failed, code=" << result << '\n';
        driver->stop();
        result = driver->setMotorSpeed(0);
        if (SL_IS_FAIL(result)) {
            std::cerr << "warning: setMotorSpeed(0) failed, code=" << result << '\n';
        }
        driver->disconnect();
        driver.reset();
        return 1;
    }

    driver->ascendScanData(nodes, node_count);

    float min_angle = std::numeric_limits<float>::max();
    float max_angle = std::numeric_limits<float>::lowest();
    float min_distance = std::numeric_limits<float>::max();
    float max_distance = 0.0f;
    int valid_count = 0;
    for (size_t index = 0; index < node_count; ++index) {
        if (!isValidPoint(nodes[index])) {
            continue;
        }

        const float angle = angleDegrees(nodes[index]);
        const float dist = distanceMeters(nodes[index]);
        min_angle = std::min(min_angle, angle);
        max_angle = std::max(max_angle, angle);
        min_distance = std::min(min_distance, dist);
        max_distance = std::max(max_distance, dist);
        ++valid_count;
    }

    if (valid_count > 0) {
        std::cout << std::fixed << std::setprecision(2)
                  << "full scan summary: valid_points=" << valid_count
                  << " angle_range=[" << min_angle << ", " << max_angle << "] deg"
                  << " distance_range=[" << min_distance << ", " << max_distance << "] m" << '\n';
    }

    std::cout << "scan sample points:" << '\n';
    int printed = 0;
    for (size_t index = 0; index < node_count && printed < 20; ++index) {
        if (!isValidPoint(nodes[index])) {
            continue;
        }

        std::cout << std::fixed << std::setprecision(2)
                  << "angle=" << std::setw(7) << angleDegrees(nodes[index]) << " deg"
                  << "  distance=" << std::setw(5) << distanceMeters(nodes[index]) << " m"
                  << "  quality=" << static_cast<int>(nodes[index].quality) << '\n';
        ++printed;
    }

    if (printed == 0) {
        std::cout << "no valid points received; check power, motor spin, and baudrate" << '\n';
    }

    driver->stop();
    result = driver->setMotorSpeed(0);
    if (SL_IS_FAIL(result)) {
        std::cerr << "warning: setMotorSpeed(0) failed, code=" << result << '\n';
    }
    driver->disconnect();
    driver.reset();
    return 0;
}