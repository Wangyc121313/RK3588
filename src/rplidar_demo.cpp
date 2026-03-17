#include <cstdlib>
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
    void operator()(sl::lidar::ILidarDriver* driver) const {
        if (driver != nullptr) {
            sl::lidar::ILidarDriver::DisposeDriver(driver);
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

    std::unique_ptr<sl::lidar::ILidarDriver, DriverDeleter> driver(sl::lidar::createLidarDriver());
    if (!driver) {
        std::cerr << "failed to create lidar driver\n";
        return 1;
    }

    std::unique_ptr<sl::IChannel, ChannelDeleter> channel(sl::createSerialPortChannel(port, baudrate));
    if (!channel) {
        std::cerr << "failed to create serial channel for " << port << '\n';
        return 1;
    }

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
        return 1;
    }

    driver->ascendScanData(nodes, node_count);

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
    return 0;
}