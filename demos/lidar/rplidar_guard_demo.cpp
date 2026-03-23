#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include <sys/select.h>
#include <unistd.h>

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

class GuardedLidar {
public:
    GuardedLidar(std::unique_ptr<sl::ILidarDriver, DriverDeleter> driver,
                 std::unique_ptr<sl::IChannel, ChannelDeleter> channel,
                 int idle_timeout_s)
        : driver_(std::move(driver))
        , channel_(std::move(channel))
        , idle_timeout_s_(std::max(1, idle_timeout_s))
        , last_demand_(std::chrono::steady_clock::now()) {
    }

    ~GuardedLidar() {
        stopMotor();
        if (driver_) {
            driver_->disconnect();
            driver_.reset();
        }
    }

    bool ensureRunning() {
        if (running_) {
            return true;
        }

        const sl_result result = driver_->startScan(false, true);
        if (SL_IS_FAIL(result)) {
            std::cerr << "startScan failed, code=" << result << '\n';
            return false;
        }
        running_ = true;
        std::cout << "lidar started" << '\n';
        return true;
    }

    void stopMotor() {
        if (!driver_) {
            return;
        }

        if (running_) {
            driver_->stop();
        }
        const sl_result motor_result = driver_->setMotorSpeed(0);
        if (SL_IS_FAIL(motor_result)) {
            std::cerr << "warning: setMotorSpeed(0) failed, code=" << motor_result << '\n';
        }

        auto* serial = dynamic_cast<sl::ISerialPortChannel*>(channel_.get());
        if (serial != nullptr) {
            serial->setDTR(true);
        }

        running_ = false;
    }

    void maybeAutoStop() {
        if (!running_) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        const auto idle_s = std::chrono::duration_cast<std::chrono::seconds>(now - last_demand_).count();
        if (idle_s >= idle_timeout_s_) {
            std::cout << "idle timeout reached (" << idle_s << "s), auto stopping lidar" << '\n';
            stopMotor();
        }
    }

    bool grabOnce() {
        if (!ensureRunning()) {
            return false;
        }

        last_demand_ = std::chrono::steady_clock::now();

        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = sizeof(nodes) / sizeof(nodes[0]);

        sl_result result = SL_RESULT_OPERATION_TIMEOUT;
        for (int attempt = 0; attempt < 3; ++attempt) {
            count = sizeof(nodes) / sizeof(nodes[0]);
            result = driver_->grabScanDataHq(nodes, count, 1000);
            if (SL_IS_OK(result)) {
                break;
            }
            if (result != SL_RESULT_OPERATION_TIMEOUT) {
                std::cerr << "grabScanDataHq failed, code=" << result << '\n';
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        if (SL_IS_FAIL(result)) {
            std::cout << "no full scan yet (timeout), try command again" << '\n';
            return true;
        }

        driver_->ascendScanData(nodes, count);

        int valid_points = 0;
        float min_distance = std::numeric_limits<float>::max();
        for (size_t i = 0; i < count; ++i) {
            if (!isValidPoint(nodes[i])) {
                continue;
            }
            ++valid_points;
            min_distance = std::min(min_distance, distanceMeters(nodes[i]));
        }

        std::cout << "grabbed scan | valid_points=" << valid_points;
        if (valid_points > 0) {
            std::cout << " | min_distance=" << min_distance << "m";
        }
        std::cout << '\n';
        return true;
    }

private:
    std::unique_ptr<sl::ILidarDriver, DriverDeleter> driver_;
    std::unique_ptr<sl::IChannel, ChannelDeleter> channel_;
    int idle_timeout_s_ = 30;
    bool running_ = false;
    std::chrono::steady_clock::time_point last_demand_;
};

}  // namespace

int main(int argc, char* argv[]) {
    const std::string port = argc > 1 ? argv[1] : "/dev/ttyUSB0";
    const int baudrate = argc > 2 ? std::atoi(argv[2]) : 115200;
    const int idle_timeout_s = argc > 3 ? std::max(3, std::atoi(argv[3])) : 5;

    auto driver_result = sl::createLidarDriver();
    if (!driver_result || *driver_result == nullptr) {
        std::cerr << "failed to create lidar driver" << '\n';
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

    sl_lidar_response_device_info_t info {};
    result = driver->getDeviceInfo(info);
    if (SL_IS_FAIL(result)) {
        std::cerr << "getDeviceInfo failed, code=" << result << '\n';
        driver->disconnect();
        return 1;
    }

    std::cout << "RPLIDAR connected on " << port << '\n';
    std::cout << "firmware: " << static_cast<int>(info.firmware_version >> 8)
              << '.' << static_cast<int>(info.firmware_version & 0xFF) << '\n';
    std::cout << "hardware: " << static_cast<int>(info.hardware_version) << '\n';
    std::cout << "idle timeout: " << idle_timeout_s << "s (recommended 3-5s)" << '\n';

    GuardedLidar lidar(std::move(driver), std::move(channel), idle_timeout_s);

    std::cout << "commands: g (grab once), c <sec> (continuous), q (quit)" << '\n';

    while (true) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(STDIN_FILENO, &read_fds);

        timeval timeout {};
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        const int ready = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "select failed" << '\n';
            break;
        }

        if (ready == 0) {
            lidar.maybeAutoStop();
            continue;
        }

        std::string line;
        if (!std::getline(std::cin, line)) {
            break;
        }

        if (line == "q") {
            break;
        }
        if (line == "g") {
            lidar.grabOnce();
            continue;
        }

        if (line.rfind("c ", 0) == 0) {
            std::istringstream iss(line.substr(2));
            int seconds = 0;
            iss >> seconds;
            seconds = std::max(1, seconds);
            const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
            while (std::chrono::steady_clock::now() < end) {
                if (!lidar.grabOnce()) {
                    break;
                }
            }
            continue;
        }

        std::cout << "unknown command. use: g | c <sec> | q" << '\n';
    }

    return 0;
}
