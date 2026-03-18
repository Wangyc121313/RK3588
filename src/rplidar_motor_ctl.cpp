#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

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

void printUsage(const char* program) {
    std::cout << "Usage:\n"
              << "  " << program << " stop [port] [baudrate] [hold_seconds]\n"
              << "  " << program << " start [port] [baudrate]\n"
              << "\nDefaults: port=/dev/ttyUSB0 baudrate=115200 hold_seconds=2\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    const std::string action = argv[1];
    const std::string port = argc > 2 ? argv[2] : "/dev/ttyUSB0";
    const int baudrate = argc > 3 ? std::atoi(argv[3]) : 115200;
    const int hold_seconds = argc > 4 ? std::max(0, std::atoi(argv[4])) : 2;

    if (action != "stop" && action != "start") {
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

    if (action == "stop") {
        driver->stop();
        result = driver->setMotorSpeed(0);
        if (SL_IS_FAIL(result)) {
            std::cerr << "setMotorSpeed(0) failed, code=" << result << '\n';
        }

        auto* serial = dynamic_cast<sl::ISerialPortChannel*>(channel.get());
        if (serial != nullptr) {
            serial->setDTR(true);
        }

        if (hold_seconds > 0) {
            std::cout << "holding stop line for " << hold_seconds << "s...\n";
            std::this_thread::sleep_for(std::chrono::seconds(hold_seconds));
        }

        std::cout << "motor stop requested\n";
    } else {
        result = driver->setMotorSpeed(DEFAULT_MOTOR_SPEED);
        if (SL_IS_FAIL(result)) {
            std::cerr << "setMotorSpeed(DEFAULT_MOTOR_SPEED) failed, code=" << result << '\n';
            driver->disconnect();
            return 1;
        }
        std::cout << "motor start requested\n";
    }

    driver->disconnect();
    return 0;
}
