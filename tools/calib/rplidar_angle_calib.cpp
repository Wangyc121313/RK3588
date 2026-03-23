#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "pipeline/calibration_profile.hpp"

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

constexpr float kPi = 3.14159265358979323846f;

float wrapDeg360(float deg) {
    float out = std::fmod(deg, 360.0f);
    if (out < 0.0f) {
        out += 360.0f;
    }
    return out;
}

float fusionOffsetDeg(float calib_deg) {
    return wrapDeg360(calib_deg - 180.0f);
}

float angleDeg(const sl_lidar_response_measurement_node_hq_t& node) {
    return static_cast<float>(node.angle_z_q14) * 90.0f / 16384.0f;
}

float distanceM(const sl_lidar_response_measurement_node_hq_t& node) {
    return static_cast<float>(node.dist_mm_q2) / 4.0f / 1000.0f;
}

bool isValidPoint(const sl_lidar_response_measurement_node_hq_t& node) {
    return node.dist_mm_q2 != 0;
}

float degToRad(float deg) {
    return deg * kPi / 180.0f;
}

float radToDeg(float rad) {
    return rad * 180.0f / kPi;
}

float angularDiffDeg(float a, float b) {
    const float diff = std::fabs(wrapDeg360(a) - wrapDeg360(b));
    return std::min(diff, 360.0f - diff);
}

struct AngleSample {
    float angle_deg;
    float dist_m;
    float weight;
};

float weightedCircularMeanDeg(const std::vector<AngleSample>& samples) {
    double sum_sin = 0.0;
    double sum_cos = 0.0;
    for (const auto& sample : samples) {
        const double rad = static_cast<double>(degToRad(sample.angle_deg));
        const double w = static_cast<double>(sample.weight);
        sum_sin += std::sin(rad) * w;
        sum_cos += std::cos(rad) * w;
    }

    if (std::fabs(sum_sin) < 1e-9 && std::fabs(sum_cos) < 1e-9) {
        return 0.0f;
    }

    const float mean_rad = static_cast<float>(std::atan2(sum_sin, sum_cos));
    return wrapDeg360(radToDeg(mean_rad));
}

float circularStdDevDeg(const std::vector<AngleSample>& samples) {
    double sum_w = 0.0;
    double sum_sin = 0.0;
    double sum_cos = 0.0;
    for (const auto& sample : samples) {
        const double rad = static_cast<double>(degToRad(sample.angle_deg));
        const double w = static_cast<double>(sample.weight);
        sum_w += w;
        sum_sin += std::sin(rad) * w;
        sum_cos += std::cos(rad) * w;
    }

    if (sum_w <= 0.0) {
        return 180.0f;
    }

    const double r = std::sqrt(sum_sin * sum_sin + sum_cos * sum_cos) / sum_w;
    const double clamped = std::max(1e-9, std::min(1.0, r));
    const double sigma_rad = std::sqrt(-2.0 * std::log(clamped));
    return radToDeg(static_cast<float>(sigma_rad));
}

void printUsage(const char* program) {
    std::cout << "Usage: " << program
              << " [port] [baud] [target_m] [tolerance_m] [duration_s] [min_quality] [output_profile]\\n"
              << "Defaults: /dev/ttyUSB0 115200 0.2 0.03 8 8\\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string port = argc > 1 ? argv[1] : "/dev/ttyUSB0";
    const int baudrate = argc > 2 ? std::atoi(argv[2]) : 115200;
    const float target_m = argc > 3 ? std::max(0.01f, static_cast<float>(std::atof(argv[3]))) : 0.2f;
    const float tol_m = argc > 4 ? std::max(0.005f, static_cast<float>(std::atof(argv[4]))) : 0.03f;
    const int duration_s = argc > 5 ? std::max(2, std::atoi(argv[5])) : 8;
    const int min_quality = argc > 6 ? std::max(0, std::atoi(argv[6])) : 8;
    const std::string output_profile = argc > 7 ? argv[7] : "";

    if (argc > 8) {
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
    std::cout << "target distance=" << target_m << " m, tolerance=" << tol_m
              << " m, collect=" << duration_s << " s\n";

    result = driver->startScan(0, 1);
    if (SL_IS_FAIL(result)) {
        std::cerr << "startScan failed, code=" << result << '\n';
        driver->disconnect();
        return 1;
    }

    const auto t0 = std::chrono::steady_clock::now();
    std::vector<AngleSample> all_hits;
    all_hits.reserve(4000);

    std::uint64_t total_points = 0;
    std::uint64_t accepted_points = 0;

    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(now - t0).count();
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

        for (size_t i = 0; i < node_count; ++i) {
            if (!isValidPoint(nodes[i])) {
                continue;
            }
            ++total_points;

            const int quality = static_cast<int>(nodes[i].quality);
            if (quality < min_quality) {
                continue;
            }

            const float dist = distanceM(nodes[i]);
            const float err = std::fabs(dist - target_m);
            if (err > tol_m) {
                continue;
            }

            const float angle = wrapDeg360(angleDeg(nodes[i]));
            const float proximity = std::max(0.05f, 1.0f - err / tol_m);
            const float quality_w = std::max(1.0f, static_cast<float>(quality));
            const float weight = proximity * quality_w;

            all_hits.push_back(AngleSample {angle, dist, weight});
            ++accepted_points;
        }
    }

    // Stop scan session before disconnect to avoid SDK teardown races.
    driver->stop();

    // Keep motor speed untouched here (no setMotorSpeed(0)) to avoid
    // explicit motor-off commands between repeated calibrations.
    driver->disconnect();

    if (all_hits.empty()) {
        std::cerr << "no points found near target distance. Try increasing tolerance or duration.\n";
        std::cerr << "hint: " << argv[0] << " " << port << " " << baudrate
                  << " " << target_m << " 0.05 12 " << min_quality << '\n';
        return 2;
    }

    std::vector<double> hist(360, 0.0);
    double hist_total = 0.0;
    for (const auto& sample : all_hits) {
        const int bin = static_cast<int>(sample.angle_deg) % 360;
        hist[bin] += sample.weight;
        hist_total += sample.weight;
    }

    int peak_bin = 0;
    for (int i = 1; i < 360; ++i) {
        if (hist[i] > hist[peak_bin]) {
            peak_bin = i;
        }
    }

    std::vector<AngleSample> peak_cluster;
    peak_cluster.reserve(all_hits.size());
    for (const auto& sample : all_hits) {
        if (angularDiffDeg(sample.angle_deg, static_cast<float>(peak_bin)) <= 8.0f) {
            peak_cluster.push_back(sample);
        }
    }

    const float calib_deg = weightedCircularMeanDeg(peak_cluster);
    const float spread_deg = circularStdDevDeg(peak_cluster);

    float mean_dist = 0.0f;
    float min_dist = std::numeric_limits<float>::max();
    float max_dist = 0.0f;
    double sum_w = 0.0;
    for (const auto& sample : peak_cluster) {
        mean_dist += sample.dist_m;
        min_dist = std::min(min_dist, sample.dist_m);
        max_dist = std::max(max_dist, sample.dist_m);
        sum_w += sample.weight;
    }
    mean_dist /= static_cast<float>(std::max<std::size_t>(1, peak_cluster.size()));

    const double confidence = (hist_total > 1e-6) ? (sum_w / hist_total) : 0.0;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n=== Angle Calibration Result ===\n";
    std::cout << "target_m=" << target_m << " tolerance_m=" << tol_m << '\n';
    std::cout << "total_valid_points=" << total_points
              << " accepted_points=" << accepted_points
              << " peak_cluster_points=" << peak_cluster.size() << '\n';
    std::cout << "cluster_distance_mean=" << mean_dist
              << " min=" << min_dist
              << " max=" << max_dist << " m\n";
    std::cout << "calibrated_deg=" << calib_deg << '\n';
    std::cout << "spread_deg=" << spread_deg
              << " confidence=" << confidence << "\n";

    if (spread_deg > 6.0f || confidence < 0.35) {
        std::cout << "status=LOW_CONFIDENCE (re-run with less clutter, larger tripod reflector, or longer duration)\n";
    } else {
        std::cout << "status=OK\n";
    }

    std::cout << "raw_front_angle_deg=" << calib_deg << '\n';
    std::cout << "fusion_offset_deg=" << fusionOffsetDeg(calib_deg) << '\n';
    std::cout << "\nSuggested usage in fusion config: lidar_offset_deg=" << fusionOffsetDeg(calib_deg) << '\n';
    if (!output_profile.empty()) {
        rk3588::modules::CalibrationProfile profile;
        profile.raw_front_angle_deg = calib_deg;
        profile.lidar_offset_deg = fusionOffsetDeg(calib_deg);
        if (rk3588::modules::saveCalibrationProfile(output_profile, profile)) {
            std::cout << "saved_profile=" << output_profile << '\n';
        } else {
            std::cerr << "failed to save calibration profile: " << output_profile << '\n';
            return 3;
        }
    }
    return 0;
}
