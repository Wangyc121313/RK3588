#pragma once

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>

#include "pipeline/app_config.hpp"

namespace rk3588::modules {

struct CalibrationProfile {
    float raw_front_angle_deg = 0.0F;
    float camera_fov_deg = 55.0F;
    float lidar_offset_deg = 0.0F;
    float lidar_fov_deg = 55.0F;
    float lidar_window_half_deg = 2.5F;
    float lidar_min_dist_m = 0.15F;
    float lidar_max_dist_m = 20.0F;
    std::uint64_t lidar_max_age_ms = 120U;
};

inline std::string trimCopy(const std::string& text) {
    const std::size_t first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return std::string();
    }
    const std::size_t last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

inline bool parseCalibrationProfile(const std::string& text, CalibrationProfile* profile) {
    if (profile == nullptr) {
        return false;
    }

    CalibrationProfile parsed = *profile;
    std::istringstream input(text);
    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trimCopy(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }

        const std::size_t sep = trimmed.find('=');
        if (sep == std::string::npos) {
            continue;
        }

        const std::string key = trimCopy(trimmed.substr(0, sep));
        const std::string value = trimCopy(trimmed.substr(sep + 1));
        if (key == "raw_front_angle_deg") {
            parsed.raw_front_angle_deg = std::stof(value);
        } else if (key == "camera_fov_deg") {
            parsed.camera_fov_deg = std::stof(value);
        } else if (key == "lidar_offset_deg") {
            parsed.lidar_offset_deg = std::stof(value);
        } else if (key == "lidar_fov_deg") {
            parsed.lidar_fov_deg = std::stof(value);
        } else if (key == "lidar_window_half_deg") {
            parsed.lidar_window_half_deg = std::stof(value);
        } else if (key == "lidar_min_dist_m") {
            parsed.lidar_min_dist_m = std::stof(value);
        } else if (key == "lidar_max_dist_m") {
            parsed.lidar_max_dist_m = std::stof(value);
        } else if (key == "lidar_max_age_ms") {
            parsed.lidar_max_age_ms = static_cast<std::uint64_t>(std::stoull(value));
        }
    }

    *profile = parsed;
    return true;
}

inline bool loadCalibrationProfile(const std::string& path, CalibrationProfile* profile) {
    if (path.empty() || profile == nullptr) {
        return false;
    }
    std::ifstream input(path);
    if (!input.is_open()) {
        return false;
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return parseCalibrationProfile(buffer.str(), profile);
}

inline bool saveCalibrationProfile(const std::string& path, const CalibrationProfile& profile) {
    if (path.empty()) {
        return false;
    }
    std::ofstream output(path, std::ios::trunc);
    if (!output.is_open()) {
        return false;
    }

    output << "# RK3588 camera-lidar calibration profile\n";
    output << "raw_front_angle_deg=" << profile.raw_front_angle_deg << "\n";
    output << "camera_fov_deg=" << profile.camera_fov_deg << "\n";
    output << "lidar_offset_deg=" << profile.lidar_offset_deg << "\n";
    output << "lidar_fov_deg=" << profile.lidar_fov_deg << "\n";
    output << "lidar_window_half_deg=" << profile.lidar_window_half_deg << "\n";
    output << "lidar_min_dist_m=" << profile.lidar_min_dist_m << "\n";
    output << "lidar_max_dist_m=" << profile.lidar_max_dist_m << "\n";
    output << "lidar_max_age_ms=" << profile.lidar_max_age_ms << "\n";
    return true;
}

inline void applyCalibrationProfile(const CalibrationProfile& profile, AppConfig* config) {
    if (config == nullptr) {
        return;
    }
    config->camera_fov_deg = std::max(1.0F, profile.camera_fov_deg);
    config->lidar_offset_deg = profile.lidar_offset_deg;
    config->lidar_fov_deg = std::max(1.0F, profile.lidar_fov_deg);
    config->lidar_window_half_deg = std::max(0.5F, profile.lidar_window_half_deg);
    config->lidar_min_dist_m = std::max(0.01F, profile.lidar_min_dist_m);
    config->lidar_max_dist_m = std::max(config->lidar_min_dist_m + 0.1F, profile.lidar_max_dist_m);
    config->lidar_max_age_ms = std::max<std::uint64_t>(20U, profile.lidar_max_age_ms);
}

}  // namespace rk3588::modules