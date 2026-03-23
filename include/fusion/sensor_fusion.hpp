#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include "core/data_types.hpp"

namespace rk3588::modules {

struct FusionConfig {
	int image_width = 640;
	float camera_fov_deg = 60.0F;
	float lidar_angle_offset_deg = 0.0F;
	float default_window_half_deg = 3.0F;
	float min_valid_distance_m = 0.15F;
};

struct LidarFovRange {
	float left_deg = 0.0F;
	float right_deg = 0.0F;
	bool crosses_zero = false;
};

class SensorFusion {
public:
	explicit SensorFusion(FusionConfig cfg) : cfg_(cfg) {}

	static float wrap360(float angle_deg) {
		float wrapped = std::fmod(angle_deg, 360.0F);
		if (wrapped < 0.0F) {
			wrapped += 360.0F;
		}
		return wrapped;
	}

	static float angularDistanceDeg(float a_deg, float b_deg) {
		float d = std::fabs(wrap360(a_deg) - wrap360(b_deg));
		if (d > 180.0F) {
			d = 360.0F - d;
		}
		return d;
	}

	float pixelToCameraAngle(float x_pixel) const {
		if (cfg_.image_width <= 0) {
			return 0.0F;
		}
		return ((x_pixel / static_cast<float>(cfg_.image_width)) - 0.5F) * cfg_.camera_fov_deg;
	}

	float cameraAngleToLidar(float camera_angle_deg) const {
		return wrap360(camera_angle_deg + cfg_.lidar_angle_offset_deg);
	}

	float pixelToLidarAngle(float x_pixel) const {
		return cameraAngleToLidar(pixelToCameraAngle(x_pixel));
	}

	LidarFovRange cameraFovToLidarRange() const {
		const float cam_left = -cfg_.camera_fov_deg * 0.5F;
		const float cam_right = cfg_.camera_fov_deg * 0.5F;

		LidarFovRange range;
		range.left_deg = cameraAngleToLidar(cam_left);
		range.right_deg = cameraAngleToLidar(cam_right);
		range.crosses_zero = range.left_deg > range.right_deg;
		return range;
	}

	static bool isAngleInRange(float angle_deg, const LidarFovRange& range) {
		const float a = wrap360(angle_deg);
		if (!range.crosses_zero) {
			return a >= range.left_deg && a <= range.right_deg;
		}
		return a >= range.left_deg || a <= range.right_deg;
	}

	std::optional<float> medianDistanceInAngleWindow(const rk3588::core::PointCloudPacket& cloud,
													 float center_angle_deg,
													 float window_half_deg = -1.0F) const {
		const float half_window = (window_half_deg > 0.0F) ? window_half_deg : cfg_.default_window_half_deg;

		std::vector<float> candidates;
		candidates.reserve(cloud.points.size());

		for (const auto& point : cloud.points) {
			if (point.distance_m < cfg_.min_valid_distance_m) {
				continue;
			}
			if (angularDistanceDeg(point.angle_deg, center_angle_deg) <= half_window) {
				candidates.push_back(point.distance_m);
			}
		}

		if (candidates.empty()) {
			return std::nullopt;
		}

		std::sort(candidates.begin(), candidates.end());
		const std::size_t mid = candidates.size() / 2;
		if ((candidates.size() & 1U) == 1U) {
			return candidates[mid];
		}
		return 0.5F * (candidates[mid - 1] + candidates[mid]);
	}

private:
	FusionConfig cfg_;
};

}  // namespace rk3588::modules
