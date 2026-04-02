#include "lidar/lidar_reader.hpp"

#include <chrono>
#include <iostream>
#include <memory>

#if RK3588_HAS_RPLIDAR_SDK
#include "fusion/sensor_fusion.hpp"
#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#endif

namespace rk3588::modules {

#if RK3588_HAS_RPLIDAR_SDK

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

std::uint64_t nowSteadyMs() {
	return static_cast<std::uint64_t>(
		std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now().time_since_epoch())
			.count());
}

}  // namespace

struct LidarReader::Impl {
	std::unique_ptr<sl::ILidarDriver, DriverDeleter> driver;
	std::unique_ptr<sl::IChannel, ChannelDeleter> channel;
};

#else

struct LidarReader::Impl {};

#endif

LidarReader::LidarReader() : impl_(std::make_unique<Impl>()) {}

LidarReader::~LidarReader() {
	stop();
}

bool LidarReader::start(const Config& config) {
	stop();
	config_ = config;
	next_scan_id_ = 0;

#if RK3588_HAS_RPLIDAR_SDK
	auto driver_result = sl::createLidarDriver();
	if (!driver_result || *driver_result == nullptr) {
		std::cerr << "lidar: failed to create driver\n";
		return false;
	}
	impl_->driver.reset(*driver_result);

	const char* port = (config_.port != nullptr) ? config_.port : "/dev/ttyUSB0";
	auto channel_result = sl::createSerialPortChannel(port, config_.baud);
	if (!channel_result || *channel_result == nullptr) {
		std::cerr << "lidar: failed to open serial channel " << port << '\n';
		impl_->driver.reset();
		return false;
	}
	impl_->channel.reset(*channel_result);

	sl_result result = impl_->driver->connect(impl_->channel.get());
	if (SL_IS_FAIL(result)) {
		std::cerr << "lidar: connect failed, code=" << result << '\n';
		impl_->channel.reset();
		impl_->driver.reset();
		return false;
	}

	result = impl_->driver->startScan(false, true);
	if (SL_IS_FAIL(result)) {
		std::cerr << "lidar: startScan failed, code=" << result << '\n';
		impl_->driver->disconnect();
		impl_->channel.reset();
		impl_->driver.reset();
		return false;
	}

	running_ = true;
	return true;
#else
	(void)config_;
	std::cerr << "lidar: rplidar sdk is not available in this build\n";
	return false;
#endif
}

void LidarReader::stop() {
	if (!running_) {
		return;
	}

#if RK3588_HAS_RPLIDAR_SDK
	if (impl_->driver) {
		impl_->driver->stop();
		impl_->driver->disconnect();
	}
#endif

	running_ = false;
	impl_->channel.reset();
	impl_->driver.reset();
}

std::optional<PointCloudPacket> LidarReader::poll(std::uint32_t timeout_ms) {
	if (!running_) {
		return std::nullopt;
	}

#if RK3588_HAS_RPLIDAR_SDK
	sl_lidar_response_measurement_node_hq_t nodes[8192];
	size_t node_count = sizeof(nodes) / sizeof(nodes[0]);
	const sl_result result = impl_->driver->grabScanDataHq(nodes, node_count, timeout_ms);
	if (SL_IS_FAIL(result)) {
		if (result != SL_RESULT_OPERATION_TIMEOUT) {
			std::cerr << "lidar: grabScanDataHq failed, code=" << result << '\n';
		}
		return std::nullopt;
	}

	impl_->driver->ascendScanData(nodes, node_count);

	PointCloudPacket cloud;
	cloud.scan_id = next_scan_id_++;
	cloud.timestamp_ms = nowSteadyMs();
	cloud.points.reserve(node_count);

	for (size_t i = 0; i < node_count; ++i) {
		if (nodes[i].dist_mm_q2 == 0) {
			continue;
		}

		const float distance_m = static_cast<float>(nodes[i].dist_mm_q2) / 4.0F / 1000.0F;
		if (distance_m < config_.min_distance_m || distance_m > config_.max_distance_m) {
			continue;
		}

		LidarPoint point;
		point.angle_deg = SensorFusion::wrap360(static_cast<float>(nodes[i].angle_z_q14) * 90.0F / 16384.0F);
		point.distance_m = distance_m;
		cloud.points.push_back(point);
	}

	if (cloud.points.empty()) {
		return std::nullopt;
	}
	return cloud;
#else
	(void)timeout_ms;
	return std::nullopt;
#endif
}

}  // namespace rk3588::modules
