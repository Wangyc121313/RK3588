#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "fusion/detection_distance_fusion.hpp"
#include "fusion/multi_target_tracker.hpp"
#include "infer/rknn_runner.hpp"

namespace rk3588::modules {

class PseudoLabelSink {
public:
    explicit PseudoLabelSink(std::string output_path,
                             std::uint32_t max_lines = 0,
                             std::string sequence_id = {},
                             std::uint32_t source_fps = 0);
    ~PseudoLabelSink();

    PseudoLabelSink(const PseudoLabelSink&) = delete;
    PseudoLabelSink& operator=(const PseudoLabelSink&) = delete;

    [[nodiscard]] bool enabled() const;
    void writeFrame(const std::string& camera_device,
                    std::uint64_t frame_id,
                    std::uint64_t timestamp_ms,
                    bool lidar_matched,
                    std::uint64_t lidar_delta_ms,
                    float camera_fov_deg,
                    float lidar_offset_deg,
                    float lidar_fov_deg,
                    float lidar_window_half_deg,
                    float lidar_min_dist_m,
                    float lidar_max_dist_m,
                    std::uint64_t lidar_max_age_ms,
                    const std::string& calibration_profile_path,
                    const std::vector<YoloDetection>& detections,
                    const std::vector<TrackEstimate>& tracks,
                    const std::vector<DistanceFusionDiagnostics>& diagnostics);

private:
    void rotateFile();
    [[nodiscard]] std::string currentOutputPath() const;
    bool ensureOpen();
    static std::string escapeJson(const std::string& text);

private:
    std::string output_path_;
    std::uint32_t max_lines_ = 0;
    std::string sequence_id_;
    std::uint32_t source_fps_ = 0;
    std::uint32_t file_index_ = 0;
    std::uint32_t current_file_lines_ = 0;
    void* handle_ = nullptr;
    bool use_stdout_ = false;
};

}  // namespace rk3588::modules
