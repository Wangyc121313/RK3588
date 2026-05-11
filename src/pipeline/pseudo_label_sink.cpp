#include "pipeline/pseudo_label_sink.hpp"

#include <cstdio>
#include <sstream>
#include <utility>

namespace {

std::uint32_t countLinesInFile(const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "r");
    if (f == nullptr) {
        return 0;
    }
    std::uint32_t lines = 0;
    int c = 0;
    while ((c = std::fgetc(f)) != EOF) {
        if (c == '\n') {
            ++lines;
        }
    }
    std::fclose(f);
    return lines;
}

}  // namespace

namespace rk3588::modules {

PseudoLabelSink::PseudoLabelSink(std::string output_path,
                                                                 std::uint32_t max_lines,
                                                                 std::string sequence_id,
                                                                 std::uint32_t source_fps)
        : output_path_(std::move(output_path)),
            max_lines_(max_lines),
            sequence_id_(std::move(sequence_id)),
            source_fps_(source_fps) {
    use_stdout_ = output_path_ == "-";
}

PseudoLabelSink::~PseudoLabelSink() {
    if (handle_ != nullptr && !use_stdout_) {
        std::fclose(static_cast<FILE*>(handle_));
        handle_ = nullptr;
    }
}

bool PseudoLabelSink::enabled() const {
    return !output_path_.empty();
}

std::string PseudoLabelSink::currentOutputPath() const {
    if (file_index_ == 0) {
        return output_path_;
    }
    return output_path_ + "." + std::to_string(file_index_);
}

void PseudoLabelSink::rotateFile() {
    if (use_stdout_) {
        return;
    }
    if (handle_ != nullptr) {
        std::fclose(static_cast<FILE*>(handle_));
        handle_ = nullptr;
    }
    ++file_index_;
    current_file_lines_ = 0;
}

bool PseudoLabelSink::ensureOpen() {
    if (handle_ != nullptr) {
        return true;
    }
    if (!enabled()) {
        return false;
    }
    if (use_stdout_) {
        handle_ = stdout;
        return true;
    }

    if (max_lines_ > 0 && current_file_lines_ == 0 && file_index_ == 0) {
        current_file_lines_ = countLinesInFile(output_path_);
        if (current_file_lines_ >= max_lines_) {
            rotateFile();
        }
    }

    handle_ = std::fopen(currentOutputPath().c_str(), "a");
    return handle_ != nullptr;
}

std::string PseudoLabelSink::escapeJson(const std::string& text) {
    std::string out;
    out.reserve(text.size() + 8);
    for (char ch : text) {
        switch (ch) {
            case '\\':
                out += "\\\\";
                break;
            case '"':
                out += "\\\"";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                out += ch;
                break;
        }
    }
    return out;
}

void PseudoLabelSink::writeFrame(const std::string& camera_device,
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
                                 const std::vector<DistanceFusionDiagnostics>& diagnostics) {
    if (!enabled() || !ensureOpen()) {
        return;
    }

    if (!use_stdout_ && max_lines_ > 0 && current_file_lines_ >= max_lines_) {
        rotateFile();
        if (!ensureOpen()) {
            return;
        }
    }

    auto* file = static_cast<FILE*>(handle_);
    if (file == nullptr) {
        return;
    }

    if (sequence_id_.empty()) {
        sequence_id_ = "seq_" + std::to_string(timestamp_ms);
    }

    std::ostringstream json;
    json << '{'
         << "\"schema\":\"rk3588_pseudo_label_v1\"," 
         << "\"sequence_id\":\"" << escapeJson(sequence_id_) << "\"," 
         << "\"source_fps\":" << source_fps_ << ','
         << "\"camera_device\":\"" << escapeJson(camera_device) << "\"," 
         << "\"frame_id\":" << frame_id << ','
         << "\"timestamp_ms\":" << timestamp_ms << ','
         << "\"lidar_matched\":" << (lidar_matched ? "true" : "false") << ','
         << "\"lidar_delta_ms\":" << lidar_delta_ms << ','
         << "\"sensor_snapshot\":{"
         << "\"camera_fov_deg\":" << camera_fov_deg << ','
         << "\"lidar_offset_deg\":" << lidar_offset_deg << ','
         << "\"lidar_fov_deg\":" << lidar_fov_deg << ','
         << "\"lidar_window_half_deg\":" << lidar_window_half_deg << ','
         << "\"lidar_min_dist_m\":" << lidar_min_dist_m << ','
         << "\"lidar_max_dist_m\":" << lidar_max_dist_m << ','
         << "\"lidar_max_age_ms\":" << lidar_max_age_ms << ','
         << "\"calibration_profile\":\"" << escapeJson(calibration_profile_path) << "\""
         << "},"
         << "\"objects\":[";

    for (std::size_t i = 0; i < detections.size(); ++i) {
        const auto& det = detections[i];
        const TrackEstimate* track = i < tracks.size() ? &(tracks[i]) : nullptr;
        const float tracked_distance =
            (track != nullptr && track->filtered_distance_m >= 0.0F) ? track->filtered_distance_m : det.distance_m;
        const DistanceFusionDiagnostics* diag = (i < diagnostics.size()) ? &diagnostics[i] : nullptr;

        if (i != 0) {
            json << ',';
        }

        json << '{'
             << "\"class_id\":" << det.class_id << ','
             << "\"class_name\":\"" << escapeJson(det.class_name) << "\","
             << "\"confidence\":" << det.confidence << ','
             << "\"distance_m\":" << tracked_distance << ','
             << "\"fusion\":{"
             << "\"raw_distance_m\":" << (diag != nullptr ? diag->raw_distance_m : -1.0F) << ','
             << "\"candidate_points\":" << (diag != nullptr ? diag->candidate_points : 0) << ','
             << "\"cluster_points\":" << (diag != nullptr ? diag->cluster_points : 0) << ','
             << "\"cluster_score\":" << (diag != nullptr ? diag->cluster_score : 0.0F) << ','
             << "\"used_fallback\":" << ((diag != nullptr && diag->used_fallback) ? "true" : "false") << ','
             << "\"used_temporal_smoothing\":" << ((diag != nullptr && diag->used_temporal_smoothing) ? "true" : "false") << ','
             << "\"rejected_by_sanity\":" << ((diag != nullptr && diag->rejected_by_sanity) ? "true" : "false")
             << "},"
             << "\"bbox\":{"
             << "\"left\":" << det.left << ','
             << "\"top\":" << det.top << ','
             << "\"right\":" << det.right << ','
             << "\"bottom\":" << det.bottom
             << "},"
             << "\"track_id\":" << (track != nullptr ? track->track_id : -1) << ','
             << "\"track_age_frames\":" << (track != nullptr ? track->age_frames : 0) << ','
             << "\"track_idle_frames\":" << (track != nullptr ? track->idle_frames : 0) << ','
             << "\"track_confirmed\":" << ((track != nullptr && track->confirmed) ? "true" : "false") << ','
             << "\"track_is_ghost\":" << ((track != nullptr && track->is_ghost) ? "true" : "false") << ','
             << "\"track_angle_deg\":" << (track != nullptr ? track->filtered_angle_deg : 0.0F)
             << '}';
    }

    json << "]}\n";
    const std::string line = json.str();
    (void)std::fwrite(line.data(), 1, line.size(), file);
    std::fflush(file);
    if (!use_stdout_) {
        ++current_file_lines_;
    }
}

}  // namespace rk3588::modules
