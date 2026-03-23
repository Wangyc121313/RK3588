#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "core/data_types.hpp"
#include "fusion/sensor_fusion.hpp"
#include "infer/rknn_runner.hpp"

namespace rk3588::modules {

struct DistanceFusionConfig {
    float min_distance_m = 0.15F;
    float max_distance_m = 20.0F;
    float window_half_deg = 2.5F;
    float sector_expand_deg = 1.25F;
    float cluster_gap_m = 0.28F;
    int min_candidate_points = 4;
    int min_cluster_points = 3;
    float min_detection_confidence = 0.45F;
    int min_box_width_px = 18;
    int min_box_height_px = 18;
    float max_box_area_ratio = 0.85F;
    float focus_box_width_ratio = 0.45F;
    float max_track_center_delta_px = 96.0F;
    std::uint32_t track_max_idle_frames = 10;
    float smoothing_alpha = 0.34F;
    float outlier_jump_ratio = 0.45F;
};

struct DistanceFusionDiagnostics {
    float raw_distance_m = -1.0F;
    float smoothed_distance_m = -1.0F;
    int candidate_points = 0;
    int cluster_points = 0;
    float cluster_score = 0.0F;
    bool used_fallback = false;
    bool used_temporal_smoothing = false;
    bool rejected_by_sanity = false;
};

class DetectionDistanceFusion {
public:
    DetectionDistanceFusion(const SensorFusion& fusion, DistanceFusionConfig cfg)
        : fusion_(fusion), cfg_(cfg) {}

    void fuse(const rk3588::core::PointCloudPacket* cloud,
              std::vector<YoloDetection>* detections,
              std::vector<DistanceFusionDiagnostics>* diagnostics = nullptr) {
        ++frame_index_;
        ageTracks();

        if (detections == nullptr) {
            return;
        }

        std::vector<DistanceFusionDiagnostics> local_diagnostics(detections->size());
        for (std::size_t i = 0; i < detections->size(); ++i) {
            auto& det = (*detections)[i];
            det.distance_m = -1.0F;
            if (cloud == nullptr || cloud->points.empty()) {
                continue;
            }

            auto diag = estimate(*cloud, det);
            local_diagnostics[i] = diag;
            if (diag.raw_distance_m < 0.0F) {
                continue;
            }

            const float center_x = 0.5F * static_cast<float>(det.left + det.right);
            const int track_index = findBestTrack(det.class_id, center_x, diag.raw_distance_m);
            if (track_index >= 0) {
                auto& track = tracks_[static_cast<std::size_t>(track_index)];
                float alpha = cfg_.smoothing_alpha;
                if (diag.cluster_points >= 6) {
                    alpha += 0.10F;
                }
                alpha = std::min(0.70F, alpha);

                const float jump_ratio = std::fabs(diag.raw_distance_m - track.distance_m) /
                    std::max(0.5F, std::fabs(track.distance_m));
                if (jump_ratio > cfg_.outlier_jump_ratio) {
                    alpha *= 0.45F;
                }

                const float smoothed = track.distance_m + alpha * (diag.raw_distance_m - track.distance_m);
                det.distance_m = clampDistance(smoothed);
                diag.smoothed_distance_m = det.distance_m;
                diag.used_temporal_smoothing = true;
                track.distance_m = det.distance_m;
                track.center_x = center_x;
                track.last_seen_frame = frame_index_;
                track.class_id = det.class_id;
            } else {
                det.distance_m = clampDistance(diag.raw_distance_m);
                diag.smoothed_distance_m = det.distance_m;
                TrackState track;
                track.class_id = det.class_id;
                track.center_x = center_x;
                track.distance_m = det.distance_m;
                track.last_seen_frame = frame_index_;
                tracks_.push_back(track);
            }

            local_diagnostics[i] = diag;
        }

        if (diagnostics != nullptr) {
            *diagnostics = std::move(local_diagnostics);
        }
    }

private:
    struct CandidatePoint {
        float distance_m = 0.0F;
        float angle_delta_deg = 0.0F;
        float weight = 1.0F;
    };

    struct ClusterSummary {
        float median_distance_m = -1.0F;
        float score = std::numeric_limits<float>::lowest();
        int point_count = 0;
    };

    struct TrackState {
        int class_id = -1;
        float center_x = 0.0F;
        float distance_m = -1.0F;
        std::uint64_t last_seen_frame = 0;
    };

    static float clampDistance(float distance_m) {
        return std::max(0.0F, distance_m);
    }

    void ageTracks() {
        tracks_.erase(
            std::remove_if(
                tracks_.begin(),
                tracks_.end(),
                [&](const TrackState& track) {
                    return frame_index_ > track.last_seen_frame &&
                           frame_index_ - track.last_seen_frame > cfg_.track_max_idle_frames;
                }),
            tracks_.end());
    }

    int findBestTrack(int class_id, float center_x, float raw_distance_m) const {
        float best_cost = std::numeric_limits<float>::max();
        int best_index = -1;

        for (std::size_t i = 0; i < tracks_.size(); ++i) {
            const auto& track = tracks_[i];
            if (track.class_id != class_id) {
                continue;
            }
            const float center_delta = std::fabs(center_x - track.center_x);
            if (center_delta > cfg_.max_track_center_delta_px) {
                continue;
            }

            const float distance_delta = std::fabs(raw_distance_m - track.distance_m);
            const float normalized_distance_delta = distance_delta /
                std::max(0.5F, std::max(std::fabs(raw_distance_m), std::fabs(track.distance_m)));
            const float cost = center_delta / std::max(1.0F, cfg_.max_track_center_delta_px) +
                               0.35F * normalized_distance_delta;
            if (cost < best_cost) {
                best_cost = cost;
                best_index = static_cast<int>(i);
            }
        }

        return best_index;
    }

    DistanceFusionDiagnostics estimate(const rk3588::core::PointCloudPacket& cloud,
                                       const YoloDetection& det) const {
        DistanceFusionDiagnostics diagnostics;
        if (det.right <= det.left || cloud.points.empty()) {
            return diagnostics;
        }

        const int box_width_px = std::max(0, det.right - det.left);
        const int box_height_px = std::max(0, det.bottom - det.top);
        const int image_width = std::max(1, fusion_.imageWidth());
        const float box_area_ratio = static_cast<float>(box_width_px * box_height_px) /
            static_cast<float>(image_width * image_width);
        if (det.confidence < cfg_.min_detection_confidence ||
            box_width_px < cfg_.min_box_width_px ||
            box_height_px < cfg_.min_box_height_px ||
            box_area_ratio > cfg_.max_box_area_ratio) {
            diagnostics.rejected_by_sanity = true;
            return diagnostics;
        }

        const float center_x = 0.5F * static_cast<float>(det.left + det.right);
        const float center_angle = fusion_.pixelToLidarAngle(center_x);
        const float focus_half_width_px = std::max(8.0F, 0.5F * cfg_.focus_box_width_ratio * static_cast<float>(box_width_px));
        const float focus_left_x = std::max(static_cast<float>(det.left), center_x - focus_half_width_px);
        const float focus_right_x = std::min(static_cast<float>(det.right), center_x + focus_half_width_px);
        const float left_angle = fusion_.pixelToLidarAngle(focus_left_x);
        const float right_angle = fusion_.pixelToLidarAngle(focus_right_x);
        const float half_span_deg = std::max(0.75F, 0.5F * SensorFusion::angularDistanceDeg(left_angle, right_angle));
        const float expand_deg = std::max(cfg_.sector_expand_deg, half_span_deg * 0.15F);
        const float sector_left = SensorFusion::wrap360(left_angle - expand_deg);
        const float sector_right = SensorFusion::wrap360(right_angle + expand_deg);

        std::vector<CandidatePoint> candidates;
        candidates.reserve(cloud.points.size() / 8);
        for (const auto& point : cloud.points) {
            if (point.distance_m < cfg_.min_distance_m || point.distance_m > cfg_.max_distance_m) {
                continue;
            }
            if (!SensorFusion::isAngleInRange(point.angle_deg, {sector_left, sector_right, sector_left > sector_right})) {
                continue;
            }

            const float angle_delta = SensorFusion::angularDistanceDeg(point.angle_deg, center_angle);
            const float normalized_delta = angle_delta / std::max(0.35F, half_span_deg + 0.35F);
            const float weight = 1.0F + std::max(0.0F, 1.5F - normalized_delta);
            candidates.push_back({point.distance_m, angle_delta, weight});
        }

        diagnostics.candidate_points = static_cast<int>(candidates.size());
        if (static_cast<int>(candidates.size()) < cfg_.min_candidate_points) {
            diagnostics.used_fallback = true;
            const auto fallback = fusion_.medianDistanceInAngleWindow(cloud, center_angle, cfg_.window_half_deg);
            if (fallback.has_value()) {
                diagnostics.raw_distance_m = *fallback;
                diagnostics.smoothed_distance_m = *fallback;
            }
            return diagnostics;
        }

        std::sort(candidates.begin(), candidates.end(), [](const CandidatePoint& a, const CandidatePoint& b) {
            return a.distance_m < b.distance_m;
        });

        ClusterSummary best_cluster;
        std::size_t start = 0;
        while (start < candidates.size()) {
            std::size_t end = start + 1;
            while (end < candidates.size()) {
                const float dynamic_gap = std::max(cfg_.cluster_gap_m, candidates[end - 1].distance_m * 0.04F);
                if (candidates[end].distance_m - candidates[end - 1].distance_m > dynamic_gap) {
                    break;
                }
                ++end;
            }

            const int count = static_cast<int>(end - start);
            float weight_sum = 0.0F;
            float weighted_angle = 0.0F;
            for (std::size_t i = start; i < end; ++i) {
                weight_sum += candidates[i].weight;
                weighted_angle += candidates[i].weight * candidates[i].angle_delta_deg;
            }
            const float mean_angle_delta = weight_sum > 0.0F ? weighted_angle / weight_sum : half_span_deg;
            const float spread = candidates[end - 1].distance_m - candidates[start].distance_m;
            const std::size_t mid = start + static_cast<std::size_t>(count / 2);
            const float median_distance = (count & 1) == 1
                ? candidates[mid].distance_m
                : 0.5F * (candidates[mid - 1].distance_m + candidates[mid].distance_m);

            const float score = weight_sum + 0.35F * static_cast<float>(count)
                - 1.2F * (mean_angle_delta / std::max(0.5F, half_span_deg))
                - 0.9F * spread;
            if (score > best_cluster.score) {
                best_cluster.score = score;
                best_cluster.point_count = count;
                best_cluster.median_distance_m = median_distance;
            }

            start = end;
        }

        if (best_cluster.point_count < cfg_.min_cluster_points || best_cluster.median_distance_m < 0.0F) {
            diagnostics.used_fallback = true;
            const auto fallback = fusion_.medianDistanceInAngleWindow(cloud, center_angle, cfg_.window_half_deg);
            if (fallback.has_value()) {
                diagnostics.raw_distance_m = *fallback;
                diagnostics.smoothed_distance_m = *fallback;
            }
            return diagnostics;
        }

        if (!passesDistanceSanity(det, best_cluster.median_distance_m, best_cluster.point_count)) {
            diagnostics.rejected_by_sanity = true;
            return diagnostics;
        }

        diagnostics.raw_distance_m = best_cluster.median_distance_m;
        diagnostics.smoothed_distance_m = best_cluster.median_distance_m;
        diagnostics.cluster_points = best_cluster.point_count;
        diagnostics.cluster_score = best_cluster.score;
        return diagnostics;
    }

    bool passesDistanceSanity(const YoloDetection& det, float distance_m, int cluster_points) const {
        const int box_height_px = std::max(0, det.bottom - det.top);
        const int box_width_px = std::max(0, det.right - det.left);

        if (distance_m < 0.8F && box_height_px < 70) {
            return false;
        }
        if (distance_m < 1.5F && box_height_px < 42) {
            return false;
        }
        if (distance_m < 2.5F && box_height_px < 24) {
            return false;
        }
        if (cluster_points < 6 && det.confidence < 0.55F) {
            return false;
        }
        if (box_width_px > 0 && box_height_px > 0 && box_width_px > box_height_px * 8 && det.confidence < 0.6F) {
            return false;
        }
        return true;
    }

private:
    const SensorFusion& fusion_;
    DistanceFusionConfig cfg_;
    std::uint64_t frame_index_ = 0;
    std::vector<TrackState> tracks_;
};

}  // namespace rk3588::modules