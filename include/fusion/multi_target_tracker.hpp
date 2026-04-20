#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace rk3588::modules {

struct TrackObservation {
    int class_id = -1;
    float confidence = 0.0F;
    float angle_deg = 0.0F;
    float distance_m = -1.0F;
    int left = 0;
    int top = 0;
    int right = 0;
    int bottom = 0;
};

struct TrackEstimate {
    int track_id = -1;
    std::uint32_t age_frames = 0;
    std::uint32_t idle_frames = 0;
    bool confirmed = false;
    bool is_ghost = false;
    float filtered_distance_m = -1.0F;
    float filtered_angle_deg = 0.0F;
    float lateral_offset_m = 0.0F;
    float radial_velocity_mps = 0.0F;
    float lateral_velocity_mps = 0.0F;
    float closing_speed_mps = 0.0F;
    float ttc_s = -1.0F;
};

struct MultiTargetTrackerConfig {
    float max_center_delta_px = 110.0F;
    float max_angle_delta_deg = 16.0F;
    float max_distance_delta_m = 2.2F;
    float min_iou_for_match = 0.05F;
    float iou_cost_weight = 0.40F;
    float distance_alpha = 0.30F;
    float angle_alpha = 0.28F;
    float velocity_alpha = 0.35F;
    float center_velocity_alpha = 0.28F;
    float ghost_velocity_decay = 0.78F;
    std::uint32_t ghost_keep_frames = 4;
    std::uint32_t max_idle_frames = 12;
    std::uint32_t min_confirmed_hits = 3;
    float min_closing_speed_mps = 0.05F;
};

class MultiTargetTracker {
public:
    explicit MultiTargetTracker(MultiTargetTrackerConfig cfg) : cfg_(cfg) {}

    void update(std::uint64_t frame_id,
                std::uint64_t timestamp_ms,
                const std::vector<TrackObservation>& observations,
                std::vector<TrackEstimate>* estimates) {
        ageTracks();
        if (estimates == nullptr) {
            return;
        }

        estimates->assign(observations.size(), {});
        std::vector<bool> used_tracks(tracks_.size(), false);

        for (std::size_t obs_index = 0; obs_index < observations.size(); ++obs_index) {
            const auto& obs = observations[obs_index];
            const int best_index = findBestTrack(obs, used_tracks);
            if (best_index >= 0) {
                used_tracks[static_cast<std::size_t>(best_index)] = true;
                (*estimates)[obs_index] = updateTrack(tracks_[static_cast<std::size_t>(best_index)], obs, frame_id, timestamp_ms);
                continue;
            }

            TrackState track;
            track.track_id = next_track_id_++;
            track.class_id = obs.class_id;
            track.center_x = centerX(obs);
            track.center_y = centerY(obs);
            track.filtered_angle_deg = obs.angle_deg;
            track.filtered_distance_m = obs.distance_m;
            track.lateral_offset_m = computeLateralOffset(obs.angle_deg, obs.distance_m);
            track.box_width_px = boxWidth(obs);
            track.box_height_px = boxHeight(obs);
            track.last_frame_id = frame_id;
            track.last_timestamp_ms = timestamp_ms;
            track.hits = 1;
            tracks_.push_back(track);
            used_tracks.push_back(true);
            (*estimates)[obs_index] = makeEstimate(track);
        }

        for (std::size_t i = 0; i < tracks_.size() && i < used_tracks.size(); ++i) {
            if (!used_tracks[i]) {
                advanceGhostTrack(tracks_[i], frame_id, timestamp_ms);
            }
        }
    }

private:
    struct TrackState {
        int track_id = -1;
        int class_id = -1;
        float center_x = 0.0F;
        float center_y = 0.0F;
        float filtered_angle_deg = 0.0F;
        float filtered_distance_m = -1.0F;
        float lateral_offset_m = 0.0F;
        float radial_velocity_mps = 0.0F;
        float lateral_velocity_mps = 0.0F;
        float center_velocity_x_pxps = 0.0F;
        float center_velocity_y_pxps = 0.0F;
        float box_width_px = 0.0F;
        float box_height_px = 0.0F;
        std::uint64_t last_frame_id = 0;
        std::uint64_t last_timestamp_ms = 0;
        std::uint32_t hits = 0;
        std::uint32_t idle_frames = 0;
        bool ghost = false;
    };

    static float centerX(const TrackObservation& obs) {
        return 0.5F * static_cast<float>(obs.left + obs.right);
    }

    static float centerY(const TrackObservation& obs) {
        return 0.5F * static_cast<float>(obs.top + obs.bottom);
    }

    static float boxWidth(const TrackObservation& obs) {
        return static_cast<float>(std::max(0, obs.right - obs.left));
    }

    static float boxHeight(const TrackObservation& obs) {
        return static_cast<float>(std::max(0, obs.bottom - obs.top));
    }

    static float intersectionOverUnion(float left_a,
                                       float top_a,
                                       float right_a,
                                       float bottom_a,
                                       float left_b,
                                       float top_b,
                                       float right_b,
                                       float bottom_b) {
        const float inter_left = std::max(left_a, left_b);
        const float inter_top = std::max(top_a, top_b);
        const float inter_right = std::min(right_a, right_b);
        const float inter_bottom = std::min(bottom_a, bottom_b);

        const float inter_w = std::max(0.0F, inter_right - inter_left);
        const float inter_h = std::max(0.0F, inter_bottom - inter_top);
        const float inter_area = inter_w * inter_h;
        const float area_a = std::max(0.0F, right_a - left_a) * std::max(0.0F, bottom_a - top_a);
        const float area_b = std::max(0.0F, right_b - left_b) * std::max(0.0F, bottom_b - top_b);
        const float denom = area_a + area_b - inter_area;
        if (denom <= 1e-3F) {
            return 0.0F;
        }
        return inter_area / denom;
    }

    static float observationTrackIoU(const TrackObservation& obs, const TrackState& track) {
        const float half_w = 0.5F * std::max(2.0F, track.box_width_px);
        const float half_h = 0.5F * std::max(2.0F, track.box_height_px);
        const float track_left = track.center_x - half_w;
        const float track_top = track.center_y - half_h;
        const float track_right = track.center_x + half_w;
        const float track_bottom = track.center_y + half_h;
        return intersectionOverUnion(static_cast<float>(obs.left),
                                     static_cast<float>(obs.top),
                                     static_cast<float>(obs.right),
                                     static_cast<float>(obs.bottom),
                                     track_left,
                                     track_top,
                                     track_right,
                                     track_bottom);
    }

    static float computeLateralOffset(float angle_deg, float distance_m) {
        if (distance_m < 0.0F) {
            return 0.0F;
        }
        return distance_m * std::sin(angle_deg * static_cast<float>(M_PI) / 180.0F);
    }

    void ageTracks() {
        tracks_.erase(
            std::remove_if(tracks_.begin(), tracks_.end(), [&](const TrackState& track) {
                return track.idle_frames > cfg_.max_idle_frames;
            }),
            tracks_.end());
    }

    int findBestTrack(const TrackObservation& obs, const std::vector<bool>& used_tracks) const {
        float best_cost = std::numeric_limits<float>::max();
        int best_index = -1;
        const float cx = centerX(obs);
        const float cy = centerY(obs);

        for (std::size_t i = 0; i < tracks_.size(); ++i) {
            if (used_tracks[i]) {
                continue;
            }
            const auto& track = tracks_[i];
            if (track.class_id != obs.class_id) {
                continue;
            }

            const float dx = cx - track.center_x;
            const float dy = cy - track.center_y;
            const float center_delta = std::sqrt(dx * dx + dy * dy);
            if (center_delta > cfg_.max_center_delta_px) {
                continue;
            }

            const float angle_delta = std::fabs(obs.angle_deg - track.filtered_angle_deg);
            if (angle_delta > cfg_.max_angle_delta_deg) {
                continue;
            }

            const float iou = observationTrackIoU(obs, track);
            if (iou < cfg_.min_iou_for_match && center_delta > 0.45F * cfg_.max_center_delta_px) {
                continue;
            }

            float cost = center_delta / std::max(1.0F, cfg_.max_center_delta_px) +
                         0.35F * angle_delta / std::max(1.0F, cfg_.max_angle_delta_deg) +
                         cfg_.iou_cost_weight * (1.0F - iou);

            if (obs.distance_m >= 0.0F && track.filtered_distance_m >= 0.0F) {
                const float distance_delta = std::fabs(obs.distance_m - track.filtered_distance_m);
                if (distance_delta > cfg_.max_distance_delta_m) {
                    continue;
                }
                cost += 0.45F * distance_delta / std::max(0.5F, cfg_.max_distance_delta_m);
            }

            if (cost < best_cost) {
                best_cost = cost;
                best_index = static_cast<int>(i);
            }
        }

        return best_index;
    }

    TrackEstimate updateTrack(TrackState& track,
                              const TrackObservation& obs,
                              std::uint64_t frame_id,
                              std::uint64_t timestamp_ms) const {
        const float prev_distance = track.filtered_distance_m;
        const float prev_lateral = track.lateral_offset_m;
        const float prev_center_x = track.center_x;
        const float prev_center_y = track.center_y;
        const std::uint64_t prev_timestamp_ms = track.last_timestamp_ms;

        track.center_x = centerX(obs);
        track.center_y = centerY(obs);
        track.filtered_angle_deg = prev_timestamp_ms == 0
            ? obs.angle_deg
            : track.filtered_angle_deg + cfg_.angle_alpha * (obs.angle_deg - track.filtered_angle_deg);

        if (obs.distance_m >= 0.0F) {
            track.filtered_distance_m = prev_distance < 0.0F
                ? obs.distance_m
                : prev_distance + cfg_.distance_alpha * (obs.distance_m - prev_distance);
        }

        const float observed_width = boxWidth(obs);
        const float observed_height = boxHeight(obs);
        if (track.box_width_px <= 0.0F || track.box_height_px <= 0.0F) {
            track.box_width_px = observed_width;
            track.box_height_px = observed_height;
        } else {
            track.box_width_px = 0.65F * track.box_width_px + 0.35F * observed_width;
            track.box_height_px = 0.65F * track.box_height_px + 0.35F * observed_height;
        }

        const float lateral_offset = computeLateralOffset(track.filtered_angle_deg, track.filtered_distance_m);
        const float dt_sec = (prev_timestamp_ms > 0 && timestamp_ms > prev_timestamp_ms)
            ? static_cast<float>(timestamp_ms - prev_timestamp_ms) / 1000.0F
            : 0.0F;
        if (dt_sec > 1e-3F) {
            const float raw_center_vx = (track.center_x - prev_center_x) / dt_sec;
            const float raw_center_vy = (track.center_y - prev_center_y) / dt_sec;
            track.center_velocity_x_pxps +=
                cfg_.center_velocity_alpha * (raw_center_vx - track.center_velocity_x_pxps);
            track.center_velocity_y_pxps +=
                cfg_.center_velocity_alpha * (raw_center_vy - track.center_velocity_y_pxps);

            if (prev_distance >= 0.0F && track.filtered_distance_m >= 0.0F) {
                const float raw_radial_velocity = (track.filtered_distance_m - prev_distance) / dt_sec;
                track.radial_velocity_mps += cfg_.velocity_alpha * (raw_radial_velocity - track.radial_velocity_mps);
            }
            const float raw_lateral_velocity = (lateral_offset - prev_lateral) / dt_sec;
            track.lateral_velocity_mps += cfg_.velocity_alpha * (raw_lateral_velocity - track.lateral_velocity_mps);
        }
        track.lateral_offset_m = lateral_offset;
        track.last_frame_id = frame_id;
        track.last_timestamp_ms = timestamp_ms;
        track.idle_frames = 0;
        track.ghost = false;
        ++track.hits;
        return makeEstimate(track);
    }

    void advanceGhostTrack(TrackState& track,
                           std::uint64_t frame_id,
                           std::uint64_t timestamp_ms) const {
        const float dt_sec = (track.last_timestamp_ms > 0 && timestamp_ms > track.last_timestamp_ms)
            ? static_cast<float>(timestamp_ms - track.last_timestamp_ms) / 1000.0F
            : 0.0F;

        if (track.idle_frames < cfg_.ghost_keep_frames && dt_sec > 1e-3F) {
            track.center_x += track.center_velocity_x_pxps * dt_sec;
            track.center_y += track.center_velocity_y_pxps * dt_sec;
            track.center_velocity_x_pxps *= cfg_.ghost_velocity_decay;
            track.center_velocity_y_pxps *= cfg_.ghost_velocity_decay;

            if (track.filtered_distance_m >= 0.0F) {
                track.filtered_distance_m = std::max(0.0F, track.filtered_distance_m + track.radial_velocity_mps * dt_sec);
            }
            track.lateral_offset_m += track.lateral_velocity_mps * dt_sec;
            track.radial_velocity_mps *= cfg_.ghost_velocity_decay;
            track.lateral_velocity_mps *= cfg_.ghost_velocity_decay;
        }

        ++track.idle_frames;
        track.ghost = track.idle_frames <= cfg_.ghost_keep_frames;
        track.last_frame_id = frame_id;
        track.last_timestamp_ms = timestamp_ms;
    }

    TrackEstimate makeEstimate(const TrackState& track) const {
        TrackEstimate estimate;
        estimate.track_id = track.track_id;
        estimate.age_frames = track.hits;
        estimate.idle_frames = track.idle_frames;
        estimate.confirmed = track.hits >= cfg_.min_confirmed_hits;
        estimate.is_ghost = track.ghost;
        estimate.filtered_distance_m = track.filtered_distance_m;
        estimate.filtered_angle_deg = track.filtered_angle_deg;
        estimate.lateral_offset_m = track.lateral_offset_m;
        estimate.radial_velocity_mps = track.radial_velocity_mps;
        estimate.lateral_velocity_mps = track.lateral_velocity_mps;
        estimate.closing_speed_mps = std::max(0.0F, -track.radial_velocity_mps);
        if (estimate.filtered_distance_m > 0.0F && estimate.closing_speed_mps >= cfg_.min_closing_speed_mps) {
            estimate.ttc_s = estimate.filtered_distance_m / estimate.closing_speed_mps;
        }
        return estimate;
    }

private:
    MultiTargetTrackerConfig cfg_;
    int next_track_id_ = 1;
    std::vector<TrackState> tracks_;
};

}  // namespace rk3588::modules