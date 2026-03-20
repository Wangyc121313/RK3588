#include "modules/mpp_encoder.hpp"

#include <cstring>
#include <iostream>

#include "core/mpp_common_utils.hpp"

namespace {

constexpr int kMppOk = 0;
constexpr int kMppErrTimeout = -1101;
constexpr int kPollTimeoutMs = 500;
constexpr std::uint32_t kMppFmtNv12 = 10;  // MPP_FMT_YUV420SP

}  // namespace

namespace rk3588::modules {

MPPEncoder::MPPEncoder() = default;

MPPEncoder::~MPPEncoder() {
    cleanup();
}

bool MPPEncoder::init(MppCodingType coding_type) {
    if (mpp_ctx_ != nullptr && mpp_api_ != nullptr) {
        return true;
    }

    coding_type_ = coding_type;

    if (mpp_create(&mpp_ctx_, &mpp_api_) != kMppOk || mpp_ctx_ == nullptr || mpp_api_ == nullptr) {
        std::cerr << "mpp_create failed\n";
        cleanup();
        return false;
    }

    if (mpp_init(mpp_ctx_, MPP_CTX_ENC, coding_type_) != kMppOk) {
        std::cerr << "mpp_init encoder failed\n";
        cleanup();
        return false;
    }

    if (mpp_enc_cfg_init(&enc_cfg_) != kMppOk || enc_cfg_ == nullptr) {
        std::cerr << "mpp_enc_cfg_init failed\n";
        cleanup();
        return false;
    }

    if (mpp_api_->control(mpp_ctx_, MPP_ENC_GET_CFG, enc_cfg_) != kMppOk) {
        std::cerr << "MPP_ENC_GET_CFG failed\n";
        cleanup();
        return false;
    }

    int timeout_ms = kPollTimeoutMs;
    (void)mpp_api_->control(mpp_ctx_, MPP_SET_INPUT_TIMEOUT, &timeout_ms);
    (void)mpp_api_->control(mpp_ctx_, MPP_SET_OUTPUT_TIMEOUT, &timeout_ms);
    return true;
}

bool MPPEncoder::configure(std::uint32_t width, std::uint32_t height, std::uint32_t fps,
                           std::uint32_t bitrate_bps, std::uint32_t gop) {
    if (enc_cfg_ == nullptr || mpp_api_ == nullptr || mpp_ctx_ == nullptr || width == 0 || height == 0) {
        return false;
    }

    width_ = width;
    height_ = height;
    hor_stride_ = core::mpp::align16(width_);
    ver_stride_ = core::mpp::align16(height_);
    fps_ = fps > 0 ? fps : 30;
    bitrate_bps_ = bitrate_bps > 0 ? bitrate_bps : (width_ * height_ * fps_ / 4);
    gop_ = gop > 0 ? gop : (fps_ * 2);

    mpp_enc_cfg_set_s32(enc_cfg_, "rc:mode", 1 /* MPP_ENC_RC_MODE_CBR */);
    mpp_enc_cfg_set_s32(enc_cfg_, "rc:fps_in_flex", 0);
    mpp_enc_cfg_set_s32(enc_cfg_, "rc:fps_in_num", static_cast<int>(fps_));
    mpp_enc_cfg_set_s32(enc_cfg_, "rc:fps_in_denorm", 1);
    mpp_enc_cfg_set_s32(enc_cfg_, "rc:fps_out_flex", 0);
    mpp_enc_cfg_set_s32(enc_cfg_, "rc:fps_out_num", static_cast<int>(fps_));
    mpp_enc_cfg_set_s32(enc_cfg_, "rc:fps_out_denorm", 1);
    mpp_enc_cfg_set_s32(enc_cfg_, "rc:gop", static_cast<int>(gop_));
    mpp_enc_cfg_set_s32(enc_cfg_, "rc:bps_target", static_cast<int>(bitrate_bps_));

    mpp_enc_cfg_set_s32(enc_cfg_, "prep:width", static_cast<int>(width_));
    mpp_enc_cfg_set_s32(enc_cfg_, "prep:height", static_cast<int>(height_));
    mpp_enc_cfg_set_s32(enc_cfg_, "prep:hor_stride", static_cast<int>(hor_stride_));
    mpp_enc_cfg_set_s32(enc_cfg_, "prep:ver_stride", static_cast<int>(ver_stride_));
    mpp_enc_cfg_set_s32(enc_cfg_, "prep:format", static_cast<int>(kMppFmtNv12));

    mpp_enc_cfg_set_s32(enc_cfg_, "codec:type", coding_type_);
    mpp_enc_cfg_set_s32(enc_cfg_, "split:mode", 0);
    mpp_enc_cfg_set_s32(enc_cfg_, "split:arg", 0);
    mpp_enc_cfg_set_s32(enc_cfg_, "split:out", 0);

    if (mpp_api_->control(mpp_ctx_, MPP_ENC_SET_CFG, enc_cfg_) != kMppOk) {
        std::cerr << "MPP_ENC_SET_CFG failed\n";
        return false;
    }

    int header_mode = 1;  // MPP_ENC_HEADER_MODE_EACH_IDR
    (void)mpp_api_->control(mpp_ctx_, MPP_ENC_SET_HEADER_MODE, &header_mode);

    input_frame_id_ = 0;
    return true;
}

bool MPPEncoder::ensureImportedBuffer(const core::FramePacket& frame) {
    if (frame.dma_fd < 0 || frame.buffer_size == 0) {
        return false;
    }

    if (imported_fd_map_.find(frame.dma_fd) != imported_fd_map_.end()) {
        return true;
    }

    MppBufferInfo info{};
    info.type = MPP_BUFFER_TYPE_EXT_DMA;
    info.fd = frame.dma_fd;
    info.size = frame.buffer_size;
    info.index = static_cast<int>(frame.buffer_index);

    MppBuffer imported = nullptr;
    if (mpp_buffer_import(&imported, &info) != kMppOk || imported == nullptr) {
        std::cerr << "mpp_buffer_import failed for fd=" << frame.dma_fd << '\n';
        return false;
    }

    imported_fd_map_[frame.dma_fd] = imported;
    return true;
}

bool MPPEncoder::buildInputFrame(const core::FramePacket& frame, MppFrame* out_frame, bool eos) {
    if (out_frame == nullptr) {
        return false;
    }

    const auto it = imported_fd_map_.find(frame.dma_fd);
    if (it == imported_fd_map_.end() || it->second == nullptr) {
        return false;
    }

    if (mpp_frame_init(out_frame) != kMppOk || *out_frame == nullptr) {
        return false;
    }

    const std::uint32_t w = frame.width;
    const std::uint32_t h = frame.height;
    const std::uint32_t hs = frame.hor_stride > 0 ? frame.hor_stride : core::mpp::align16(w);
    const std::uint32_t vs = frame.ver_stride > 0 ? frame.ver_stride : core::mpp::align16(h);

    mpp_frame_set_width(*out_frame, static_cast<int>(w));
    mpp_frame_set_height(*out_frame, static_cast<int>(h));
    mpp_frame_set_hor_stride(*out_frame, static_cast<int>(hs));
    mpp_frame_set_ver_stride(*out_frame, static_cast<int>(vs));
    mpp_frame_set_fmt(*out_frame, static_cast<MppFrameFormat>(kMppFmtNv12));
    mpp_frame_set_buffer(*out_frame, it->second);

    std::int64_t pts_us = frame.pts_us;
    std::int64_t dts_us = frame.dts_us;
    if (pts_us < 0 && dts_us >= 0) {
        pts_us = dts_us;
    }
    if (dts_us < 0 && pts_us >= 0) {
        dts_us = pts_us;
    }
    if (pts_us < 0 || dts_us < 0) {
        const std::int64_t step_us = (fps_ > 0) ? (1000000LL / static_cast<std::int64_t>(fps_)) : 33333LL;
        const std::int64_t fallback = static_cast<std::int64_t>(input_frame_id_) * step_us;
        if (pts_us < 0) {
            pts_us = fallback;
        }
        if (dts_us < 0) {
            dts_us = pts_us;
        }
    }

    mpp_frame_set_pts(*out_frame, pts_us);
    mpp_frame_set_dts(*out_frame, dts_us);
    if (eos) {
        mpp_frame_set_eos(*out_frame, 1);
    }

    ++input_frame_id_;
    return true;
}

bool MPPEncoder::encodeFrame(const core::FramePacket& frame, bool eos) {
    if (mpp_ctx_ == nullptr || mpp_api_ == nullptr) {
        return false;
    }
    if (frame.pixel_format != 0x3231564eU /* V4L2_PIX_FMT_NV12 */) {
        return false;
    }
    if (!ensureImportedBuffer(frame)) {
        return false;
    }

    MppFrame input = nullptr;
    if (!buildInputFrame(frame, &input, eos) || input == nullptr) {
        return false;
    }

    const int ret = mpp_api_->encode_put_frame(mpp_ctx_, input);
    (void)mpp_frame_deinit(&input);
    return ret == kMppOk;
}

bool MPPEncoder::ensureCpuBuffer(std::size_t required_size) {
    if (required_size == 0) {
        return false;
    }
    if (cpu_input_buffer_ != nullptr && cpu_input_buffer_size_ >= required_size) {
        return true;
    }

    if (cpu_input_buffer_ != nullptr) {
        mpp_buffer_put(cpu_input_buffer_);
        cpu_input_buffer_ = nullptr;
        cpu_input_buffer_size_ = 0;
    }

    MppBuffer buffer = nullptr;
    if (mpp_buffer_get(nullptr, &buffer, required_size) != kMppOk || buffer == nullptr) {
        std::cerr << "mpp_buffer_get failed for cpu input size=" << required_size << '\n';
        return false;
    }

    cpu_input_buffer_ = buffer;
    cpu_input_buffer_size_ = required_size;
    return true;
}

bool MPPEncoder::encodeNv12Cpu(const std::uint8_t* nv12,
                               std::uint32_t width,
                               std::uint32_t height,
                               std::uint32_t src_stride,
                               std::int64_t pts_us,
                               std::int64_t dts_us,
                               bool eos) {
    if (mpp_ctx_ == nullptr || mpp_api_ == nullptr || nv12 == nullptr || width == 0 || height == 0) {
        return false;
    }

    const std::uint32_t hs = core::mpp::align16(width);
    const std::uint32_t vs = core::mpp::align16(height);
    const std::size_t required = static_cast<std::size_t>(hs) * static_cast<std::size_t>(vs) * 3 / 2;
    if (!ensureCpuBuffer(required)) {
        return false;
    }

    void* dst_ptr = mpp_buffer_get_ptr(cpu_input_buffer_);
    if (dst_ptr == nullptr) {
        return false;
    }

    std::uint8_t* dst = static_cast<std::uint8_t*>(dst_ptr);
    std::memset(dst, 0, required);

    const std::uint32_t in_stride = (src_stride >= width) ? src_stride : width;
    const std::uint8_t* src_y = nv12;
    const std::uint8_t* src_uv = nv12 + static_cast<std::size_t>(in_stride) * static_cast<std::size_t>(height);
    std::uint8_t* dst_y = dst;
    std::uint8_t* dst_uv = dst + static_cast<std::size_t>(hs) * static_cast<std::size_t>(vs);

    for (std::uint32_t y = 0; y < height; ++y) {
        std::memcpy(dst_y + static_cast<std::size_t>(y) * hs,
                    src_y + static_cast<std::size_t>(y) * in_stride,
                    width);
    }
    for (std::uint32_t y = 0; y < (height / 2); ++y) {
        std::memcpy(dst_uv + static_cast<std::size_t>(y) * hs,
                    src_uv + static_cast<std::size_t>(y) * in_stride,
                    width);
    }

    MppFrame input = nullptr;
    if (mpp_frame_init(&input) != kMppOk || input == nullptr) {
        return false;
    }

    mpp_frame_set_width(input, static_cast<int>(width));
    mpp_frame_set_height(input, static_cast<int>(height));
    mpp_frame_set_hor_stride(input, static_cast<int>(hs));
    mpp_frame_set_ver_stride(input, static_cast<int>(vs));
    mpp_frame_set_fmt(input, static_cast<MppFrameFormat>(kMppFmtNv12));
    mpp_frame_set_buffer(input, cpu_input_buffer_);

    mpp_frame_set_pts(input, pts_us);
    mpp_frame_set_dts(input, dts_us);
    if (eos) {
        mpp_frame_set_eos(input, 1);
    }

    ++input_frame_id_;
    const int ret = mpp_api_->encode_put_frame(mpp_ctx_, input);
    (void)mpp_frame_deinit(&input);
    return ret == kMppOk;
}

int MPPEncoder::getPacket(EncodedPacket* out_packet) {
    if (out_packet == nullptr || mpp_api_ == nullptr || mpp_ctx_ == nullptr) {
        return -1;
    }

    *out_packet = EncodedPacket{};

    MppPacket packet = nullptr;
    const int ret = mpp_api_->encode_get_packet(mpp_ctx_, &packet);
    if (ret == kMppErrTimeout || packet == nullptr) {
        return 0;
    }
    if (ret != kMppOk) {
        return -1;
    }

    void* pos = mpp_packet_get_pos(packet);
    const std::size_t len = mpp_packet_get_length(packet);
    if (pos == nullptr || len == 0) {
        (void)mpp_packet_deinit(&packet);
        return 0;
    }

    out_packet->data = pos;
    out_packet->len = len;
    out_packet->dts_us = mpp_packet_get_dts(packet);
    out_packet->pts_us = mpp_packet_get_pts(packet);
    out_packet->eos = (mpp_packet_get_eos(packet) != 0);
    out_packet->handle = packet;
    return 1;
}

bool MPPEncoder::releasePacket(EncodedPacket* packet) {
    if (packet == nullptr) {
        return false;
    }
    if (packet->handle != nullptr) {
        if (mpp_packet_deinit(&packet->handle) != kMppOk) {
            return false;
        }
    }
    packet->data = nullptr;
    packet->len = 0;
    packet->dts_us = -1;
    packet->pts_us = -1;
    packet->eos = false;
    return true;
}

void MPPEncoder::cleanup() {
    for (auto& it : imported_fd_map_) {
        if (it.second != nullptr) {
            mpp_buffer_put(it.second);
        }
    }
    imported_fd_map_.clear();

    if (cpu_input_buffer_ != nullptr) {
        mpp_buffer_put(cpu_input_buffer_);
        cpu_input_buffer_ = nullptr;
        cpu_input_buffer_size_ = 0;
    }

    if (enc_cfg_ != nullptr) {
        mpp_enc_cfg_deinit(enc_cfg_);
        enc_cfg_ = nullptr;
    }

    if (mpp_ctx_ != nullptr) {
        mpp_destroy(mpp_ctx_);
        mpp_ctx_ = nullptr;
        mpp_api_ = nullptr;
    }
}

}  // namespace rk3588::modules
