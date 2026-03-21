#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "core/data_types.hpp"

#include <rockchip/mpp_buffer.h>
#include <rockchip/mpp_frame.h>
#include <rockchip/mpp_packet.h>
#include <rockchip/rk_mpi.h>
#include <rockchip/rk_mpi_cmd.h>

namespace rk3588::modules {

struct EncodedPacket {
    const void* data = nullptr;
    std::size_t len = 0;
    std::int64_t dts_us = -1;
    std::int64_t pts_us = -1;
    bool eos = false;
    MppPacket handle = nullptr;
};

class MPPEncoder {
public:
    MPPEncoder();
    ~MPPEncoder();

    bool init(MppCodingType coding_type = MPP_VIDEO_CodingAVC);
    bool configure(std::uint32_t width, std::uint32_t height, std::uint32_t fps = 30,
                   std::uint32_t bitrate_bps = 2 * 1024 * 1024, std::uint32_t gop = 60);
    bool encodeFrame(const core::FramePacket& frame, bool eos = false);
    bool encodeNv12Cpu(const std::uint8_t* nv12,
                       std::uint32_t width,
                       std::uint32_t height,
                       std::uint32_t src_stride,
                       std::int64_t pts_us,
                       std::int64_t dts_us,
                       bool eos = false);
    bool getCodecHeader(std::vector<std::uint8_t>* header);

    // Returns -1 on error, 0 when no packet available, 1 when one packet is returned.
    int getPacket(EncodedPacket* out_packet);
    bool releasePacket(EncodedPacket* packet);

private:
    bool ensureImportedBuffer(const core::FramePacket& frame);
    bool buildInputFrame(const core::FramePacket& frame, MppFrame* out_frame, bool eos);
    bool ensureCpuBuffer(std::size_t required_size);
    void cleanup();

private:
    MppCtx mpp_ctx_ = nullptr;
    MppApi* mpp_api_ = nullptr;
    MppEncCfg enc_cfg_ = nullptr;
    MppCodingType coding_type_ = MPP_VIDEO_CodingAVC;

    std::uint32_t width_ = 0;
    std::uint32_t height_ = 0;
    std::uint32_t hor_stride_ = 0;
    std::uint32_t ver_stride_ = 0;
    std::uint32_t fps_ = 30;
    std::uint32_t bitrate_bps_ = 2 * 1024 * 1024;
    std::uint32_t gop_ = 60;
    std::uint64_t input_frame_id_ = 0;

    std::map<int, MppBuffer> imported_fd_map_;
    MppBuffer cpu_input_buffer_ = nullptr;
    std::size_t cpu_input_buffer_size_ = 0;
};

}  // namespace rk3588::modules
