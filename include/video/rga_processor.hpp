#pragma once

#include <cstdint>
#include <vector>

namespace rk3588::modules {

class RGAProcessor {
public:
	bool init(int src_w, int src_h, int dst_w, int dst_h);

	// CPU pointer version for MVP validation. Next step can switch to DMA fd zero-copy path.
	bool processNv12ToRgbResize(const std::uint8_t* src_nv12, std::uint8_t* dst_rgb);

	// Camera zero-copy path: source image comes from a V4L2 exported DMA fd.
	// Destination is RGB888 in caller-provided CPU buffer, ready for RKNN preprocessing/debug dump.
	bool processDmaFdToRgbResize(int src_dma_fd, std::uint32_t src_fourcc, std::uint8_t* dst_rgb);

	// Packed422 (YUYV family) CPU buffer to NV12 CPU buffer via RGA color conversion.
	// Returns false when format/stride is unsupported so caller can fallback to CPU conversion.
	bool processPacked422ToNv12(const std::uint8_t* src,
	std::uint32_t src_fourcc,
	std::uint32_t src_stride,
	std::uint8_t* dst_nv12,
	std::uint32_t dst_stride);

	[[nodiscard]] int srcWidth() const { return src_w_; }
	[[nodiscard]] int srcHeight() const { return src_h_; }
	[[nodiscard]] int dstWidth() const { return dst_w_; }
	[[nodiscard]] int dstHeight() const { return dst_h_; }

private:
	int src_w_ = 0;
	int src_h_ = 0;
	int dst_w_ = 0;
	int dst_h_ = 0;
	std::vector<std::uint8_t> rgb_src_stage_;
};

}  // namespace rk3588::modules
