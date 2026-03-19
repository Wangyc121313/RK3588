#pragma once

#include <cstdint>
#include <vector>

namespace rk3588::modules {

class RGAProcessor {
public:
    bool init(int src_w, int src_h, int dst_w, int dst_h);

    // CPU pointer version for MVP validation. Next step can switch to DMA fd zero-copy path.
    bool processNv12ToRgbResize(const std::uint8_t* src_nv12, std::uint8_t* dst_rgb);

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
