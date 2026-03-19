#include "modules/rga_processor.hpp"

#include <cstring>
#include <iostream>

#include "RgaUtils.h"
#include "im2d.hpp"

namespace rk3588::modules {

bool RGAProcessor::init(int src_w, int src_h, int dst_w, int dst_h) {
    if (src_w <= 0 || src_h <= 0 || dst_w <= 0 || dst_h <= 0) {
        return false;
    }

    src_w_ = src_w;
    src_h_ = src_h;
    dst_w_ = dst_w;
    dst_h_ = dst_h;

    rgb_src_stage_.resize(static_cast<std::size_t>(src_w_) * src_h_ * 3);
    return true;
}

bool RGAProcessor::processNv12ToRgbResize(const std::uint8_t* src_nv12, std::uint8_t* dst_rgb) {
    if (src_nv12 == nullptr || dst_rgb == nullptr || src_w_ <= 0 || src_h_ <= 0 || dst_w_ <= 0 || dst_h_ <= 0) {
        return false;
    }

    const int src_nv12_size = src_w_ * src_h_ * 3 / 2;
    const int src_rgb_size = src_w_ * src_h_ * 3;
    const int dst_rgb_size = dst_w_ * dst_h_ * 3;

    auto src_nv12_handle = importbuffer_virtualaddr(const_cast<std::uint8_t*>(src_nv12), src_nv12_size);
    auto src_rgb_handle = importbuffer_virtualaddr(rgb_src_stage_.data(), src_rgb_size);
    auto dst_rgb_handle = importbuffer_virtualaddr(dst_rgb, dst_rgb_size);

    if (src_nv12_handle == 0 || src_rgb_handle == 0 || dst_rgb_handle == 0) {
        std::cerr << "RGA importbuffer_virtualaddr failed" << '\n';
        if (src_nv12_handle) releasebuffer_handle(src_nv12_handle);
        if (src_rgb_handle) releasebuffer_handle(src_rgb_handle);
        if (dst_rgb_handle) releasebuffer_handle(dst_rgb_handle);
        return false;
    }

    rga_buffer_t src_nv12_img = wrapbuffer_handle(src_nv12_handle, src_w_, src_h_, RK_FORMAT_YCbCr_420_SP);
    rga_buffer_t src_rgb_img = wrapbuffer_handle(src_rgb_handle, src_w_, src_h_, RK_FORMAT_RGB_888);
    rga_buffer_t dst_rgb_img = wrapbuffer_handle(dst_rgb_handle, dst_w_, dst_h_, RK_FORMAT_RGB_888);

    IM_STATUS status = imcheck(src_nv12_img, src_rgb_img, {}, {}, 0);
    if (status != IM_STATUS_NOERROR) {
        std::cerr << "RGA imcheck(cvtcolor) failed: " << imStrError(status) << '\n';
        releasebuffer_handle(src_nv12_handle);
        releasebuffer_handle(src_rgb_handle);
        releasebuffer_handle(dst_rgb_handle);
        return false;
    }

    status = imcvtcolor(src_nv12_img, src_rgb_img, RK_FORMAT_YCbCr_420_SP, RK_FORMAT_RGB_888);
    if (status != IM_STATUS_SUCCESS) {
        std::cerr << "RGA imcvtcolor failed: " << imStrError(status) << '\n';
        releasebuffer_handle(src_nv12_handle);
        releasebuffer_handle(src_rgb_handle);
        releasebuffer_handle(dst_rgb_handle);
        return false;
    }

    status = imcheck(src_rgb_img, dst_rgb_img, {}, {}, 0);
    if (status != IM_STATUS_NOERROR) {
        std::cerr << "RGA imcheck(resize) failed: " << imStrError(status) << '\n';
        releasebuffer_handle(src_nv12_handle);
        releasebuffer_handle(src_rgb_handle);
        releasebuffer_handle(dst_rgb_handle);
        return false;
    }

    status = imresize(src_rgb_img, dst_rgb_img);
    if (status != IM_STATUS_SUCCESS) {
        std::cerr << "RGA imresize failed: " << imStrError(status) << '\n';
        releasebuffer_handle(src_nv12_handle);
        releasebuffer_handle(src_rgb_handle);
        releasebuffer_handle(dst_rgb_handle);
        return false;
    }

    releasebuffer_handle(src_nv12_handle);
    releasebuffer_handle(src_rgb_handle);
    releasebuffer_handle(dst_rgb_handle);
    return true;
}

}  // namespace rk3588::modules
