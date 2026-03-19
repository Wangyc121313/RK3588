#include <chrono>
#include <cstdint>
#include <iostream>
#include <vector>

#include "modules/rga_processor.hpp"

int main() {
    constexpr int src_w = 640;
    constexpr int src_h = 480;
    constexpr int dst_w = 640;
    constexpr int dst_h = 640;

    rk3588::modules::RGAProcessor processor;
    if (!processor.init(src_w, src_h, dst_w, dst_h)) {
        std::cerr << "RGAProcessor init failed" << '\n';
        return 1;
    }

    std::vector<std::uint8_t> src_nv12(static_cast<std::size_t>(src_w) * src_h * 3 / 2, 128);
    std::vector<std::uint8_t> dst_rgb(static_cast<std::size_t>(dst_w) * dst_h * 3, 0);

    const auto t0 = std::chrono::steady_clock::now();
    const bool ok = processor.processNv12ToRgbResize(src_nv12.data(), dst_rgb.data());
    const auto t1 = std::chrono::steady_clock::now();
    const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

    if (!ok) {
        std::cerr << "RGA process failed" << '\n';
        return 1;
    }

    std::cout << "rga_demo success: NV12 " << src_w << "x" << src_h
              << " -> RGB " << dst_w << "x" << dst_h
              << ", elapsed=" << elapsed_us << " us" << '\n';

    return 0;
}
