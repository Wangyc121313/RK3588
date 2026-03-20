#pragma once

#include <cstdint>
#include <vector>

#include "modules/rknn_runner.hpp"

namespace rk3588::modules {

void drawRectangleNv12(std::uint8_t* nv12,
                       int width,
                       int height,
                       int stride,
                       int left,
                       int top,
                       int right,
                       int bottom,
                       std::uint8_t y,
                       std::uint8_t u,
                       std::uint8_t v,
                       int thickness = 2);

void drawDetectionsNv12(std::uint8_t* nv12,
                        int width,
                        int height,
                        int stride,
                        const std::vector<YoloDetection>& detections,
                        int thickness = 2);

}  // namespace rk3588::modules
