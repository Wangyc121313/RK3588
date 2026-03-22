#pragma once

#include <cstdint>
#include <string>
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

void drawTextNv12(std::uint8_t* nv12,
                  int width,
                  int height,
                  int stride,
                  int x,
                  int y,
                  const std::string& text,
                  std::uint8_t yy,
                  std::uint8_t uu,
                  std::uint8_t vv,
                  int scale = 2);

void drawHudLinesNv12(std::uint8_t* nv12,
                      int width,
                      int height,
                      int stride,
                      const std::vector<std::string>& lines,
                      int x = 8,
                      int y = 8,
                      int scale = 2);

}  // namespace rk3588::modules
