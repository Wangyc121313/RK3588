#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "infer/rknn_runner.hpp"

namespace rk3588::modules {

void drawRectangleRgb(std::uint8_t* rgb, int width, int height,
					  int left, int top, int right, int bottom,
					  std::uint8_t r, std::uint8_t g, std::uint8_t b,
					  int thickness = 2);

void drawTextRgb(std::uint8_t* rgb, int width, int height,
				 int x, int y, const std::string& text,
				 std::uint8_t r, std::uint8_t g, std::uint8_t b,
				 int scale = 2);

void drawDetectionsRgb(std::uint8_t* rgb, int width, int height,
					   const std::vector<YoloDetection>& detections);

}  // namespace rk3588::modules
