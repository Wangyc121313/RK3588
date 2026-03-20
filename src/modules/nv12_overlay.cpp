#include "modules/nv12_overlay.hpp"

#include <algorithm>

namespace rk3588::modules {

namespace {

int clampi(int value, int low, int high) {
    return std::max(low, std::min(value, high));
}

void rgbToYuv(std::uint8_t r,
              std::uint8_t g,
              std::uint8_t b,
              std::uint8_t* y,
              std::uint8_t* u,
              std::uint8_t* v) {
    if (y == nullptr || u == nullptr || v == nullptr) {
        return;
    }

    const int yi = ((66 * static_cast<int>(r) + 129 * static_cast<int>(g) + 25 * static_cast<int>(b) + 128) >> 8) + 16;
    const int ui = ((-38 * static_cast<int>(r) - 74 * static_cast<int>(g) + 112 * static_cast<int>(b) + 128) >> 8) + 128;
    const int vi = ((112 * static_cast<int>(r) - 94 * static_cast<int>(g) - 18 * static_cast<int>(b) + 128) >> 8) + 128;

    *y = static_cast<std::uint8_t>(clampi(yi, 0, 255));
    *u = static_cast<std::uint8_t>(clampi(ui, 0, 255));
    *v = static_cast<std::uint8_t>(clampi(vi, 0, 255));
}

void setNv12Pixel(std::uint8_t* nv12,
                  int width,
                  int height,
                  int stride,
                  int x,
                  int y,
                  std::uint8_t yy,
                  std::uint8_t uu,
                  std::uint8_t vv) {
    if (nv12 == nullptr || width <= 0 || height <= 0 || stride < width) {
        return;
    }
    if (x < 0 || x >= width || y < 0 || y >= height) {
        return;
    }

    std::uint8_t* y_plane = nv12;
    std::uint8_t* uv_plane = nv12 + static_cast<std::size_t>(stride) * static_cast<std::size_t>(height);

    y_plane[static_cast<std::size_t>(y) * static_cast<std::size_t>(stride) + static_cast<std::size_t>(x)] = yy;

    const int uv_x = x & ~1;
    const int uv_y = y & ~1;
    const std::size_t uv_idx = static_cast<std::size_t>(uv_y / 2) * static_cast<std::size_t>(stride) + static_cast<std::size_t>(uv_x);
    uv_plane[uv_idx] = uu;
    uv_plane[uv_idx + 1] = vv;
}

}  // namespace

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
                       int thickness) {
    if (nv12 == nullptr || width <= 0 || height <= 0 || stride < width) {
        return;
    }

    left = clampi(left, 0, width - 1);
    right = clampi(right, 0, width - 1);
    top = clampi(top, 0, height - 1);
    bottom = clampi(bottom, 0, height - 1);
    if (right <= left || bottom <= top) {
        return;
    }

    const int t = std::max(1, thickness);

    for (int k = 0; k < t; ++k) {
        const int y_top = clampi(top + k, 0, height - 1);
        const int y_bottom = clampi(bottom - k, 0, height - 1);
        for (int x = left; x <= right; ++x) {
            setNv12Pixel(nv12, width, height, stride, x, y_top, y, u, v);
            setNv12Pixel(nv12, width, height, stride, x, y_bottom, y, u, v);
        }

        const int x_left = clampi(left + k, 0, width - 1);
        const int x_right = clampi(right - k, 0, width - 1);
        for (int yy = top; yy <= bottom; ++yy) {
            setNv12Pixel(nv12, width, height, stride, x_left, yy, y, u, v);
            setNv12Pixel(nv12, width, height, stride, x_right, yy, y, u, v);
        }
    }
}

void drawDetectionsNv12(std::uint8_t* nv12,
                        int width,
                        int height,
                        int stride,
                        const std::vector<YoloDetection>& detections,
                        int thickness) {
    if (nv12 == nullptr || width <= 0 || height <= 0 || stride < width) {
        return;
    }

    for (const auto& det : detections) {
        const std::uint8_t r = static_cast<std::uint8_t>((53 * (det.class_id + 3)) % 255);
        const std::uint8_t g = static_cast<std::uint8_t>((97 * (det.class_id + 7)) % 255);
        const std::uint8_t b = static_cast<std::uint8_t>((193 * (det.class_id + 11)) % 255);

        std::uint8_t y = 235;
        std::uint8_t u = 128;
        std::uint8_t v = 128;
        rgbToYuv(r, g, b, &y, &u, &v);

        drawRectangleNv12(nv12,
                          width,
                          height,
                          stride,
                          det.left,
                          det.top,
                          det.right,
                          det.bottom,
                          y,
                          u,
                          v,
                          thickness);
    }
}

}  // namespace rk3588::modules
