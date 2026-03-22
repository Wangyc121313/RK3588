#include "modules/nv12_overlay.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>

namespace rk3588::modules {

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
                  int scale);

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

using Glyph = std::array<std::uint8_t, 7>;  // 5-bit rows

Glyph glyphFor(char c) {
    const char ch = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    switch (ch) {
        case 'a': return {0x00, 0x0e, 0x01, 0x0f, 0x11, 0x0f, 0x00};
        case 'b': return {0x10, 0x10, 0x1e, 0x11, 0x11, 0x1e, 0x00};
        case 'c': return {0x00, 0x0e, 0x11, 0x10, 0x11, 0x0e, 0x00};
        case 'd': return {0x01, 0x01, 0x0f, 0x11, 0x11, 0x0f, 0x00};
        case 'e': return {0x00, 0x0e, 0x11, 0x1f, 0x10, 0x0e, 0x00};
        case 'f': return {0x03, 0x04, 0x0f, 0x04, 0x04, 0x04, 0x00};
        case 'g': return {0x00, 0x0f, 0x11, 0x0f, 0x01, 0x0e, 0x00};
        case 'h': return {0x10, 0x10, 0x1e, 0x11, 0x11, 0x11, 0x00};
        case 'i': return {0x04, 0x00, 0x0c, 0x04, 0x04, 0x0e, 0x00};
        case 'j': return {0x02, 0x00, 0x06, 0x02, 0x12, 0x0c, 0x00};
        case 'k': return {0x10, 0x12, 0x14, 0x18, 0x14, 0x12, 0x00};
        case 'l': return {0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e, 0x00};
        case 'm': return {0x00, 0x1a, 0x15, 0x15, 0x15, 0x15, 0x00};
        case 'n': return {0x00, 0x1e, 0x11, 0x11, 0x11, 0x11, 0x00};
        case 'o': return {0x00, 0x0e, 0x11, 0x11, 0x11, 0x0e, 0x00};
        case 'p': return {0x00, 0x1e, 0x11, 0x1e, 0x10, 0x10, 0x00};
        case 'q': return {0x00, 0x0f, 0x11, 0x0f, 0x01, 0x01, 0x00};
        case 'r': return {0x00, 0x16, 0x19, 0x10, 0x10, 0x10, 0x00};
        case 's': return {0x00, 0x0f, 0x10, 0x0e, 0x01, 0x1e, 0x00};
        case 't': return {0x04, 0x1f, 0x04, 0x04, 0x04, 0x03, 0x00};
        case 'u': return {0x00, 0x11, 0x11, 0x11, 0x13, 0x0d, 0x00};
        case 'v': return {0x00, 0x11, 0x11, 0x11, 0x0a, 0x04, 0x00};
        case 'w': return {0x00, 0x11, 0x11, 0x15, 0x15, 0x0a, 0x00};
        case 'x': return {0x00, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x00};
        case 'y': return {0x00, 0x11, 0x11, 0x0f, 0x01, 0x0e, 0x00};
        case 'z': return {0x00, 0x1f, 0x02, 0x04, 0x08, 0x1f, 0x00};
        case '0': return {0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e};
        case '1': return {0x04, 0x0c, 0x14, 0x04, 0x04, 0x04, 0x1f};
        case '2': return {0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f};
        case '3': return {0x1e, 0x01, 0x01, 0x0e, 0x01, 0x01, 0x1e};
        case '4': return {0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02};
        case '5': return {0x1f, 0x10, 0x10, 0x1e, 0x01, 0x01, 0x1e};
        case '6': return {0x0e, 0x10, 0x10, 0x1e, 0x11, 0x11, 0x0e};
        case '7': return {0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08};
        case '8': return {0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e};
        case '9': return {0x0e, 0x11, 0x11, 0x0f, 0x01, 0x01, 0x0e};
        case '.': return {0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x0c};
        case '-': return {0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00};
        case '_': return {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f};
        case ':': return {0x00, 0x0c, 0x0c, 0x00, 0x0c, 0x0c, 0x00};
        case '/': return {0x01, 0x02, 0x04, 0x08, 0x10, 0x00, 0x00};
        case ' ': return {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        default:  return {0x00, 0x0e, 0x11, 0x02, 0x04, 0x00, 0x04};
    }
}

void fillRectNv12(std::uint8_t* nv12,
                  int width,
                  int height,
                  int stride,
                  int left,
                  int top,
                  int right,
                  int bottom,
                  std::uint8_t y,
                  std::uint8_t u,
                  std::uint8_t v) {
    if (nv12 == nullptr || width <= 0 || height <= 0 || stride < width) {
        return;
    }

    left = clampi(left, 0, width - 1);
    right = clampi(right, 0, width - 1);
    top = clampi(top, 0, height - 1);
    bottom = clampi(bottom, 0, height - 1);
    if (right < left || bottom < top) {
        return;
    }

    for (int yy = top; yy <= bottom; ++yy) {
        for (int xx = left; xx <= right; ++xx) {
            setNv12Pixel(nv12, width, height, stride, xx, yy, y, u, v);
        }
    }
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

        char label[160] = {0};
        if (det.distance_m > 0.0F) {
            std::snprintf(label,
                          sizeof(label),
                          "%s %.2f %.2fm",
                          det.class_name.empty() ? "obj" : det.class_name.c_str(),
                          det.confidence,
                          det.distance_m);
        } else {
            std::snprintf(label,
                          sizeof(label),
                          "%s %.2f N/A",
                          det.class_name.empty() ? "obj" : det.class_name.c_str(),
                          det.confidence);
        }
        const int label_x = clampi(det.left, 0, std::max(0, width - 1));
        const int label_y = clampi(det.top - 16, 0, std::max(0, height - 1));
        drawTextNv12(nv12, width, height, stride, label_x, label_y, label, y, u, v, 2);
    }
}

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
                  int scale) {
    if (nv12 == nullptr || width <= 0 || height <= 0 || stride < width || text.empty()) {
        return;
    }

    const int s = std::max(1, scale);
    int cursor_x = x;
    for (char c : text) {
        const Glyph glyph = glyphFor(c);
        for (int gy = 0; gy < 7; ++gy) {
            for (int gx = 0; gx < 5; ++gx) {
                const bool on = (glyph[static_cast<std::size_t>(gy)] >> (4 - gx)) & 0x1;
                if (!on) {
                    continue;
                }
                for (int dy = 0; dy < s; ++dy) {
                    for (int dx = 0; dx < s; ++dx) {
                        setNv12Pixel(nv12, width, height, stride,
                                     cursor_x + gx * s + dx,
                                     y + gy * s + dy,
                                     yy, uu, vv);
                    }
                }
            }
        }
        cursor_x += 6 * s;
        if (cursor_x >= width - 6 * s) {
            break;
        }
    }
}

void drawHudLinesNv12(std::uint8_t* nv12,
                      int width,
                      int height,
                      int stride,
                      const std::vector<std::string>& lines,
                      int x,
                      int y,
                      int scale) {
    if (nv12 == nullptr || width <= 0 || height <= 0 || stride < width || lines.empty()) {
        return;
    }

    const int s = std::max(1, scale);
    const int line_h = 8 * s;
    int max_chars = 0;
    for (const auto& line : lines) {
        max_chars = std::max(max_chars, static_cast<int>(line.size()));
    }

    const int box_w = std::min(width - x - 1, max_chars * 6 * s + 8);
    const int box_h = std::min(height - y - 1, static_cast<int>(lines.size()) * line_h + 8);
    if (box_w > 0 && box_h > 0) {
        fillRectNv12(nv12, width, height, stride,
                     x - 4, y - 4, x - 4 + box_w, y - 4 + box_h,
                     32, 128, 128);
    }

    int cursor_y = y;
    for (const auto& line : lines) {
        drawTextNv12(nv12, width, height, stride, x, cursor_y, line,
                     235, 128, 128, s);
        cursor_y += line_h;
        if (cursor_y >= height - line_h) {
            break;
        }
    }
}

}  // namespace rk3588::modules
