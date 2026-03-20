#include "modules/frame_overlay.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <string>

namespace rk3588::modules {

namespace {

int clampi(int v, int low, int high) {
    return std::max(low, std::min(v, high));
}

void setPixel(std::uint8_t* rgb, int width, int height, int x, int y,
              std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    if (x < 0 || y < 0 || x >= width || y >= height) {
        return;
    }
    const std::size_t idx = (static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x)) * 3;
    rgb[idx + 0] = r;
    rgb[idx + 1] = g;
    rgb[idx + 2] = b;
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
        case '%': return {0x19, 0x19, 0x02, 0x04, 0x08, 0x13, 0x13};
        case ' ': return {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        default:  return {0x00, 0x0e, 0x11, 0x02, 0x04, 0x00, 0x04};
    }
}

}  // namespace

void drawRectangleRgb(std::uint8_t* rgb, int width, int height,
                      int left, int top, int right, int bottom,
                      std::uint8_t r, std::uint8_t g, std::uint8_t b,
                      int thickness) {
    if (rgb == nullptr || width <= 0 || height <= 0) {
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
        const int y1 = clampi(top + k, 0, height - 1);
        const int y2 = clampi(bottom - k, 0, height - 1);
        for (int x = left; x <= right; ++x) {
            setPixel(rgb, width, height, x, y1, r, g, b);
            setPixel(rgb, width, height, x, y2, r, g, b);
        }

        const int x1 = clampi(left + k, 0, width - 1);
        const int x2 = clampi(right - k, 0, width - 1);
        for (int y = top; y <= bottom; ++y) {
            setPixel(rgb, width, height, x1, y, r, g, b);
            setPixel(rgb, width, height, x2, y, r, g, b);
        }
    }
}

void drawTextRgb(std::uint8_t* rgb, int width, int height,
                 int x, int y, const std::string& text,
                 std::uint8_t r, std::uint8_t g, std::uint8_t b,
                 int scale) {
    if (rgb == nullptr || width <= 0 || height <= 0 || text.empty()) {
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
                        setPixel(rgb, width, height,
                                 cursor_x + gx * s + dx,
                                 y + gy * s + dy,
                                 r, g, b);
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

void drawDetectionsRgb(std::uint8_t* rgb, int width, int height,
                       const std::vector<YoloDetection>& detections) {
    if (rgb == nullptr || width <= 0 || height <= 0) {
        return;
    }

    for (const auto& det : detections) {
        const std::uint8_t cr = static_cast<std::uint8_t>((53 * (det.class_id + 3)) % 255);
        const std::uint8_t cg = static_cast<std::uint8_t>((97 * (det.class_id + 7)) % 255);
        const std::uint8_t cb = static_cast<std::uint8_t>((193 * (det.class_id + 11)) % 255);

        drawRectangleRgb(rgb, width, height, det.left, det.top, det.right, det.bottom, cr, cg, cb, 2);

        const int label_x = clampi(det.left, 0, std::max(0, width - 1));
        const int label_y = clampi(det.top - 14, 0, std::max(0, height - 1));
        const std::string label = det.class_name + " " + std::to_string(det.confidence).substr(0, 4);
        drawTextRgb(rgb, width, height, label_x, label_y, label, cr, cg, cb, 2);
    }
}

}  // namespace rk3588::modules
