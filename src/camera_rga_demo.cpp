#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "core/bounded_queue.hpp"
#include "core/data_types.hpp"
#include "modules/camera_capture.hpp"
#include "modules/rga_processor.hpp"

namespace {

void savePpm(const std::string& path, const std::vector<std::uint8_t>& rgb, std::uint32_t width, std::uint32_t height) {
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("failed to open output file: " + path);
    }

    output << "P6\n" << width << ' ' << height << "\n255\n";
    output.write(reinterpret_cast<const char*>(rgb.data()), static_cast<std::streamsize>(rgb.size()));
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string device = argc > 1 ? argv[1] : "/dev/video0";
    const int run_seconds = argc > 2 ? std::max(1, std::atoi(argv[2])) : 3;
    const std::uint32_t src_w = argc > 3 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[3]))) : 640;
    const std::uint32_t src_h = argc > 4 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[4]))) : 480;
    const std::uint32_t dst_w = argc > 5 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[5]))) : 640;
    const std::uint32_t dst_h = argc > 6 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[6]))) : 640;

    rk3588::core::BoundedQueue<rk3588::core::FramePacket> frame_queue(4);
    rk3588::modules::CameraCapture camera;
    rk3588::modules::RGAProcessor rga;

    if (!camera.init(device, src_w, src_h, 4)) {
        std::cerr << "camera init failed" << '\n';
        return 1;
    }

    if (!camera.start(&frame_queue)) {
        std::cerr << "camera thread start failed" << '\n';
        return 1;
    }

    std::uint64_t consumed = 0;
    bool initialized = false;
    bool saved_first = false;
    std::vector<std::uint8_t> rgb;

    const auto start = std::chrono::steady_clock::now();
    while (true) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= run_seconds) {
            break;
        }

        rk3588::core::FramePacket frame;
        if (!frame_queue.pop_for(frame, std::chrono::milliseconds(200))) {
            continue;
        }

        if (!initialized) {
            if (!rga.init(static_cast<int>(frame.width), static_cast<int>(frame.height),
                          static_cast<int>(dst_w), static_cast<int>(dst_h))) {
                std::cerr << "rga init failed" << '\n';
                camera.requeueBuffer(frame.buffer_index);
                break;
            }
            rgb.resize(static_cast<std::size_t>(dst_w) * dst_h * 3);
            initialized = true;
        }

        const bool ok = rga.processDmaFdToRgbResize(frame.dma_fd, frame.pixel_format, rgb.data());
        if (!camera.requeueBuffer(frame.buffer_index)) {
            std::cerr << "failed to requeue camera buffer index=" << frame.buffer_index << '\n';
        }

        if (!ok) {
            std::cerr << "rga process failed for frame=" << frame.frame_id << '\n';
            continue;
        }

        ++consumed;
        if (!saved_first) {
            try {
                savePpm("camera_rga_first_frame.ppm", rgb, dst_w, dst_h);
                std::cout << "saved first RGB frame to camera_rga_first_frame.ppm" << '\n';
                saved_first = true;
            } catch (const std::exception& ex) {
                std::cerr << "save ppm failed: " << ex.what() << '\n';
            }
        }
    }

    camera.stop();
    frame_queue.close();

    const double fps = static_cast<double>(consumed) / static_cast<double>(run_seconds);
    std::cout << "camera_rga_demo done: consumed_frames=" << consumed
              << " run_seconds=" << run_seconds
              << " avg_fps=" << fps << '\n';

    return 0;
}
