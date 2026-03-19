#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>

#include "core/bounded_queue.hpp"
#include "core/data_types.hpp"
#include "modules/camera_capture.hpp"

namespace {

void savePpm(const std::string& path, const rk3588::core::FramePacket& frame, std::uint32_t width, std::uint32_t height) {
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("failed to open output file: " + path);
    }

    output << "P6\n" << width << ' ' << height << "\n255\n";
    output.write(reinterpret_cast<const char*>(frame.pixels.data()), static_cast<std::streamsize>(frame.pixels.size()));
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string device = argc > 1 ? argv[1] : "/dev/video0";
    const int run_seconds = argc > 2 ? std::max(1, std::atoi(argv[2])) : 10;
    const std::uint32_t width = argc > 3 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[3]))) : 640;
    const std::uint32_t height = argc > 4 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[4]))) : 480;

    rk3588::core::BoundedQueue<rk3588::core::FramePacket> frame_queue(4);
    rk3588::modules::CameraCapture camera;

    if (!camera.init(device, width, height, 4)) {
        std::cerr << "camera init failed" << '\n';
        return 1;
    }

    if (!camera.start(&frame_queue)) {
        std::cerr << "camera thread start failed" << '\n';
        return 1;
    }

    std::uint64_t consumed = 0;
    bool saved_first = false;
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

        ++consumed;
        if (!saved_first) {
            try {
                savePpm("camera_thread_first_frame.ppm", frame, width, height);
                saved_first = true;
            } catch (const std::exception& ex) {
                std::cerr << "save first frame failed: " << ex.what() << '\n';
            }
        }
    }

    camera.stop();
    frame_queue.close();

    const double fps = static_cast<double>(consumed) / static_cast<double>(run_seconds);
    std::cout << "camera_thread_demo done: consumed_frames=" << consumed
              << " run_seconds=" << run_seconds
              << " avg_fps=" << fps << '\n';
    std::cout << "first frame saved to camera_thread_first_frame.ppm" << '\n';
    return 0;
}
