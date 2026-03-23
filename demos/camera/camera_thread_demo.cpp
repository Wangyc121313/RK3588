#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include "core/bounded_queue.hpp"
#include "core/data_types.hpp"
#include "camera/camera_capture.hpp"

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
    bool printed_first = false;
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
        if (!printed_first) {
            std::cout << "first frame metadata: index=" << frame.buffer_index
                      << " dma_fd=" << frame.dma_fd
                      << " w=" << frame.width
                      << " h=" << frame.height
                      << " fourcc=" << frame.pixel_format << '\n';
            printed_first = true;
        }

        if (!camera.requeueBuffer(frame.buffer_index)) {
            std::cerr << "failed to requeue camera buffer index=" << frame.buffer_index << '\n';
        }
    }

    camera.stop();
    frame_queue.close();

    const double fps = static_cast<double>(consumed) / static_cast<double>(run_seconds);
    std::cout << "camera_thread_demo done: consumed_frames=" << consumed
              << " run_seconds=" << run_seconds
              << " avg_fps=" << fps << '\n';
    return 0;
}
