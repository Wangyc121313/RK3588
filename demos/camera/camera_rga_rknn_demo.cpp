#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "core/bounded_queue.hpp"
#include "core/data_types.hpp"
#include "camera/camera_capture.hpp"
#include "video/rga_processor.hpp"
#include "infer/rknn_runner.hpp"

int main(int argc, char* argv[]) {
    const std::string model_path = argc > 1 ? argv[1] : "models/yolov8n.rknn";
    const std::string device = argc > 2 ? argv[2] : "/dev/video0";
    const int run_seconds = argc > 3 ? std::max(1, std::atoi(argv[3])) : 3;
    const std::uint32_t src_w = argc > 4 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[4]))) : 640;
    const std::uint32_t src_h = argc > 5 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[5]))) : 480;
    const std::uint32_t dst_w = argc > 6 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[6]))) : 640;
    const std::uint32_t dst_h = argc > 7 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[7]))) : 640;
    const std::string labels_path = argc > 8
        ? argv[8]
        : "third_party/rknn_model_zoo/examples/yolov8/model/coco_80_labels_list.txt";

    rk3588::core::BoundedQueue<rk3588::core::FramePacket> frame_queue(4);
    rk3588::modules::CameraCapture camera;
    rk3588::modules::RGAProcessor rga;
    rk3588::modules::RKNNRunner rknn;

    if (!camera.init(device, src_w, src_h, 4)) {
        std::cerr << "camera init failed" << '\n';
        return 1;
    }
    if (!camera.start(&frame_queue)) {
        std::cerr << "camera start failed" << '\n';
        return 1;
    }

    bool initialized = false;
    std::uint64_t consumed = 0;
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

            if (!rknn.init(model_path, static_cast<int>(dst_w), static_cast<int>(dst_h), labels_path)) {
                std::cerr << "rknn init failed" << '\n';
                camera.requeueBuffer(frame.buffer_index);
                break;
            }

            initialized = true;
        }

        const bool rga_ok = rga.processDmaFdToRgbResize(frame.dma_fd, frame.pixel_format, rgb.data());
        if (!camera.requeueBuffer(frame.buffer_index)) {
            std::cerr << "failed to requeue camera buffer index=" << frame.buffer_index << '\n';
        }
        if (!rga_ok) {
            std::cerr << "rga process failed frame=" << frame.frame_id << '\n';
            continue;
        }

        std::vector<rk3588::modules::YoloDetection> detections;
        const bool infer_ok = rknn.inferRgb(
            rgb.data(), rgb.size(), static_cast<int>(dst_w), static_cast<int>(dst_h), &detections);
        if (!infer_ok) {
            std::cerr << "rknn infer failed frame=" << frame.frame_id << '\n';
            continue;
        }

        for (const auto& det : detections) {
            std::cout << det.class_name
                      << " @ (" << det.left << ' ' << det.top << ' ' << det.right << ' ' << det.bottom << ") "
                      << det.confidence << '\n';
        }

        ++consumed;
    }

    camera.stop();
    frame_queue.close();

    const double fps = static_cast<double>(consumed) / static_cast<double>(run_seconds);
    std::cout << "camera_rga_rknn_demo done: consumed_frames=" << consumed
              << " run_seconds=" << run_seconds
              << " avg_fps=" << fps << '\n';
    return 0;
}
