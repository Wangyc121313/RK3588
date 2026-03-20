#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "core/bounded_queue.hpp"
#include "core/data_types.hpp"
#include "modules/camera_capture.hpp"
#include "modules/frame_overlay.hpp"
#include "modules/rga_processor.hpp"
#include "modules/rknn_runner.hpp"

namespace {

class FfmpegRtspStreamer {
public:
    ~FfmpegRtspStreamer() { stop(); }

    bool start(const std::string& url, int width, int height, int fps, bool rtsp_listen_mode) {
        if (pipe_ != nullptr || width <= 0 || height <= 0 || fps <= 0) {
            return false;
        }

        width_ = width;
        height_ = height;
        frame_size_ = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_) * 3;

        const std::string cmd =
            "ffmpeg -hide_banner -loglevel error "
            "-f rawvideo -pixel_format rgb24 "
            "-video_size " + std::to_string(width_) + "x" + std::to_string(height_) + " "
            "-framerate " + std::to_string(fps) + " -i - "
            "-an -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p "
            "-f rtsp " + (rtsp_listen_mode ? std::string("-rtsp_flags listen ") : std::string()) +
            "-rtsp_transport tcp \"" + url + "\"";

        pipe_ = popen(cmd.c_str(), "w");
        if (pipe_ == nullptr) {
            std::cerr << "failed to start ffmpeg streamer" << '\n';
            return false;
        }
        return true;
    }

    bool pushFrame(const std::uint8_t* rgb, std::size_t bytes) {
        if (pipe_ == nullptr || rgb == nullptr || bytes < frame_size_) {
            return false;
        }

        const std::size_t written = fwrite(rgb, 1, frame_size_, pipe_);
        if (written != frame_size_) {
            return false;
        }
        fflush(pipe_);
        return true;
    }

    void stop() {
        if (pipe_ != nullptr) {
            pclose(pipe_);
            pipe_ = nullptr;
        }
    }

private:
    FILE* pipe_ = nullptr;
    int width_ = 0;
    int height_ = 0;
    std::size_t frame_size_ = 0;
};

}  // namespace

int main(int argc, char* argv[]) {
    const std::string model_path = argc > 1 ? argv[1] : "models/yolov8n.rknn";
    const std::string rtsp_url = argc > 2 ? argv[2] : "rtsp://127.0.0.1:8554/live";
    const std::string device = argc > 3 ? argv[3] : "/dev/video0";
    const int run_seconds = argc > 4 ? std::max(1, std::atoi(argv[4])) : 30;
    const std::uint32_t src_w = argc > 5 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[5]))) : 640;
    const std::uint32_t src_h = argc > 6 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[6]))) : 480;
    const std::uint32_t dst_w = argc > 7 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[7]))) : 640;
    const std::uint32_t dst_h = argc > 8 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[8]))) : 640;
    const int fps = argc > 9 ? std::max(1, std::atoi(argv[9])) : 25;
    const std::string labels_path = argc > 10
        ? argv[10]
        : "third_party/rknn_model_zoo/examples/yolov8/model/coco_80_labels_list.txt";
    const bool rtsp_listen_mode = [] {
        const char* env = std::getenv("RK3588_RTSP_LISTEN");
        return env != nullptr && env[0] != '\0' && env[0] != '0';
    }();

    rk3588::core::BoundedQueue<rk3588::core::FramePacket> frame_queue(4);
    rk3588::modules::CameraCapture camera;
    rk3588::modules::RGAProcessor rga;
    rk3588::modules::RKNNRunner rknn;
    FfmpegRtspStreamer streamer;

    if (!camera.init(device, src_w, src_h, 4)) {
        std::cerr << "camera init failed" << '\n';
        return 1;
    }
    if (!camera.start(&frame_queue)) {
        std::cerr << "camera start failed" << '\n';
        return 1;
    }

    std::vector<std::uint8_t> rgb(static_cast<std::size_t>(dst_w) * dst_h * 3);
    bool initialized = false;
    std::uint64_t consumed = 0;

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
            if (!rknn.init(model_path, static_cast<int>(dst_w), static_cast<int>(dst_h), labels_path)) {
                std::cerr << "rknn init failed" << '\n';
                camera.requeueBuffer(frame.buffer_index);
                break;
            }
            if (!streamer.start(rtsp_url, static_cast<int>(dst_w), static_cast<int>(dst_h), fps,
                                rtsp_listen_mode)) {
                std::cerr << "ffmpeg rtsp streamer start failed, check RTSP server url=" << rtsp_url << '\n';
                camera.requeueBuffer(frame.buffer_index);
                break;
            }
            std::cout << "streaming to " << rtsp_url
                      << " mode=" << (rtsp_listen_mode ? "listen" : "push") << '\n';
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

        rk3588::modules::drawDetectionsRgb(
            rgb.data(), static_cast<int>(dst_w), static_cast<int>(dst_h), detections);

        if (!streamer.pushFrame(rgb.data(), rgb.size())) {
            std::cerr << "stream push failed (ffmpeg pipe broken?)" << '\n';
            break;
        }

        ++consumed;
    }

    camera.stop();
    frame_queue.close();
    streamer.stop();

    const double fps_out = run_seconds > 0
        ? static_cast<double>(consumed) / static_cast<double>(run_seconds)
        : 0.0;
    std::cout << "camera_rga_rknn_rtsp_demo done: consumed_frames=" << consumed
              << " run_seconds=" << run_seconds
              << " avg_fps=" << fps_out << '\n';
    return 0;
}
