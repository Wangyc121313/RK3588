#include <chrono>
#include <cstdint>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <linux/videodev2.h>

#include "core/bounded_queue.hpp"
#include "core/data_types.hpp"
#include "modules/camera_capture.hpp"
#include "modules/mpp_encoder.hpp"
#include "modules/nv12_overlay.hpp"
#include "modules/rga_processor.hpp"
#include "modules/rknn_runner.hpp"

namespace {

bool convertYuyvToNv12(const std::uint8_t* yuyv,
                       std::uint32_t width,
                       std::uint32_t height,
                       std::uint32_t yuyv_stride,
                       std::uint32_t nv12_stride,
                       std::vector<std::uint8_t>* nv12_out) {
    if (yuyv == nullptr || nv12_out == nullptr || width == 0 || height == 0) {
        return false;
    }
    if ((width & 1U) != 0U || (height & 1U) != 0U) {
        return false;
    }

    const std::uint32_t in_stride = yuyv_stride >= width * 2 ? yuyv_stride : width * 2;
    const std::uint32_t out_stride = nv12_stride >= width ? nv12_stride : width;
    const std::size_t out_size = static_cast<std::size_t>(out_stride) * height * 3 / 2;
    nv12_out->assign(out_size, 0);

    std::uint8_t* dst_y = nv12_out->data();
    std::uint8_t* dst_uv = dst_y + static_cast<std::size_t>(out_stride) * height;

    for (std::uint32_t y = 0; y < height; ++y) {
        const std::uint8_t* src_row = yuyv + static_cast<std::size_t>(y) * in_stride;
        std::uint8_t* y_row = dst_y + static_cast<std::size_t>(y) * out_stride;
        std::uint8_t* uv_row = dst_uv + static_cast<std::size_t>(y / 2) * out_stride;

        for (std::uint32_t x = 0; x < width; x += 2) {
            const std::uint8_t y0 = src_row[x * 2 + 0];
            const std::uint8_t u = src_row[x * 2 + 1];
            const std::uint8_t y1 = src_row[x * 2 + 2];
            const std::uint8_t v = src_row[x * 2 + 3];

            y_row[x] = y0;
            y_row[x + 1] = y1;

            if ((y & 1U) == 0U) {
                uv_row[x] = u;
                uv_row[x + 1] = v;
            }
        }
    }

    return true;
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string device = argc > 1 ? argv[1] : "/dev/video0";
    const std::uint32_t width = argc > 2 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[2]))) : 640;
    const std::uint32_t height = argc > 3 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[3]))) : 480;
    const int run_seconds = argc > 4 ? std::max(1, std::atoi(argv[4])) : 5;
    const std::string model_path = argc > 5 ? argv[5] : "models/yolov8n.rknn";
    const std::uint32_t model_w = argc > 6 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[6]))) : 640;
    const std::uint32_t model_h = argc > 7 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[7]))) : 640;
    const std::string labels_path = argc > 8
        ? argv[8]
        : "third_party/rknn_model_zoo/examples/yolov8/model/coco_80_labels_list.txt";

    rk3588::core::BoundedQueue<rk3588::core::FramePacket> queue(4);
    rk3588::modules::CameraCapture camera;
    rk3588::modules::MPPEncoder encoder;
    rk3588::modules::RGAProcessor rga;
    rk3588::modules::RKNNRunner rknn;

    if (!camera.init(device, width, height, 4) || !camera.start(&queue)) {
        std::cerr << "camera init/start failed\n";
        return 1;
    }

    bool initialized = false;
    std::uint64_t input_frames = 0;
    std::uint64_t output_packets = 0;
    std::vector<std::uint8_t> rgb;
    std::vector<std::uint8_t> nv12_scratch;

    const auto start = std::chrono::steady_clock::now();
    while (true) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= run_seconds) {
            break;
        }

        rk3588::core::FramePacket frame;
        if (!queue.pop_for(frame, std::chrono::milliseconds(200))) {
            continue;
        }

        if (!initialized) {
            if (!encoder.init() || !encoder.configure(frame.width, frame.height, 30)) {
                std::cerr << "mpp encoder init/config failed\n";
                camera.requeueBuffer(frame.buffer_index);
                break;
            }

            if (!rga.init(static_cast<int>(frame.width), static_cast<int>(frame.height),
                          static_cast<int>(model_w), static_cast<int>(model_h))) {
                std::cerr << "rga init failed\n";
                camera.requeueBuffer(frame.buffer_index);
                break;
            }

            if (!rknn.init(model_path, static_cast<int>(model_w), static_cast<int>(model_h), labels_path)) {
                std::cerr << "rknn init failed\n";
                camera.requeueBuffer(frame.buffer_index);
                break;
            }

            rgb.resize(static_cast<std::size_t>(model_w) * model_h * 3);
            initialized = true;
        }

        std::vector<rk3588::modules::YoloDetection> detections;
        const bool rga_ok = rga.processDmaFdToRgbResize(frame.dma_fd, frame.pixel_format, rgb.data());
        if (!rga_ok) {
            std::cerr << "rga process failed frame_id=" << frame.frame_id << '\n';
            camera.requeueBuffer(frame.buffer_index);
            continue;
        }
        if (!rknn.inferRgb(rgb.data(), rgb.size(), static_cast<int>(frame.width), static_cast<int>(frame.height), &detections)) {
            std::cerr << "rknn infer failed frame_id=" << frame.frame_id << '\n';
            camera.requeueBuffer(frame.buffer_index);
            continue;
        }

        bool ok = false;
        if (frame.pixel_format == V4L2_PIX_FMT_NV12 && frame.cpu_addr != 0) {
            rk3588::modules::drawDetectionsNv12(
                reinterpret_cast<std::uint8_t*>(frame.cpu_addr),
                static_cast<int>(frame.width),
                static_cast<int>(frame.height),
                static_cast<int>(frame.hor_stride > 0 ? frame.hor_stride : frame.width),
                detections,
                3);
            ok = encoder.encodeFrame(frame, false);
        } else if (frame.pixel_format == V4L2_PIX_FMT_YUYV && frame.cpu_addr != 0) {
            const std::uint32_t nv12_stride = (frame.width + 15U) & ~15U;
            if (!convertYuyvToNv12(
                    reinterpret_cast<const std::uint8_t*>(frame.cpu_addr),
                    frame.width,
                    frame.height,
                    frame.hor_stride,
                    nv12_stride,
                    &nv12_scratch)) {
                std::cerr << "yuyv->nv12 convert failed frame_id=" << frame.frame_id << '\n';
                camera.requeueBuffer(frame.buffer_index);
                continue;
            }

            rk3588::modules::drawDetectionsNv12(
                nv12_scratch.data(),
                static_cast<int>(frame.width),
                static_cast<int>(frame.height),
                static_cast<int>(nv12_stride),
                detections,
                3);

            ok = encoder.encodeNv12Cpu(
                nv12_scratch.data(),
                frame.width,
                frame.height,
                nv12_stride,
                frame.pts_us,
                frame.dts_us,
                false);
        } else {
            std::cerr << "unsupported pixel format or unavailable cpu_addr, fourcc=0x"
                      << std::hex << frame.pixel_format << std::dec
                      << " cpu_addr=" << frame.cpu_addr << '\n';
        }

        camera.requeueBuffer(frame.buffer_index);
        if (!ok) {
            std::cerr << "encode failed at frame_id=" << frame.frame_id << '\n';
            continue;
        }
        ++input_frames;

        while (true) {
            rk3588::modules::EncodedPacket pkt;
            const int got = encoder.getPacket(&pkt);
            if (got <= 0) {
                break;
            }
            ++output_packets;
            encoder.releasePacket(&pkt);
        }
    }

    camera.stop();
    queue.close();

    std::cout << "mpp_encoder_demo done: input_frames=" << input_frames
              << " output_packets=" << output_packets
              << " run_seconds=" << run_seconds << '\n';
    return 0;
}
