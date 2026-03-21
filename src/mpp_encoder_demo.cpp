#include <chrono>
#include <cstdint>
#include <algorithm>
#include <cstdio>
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
#include "modules/zlm_rtsp_publisher.hpp"

namespace {

bool isPacked422(std::uint32_t fourcc) {
    return fourcc == V4L2_PIX_FMT_YUYV || fourcc == V4L2_PIX_FMT_YVYU ||
           fourcc == V4L2_PIX_FMT_UYVY || fourcc == V4L2_PIX_FMT_VYUY;
}

const char* fourccName(std::uint32_t fourcc) {
    switch (fourcc) {
        case V4L2_PIX_FMT_YUYV:
            return "YUYV";
        case V4L2_PIX_FMT_YVYU:
            return "YVYU";
        case V4L2_PIX_FMT_UYVY:
            return "UYVY";
        case V4L2_PIX_FMT_VYUY:
            return "VYUY";
        case V4L2_PIX_FMT_NV12:
            return "NV12";
        default:
            return "UNKNOWN";
    }
}

std::uint32_t forced422FourccFromEnv() {
    const char* env = std::getenv("RK3588_FORCE_422");
    if (env == nullptr || env[0] == '\0') {
        return 0;
    }
    const std::string s(env);
    if (s == "YUYV" || s == "yuyv") {
        return V4L2_PIX_FMT_YUYV;
    }
    if (s == "YVYU" || s == "yvyu") {
        return V4L2_PIX_FMT_YVYU;
    }
    if (s == "UYVY" || s == "uyvy") {
        return V4L2_PIX_FMT_UYVY;
    }
    if (s == "VYUY" || s == "vyuy") {
        return V4L2_PIX_FMT_VYUY;
    }
    return 0;
}

void decode422Pair(std::uint32_t fourcc,
                   const std::uint8_t* src4,
                   std::uint8_t* y0,
                   std::uint8_t* y1,
                   std::uint8_t* u,
                   std::uint8_t* v) {
    switch (fourcc) {
        case V4L2_PIX_FMT_YUYV:
            *y0 = src4[0];
            *u = src4[1];
            *y1 = src4[2];
            *v = src4[3];
            break;
        case V4L2_PIX_FMT_YVYU:
            *y0 = src4[0];
            *v = src4[1];
            *y1 = src4[2];
            *u = src4[3];
            break;
        case V4L2_PIX_FMT_UYVY:
            *u = src4[0];
            *y0 = src4[1];
            *v = src4[2];
            *y1 = src4[3];
            break;
        case V4L2_PIX_FMT_VYUY:
            *v = src4[0];
            *y0 = src4[1];
            *u = src4[2];
            *y1 = src4[3];
            break;
        default:
            *y0 = *y1 = *u = *v = 0;
            break;
    }
}

std::uint32_t detectPacked422Fourcc(const std::uint8_t* src,
                                    std::uint32_t width,
                                    std::uint32_t height,
                                    std::uint32_t src_stride) {
    if (src == nullptr || width < 2 || height == 0) {
        return V4L2_PIX_FMT_YUYV;
    }

    const std::uint32_t in_stride = src_stride >= width * 2 ? src_stride : width * 2;
    const std::uint32_t candidates[] = {
        V4L2_PIX_FMT_YUYV,
        V4L2_PIX_FMT_YVYU,
        V4L2_PIX_FMT_UYVY,
        V4L2_PIX_FMT_VYUY,
    };

    double best_score = -1e30;
    std::uint32_t best_fourcc = V4L2_PIX_FMT_YUYV;

    const std::uint32_t rows = std::min<std::uint32_t>(height, 64);
    const std::uint32_t cols = std::min<std::uint32_t>(width, 640);

    for (const auto fourcc : candidates) {
        double sum_y = 0.0;
        double sum_yy = 0.0;
        double sum_u = 0.0;
        double sum_uu = 0.0;
        double sum_v = 0.0;
        double sum_vv = 0.0;
        std::uint64_t n = 0;

        for (std::uint32_t y = 0; y < rows; ++y) {
            const std::uint8_t* row = src + static_cast<std::size_t>(y) * in_stride;
            for (std::uint32_t x = 0; x + 1 < cols; x += 2) {
                std::uint8_t y0 = 0;
                std::uint8_t y1 = 0;
                std::uint8_t u = 0;
                std::uint8_t v = 0;
                decode422Pair(fourcc, row + x * 2, &y0, &y1, &u, &v);

                sum_y += static_cast<double>(y0) + static_cast<double>(y1);
                sum_yy += static_cast<double>(y0) * y0 + static_cast<double>(y1) * y1;
                sum_u += static_cast<double>(u);
                sum_uu += static_cast<double>(u) * u;
                sum_v += static_cast<double>(v);
                sum_vv += static_cast<double>(v) * v;
                ++n;
            }
        }

        if (n == 0) {
            continue;
        }

        const double y_count = static_cast<double>(n) * 2.0;
        const double mean_y = sum_y / y_count;
        const double mean_u = sum_u / static_cast<double>(n);
        const double mean_v = sum_v / static_cast<double>(n);

        const double var_y = (sum_yy / y_count) - mean_y * mean_y;
        const double var_u = (sum_uu / static_cast<double>(n)) - mean_u * mean_u;
        const double var_v = (sum_vv / static_cast<double>(n)) - mean_v * mean_v;

        const double score = var_y - 0.6 * (var_u + var_v) -
                             0.08 * (std::abs(mean_u - 128.0) + std::abs(mean_v - 128.0));
        if (score > best_score) {
            best_score = score;
            best_fourcc = fourcc;
        }
    }

    return best_fourcc;
}

bool envEnabled(const char* name) {
    const char* env = std::getenv(name);
    return env != nullptr && env[0] != '\0' && env[0] != '0';
}

bool convertPacked422ToNv12(const std::uint8_t* src,
                            std::uint32_t width,
                            std::uint32_t height,
                            std::uint32_t src_stride,
                            std::uint32_t src_fourcc,
                            std::uint32_t nv12_stride,
                            bool swap_uv,
                            std::vector<std::uint8_t>* nv12_out) {
    if (src == nullptr || nv12_out == nullptr || width == 0 || height == 0) {
        return false;
    }
    if ((width & 1U) != 0U || (height & 1U) != 0U) {
        return false;
    }
    if (!isPacked422(src_fourcc)) {
        return false;
    }

    const std::uint32_t in_stride = src_stride >= width * 2 ? src_stride : width * 2;
    const std::uint32_t out_stride = nv12_stride >= width ? nv12_stride : width;
    const std::size_t out_size = static_cast<std::size_t>(out_stride) * height * 3 / 2;
    nv12_out->assign(out_size, 0);

    std::uint8_t* dst_y = nv12_out->data();
    std::uint8_t* dst_uv = dst_y + static_cast<std::size_t>(out_stride) * height;

    for (std::uint32_t y = 0; y < height; ++y) {
        const std::uint8_t* src_row = src + static_cast<std::size_t>(y) * in_stride;
        std::uint8_t* y_row = dst_y + static_cast<std::size_t>(y) * out_stride;
        std::uint8_t* uv_row = dst_uv + static_cast<std::size_t>(y / 2) * out_stride;

        for (std::uint32_t x = 0; x < width; x += 2) {
            std::uint8_t y0 = 0;
            std::uint8_t y1 = 0;
            std::uint8_t u = 0;
            std::uint8_t v = 0;

            switch (src_fourcc) {
                case V4L2_PIX_FMT_YUYV:
                case V4L2_PIX_FMT_YVYU:
                case V4L2_PIX_FMT_UYVY:
                case V4L2_PIX_FMT_VYUY:
                    decode422Pair(src_fourcc, src_row + x * 2, &y0, &y1, &u, &v);
                    break;
                default:
                    return false;
            }

            if (swap_uv) {
                std::swap(u, v);
            }

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
    const int run_seconds = argc > 4 ? std::atoi(argv[4]) : 0;
    const std::string model_path = argc > 5 ? argv[5] : "models/yolov8n.rknn";
    const std::uint32_t model_w = argc > 6 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[6]))) : 640;
    const std::uint32_t model_h = argc > 7 ? static_cast<std::uint32_t>(std::max(1, std::atoi(argv[7]))) : 640;
    const std::string labels_path = argc > 8
        ? argv[8]
        : "third_party/rknn_model_zoo/examples/yolov8/model/coco_80_labels_list.txt";
    const std::string rtsp_url = argc > 9 ? argv[9] : "rtsp://127.0.0.1:8554/live/camera";
    const int fps = argc > 10 ? std::max(1, std::atoi(argv[10])) : 30;
    const std::string dump_h264_path = argc > 11 ? argv[11] : "";
    const int infer_every_n_frames = argc > 12 ? std::max(1, std::atoi(argv[12])) : 5;
    const bool swap_uv = envEnabled("RK3588_YUV_SWAP_UV");
    const std::uint32_t forced_422_fourcc = forced422FourccFromEnv();

    rk3588::core::BoundedQueue<rk3588::core::FramePacket> queue(1);
    rk3588::modules::CameraCapture camera;
    rk3588::modules::MPPEncoder encoder;
    rk3588::modules::RGAProcessor rga;
    rk3588::modules::RKNNRunner rknn;
    rk3588::modules::ZlmRtspPublisher streamer;

    if (!camera.init(device, width, height, 4) || !camera.start(&queue)) {
        std::cerr << "camera init/start failed\n";
        return 1;
    }

    bool initialized = false;
    std::uint64_t input_frames = 0;
    std::uint64_t output_packets = 0;
    std::uint64_t encode_frames = 0;
    std::vector<std::uint8_t> rgb;
    std::vector<std::uint8_t> nv12_scratch;
    std::vector<rk3588::modules::YoloDetection> last_detections;
    std::uint32_t actual_422_fourcc = 0;
    FILE* dump_file = nullptr;

    if (!dump_h264_path.empty()) {
        dump_file = std::fopen(dump_h264_path.c_str(), "wb");
        if (dump_file == nullptr) {
            std::cerr << "failed to open dump file: " << dump_h264_path << '\n';
        } else {
            std::cout << "dumping h264 stream to " << dump_h264_path << '\n';
        }
    }

    const auto start = std::chrono::steady_clock::now();
    while (true) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start).count();
        if (run_seconds > 0 && elapsed >= run_seconds) {
            break;
        }

        rk3588::core::FramePacket frame;
        if (!queue.pop_for(frame, std::chrono::milliseconds(200))) {
            continue;
        }

        if (!initialized) {
            const std::uint32_t gop = static_cast<std::uint32_t>(std::max(10, fps / 2));
            if (!encoder.init() ||
                !encoder.configure(frame.width,
                                  frame.height,
                                  static_cast<std::uint32_t>(fps),
                                  2 * 1024 * 1024,
                                  gop)) {
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

            std::cout << "camera format fourcc=0x" << std::hex << frame.pixel_format << std::dec
                      << " infer_every_n_frames=" << infer_every_n_frames
                      << " swap_uv=" << (swap_uv ? "1" : "0")
                      << " gop=" << gop;
            if (forced_422_fourcc != 0) {
                std::cout << " force422=" << fourccName(forced_422_fourcc);
            }
            std::cout << '\n';

            if (!streamer.start(rtsp_url, frame.width, frame.height, static_cast<std::uint32_t>(fps))) {
                std::cerr << "zlm rtsp publisher start failed, url=" << rtsp_url << '\n';
                camera.requeueBuffer(frame.buffer_index);
                break;
            }
            std::cout << "streaming h264 to " << streamer.publishUrl() << " fps=" << fps << '\n';

            std::vector<std::uint8_t> codec_header;
            if (encoder.getCodecHeader(&codec_header) && !codec_header.empty()) {
                if (dump_file != nullptr) {
                    (void)std::fwrite(codec_header.data(), 1, codec_header.size(), dump_file);
                    std::fflush(dump_file);
                }
                if (!streamer.pushPacket(codec_header.data(), codec_header.size(), 0, 0)) {
                    std::cerr << "failed to push codec header to rtsp streamer\n";
                    camera.requeueBuffer(frame.buffer_index);
                    break;
                }
            } else {
                std::cerr << "warning: unable to fetch MPP codec header (SPS/PPS)\n";
            }

            rgb.resize(static_cast<std::size_t>(model_w) * model_h * 3);
            initialized = true;
        }

        // Always keep the newest frame to reduce end-to-end latency.
        rk3588::core::FramePacket latest;
        while (queue.pop_for(latest, std::chrono::milliseconds(0))) {
            camera.requeueBuffer(frame.buffer_index);
            frame = std::move(latest);
        }

        const bool do_infer = (encode_frames % static_cast<std::uint64_t>(infer_every_n_frames) == 0) ||
                              last_detections.empty();
        if (do_infer) {
            const bool rga_ok = rga.processDmaFdToRgbResize(frame.dma_fd, frame.pixel_format, rgb.data());
            if (!rga_ok) {
                std::cerr << "rga process failed frame_id=" << frame.frame_id << '\n';
                camera.requeueBuffer(frame.buffer_index);
                continue;
            }
            std::vector<rk3588::modules::YoloDetection> detections;
            if (!rknn.inferRgb(rgb.data(), rgb.size(), static_cast<int>(frame.width), static_cast<int>(frame.height), &detections)) {
                std::cerr << "rknn infer failed frame_id=" << frame.frame_id << '\n';
                camera.requeueBuffer(frame.buffer_index);
                continue;
            }
            last_detections = std::move(detections);
        }

        bool ok = false;
        if (frame.pixel_format == V4L2_PIX_FMT_NV12 && frame.cpu_addr != 0) {
            rk3588::modules::drawDetectionsNv12(
                reinterpret_cast<std::uint8_t*>(frame.cpu_addr),
                static_cast<int>(frame.width),
                static_cast<int>(frame.height),
                static_cast<int>(frame.hor_stride > 0 ? frame.hor_stride : frame.width),
                last_detections,
                3);
            ok = encoder.encodeFrame(frame, false);
        } else if (isPacked422(frame.pixel_format) && frame.cpu_addr != 0) {
            if (actual_422_fourcc == 0) {
                if (forced_422_fourcc != 0) {
                    actual_422_fourcc = forced_422_fourcc;
                } else {
                    actual_422_fourcc = detectPacked422Fourcc(
                        reinterpret_cast<const std::uint8_t*>(frame.cpu_addr),
                        frame.width,
                        frame.height,
                        frame.hor_stride);
                }
                std::cout << "packed422 actual decode format=" << fourccName(actual_422_fourcc)
                          << " (camera says " << fourccName(frame.pixel_format) << ")\n";
            }

            const std::uint32_t nv12_stride = (frame.width + 15U) & ~15U;
            if (!convertPacked422ToNv12(
                    reinterpret_cast<const std::uint8_t*>(frame.cpu_addr),
                    frame.width,
                    frame.height,
                    frame.hor_stride,
                    actual_422_fourcc,
                    nv12_stride,
                    swap_uv,
                    &nv12_scratch)) {
                std::cerr << "packed422->nv12 convert failed frame_id=" << frame.frame_id << '\n';
                camera.requeueBuffer(frame.buffer_index);
                continue;
            }

            rk3588::modules::drawDetectionsNv12(
                nv12_scratch.data(),
                static_cast<int>(frame.width),
                static_cast<int>(frame.height),
                static_cast<int>(nv12_stride),
                last_detections,
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
        ++encode_frames;

        while (true) {
            rk3588::modules::EncodedPacket pkt;
            const int got = encoder.getPacket(&pkt);
            if (got <= 0) {
                break;
            }
            const std::uint64_t dts_ms = pkt.dts_us >= 0 ? static_cast<std::uint64_t>(pkt.dts_us) / 1000U : 0;
            const std::uint64_t pts_ms = pkt.pts_us >= 0 ? static_cast<std::uint64_t>(pkt.pts_us) / 1000U : dts_ms;
            if (!streamer.pushPacket(pkt.data, pkt.len, dts_ms, pts_ms)) {
                std::cerr << "rtsp push packet failed" << '\n';
                encoder.releasePacket(&pkt);
                camera.stop();
                queue.close();
                streamer.stop();
                if (dump_file != nullptr) {
                    std::fclose(dump_file);
                    dump_file = nullptr;
                }
                return 1;
            }
            if (dump_file != nullptr) {
                (void)std::fwrite(pkt.data, 1, pkt.len, dump_file);
                std::fflush(dump_file);
            }
            ++output_packets;
            encoder.releasePacket(&pkt);
        }
    }

    camera.stop();
    queue.close();
    streamer.stop();
    if (dump_file != nullptr) {
        std::fclose(dump_file);
        dump_file = nullptr;
    }

    std::cout << "mpp_encoder_demo done: input_frames=" << input_frames
              << " output_packets=" << output_packets
              << " encode_frames=" << encode_frames
              << " run_seconds=" << run_seconds << '\n';
    return 0;
}
