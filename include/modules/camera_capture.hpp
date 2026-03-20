#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "core/bounded_queue.hpp"
#include "core/data_types.hpp"

namespace rk3588::modules {

class CameraCapture {
public:
    CameraCapture() = default;
    ~CameraCapture();

    bool init(const std::string& device, std::uint32_t width, std::uint32_t height, std::uint32_t buffer_count = 4);
    bool start(core::BoundedQueue<core::FramePacket>* frame_queue);
    bool requeueBuffer(std::uint32_t buffer_index);
    void stop();

private:
    struct Buffer {
        void* start = nullptr;
        std::size_t length = 0;
        int dma_fd = -1;
        bool dequeued = false;
    };

    bool openDevice();
    bool configureDevice();
    bool allocateBuffers();
    bool streamOn();
    void streamOff();
    void releaseBuffers();
    void closeDevice();
    void captureLoop();

    static int xioctl(int fd, unsigned long request, void* arg);

private:
    std::string device_ = "/dev/video0";
    std::uint32_t width_ = 640;
    std::uint32_t height_ = 480;
    std::uint32_t hor_stride_ = 0;
    std::uint32_t ver_stride_ = 0;
    std::uint32_t pixel_format_ = 0;
    std::uint32_t buffer_count_ = 4;

    int fd_ = -1;
    std::vector<Buffer> buffers_;

    std::atomic<bool> running_ {false};
    std::thread capture_thread_;
    core::BoundedQueue<core::FramePacket>* frame_queue_ = nullptr;
    std::atomic<std::uint64_t> frame_id_ {0};
    mutable std::mutex io_mutex_;
};

}  // namespace rk3588::modules
