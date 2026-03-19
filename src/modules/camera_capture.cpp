#include "modules/camera_capture.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>

namespace rk3588::modules {

namespace {

std::uint64_t nowMs() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

}  // namespace

CameraCapture::~CameraCapture() {
    stop();
}

bool CameraCapture::init(const std::string& device, std::uint32_t width, std::uint32_t height, std::uint32_t buffer_count) {
    device_ = device;
    width_ = width;
    height_ = height;
    buffer_count_ = buffer_count;

    if (!openDevice()) {
        return false;
    }
    if (!configureDevice()) {
        closeDevice();
        return false;
    }
    if (!allocateBuffers()) {
        closeDevice();
        return false;
    }
    if (!streamOn()) {
        releaseBuffers();
        closeDevice();
        return false;
    }

    return true;
}

bool CameraCapture::start(core::BoundedQueue<core::FramePacket>* frame_queue) {
    if (fd_ < 0 || frame_queue == nullptr) {
        return false;
    }
    if (running_.exchange(true)) {
        return false;
    }

    frame_queue_ = frame_queue;
    capture_thread_ = std::thread(&CameraCapture::captureLoop, this);
    return true;
}

void CameraCapture::stop() {
    if (!running_.exchange(false)) {
        // Even if thread is not running, make sure device resources are released if init() succeeded.
        if (fd_ >= 0) {
            streamOff();
            releaseBuffers();
            closeDevice();
        }
        return;
    }

    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }

    streamOff();
    releaseBuffers();
    closeDevice();
}

bool CameraCapture::openDevice() {
    fd_ = open(device_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "open " << device_ << " failed: " << std::strerror(errno) << '\n';
        return false;
    }
    return true;
}

bool CameraCapture::configureDevice() {
    v4l2_capability capability {};
    if (xioctl(fd_, VIDIOC_QUERYCAP, &capability) < 0) {
        std::cerr << "VIDIOC_QUERYCAP failed: " << std::strerror(errno) << '\n';
        return false;
    }

    if ((capability.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0 ||
        (capability.capabilities & V4L2_CAP_STREAMING) == 0) {
        std::cerr << "device does not support required V4L2 capture/streaming capability\n";
        return false;
    }

    v4l2_format format {};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width_;
    format.fmt.pix.height = height_;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    format.fmt.pix.field = V4L2_FIELD_NONE;

    if (xioctl(fd_, VIDIOC_S_FMT, &format) < 0) {
        std::cerr << "VIDIOC_S_FMT failed: " << std::strerror(errno) << '\n';
        return false;
    }

    if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
        std::cerr << "camera did not accept YUYV format\n";
        return false;
    }

    width_ = format.fmt.pix.width;
    height_ = format.fmt.pix.height;
    return true;
}

bool CameraCapture::allocateBuffers() {
    v4l2_requestbuffers request {};
    request.count = buffer_count_;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd_, VIDIOC_REQBUFS, &request) < 0) {
        std::cerr << "VIDIOC_REQBUFS failed: " << std::strerror(errno) << '\n';
        return false;
    }

    if (request.count < 2) {
        std::cerr << "insufficient V4L2 buffers returned by driver\n";
        return false;
    }

    buffers_.resize(request.count);
    for (std::uint32_t i = 0; i < request.count; ++i) {
        v4l2_buffer buffer {};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;

        if (xioctl(fd_, VIDIOC_QUERYBUF, &buffer) < 0) {
            std::cerr << "VIDIOC_QUERYBUF failed: " << std::strerror(errno) << '\n';
            return false;
        }

        buffers_[i].length = buffer.length;
        buffers_[i].start = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buffer.m.offset);
        if (buffers_[i].start == MAP_FAILED) {
            buffers_[i].start = nullptr;
            std::cerr << "mmap failed: " << std::strerror(errno) << '\n';
            return false;
        }

        if (xioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
            std::cerr << "VIDIOC_QBUF failed: " << std::strerror(errno) << '\n';
            return false;
        }
    }

    return true;
}

bool CameraCapture::streamOn() {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        std::cerr << "VIDIOC_STREAMON failed: " << std::strerror(errno) << '\n';
        return false;
    }
    return true;
}

void CameraCapture::streamOff() {
    if (fd_ < 0) {
        return;
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMOFF, &type) < 0) {
        // stream may already be off when tearing down after an error.
    }
}

void CameraCapture::releaseBuffers() {
    for (auto& buffer : buffers_) {
        if (buffer.start != nullptr) {
            munmap(buffer.start, buffer.length);
            buffer.start = nullptr;
            buffer.length = 0;
        }
    }
    buffers_.clear();
}

void CameraCapture::closeDevice() {
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

void CameraCapture::captureLoop() {
    while (running_.load()) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);

        timeval timeout {};
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        const int ready = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ready <= 0) {
            continue;
        }

        v4l2_buffer buffer {};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
            if (errno != EAGAIN) {
                std::cerr << "VIDIOC_DQBUF failed: " << std::strerror(errno) << '\n';
            }
            continue;
        }

        const auto* yuyv = static_cast<const std::uint8_t*>(buffers_[buffer.index].start);

        core::FramePacket frame;
        frame.frame_id = frame_id_.fetch_add(1);
        frame.timestamp_ms = nowMs();
        frame.pixels = yuyvToRgb(yuyv);
        frame_queue_->push(std::move(frame));

        if (xioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
            std::cerr << "VIDIOC_QBUF failed: " << std::strerror(errno) << '\n';
        }
    }
}

int CameraCapture::xioctl(int fd, unsigned long request, void* arg) {
    int result = 0;
    do {
        result = ioctl(fd, request, arg);
    } while (result == -1 && errno == EINTR);
    return result;
}

std::uint8_t CameraCapture::clampToByte(int value) {
    if (value < 0) {
        return 0;
    }
    if (value > 255) {
        return 255;
    }
    return static_cast<std::uint8_t>(value);
}

std::vector<std::uint8_t> CameraCapture::yuyvToRgb(const std::uint8_t* yuyv) const {
    std::vector<std::uint8_t> rgb(static_cast<std::size_t>(width_) * height_ * 3);
    std::size_t rgb_index = 0;

    for (std::size_t i = 0; i + 3 < static_cast<std::size_t>(width_) * height_ * 2; i += 4) {
        const int y0 = yuyv[i + 0];
        const int u = yuyv[i + 1] - 128;
        const int y1 = yuyv[i + 2];
        const int v = yuyv[i + 3] - 128;

        const auto convert = [&](int y) {
            const int c = y - 16;
            const int d = u;
            const int e = v;
            const int r = (298 * c + 409 * e + 128) >> 8;
            const int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
            const int b = (298 * c + 516 * d + 128) >> 8;

            rgb[rgb_index++] = clampToByte(r);
            rgb[rgb_index++] = clampToByte(g);
            rgb[rgb_index++] = clampToByte(b);
        };

        convert(y0);
        convert(y1);
    }

    return rgb;
}

}  // namespace rk3588::modules
