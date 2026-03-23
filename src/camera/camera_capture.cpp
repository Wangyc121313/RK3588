#include "camera/camera_capture.hpp"

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

std::int64_t timevalToUs(const timeval& tv) {
    return static_cast<std::int64_t>(tv.tv_sec) * 1000000LL +
           static_cast<std::int64_t>(tv.tv_usec);
}

bool isPacked422(std::uint32_t fourcc) {
    return fourcc == V4L2_PIX_FMT_YUYV || fourcc == V4L2_PIX_FMT_YVYU ||
           fourcc == V4L2_PIX_FMT_UYVY || fourcc == V4L2_PIX_FMT_VYUY;
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

bool CameraCapture::requeueBuffer(std::uint32_t buffer_index) {
    if (fd_ < 0 || buffer_index >= buffers_.size()) {
        return false;
    }

    if (!buffers_[buffer_index].dequeued) {
        return false;
    }

    v4l2_buffer buffer {};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = buffer_index;

    {
        std::lock_guard<std::mutex> lock(io_mutex_);
        if (xioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
            std::cerr << "VIDIOC_QBUF failed: " << std::strerror(errno) << '\n';
            return false;
        }
    }

    buffers_[buffer_index].dequeued = false;
    return true;
}

void CameraCapture::stop() {
    if (!running_.exchange(false)) {
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

    bool configured = false;
    const std::uint32_t candidates[] = {V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_YUYV};

    v4l2_format format {};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width_;
    format.fmt.pix.height = height_;
    format.fmt.pix.field = V4L2_FIELD_NONE;

    for (const auto pixel_format : candidates) {
        format.fmt.pix.pixelformat = pixel_format;
        if (xioctl(fd_, VIDIOC_S_FMT, &format) == 0 && format.fmt.pix.pixelformat == pixel_format) {
            configured = true;
            break;
        }
    }

    if (!configured) {
        std::cerr << "camera did not accept NV12/YUYV formats\n";
        return false;
    }

    width_ = format.fmt.pix.width;
    height_ = format.fmt.pix.height;
    hor_stride_ = (format.fmt.pix.bytesperline > 0)
        ? format.fmt.pix.bytesperline
        : width_;
    ver_stride_ = height_;
    pixel_format_ = format.fmt.pix.pixelformat;

    v4l2_streamparm streamparm {};
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    streamparm.parm.capture.timeperframe.numerator = 1;
    streamparm.parm.capture.timeperframe.denominator = 30;
    if (xioctl(fd_, VIDIOC_S_PARM, &streamparm) == 0) {
        const auto& tpf = streamparm.parm.capture.timeperframe;
        if (tpf.numerator > 0 && tpf.denominator > 0) {
            const double fps = static_cast<double>(tpf.denominator) / static_cast<double>(tpf.numerator);
            std::cout << "camera configured: " << width_ << "x" << height_
                      << " fourcc=0x" << std::hex << pixel_format_ << std::dec
                      << " fps=" << fps << '\n';
        }
    }

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

        v4l2_exportbuffer expbuf {};
        expbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        expbuf.index = i;
        expbuf.plane = 0;
        expbuf.flags = O_CLOEXEC;

        if (xioctl(fd_, VIDIOC_EXPBUF, &expbuf) < 0) {
            std::cerr << "VIDIOC_EXPBUF failed: " << std::strerror(errno) << '\n';
            return false;
        }

        buffers_[i].dma_fd = expbuf.fd;
        buffers_[i].dequeued = false;

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
        if (buffer.dma_fd >= 0) {
            close(buffer.dma_fd);
            buffer.dma_fd = -1;
        }
        buffer.dequeued = false;
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
    bool logged_bytesused = false;
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

        {
            std::lock_guard<std::mutex> lock(io_mutex_);
            if (xioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
                if (errno != EAGAIN) {
                    std::cerr << "VIDIOC_DQBUF failed: " << std::strerror(errno) << '\n';
                }
                continue;
            }
        }

        if (buffer.index >= buffers_.size()) {
            std::cerr << "invalid buffer index from VIDIOC_DQBUF: " << buffer.index << '\n';
            continue;
        }

        buffers_[buffer.index].dequeued = true;

        if (!logged_bytesused) {
            const std::uint32_t expected_bytes = (pixel_format_ == V4L2_PIX_FMT_NV12)
                ? (width_ * height_ * 3U / 2U)
                : ((isPacked422(pixel_format_) ? (width_ * height_ * 2U) : 0U));
            std::cout << "camera frame bytesused=" << buffer.bytesused
                      << " expected=" << expected_bytes
                      << " bytesperline=" << hor_stride_
                      << " fourcc=0x" << std::hex << pixel_format_ << std::dec << '\n';
            logged_bytesused = true;
        }

        core::FramePacket frame;
        frame.frame_id = frame_id_.fetch_add(1);
        frame.timestamp_ms = nowMs();
        frame.pts_us = timevalToUs(buffer.timestamp);
        frame.dts_us = frame.pts_us;
        frame.width = width_;
        frame.height = height_;
        frame.hor_stride = hor_stride_;
        frame.ver_stride = ver_stride_;
        frame.pixel_format = pixel_format_;
        frame.buffer_index = buffer.index;
        frame.buffer_size = static_cast<std::uint32_t>(buffers_[buffer.index].length);
        frame.dma_fd = buffers_[buffer.index].dma_fd;
        frame.cpu_addr = reinterpret_cast<std::uintptr_t>(buffers_[buffer.index].start);

        core::FramePacket dropped_frame;
        if (!frame_queue_->pushWithDrop(std::move(frame), &dropped_frame)) {
            requeueBuffer(buffer.index);
            continue;
        }

        if (dropped_frame.buffer_size > 0) {
            (void)requeueBuffer(dropped_frame.buffer_index);
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

}  // namespace rk3588::modules
