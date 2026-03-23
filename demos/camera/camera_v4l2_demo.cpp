#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Buffer {
    void* start = nullptr;
    size_t length = 0;
};

int xioctl(int fd, unsigned long request, void* arg) {
    int result = 0;
    do {
        result = ioctl(fd, request, arg);
    } while (result == -1 && errno == EINTR);
    return result;
}

uint8_t clampToByte(int value) {
    if (value < 0) {
        return 0;
    }
    if (value > 255) {
        return 255;
    }
    return static_cast<uint8_t>(value);
}

std::vector<uint8_t> yuyvToRgb(const uint8_t* yuyv, uint32_t width, uint32_t height) {
    std::vector<uint8_t> rgb(width * height * 3);
    size_t rgb_index = 0;

    for (size_t index = 0; index + 3 < static_cast<size_t>(width) * height * 2; index += 4) {
        const int y0 = yuyv[index + 0];
        const int u = yuyv[index + 1] - 128;
        const int y1 = yuyv[index + 2];
        const int v = yuyv[index + 3] - 128;

        const auto convert_pixel = [&](int y) {
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

        convert_pixel(y0);
        convert_pixel(y1);
    }

    return rgb;
}

void savePpm(const std::string& path, const std::vector<uint8_t>& rgb, uint32_t width, uint32_t height) {
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("failed to open output file: " + path);
    }

    output << "P6\n" << width << ' ' << height << "\n255\n";
    output.write(reinterpret_cast<const char*>(rgb.data()), static_cast<std::streamsize>(rgb.size()));
}

std::string fourccToString(uint32_t pixel_format) {
    std::string result(4, ' ');
    result[0] = static_cast<char>(pixel_format & 0xFF);
    result[1] = static_cast<char>((pixel_format >> 8) & 0xFF);
    result[2] = static_cast<char>((pixel_format >> 16) & 0xFF);
    result[3] = static_cast<char>((pixel_format >> 24) & 0xFF);
    return result;
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string device = argc > 1 ? argv[1] : "/dev/video0";
    const std::string output_path = argc > 2 ? argv[2] : "frame.ppm";
    constexpr uint32_t width = 640;
    constexpr uint32_t height = 480;
    constexpr uint32_t buffer_count = 4;

    int fd = open(device.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "open " << device << " failed: " << std::strerror(errno) << '\n';
        return 1;
    }

    std::vector<Buffer> buffers;

    try {
        v4l2_capability capability {};
        if (xioctl(fd, VIDIOC_QUERYCAP, &capability) < 0) {
            throw std::runtime_error("VIDIOC_QUERYCAP failed: " + std::string(std::strerror(errno)));
        }

        if ((capability.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0) {
            throw std::runtime_error("device does not support video capture");
        }

        if ((capability.capabilities & V4L2_CAP_STREAMING) == 0) {
            throw std::runtime_error("device does not support streaming I/O");
        }

        v4l2_format format {};
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = width;
        format.fmt.pix.height = height;
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        format.fmt.pix.field = V4L2_FIELD_NONE;

        if (xioctl(fd, VIDIOC_S_FMT, &format) < 0) {
            throw std::runtime_error("VIDIOC_S_FMT failed: " + std::string(std::strerror(errno)));
        }

        if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
            throw std::runtime_error("camera did not accept YUYV, negotiated format is " +
                                     fourccToString(format.fmt.pix.pixelformat));
        }

        v4l2_requestbuffers request {};
        request.count = buffer_count;
        request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        request.memory = V4L2_MEMORY_MMAP;

        if (xioctl(fd, VIDIOC_REQBUFS, &request) < 0) {
            throw std::runtime_error("VIDIOC_REQBUFS failed: " + std::string(std::strerror(errno)));
        }

        if (request.count < 2) {
            throw std::runtime_error("insufficient V4L2 buffers returned by driver");
        }

        buffers.resize(request.count);
        for (uint32_t index = 0; index < request.count; ++index) {
            v4l2_buffer buffer {};
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = index;

            if (xioctl(fd, VIDIOC_QUERYBUF, &buffer) < 0) {
                throw std::runtime_error("VIDIOC_QUERYBUF failed: " + std::string(std::strerror(errno)));
            }

            buffers[index].length = buffer.length;
            buffers[index].start = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buffer.m.offset);
            if (buffers[index].start == MAP_FAILED) {
                buffers[index].start = nullptr;
                throw std::runtime_error("mmap failed: " + std::string(std::strerror(errno)));
            }

            if (xioctl(fd, VIDIOC_QBUF, &buffer) < 0) {
                throw std::runtime_error("VIDIOC_QBUF failed: " + std::string(std::strerror(errno)));
            }
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
            throw std::runtime_error("VIDIOC_STREAMON failed: " + std::string(std::strerror(errno)));
        }

        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);
        timeval timeout {};
        timeout.tv_sec = 2;
        timeout.tv_usec = 0;

        const int ready = select(fd + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ready < 0) {
            throw std::runtime_error("select failed: " + std::string(std::strerror(errno)));
        }
        if (ready == 0) {
            throw std::runtime_error("capture timeout after 2 seconds");
        }

        v4l2_buffer buffer {};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd, VIDIOC_DQBUF, &buffer) < 0) {
            throw std::runtime_error("VIDIOC_DQBUF failed: " + std::string(std::strerror(errno)));
        }

        const auto* yuyv = static_cast<const uint8_t*>(buffers[buffer.index].start);
        auto rgb = yuyvToRgb(yuyv, format.fmt.pix.width, format.fmt.pix.height);
        savePpm(output_path, rgb, format.fmt.pix.width, format.fmt.pix.height);

        std::cout << "captured " << format.fmt.pix.width << "x" << format.fmt.pix.height
                  << " frame from " << device << " and saved to " << output_path << '\n';

        if (xioctl(fd, VIDIOC_QBUF, &buffer) < 0) {
            throw std::runtime_error("VIDIOC_QBUF after capture failed: " + std::string(std::strerror(errno)));
        }

        if (xioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
            throw std::runtime_error("VIDIOC_STREAMOFF failed: " + std::string(std::strerror(errno)));
        }
    } catch (const std::exception& ex) {
        std::cerr << "camera demo error: " << ex.what() << '\n';
        for (const auto& buffer : buffers) {
            if (buffer.start != nullptr) {
                munmap(buffer.start, buffer.length);
            }
        }
        close(fd);
        return 1;
    }

    for (const auto& buffer : buffers) {
        if (buffer.start != nullptr) {
            munmap(buffer.start, buffer.length);
        }
    }
    close(fd);
    return 0;
}