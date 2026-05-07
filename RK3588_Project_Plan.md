# 基于RK3588的视觉与激光雷达融合感知系统

> **项目定位**：嵌入式Linux应用开发 + 端侧AI部署 + 多传感器融合  
> **目标岗位**：嵌入式软件开发（消费电子、车企）  
> **开发周期**：6周  
> **硬件平台**：Orange Pi 5（RK3588）

---

## 目录

1. [项目概述](#1-项目概述)
2. [硬件物料清单](#2-硬件物料清单)
3. [系统架构设计](#3-系统架构设计)
4. [技术栈总览](#4-技术栈总览)
5. [模块详细设计](#5-模块详细设计)
6. [开发计划（6周）](#6-开发计划6周)
7. [关键技术难点](#7-关键技术难点)
8. [性能指标目标](#8-性能指标目标)
9. [简历描述模板](#9-简历描述模板)
10. [参考资料](#10-参考资料)
11. [项目启动清单（已到货）](#11-项目启动清单已到货)

---

## 1. 项目概述

### 1.1 背景

室内移动机器人、消费级运动相机、低速自动驾驶等场景均需要将视觉信息与测距信息融合，实现对目标的精确感知。视觉（摄像头）负责“识别物体”，激光雷达负责“精确定位”，两者的结合是构建感知系统的核心。

两种方式结合的必要性：
- 互补优势：摄像头拥有丰富的语义信息（颜色、纹理），能判断物体是人还是车；激光雷达提供高精度的深度信息（±3cm），弥补视觉在测距上的不确定性。
- 环境鲁棒性：视觉易受光照（过曝、黑暗）影响，而激光雷达是主动传感器，在夜间依然能稳定探测。

本项目以瑞芯微 RK3588 为核心平台，融合 USB 摄像头与 RPLIDAR A1 激光雷达两路传感器数据，完成目标检测、距离估计与实时视频推流，构建一套完整的嵌入式感知系统。

### 1.2 核心功能

- **视频采集**：V4L2 驱动读取 USB 摄像头，实时获取视频帧
- **目标检测**：RKNN 加速 YOLOv8n INT8 量化模型，NPU 实时推理
- **激光雷达采集**：RPLIDAR A1 获取 360° 平面点云数据
- **传感器数据融合**：将雷达测距数据按区域关联检测目标，输出目标距离
- **RTSP推流**：FFmpeg 将带有检测结果叠加的视频流实时推出，VLC 可直接观看

### 1.3 项目亮点

- 完整的 C++ 多线程 Pipeline 设计，各模块解耦，线程安全
- 使用 RGA 硬件加速图像前处理，减少 CPU 占用，体现平台级性能优化能力
- 结合光学工程背景（激光雷达系统）与嵌入式 AI 部署能力
- 覆盖从传感器驱动调用到网络推流的完整嵌入式软件栈
- 量化性能数据（FPS、延迟、NPU 利用率、RGA 加速比）

---

## 2. 硬件物料清单

| 物料 | 型号 | 数量 | 参考价格 | 说明 |
|------|------|------|----------|------|
| 开发板 | Orange Pi 5（RK3588，4GB） | 1 | ¥800 | 主运行平台，含 6 TOPS NPU |
| 存储卡 | TF 卡 64GB Class10 | 1 | ¥50 | 烧录 Ubuntu 22.04 系统镜像 |
| 散热 | 主动散热风扇套件 | 1 | ¥20 | NPU 高负载时防止降频 |
| 摄像头 | USB 摄像头（UVC 协议） | 1 | ¥40 | V4L2 采集，免驱动 |
| 激光雷达 | RPLIDAR A1M8（思岚科技） | 1 | ¥350 | 含 USB 转串口模块，Linux 免驱 |
| 固定支架 | 万能支架或亚克力板 | 1 | ¥20 | 固定两传感器相对位置，标定后不可移动 |
| **合计** | | | **¥1280** | |

### 2.1 接口说明

- RPLIDAR A1 自带 CP2102 USB 转串口芯片，插入 USB 口后识别为 `/dev/ttyUSB0`，无需额外驱动
- USB 摄像头识别为 `/dev/video0`，UVC 协议 Linux 原生支持
- 两个传感器均通过 USB 连接，无需额外扩展板

---

## 3. 系统架构设计

### 3.1 整体数据流

```
[RPLIDAR A1]
    │ UART/USB
    ▼
[雷达采集线程] ──▶ [点云环形缓冲区（带时间戳）]
                              │
                              │
[USB 摄像头]                  ▼
    │ V4L2              [融合模块]
    ▼                    │  区域匹配
[采集线程] ──▶ [帧队列]  │  距离赋值
                  │      │  时间同步
                  ▼      ▼
           [RGA 硬件加速前处理]
              resize + CSC
                  │
                  ▼
              [推理线程（RKNN YOLOv8）]
                        │
                   [融合结果队列]
                        │
                        ▼
                  [RTSP推流线程]
                   FFmpeg编码
                   RTSP推出
```

### 3.2 线程架构

| 线程 | 优先级 | 职责 | 通信方式 |
|------|--------|------|----------|
| 雷达采集线程 | 高 | UART读取点云，写入环形缓冲区 | 共享内存 + 互斥锁 |
| 摄像头采集线程 | 高 | V4L2 采帧，写入有界帧队列 | 线程安全队列（上限4帧） |
| 推理+融合线程 | 中 | RGA 前处理 + RKNN 推理 + 数据融合，写入结果队列 | 线程安全队列 |
| 推流线程 | 低 | FFmpeg 编码 + RTSP 推流 | 结果队列消费 |

### 3.3 队列策略

- 帧队列**有界**（上限4帧），推理跟不上时**丢弃最旧帧**，避免内存无限增长
- 点云缓冲区使用**环形缓冲区**，保存最近 5 圈数据，每帧带 Unix 时间戳
- 融合时查找时间戳差值最小的点云帧，实现软时间同步
- RGA 硬件加速 resize 与色彩空间转换，零 CPU 拷贝

---

## 4. 技术栈总览

### 4.1 核心技术

| 技术 | 用途 | 对应知识点 |
|------|------|-----------|
| C++17 | 推理引擎、多线程 Pipeline 主体 | STL、智能指针、多线程、RAII |
| V4L2 | USB 摄像头采集 | Linux 设备驱动接口、ioctl、mmap |
| librga (RGA) | 图像 resize / 色彩空间转换硬件加速 | 硬件加速调用、DMA buffer、零拷贝 |
| RKNN-Toolkit2 | YOLOv8 模型转换与 INT8 量化 | 模型量化、PTQ、精度验证 |
| RKNN Runtime | NPU 推理加速 | C++ API、内存管理、异步推理 |
| RPLIDAR SDK | 激光雷达数据采集 | UART 通信、点云解析 |
| FFmpeg | H.264 编码 + RTSP 推流 | AVCodec、AVFormatContext、推流协议 |
| CMake | 工程构建与交叉编译 | CMakeLists、工具链配置 |

### 4.2 开发环境

- **Host 开发机**：Ubuntu 22.04 x86_64（用于模型转换、代码编写）
- **Target 设备**：Orange Pi 5，Ubuntu 22.04 ARM64
- **编译方式**：初期直接在板子上本地编译，后期可配置交叉编译
- **调试工具**：GDB、AddressSanitizer、perf
- **版本管理**：Git + GitHub

---

## 5. 模块详细设计

### 5.1 模块一：模型准备（Week 1，在 x86 Host 完成）

**目标**：将 YOLOv8n 转换为 RK3588 可用的 RKNN INT8 量化模型

**步骤**：

```
PyTorch YOLOv8n → 导出 ONNX → RKNN-Toolkit2 转换 → INT8 PTQ 量化 → 精度验证
```

**关键代码逻辑**：

```python
# 模型转换核心流程（在 x86 Host 上执行）
from rknn.api import RKNN

rknn = RKNN(verbose=True)
rknn.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]],
            target_platform='rk3588')
rknn.load_onnx(model='yolov8n.onnx')
rknn.build(do_quantization=True, dataset='./calibration_dataset.txt')
rknn.export_rknn('./yolov8n.rknn')
```

**验收标准**：模型量化后 mAP 损失 < 2%，模型文件大小 < 10MB

---

### 5.2 模块二：摄像头采集（Week 1-2）

**目标**：V4L2 采集 640x480 NV12 格式视频帧，写入线程安全队列

**关键实现点**：

```cpp
// V4L2 初始化核心流程
class CameraCapture {
public:
    bool init(const std::string& device, int width, int height);
    // 1. open() 打开设备文件
    // 2. VIDIOC_QUERYCAP 验证设备能力
    // 3. VIDIOC_S_FMT 设置格式（NV12，640x480）
    // 4. VIDIOC_REQBUFS 申请 MMAP 缓冲区（4个）
    // 5. VIDIOC_QBUF 入队所有缓冲区
    // 6. VIDIOC_STREAMON 开始采集

    cv::Mat captureFrame();
    // 7. select() 等待帧就绪
    // 8. VIDIOC_DQBUF 取出帧
    // 9. 拷贝数据
    // 10. VIDIOC_QBUF 归还缓冲区
};
```

**验收标准**：稳定采集 30fps，无掉帧，内存占用稳定不增长

---

### 5.3 模块三：RKNN 推理引擎（Week 2-3）

**目标**：C++ 封装 RKNN API，实现 NPU 推理 + 后处理

**关键实现点**：

```cpp
class RKNNDetector {
public:
    bool init(const std::string& model_path);
    std::vector<Detection> detect(const cv::Mat& frame);

private:
    // 前处理由 RGA 模块完成（见 5.5），此处接收已处理的 RGB 缓冲区

    // 后处理：解析 YOLOv8 输出头, decode boxes, NMS
    // YOLOv8 输出格式：[batch, 84, 8400]
    // 84 = 4(box) + 80(class scores)
    std::vector<Detection> postprocess(rknn_output* outputs, float conf_thresh = 0.5f);

    // NMS 实现
    std::vector<int> nms(std::vector<Detection>& dets, float iou_thresh = 0.45f);
};

struct Detection {
    float x, y, w, h;   // 检测框（归一化坐标）
    float confidence;
    int class_id;
    int zone_id;         // 融合后填入：所属区域编号
    float distance;      // 融合后填入：该区域雷达测距（米）
};
```

**验收标准**：单帧推理耗时 < 50ms，NPU 利用率 > 80%，检测结果与原始 PyTorch 模型一致

---

### 5.4 模块四：激光雷达采集（Week 3）

**目标**：使用 RPLIDAR SDK 读取 360° 点云，写入带时间戳的环形缓冲区

**关键实现点**：

```cpp
class LidarCapture {
public:
    bool init(const std::string& port = "/dev/ttyUSB0", int baudrate = 115200);
    void startScan();   // 启动扫描线程

private:
    // 使用 rplidar_sdk 的 ILidarDriver 接口
    // 每圈扫描完成后，回调写入环形缓冲区

    struct PointCloud {
        uint64_t timestamp_ms;               // Unix 时间戳（毫秒）
        std::vector<std::pair<float, float>> points;  // (angle_deg, distance_m)
    };

    // 环形缓冲区，保存最近 5 圈
    std::array<PointCloud, 5> ring_buffer_;
    std::atomic<int> write_idx_{0};
    std::mutex mtx_;
};
```

**验收标准**：稳定读取 10Hz 点云，每圈约 360 个点，数据无丢失

---

### 5.5 模块五：RGA 硬件加速前处理（Week 4，嵌入式核心模块）

**目标**：使用 RK3588 内置 RGA（Raster Graphic Acceleration）硬件单元替代 OpenCV 软件实现的图像 resize 和色彩空间转换，降低 CPU 占用，提升 Pipeline 吞吐

> **面试价值**：这是最能体现「嵌入式软件工程师」能力的模块。使用平台级硬件加速器优化性能，而非仅靠算法，是大疆/影石/车企嵌入式岗位的核心考察点。

#### 5.5.1 RGA 概述

RK3588 内置 3 个 RGA 核心，支持以下硬件加速操作：
- 图像缩放（resize）
- 色彩空间转换（NV12 / NV16 / YUV420 → RGB）
- 图像旋转（90/180/270）
- Alpha 混合

本项目主要利用 **resize + 色彩空间转换**，将 V4L2 采集的 NV12 640x480 帧转换为 RKNN 推理所需的 RGB 640x640 输入。

#### 5.5.2 关键实现

```cpp
#include "im2d.h"       // librga 头文件
#include "RgaUtils.h"

class RGAProcessor {
public:
    bool init(int src_w, int src_h, int dst_w, int dst_h);

    // 使用 RGA 硬件完成 NV12 -> RGB + resize，零 CPU 拷贝
    // 返回 DMA buffer fd，可直接传给 RKNN 输入
    int process(int src_dma_fd);

private:
    rga_buffer_t src_buf_;   // 源图像 handle（NV12 640x480）
    rga_buffer_t dst_buf_;   // 目标图像 handle（RGB 640x640）
    int dst_dma_fd_;         // 输出的 DMA buffer fd

    // 核心调用链
    // 1. importbuffer_fd() -- 将 DMA fd 导入 RGA handle
    // 2. imcvtcolor()      -- NV12 -> RGB 色彩空间转换
    // 3. imresize()        -- 640x480 -> 640x640 缩放
    // 4. improcess()       -- 合并为一次 RGA 调用（减少 HW 调度开销）
};
```

#### 5.5.3 性能对比（核心卖点）

| 前处理方式 | 耗时（预估） | CPU 占用 | 说明 |
|-----------|-------------|----------|------|
| OpenCV `cv::resize` + `cv::cvtColor` | ~8-12ms | 单核 30%+ | 纯 CPU 软件实现 |
| RGA 硬件加速 `imresize` + `imcvtcolor` | ~1-2ms | ~0% | 硬件 DMA 搬运，CPU 无参与 |

> 简历可写："使用 RGA 硬件加速替代 OpenCV 软件前处理，**前处理耗时降低 80%+，CPU 占用从 30% 降至接近 0%**"

**验收标准**：
- RGA 调用成功，输出图像与 OpenCV 输出像素级一致
- 单帧前处理耗时 < 3ms
- 前处理期间 CPU 占用接近 0%（通过 `htop` 验证）
- 使用 DMA buffer 直通 RKNN 输入，无额外内存拷贝

---

### 5.6 模块六：传感器数据融合（Week 4）

**目标**：将雷达测距数据与视觉检测结果按空间区域关联，输出带距离的检测结果

> **设计思路**：本项目的融合重点是**嵌入式多线程数据管理**（时间同步、环形缓冲区、线程安全共享），而非复杂的感知算法。采用简单直观的区域匹配方案，降低算法复杂度，聚焦系统工程能力。

#### 5.6.1 区域划分方案

```
摄像头 FOV（水平约 55 度）划分为 N 个区域：

     Zone 0    Zone 1    Zone 2    Zone 3    Zone 4
   +----------+---------+---------+---------+---------+
   |  -30deg  | -18deg  |  -6deg  |  +6deg  | +18deg  |
   | ~ -18deg | ~ -6deg | ~ +6deg | ~ +18deg| ~ +30deg|
   +----------+---------+---------+---------+---------+

雷达 360 度中对应 FOV 的角度范围，同样划分为 N 个区域。
每个区域取雷达点云中该角度范围内的最近距离值。
```

#### 5.6.2 融合实现

```cpp
class SensorFusion {
public:
    static constexpr int NUM_ZONES = 5;
    static constexpr float FOV_DEG = 55.0f;
    static constexpr float ZONE_WIDTH = FOV_DEG / NUM_ZONES;  // 12 deg/zone

    struct Config {
        float angle_offset;   // 雷达与摄像头 0 度方向的偏差
        int image_width;      // 图像像素宽度
    };

    // 核心融合：为每个检测框分配区域，查询该区域雷达距离
    void fuse(std::vector<Detection>& detections,
              const LidarCapture::PointCloud& cloud) {

        // Step 1: 预处理——按区域聚合雷达最近距离
        std::array<float, NUM_ZONES> zone_dist;
        zone_dist.fill(-1.0f);
        for (auto& [angle, dist] : cloud.points) {
            int zone = angleToZone(angle - config_.angle_offset);
            if (zone >= 0 && zone < NUM_ZONES && dist > 0.15f) {
                if (zone_dist[zone] < 0 || dist < zone_dist[zone])
                    zone_dist[zone] = dist;
            }
        }

        // Step 2: 为每个检测目标分配区域距离
        for (auto& det : detections) {
            float center_x = (det.x + det.w / 2.0f) * config_.image_width;
            int zone = pixelToZone(center_x);
            det.zone_id = zone;
            det.distance = (zone >= 0 && zone < NUM_ZONES) ? zone_dist[zone] : -1.0f;
        }
    }

private:
    Config config_;
    int angleToZone(float angle) { return (int)((angle + FOV_DEG / 2) / ZONE_WIDTH); }
    int pixelToZone(float px) { return (int)(px / (config_.image_width / NUM_ZONES)); }
};
```

**验收标准**：融合延迟 < 2ms，能正确区分不同区域目标并赋予对应距离，时间同步误差 < 100ms

---

### 5.7 模块七：RTSP 推流（Week 5）

**目标**：FFmpeg 将叠加检测结果的视频帧编码为 H.264，通过 RTSP 协议推出

**关键实现点**：

```cpp
class RTSPStreamer {
public:
    bool init(const std::string& rtsp_url = "rtsp://localhost:8554/live",
              int width = 640, int height = 480, int fps = 25);

    // 将带检测框和距离信息的帧推入流
    void pushFrame(const cv::Mat& frame,
                   const std::vector<Detection>& detections);

private:
    // FFmpeg 推流核心
    // AVFormatContext → AVCodecContext(H.264) → AVStream → avformat_write_header
    // 每帧: cv::Mat → AVFrame → avcodec_send_frame → av_write_frame

    // 叠加检测结果到图像
    void drawDetections(cv::Mat& frame, const std::vector<Detection>& dets) {
        for (auto& det : dets) {
            // 画检测框
            cv::rectangle(frame, ...);
            // 标注类别 + 置信度 + 距离
            std::string label = class_names_[det.class_id]
                + " " + std::to_string(det.confidence).substr(0, 4)
                + " " + (det.distance > 0 ? std::to_string(det.distance).substr(0, 4) + "m" : "N/A");
            cv::putText(frame, label, ...);
        }
    }
};
```

**验收标准**：RTSP 流可被 VLC 正常拉取播放，推流延迟 < 500ms，码率稳定在 1-2Mbps

---

## 6. 开发计划（6周）

### Week 1：硬件验收 + 环境搭建

- [ ] 上电与散热确认：安装风扇，开机进入系统后观察 10 分钟温度是否稳定
- [ ] 基础连通：配置局域网 IP，确保可通过 SSH 登录
- [ ] 设备识别检查：
    - [ ] `ls /dev/video*` 能看到摄像头设备（预期 `/dev/video0`）
    - [ ] `ls /dev/ttyUSB*` 能看到雷达串口（预期 `/dev/ttyUSB0`）
- [ ] 系统与工具链安装：`build-essential cmake git pkg-config v4l-utils ffmpeg`
- [ ] 摄像头最小验收：`v4l2-ctl --list-formats-ext` 可正常列出格式
- [ ] 雷达最小验收：运行 SDK demo，确认持续输出角度与距离
- [ ] 建立项目目录结构与仓库：`src/ include/ third_party/ scripts/ docs/`
- [ ] 验收：板卡稳定运行 + 摄像头/雷达均被系统识别 + SSH 开发链路可用

### Week 2：模型转换 + 推理引擎

- [ ] 烧录 Orange Pi 5 Ubuntu 22.04 镜像，SSH 连通
- [ ] 安装 RKNN-Toolkit2（Host），验证 Python 环境
- [ ] 下载 YOLOv8n，导出 ONNX，完成 RKNN INT8 量化转换
- [ ] 板子上跑官方 RKNN YOLOv8 C++ demo，验证推理输出正确
- [ ] 验收：能在板子上对一张图片输出检测框

### Week 3：摄像头采集 + 多线程基础

- [ ] 编写 V4L2 最小 demo：采一帧保存为图片，验证格式
- [ ] 封装 `CameraCapture` 类，实现稳定 30fps 采集
- [ ] 单线程串联：采帧 → 推理（可先Mock）→ 打印结果
- [ ] 验收：实时视频流 + 稳定入队出队，帧率 > 15fps

### Week 4：RKNN 推理引擎 + 激光雷达

- [ ] 设计线程安全队列（模板类，支持超时 pop）
- [ ] 将采集和推理拆分为独立线程，通过队列解耦（接入真实 RKNN）
- [ ] 编写激光雷达最小 demo：打印原始点云数据到终端
- [ ] 封装 `LidarCapture` 类，实现带时间戳环形缓冲区
- [ ] 验收：多线程稳定运行 10 分钟无崩溃，AddressSanitizer 无报错

### Week 5：RGA 硬件加速 + 传感器融合

- [ ] 编写 RGA 最小 demo：NV12→RGB 转换，与 OpenCV 输出对比验证正确性
- [ ] 封装 `RGAProcessor` 类，实现 DMA buffer 直通 RKNN 输入
- [ ] 性能对比测试：记录 OpenCV vs RGA 前处理耗时和 CPU 占用率
- [ ] 实现 `SensorFusion` 类，完成区域划分与距离关联
- [ ] 实现时间戳同步逻辑，选取最近点云帧
- [ ] 验收：RGA 前处理 < 3ms，终端输出 "person 0.92 zone2 1.48m" 格式的融合结果

### Week 6：RTSP 推流 + 性能优化 + 测试收尾

- [ ] 搭建本地 RTSP 服务器（mediamtx，Docker 运行）
- [ ] 实现 FFmpeg H.264 编码 + RTSP 推流基础版
- [ ] 实现 `drawDetections()` 将检测框和距离叠加到视频帧
- [ ] 联调：VLC 拉取 RTSP 流，实时显示带检测结果的视频
- [ ] 使用 perf 分析 CPU 热点，确认 RGA 加速效果
- [ ] 优化 DMA buffer 生命周期管理，确认零拷贝路径无内存泄漏
- [ ] 统计完整性能数据：端到端 FPS、各模块耗时、NPU 利用率、RGA 加速比
- [ ] 验收：VLC 流畅播放，延迟 < 500ms；整体 FPS > 20，端到端延迟 < 200ms

### Week 6（下半周）：测试收尾 + 文档

- [ ] 连续运行 2 小时稳定性测试，记录 CPU/内存/温度
- [ ] 边界测试：无目标场景、多目标场景、低光场景
- [ ] 修复发现的问题，完善错误处理和异常退出逻辑
- [ ] 实现 systemd 服务配置，开机自启动
- [ ] 代码整理：去除调试代码，统一命名规范，添加关键注释
- [ ] 撰写 README（环境搭建、编译方法、运行方法、架构说明）
- [ ] 录制 Demo 视频：实拍运行效果，上传 B 站或 YouTube
- [ ] 整理性能数据表，更新简历描述
- [ ] 代码上传 GitHub，配置 CI 编译检查
- [ ] 验收：2 小时无崩溃，内存占用稳定；GitHub 仓库完整，README 可供他人复现

## 7. 关键技术难点

### 7.1 传感器时间同步

**问题**：摄像头 30fps，雷达 10Hz，两者时钟不同步，融合时可能用到错误的点云帧

**解决方案**：
```
- 所有数据入队时打上 Unix 时间戳（ms 级）
- 融合时遍历点云环形缓冲区，选择时间差最小的一圈
- 若时间差 > 150ms 则认为融合无效，distance 返回 -1
```

### 7.2 RGA 硬件加速集成

**问题**：RGA 使用 DMA buffer 进行零拷贝操作，需正确管理 buffer 的分配、导入和释放，否则会导致内存泄漏或段错误

**解决方案**：
```
- 使用 RAII 封装 DMA buffer fd，在析构时自动 close
- importbuffer_fd 导入的 RGA handle 需在使用后 releasebuffer_handle 释放
- 通过多次运行 + htop 监控确认无 fd 泄漏
- 输出图像与 OpenCV 逐像素对比，保证色彩转换正确性
```

### 7.3 多线程内存安全

**问题**：多线程并发访问共享数据，可能导致数据竞争或死锁

**解决方案**：
```
- 帧队列和结果队列封装成独立的线程安全类
- 点云缓冲区用 atomic<int> 管理写指针，读时加 shared_lock
- 开发期间全程开启 AddressSanitizer + ThreadSanitizer
- 退出时用 atomic<bool> 标志位通知所有线程安全退出
```

---

## 8. 性能指标目标

| 指标 | 目标值 | 测量方法 |
|------|--------|----------|
| 编码平均帧率（avg_fps） | ≥ 20 fps | `mpp_encoder_demo done` 统计 + `perf_stress_suite.sh` |
| 帧率利用率（fps_util_pct） | ≥ 80%（目标 25fps） | `fps_util_pct = avg_fps / target_fps * 100%` |
| 帧交付率（frame_delivery_ratio） | ≥ 0.98 | `output_packets / input_frames` |
| CPU 均值占用（cpu_avg_pct） | < 30% | `/proc/<pid>/stat` 每秒采样 |
| CPU P95（cpu_p95_pct） | < 45% | runtime.csv 分位数统计 |
| CPU 峰值（cpu_peak_pct） | < 70% | runtime.csv 峰值统计 |
| 内存峰值 RSS（rss_peak_mb） | < 300 MB | `/proc/<pid>/stat` RSS 采样 |
| 虚拟内存峰值 VSZ（vsz_peak_mb） | 记录值（趋势稳定） | `/proc/<pid>/stat` VSZ 采样 |
| 线程峰值（threads_peak） | 记录值（无异常增长） | `/proc/<pid>/task` 统计 |
| FD 峰值（fd_peak） | 记录值（无泄漏趋势） | `/proc/<pid>/fd` 统计 |
| 温度均值/峰值（temp_avg/temp_peak） | 峰值 < 75°C | `/sys/class/thermal/thermal_zone0/temp` |
| 磁盘读写吞吐（io_read/io_write） | 记录值（无异常抖动） | `/proc/<pid>/io` 差分统计 |
| 上下文切换速率（ctx_switch_avg_ps） | 记录值（避免异常飙升） | `/proc/<pid>/status` 差分统计 |
| 网络发送速率（net_tx_avg_kbps） | 记录值（与码率匹配） | `/sys/class/net/<iface>/statistics` 差分 |
| NPU 负载（rknpu_avg_pct） | 尽可能高且稳定 | `/sys/kernel/debug/rknpu/load` 采样 |
| RGA 前处理耗时 | < 3 ms | RGA 调用前后时间戳 |
| RGA vs OpenCV 加速比 | ≥ 4x | 同分辨率同输入耗时对比 |
| 融合延迟 | < 2 ms | 融合函数入口/出口时间戳 |
| RTSP 推流延迟 | < 500 ms | VLC 拉流与本地时间戳对比 |
| 端到端延迟 | < 200 ms | 采帧时间戳到推流输出时间戳 |
| 稳定运行时长 | ≥ 2 小时 | 无崩溃、无内存/FD 持续增长 |

### 8.1 统一压测脚本（新增）

```bash
# 快速验证（1轮，6秒）
./scripts/perf_stress_suite.sh 6 1 perf_runs_quick

# 正式压测（3轮，每轮30分钟）
./scripts/perf_stress_suite.sh 1800 3 perf_runs_final

# 指定目标 FPS（用于 fps_util_pct）
TARGET_FPS=25 ./scripts/perf_stress_suite.sh 1800 3 perf_runs_final
```

输出物：

- `round_x/mpp.log`：原始程序日志
- `round_x/runtime.csv`：每秒采样（CPU/RSS/VSZ/线程/FD/温度/IO/网络/NPU）
- `round_x/summary.txt`：单轮关键指标汇总
- `overall_summary.csv`：多轮对比
- `report.md`：可直接粘贴到周报/简历附件的简版报告

---

## 9. 简历描述模板

### 完整版（详细）

> 独立设计并实现基于 RK3588 的视觉-激光雷达融合感知系统，**端到端延迟 < 200ms，推理帧率 25fps**。核心工作包括：(1) 设计四线程生产者-消费者 Pipeline，通过有界队列 + 丢帧策略实现模块解耦，连续运行 2 小时无内存泄漏；(2) 使用 RGA 硬件加速替代 OpenCV 软件前处理，**前处理耗时降低 80%+，CPU 占用从 30% 降至接近 0%**；(3) 完成 YOLOv8n INT8 量化部署（mAP 损失 < 2%），NPU 利用率 ≥ 80%；(4) 集成 RPLIDAR 点云数据实现区域级测距融合，FFmpeg RTSP 实时推流。

### 简洁版（一行）

> 基于 RK3588 + C++ 实现视觉/激光雷达融合感知系统，含 RGA 硬件加速、RKNN INT8 部署、多线程 Pipeline、RTSP 推流，端到端延迟 < 200ms。

---

## 10. 参考资料

### 官方文档

- [RKNN-Toolkit2 GitHub](https://github.com/rockchip-linux/rknn-toolkit2)
- [librga GitHub](https://github.com/airockchip/librga)
- [RPLIDAR SDK GitHub](https://github.com/Slamtec/rplidar_sdk)
- [Orange Pi 5 官方文档](http://www.orangepi.org/html/hardWare/computerAndMicrocontrollers/details/Orange-Pi-5.html)
- [V4L2 官方文档](https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/v4l2.html)
- [FFmpeg 官方文档](https://ffmpeg.org/documentation.html)

### 工具

- mediamtx（本地 RTSP 服务器）：`docker run -p 8554:8554 bluenviron/mediamtx`
- AddressSanitizer：`cmake -DCMAKE_CXX_FLAGS="-fsanitize=address" ..`
- ThreadSanitizer：`cmake -DCMAKE_CXX_FLAGS="-fsanitize=thread" ..`
- perf 性能分析：`perf stat ./your_program`

---

## 11. 项目记录

day1-2：(3.15&3.16)
1.整理PRLIDAR A1、Orange Pi 5的官方datasheet；

2.烧录ubuntu镜像(.img文件，版本推荐：Orangepi5_1.2.2_ubuntu_jammy_server_linux5.10.160.7z)至tf卡并插入开发板，HDMI连接显示器，USB连接键盘；

3.通过网线接入网络（eht0网口），先在windows上得到p.nju.edu.cn的IP地址（powershell：nslookup p.nju.edu.cn），然后在开发板上ping通，确认网络连通性，由于无感知认证并不显示该开发板（无感知设备列表中并不显示，但在当前在线的设备信息中可以看到开发板对应的MAC地址），需要手写自动登录脚本：
```bash
# 手动触发校园网认证
curl -k -X POST 'https://p.nju.edu.cn/api/portal/v1/login' \
  -H 'Content-Type: application/json' \
  -d '{"username":"学号","password":"密码"}'
# 自动登录脚本（login.sh）
cat > ~/login_nju.sh << 'EOF'
#!/bin/bash
sleep 15
curl -k -X POST 'https://p.nju.edu.cn/api/portal/v1/login' \
  -H 'Content-Type: application/json' \
  -d '{"username":"学号","password":"密码"}'
EOF

chmod +x ~/login_nju.sh
crontab -e
# 添加：@reboot /home/orangepi/login_nju.sh
```

4.在vscode中SSH连接，使用remote ssh连接orangepi@192.168.1.xxx；

5.安装常用工具包：
```bash
sudo apt update
sudo apt install -y build-essential cmake git pkg-config v4l-utils python3-pip ffmpeg
```

6.检查NPU驱动，安装工具链：
```bash
# 检查 NPU 驱动
cat /proc/version       # 确认内核版本，RK3588S 通常是 5.10+
# 安装基础工具链
sudo apt update
sudo apt install -y build-essential cmake git pkg-config v4l-utils ffmpeg python3-pip
```

day 3-4：(3.17&3.18)
1.编写V4L2采集demo，使用cmake进行编译成可执行文件，连接硬件并验证摄像头采集功能：
```bash
# 执行cmake得到build文件下的可执行文件
# 插上usb摄像头
ls /dev/video*  # 确认摄像头设备节点（如/dev/video0）
v4l2-ctl --list-formats-ext -d /dev/video0      # 查看支持的格式和分辨率
v4l2-ctl -d /dev/video0 --get-fmt-video         # 查看当前格式，默认格式MJPEG
```
返回：
> /dev/video0 /dev/video1
```bash
# 采集一帧保存为图片，验证格式
./camera_capture_demo  # 运行采集demo，验证能否成功采集到
```
得到ppm格式的文件，使用`ffmpeg -i frame.ppm frame.jpg`转换为jpg格式，确认图像内容正确。

执行demo代码可改为：
```bash
./build/camera_capture_demo /dev/video1
```

2.编写RPLIDAR采集demo，使用cmake编译成可执行文件，连接硬件并验证雷达数据读取功能：
```bash
# 执行cmake得到build文件下的可执行文件
# 插上rplidar激光雷达
ls /dev/ttyUSB*  # 确认激光雷达设备节点（如/dev/ttyUSB0）
```
返回：
> /dev/ttyUSB0
```bash
# 运行采集demo，验证能否成功采集到
./rplidar_demo /dev/ttyUSB0 115200  # 运行雷达采集demo，确认能持续输出角度和距离数据
```
发现出现buffer overflow错误，修改代码增加缓冲区大小后再次运行，确认能稳定输出数据。

发现激光雷达只要上电就开始空转，设计控制逻辑以在需要采集时启动雷达，否则停止转动，实现方式是正常采集后实现一个非常长时间的停转。具体方式见rplidar_timed_demo.cpp。

3.（X86-Ubuntu）新建rknn_demo_test文件夹并克隆官方仓库：
```bash
git clone https://github.com/airockchip/rknn-toolkit2.git
git clone https://github.com/airockchip/rknn_model_zoo.git
```

4.使用python脚本进行onnx->rknn模型（int8）的转换：
```bash
# 前置准备
sudo apt install -y adb gcc-aarch64-linux-gnu g++-aarch64-linux-gnu cmake make
# python虚拟环境，根文件夹下执行
python3 -m venv rknn_env
source rknn_env/bin/activate
python -m pip install -U pip
pip install -r rknn-toolkit2/rknn-toolkit2/packages/x86_64/requirements_cp310-2.3.2.txt
pip install rknn-toolkit2/rknn-toolkit2/packages/x86_64/rknn_toolkit2-2.3.2-cp310-cp310-manylinux_2_17_x86_64.manylinux2014_x86_64.whl
# 根据官方仓库model_zoo中的yolov8教程，进行模型转换，进入yolov8文件夹
chmod +x download_model.sh
./download_model.sh
cd /home/wangyc/桌面/rknn_demo/rknn_model_zoo/examples/yolov8/python
python3 convert.py ../model/yolov8n.onnx rk3588 i8 ../model/yolov8.rknn #注意onnx版本
# 设置交叉编译前缀
export GCC_COMPILER=/usr/bin/aarch64-linux-gnu
# 编译 RK3588 Linux demo
./build-linux.sh -t rk3588 -a aarch64 -b Release
# scp传输文件
scp -r rknn_model_zoo/install/rk3588_linux_aarch64/rknn_yolov8_demo orangepi@180.209.2.141:~/rknn_demo_test/
```

5.在开发板上运行RKNN推理demo，验证模型转换和推理功能：
```bash
cd ~/rknn_demo_test/rknn_yolov8_demo
export LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH
./rknn_yolov8_demo model/yolov8.rknn model/bus.jpg
ls -lh out.png
```
可以看到输出(output.png)，分辨率为640×640

6.设计多线程Pipeline架构，定义线程职责和通信方式：

- 线程1：摄像头采集线程，使用V4L2接口采集视频帧，写入线程安全的帧队列
- 线程2：激光雷达扫描线程，使用RPLIDAR SDK读取点云数据，写入带时间戳的环形缓冲区
- 线程3：从帧队列获取帧，按时间戳从点云缓冲区获取对应的雷达数据，进行区域划分和距离关联，输出带距离的检测结果
- 线程4：RTSP推流线程，接收带检测结果的帧，使用FFmpeg编码并通过RTSP协议推流

7.实现线程安全的帧队列和点云缓冲区，实现两个核心容器类：

- 线程安全有界队列（`bounded_queue.hpp`）：支持多生产者多消费者；buffer 满时pop_front再push；提供超时 pop 功能，避免死锁；使用互斥量进行上锁，条件变量进行通知
- 带时间戳的点云环形缓冲区（`lidar_ring_buffer.hpp`）：支持多生产者多消费者；write写到当前槽位，写指针循环；match与对应时间戳差值最小的一帧；使用 `atomic<int>` 管理写指针，读时加 shared_lock；提供接口获取最近的点云数据

day5-6：(3.19&3.20)
1.实现RGAProcessor类，使用RGA硬件加速前处理，完成NV12到RGB的色彩空间转换和图像缩放：
```bash
# 检查RGA驱动
sudo cat /sys/kernel/debug/rkrag/driver_version
# 克隆官方librga仓库
cd third_party
git clone https://github.com/airockchip/librga.git
cd librga
# 添加rga_demo和rga_processor，接入CMake并编译通过demo测试
cmake --build build -j4
cd /home/orangepi/RK3588 && ./build/rga_demo
```
输出：
> NV12 640x480 -> RGB 640x640, elapsed=6209 us

2.实现CameraCapture线程，使用V4L2接口采集视频帧，写入线程安全的帧队列，保证至少30fps的采集速度：
```bash
./build/camera_thread_demo /dev/video0 5 640 480
```
输出：
> camera_thread_demo done: consumed_frames=147 run_seconds=5 avg_fps=29.4 first frame saved to camera_thread_first_frame.ppm

测试程序显示平均帧率约为29.4fps，满足30fps的采集速度要求。

3.输出DMA buffer fd供RKNN输入，实现src/modules/中的三个核心模块（**具体实现**）和include/modules中（**对外声明**）的三个核心声明：

(1) `camera_capture`：采集线程，采集到的帧通过DMA buffer传递给RGA处理；经过测试发现摄像头采集帧为YUYV格式，因此该函数主要讲YUYV格式转换为RGB格式（通过DMA fd）；队列中传递“buffer 索引 + dma fd + 时间戳 + 尺寸”；消费者处理完后必须回传索引给相机线程再 QBUF，避免覆盖正在处理的帧

(2) `rga_processor`：接收DMA buffer fd（src dma fd），使用RGA进行前处理，输出处理后的DMA buffer fd供RKNN输入(fd -> fd)，实现颜色转换和缩放；输出使用预分配的一组 dst dma fd

(3)`rknn_runner`：直接将RGA输出的dma fd绑定到RKNN输入tensor内存中，不做`memcpy()`到CPU缓冲区;接收RGA输出的DMA buffer fd，结合官方yolov8代码（复制之前的rknn文件至models文件夹，稀疏克隆官方model_zoo中的yolov8代码至本地）进行推理，输出检测结果

线程与队列模型：
- Camera 生产 CameraBufferRef（借用句柄）。
- RGA/Infer 线程消费 CameraBufferRef，产出 InferenceInputRef（RGA 输出池索引）。
- 推理完成后分别归还 RGA 输出池和 Camera buffer 索引。

这样整条链只传轻量元信息，数据始终留在 DMA buffer。
<!-- 4.对比OpenCV软件实现的前处理，验证RGA加速效果，记录前处理耗时和CPU占用率。 -->

day7-8：(3.21&3.22)
1.首先尝试FFmpeg命令行推流，使用Docker起一个临时本地RTSP服务器，通过VLC拉取到视频流，此时发现Docker拉取镜像超时，切换到本机FFmpeg监听本地端口时也失败。

2.此时注意到瑞芯微官方的MPP（Media Process Platform）库，提供了对RK3588上硬件编码器的访问接口，使用MPP库可以直接将处理后的帧编码为H.264格式，并通过RTSP协议推流到本地服务器，同时，使用MPP库可以更高效地利用RK3588的硬件资源（fd直接传递），降低CPU负载，提升推流性能。

3.克隆官方的MPP库，从aarch64执行安装到/usr/.local，
```bash
git clone https://github.com/rockchip-linux/mpp
cd /home/orangepi/RK3588/third_party/mpp-develop/build/linux/aarch64 && sudo cmake --install .
sudo ldconfig && ldconfig -p
```
此时/usr/local/lib下出现了libmpp.so等相关库文件，验证安装成功。

4.重新梳理了一下YOLO+RGA+RTSP的整体流程，发现RKNN跑完输出的是tensor，即推理结果张量，并不是直接能够编码的视频帧，因此以摄像头采集为基准，构成两个分支：
- 视频分支：相机帧 -> 经 RGA 色彩/缩放 -> MPP 编码 -> 推流
- 推理分支：相机帧 -> RGA 转模型输入格式 -> RKNN -> 得到 bbox/class/conf
- 融合点：把 bbox 画回“视频分支的那帧”，再去编码

5.MPP的常见编码输入格式为NV12，与目前摄像头输出的YUYV格式不符。

两者对比如下：
|指标|YUYV|NV12|
|---|---|---|
|内存占用（1080p）|4,147,200 bytes（3.95 MB）|3,110,400 bytes（2.97 MB）少 25%|
|硬件支持|USB 摄像头原生输出|MPP/RGA/RKNN 原生输入|
|格式转换|送 NPU 前必须转换|直接送 MPP 编码，零转换|

注意到YUYV为相机输出的原始格式，同时，YUYV格式到NV12格式的转换只需进行元素的重排，且MPP编码器更高效地处理NV12格式，因此在RGA前处理阶段增加了从YUYV到NV12的转换步骤，确保后续编码和推理的效率。不选择MJPEG格式的原因是MJPEG是压缩格式，解码需额外使用MPP。通过运行：
```bash
v4l2-ctl --list-formats-ext -d /dev/video0 
```
得到该相机只有在640x480分辨率下支持30FPS的YUYV格式输出。

最后的整体视频编解码与推流链路可以表示为：

```text
Camera (YUYV 640x480) -> RGA (YUYV->RGB) -> RKNN
                      -> RGA (YUYV->NV12) -> MPP (H.264 编码) -> RTSP 推流
```

6.尝试了使用MediaMTX进行RTSP推流，发现总是出现404问题；更换方法，参考[RK-MediaProject](https://github.com/JoeChen2me/RK-MediaProject)中的方案，使用内置 ZLMediaKit C API 发布 RTSP，成功实现在windows端用 VLC 拉流显示带检测结果的视频。
```bash
./build/mpp_encoder_demo /dev/video0 640 480 0 models/yolov8n.rknn 640 640 third_party/rknn_model_zoo/examples/yolov8/model/coco_80_labels_list.txt rtsp://0.0.0.0:8554/live/camera 25 '' 5 /dev/ttyUSB0 115200 11.7 55 2.5 0.15 6.0 120
```

day9-10：(3.23&3.24)

1.VLC拉流成功后尝试使用WebRTC，配置对应前端界面，对应启动脚本：
```bash
DEVICE=/dev/video1 START_DEBUG_UI=1 ./scripts/run_webrtc.sh
```
端口为8090，浏览器打开[WebRTC](http://127.0.0.1:8090/?app=live&stream=camera)即可。

2.传感器数据融合算法：

(1)激光雷达基本参数确定：

查阅思岚A1M8 RPLiDAR的官方文档，发现其输出数据分为Distance、Heading、Quality、Start Flag，其中Start Flag用于标识每圈扫描的开始。

查阅官方文档，得到其基本参数为扫描频率5.5Hz，对应单圈时间约为181ms，采样频率8000Hz，单圈有效点数约为1454。

执行rplidar_timed_demo，输出转速与点数数据，得到转速、每圈有效点数：
>scan_hz = 6.79612      
>points_per_scan = 943  

即测得每圈时间约为147ms、单圈有效点数为943。该结果与官方文档中5.5Hz、1454点的参数存在一定差异，可能是由于实际环境因素或设备状态导致的性能波动，但基本符合预期范围。

(2)标定激光雷达光轴方向与摄像头光轴平行：

由于没有用兰丁胶固定底座导致每次上电重新转动时自身会有一个小偏移。固定后通过rplidar_angle_calib程序进行标定：在激光雷达正前方约30cm处的同一高度位置放置一三脚架，固定三脚架位置不变多次测试30cm对应的输出角度：
```bash
./build/rplidar_angle_calib /dev/ttyUSB0 115200 0.3 0.05 8 8
```
输出：
>RPLIDAR connected on /dev/ttyUSB0
>target distance=0.3 m, tolerance=0.05 m, collect=8 s
>
>=== Angle Calibration Result ===
>target_m=0.300 tolerance_m=0.050
>total_valid_points=42025 accepted_points=705 peak_cluster_points=662
>cluster_distance_mean=0.303 min=0.300 max=0.309 m
>calibrated_deg=191.701
>spread_deg=1.300 confidence=0.986
>status=OK
>raw_front_angle_deg=191.701
>fusion_offset_deg=11.701
>
>Suggested usage in fusion config: lidar_offset_deg=11.701
>

开关机测试均发现该角度基本不变，因此可以认为激光雷达的光轴方向与摄像头光轴平行。标定测得的原始前向角为1911191.701度，但当前融合实现使用的是其反向零位修正值，因此运行配置中应使用 wrap360(191.701 - 180) = 11.701 度，并以该角度左右30度的范围作为与摄像头采集区域重叠的部分。

(3)雷达-摄像头坐标系映射改进方案

当前确认的近似模型如下：

- 摄像头水平 FOV 取厂家标称值 55 度
- 激光雷达与摄像头前后方向基本对齐，上下高度已尽量对齐
- 左右方向保留固定基线偏移，取 `b = -0.08 m`
- 当前阶段不引入俯仰角和完整 3D 外参，仅采用地面平面 2D 刚体近似

建议的映射步骤：

①对雷达点 `(r, theta_l)` 先转为雷达平面坐标：

```text
X_l = r * sin(theta_l)
Z_l = r * cos(theta_l)
```

②按照 `b = -0.08 m` 做雷达到相机的横向平移，得到相机平面坐标：

```text
X_c = X_l + b
Z_c = Z_l
```

③再由相机平面坐标反算相机视角：

```text
theta_c = atan2(X_c, Z_c)
```

④若不使用真实内参，则继续采用水平视场角近似把 `theta_c` 映射到像素横坐标：

```text
u ~= (theta_c / 55deg + 0.5) * image_width
```

⑤后续融合时优先使用投影后的 `u` 与检测框中心或检测框横向范围做关联，而不是仅用固定角度窗；距离输出可优先保留 `Z_c` 作为前向距离。

该方案的优点是实现成本低、可直接接入现有角度窗融合框架，同时显式考虑了 8 cm 左右基线带来的近距离视差误差；待该近似方案稳定后，再考虑引入小偏航角和更完整的外参标定。

3.最终的压力测试：

连续运行一小时，输出各项性能指标，确认系统稳定性和性能表现：

```bash
./scripts/perf_stress_suite.sh 3600 1 reports/perf
```
## 12.CMakeLists.txt 分析
```cmake
```

## 12.常用Shell命令
```bash
sudo reboot                                 # 重启系统
sudo poweroff                               # 关机
cat /sys/class/thermal/thermal_zone0/temp   # 查看CPU温度（数值除以1000为摄氏度）
free -h                                     # 查看内存使用情况
htop                                        # 实时查看系统资源占用情况
df -h /                                     # 查看磁盘使用情况
scp local_file user@remote_host:/path/to/destination  
                                            # 从本地复制文件到远程服务器
chmod +x script.sh                          # 赋予脚本执行权限
fuer
ps
```

*文档版本：v2.0 | 最后更新：2026年3月*