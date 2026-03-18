# 基于RK3588的视觉与激光雷达融合感知系统

> **项目定位**：嵌入式Linux应用开发 + 端侧AI部署 + 多传感器融合  
> **目标岗位**：嵌入式软件开发（消费电子、车企）  
> **开发周期**：6周  
> **硬件平台**：Orange Pi 5（RK3588S）

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

本项目以瑞芯微 RK3588S 为核心平台，融合 USB 摄像头与 RPLIDAR A1 激光雷达两路传感器数据，完成目标检测、距离估计与实时视频推流，构建一套完整的嵌入式感知系统。

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
| 开发板 | Orange Pi 5（RK3588S，4GB） | 1 | ¥800 | 主运行平台，含 6 TOPS NPU |
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
摄像头 FOV（水平约 60 度）划分为 N 个区域：

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
    static constexpr float FOV_DEG = 60.0f;
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
| 视觉推理帧率 | ≥ 20 fps | 推理线程计数器，每秒统计一次 |
| NPU 利用率 | ≥ 80% | `cat /sys/kernel/debug/rknpu/load` |
| RGA 前处理耗时 | < 3 ms | 调用前后时间戳差值 |
| RGA vs OpenCV 加速比 | ≥ 4x | 同任务耗时对比 |
| RGA 前处理 CPU 占用 | ~0% | `htop` 监控单核占用 |
| 融合延迟 | < 2 ms | 融合函数入口/出口时间戳差值 |
| RTSP 推流延迟 | < 500 ms | VLC 拉流，与本地显示对比 |
| 端到端延迟 | < 200 ms | 采帧时间戳到结果输出时间戳 |
| 稳定运行时长 | ≥ 2 小时 | 无崩溃，内存占用不持续增长、无 fd 泄漏 |

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
v4l2-ctl --list-formats-ext -d /dev/video0  # 查看支持的格式和分辨率
```
返回：
> /dev/video0 /dev/video1
> USB Camera: USB Camera (usb-fc800000.usb-1):
> /dev/video0
> /dev/video1
> /dev/mediae
```bash
# 采集一帧保存为图片，验证格式
./camera_capture_demo  # 运行采集demo，验证能否成功采集到
```
得到ppm格式的文件，使用`ffmpeg -i frame.ppm frame.jpg`转换为jpg格式，确认图像内容正确。

默认使用video0采集（真正的视频流节点，Motion-JPEG, compressed），经过测试，发现video1为元数据节点（YUYV 4:2:2），不提供图像帧。

执行demo代码可改为：
```bash
./build/camera_capture_demo /dev/video0
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

3.（Ubuntu Host）新建rknn_demo_test文件夹并克隆官方仓库：
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

5.设计多线程Pipeline架构，定义线程职责和通信方式

6.实现线程安全的帧队列和点云缓冲区

## 12.常用Shell命令
```bash
sudo reboot                                 # 重启系统
sudo poweroff                               # 关机
cat /sys/class/thermal/thermal_zone0/temp   # 查看CPU温度（数值除以1000为摄氏度）
free -h                                     # 查看内存使用情况
top                                         # 实时查看系统资源占用情况
df -h /                                     # 查看磁盘使用情况
scp local_file user@remote_host:/path/to/destination  # 从本地复制文件到远程服务器

chmod +x script.sh                          # 赋予脚本执行权限
```

*文档版本：v2.0 | 最后更新：2026年3月*
