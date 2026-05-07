# RK3588 多模态感知系统技术手册

本文档面向面试与项目交接，重点阐述多线程架构、传感器融合算法与多目标跟踪算法的设计与实现。

---

## 一、多线程架构

### 1.1 线程模型

本项目采用**单事件循环 + 一个后台采集线程**的架构，而非最初项目计划中的四线程模式。实际落地方案更为简洁可靠：

```
┌─────────────────────────────────────────────────────┐
│  主线程 (事件循环)                                    │
│                                                     │
│  BoundedQueue ─→ RGA 前处理 ─→ RKNN 推理             │
│       │                  ↓                          │
│       │           检测结果复用（infer_every_n_frames） │
│       │                  ↓                          │
│       │           LiDAR 时间同步 + 融合 + 跟踪         │
│       │                  ↓                          │
│       │           NV12 Overlay + MPP 编码 + RTSP/WebRTC│
│       │                                             │
└───────┼─────────────────────────────────────────────┘
        │
┌───────┴─────────────────────────────────────────────┐
│  LiDAR 采集线程                                      │
│                                                     │
│  RPLIDAR SDK ─→ LidarReader ─→ LidarAdapter        │
│       ↓                                             │
│  LidarRingBuffer (环形缓冲区, capacity=5)             │
│       ↓                                             │
│  atomic<scan_count> 计数                             │
└─────────────────────────────────────────────────────┘
```

### 1.2 核心容器

#### BoundedQueue（有界队列）

位于 `include/core/bounded_queue.hpp`。Camera 采帧入队，主线程取帧处理。

```cpp
template <typename T>
class BoundedQueue {
    // 核心行为：
    // push(): 队列满时自动 pop_front（丢弃最旧帧），保证延迟有界
    // pop_for(timeout): 带超时的阻塞 pop，避免空转
    // close(): 通知所有等待者退出
};
```

**设计要点**：
- 单生产者（Camera 线程）单消费者（主线程），`std::mutex` + `std::condition_variable`
- 满时丢旧帧而非阻塞：防止 Camera 线程被推理拖慢
- 超时 pop：主线程每 200ms 检查一次是否有新帧，无帧则继续循环（避免死等）

#### LidarRingBuffer（雷达环形缓冲区）

位于 `include/core/lidar_ring_buffer.hpp`。保存最近 5 圈完整点云数据。

```cpp
class LidarRingBuffer {
    static constexpr size_t kCapacity = 5;

    // write(): LiDAR 线程写入，lock_guard 保护
    // readClosestWithInfo(): 主线程读取，按时间戳差值最小匹配
    //   同时估算雷达扫描周期（用于后续年龄补偿）
};
```

**设计要点**：
- 写指针 `write_index_` 循环递增，新数据覆盖最旧槽位
- `valid_[i]` 标记槽位是否曾被写入（防止读到未初始化数据）
- 时间匹配：遍历 5 个槽位，选择 `|cloud_ts - frame_ts|` 最小的
- 额外输出 `estimated_scan_period_ms`：通过相邻点云帧时间戳差值的均值估算，用于自适应年龄阈值

### 1.3 双分支数据流

每帧数据在 Camera 采集后分为两条路径：

```text
Camera (V4L2, YUYV 640x480, DMA fd)
  │
  ├── 推理分支（仅 infer_every_n_frames 帧执行）
  │   RGA: DMA fd → RGB 640x640（零拷贝）
  │   RKNN: RGB → YOLOv8n INT8 → bbox/class/conf
  │   LiDAR: 最近点云帧 → Fusion → 距离赋值
  │   Tracker: 关联 + 平滑 → track_id/velocity/TTC
  │
  └── 视频分支（每帧执行）
      422→NV12 格式转换（RGA 优先，CPU 回退）
      NV12 Overlay: 绘制检测框 + 可选 HUD
      MPP: H.264 硬编码
      ZLMediaKit: RTSP/WebRTC 推流
```

### 1.4 DMA 零拷贝路径

```
Camera DMA fd ──→ RGA importbuffer_fd ──→ RGA 硬件处理 ──→ RKNN set_input_fd
     ↑                                                           │
     └───────────── Camera requeueBuffer(index) ─────────────────┘
```

- `FramePacket` 只传递 `buffer_index` + `dma_fd` + 元信息，不传递像素数据
- RGA 和 RKNN 直接通过 DMA fd 访问内存，无 CPU `memcpy`
- 推理完成后归还 `buffer_index` 给 Camera 线程，释放 V4L2 buffer 用于下一帧

### 1.5 线程安全退出

```cpp
// LiDAR 线程：atomic<bool> 标志位
std::atomic<bool> lidar_running{true};
lidar_thread = std::thread([&] {
    while (lidar_running.load()) { /* poll & write */ }
});

// 退出时：
lidar_running = false;
lidar_thread.join();

// 主线程：BoundedQueue::close() 唤醒所有等待者
queue.close();
```

---

## 二、传感器融合算法

### 2.1 坐标系映射

#### 物理安装

```
           LiDAR (0°)                摄像头光轴
              ↑                          ↑
              │        基线 ~8cm          │
    ──────────┼──────────┬───────────────┼────────→ 前方 (Z)
              │          │               │
    [LiDAR]───┘          └──[Camera]─────┘

    LiDAR 0° 与摄像头光轴偏移量: lidar_offset_deg ≈ 11.7°
    （通过 rplidar_angle_calib 标定得到）
```

#### 核心转换（SensorFusion 类）

```cpp
// 1. 像素坐标 → 摄像头角度（基于水平 FOV ≈ 55° 的近似）
float pixelToCameraAngle(x_pixel) {
    // 归一化到 [-0.5, 0.5]，乘以 FOV
    return ((x_pixel / image_width) - 0.5) * camera_fov_deg;
}

// 2. 摄像头角度 → LiDAR 角度（加固定偏移）
float cameraAngleToLidar(camera_angle) {
    return wrap360(camera_angle + lidar_angle_offset);
}

// 3. LiDAR 角度 → 摄像头角度（反向）
float lidarAngleToCamera(lidar_angle) {
    return wrap360(lidar_angle - lidar_angle_offset);
}
```

**设计意图**：当前阶段采用平面 2D 近似模型，不使用完整内参矩阵。水平 FOV 取厂家标称值 55°，角度到像素的映射为线性近似。此方案实现成本低，精度对标定误差不敏感。

### 2.2 鲁棒聚类距离估计（DetectionDistanceFusion）

这是本项目中最核心的融合算法，位于 `include/fusion/detection_distance_fusion.hpp`。

#### 整体流程

```text
每帧每检测框：
  1. 前置过滤（Sanity Check）
     ├── confidence < 阈值? → 跳过
     ├── 检测框过小/过大? → 跳过
     └── 面积占比异常? → 跳过

  2. 候选点收集
     检测框中心 → 换算为 LiDAR 角度 → 构建扇形搜索区
     搜索区 = [focus_left - expand, focus_right + expand]
     在雷达点云中筛选角度在搜索区内的点

  3. 鲁棒聚类（仅 robust 模式）
     候选点按距离排序 → 贪心聚类（按距离差 gap 切分）
     每个 cluster 打分：
       score = weight_sum
             + 0.35 * count              // 点数奖励
             - 1.2 * 角度偏差            // 偏离惩罚
             - 0.9 * 距离散布            // 散布惩罚
     选择最高分 cluster，取中位数距离

  4. 回退策略（Fallback Chain）
     候选点不足 min_candidate_points →
       ① 稀疏回退：检查小目标特判（Sparse Mode）
       ② 角度窗中位数回退：扩大搜索窗取 median
       ③ 返回 -1（无有效距离）

  5. 时序平滑
     按 class + center_x 关联前一帧 track
     EMA 平滑：distance = prev * (1-α) + raw * α
     异常跳跃检测：jump_ratio > 0.45 → α *= 0.45（保守化）

  6. 合理性检查（passesDistanceSanity）
     距离 < 0.8m 但检测框高 < 70px → 拒绝
     距离 < 1.5m 但检测框高 < 42px → 拒绝
     宽高比 > 8:1 且置信度低 → 拒绝
```

#### 关键参数

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `window_half_deg` | 2.5° | 角度窗半宽 |
| `cluster_gap_m` | 0.28m | 聚类切分距离阈值 |
| `min_candidate_points` | 4 | 最少候选点数 |
| `min_cluster_points` | 3 | 最少聚类点数 |
| `smoothing_alpha` | 0.34 | EMA 平滑系数 |
| `outlier_jump_ratio` | 0.45 | 异常跳跃判定比 |

---

## 三、多目标跟踪算法

### 3.1 核心思路

位于 `include/fusion/multi_target_tracker.hpp`。基于 **IoU + 中心距离 + 角度 + 距离** 的四维联合关联，配合 ghost 预测机制处理短时丢检。

### 3.2 轨迹状态机

```text
     det_match                        idle > max_idle
  NEW ───→ CONFIRMED (hits ≥ 3)  ─────────→ 删除
              │ ↑                          ↑
              │ │ re-detected              │
              ▼ │                          │
           GHOST (idle ≤ ghost_keep) ──────┘
              │
              │ idle > ghost_keep
              └──→ 变为普通丢失轨迹
```

### 3.3 关联算法

对每一帧的每个检测框（observation），在所有未匹配的活跃轨迹中寻找最优匹配：

```cpp
findBestTrack(obs, used_tracks):
    for each track:
        // 逐层过滤
        if class_id 不匹配 → skip
        if 中心距离 > max_center_delta_px → skip
        if 角度差 > max_angle_delta_deg → skip
        if IoU < min_iou AND 中心距离大 → skip
        if 距离差 > max_distance_delta_m → skip

        // 多维度代价函数
        cost = center_delta / max_center_delta       // 中心位置代价
             + 0.35 * angle_delta / max_angle_delta  // 角度代价
             + iou_weight * (1 - IoU)                // IoU 代价
             + 0.45 * distance_delta / max_distance  // 距离代价

    return argmin(cost)  // 全局最优匹配
```

**设计要点**：
- 不依赖匈牙利算法（项目规模小，贪心匹配已足够），每帧直接选全局最优
- 多维度代价在遮挡场景下比纯 IoU 更鲁棒
- 距离维度仅在两帧都有有效距离时参与计算

### 3.4 状态更新（updateTrack）

匹配成功后，用 EMA（指数滑动平均）更新轨迹状态：

```cpp
// 位置：直接更新为观测值（检测框中心）
track.center_x = obs.center_x;
track.center_y = obs.center_y;

// 角度：α=0.28 平滑
track.angle = prev_angle + 0.28 * (obs_angle - prev_angle);

// 距离：α=0.30 平滑
track.distance = prev_dist + 0.30 * (obs_dist - prev_dist);

// 检测框尺寸：EMA 平滑
track.box_w = 0.65 * prev_w + 0.35 * obs_w;

// 速度估计（有 dt 时）
raw_vx = (cur_center_x - prev_center_x) / dt;
track.vx += 0.28 * (raw_vx - track.vx);  // 中心速度
track.vr += 0.35 * (raw_vr - track.vr);  // 径向速度
```

### 3.5 Ghost 预测机制

当某个轨迹在当前帧未匹配到观测时，进入 ghost 状态：

```cpp
advanceGhostTrack(track, dt):
    if idle_frames < ghost_keep_frames:
        // 按最后速度外推位置
        track.center_x += vx * dt;
        track.center_y += vy * dt;
        track.distance  += vr * dt;

        // 速度衰减
        vx *= ghost_velocity_decay;  // 0.78
        vy *= ghost_velocity_decay;
        vr *= ghost_velocity_decay;

    idle_frames++;
    track.is_ghost = (idle_frames <= ghost_keep_frames);
```

**设计意图**：短时遮挡（1-4 帧）期间用速度外推维持轨迹，避免 ID 切换。超过 `ghost_keep_frames` 后停止外推但轨迹仍保留（直到 `max_idle_frames` 后删除），为可能的"重新出现"留出窗口。

### 3.6 碰撞时间估计（TTC）

```cpp
if (distance > 0 && closing_speed >= min_closing_speed (0.05 m/s)):
    TTC = distance / closing_speed
else:
    TTC = -1  // 无效
```

其中 `closing_speed = max(0, -radial_velocity)`，即目标靠近时的正向速度。

### 3.7 关键参数一览

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `min_iou_for_match` | 0.05 | 匹配最小 IoU 阈值 |
| `iou_cost_weight` | 0.40 | IoU 在代价函数中的权重 |
| `max_center_delta_px` | 96~110 | 中心距离过滤上限 |
| `ghost_keep_frames` | 4 | ghost 外推最大帧数 |
| `max_idle_frames` | 12 | 轨迹最大空闲帧数 |
| `ghost_velocity_decay` | 0.78 | ghost 速度衰减系数 |
| `min_confirmed_hits` | 3 | 确认轨迹所需命中数 |

---

## 四、性能特征

### 4.1 Pipeline 阶段耗时（典型值，640x480 输入）

| 阶段 | 耗时 | 备注 |
|------|------|------|
| Camera 采集 | < 1ms | V4L2 DMA buffer 入队 |
| RGA 前处理 (YUYV→RGB) | ~1-2ms | RGA 硬件加速 |
| RKNN 推理 (YOLOv8n) | ~25-35ms | NPU，每 5 帧跑一次 |
| 融合 (Fusion) | < 1ms | 纯 CPU 计算 |
| 跟踪 (Tracker) | < 0.5ms | 纯 CPU 计算 |
| MPP 编码 (H.264) | ~3-5ms | 硬件编码 |
| RTSP/WebRTC 推送 | ~1-3ms | ZLMediaKit C API |

### 4.2 设计权衡

| 决策 | 原因 |
|------|------|
| 单事件循环 vs 多线程 | 简化同步逻辑，避免多消费者竞争；推理间隙足够处理其他任务 |
| 有界队列丢旧帧 | 保证端到端延迟有上界，防止内存无限增长 |
| infer_every_n_frames=5 | RKNN 推理是瓶颈（~30ms），复用检测结果到中间帧，吞吐提升 5x |
| header-only fusion/tracking | 编译期内联优化，无链接依赖，方便快速迭代 |
| 传统融合 + 聚类 vs 深度学习 | 可解释、可调试、计算量小，适合嵌入式平台 |
