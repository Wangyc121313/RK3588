# RK3588 项目多传感器融合算法解读

本文档目标是回答四个问题：

1. 现在到底融合了哪些传感器。
2. 融合是在什么阶段发生的。
3. 每一层算法具体做了什么。
4. 这套方法的优点、缺点和调参抓手是什么。

## 1. 先说结论

当前项目的“多传感器融合”本质上是一个三段式晚融合方案：

1. 相机负责 `目标类别 + 2D 检测框`。
2. LiDAR 负责 `目标距离候选`。
3. 跟踪器负责 `跨帧稳定化 + 速度/TTC 估计`。

它不是点云和图像特征一起进一个神经网络的“深度早融合”，而是典型的工程规则融合，特点是：

- 可解释。
- 可调参。
- 对算力友好。
- 适合 RK3588 这种边缘设备实时跑。

核心链路在 [src/pipeline/perception_pipeline.cpp](/home/orangepi/RK3588/src/pipeline/perception_pipeline.cpp#L452)。

## 2. 代码里的真实处理顺序

主流程如下：

```text
相机帧 -> RGA 缩放/格式转换 -> RKNN YOLO 检测
     -> 读取时间最近的 LiDAR 点云
     -> DetectionDistanceFusion 给每个检测框估距离
     -> MultiTargetTracker 做跨帧跟踪与平滑
     -> 输出 telemetry / pseudo label / 叠加视频
```

对应代码位置：

- 相机与 LiDAR 初始化：[src/pipeline/perception_pipeline.cpp](/home/orangepi/RK3588/src/pipeline/perception_pipeline.cpp#L431)
- 构造几何融合器 `SensorFusion`：[src/pipeline/perception_pipeline.cpp](/home/orangepi/RK3588/src/pipeline/perception_pipeline.cpp#L455)
- 构造距离融合器 `DetectionDistanceFusion`：[src/pipeline/perception_pipeline.cpp](/home/orangepi/RK3588/src/pipeline/perception_pipeline.cpp#L460)
- 构造时序跟踪器 `MultiTargetTracker`：[src/pipeline/perception_pipeline.cpp](/home/orangepi/RK3588/src/pipeline/perception_pipeline.cpp#L467)
- 每帧执行距离融合：[src/pipeline/perception_pipeline.cpp](/home/orangepi/RK3588/src/pipeline/perception_pipeline.cpp#L725)
- 每帧执行跟踪平滑：[src/pipeline/perception_pipeline.cpp](/home/orangepi/RK3588/src/pipeline/perception_pipeline.cpp#L733)

## 3. 第一层：几何对齐不是标定矩阵，而是“水平角映射”

几何映射在 [include/fusion/sensor_fusion.hpp](/home/orangepi/RK3588/include/fusion/sensor_fusion.hpp#L11)。

它做的事情很朴素：

1. 用图像宽度和相机水平视场角，把像素横坐标映射成相机视角。
2. 再加上 `lidar_angle_offset_deg`，把相机角度转到 LiDAR 角度坐标系。
3. 在 LiDAR 点云里按角度窗口筛点。

对应关系可以写成：

$$
\theta_{cam} = \left(\frac{x}{W} - 0.5\right) \cdot FOV
$$

$$
\theta_{lidar} = wrap360(\theta_{cam} + offset)
$$

这说明当前融合假设是：

- 相机和 LiDAR 的主要相对误差集中在水平旋转方向。
- 不显式建模高度差、俯仰角、外参矩阵、镜头畸变。
- 更像“2D 框中心角 -> LiDAR 扇区”的工程近似法。

这也是为什么当前方案实现简单、速度快，但上限受限于标定精度。

## 4. 第二层：距离融合的核心不是最近点，而是“扇区筛点 + 聚类 + 中值”

距离融合在 [include/fusion/detection_distance_fusion.hpp](/home/orangepi/RK3588/include/fusion/detection_distance_fusion.hpp#L13)。

### 4.1 输入是什么

对每个检测框，算法拿到：

- 框的位置和大小。
- 检测置信度。
- 一帧 LiDAR 点云。

### 4.2 第一步：先做检测框有效性过滤

融合不是对所有框都算距离，而是先过滤明显不可信的框：

- 置信度太低。
- 框太小。
- 框面积占整帧比例过大。

这部分在 `estimate()` 开头，见 [include/fusion/detection_distance_fusion.hpp](/home/orangepi/RK3588/include/fusion/detection_distance_fusion.hpp#L176)。

这里的意义是：很大的误检框或者极小的远处噪声框，没必要强行绑定 LiDAR 距离。

### 4.3 第二步：从检测框提取“聚焦扇区”

算法不是直接用整个 bbox 对应的角度范围，而是只取中间一段“focus box”：

- 先求 bbox 中心。
- 再取 bbox 宽度的一部分作为 focus 区域。
- 把 focus 区域左右边界映射到 LiDAR 角度。
- 再额外扩一小段角度 `sector_expand_deg`。

这样做的目的，是减少 bbox 边缘背景对测距的污染。

简单理解：

- YOLO 框常常比真实物体更宽。
- 框边缘更容易混进背景、地面、墙面或旁边目标。
- 所以当前实现更相信“框中心附近”的 LiDAR 点。

### 4.4 第三步：在扇区内收集候选点，并给中心附近点更高权重

候选点筛选条件有两个：

1. 距离要在 `min_distance_m ~ max_distance_m` 内。
2. 角度要落在扇区范围内。

然后每个点会根据自己离中心角的偏差得到一个权重：越靠近中心，权重越高。

注意，这个权重不是直接用来做加权平均距离，而是主要用于后面的簇评分。也就是说，当前方法仍然偏向“选最合理簇”，而不是“连续回归”。

### 4.5 第四步：两种模式

配置开关：`RK3588_DISTANCE_FUSION_MODE`

- `legacy`：传统中值法。
- `robust`：当前默认的稳健聚类法。

#### legacy 模式

逻辑很简单：

1. 如果候选点足够多，直接取距离中值。
2. 如果候选点不够，就退回到一个固定角窗口里的中值。

优点：

- 简单。
- 行为稳定。
- 易调试。

缺点：

- 容易被背景点或者多目标混合点污染。

#### robust 模式

这是当前主路径，逻辑更完整：

1. 如果候选点很少，先尝试 `sparse` 稀疏估计。
2. 如果候选点足够多，就按距离排序。
3. 把相邻距离差小于动态阈值的点归为同一簇。
4. 对每个簇计算一个分数。
5. 选择分数最高的簇，用该簇距离中值作为原始测距。

动态簇阈值近似是：

$$
gap = max(cluster\_gap\_m, 0.04 \cdot distance)
$$

这表示目标越远，允许同一簇内的距离波动也略大一点。

簇评分同时考虑：

- 点数多不多。
- 中心角附近权重大不大。
- 簇内角偏差是否小。
- 距离 spread 是否太大。

所以这一步的核心思路是：

“不要信最近点，也不要信所有点平均值，而是选一个既集中、又靠近框中心、又足够成簇的那批点。”

这比“最近点法”更稳，也比“全扇区中值法”更抗背景干扰。

### 4.6 第五步：距离 sanity check

得到距离以后，还会再做一次规则校验，见 [include/fusion/detection_distance_fusion.hpp](/home/orangepi/RK3588/include/fusion/detection_distance_fusion.hpp#L402)。

例如：

- 如果估计距离非常近，但框高度却很小，就判定不合理。
- 候选支持点太少且检测置信度又低，也不接受。
- 框形状异常细长且置信度不高，也拒绝。

这层逻辑本质上是“视觉尺寸先验”在做最终兜底。

一句话总结：

LiDAR 负责给距离候选，视觉框大小负责做常识审查。

## 5. 第三层：时序融合不是 Kalman，而是轻量规则 tracker

跟踪在 [include/fusion/multi_target_tracker.hpp](/home/orangepi/RK3588/include/fusion/multi_target_tracker.hpp#L10)。

它的作用不只是“给 track_id”，更重要的是二次稳定距离，并估算速度和 TTC。

### 5.1 观测如何匹配旧轨迹

每个新观测会和已有轨迹按代价匹配，代价项包括：

- bbox 中心距离。
- 角度差。
- IoU。
- 如果距离存在，再加距离差。

这是一个明显的“视觉几何 + 距离几何”混合匹配，不是单纯靠 IoU。

它比纯视觉 tracker 更适合当前项目，因为同类目标一多，IoU 单独用很容易串轨。

### 5.2 更新后会做哪些滤波

匹配成功后，会对这些量做指数平滑：

- 角度。
- 距离。
- 框中心速度。
- 径向速度。
- 横向速度。

因此最终展示给外层的 `distance_m`，很多时候已经不是原始 LiDAR 估计值，而是经过融合平滑后的 `filtered_distance_m`。

这也是你看 telemetry 时要注意的地方：

- `raw_distance_m` 更接近当帧距离融合器输出。
- `distance_m` 更接近最终业务消费值。

### 5.3 ghost track 在解决什么问题

如果某一帧没匹配到观测，tracker 不会立刻删轨，而是短时间外推：

- 按已有中心速度继续推 bbox 中心。
- 按已有径向/横向速度继续推距离与横向偏移。
- 每帧做速度衰减。

这对以下情况有价值：

- YOLO 短时漏检。
- LiDAR 某一帧点很稀疏。
- 目标边缘出画导致短时观测不完整。

代价是：如果漏检时间太长，ghost 也会漂。

## 6. 当前算法的优点

### 6.1 工程上很实用

- 计算轻。
- 延迟低。
- 在 RK3588 上容易跑满实时链路。

### 6.2 可解释性很强

出现误差时，你可以直接从 telemetry 看出问题在：

- 几何偏移不准。
- 候选点太少。
- 聚类选错簇。
- sanity check 误拒绝。
- tracker 过度平滑。

### 6.3 已经考虑了真实场景中的典型噪声

- 稀疏点云。
- 背景点混入。
- 漏检。
- 距离跳变。

这说明它不是 demo 级的“拍脑袋最近点法”，而是已经有一套较成熟的工程稳健性设计。

## 7. 当前算法的局限

### 7.1 只做了水平角度融合

没有完整外参，也没有 3D 投影模型，所以：

- 不擅长处理相机和 LiDAR 高度差造成的几何偏差。
- 不擅长处理俯仰变化。
- 物体上下结构信息没有被利用。

### 7.2 依赖 bbox 质量

如果 YOLO 框偏大、偏移、截断严重，LiDAR 扇区选择也会跟着偏。

### 7.3 多目标近距离并排时仍可能串扰

特别是：

- 两个同类目标角度很近。
- LiDAR 角分辨率不够。
- 扇区内同时落入多个物体点簇。

这时 robust 聚类虽然比 legacy 好，但也不一定总能选对目标簇。

### 7.4 tracker 仍是启发式模型

它没有显式状态协方差，没有观测噪声建模，也没有全局最优分配，因此在复杂交叉场景下上限有限。

## 8. 现在最值得关注的调参项

### 8.1 距离融合

- `RK3588_DISTANCE_FUSION_MODE`：先比较 `robust` 和 `legacy`。
- `lidar_window_half_deg`：角窗口过大容易吃背景，过小容易没点。
- `lidar_offset_deg`：这是几何对齐最敏感参数之一。
- `lidar_min_dist_m` / `lidar_max_dist_m`：直接决定候选点过滤范围。

### 8.2 跟踪

- `RK3588_TRACKER_MIN_IOU`
- `RK3588_TRACKER_IOU_WEIGHT`
- `RK3588_TRACKER_GHOST_KEEP_FRAMES`
- `RK3588_TRACKER_MAX_IDLE_FRAMES`
- `RK3588_TRACKER_CENTER_VEL_ALPHA`
- `RK3588_TRACKER_GHOST_DECAY`

如果你的问题是“距离抖”，优先看融合层和 distance smoothing。

如果你的问题是“ID 跳”或“目标短时消失”，优先看 tracker。

## 9. 这次顺手修掉的一个融合 bug

在距离融合的检测框 sanity 过滤里，原实现用的是：

$$
box\_area\_ratio = \frac{box\_w \cdot box\_h}{image\_width^2}
$$

这在方形输入下问题不明显，但对当前常见的 `640x480` 原始画面并不正确，因为分母应该是整帧面积：

$$
box\_area\_ratio = \frac{box\_w \cdot box\_h}{image\_width \cdot image\_height}
$$

否则会低估大框面积占比，导致异常大框更不容易被拒绝。这个问题已经修正：

- [include/fusion/sensor_fusion.hpp](/home/orangepi/RK3588/include/fusion/sensor_fusion.hpp#L11)
- [include/fusion/detection_distance_fusion.hpp](/home/orangepi/RK3588/include/fusion/detection_distance_fusion.hpp#L176)
- [src/pipeline/perception_pipeline.cpp](/home/orangepi/RK3588/src/pipeline/perception_pipeline.cpp#L455)

## 10. 你可以怎么继续深入

如果你要继续往下读代码，我建议按这个顺序：

1. 先读 [include/fusion/sensor_fusion.hpp](/home/orangepi/RK3588/include/fusion/sensor_fusion.hpp#L11)，先把角度映射看懂。
2. 再读 [include/fusion/detection_distance_fusion.hpp](/home/orangepi/RK3588/include/fusion/detection_distance_fusion.hpp#L53)，重点看 `estimate()`。
3. 最后读 [include/fusion/multi_target_tracker.hpp](/home/orangepi/RK3588/include/fusion/multi_target_tracker.hpp#L55)，理解为什么最终距离会比 raw 更稳。

如果只记一句话，可以记这个：

“当前项目不是深度学习式多模态融合，而是相机做检测、LiDAR 做测距、tracker 做时序稳定的三段式工程融合。”