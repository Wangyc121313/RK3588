# JD 技能补全路线图

本文档针对 DJI 嵌入式/AI 感知岗位 JD 要求，在已有 RK3588 项目基础上，规划最小可行补全方案。每个模块定位为"有实操经验、能面试讲述"即可，不强求完整产品化。

---

## 总览

| JD 要求 | 现状 | 补全目标 | 预计任务量 |
|---------|------|---------|-----------|
| PyTorch 训练经验 | 只做过模型转换+部署，未做过训练 | 用 PyTorch 完整训练一个轻量模型并部署到板端 | 小 |
| 自监督/多模态学习融合 | 仅有传统 rule-based 融合 | 实现轻量 MLP 融合头，输出距离修正残差 | 中 |
| 2D/3D 目标检测与点云感知 | 仅有 2D 检测 + 距离估计 | 离线跑通 PointPillars 或其他轻量 3D 检测 | 中 |
| ROS 开发经验 | 零 | 将 camera capture 模块封装为 ROS2 node | 小 |
| VLM/VLA 端到端自动驾驶 | 零 | 暂不涉及，短期不现实 | — |

---

## 一、PyTorch 训练入门

### 目标

走通"数据准备 → 训练 → 验证 → 导出 ONNX → 转 RKNN → 板端部署"的完整链路。

### 方案

选取 COCO 的 person/car/bicycle 三个子类（约 2 万张图），用 YOLOv8n 做微调训练：

```text
工作流：
  1. COCO 子集抽取（Python 脚本，过滤 annotation）
  2. 用 Ultralytics YOLOv8 训练（python train.py，CPU 可跑，epochs=50）
  3. 导出 ONNX → 转 RKNN（复用现有 RKNN 转换流程）
  4. 部署到板端，对比微调前后 mAP
```

### 产出

- 训练脚本与配置
- 微调前后的精度对比表
- 一条端到端可复现指令

### 前置知识

- PyTorch 基础：Dataset/DataLoader、optimizer、loss
- YOLOv8 训练流程（Ultralytics 封装简单）

### 验收标准

- 在本地或服务器上完成一次完整训练
- 量化后 mAP 损失 < 3%
- 板端推理结果无明显退化

---

## 二、轻量学习融合头

### 目标

在现有传统融合（`DetectionDistanceFusion`）基础上，增加一个可学习的距离修正模块，实现"传统保底 + 学习增强"的混合门控方案。

### 方案

```text
输入特征构造（每个检测框）：
  ├── 几何特征：[center_x, center_y, box_w, box_h, box_area_ratio]
  ├── 置信度：[confidence]
  ├── 传统融合输出：[raw_distance_m, candidate_points, cluster_score]
  └── 角度特征：[center_angle_deg]

模型结构（轻量 MLP）：
  Input(9) → FC(32) → ReLU → FC(16) → ReLU → FC(1)
  输出：距离修正残差 delta_m

训练数据：
  - GT 距离来自 LiDAR 角度窗中位数（已有传统融合作 baseline）
  - 使用现有 JSONL 伪标签作为数据集
  - 损失函数：Huber Loss（比 MSE 对异常值鲁棒）

门控回退策略：
  if cluster_score < 阈值 or candidate_points < 最小点数:
      use_traditional_fusion()
  else:
      distance = traditional_output + learning_correction * confidence_weight
```

### 产出

- 训练数据生成脚本（从现有 JSONL 提取）
- MLP 模型定义 + 训练脚本
- 融合策略对比报告（传统 vs 学习 vs 混合）

### 前置知识

- PyTorch 基础（与模块一共享）
- 特征工程：几何/统计特征提取
- 门控融合范式

### 验收标准

- 在遮挡/低光场景下距离抖动比纯传统方案降低
- 学习融合失效时自动回退，不引入灾难性退化

---

## 三、点云 3D 感知入门

### 目标

理解 3D 目标检测的基本概念，离线跑通一个轻量 3D 检测模型。

### 方案

```text
方案 A（推荐，更轻量）：KITTI + PointPillars
  - KITTI 数据集（公开，~15GB）
  - PointPillars 论文实现（PyTorch）
  - 离线跑推理，产出一组可视化结果

方案 B（与本项目结合更紧）：RGB-D 深度估计
  - 用单目深度估计模型（如 MiDaS/DepthAnything）
  - 将 RGB + 深度图投影为伪点云
  - 与 LiDAR 真值做对比
```

推荐方案 A，因为 PointPillars 是工业界最常用的轻量 3D 检测器，面试常问。

### 产出

- PointPillars 推理运行日志与可视化截图
- 一篇简短的阅读笔记（PointPillars 核心思路、voxelization 流程）
- 对 KITTI 数据格式的理解说明

### 前置知识

- 3D 坐标变换：相机坐标系 ↔ LiDAR 坐标系 ↔ 世界坐标系
- 点云体素化（voxelization）
- 3D anchor 与 3D NMS 概念

### 验收标准

- 能在本地跑通 PointPillars 推理
- 能用文字讲清楚 point cloud → voxel → feature → detection head 的流程

---

## 四、ROS2 基础集成

### 目标

将现有项目的一个模块封装为 ROS2 node，理解 topic publish/subscribe 机制。

### 方案

```text
步骤：
  1. 安装 ROS2 Humble（Ubuntu 22.04 对应版本）
  2. 将 CameraCapture 封装为 ROS2 node：
     - 发布 topic: /camera/image_raw (sensor_msgs/Image)
     - 发布 topic: /camera/info (sensor_msgs/CameraInfo)
  3. 用 ros2 bag 录制一段数据
  4. 用 rviz2 可视化

代码量预估：~100 行 ROS2 胶水代码，不改变现有核心逻辑
```

### 产出

- 一个 ROS2 节点源文件
- ros2 bag 录制文件（用于面试展示）
- rviz2 可视化截图

### 前置知识

- ROS2 基础概念：node、topic、message、qos
- 理解 publisher/subscriber 模型

### 验收标准

- `ros2 run` 启动后能在 topic list 中看到发布的 topic
- rviz2 能正常显示摄像头画面

---

## 建议执行顺序

```
1. ROS2 集成（1 天）        ← 最快出成果，简历立即可写
2. PyTorch 训练（2-3 天）    ← 补最大短板
3. 学习融合头（3-5 天）      ← 本项目核心亮点升级
4. 点云 3D 感知（3-5 天）    ← 扩展知识面
```

前两个模块可以在投递前快速完成，后两个模块根据时间灵活安排。
