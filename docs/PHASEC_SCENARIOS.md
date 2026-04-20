# PhaseC 标准测试场景

本文档定义 PhaseC 的三类标准场景，用于统一评估多传感器融合与时序跟踪能力。

## 场景 1：静止目标（static_target）

目的：评估距离估计抖动与静态轨迹稳定性。

执行建议：
- 在相机视场中心放置 1-2 个静止目标（建议距离 2m-4m）。
- 采集期间尽量保持设备与环境静止。
- 场景时长建议 60-90 秒。

重点指标：
- 距离抖动（distance_jitter_std_m）
- 跟踪保持率（track_retention_ratio）
- 端到端时延（capture_to_encode_ms）

## 场景 2：接近目标（approaching_target）

目的：评估动态距离变化下的跟踪连续性与时序稳定性。

执行建议：
- 让单个目标沿近似直线缓慢接近设备。
- 保证目标尽量持续在相机视场中。
- 场景时长建议 60-90 秒。

重点指标：
- 跟踪保持率（track_retention_ratio）
- ID 切换代理率（id_switch_proxy_rate）
- 端到端时延（capture_to_encode_ms）

## 场景 3：交叉遮挡（crossing_occlusion）

目的：评估遮挡/交叉情况下轨迹连续性与碎片化程度。

执行建议：
- 至少两个目标交叉通过，制造短时遮挡。
- 保持遮挡持续时间短（约 1-3 秒）并重复多次。
- 场景时长建议 60-120 秒。

重点指标：
- 跟踪保持率（track_retention_ratio）
- ID 切换代理率（id_switch_proxy_rate）
- 轨迹碎片率（track_fragmentation_rate）

## 统一采集约束

- 固定分辨率、模型、融合模式与跟踪参数。
- 同一轮 PhaseC 测试建议使用同一相机设备与安装位姿。
- 每个场景单独输出伪标签、telemetry 与视频文件。
- 每轮输出目录建议结构：

```text
reports/phasec/run_YYYYMMDD_HHMMSS/
  static_target/
  approaching_target/
  crossing_occlusion/
  metrics_table.md
  metrics_table.csv
  showcase/
```
