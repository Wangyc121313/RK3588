```text
RK3588/                                      # 项目根目录：RK3588 板端视觉/感知/推流工程
├─ CMakeLists.txt                            # 顶层 CMake 入口：只负责组织子模块、子目标与构建开关
│
├─ cmake/                                    # CMake 模块目录：放依赖探测、内部库定义、应用/工具/demo 目标配置
│  ├─ deps/                                  # 第三方依赖探测：把“找库/找头文件”与“定义目标”解耦
│  │  ├─ mpp.cmake                           # Rockchip MPP 编码依赖探测与导出变量
│  │  ├─ rga.cmake                           # Rockchip RGA 图像处理依赖探测与导出变量
│  │  ├─ rknn.cmake                          # RKNN 推理依赖探测与导出变量
│  │  ├─ rplidar.cmake                       # RPLIDAR SDK 依赖探测与导出变量
│  │  ├─ zlm.cmake                           # ZLMediaKit C API 探测：用于 RTSP / WebRTC 媒体出口
│  │  └─ ffmpeg.cmake                        # FFmpeg 依赖探测：用于备用 RTSP/WebRTC 适配或转封装
│  │
│  ├─ modules/                               # 内部模块库定义：先产出稳定库，再让 app/demo/tool 去链接
│  │  ├─ core.cmake                          # rk_core：公共数据结构、并发容器、通用辅助函数
│  │  ├─ camera.cmake                        # rk_camera：摄像头采集模块库
│  │  ├─ infer.cmake                         # rk_infer：RKNN 推理模块库
│  │  ├─ fusion.cmake                        # rk_fusion：视觉与激光融合模块库
│  │  ├─ lidar.cmake                         # rk_lidar：激光雷达读取与适配模块库
│  │  ├─ video_base.cmake                    # rk_video_base：视频公共类型、时间戳、码流包结构
│  │  ├─ video_overlay.cmake                 # rk_video_overlay：框线/文字/NV12 覆盖绘制模块库
│  │  ├─ video_rga.cmake                     # rk_video_rga：缩放/颜色空间转换模块库
│  │  ├─ video_encode.cmake                  # rk_video_encode：MPP 编码模块库
│  │  ├─ video_publish_rtsp.cmake            # rk_video_publish_rtsp：RTSP 发布模块库
│  │  ├─ video_publish_webrtc.cmake          # rk_video_publish_webrtc：WebRTC 发布模块库
│  │  ├─ video_publish_hub.cmake             # rk_video_publish_hub：多路发布器管理与广播
│  │  └─ pipeline.cmake                      # rk_pipeline：把采集/推理/叠框/编码/发布串起来的流程库
│  │
│  ├─ apps/                                  # 正式应用目标定义：面向长期运行的可执行程序
│  │  ├─ perception_app.cmake                # 感知主程序 target 定义
│  │  └─ stream_gateway_app.cmake            # 视频流网关/媒体出口主程序 target 定义
│  │
│  ├─ demos/                                 # 演示程序目标定义：验证单模块或最小链路
│  │  ├─ camera_demos.cmake                  # 摄像头相关 demo 的 target 定义
│  │  ├─ video_demos.cmake                   # RGA/编码/推流相关 demo 的 target 定义
│  │  ├─ lidar_demos.cmake                   # LiDAR 相关 demo 的 target 定义
│  │  └─ fusion_demos.cmake                  # 融合/多线程链路 demo 的 target 定义
│  │
│  └─ tools/                                 # 工具程序目标定义：校准、诊断、电机控制等
│     ├─ calib_tools.cmake                   # 标定工具 target 定义
│     ├─ motor_tools.cmake                   # 电机控制工具 target 定义
│     └─ diagnostics_tools.cmake             # 诊断工具 target 定义
│
├─ include/                                  # 对外头文件目录：建议后续统一到命名空间式头文件路径
│  └─ rk3588/                                # 项目统一头文件前缀：避免头文件名冲突，利于库化
│     ├─ core/                               # 基础层接口：公共类型与底层通用组件
│     │  ├─ data_types.hpp                   # 帧、点云、检测结果等跨模块共享数据结构
│     │  ├─ bounded_queue.hpp                # 线程安全有界队列：跨线程传递帧与消息
│     │  ├─ lidar_ring_buffer.hpp            # 激光雷达时序缓存：便于和视频帧时间对齐
│     │  ├─ time_utils.hpp                   # 时间戳/时钟辅助函数
│     │  └─ mpp_common_utils.hpp             # MPP 相关公共辅助函数
│     │
│     ├─ camera/                             # 摄像头模块接口
│     │  ├─ camera_capture.hpp               # V4L2 摄像头采集封装
│     │  └─ camera_formats.hpp               # 像素格式/采集参数辅助定义
│     │
│     ├─ infer/                              # 推理模块接口
│     │  ├─ rknn_runner.hpp                  # RKNN 推理封装
│     │  └─ detection_types.hpp              # 检测框、类别、置信度等推理结果类型
│     │
│     ├─ fusion/                             # 融合模块接口
│     │  ├─ sensor_fusion.hpp                # 相机与激光雷达融合逻辑接口
│     │  └─ fusion_config.hpp                # 融合参数、视场角、映射配置
│     │
│     ├─ lidar/                              # LiDAR 模块接口
│     │  ├─ lidar_reader.hpp                 # 激光雷达采集读取接口：屏蔽 SDK 细节
│     │  ├─ lidar_adapter.hpp                # SDK 数据到项目内部点云结构的适配层
│     │  └─ lidar_types.hpp                  # 点云、扫描状态、设备参数类型
│     │
│     ├─ video/                              # 视频处理与发布模块接口
│     │  ├─ frame_overlay.hpp                # RGB 帧框线/文字叠加接口
│     │  ├─ nv12_overlay.hpp                 # NV12 帧叠加接口：编码前直接画框/文字
│     │  ├─ rga_processor.hpp                # RGA 缩放/色彩转换接口
│     │  ├─ mpp_encoder.hpp                  # MPP 编码器接口：输出 H.264/H.265
│     │  ├─ encoded_packet.hpp               # 编码码流包结构：供多个发布器复用
│     │  ├─ stream_publisher.hpp             # 统一发布接口：RTSP / WebRTC 共用抽象
│     │  ├─ publisher_hub.hpp                # 发布器中心：把同一份码流广播给多个输出
│     │  ├─ zlm_rtsp_publisher.hpp           # 基于 ZLMediaKit 的 RTSP 发布器
│     │  ├─ ffmpeg_rtsp_publisher.hpp        # 基于 FFmpeg 的备用 RTSP 发布器
│     │  ├─ webrtc_publisher.hpp             # WebRTC 发布器：用于浏览器端显示
│     │  └─ video_runtime_config.hpp         # 视频输出、码率、帧率、协议选择等配置
│     │
│     └─ pipeline/                           # 上层流程编排接口：应用只做装配，不直接堆业务细节
│        ├─ perception_pipeline.hpp          # 感知全链路流程：采集→预处理→推理→融合→叠加→编码
│        ├─ streaming_pipeline.hpp           # 视频流流程：采集→预处理→推理→叠加→编码→发布
│        ├─ app_config.hpp                   # 应用级配置：模型路径、设备路径、流地址、分辨率等
│        └─ pipeline_factory.hpp             # 按配置构建不同 pipeline 与 publisher 组合
│
├─ src/                                      # 源码实现目录：与 include 中的业务域尽量一一对应
│  ├─ core/                                  # 核心实现：如果有从 header-only 迁出的逻辑放这里
│  │  ├─ time_utils.cpp                      # 时间工具实现
│  │  └─ mpp_common_utils.cpp                # MPP 公共辅助实现
│  │
│  ├─ camera/                                # 摄像头实现
│  │  ├─ camera_capture.cpp                  # V4L2 摄像头采集实现
│  │  └─ camera_formats.cpp                  # 摄像头格式辅助实现
│  │
│  ├─ infer/                                 # 推理实现
│  │  ├─ rknn_runner.cpp                     # RKNN 推理实现
│  │  └─ detection_types.cpp                 # 检测结果辅助实现（可选）
│  │
│  ├─ fusion/                                # 融合实现：建议把原 header-only 逐步迁到这里
│  │  ├─ sensor_fusion.cpp                   # 传感器融合实现
│  │  └─ fusion_config.cpp                   # 融合配置加载/校验实现
│  │
│  ├─ lidar/                                 # LiDAR 实现：把 SDK 细节从 app/demo 中抽出来
│  │  ├─ lidar_reader.cpp                    # 激光雷达读取实现
│  │  ├─ lidar_adapter.cpp                   # 激光点数据适配实现
│  │  └─ lidar_types.cpp                     # LiDAR 类型辅助实现（可选）
│  │
│  ├─ video/                                 # 视频处理与发布实现
│  │  ├─ frame_overlay.cpp                   # RGB 叠框绘制实现
│  │  ├─ nv12_overlay.cpp                    # NV12 叠框绘制实现
│  │  ├─ rga_processor.cpp                   # RGA 处理实现
│  │  ├─ mpp_encoder.cpp                     # MPP 编码实现
│  │  ├─ encoded_packet.cpp                  # 编码码流包辅助实现（可选）
│  │  ├─ publisher_hub.cpp                   # 多发布器广播管理实现
│  │  ├─ zlm_rtsp_publisher.cpp              # ZLMediaKit RTSP 发布实现
│  │  ├─ ffmpeg_rtsp_publisher.cpp           # FFmpeg RTSP 发布实现
│  │  ├─ webrtc_publisher.cpp                # WebRTC 发布实现：浏览器网页展示用
│  │  └─ video_runtime_config.cpp            # 视频运行时配置实现
│  │
│  └─ pipeline/                              # 业务流程装配实现：减薄 apps/*.cpp
│     ├─ perception_pipeline.cpp             # 感知主流程实现
│     ├─ streaming_pipeline.cpp              # 推流主流程实现：支持 rtsp / webrtc / both
│     ├─ app_config.cpp                      # 配置解析与校验实现
│     └─ pipeline_factory.cpp                # 根据参数构造具体模块组合
│
├─ apps/                                     # 正式应用入口目录：只保留轻量 main，负责解析参数和启动 pipeline
│  ├─ perception_main.cpp                    # 感知主程序入口：相机+推理+融合+编码+发布
│  └─ stream_gateway_main.cpp                # 视频出口主程序入口：支持 VLC 的 RTSP 与浏览器的 WebRTC
│
├─ demos/                                    # 演示程序目录：保留验证性质代码，不承担长期主业务
│  ├─ camera/                                # 摄像头/采集链路 demo
│  │  ├─ camera_v4l2_demo.cpp                # 最基础 V4L2 摄像头采集验证
│  │  ├─ camera_thread_demo.cpp              # 采集线程与队列验证
│  │  ├─ camera_rga_demo.cpp                 # 摄像头 + RGA 缩放/色彩转换验证
│  │  └─ camera_rga_rknn_demo.cpp            # 摄像头 + RGA + RKNN 最小视觉链路验证
│  │
│  ├─ video/                                 # 视频相关 demo
│  │  ├─ rga_demo.cpp                        # 纯 RGA 处理验证
│  │  ├─ mpp_encoder_demo.cpp                # 单独验证 MPP 编码能力
│  │  ├─ rtsp_publish_demo.cpp               # 单独验证 RTSP 发布
│  │  └─ webrtc_publish_demo.cpp             # 单独验证 WebRTC 发布
│  │
│  ├─ lidar/                                 # LiDAR demo：验证设备、数据、时序与视场过滤
│  │  ├─ rplidar_demo.cpp                    # 雷达基础采样演示
│  │  ├─ rplidar_timed_demo.cpp              # 带时间戳的采样演示
│  │  ├─ rplidar_guard_demo.cpp              # 守护/持续采样演示
│  │  └─ rplidar_fov_filter_demo.cpp         # 视场角过滤演示
│  │
│  └─ fusion/                                # 融合与多线程链路 demo
│     ├─ pipeline_thread_demo.cpp            # 多线程 pipeline 基础演示
│     └─ fusion_visual_demo.cpp              # 融合结果可视化演示（建议新增）
│
├─ tools/                                    # 工具目录：为生产调试服务，不混入 demo
│  ├─ calib/                                 # 标定工具
│  │  ├─ rplidar_angle_calib.cpp             # LiDAR 角度标定工具
│  │  └─ camera_lidar_extrinsic_calib.cpp    # 相机与雷达外参标定工具（建议新增）
│  │
│  ├─ motor_ctl/                             # 电机与设备控制工具
│  │  └─ rplidar_motor_ctl.cpp               # RPLIDAR 电机启停/控制工具
│  │
│  └─ diagnostics/                           # 诊断工具：建议新增，便于现场排障
│     ├─ camera_caps_probe.cpp               # 摄像头格式/分辨率能力探测工具
│     ├─ rknn_model_info.cpp                 # RKNN 模型信息探测工具
│     ├─ mpp_caps_probe.cpp                  # MPP 编码能力探测工具
│     └─ stream_check.cpp                    # 推流连通性与协议配置检查工具
│
├─ tests/                                    # 测试目录：为长期维护留出口
│  ├─ unit/                                  # 单元测试：针对核心算法和工具函数
│  │  ├─ test_bounded_queue.cpp              # 队列行为测试
│  │  ├─ test_sensor_fusion.cpp              # 融合逻辑测试
│  │  └─ test_publisher_hub.cpp              # 多发布器广播行为测试
│  │
│  ├─ integration/                           # 集成测试：针对完整链路
│  │  ├─ test_camera_to_encode.cpp           # 摄像头→编码链路测试
│  │  ├─ test_rtsp_publish.cpp               # RTSP 推流链路测试
│  │  └─ test_webrtc_publish.cpp             # WebRTC 推流链路测试
│  │
│  └─ data/                                  # 测试数据目录：样例图片、点云、标签、配置等
│
├─ scripts/                                  # 运维/联调脚本
│  ├─ camera_preview.sh                      # 摄像头预览脚本
│  ├─ lidar_capture_once.sh                  # 单次 LiDAR 抓取脚本
│  ├─ lidar_release.sh                       # 释放 LiDAR 占用脚本
│  ├─ perf_stress_suite.sh                   # 性能压力测试脚本
│  ├─ run_rtsp_demo.sh                       # 启动 RTSP 演示脚本（建议新增）
│  └─ run_webrtc_demo.sh                     # 启动 WebRTC 演示脚本（建议新增）
│
├─ models/                                   # 模型目录：RKNN 模型与配套标签
│  ├─ yolov8n.rknn                           # 当前目标检测模型
│  ├─ coco_80_labels.txt                     # 类别标签文件（建议新增并收敛到本仓库）
│  └─ README.md                              # 模型来源、版本、输入尺寸说明
│
├─ third_party/                              # 第三方依赖源码或二进制目录
│  ├─ ZLMediaKit/                            # 媒体服务栈：RTSP / WebRTC / 更多协议能力
│  ├─ librga/                                # Rockchip RGA SDK
│  ├─ rplidar_sdk/                           # RPLIDAR SDK
│  └─ rknn_model_zoo/                        # RKNN 模型样例与参考资源
│
├─ reports/                                  # 报告与输出目录：建议只保留基线样本，日常运行产物尽量忽略
│  ├─ perf/                                  # 性能测试报告
│  │  ├─ baseline/                           # 固定保留的基线性能结果
│  │  └─ latest/                             # 最新一次性能结果链接或输出目录
│  │
│  ├─ screenshots/                           # 截图与展示图片
│  └─ logs/                                  # 导出的运行日志样本
│
├─ docs/                                     # 文档目录：让结构真正可维护
│  ├─ architecture.md                        # 总体架构文档：模块关系、数据流、线程模型、协议出口
│  ├─ build.md                               # 构建文档：依赖、交叉编译、板端部署
│  ├─ runtime.md                             # 运行文档：参数、环境变量、启动方式
│  ├─ streaming.md                           # 推流文档：VLC 的 RTSP 和浏览器的 WebRTC 用法
│  └─ calibration.md                         # 标定文档：相机/LiDAR 校准步骤与参数说明
│
├─ .gitignore                                # Git 忽略规则：忽略 build、运行日志、临时报告、缓存等
└─ README.md                                 # 项目首页说明：快速开始、目录说明、推荐入口、演示方式
```