# 工创比赛源码交接

这份源码快照用于离线图像处理、模型实验、定位诊断和加工文件导出。

| 路径 | 用途 |
| --- | --- |
| real_capture | 相机采集、标注复核、YOLO 训练与离线评估 |
| print_signs | 标识打印素材处理 |
| print_parts | CAD 零件筛选、STL 与 DXF 导出 |
| simple_flight_ws/src | 从本地 Jetson 备份提取的定位健康检查、预检摘要、录像命名及对应测试 |
| export_recordings.ps1 | Windows 端录像导出 |
| [jetson_backup_20261116](jetson_backup_20261116/README.md) | Jetson 历史源码备份，含坐标校准、四条航线和到点门限 |

脚本保留项目原有路径。运行训练或 CAD 导出前，请按本机目录准备对应的数据、YOLOv5 源码、依赖及 CAD 原件。实飞任务、人员识别联动投放、原始录像、模型权重及历史部署包不在此快照内。

## Jetson 源码备份

2026-11-16 快照包含地面坐标校准、三个历史校准文件、四条固定航线配置，以及航点切换、悬停和人工接管的离线逻辑。105 项原有离线测试通过。目录附 SHA-256 清单。

当前 Jetson 板端版本尚未逐文件核对。实飞任务入口、视觉联动自动投放和 PX4 固件未收录。

[查看 Jetson 备份说明](jetson_backup_20261116/README.md)
