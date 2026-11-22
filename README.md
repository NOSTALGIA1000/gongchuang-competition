# 工创比赛源码交接

这份源码快照用于离线图像处理、模型实验、定位诊断和加工文件导出。

| 路径 | 用途 |
| --- | --- |
| real_capture | 相机采集、标注复核、YOLO 训练与离线评估 |
| print_signs | 标识打印素材处理 |
| print_parts | CAD 零件筛选、STL 与 DXF 导出 |
| simple_flight_ws/src | 从本地 Jetson 备份提取的定位健康检查、预检摘要、录像命名及对应测试 |
| export_recordings.ps1 | Windows 端录像导出 |

脚本保留项目原有路径。运行训练或 CAD 导出前，请按本机目录准备对应的数据、YOLOv5 源码、依赖及 CAD 原件。实飞任务、人员识别联动投放、原始录像、模型权重及历史部署包不在此快照内。
