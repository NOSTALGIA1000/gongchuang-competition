# 飞控源码快照 2026-11-22

当前部署在 Jetson (yahboom, 10.42.0.1) 上的源码快照，三个 ROS 包。

## 目录

```
simple_flight_ws/src/
  simple_flight/         定位链（FAST-LIO / 位姿桥 / 启动监督），未改动
  simple_flight_mission/ 飞行任务包，2026-11-20 之后迭代
  simple_flight_payload/ 舱门/投放控制，未改动
```

## simple_flight_mission 近期改动（相对 11-16 备份）

- 任务区进场末段按剩余距离收缩设定点超前量，实现减速（`route_controller._target_lead`）
- 任务完成后在航点停满 `hold_s`（1 s）再离场（`zone_dwell_after_task`）
- 危险品识别失败时跳过该投放、继续飞人员区并按配置返航（`continue_after_target_not_found`）
- 标注连续保持 3 s 才开舱门，12 s 兜底（`mark_hold_s` / `mark_max_wait_s`）
- 返航段从飞机实际位置起步，不再从航点起步（`_begin_return(start=...)`）
- 航点仍为 5 个，返航 `return_indices: [3,1,0]`

校准文件 `coordinates.json` 未包含在本快照（现场坐标，板端 `calibration/` 目录）。

## 相关路径

- 板端备份：`/home/jetson/deployment_backups/taskzone_stillness_20261120/before.tar.gz`
- 部署清单：本仓库或本地 `flight_control_review_20261115/taskzone_stillness_candidate_20261120/deployed/deployed_manifest.json`
