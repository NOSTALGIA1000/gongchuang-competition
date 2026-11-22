# 工创比赛 — 智能低空消防无人机

上海市工程实践与创新能力大赛「智能低空消防」赛项。四旋翼无人机按固定航线自主飞行，识别危险品与人员目标并投放物资。

硬件：Jetson Orin + PX4 (FMU v6C) + 凌美 MID-360 雷达 + 机腹相机。定位用 FAST-LIO，视觉用 YOLOv5，ROS2 Humble。

## 目录

| 路径 | 用途 |
| --- | --- |
| [flight_control_20261122](flight_control_20261122/) | **当前部署的飞控源码**（三个 ROS 包，含算法逻辑） |
| [jetson_backup_20261116](jetson_backup_20261116/) | 11-16 历史备份，含坐标校准与四条航线 |
| real_capture | 相机采集、标注复核、YOLO 训练与离线评估 |
| print_signs / print_parts | 标识与 CAD 零件导出 |
| export_recordings.ps1 | Windows 端录像导出 |

---

# 飞控算法逻辑

飞行任务包在 `flight_control_20261122/simple_flight_ws/src/simple_flight_mission/`。核心是三个文件：`route_controller.py`（航点推进）、`visual_task.py`（视觉投放）、`competition_controller.py`（任务编排）。

## 1. 坐标与定位

- 起飞时锁定位姿为原点 `(x0, y0, z0, yaw)`，`yaw` 为机头方向。
- 航点配置为相对坐标 `(forward_m, left_m)`：前方为正、左侧为正。
- 世界坐标转换：

  ```python
  x = x0 + forward*cos(yaw) - left*sin(yaw)
  y = y0 + forward*sin(yaw) + left*cos(yaw)
  ```

- 现场实测坐标存 `calibration/coordinates.json`（10 个点名），通过 `point_names` 按名绑定，非按数值。
- 位置来源：FAST-LIO → 位姿桥 → PX4 EKF 外部视觉位姿（`EKF2_EV_CTRL=11`）→ `/mavros/local_position/pose`。任务层与 PX4 用的是同一份融合估计。

## 2. 航段推进（关键机制）

飞机用 OFFBOARD **纯位置指令**控制，不发速度前馈。设定点始终钉在航线前方一个超前量 `target_lead_m`（默认 0.25 m）：

```python
progress = min(length,
               progress + horizontal_mps*dt,
               max(progress, along + target_lead_m))
```

由于 PX4 位置环是比例控制（`MPC_XY_P = 0.95`），实际巡航速度：

```
v ≈ MPC_XY_P × target_lead_m ≈ 0.95 × 0.25 ≈ 0.24 m/s
```

实测三腿平均 0.18–0.24 m/s，与 `0.95 × 实测位置误差` 逐腿吻合。**配置里的 `horizontal_mps: 0.3` 只是上限，真实速度由超前量决定**——这是本项目调参的核心结论。

## 3. 进场末段减速

只在飞向任务区（task_zone）的最后一段生效。按剩余距离收缩超前量：

```python
remaining = length - along
lead = min(target_lead_m, max(0.04, remaining * taper))
```

速度 ≈ 增益 × 超前量，收缩超前量就是减速。剩余 25 cm 时速度 24 cm/s，15 cm 时 14 cm/s，5 cm 时 4.8 cm/s。避障航点与返航段保持固定超前量。

## 4. 到点与停留

- 到点判据：距航点 ≤15 cm、水平速度 ≤0.1 m/s、航向差 ≤5°，连续 1 s。
- 任务完成后在航点停满 `hold_s`（任务区 1 s）再离场。

## 5. 视觉任务

每个任务区一个 `VisualTask`。危险品区用舱门 1（识别 kind 2），人员区用舱门 2（识别 kind 0），人员区先在通道内扫描搜索。

投放门限（评分要求标注停留 3 s 以上）：

```python
# 新鲜帧 + 正确类别 + 像素误差 ≤ 250 px
marked = fresh and correct_class and pixel_error <= 250

# 连续保持标注 3 s 才开舱门，帧间隔 >0.75 s 重新计时
if mark_last - mark_since >= 3.0:
    open()

# 从首次看到目标起 12 s 兜底：有有效帧就投，没有则放弃该点
```

人员区任务完成后回到原航点 5.2，再按返航线回家。

## 6. 任务编排

状态机主线：

```
WAITING → PRESTREAM → READY → SWITCHING → ARMING → TAKEOFF
→ TRANSIT / POINT_HOLD（循环航点）→ RETURN → RETURN_HOLD → DESCEND → LAND_WAIT
```

视觉任务子状态：`SEARCH → ALIGN → OPEN_WAIT → DROP_DWELL → CLOSE_WAIT → RETURN_ANCHOR → DONE`。

关键分支：

- **危险品没识别到**：跳过该投放，把航点标记为完成，继续飞人员区，按配置正常返航（`continue_after_target_not_found`）。
- **正常完成**：按 `return_indices: [3,1,0]` 原路返回。
- **异常/超时**：按 `_breadcrumbs` 逆序返回（飞过的点倒着走），任务区不停留。
- **返航段起点**：从飞机实际位置起步，不强制回到航点，避免把残余偏移变成整段航线的横向基准。

## 7. 安全边界

- SWA 开关随时人工接管（POSCTL）。
- 航向漂移 >10°、航线横向偏差 >0.4 m、速度/倾角超限 → 退出。
- 落地前校验飞控落地检测参数（`LNDMC_Z_VEL_MAX`、`COM_DISARM_LAND` 等），不兼容则拒绝起飞。
- 任务总预算 180 s，超时强制返航。
- 未停稳 / 定位失效 / 视觉进程重启均有退出或兜底路径。

---

## 说明

- 校准文件 `coordinates.json`（现场实测坐标）不在本仓库。
- 模型权重（YOLOv5 `.engine`）不在本仓库，仅算法与源码。
- 离线测试覆盖航点推进、视觉投放、返航、安全边界等，`simple_flight_mission/test/` 下。
