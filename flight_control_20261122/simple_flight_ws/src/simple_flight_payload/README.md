# 比赛控制台与舱门：2026-10-30

**2026-10-30 比赛自动锁桨收尾已部署：** 返航落地且原点范围内静止确认约0.5秒后，比赛入口请求一次普通锁桨，等待实际armed=false后完成。没有强制锁桨或飞控参数修改，ACK不当作实际上锁。未落地、弹跳、陈旧反馈、人工接管或非比赛收尾不发该请求；不接受/超时不强制重发。原“只等PX4自锁、程序不发锁桨”描述是历史比赛行为。普通route_node仍保持旧策略。

**2026-10-30 启动确认位置纠正，已部署（`ready_enter_20261030_37bb7a98`）：** 运行`bash ~/simple_flight_ws/competitionctl.sh run 1`后，在同一个执行进程中先完成初始两门关闭、实时检查、静止准备与地面目标预发送，进入READY再提示Enter。按Enter后才请求OFFBOARD/解锁/起飞，不重启进程、不重复上述准备。准备阶段会实际关闭舱门；未确认前不请求OFFBOARD或解锁。等待期间20Hz地面目标与实时安全检查持续运行，提前Enter丢弃、EOF/Ctrl+C取消、SWA不能绕过Enter；180秒从确认开始计。仅交互SSH支持run，旧GUI后台run继续拒绝。下方“Enter后才关门”和独立只读检查后重启执行是前一版行为，已替换。

**2026-10-30 最新启动方式，已部署：** 在交互SSH终端运行`bash ~/simple_flight_ws/competitionctl.sh run 1`，先准备相机/录像并运行只读预检，通过后显示“准备完成。按 Enter 后将关闭两舱门并自动起飞；Ctrl+C 取消。”只有提示后按空白Enter才进入原关门/实时检查/起飞流程；准备期间提前按的Enter会丢弃。等待不计入180秒任务预算。预检失败、Ctrl+C、终端断开均不启动。非交互/管道输入会拒绝run；因此旧桌面控制台的nohup后台启动按钮暂不能用于run，请使用交互SSH命令。check/vision/status仍可非交互。下文“run直接起飞”及桌面按钮启动是历史行为，以本条为准。

本版已部署到 Nano 的 `~/simple_flight_ws`；视觉仍在 `~/visual_recognition_ws`。
与 FLMG 地面站工程分开，不使用 flmg_uav 同步脚本，不改飞控参数或定位服务。

## 下午现场使用

电脑桌面入口：**无人机比赛控制台.cmd**。输入 SSH 地址 `10.42.0.1`、账号 `jetson` 与原登录密码；密码仅在内存中使用，不保存。

1. **开启并确认遥控器链路，切到 Position 档。** 本次后续核验时飞控USB已自动恢复，位置/高度有效；当前未解锁、AUTO.LOITER、遥控输入未就绪，真实只读预检退出2。
2. 拆桨、卸下物资，打开控制台并连接。点击“画面与录像”，浏览器显示真实下视摄像头。危险品红框、人员蓝框，干扰标志不触发投放。
3. 分别用“1关180° / 1开0° / 2关180° / 2开0°”检查机构。门1=消防/AUX7，门2=救援/AUX8。默认/关闭=180°，打开=0°；开门测试顺序为先到180°→开到0°→再关回180°，并把上锁脉宽写成关门位，避免弹回90°。
4. 选路线1～4，遥控器保持 Position 档、SWA低位、油门低位，点击“起飞前检查”。此操作只读，不会驱动舵机或飞行。检查失败时看日志原因，启动按钮保持禁用。
5. 由飞手确认场地与起飞条件后，点击“启动飞行”。该按钮会实际自动起飞；程序先关闭两门，再完成预飞检查、预发送目标和飞控模式/解锁请求。不再需要起飞时拨一次SWA。
6. “停止任务 / 遥控接管”请求原有POSCTL交接，**不是自动降落或停桨**。独立遥控模式切换仍有效；Notebook启动下，SWA先高再低可中止，油门抬高再拉低的原接管逻辑保留。关闭窗口不停止已启动任务。

首次带桨验收须由你现场完成。只读检查、虚拟ROS完整流程和实际飞行动力学是不同验收范围。

## 自动任务行为

起飞 → 原固定路线 → 危险品任务区 → 识别正确 Hazardous（不选 Interference）→ 对准目标中心后立即开门1、保持1秒落料、关门 → 原任务航点 → 人员任务区 → 搜索识别 Person → 对准中心后立即开门2、保持1秒落料、关门 → 原任务航点 → 沿原路线返航 → 降落并等待飞控自动上锁。按用户2026-10-30最新要求，取消投放前稳定计时；对准使用新鲜画面中心误差≤12px，未对准不投放。

- 第一次找到目标后，以世界XY方向各一次8cm小位移，计算图像像素/实际位置的二维雅可比，不假定画面上下对应机头方向。后续任务在同高度、同航向下复用本次测量，不写入猜测的永久标定。
- 任务区移动目标限制为0.12m/s；视觉修正距首次捕获位置不超过0.35m。探测与搜索通道的真实净空必须在现场检查。
- 人员未出现时，沿既有救援通道的前向1.20～2.87m范围分段搜索，左向坐标沿用本条路线；识别和投放后回到原任务航点。
- 目标过期或回放图不会用于投放；每区最多一次投放。没有正确目标、标定失败、命令失败/结果未知会结束该任务并沿既有通道返航，不盲投。
- 180秒计时包含启动后的关门、静止确认与预发送阶段；预算根据已走航段、返航距离和下降速度提前留出返航时间。旧 `route_node` 的300秒设置保持原状，比赛使用新入口。
- 命令ACK只代表飞控接受；没有舱门/落料传感器，不会把ACK声称为机械动作或物资落下已确认。

## 舵机参数

用户指定两门初始关闭180°、打开0°。当前配置采用显式的标准线性映射：0～180°对应1000～2000微秒，关门2000us、开门1000us；**真实舵机角度仍须按现场机构确认**。

实际配置：`~/simple_flight_ws/config/doors.json`。可改各门 `open_us` / `close_us` 为实测脉宽，优先于角度字段；也可按舵机规格改角度映射。修改外部JSON不需重建。当前实读AUX7/8功能为301/302、范围1000～2000us、无反向；COM_PREARM_MODE=0保持不变。QGC输出范围、反向位与配置必须一致，发送前会只读核对。

Nano / Linux 手动命令（实际动作，仅拆桨未解锁调试）：

```bash
source /opt/ros/humble/setup.bash
source ~/simple_flight_ws/install/setup.bash
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
ros2 run simple_flight_payload doorctl 1 close --config ~/simple_flight_ws/config/doors.json --ground-test --execute
ros2 run simple_flight_payload doorctl 1 open  --config ~/simple_flight_ws/config/doors.json --ground-test --execute
ros2 run simple_flight_payload doorctl 2 close --config ~/simple_flight_ws/config/doors.json --ground-test --execute
ros2 run simple_flight_payload doorctl 2 open  --config ~/simple_flight_ws/config/doors.json --ground-test --execute
```

微调时用 `doorctl 1 set --pwm 1500 --ground-test --execute`，逐步改变PWM；不加 `--execute` 只打印预览。
飞行投放底层为 `/mavros/cmd/command` / MAV_CMD_DO_SET_ACTUATOR(187)。地面 `--ground-test` 用 MAV_CMD_ACTUATOR_TEST(310) 驱动舵机，并用 DO_SET_ACTUATOR + `PWM_AUX_DIS*`=关门脉宽把默认位固定在180°；`open` 序列为 home180→open0→close180。命令超时均不自动重发。

## 不用控制台的 Nano 命令

```bash
bash ~/simple_flight_ws/competitionctl.sh vision   # 只启动真实相机与录像
bash ~/simple_flight_ws/competitionctl.sh check 1 # 路线1只读检查
bash ~/simple_flight_ws/competitionctl.sh run 1   # 准备/预检后等待Enter，再自动起飞；1可改2/3/4
bash ~/simple_flight_ws/competitionctl.sh status
bash ~/simple_flight_ws/competitionctl.sh stop    # 请求交还飞手，不是降落
bash ~/simple_flight_ws/competitionctl.sh stop-vision # 无任务时停止本工具启动的视觉
```

同一时间只运行一个控制任务。独立视觉服务器不接受飞行控制HTTP请求；所有启动/停止均走已认证SSH。

## 录像、日志与重新构建

- 实时页面：`http://10.42.0.1:8088`。
- 录像：`~/simple_flight_ws/log/competition_20261030_*.avi`，5FPS MJPEG，带检测框和新鲜任务状态。
- 任务事件：`~/simple_flight_ws/log/competition/<run_id>/events.jsonl`。
- 最近状态：`~/simple_flight_ws/log/competition_status.json`；启动日志 `log/console_run.log`。
- 本次实测相机约2.5～5FPS，15次采样均新鲜；录制的43帧AVI可正常读取。该结果不替代实际飞行、光照与投放精度验收。

Nano 时钟落后会导致 Python 构建缓存跳过新文件。后续改源码用：

```bash
bash ~/simple_flight_ws/src/simple_flight_payload/scripts/rebuild_competition.sh
```

该脚本只处理这两个Python包，并核对/刷新源码、构建、安装文件，避免加载旧版本。不会改系统时间或厂商库。

## 本次已做的验证

- Windows：任务包、舱门和视觉核心共258项通过，另13个subtest通过。
- Nano 隔离ROS域77 + localhost：只读零输出、缩短距离的完整比赛消息流程2项通过。虚拟对端收到初始关门1/2，再开关1、开关2；不是连接真实飞控的测试。
- Nano 视觉核心/HTTP：6项通过；真实相机取流与录像可读检查通过。
- 真实飞控串口和定位已恢复，最终只读预检因遥控输入/预飞/模式未就绪而退出2；未执行真实解锁、舵机动作或飞行。

## 回退

部署前源码备份：`~/simple_flight_ws/backups/codex_20261030_1209_source.tar.gz`。
现有定位服务、模型、飞控参数均未更改。需要回退任务时，先停止任务并确认未解锁，从该备份恢复原任务包源码后重新构建；原入口仍为 `route_node --route 1/2/3`，由原SWA流程启动。不要在飞行中恢复或重启。

接口资料：[PX4通用执行器控制](https://docs.px4.io/main/en/payloads/generic_actuator_control)、[QGC执行器测试命令实现](https://github.com/mavlink/qgroundcontrol/blob/master/src/Vehicle/Actuators/ActuatorTesting.cc)。
