# 传感器确认式飞行序列（Sequencer）设计

日期：2026-09-13

## 1. 背景与问题

固件现有两条飞行控制路径：

1. **低层 setpoint 流**（`crtp_commander_generic.c` + `position_controller_pid.c`）：Python 侧以 20Hz 持续发送 `send_position_setpoint`/`send_zdistance_setpoint`，`python/t5_hover_land.py`、`t6_flight_sequence.py` 已多次实机验证。
2. **高层规划器**（`crtp_commander_high_level.c` + `planner.c`）：`takeoff2`/`land2`/`go_to` 等命令基于时间轴的多项式轨迹（poly4d），`python/t7_hl_commander_check.py` 已确认固件编译了这条路径，但**从未做过飞行验证**。

`t5`/`t6`/`t7` 的注释里记录了两个已确认的根因，合起来会导致"提前完成/提前降落"类事故：

- `position_controller_pid.c` 的 z 轴位置 PID（`positionController()` 约 218 行）**覆盖**而非叠加 planner 给出的速度前馈——平缓的爬升/下降斜坡会让瞬时位置误差一直很小，PID 算出的推力修正也很小，实际爬升/下降明显滞后于计划；直到斜坡终点固定不再前移，误差才真正积累、PID 才发力。
- 即使不考虑上一条，`stateEstimate.z` 本身相对真实高度有 0.1~0.2m 的滤波延迟。

两者叠加的结果：只要"完成"信号纯粹基于时钟/duration，就随时可能在飞机还没到（起飞）或还没落地（降落）时误判为"完成"，进而触发下一步动作（尤其是降落时的切电机）——这正是 `t5_hover_land.py` 记录的实机摔机成因。

修复方式经讨论确定为：**完成信号跟"计划状态"解耦，只认物理测量**（VL53L1 原始值），`timeout` 只作为安全兜底、不作为默认成功。至于 `position_controller_pid.c` 本身缺前馈的问题，评估后**不在本次范围内**（见第 8 节）。

## 2. 目标与非目标

**本次目标**：在固件里新增一个可被 CRTP 上传、自主执行的"飞行序列"能力，支持三种步骤：`TAKEOFF_SENSOR`、`DELAY`、`LAND_SENSOR`，以及 `CANCEL`（中止并执行 LAND_SENSOR）、`STOP`（复用现有急停，立即断电）。序列一旦 `START`，由固件在专职任务里自主跑完全程，Python 不需要持续发包驱动，只负责上传、启动、监控、必要时 cancel/stop。

**非目标（本次不做）**：
- `GOTO_SENSOR`（水平位移步骤）——协议预留步骤类型和字段位置，但本次拒绝上传、不实现执行逻辑。
- 修复 `position_controller_pid.c` 缺前馈的问题——见第 8 节，记为独立事项。
- QMC5883L 磁力计融合进 yaw 估计——见第 8 节，记为独立事项。
- 不改动/不依赖现有 `planner.c`/`pptraj` 时间轴规划器，老的 `takeoff2`/`land2`/`go_to` 命令行为不变。

## 3. 架构

新增独立模块 `components/core/crazyflie/modules/{interface,src}/sequencer.h/.c`：

- **专职 FreeRTOS 任务** `sequencerTask`：`SEQ_START` 之后开始运行，按固定周期（20ms，接近 VL53L1 驱动自身 25ms 的刷新节奏）醒来一次，读取 `rangeGet(rangeDown)`，推进当前步骤的斜坡/`stable_hold`/`timeout` 状态机，把算出的 setpoint（`x=0,y=0,yaw=0` 固定 + 动态 `z`）写入一个互斥锁保护的共享结构。`state==IDLE` 时任务阻塞在通知上，不空转。
- **CRTP 入口复用现有端口**：`crtp_commander_high_level.c` 的 `enum TrajectoryCommand_e` 追加新 opcode，`handleCommand()` 分发给 `sequencer.c` 暴露的函数；现有 `takeoff2`/`land2`/`go_to`/`planner.c` 代码不改动。
- **两个必要的钩子**：
  1. `crtpCommanderHighLevelGetSetpoint()`：先检查 sequencer 是否 active，是则从共享结构拷贝 setpoint 返回，否则走原有 `plan_current_goal()` 分支。
  2. `crtpCommanderHighLevelIsStopped()`：改为 `plan_is_stopped(&planner) && sequencerIsIdle()`——否则序列运行中收到 `STOP`，`commander.c` 不会把 setpoint 清零，达不到"立即断电"效果。
- **互斥前提**：`SEQ_START` 要求 `plan_is_stopped(&planner) == true`，防止老规划器和新 sequencer 同时争抢 `crtpCommanderHighLevelGetSetpoint()` 的输出。

`sequencer.c` 不依赖 `crtp_commander_high_level.c` 内部的 `pos`/`yaw` 静态变量（见第 4 节），因此除了上面两个钩子和 CRTP 分发外，与老代码没有共享状态耦合。

## 4. 水平位置与朝向：固定 x0=y0=yaw0=0

序列全程（`TAKEOFF_SENSOR`→`DELAY`→`LAND_SENSOR`）持续输出固定闭环目标：`mode.x=modeAbs,position.x=0`、`mode.y=modeAbs,position.y=0`、`mode.yaw=modeAbs,attitude.yaw=0`，走 `position_controller_pid.c` 现有的 `positionController()` 逻辑（208~219 行），如果估计值偏离这个目标，控制器会像正常悬停一样输出纠正量。

本机已安装 PMW3901 光流模块，Kalman 估计器下 x/y 是真实的光流闭环，不是纯加速度计双积分——上述"偏离会被纠正"对本机成立。yaw 目前是纯陀螺积分（无磁力计融合，见第 8 节），几秒量级的序列里陀螺漂移可忽略，闭环同样有意义。

只有 `TAKEOFF_SENSOR` 起始 z 需要读取一次当前实际高度，直接从 `crtpCommanderHighLevelGetSetpoint(setpoint, state)` 传入的 `state` 参数读 `state->position.z`，不引入新的共享状态。

## 5. 线协议

复用现有 CRTP 端口（`CRTP_PORT_SETPOINT_HL`），`CRTP_MAX_DATA_SIZE=30` 字节。`enum TrajectoryCommand_e` 新增：

| opcode | 值 | 载荷 | 说明 |
|---|---|---|---|
| `COMMAND_SEQ_CLEAR` | 11 | 无 | 清空步骤缓冲区（count=0）。仅 `state==IDLE` 时有效 |
| `COMMAND_SEQ_ADD_STEP` | 12 | `uint8_t step_type` + `float params[7]`（28字节）= 29字节，加 1字节 opcode 正好 30 字节 | 追加一步。仅 `IDLE` 时有效，缓冲区上限 16 步 |
| `COMMAND_SEQ_START` | 13 | 无 | 校验通过后启动（见第 6 节校验规则） |
| `COMMAND_SEQ_CANCEL` | 14 | 无 | 仅 `RUNNING` 时有效；合成一个 `LAND_SENSOR` 步（`target=0`, `duration=seq.cancelLandDurationS`）立即执行 |
| 复用 `COMMAND_STOP` | 3 | 无（已有） | 急停：清空序列到 `IDLE` + 原有 `plan_stop()`，见第 3 节 `IsStopped()` 改动 |

`step_type` 取值与 `params[]` 语义：

```
0 = TAKEOFF_SENSOR : params[0]=target_m, params[1]=stable_hold_s, params[2]=timeout_s
1 = DELAY           : params[0]=hold_time_s
2 = LAND_SENSOR     : params[0]=target_m, params[1]=duration_s
3 = GOTO_SENSOR（预留，本次上传直接拒绝，返回 ENOEXEC）
```

`params[7]` 的宽度按 `GOTO_SENSOR` 未来需要的 7 个 float（`offset_x/offset_y/z/tol_xy/tol_z/stable_hold/timeout`）预留，协议格式本次不用再改。响应沿用现有 ack 机制（`p.data[3]=ret`，0=成功，`ENOEXEC`=拒绝）。

## 6. 状态机与校验规则

顶层状态：`IDLE` → `RUNNING`（执行 `steps[idx]`）→（可选）`LANDING` → `IDLE`。`LANDING` 是 `LAND_SENSOR` 步骤的执行态，序列里正常走到的 `LAND_SENSOR` 步骤、`CANCEL`、`TAKEOFF_SENSOR` 超时中止，三者共用同一套 `LANDING` 逻辑。

- **TAKEOFF_SENSOR**：z 以 `seq.takeoffVelMps`（可调参数）匀速斜坡爬向 `target_m`，起点为进入本步骤时的 `state->position.z`。每 tick 读 `rangeGet(rangeDown)`（原始 mm），在 `±seq.tolTakeoffMm` 容差内连续保持 `stable_hold_s` → 完成，进入下一步（计时器归零）。若 `timeout_s` 到仍未确认 → **整个序列转入 `LANDING`**（已确认策略，不是"当作完成继续走"）。
- **DELAY**：setpoint 钉死不变（水平/朝向固定 0，z 保持进入本步时的值），`hold_time_s` 到即进入下一步，不涉及传感器。
- **LAND_SENSOR**：z 按 `duration_s` 斜坡降向 `target_m`（通常 0）。同时持续读原始 mm 值判断触地：连续 `seq.touchdownConfirmS` 都 `≤ seq.touchdownMm` 才确认触地；斜坡走完后若还没确认，继续等待并保持发送 `target_m`，直到 `seq.landMaxWaitS` 硬上限强制切电机。确认触地或硬上限到达 → 切电机（setpoint 归零等效于 `IsStopped()` 生效）、整个序列回 `IDLE`。这一步是终态，不管是序列自身的最后一步还是被 `CANCEL`/超时顶上来的，结果一致。
- **`SEQ_START` 校验**（仿照 `t6_flight_sequence.py` 的 `validate_flight_plan` 思路，固件侧硬校验，不信任 Python 预校验）：缓冲区非空；最后一步必须是 `LAND_SENSOR`；`plan_is_stopped(&planner)==true`；否则拒绝，返回 `ENOEXEC`。
- **`SEQ_ADD_STEP` 校验**：`state!=IDLE`、缓冲区已满、`step_type` 未知/是 `GOTO_SENSOR`、或参数越界（`target_m` 不在 `(0, 2.0]`m、`stable_hold_s<0`、`timeout_s<=stable_hold_s`、`hold_time_s<=0`、`duration_s<=0`）都直接拒绝，不进缓冲区。
- **`SEQ_CANCEL` 校验**：`state!=RUNNING` 时视为 no-op（已经在 `LANDING`/`IDLE` 不重复触发）。
- **传感器读数容错**：若 `rangeGet(rangeDown)` 原始值 `> seq.rangeSaneMaxMm`（默认 4000mm，对应 VL53L1 datasheet 有效上限），本 tick 视为"不可信"，既不计入 `stable_hold`/触地的连续计时，也不重置已累积的计时——只是跳过这一帧，靠 `timeout`/`landMaxWaitS` 兜底，不会因为偶发野值直接判失败。

## 7. 可调参数与遥测

`PARAM_GROUP_START(seq)`（默认值取自 Python 脚本里已经实机调出来的经验值）：

| 参数 | 默认值 | 来源/理由 |
|---|---|---|
| `seq.takeoffVelMps` | 0.15 | `t7_hl_commander_check.py` `TAKEOFF_VELOCITY_MPS` |
| `seq.tolTakeoffMm` | 30 | `t7` `HOVER_SETTLE_TOLERANCE_MM` |
| `seq.touchdownMm` | 80 | 介于 `t5`(60) 与 `t6`(90) 之间，取中间值，留待实机微调 |
| `seq.touchdownConfirmS` | 0.3 | `t6` `TOUCHDOWN_CONFIRM_S` |
| `seq.landMaxWaitS` | 5.0 | `t6` `TOUCHDOWN_MAX_WAIT_S` |
| `seq.cancelLandDurationS` | 2.0 | 对齐用户给出的示例 `Seq[5]: LAND, ..., duration=2.0s` |
| `seq.rangeSaneMaxMm` | 4000 | VL53L1 datasheet 有效量程上限，与现有 Python 脚本 `RANGE_SANE_MAX_MM` 一致 |

`LOG_GROUP_START(seq)`：`state`（0=IDLE/1=RUNNING/2=LANDING）、`stepIdx`（uint8，IDLE 时为 0xFF）、`stepType`、`elapsedMs`（当前步骤已耗时，用于跟 `timeout`/`stable_hold` 对照）。

## 8. 本次不做、但已记录的独立事项

两项都已写入项目记忆，供后续任务参考：

- **`position_controller_pid.c` 缺速度前馈**：`positionController()` 用 `runPid()` 的结果覆盖而非叠加 planner 的 `ev.vel`，是"提前完成"类 bug 的根因之一。本次通过传感器确认绕开了它，不需要修，但这是共享控制环代码，影响所有飞行模式（包括老的 `takeoff2`/`land2`/`go_to`），如果以后要修，需要单独验证，不建议顺带塞进本次改动。
- **QMC5883L 磁力计融合进 yaw**：驱动已经在读原始 mag 数据（`sensorData.mag`），但 `estimator_complementary.c`/`estimator_kalman.c` 完全没有消费它，yaw 目前是纯陀螺积分、零点是开机时的任意朝向。本次序列的 yaw0=0 就用这个现有估计值，足够（几秒量级、无主动转向）。等以后 `GOTO_SENSOR` 需要跨次飞行的一致世界系偏移时，这个事项会变成前置依赖。

## 9. Python 验证脚本

新增 `python/t8_sensor_sequence_check.py`，沿用 `t6`/`t7` 的约定：

- 连接、读取并确认 `commander.enHighLevel`；确认当前 `stabilizer.estimator`（本机装了光流，期望是 kalman=2，非 2 则警告但不阻止）。
- cflib 不认识新 opcode，手工构造 `CRTPPacket` 发送 `SEQ_CLEAR`/`SEQ_ADD_STEP`×N/`SEQ_START`。
- 订阅 `range.zrange`、`stateEstimate.x/y/z`、`seq.state/stepIdx/stepType/elapsedMs` 日志，按 t6/t7 的节流方式打印进度。
- 独立的 Python 侧看门狗（日志新鲜度 + 总时长硬上限）：触发时发送现有 `COMMAND_STOP`（急停）——固件自主执行不代表放弃这层兜底，双保险，与 t5/t6/t7 一致。
- 默认序列即最小三步：`TAKEOFF_SENSOR(target=0.3, stable_hold=0.5, timeout=3.0) → DELAY(hold_time=1.0) → LAND_SENSOR(target=0, duration=2.0)`。
- `Ctrl+C` 优先发 `SEQ_CANCEL`，如果发送本身失败再退化为 `STOP`。

## 10. 未决风险 / 留给实现计划细化的点

- `seq.touchdownMm` 默认 80mm 是经验估计，未在本机实测验证，预计首次试飞后需要调整（同 t5/t6/t7 的调参历史）。
- `sequencerTask` 与 `crtpCommanderHighLevelTask`/stabilizer 任务之间的共享结构加锁方式（互斥量 vs 关中断拷贝）留给实现计划里具体定夺，需评估 20ms 任务周期下的锁竞争开销。
- CRTP 命令的具体 C 结构体/`ENOEXEC` 错误码到 Python 侧的呈现方式（当前只有一个整数 ack）留给实现计划细化，是否需要更细的错误码枚举。
