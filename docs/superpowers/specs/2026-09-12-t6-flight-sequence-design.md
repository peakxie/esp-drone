# t6_flight_sequence.py 设计文档

## 背景

`python/` 目录下已有一系列递进的飞行测试脚本（t0/t3/t4*/t5/t5b），其中 `t5_hover_land.py` 是第一个真正离地飞行的脚本：起飞爬升→定高悬停→降落，三个阶段硬编码在 `main()` 里，通过 `send_position_setpoint` 让固件 `position_controller_pid.c` 的位置外环闭环控制 x/y/z，并踩出了一整套安全约定（20Hz 发送、日志新鲜度看门狗、硬性总时长上限、Ctrl+C/看门狗触发走斜坡降落而不是瞬间停桨、触地判定用 `range.zrange` 而不是融合后的高度估计等）。

现在需要一个新脚本，能在**一次连接内按顺序执行多条命令**（起飞、降落、悬停、飞向指定点、改变高度等），而不是每种测试都单独写一个脚本。新脚本要完整复用 t5 已经验证过的安全模型，不能因为"支持多命令"而引入新的风险面。

## 目标

- 新增 `python/t6_flight_sequence.py`，在脚本顶部用一个 Python 列表声明"飞行计划"（一串命令），单次连接顺序执行。
- 命令集：`takeoff` / `land` / `hover`（悬停）/ `goto`（指定定点，同时承担"定高"语义）。
- 完整复用 t5 的安全约定：看门狗、日志新鲜度检查、总飞行时长硬上限、触地确认、紧急下降、Ctrl+C 处理。

## 非目标

- 不引入固件的 `HighLevelCommander`（`crtp_commander_high_level.c` / `COMMAND_TAKEOFF_2` 等）。经调研，ESP-Drone 固件确实编译了这部分代码，但从未做过飞行验证，风险未知，本次不采用；继续沿用 t5 已验证的 `send_position_setpoint` 路线。
- 不支持 yaw 目标（横摆角）。全程固定为起飞时的 `yaw0`，跟 t5 一致——项目里还没有验证过 yaw 控制。
- 不支持外部配置文件/命令行参数指定飞行计划，命令列表就是脚本内的 Python 常量，改测试就改这个列表（跟 t5/t5b 各自是独立脚本文件的风格一致）。
- 不做轨迹平滑（贝塞尔/多项式等），过渡方式统一为线性斜坡。

## 命令集设计

命令列表是元组列表，每个元组 `(命令名, *参数)`：

```python
FLIGHT_PLAN = [
    ("takeoff", 0.5),                # 起飞爬升到绝对高度 0.5m
    ("hover", 3.0),                  # 原地悬停 3.0s
    ("goto", 0.3, 0.0, 0.5, 2.0),    # dx=0.3m, dy=0, h=0.5m（绝对），2.0s 内斜坡过渡
    ("hover", 2.0),
    ("goto", 0.0, 0.0, 0.3, 1.5),    # dx/dy 不变，只降高度到 0.3m —— 复用 goto 表达"定高"
    ("land",),                       # 降落
]
```

- `takeoff(height_m, duration_s=TAKEOFF_TIME_S)`：从地面斜坡爬升到绝对高度 `height_m`，x/y 钉在起飞点（对应 t5 阶段1，斜坡起点固定为 `LIFTOFF_HEIGHT_M` 常量，不是 0）。
- `hover(duration_s)`：保持当前目标（dx/dy/h）不变，持续发送 `duration_s`（对应 t5 阶段2 `hold`）。
- `goto(dx, dy, h, duration_s)`：把目标从当前值线性斜坡过渡到 `(dx, dy, h)`——**`dx`/`dy` 是相对起飞点 `(x0, y0)` 的偏移量，`h` 是绝对高度**。统一承担"指定定点"和"定高"两种语义：`dx=dy=0` 只变高度时就是"定高"。
- `land(duration_s=LAND_TIME_S)`：斜坡降到 `LAND_HEIGHT_M`（x/y 保持当前值不变）→ `wait_for_touchdown`（`range.zrange` 触地确认，阈值/确认时长沿用 t5 的 90mm/0.3s 结论）→ 停桨（15 次 `send_stop_setpoint`，不做推力斜坡——t5 已证实推力斜坡在 `altHoldMode` 下会被误解读成油门摇杆，导致"落地又弹起再摔下去"）。
- yaw 全程固定为 `yaw0`，不作为参数。

## 内部架构

- **通用过渡原语**：把 t5 的 `send_ramp`（只斜坡高度）泛化为 `ramp_to(dx, dy, h, duration_s)`，同时线性插值 dx/dy/h 三个量，内部调用 `cf.commander.send_position_setpoint(x0+dx, y0+dy, h, yaw0)`。`takeoff`/`goto`/`land`/紧急下降全部复用这一个函数。
- **当前目标记录**：`current_target = {"dx": 0.0, "dy": 0.0, "h": 0.0}`，记录"最后一次发出的目标"，供 `ramp_to` 的斜坡起点、紧急下降的起点使用。
- **`hold(duration_s)`**：按 `current_target` 的值持续发送 `duration_s`，供 `hover` 命令使用。
- **`wait_for_touchdown(max_wait_s)`**：逻辑与 t5 完全一致（连续 `TOUCHDOWN_CONFIRM_S` 内 `range.zrange <= TOUCHDOWN_ZRANGE_MM` 才判定触地），供 `land` 命令使用。
- 以下逐字复用 t5 的实现，不做修改：
  - `watchdog_ok()`（总时长硬上限 + 日志新鲜度检查）
  - `print_status()`（x/y/z/thrust 打印，靠近地面时不受节流限制）
  - 起飞前 `range.zrange` 合理性检查（`RANGE_SANE_MAX_MM` 上限，不设下限）
  - `velCtlPid.vxKi`/`vyKi` 运行时覆盖
  - `read_current_estimator()` 检查是否为 kalman，非 kalman 时打印漂移警告但仍尝试起飞
- **命令分发**：`dict` 把命令名映射到处理函数，`main()` 里对 `FLIGHT_PLAN` 顺序遍历分发执行。

## 静态校验（连接前执行，不依赖飞机在线）

在 `open_link` 之前对 `FLIGHT_PLAN`做一次纯本地校验，失败直接退出，不尝试起飞：

- 命令名必须是已知命令（`takeoff`/`hover`/`goto`/`land`），参数个数必须匹配。
- 第一条命令必须是 `takeoff`，否则报错拒绝执行（没有起飞就不可能有后续任何命令的前提条件成立）。
- `goto` 的 `dx`/`dy` 绝对值不超过新增常量 `MAX_XY_OFFSET_M`（建议默认 1.0m，房间净空有限），`h` 在 `(0, MAX_HEIGHT_M]`（建议默认 1.0m）范围内——防止手误参数导致意外大位移/大高度。`takeoff` 的 `height_m` 同样受 `MAX_HEIGHT_M` 约束。

## 安全行为

- **全局硬上限**：`MAX_FLIGHT_TIME_S` 是整个命令列表执行期间共用的**单一** deadline（在第一条命令开始执行前设定一次），不是每条命令单独计时——避免多命令场景下总时长失控。
- **自动补 land**：如果 `FLIGHT_PLAN` 最后一条不是 `land`，正常执行完所有命令后自动追加一次完整 `land`（斜坡+触地确认+停桨），不允许悬在半空直接结束脚本。
- **紧急下降**：`FlightAbort`（`watchdog_ok()` 返回 False 时主动抛出）或 `KeyboardInterrupt`，不管当前正在执行哪条命令，统一从 `current_target` 记录的最后目标用 `EMERGENCY_LAND_TIME_S` 斜坡降到地面，然后停桨——同 t5。

## 复用范围之外的部分

- `config.py`（URI、`connect_with_timeout`）不改动，直接复用。
- 日志订阅（`range.zrange`/`stateEstimate.x/y/z`/`stabilizer.yaw`/`stabilizer.thrust`，20Hz `LogConfig`）原样复用 t5 的字段和周期。

## 验证方式

这是真实起飞的飞控测试脚本，无法用单元测试验证飞行安全性；验证方式是：
1. 静态校验单独可测（构造几个非法 `FLIGHT_PLAN` 例子，确认在连接前就被拒绝，不会尝试连接/起飞）。
2. 实机测试：先用只含 `("takeoff", 0.3)` + `("hover", 2.0)` + `("land",)` 的最小计划验证行为与 t5 一致，再逐步加入 `goto` 验证水平移动。
