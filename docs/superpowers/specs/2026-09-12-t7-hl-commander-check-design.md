# t7_hl_commander_check.py 设计文档

## 背景

`python/` 目录下现有的飞行脚本（t5_hover_land.py、t6_flight_sequence.py）全部走同一条路径：Python 侧用 `send_position_setpoint` 周期性发送 setpoint（20Hz），由固件 `position_controller_pid.c` 的位置外环闭环控制，t6 的设计文档里明确记录了"不使用固件的 `HighLevelCommander`（`crtp_commander_high_level.c`），因为这部分代码从未做过飞行验证"这一决定。

现在需要迈出这一步：验证 ESP-Drone 固件里编译进去的 `HighLevelCommander`（CRTP 端口 0x08）路径本身能不能正常起降，为以后是否要迁移到这条路径（用官方 `PositionHlCommander` 减少手写 ramp 代码）提供实测依据。

调研（对固件源码逐行核对，非二次转述）发现两类事实：

1. **命令兼容性**：`crtp_commander_high_level.c` 的命令分发里，`COMMAND_TAKEOFF_2`(7)、`COMMAND_LAND_2`(8)、`COMMAND_STOP`(3)、`COMMAND_GO_TO`(4，旧版) 均已实现；新版 `COMMAND_GO_TO_2`(12)/`COMMAND_START_TRAJECTORY_2` 等在这份固件的枚举里根本不存在。但固件上报的 `PROTOCOL_VERSION=4`（`config.h:46`），cflib 的 `HighLevelCommander.go_to()` 在协议版本 <8 时会自动选旧版 `COMMAND_GO_TO`，不会触发不存在的 `_2` 命令——本次测试不使用 `go_to`，这条只是排雷确认。
2. **必须的前提 param（关键新发现）**：`commander.c:48` 定义 `static bool enableHighLevel = false`，对外暴露为运行时 param `commander.enHighLevel`。`commanderGetSetpoint()`（`commander.c:104-116`）只有在这个 param 为真时才会把 setpoint 交给 `crtpCommanderHighLevelGetSetpoint()`；否则（包括默认状态）setpoint 恒为 `nullSetpoint`（停桨态）。也就是说**不显式把这个 param 设成 1，`take_off()`/`land()` 命令会被固件静默忽略，电机不会响应，Python 侧的 `time.sleep(duration_s)` 还是会正常走完**——这是一个静默失败模式，必须在起飞前设置并回读确认。
3. `commander.c:80-88`：任何一次低层 setpoint（`send_position_setpoint`）入队都会强制调用 `crtpCommanderHighLevelStop()`，把高层规划器打回 idle。因此本脚本必须是纯高层命令路径，不能像 t6 那样混用低层 setpoint 发送。
4. x/y 位置估计只有在 kalman 估计器下才是真实值；默认的 complementary 估计器下 `position_estimator_altitude.c` 把 x/y 硬编码为 0（`position_estimator_altitude.c:106-108`）。本脚本只测垂直的 `take_off`/`land`（不测 `go_to`），水平方向的风险等同于 t5/t6 已知的"非 kalman 时缓慢漂移"警告，不是新增风险。

## 目标

- 新增 `python/t7_hl_commander_check.py`，用官方 `cflib.positioning.position_hl_commander.PositionHlCommander` 做一次最小的真实起降飞行测试：起飞到低高度 → 悬停几秒 → 降落 → 停桨。
- 验证固件的 `HighLevelCommander` 路径（`crtp_commander_high_level.c` 的 takeoff2/land2/stop）在真实硬件上能否正常工作，为后续是否迁移到这条路径提供实测依据。
- 起飞前显式设置并回读确认 `commander.enHighLevel=1`，避免静默失败。

## 非目标

- 不测试 `go_to`/`spiral`/预定义轨迹（`start_trajectory`）——本次只验证垂直起降这一最小闭环，水平移动留给以后单独验证。
- 不混用低层 `send_position_setpoint`（会打断高层规划器，见背景第 3 点）。
- 不引入 `cflib.utils.reset_estimator`：其依赖的 `kalman.varPX/varPY/varPZ` 日志和 `kalman.resetEstimation` param 虽然在固件里存在（`estimator_kalman.c`），但这个项目至今未验证过 kalman 是否真的在跑（`stabilizer.estimator` 的选择机制独立于这个工具函数），引入一个新的未验证等待逻辑会在本来就是"初次验证"的测试里叠加新的风险面。继续复用 t6 已有的 `read_current_estimator()` 只读警告即可。
- 不支持外部命令行参数/配置文件，高度、悬停时长等就是脚本内常量，改测试就改常量——跟 t5/t6/t5b 的风格一致。
- 不做斜坡/看门狗式的逐帧中断（`PositionHlCommander.take_off()`/`land()` 是阻塞调用，内部靠 `time.sleep()` 等固件规划器自己完成轨迹，Python 侧拿不到每帧插断的机会，这跟 t5/t6 的架构本质不同，见"安全模型"一节）。

## 命令流程

```python
with PositionHlCommander(cf, default_height=TAKEOFF_HEIGHT_M,
                          default_velocity=DEFAULT_VELOCITY_MPS) as pc:
    # __enter__ 已经执行 take_off()
    time.sleep(HOVER_TIME_S)
    # __exit__（正常退出或异常/Ctrl+C）自动执行 land()
```

- 起飞高度：`TAKEOFF_HEIGHT_M = 0.3`（沿用 t5/t6 首次测试的保守高度）。
- 起飞/降落速度：用 `PositionHlCommander` 默认的 `0.5 m/s`（`default_velocity`），不额外调整——先确认基本行为，不追求速度调优。
- 悬停：`HOVER_TIME_S = 3.0`，退出 with 块自动降落停桨（`PositionHlCommander.land()` 内部已经调用 `stop()`）。

## 起飞前检查（连接后、进入 `with PositionHlCommander` 之前）

逐字复用 t6 的实现：
- `connect_with_timeout`（`config.py`）建链。
- 订阅 `range.zrange`/`stateEstimate.z`（`aux_lg`，20Hz），等到第一帧到达确认遥测正常，否则拒绝起飞。
- `range.zrange` 起飞前合理性检查（`RANGE_SANE_MAX_MM` 上限）。
- `read_current_estimator()`：读取 `stabilizer.estimator`，非 kalman 时打印漂移警告但仍尝试起飞（跟 t6 一致，不是新行为）。

新增一步（本脚本特有）：
- `set_and_verify_param(cf, 'commander', 'enHighLevel', 1)`：设置 `commander.enHighLevel=1` 后立即回读确认（同 `read_current_estimator` 的"设置 param + 回调等值"模式）。回读失败或超时直接放弃起飞，不进入 `PositionHlCommander`。

## 安全模型

`PositionHlCommander.take_off()`/`land()` 是阻塞调用，内部只是根据"距离/速度"算一个 `duration_s` 然后 `time.sleep()`，实际的轨迹插值完全在固件端的规划器里完成——Python 侧没有 t5/t6 那种"每帧发送、每帧检查看门狗"的介入点。因此安全模型改成：

1. **正常/异常路径统一走 `land()`**：用 `with PositionHlCommander(...) as pc:` 语法，`__exit__` 保证无论是正常执行完、悬停时抛异常，还是 Ctrl+C（`KeyboardInterrupt`），都会调用一次 `land()`（内部再调 `stop()` 停桨）。
2. **后台监控线程（只读监控 + 单一兜底动作）**：独立线程周期检查：
   - 日志新鲜度：超过 `LOG_STALE_TIMEOUT_S`（建议默认 0.3s，同 t6）没收到新的 `aux_lg` 帧，判定链路/主控异常。
   - 总时长硬上限：超过 `MAX_FLIGHT_TIME_S`（建议默认 15.0s，覆盖起飞+悬停+降落全过程：`0.3m / 0.5m/s` 起降各约 0.6s + 3s 悬停，留足余量）。
   - 触发以上任一条件时，直接调用 `cf.high_level_commander.stop()`——这是固件里的立即停桨命令（`COMMAND_STOP`），不是斜坡下降。因为本次起飞高度只有 0.3m，直接停桨掉落的风险可接受；`PositionHlCommander` 本身不提供"从任意状态平滑降落"的原语，强行模拟斜坡反而会跟固件规划器已经在执行的轨迹冲突（`HighLevelCommander.go_to()` 文档里明确警告过"避免重叠的 go_to 命令"，`land()`/`takeoff()` 同理）。
   - 监控线程只做展示 + 这一个兜底动作，不做更复杂的重试/斜坡逻辑——复杂度留给以后如果这条路径证明可靠再迭代。
3. **状态打印**：监控线程里按 `STATUS_PRINT_PERIOD_S`（建议默认 0.3s，同 t6）节流打印 `range.zrange`/`stateEstimate.z`，跟 t6 的 `print_status` 一样，纯展示不参与控制。

## 内部架构

- `set_and_verify_param(cf, group, name, value, timeout_s)`：通用的"设置 param + 回调确认新值"辅助函数，`read_current_estimator()`（只读版本）和新增的 `commander.enHighLevel` 设置共用这个模式，避免重复。
- 后台监控线程封装成一个 `Watchdog` 类或简单函数 + `threading.Thread(daemon=True)`，构造时传入 `cf`、`state` 字典引用、`stop_event`，`main()` 退出前 `stop_event.set()` 并 `join()`。
- `main()` 结构：cflib 只在 `main()` 内 import（跟 t6 一致，模块本身不强依赖 cflib）→ 建链 → 起飞前检查 → 启动监控线程 → `with PositionHlCommander(...) as pc: time.sleep(HOVER_TIME_S)` → 停监控线程 → `close_link()`。
- `config.py`（`URI`、`connect_with_timeout`）不改动，直接复用。

## 验证方式

这是真实起降的飞控测试脚本，无法用单元测试验证飞行安全性。验证方式：
1. 静态可测的部分：`set_and_verify_param` 的参数校验逻辑（比如非法 group/name/超时行为）可以脱离飞机做单测，同 `test_t6_flight_sequence.py` 的思路。
2. 实机测试：室内、地面平整、四周留够 1m 净空、旁边有人随时准备断电/接住飞机。先确认 `commander.enHighLevel` 回读成功，再观察 `take_off()` 是否真的爬升到 0.3m（而不是静默停在地面——这正是背景里发现的静默失败模式），悬停 3s 是否稳定，`land()` 是否正常降落停桨。

## 重要提示（写入脚本头部注释）

本脚本验证的是固件里**从未做过飞行验证**的 `HighLevelCommander` 代码路径，跟 t5/t6 已经验证过的 `send_position_setpoint` 路径是两条独立的固件代码分支，t5/t6 的实测经验（触地判定阈值、看门狗节奏等）不能直接迁移过来当作"已验证"的保证。首次测试务必用最低高度、最短悬停时间，人员在旁随时准备断电。
