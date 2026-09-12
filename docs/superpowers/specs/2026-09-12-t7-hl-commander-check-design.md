# t7_hl_commander_check.py 设计文档

## 背景

`python/` 目录下现有的飞行脚本（t5_hover_land.py、t6_flight_sequence.py）全部走同一条路径：Python 侧用 `send_position_setpoint` 周期性发送 setpoint（20Hz），由固件 `position_controller_pid.c` 的位置外环闭环控制，t6 的设计文档里明确记录了"不使用固件的 `HighLevelCommander`（`crtp_commander_high_level.c`），因为这部分代码从未做过飞行验证"这一决定。

现在需要迈出这一步：验证 ESP-Drone 固件里编译进去的 `HighLevelCommander`（CRTP 端口 0x08）路径本身能不能正常起降，为以后是否要迁移到这条路径（用官方 `PositionHlCommander` 减少手写 ramp 代码）提供实测依据。

调研（对固件源码逐行核对，非二次转述）发现两类事实：

1. **命令兼容性**：`crtp_commander_high_level.c` 的命令分发里，`COMMAND_TAKEOFF_2`(7)、`COMMAND_LAND_2`(8)、`COMMAND_STOP`(3)、`COMMAND_GO_TO`(4，旧版) 均已实现；新版 `COMMAND_GO_TO_2`(12)/`COMMAND_START_TRAJECTORY_2` 等在这份固件的枚举里根本不存在。但固件上报的 `PROTOCOL_VERSION=4`（`config.h:46`），cflib 的 `HighLevelCommander.go_to()` 在协议版本 <8 时会自动选旧版 `COMMAND_GO_TO`，不会触发不存在的 `_2` 命令——本次测试不使用 `go_to`，这条只是排雷确认。
2. **必须的前提 param（关键新发现）**：`commander.c:48` 定义 `static bool enableHighLevel = false`，对外暴露为运行时 param `commander.enHighLevel`。`commanderGetSetpoint()`（`commander.c:104-116`）只有在这个 param 为真时才会把 setpoint 交给 `crtpCommanderHighLevelGetSetpoint()`；否则（包括默认状态）setpoint 恒为 `nullSetpoint`（停桨态）。也就是说**不显式把这个 param 设成 1，`take_off()`/`land()` 命令会被固件静默忽略，电机不会响应，Python 侧的 `time.sleep(duration_s)` 还是会正常走完**——这是一个静默失败模式，必须在起飞前设置并回读确认。
3. `commander.c:80-88`：任何一次低层 setpoint（`send_position_setpoint`）入队都会强制调用 `crtpCommanderHighLevelStop()`，把高层规划器打回 idle。因此本脚本必须是纯高层命令路径，不能像 t6 那样混用低层 setpoint 发送。
4. x/y 位置估计只有在 kalman 估计器下才是真实值；默认的 complementary 估计器下 `position_estimator_altitude.c` 把 x/y 硬编码为 0（`position_estimator_altitude.c:106-108`）。本脚本只测垂直的 `take_off`/`land`（不测 `go_to`），水平方向的风险等同于 t5/t6 已知的"非 kalman 时缓慢漂移"警告，不是新增风险。
5. **`land()` 在 `stop()` 之后不是无害的冗余动作（安全模型的修正，来自实现完成后的最终评审，最终评审时逐行核对了 `planner.c`）**：`planner.c` 的规划器状态机里，`plan_takeoff()`（`planner.c:138`）会检查 `state != TRAJECTORY_STATE_IDLE` 就拒绝执行，但 `plan_land()`（`planner.c:153`）只检查 `state == TRAJECTORY_STATE_LANDING` 才拒绝——从 `TRAJECTORY_STATE_IDLE`（也就是 `plan_stop()` 停桨后的状态，`planner.c:69-71`）调用 `land()` 完全会被接受，规划器会重新规划一条轨迹并把状态切回 `LANDING`（活跃状态）。`position_controller_pid.c` 的 `thrustBase`（注释明确写"should just lift the drone"）意味着这条重新激活的轨迹会让电机重新获得接近悬停的推力，不是"发了也没用的空指令"。**结论：任何已经调用过 `stop()` 的飞行阶段之后，绝对不能再无条件调用 `land()`。** 本文档最初的"安全模型"一节曾经错误地断言这种冗余调用无害，已在下面的安全模型一节纠正。

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

**修正（最终评审后）：不使用 `with PositionHlCommander(...)` 语法。** 最初设计用 `with` 让 `__exit__` 自动兜底调用 `land()`，但这意味着"任何时候退出 with 块都会再发一次 `land()`"，包括看门狗已经调用过 `stop()` 之后——而背景第 5 点已经证实这不是无害冗余，会重新让电机获得推力。改成显式调用，用 `stop_event` 门控是否还要发 `land()`：

```python
pc = PositionHlCommander(cf, default_height=TAKEOFF_HEIGHT_M,
                          default_velocity=DEFAULT_VELOCITY_MPS)
pc.take_off(velocity=TAKEOFF_VELOCITY_MPS)  # 相当于原来 __enter__
# ...悬停循环，看门狗可能在这期间 stop_event.set()...
if not stop_event.is_set():
    pc.land(velocity=LANDING_VELOCITY_MPS)  # 只有确实还没被看门狗停桨时才降落；已经 stop() 过就不再调 land()
```

- 起飞高度：`TAKEOFF_HEIGHT_M`，首次测试用 `0.3`（沿用 t5/t6 首次测试的保守高度），第三次实机测试后下调到 `0.15`（见下方"起飞高度/降落速度"条目）。
- **起飞速度（第二次实机测试后修正）**：原计划用 `PositionHlCommander` 默认的 `0.5 m/s` 不做调优，但第二次实机测试暴露这架机器动力余量不够在 `0.3m / 0.5m/s = 0.6s` 内跟上爬升指令——`take_off()` 返回时 zrange 几乎没有升高，thrust 却已冲到 38737，随后又花了近 3s thrust 才顶到 UINT16_MAX 附近（高度环因跟丢轨迹而积分饱和，把合力顶穿了电机 PWM 量程,在混控里挤占了姿态修正的推力余量,固件侧已加 `thrustMax` 90% 上限保护,见 `position_controller_pid.c`）。
- **起飞高度/降落速度（第三次实机测试后修正，已确认机体动力偏弱、约 60g）**：第三次测试暴露两个新问题——(1) 悬停期间 zrange 全程单调爬升，直到 3s 悬停窗口结束才刚摸到 0.3m 目标，真实爬升速度远跟不上指令,"悬停"实际全程都在爬升,从未真正稳定在目标高度；(2) `land()` 时飞机仍处于爬升过渡态（高度环还没收敛），而 `PositionHlCommander.land()` 是按 Python 侧假设的高度（不是实时测量值）除以速度算出固定降落时长，对这台动力紧张的机器来说下降窗口太短、速度太快，控制器来不及主动刹停，表现为从高空直接掉落。对应修正：`TAKEOFF_HEIGHT_M` 下调到 `0.15`（减少总爬升距离/时间需求）；新增 `LANDING_VELOCITY_MPS = 0.1`，`pc.land(velocity=LANDING_VELOCITY_MPS)` 单独放慢降落（不再用 `default_velocity` 的 `0.5 m/s`），拉长降落时长、留出刹停余量。
- **起飞时长拆分为死区+爬升（第四次实机测试后修正）**：逐帧看第三次测试的爬升阶段日志，前 1.2~1.5s 电机已经在加推力但 zrange 基本没有变化（螺旋桨/电机需要时间克服自身惯性和静摩擦才能进入有效爬升），只有最后一段时间才真正开始爬升。之前用单一 `TAKEOFF_VELOCITY_MPS` 常量按 `height / velocity` 算总时长，死区会直接从爬升预算里扣掉，导致留给真实爬升的时间比预期短得多。因此改成显式的两段时间相加：`TAKEOFF_SPOOLUP_TIME_S = 1.5`（死区，取自实测日志上限）+ `TAKEOFF_CLIMB_TIME_S = 2.0`（死区结束后留给真实爬升的时间），`TAKEOFF_VELOCITY_MPS = TAKEOFF_HEIGHT_M / (TAKEOFF_SPOOLUP_TIME_S + TAKEOFF_CLIMB_TIME_S)` 反过来换算成 `PositionHlCommander.take_off()` 唯一接受的 `velocity` 参数（该 API 没有直接传总时长的接口）。
- 悬停：`HOVER_TIME_S = 3.0`。
- **起飞后自动校验（新增，最终评审 Important #4）**：`take_off()` 返回后，无论电机实际有没有转，Python 侧都会正常往下走——这正是背景第 2 点"静默失败"的同一类症状。`take_off()` 返回后必须检查 `state['zrange_mm']`/`state['z_est']` 相对起飞前的 `zrange0` 确实发生了预期方向的变化（`range.zrange` 是下视测距，离地爬升时读数会**变大**，不是变小——同 t5/t6 对 `zrange` 的用法），不满足就打印警告（不需要因此中止降落流程，但必须让操作者知道"可能没有真的离地"）。

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

1. **正常/异常路径统一走"stop_event 门控的 land"，不是"with 自动 land"（最终评审后修正）**：不用 `with`，显式调用 `pc.take_off()`；悬停循环结束后，只有 `stop_event` 没被设置（即看门狗没有介入过）才调用 `pc.land()`——已经被看门狗 `stop()` 过的飞行阶段绝不再调 `land()`（背景第 5 点）。这个"检查 `stop_event` → 调用 `pc.land()`"之间仍有一个极窄的竞争窗口（`pc.land()` 不能加锁，见第 4 点），属于有意接受、已收窄（见下一条）的残余风险，不是这次修复要彻底消灭的目标。`emergency_stop()`（看门狗触发时调用）内部**先 `stop_event.set()` 再发 `stop()`**，不是反过来——这样主线程能尽早看到标志位，缩小上面那个窗口。紧急处理路径（第 2 点）额外用一个 `took_off` 标志门控，且这个标志不是"曾经调用过 `pc.take_off()`"，而是"现在飞机是不是应该被当成还在空中"：`pc.take_off()` 成功返回后置 `True`，`pc.land()` 成功返回后立刻清回 `False`（在任何后续可能抛异常的语句之前）。原因是同一类"重新上电"问题在两处都会踩到——`take_off()` 本身抛异常时飞机可能还停在 `IDLE`；`land()` 成功返回后，固件规划器已经自己从 `LANDING` 走回 `IDLE`（`plan_current_goal()` 在 `plan_is_finished()` 之后自动切回），这之后如果 `stop()` 发送或紧跟着的打印抛异常，`took_off` 必须已经是 `False`，否则紧急处理会把刚落地的飞机又送一次 `land()`，从 `IDLE` 重新给电机推力（背景第 5 点，最终评审第三轮发现的残余漏洞）。
2. **`__enter__`/`take_off()` 期间以及悬停期间的异常/Ctrl+C，统一走同一个 nested try/except 兜底（复用 t6 已验证的模式）**：
   ```python
   try:
       ...
   except (KeyboardInterrupt, Exception) as exc:
       print(f"触发紧急处理：{exc!r}", flush=True)  # 必须打印真实异常，不能只打印固定文案
       try:
           if took_off and not stop_event.is_set():
               with hl_lock:
                   cf.high_level_commander.land(LAND_HEIGHT_M, EMERGENCY_LAND_TIME_S)
               time.sleep(EMERGENCY_LAND_TIME_S)
       except (KeyboardInterrupt, Exception):
           pass  # land 本身失败也不再重入，直接走到下面的停桨（同 t6 的 stop_motors 兜底）
       with hl_lock:
           cf.high_level_commander.stop()
       stop_event.set()
   ```
   `stop()` 放在最外层、没有任何可能跳过它的路径——这是唯一保证"无论前面发生什么，最终都会停桨"的调用，同 t6 `stop_motors()` 放在 `finally`/兜底末尾的思路。
3. **后台监控线程（只读监控 + 单一兜底动作）**：独立线程周期检查：
   - 日志新鲜度：超过 `LOG_STALE_TIMEOUT_S`（建议默认 0.3s，同 t6）没收到新的 `aux_lg` 帧，判定链路/主控异常。
   - 总时长硬上限：超过 `MAX_FLIGHT_TIME_S`（建议默认 15.0s，覆盖起飞+悬停+降落全过程：`0.3m / 0.5m/s` 起降各约 0.6s + 3s 悬停，留足余量）。
   - 触发以上任一条件时，直接调用 `cf.high_level_commander.stop()`——这是固件里的立即停桨命令（`COMMAND_STOP`），不是斜坡下降。因为本次起飞高度只有 0.3m，直接停桨掉落的风险可接受；`PositionHlCommander` 本身不提供"从任意状态平滑降落"的原语，强行模拟斜坡反而会跟固件规划器已经在执行的轨迹冲突（`HighLevelCommander.go_to()` 文档里明确警告过"避免重叠的 go_to 命令"，`land()`/`takeoff()` 同理）。
   - 监控线程只做展示 + 这一个兜底动作，不做更复杂的重试/斜坡逻辑——复杂度留给以后如果这条路径证明可靠再迭代。
   - **触发后打印循环随即退出（`break`），不追求继续打印（澄清，最终评审 Minor #4）**：理想情况下紧急处理期间的遥测是事后排查最需要的数据，但要在不打乱 `stop_event` 语义的前提下让打印循环持续到紧急降落/停桨结束，需要额外一套跟 `stop_event` 解耦的计时器，本次先不做——现状是触发后立即停止打印，不是"应该继续打印却被遗漏了"，留给以后如果这条路径证明可靠、需要更细的事后复盘时再补。
4. **互斥锁的范围要限定在"实际发包"那一刻，不能把 `pc.take_off()`/`pc.land()` 整个调用都锁住（新增，最终评审 Important #5，实现时的重要澄清）**：`PositionHlCommander.take_off()`/`land()` 内部把"发包"和"`time.sleep(duration_s)`"揉在一次调用里——如果拿锁包住整个调用，看门狗线程在这几百毫秒的爬升/下降期间会因为抢不到锁而完全打不出 `stop()`，等于看门狗在恰恰最需要它介入的窗口失效。因此：
   - 看门狗的 `stop()` 和紧急处理路径里直接调用的 `cf.high_level_commander.land()`/`stop()`（这两处都是我们自己写的、瞬时返回的裸调用）用 `threading.Lock()` 互斥，看门狗触发前重新确认一次 `stop_event` 还没被设置。
   - `pc.take_off()`/`pc.land()` 本身不加锁——真正杜绝"`stop()` 之后又发 `land()`"这个危险场景的机制是 1 点里的 `stop_event` 门控（发 `land()` 前检查 `stop_event`），不是互斥锁；锁只是缩小"两个裸调用互相打断"这一更小的残余风险，不是本设计防止重新上电的主要手段。
5. **状态打印**：监控线程里按 `STATUS_PRINT_PERIOD_S`（建议默认 0.3s，同 t6）节流打印 `range.zrange`/`stateEstimate.z`，跟 t6 的 `print_status` 一样，纯展示不参与控制。

## 内部架构

- `set_and_verify_param(cf, group, name, value, timeout_s)`：通用的"设置 param + 回调确认新值"辅助函数，`read_current_estimator()`（只读版本）和新增的 `commander.enHighLevel` 设置共用这个模式，避免重复。
- 后台监控线程封装成一个 `Watchdog` 类或简单函数 + `threading.Thread(daemon=True)`，构造时传入 `cf`、`state` 字典引用、`stop_event`、包住 `high_level_commander` 调用的 `threading.Lock`，`main()` 退出前 `stop_event.set()` 并 `join()`。
- `main()` 结构：cflib 只在 `main()` 内 import（跟 t6 一致，模块本身不强依赖 cflib）→ 建链 → 起飞前检查 → 启动监控线程 → 显式 `pc.take_off()`（不用 `with`）→ 起飞后校验 zrange/z_est 变化 → 悬停循环 → `stop_event` 没被设置才 `pc.land()` → 停监控线程 → `close_link()`。异常路径见"安全模型"第 2 点的 nested try/except。
- `config.py`（`URI`、`connect_with_timeout`）不改动，直接复用。

## 验证方式

这是真实起降的飞控测试脚本，无法用单元测试验证飞行安全性。验证方式：
1. 静态可测的部分：`set_and_verify_param` 的参数校验逻辑（比如非法 group/name/超时行为）可以脱离飞机做单测，同 `test_t6_flight_sequence.py` 的思路。
2. 实机测试：室内、地面平整、四周留够 1m 净空、旁边有人随时准备断电/接住飞机。先确认 `commander.enHighLevel` 回读成功，再观察起飞后自动校验是否确认了 zrange/z_est 的变化（而不是静默停在地面——这正是背景里发现的静默失败模式），悬停 3s 是否稳定，`land()` 是否正常降落停桨。

## 重要提示（写入脚本头部注释）

本脚本验证的是固件里**从未做过飞行验证**的 `HighLevelCommander` 代码路径，跟 t5/t6 已经验证过的 `send_position_setpoint` 路径是两条独立的固件代码分支，t5/t6 的实测经验（触地判定阈值、看门狗节奏等）不能直接迁移过来当作"已验证"的保证。首次测试务必用最低高度、最短悬停时间，人员在旁随时准备断电。
