#!/usr/bin/env python3
# T7：验证固件的 HighLevelCommander（CRTP 端口 0x08，crtp_commander_high_level.c）能否正常起降。
# t5_hover_land.py / t6_flight_sequence.py 走的是另一条固件路径——Python 侧用 send_position_setpoint
# 周期发送 setpoint，固件 position_controller_pid.c 的位置外环闭环控制，这条路径已经过多次实机验证。
# HighLevelCommander 是完全不同的固件代码分支：固件确实编译了 crtp_commander_high_level.c
# （takeoff2/land2/stop/go_to 均已实现，调研见 docs/superpowers/specs/2026-09-12-t7-hl-commander-
# check-design.md），但从未做过飞行验证。本脚本就是第一次验证：用官方 cflib.positioning.
# position_hl_commander.PositionHlCommander 做一次最小的起飞→悬停→降落，不测水平移动（go_to），
# 把风险面压到最低。
#
# 起飞前必须确认的固件前提（否则起飞命令会被静默忽略，电机不会响应）：
#   commander.c 里 enableHighLevel 默认 false，commanderGetSetpoint() 只有这个 param 为真时才会把
#   setpoint 交给高层规划器。本脚本起飞前会显式设置 commander.enHighLevel=1 并回读确认，失败则
#   拒绝起飞（不会尝试 take_off）。
#
# 本脚本必须是纯高层命令路径：commander.c 里任何一次低层 send_position_setpoint 都会把高层规划器
# 强制打回 idle（crtpCommanderHighLevelStop()），所以本脚本全程不调用 cf.commander.send_*，只用
# cf.high_level_commander / PositionHlCommander。
#
# 安全模型（跟 t5/t6 的"每帧插断斜坡"本质不同——PositionHlCommander.take_off()/land() 是阻塞调用，
# 轨迹插值完全在固件规划器里完成，Python 侧没有逐帧介入点）：
#   - 不使用 with PositionHlCommander(...) 语法：固件的 planner.c 里 plan_land() 只拒绝已经在
#     LANDING 状态的重入，不拒绝从 IDLE（也就是 stop() 之后）重新进入——也就是说 stop() 之后
#     再调一次 land() 不是无害的冗余动作，会让电机重新获得接近悬停的推力。用 with 语法会导致
#     __exit__ 在看门狗已经 stop() 过之后还无条件再调一次 land()，正好踩中这个坑。改成显式调用
#     pc.take_off()/pc.land()，land() 前先检查 stop_event 有没有被看门狗设置过，设置过就跳过。
#   - take_off()/悬停期间发生的 Ctrl+C/异常，统一用 try/except 兜底：打印真实异常
#     （{exc!r}，不是固定文案），如果 took_off 仍为 True（飞机被认为还在空中）且 stop_event
#     还没被设置就尝试一次 land()+sleep——只看 stop_event 不够：take_off() 本身抛异常时飞机
#     可能还在 IDLE，land() 从 IDLE 一样会被 planner.c 接受并重新给电机推力；pc.land() 成功
#     返回之后 took_off 也会被清回 False，理由相同（详见 took_off 定义处的注释）。
#     无论是否成功，最后都无条件调一次 stop()（同 t6 的 stop_motors 兜底思路——stop() 是
#     全程唯一保证"最终一定会停桨"的调用，前面任何步骤失败都不能跳过它）。
#   - 后台看门狗线程：日志新鲜度（LOG_STALE_TIMEOUT_S）+ 总时长硬上限（MAX_FLIGHT_TIME_S），
#     触发时直接调用 cf.high_level_commander.stop()（立即停桨，不是斜坡——PositionHlCommander
#     不提供"从任意状态平滑降落"的原语，且 HighLevelCommander.go_to()/land() 的文档明确警告过
#     不要叠加/打断正在执行的轨迹）。起飞高度只有 0.3m，直接停桨掉落的风险可接受。
#   - hl_lock 只包住看门狗和异常兜底里直接发的 cf.high_level_commander.stop()/land() 裸调用，
#     不包住 pc.take_off()/pc.land()（也不包住任何 time.sleep()）——这两个调用内部把"发包"
#     和"sleep(duration_s)"揉在一起，锁住整个调用会让看门狗在起降的几百毫秒里完全打不出
#     stop()，等于看门狗在最需要它的窗口失效。真正防止"stop() 之后又发 land()"的机制是
#     stop_event 门控，不是这个锁；锁只是缩小两个裸调用互相打断的更小残余风险。
#
# 已知行为差异（跟 t5/t6 不同，不是 bug）：cflib 的 HighLevelCommander.takeoff()/land() 默认把
# yaw 目标钉在绝对 0 弧度（yaw=0.0, useCurrentYaw=False），而不是像 t5/t6 那样保持起飞时的
# yaw0。PositionHlCommander.take_off()/land() 没有暴露 yaw 参数，无法覆盖这一行为。如果飞机
# 通电时朝向不在 0 弧度附近，起飞过程中可能会看到轻微自转——这是预期行为，不代表异常，靠
# print_status 里的 yaw 读数确认即可。
#
# 首次测试建议：室内、地面平整、四周留够 1m 净空、旁边有人随时准备断电/接住飞机。

import threading
import time

from config import URI, connect_with_timeout

ESTIMATOR_NAMES = {0: "any", 1: "complementary", 2: "kalman"}

TAKEOFF_HEIGHT_M = 0.15        # 起飞绝对高度（第三次实机测试后从 0.3 下调，见下方注释）
DEFAULT_VELOCITY_MPS = 0.5     # PositionHlCommander 构造时的默认速度；land() 显式传参覆盖，见下

# 第三次实机测试（已确认动力偏弱，机体约 60g）暴露两个问题：
#   1. 悬停期间 zrange 全程单调爬升、直到 3s 悬停窗口结束才刚摸到 0.3m 目标——真实爬升
#      速度远跟不上指令，"悬停"实际上全程都在爬升，从未真正稳定在目标高度。把目标高度
#      降到 0.15m（TAKEOFF_HEIGHT_M，见上），减少总爬升距离/时间需求。
#   2. land() 时飞机仍处于爬升过渡态（高度环还没收敛），PositionHlCommander.land() 又是
#      按 Python 侧假设的高度（不是实时测量值）/ 默认 0.5m/s 算出固定降落时长，对这台
#      动力紧张的机器来说降落窗口太短、下降速度太快，表现为控制器来不及主动刹停、直接
#      掉了下来。降落单独给一个远低于默认值的速度，拉长降落时长，留出刹停余量。
LANDING_VELOCITY_MPS = 0.1

# 第二次实机测试暴露的问题：take_off() 用默认 0.5m/s 算出来的爬升时长只有 0.6s
# （height / velocity）。这架机器动力余量不够在 0.6s 内跟上这个高度指令，高度环因跟丢
# 目标而把合力顶到 UINT16_MAX 附近，进而在混控里挤占了姿态修正的推力余量（固件侧已经
# 加了 thrustMax=90% 上限保护，见 position_controller_pid.c）。单独放慢起飞速度、拉长
# 爬升时间，给动力和控制器留出跟踪轨迹的余量。
#
# 第四次实机测试证伪了上面这个"死区"猜想，并结合固件代码定位到真正的原因，见下：
# 把 TAKEOFF_HEIGHT_M 下调到 0.15 之后，本来想按"死区 1.5s + 真实爬升 2.0s"给起飞更长的
# 总时长（3.5s），结果整个 3.5s 起飞窗口里 zrange 完全没有离开地面噪声范围，take_off()
# 返回时反而比起飞前更低；一旦 take_off() 返回、目标高度不再按轨迹爬升而是定死在 0.15m，
# zrange 立刻在同样的 thrust 水平上开始稳定爬升。这说明问题不是"电机需要更多时间/时长
# 不够"，而是反过来——时长越长，规划轨迹每一时刻的目标位置离飞机当前实际位置就越近，
# 喂给高度环的位置误差就越小，控制器算出来的爬升速度指令也越小，thrust 顶不上去；只有
# take_off() 结束、目标位置固定不再前移之后，误差才积累到足够大，PID 才真正发力爬升。
# 固件证据（position_controller_pid.c:212-214 的 positionController()）：
#   if (setpoint->mode.z == modeAbs) {
#     setpoint->velocity.z = runPid(state->position.z, &this.pidZ, setpoint->position.z, DT);
#   }
# 高层规划器（crtp_commander_high_level.c:311-331）算出的速度前馈 ev.vel.z 会被这一行
# 覆盖掉，z 方向完全靠"当前高度 vs 规划器给的目标高度"过一个纯位置误差 PID（kp=1.6,
# ki=0.5），规划器算好的"这一时刻该多快"完全没用上——轨迹越平缓（时长越长），这个瞬时
# 误差就越小，PID 越不会使劲。因此撤回上面的死区拆分，起飞速度改回第三次测试里验证过、
# 表现明显更好的固定值（0.3m 目标高度、2.0s 时长时起飞阶段本身就能看到 23mm->61mm 的
# 爬升，比这次 3.5s 版本好得多）。
TAKEOFF_VELOCITY_MPS = 0.15

# 第四次实机测试还发现：固件默认的 posCtlPid.thrustBase（高度环前馈基准，见
# position_controller_pid.c 里"thrustBase should just lift the drone"的注释，本来就是留给
# "更重机身/更旧电池"调的参数）明显低于这架机器实测的真实爬升推力（日志里稳定爬升时
# thrust 落在 45000~55000 区间）。基准值偏低意味着控制器每次都要靠位置误差慢慢积分才能
# 顶到有效推力，起飞响应更慢、更依赖 hover 阶段的误差累积。这里在起飞前把它临时调高，
# 跟 commander.enHighLevel 一样用 set_and_verify_param 设置+回读确认；这是运行时 PARAM，
# 断电重启会恢复固件编译进去的默认值，不是永久改动。设置失败只降级为警告（不像
# enHighLevel 失败那样直接拒绝起飞）——thrustBase 调不上去，飞机大概率还是能飞，只是
# 响应更慢，不是电机完全不响应的静默失败模式。
THRUST_BASE_OVERRIDE = 45000   # 取实测爬升区间（45000~55000）的下限，留出 PID 向上调的余量

# 悬停不再是固定睡 HOVER_SETTLE_MAX_WAIT_S 秒后无条件降落——第三次测试暴露过 land() 时
# 飞机仍处于爬升过渡态、高度环还没收敛就被打断降落，表现为直接掉落。改成轮询 zrange，
# 进入目标高度容差范围并稳定 HOVER_SETTLE_DWELL_S 才认为真的悬停住了，最长等待
# HOVER_SETTLE_MAX_WAIT_S 兜底（超时也会继续走降落，只是打印警告，不无限等下去）。
HOVER_SETTLE_TOLERANCE_MM = 30  # 目标高度容差
HOVER_SETTLE_DWELL_S = 0.5      # 进入容差范围后要持续这么久才算稳定，避免单帧噪声凑巧命中
HOVER_SETTLE_MAX_WAIT_S = 6.0   # 等待收敛的最长时间

LOG_WAIT_TIMEOUT_S = 2.0       # 起飞前等待第一帧遥测的超时
LOG_STALE_TIMEOUT_S = 0.3      # 看门狗：日志新鲜度阈值
MAX_FLIGHT_TIME_S = 15.0       # 看门狗：覆盖起飞+悬停+降落全程的硬上限
STATUS_PRINT_PERIOD_S = 0.3    # 状态打印节流周期
PARAM_SET_TIMEOUT_S = 2.0      # commander.enHighLevel 设置后回读确认的超时
EMERGENCY_LAND_TIME_S = 1.0    # __enter__（take_off）期间异常时，兜底 land 的时长
LAND_HEIGHT_M = 0.0

RANGE_SANE_MAX_MM = 4000       # 起飞前地面测距合理性上限，不设下限（同 t5/t6）
MIN_TAKEOFF_ZRANGE_RISE_MM = 100  # 起飞后 zrange 至少要比起飞前升高这么多，才认为真的离地了


def read_current_estimator(cf, timeout_s=2.0):
    """读取 stabilizer.estimator 参数，返回当前值（1=complementary, 2=kalman），超时返回 None。"""
    got_value = threading.Event()
    holder = {"value": None}

    def estimator_cb(_name, value):
        holder["value"] = int(value)
        got_value.set()

    cf.param.add_update_callback(group="stabilizer", name="estimator", cb=estimator_cb)
    cf.param.request_param_update("stabilizer.estimator")
    got_value.wait(timeout=timeout_s)

    if holder["value"] is None:
        print("警告：读取 stabilizer.estimator 超时，未确认当前估计器。", flush=True)
    else:
        label = ESTIMATOR_NAMES.get(holder["value"], "unknown")
        print(f"当前 stabilizer.estimator = {holder['value']} ({label})", flush=True)

    return holder["value"]


def set_and_verify_param(cf, group, name, value, timeout_s=PARAM_SET_TIMEOUT_S):
    """设置 group.name = value，然后通过回调等待固件确认新值生效。成功返回 True；
    回读超时或者回读到的值跟期望值不一致都返回 False（调用方应据此拒绝起飞，而不是
    假设设置一定成功——commander.enHighLevel 这种"设置失败=静默忽略"的 param 尤其需要
    这一步，否则 take_off() 会看起来正常执行但固件其实完全没反应）。"""
    full_name = f"{group}.{name}"
    got_value = threading.Event()
    holder = {"value": None}

    def value_cb(_name, new_value):
        holder["value"] = new_value
        got_value.set()

    cf.param.add_update_callback(group=group, name=name, cb=value_cb)
    cf.param.set_value(full_name, str(value))
    got_value.wait(timeout=timeout_s)

    if not got_value.is_set():
        print(f"警告：设置 {full_name}={value} 后回读超时（{timeout_s:.1f}s 内没有收到确认）。", flush=True)
        return False

    if str(holder["value"]) != str(value):
        print(f"警告：设置 {full_name}={value} 后回读到的值是 {holder['value']!r}，与期望值不一致。", flush=True)
        return False

    print(f"已确认 {full_name} = {holder['value']}。", flush=True)
    return True


def main():
    # cflib 只在这里 import：让本模块的纯逻辑函数（set_and_verify_param、read_current_estimator）
    # 在没有装 cflib 的机器上也能被 import 和单测。
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.positioning.position_hl_commander import PositionHlCommander

    cflib.crtp.init_drivers()

    cf = Crazyflie()
    if not connect_with_timeout(cf, URI):
        return

    state = {
        "zrange_mm": None,
        "last_log_t": None,
        "z_est": None,
        "x_est": None,
        "y_est": None,
        "yaw_est": None,
        "thrust_est": None,
    }

    def aux_cb(_timestamp, data, _logconf):
        state["zrange_mm"] = data["range.zrange"]
        state["z_est"] = data["stateEstimate.z"]
        state["x_est"] = data["stateEstimate.x"]
        state["y_est"] = data["stateEstimate.y"]
        state["yaw_est"] = data["stabilizer.yaw"]
        state["thrust_est"] = data["stabilizer.thrust"]
        state["last_log_t"] = time.monotonic()

    aux_lg = LogConfig(name="aux", period_in_ms=50)
    aux_lg.add_variable("range.zrange", "uint16_t")
    aux_lg.add_variable("stateEstimate.z", "float")
    aux_lg.add_variable("stateEstimate.x", "float")
    aux_lg.add_variable("stateEstimate.y", "float")
    aux_lg.add_variable("stabilizer.yaw", "float")
    aux_lg.add_variable("stabilizer.thrust", "float")
    cf.log.add_config(aux_lg)
    aux_lg.data_received_cb.add_callback(aux_cb)
    aux_lg.start()

    try:
        estimator = read_current_estimator(cf)
        if estimator != 2:
            print(
                "警告：当前不是 kalman 估计器——没有检测到光流 deck，或者 "
                "CONFIG_SENSORS_ENABLE_DECK 没有开。本次只测垂直起降，没有水平位置修正时"
                "飞机可能会缓慢漂移，请留意周围净空。",
                flush=True,
            )

        deadline = time.monotonic() + LOG_WAIT_TIMEOUT_S
        while state["last_log_t"] is None and time.monotonic() < deadline:
            time.sleep(0.02)
        if state["last_log_t"] is None:
            print("错误：等不到 range.zrange/stateEstimate.z 日志，遥测未连通，放弃起飞。", flush=True)
            return

        zrange0 = state["zrange_mm"]
        if zrange0 is None or zrange0 > RANGE_SANE_MAX_MM:
            print(
                f"错误：起飞前 range.zrange={zrange0}mm 超出合理范围（上限 {RANGE_SANE_MAX_MM}mm），"
                "怀疑测距传感器读数异常，放弃起飞。"
                " 请确认飞机放在平整地面、传感器朝下且未被遮挡。",
                flush=True,
            )
            return

        if not set_and_verify_param(cf, "commander", "enHighLevel", 1, timeout_s=PARAM_SET_TIMEOUT_S):
            print(
                "错误：commander.enHighLevel 设置/回读失败，拒绝起飞——高层命令固件端不会被"
                "处理（commander.c 的 commanderGetSetpoint() 只有这个 param 为真时才会把 setpoint "
                "交给高层规划器）。",
                flush=True,
            )
            return

        if not set_and_verify_param(cf, "posCtlPid", "thrustBase", THRUST_BASE_OVERRIDE, timeout_s=PARAM_SET_TIMEOUT_S):
            print(
                "警告：posCtlPid.thrustBase 设置/回读失败，继续用固件编译进去的默认值起飞——"
                "这个值大概率明显低于这架机器的真实悬停推力，预期起飞/爬升响应会更慢，"
                "不是电机完全不响应的静默失败模式，不因此中止起飞。",
                flush=True,
            )

        print(f"起飞前地面测距 = {zrange0}mm，commander.enHighLevel 已确认，开始起飞。", flush=True)

        hl_lock = threading.Lock()
        stop_event = threading.Event()
        flight_deadline = time.monotonic() + MAX_FLIGHT_TIME_S
        last_status_print = [0.0]

        def emergency_stop(reason):
            print(reason, flush=True)
            # 先设 stop_event 再发 stop()：如果反过来，主线程可能在 stop() 已经发出、
            # stop_event 还没置位的这一瞬间读到"未触发"，进而调用 pc.land()——land() 在
            # stop() 之后会让电机重新获得推力（见下面 land 分支的注释）。先置位能让主线程
            # 尽早看到，缩小这个窗口（pc.land() 本身不能加锁，见下方注释，所以这个窗口没法
            # 完全消除，只能尽量缩小）。
            stop_event.set()
            with hl_lock:
                cf.high_level_commander.stop()

        def watchdog_loop():
            while not stop_event.is_set():
                now = time.monotonic()
                if now > flight_deadline:
                    emergency_stop("警告：飞行总时长超过硬上限，触发立即停桨。")
                    break
                if state["last_log_t"] is None or (now - state["last_log_t"]) > LOG_STALE_TIMEOUT_S:
                    emergency_stop("警告：高度日志已停止刷新，链路/主控可能异常，触发立即停桨。")
                    break
                if now - last_status_print[0] >= STATUS_PRINT_PERIOD_S:
                    last_status_print[0] = now
                    print(
                        f"    z={state['z_est']}  zrange={state['zrange_mm']}mm  "
                        f"thrust={state['thrust_est']}  x={state['x_est']}  y={state['y_est']}  "
                        f"yaw={state['yaw_est']}",
                        flush=True,
                    )
                time.sleep(0.05)

        watchdog_thread = threading.Thread(target=watchdog_loop, daemon=True)
        watchdog_thread.start()

        # took_off 定义在 try 之外：即使 PositionHlCommander(...) 构造或 take_off() 本身抛异常，
        # except 分支也能安全读到这个变量。它标记的是"现在飞机是不是应该被当成在空中"，不是
        # "曾经起飞过"——pc.land() 成功返回、飞机已经落地之后会被清回 False，见下面。
        took_off = False
        try:
            pc = PositionHlCommander(
                cf,
                default_height=TAKEOFF_HEIGHT_M,
                default_velocity=DEFAULT_VELOCITY_MPS,
            )
            pc.take_off(velocity=TAKEOFF_VELOCITY_MPS)
            took_off = True

            zrange_after_takeoff = state["zrange_mm"]
            if (
                zrange_after_takeoff is None
                or zrange0 is None
                or zrange_after_takeoff < zrange0 + MIN_TAKEOFF_ZRANGE_RISE_MM
            ):
                print(
                    f"警告：起飞前后 zrange {zrange0}mm -> {zrange_after_takeoff}mm，没有观察到"
                    f"预期的升高（预期至少上升 {MIN_TAKEOFF_ZRANGE_RISE_MM}mm）。飞机可能没有真的"
                    "起飞（比如 commander.enHighLevel 没有真正生效，这正是本脚本要抓的静默失败"
                    "模式）。仍会继续走完悬停/降落流程，请现场确认飞机状态。",
                    flush=True,
                )
            else:
                print(f"已确认离地：zrange {zrange0}mm -> {zrange_after_takeoff}mm。", flush=True)

            target_zrange_mm = zrange0 + int(round(TAKEOFF_HEIGHT_M * 1000))
            print(
                f"悬停：等待 zrange 稳定进入目标 {target_zrange_mm}mm 附近（容差 "
                f"±{HOVER_SETTLE_TOLERANCE_MM}mm），最长等待 {HOVER_SETTLE_MAX_WAIT_S:.1f}s...",
                flush=True,
            )
            t0 = time.monotonic()
            settled_since = None
            while time.monotonic() - t0 < HOVER_SETTLE_MAX_WAIT_S and not stop_event.is_set():
                zr = state["zrange_mm"]
                if zr is not None and abs(zr - target_zrange_mm) <= HOVER_SETTLE_TOLERANCE_MM:
                    if settled_since is None:
                        settled_since = time.monotonic()
                    elif time.monotonic() - settled_since >= HOVER_SETTLE_DWELL_S:
                        print(f"已稳定在目标高度附近（zrange={zr}mm），结束悬停等待。", flush=True)
                        break
                else:
                    settled_since = None
                time.sleep(0.1)
            else:
                if not stop_event.is_set():
                    print(
                        f"警告：等待 {HOVER_SETTLE_MAX_WAIT_S:.1f}s 后仍未稳定在目标高度附近"
                        f"（当前 zrange={state['zrange_mm']}mm，目标 {target_zrange_mm}mm），"
                        "仍会继续走降落流程——land() 是按假设已到达目标高度算的固定降落时长，"
                        "如果实际还没到/还在剧烈变化，这个假设就是错的，请现场确认飞机状态。",
                        flush=True,
                    )

            if stop_event.is_set():
                print(
                    "看门狗已经触发过停桨，跳过 land()——在这份固件上，land() 在 stop() 之后"
                    "不是无害的冗余动作，会让电机重新获得接近悬停的推力（planner.c 的 "
                    "plan_land() 只拒绝已经在 LANDING 状态的重入，不拒绝从 IDLE 重新进入）。",
                    flush=True,
                )
            else:
                # pc.land() 内部把发包和 sleep(duration_s) 揉在一起，不能像裸调用那样用
                # hl_lock 包住（否则看门狗在整条降落斜坡期间都打不出 stop()）。这意味着从
                # 上面的 stop_event 检查到这里调用 pc.land()，仍有一个极窄的竞争窗口——
                # 看门狗可能恰好在这一瞬间触发。这是有意接受的、经过收窄（emergency_stop
                # 先置位再发包）的残余风险，不是遗漏。
                print("命令 land：降落...", flush=True)
                pc.land(velocity=LANDING_VELOCITY_MPS)
                # pc.land() 内部 sleep(duration_s) 已经等完，此刻固件的规划器已经自己从
                # LANDING 走回 IDLE（planner.c 的 plan_current_goal() 在 plan_is_finished()
                # 之后自动切回 IDLE）。所以从这一行起，"起飞过"这个状态已经结束——如果接下来
                # 的 stop() 发送或下面的 print 抛异常（比如链路在这时断开），下面 except 分支
                # 绝不能再把这当成"飞机还在空中，需要紧急 land()"，否则又是从 IDLE 重新
                # 上电的同一个问题。必须在这里立刻清掉 took_off，不能等到 finally。
                took_off = False
                with hl_lock:
                    cf.high_level_commander.stop()  # land() 之后再补一次 stop() 总是安全的
                print("飞行结束（已降落）。", flush=True)
        except (KeyboardInterrupt, Exception) as exc:
            print(f"触发紧急处理：{exc!r}", flush=True)
            try:
                # 只有 took_off 仍为 True（飞机被认为还在空中）且看门狗没有停桨过，才尝试
                # land()——take_off() 本身抛异常时飞机可能还在 IDLE 状态，land() 从 IDLE 一样
                # 会被 planner.c 接受并重新给电机推力，跟"stop() 之后再 land()"是同一类问题；
                # pc.land() 成功返回之后 took_off 也会被清回 False（见上面），同一个理由。
                if took_off and not stop_event.is_set():
                    with hl_lock:
                        cf.high_level_commander.land(LAND_HEIGHT_M, EMERGENCY_LAND_TIME_S)
                    time.sleep(EMERGENCY_LAND_TIME_S)
            except (KeyboardInterrupt, Exception):
                pass  # land 本身失败也不再重入，直接走到下面的停桨
            with hl_lock:
                cf.high_level_commander.stop()
            stop_event.set()
        finally:
            stop_event.set()
            watchdog_thread.join(timeout=1.0)

    finally:
        aux_lg.stop()
        cf.close_link()


if __name__ == "__main__":
    main()
