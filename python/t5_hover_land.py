#!/usr/bin/env python3
# T5：第一次真正离地的自动化测试——起飞爬升到 50cm，定高悬停 1s，然后自动降落。
#
# 和 t4* 系列的区别：t4b/t4c/t4d/t4e 全部是"桨叶已拆、机身固定在支架上"的台架测试，
# 从来没有真正离地飞行过。这个脚本是第一次真正起飞，所以：
#   1. 用官方高层 setpoint（send_hover_setpoint，对应固件 crtp_commander_generic.c 里的
#      hoverType：body 坐标系 vx/vy + 世界坐标系绝对高度 zDistance），把爬升/悬停/下降的
#      速度和高度环全部交给固件的 position_controller_pid.c 闭环处理，脚本只管给"目标值"，
#      不直接发姿态/推力（跟 t4b/t4c 反过来）。
#   2. 定高悬停依赖 range.zrange（VL53L1X）喂给状态估计器；需要 CONFIG_SENSORS_ENABLE_DECK=y
#      让光流/测距 deck 生效——光流一在线，sensors_mpu6050_hm5883L_ms5611.c 里
#      flowdeck2Test()==true 会自动调用 setCommandermode(POSHOLD_MODE)，把估计器切到
#      kalman（见 estimator.c 的 registerRequiredEstimator）。如果没检测到光流 deck，
#      下面会打印警告但仍然尝试起飞——此时只有测距/气压的定高，没有水平位置修正，
#      不能指望它能稳稳停在原地不漂。
#   3. 起飞前必须先确认 range.zrange 有正常读数（几十~几百 mm 量级），否则说明测距没连上，
#      绝不能盲目起飞——没有可靠高度反馈的"起飞"等于盲降油门，是最容易炸机的场景。
#
# 安全约定（延续 t4c/t4e）：
#   - 全程以 SEND_PERIOD 周期性发送 setpoint，避免 commander watchdog（COMMANDER_WDT_TIMEOUT_*）
#     超时进入 fallback。
#   - 用高度日志做"链路存活"看门狗：超过 LOG_STALE_TIMEOUT_S 没收到新帧，立即认为链路/主控
#     异常，跳过剩余阶段直接执行紧急下降。
#   - Ctrl+C 不会瞬间切电机——会从当前高度做一次快速但连续的下降斜坡再停桨，避免半空自由落体。
#   - 全程有一个 MAX_FLIGHT_TIME_S 硬上限，超时无条件进入紧急下降，防止任何逻辑错误导致
#     "悬停指令一直发、飞机一直不降落"。
#
# 首次测试建议：室内、地面平整、四周留够 1m 净空、旁边有人随时准备断电，禁螺旋桨保护罩可选但推荐。
#
# 2026-09 更新：t5b_zdistance_hold.py（开环，roll/pitch 强制 0）比这里（闭环）漂得更快更狠，
# 证实水平方向存在一个持续偏置，之前用的 send_hover_setpoint 只有速度闭环（vx=vy=0），没有
# 位置误差反馈——速度纠正到 0 之后，之前已经漂掉的位置是回不来的。这次改成
# send_position_setpoint(x0, y0, target_h, yaw0)，把 x/y/yaw 全程钉在起飞时的绝对坐标，让
# position_controller_pid.c 里本来就存在、但这个脚本一直没用到的位置外环（pidX/pidY，
# kp=1.9, ki=0.1）参与闭环，主动纠正累积位置误差，不只是纠正速度。
# 注意：这条 mode.x/y=modeAbs 路径在这个项目里是第一次真正启用飞行验证过，建议第一次测试时
# TARGET_HEIGHT_M 用较低值、HOVER_TIME_S 缩短，并确保离墙/障碍物净空比之前更大（>2m）。
# 同时把 velCtlPid.vxKi/vyKi 从默认 1.0 临时调大（运行时 param，断电重启会恢复默认），
# 让内层速度环更快压制恒定偏置，跟外层位置环配合使用。
#
# 2026-09 更新：实测发现即使 wait_for_touchdown() 判定触地后才停桨，落地前最后还是有一小段
# "直接掉下去"的感觉。根源不在触地判定，而在判定完之后那一步——之前是直接调 15 次
# send_stop_setpoint()，油门瞬间归零；但那一刻 controller_pid.c 算出来的 control.thrust
# （对应日志 stabilizer.thrust）大概率还不是 0，瞬间砍掉推力就是那一小段自由落体的来源。
# 现在改成：停桨前先跑一段 ramp_down_thrust()，以当时实测的 stabilizer.thrust 为起点，用
# 原始 setpoint（cf.commander.send_setpoint(0,0,0,thrust)，同 t4b/t4c 用的接口）线性斜坡
# 降到 0，而不是直接猜一个悬停推力去斜坡——这台机还没做过真正的悬停推力标定，猜的数不可信。

import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

from config import URI, connect_with_timeout

ESTIMATOR_NAMES = {0: "any", 1: "complementary", 2: "kalman"}

TARGET_HEIGHT_M = 0.50     # 目标悬停高度
LIFTOFF_HEIGHT_M = 0.03    # 起飞斜坡的起点目标高度，避免第一帧就是 0（等同于"还没起飞"）
LAND_HEIGHT_M = 0.0        # 降落斜坡的终点目标高度

TAKEOFF_TIME_S = 2.0       # 0.03m -> 0.50m 爬升斜坡时长
HOVER_TIME_S = 1.0         # 到达目标高度后悬停时长（题目要求：1s）
LAND_TIME_S = 2.0          # 0.50m -> 0m 下降斜坡时长
TOUCHDOWN_ZRANGE_MM = 60   # 判定"已经落地"的测距阈值：起飞前贴地读数是几十mm量级（比如18mm），
                           # 留出余量给 VL53L1X 近距噪声，避免刚好卡在临界值附近反复跳变
TOUCHDOWN_CONFIRM_S = 0.15 # 连续这么久测距都在阈值以下，才真的认为落地（防抖，防止噪声偶尔
                           # 单帧扫过阈值就误判）
TOUCHDOWN_MAX_WAIT_S = 5.0 # 兜底上限：一直没等到"确认落地"（比如测距异常）也最多等这么久
                           # 就强制停桨，不能无限悬停耗光电量
                           # 2026-09 诊断性临时调大（原 2.0）：t5 实测发现降落末段 z 卡在离地
                           # 6~9cm 反复横跳、2s 内从未跌破 60mm 阈值，不确定是"还没等到"还是
                           # "真收敛在一个非零稳态"，先给够时间看曲线走向，定位完再改回合理值

SEND_PERIOD = 0.05         # 20Hz 发送 hover setpoint，同 t4c/t4e
STATUS_PRINT_PERIOD_S = 0.3    # 飞行全程打印一次 x/y/z 轨迹的间隔，方便复盘水平漂移
EMERGENCY_LAND_TIME_S = 1.0    # Ctrl+C/看门狗触发时，从当前高度紧急下降到 0 的时长（比正常降落更快）

LOG_WAIT_TIMEOUT_S = 2.0   # 等待第一帧高度日志的超时：等不到就说明遥测没通，绝不能盲目起飞
LOG_STALE_TIMEOUT_S = 0.3  # 运行中超过这么久没收到新日志帧，视为链路/主控可能已经异常
MAX_FLIGHT_TIME_S = 15.0   # 从起飞指令发出到必须已经落地停桨的硬上限（保险丝，不依赖任何传感器判断）

RANGE_SANE_MAX_MM = 4000   # zrange 合理范围上限：VL53L1X 有效量程外/悬空无回波，说明没测到地面
# 注意：不设下限。VL53L1X 在贴近量程下限（几十 mm 以内）本身读数就偏随机（sensor 物理特性，
# 不是故障），实测起飞前贴地就见过 18mm 这种正常但偏低的读数——用一个固定下限去卡它，卡掉的
# 是正常起飞，不是真的故障；真正的"没测到地面/被挡住"用上限（读数异常大或超出量程）就够判断了。

VX_KI_OVERRIDE = 2.0   # velCtlPid.vxKi 默认 1.0，先保守翻倍——一次调太猛容易在闭环里振荡
VY_KI_OVERRIDE = 2.0   # velCtlPid.vyKi 默认 1.0，同上

THRUST_RAMP_DOWN_TIME_S = 0.3  # 停桨前，从实测 thrust 线性斜坡降到 0 的时长
THRUST_RAMP_START_CAP = 50000  # 起点推力的 sanity 上限（满量程 65535），只是防止读到的那一帧
                                # 恰好是脏数据/尖峰，不是悬停推力估计值——这台机还没标定过真实
                                # 悬停推力，斜坡起点必须用当时实测值，不能瞎猜一个数


class FlightAbort(Exception):
    """内部信号：立即停止当前阶段，转入紧急下降。"""


def read_current_estimator(cf, timeout_s=2.0):
    """读取 stabilizer.estimator 参数，返回当前值（1=complementary, 2=kalman），超时返回 None。"""
    import threading

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


def main():
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
        "x0": None,
        "y0": None,
        "yaw0": None,
    }

    def aux_cb(_timestamp, data, _logconf):
        state["zrange_mm"] = data["range.zrange"]
        state["z_est"] = data["stateEstimate.z"]
        state["x_est"] = data["stateEstimate.x"]
        state["y_est"] = data["stateEstimate.y"]
        state["yaw_est"] = data["stabilizer.yaw"]
        state["thrust_est"] = data["stabilizer.thrust"]
        if state["x0"] is None:
            state["x0"] = state["x_est"]
            state["y0"] = state["y_est"]
            state["yaw0"] = state["yaw_est"]
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
                "CONFIG_SENSORS_ENABLE_DECK 没有开。定高仍会依赖测距/气压工作，但没有水平位置"
                "修正，飞机可能会缓慢漂移，请留意周围净空。",
                flush=True,
            )

        cf.param.set_value("velCtlPid.vxKi", str(VX_KI_OVERRIDE))
        cf.param.set_value("velCtlPid.vyKi", str(VY_KI_OVERRIDE))
        time.sleep(0.2)
        print(
            f"已将 velCtlPid.vxKi/vyKi 覆盖为 {VX_KI_OVERRIDE}/{VY_KI_OVERRIDE}（默认 1.0，"
            "断电重启会恢复默认）。",
            flush=True,
        )

        # 等第一帧高度日志，确认遥测通了，绝不能在没有高度反馈的情况下起飞
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

        print(f"起飞前地面测距 = {zrange0}mm，遥测正常，开始起飞。", flush=True)

        flight_deadline = time.monotonic() + MAX_FLIGHT_TIME_S
        current_target_h = [0.0]  # 用 list 装着，方便在闭包/异常处理里读取"最后一次发出的目标高度"

        def watchdog_ok():
            if time.monotonic() > flight_deadline:
                print("警告：飞行总时长超过硬上限，强制转入紧急下降。", flush=True)
                return False
            if state["last_log_t"] is None or (time.monotonic() - state["last_log_t"]) > LOG_STALE_TIMEOUT_S:
                print("警告：高度日志已停止刷新，链路/主控可能异常，强制转入紧急下降。", flush=True)
                return False
            return True

        last_status_print = [0.0]

        def print_status(target_h):
            now = time.monotonic()
            if now - last_status_print[0] < STATUS_PRINT_PERIOD_S:
                return
            last_status_print[0] = now
            x0, y0 = state["x0"], state["y0"]
            dx = (state["x_est"] - x0) if (x0 is not None and state["x_est"] is not None) else None
            dy = (state["y_est"] - y0) if (y0 is not None and state["y_est"] is not None) else None
            print(
                f"    target_h={target_h:.2f}m  z={state['z_est']}  zrange={state['zrange_mm']}mm  "
                f"thrust={state['thrust_est']}  "
                f"x={state['x_est']}(drift={dx})  y={state['y_est']}(drift={dy})",
                flush=True,
            )

        def send_ramp(start_h, end_h, duration_s):
            """线性斜坡把 zDistance 从 start_h 发到 end_h，x/y/yaw 全程钉在起飞时的绝对坐标不变
            （modeAbs 位置闭环，不是速度闭环），交给 position_controller_pid.c 的 pidX/pidY 主动
            纠正累积位置误差。"""
            t0 = time.monotonic()
            while True:
                now = time.monotonic()
                elapsed = now - t0
                frac = min(1.0, elapsed / duration_s) if duration_s > 0 else 1.0
                target_h = start_h + (end_h - start_h) * frac
                current_target_h[0] = target_h

                cf.commander.send_position_setpoint(state["x0"], state["y0"], target_h, state["yaw0"])
                print_status(target_h)

                if not watchdog_ok():
                    raise FlightAbort()

                if frac >= 1.0:
                    break
                time.sleep(SEND_PERIOD)

        def hold(height_m, duration_s):
            t0 = time.monotonic()
            while time.monotonic() - t0 < duration_s:
                current_target_h[0] = height_m
                cf.commander.send_position_setpoint(state["x0"], state["y0"], height_m, state["yaw0"])
                print_status(height_m)
                if not watchdog_ok():
                    raise FlightAbort()
                time.sleep(SEND_PERIOD)

        def wait_for_touchdown(max_wait_s):
            """下降斜坡结束时目标高度已经到 0，但实测 stateEstimate.z 有 0.1~0.2m 量级的滞后
            （position_controller_pid.c 闭环响应 + EKF 滤波延迟），如果按固定时长悬停后就
            无条件停桨，飞机往往还悬在半空几厘米到十几厘米——send_stop_setpoint 对应固件
            crtp_commander_generic.c 里的 stopType，直接把 setpoint 清零、推力瞬间归零，
            半空停桨就是自由落体摔下去，不是"落地"。这里改成持续发 LAND_HEIGHT_M 位置指令，
            用最直接的地面测距 range.zrange（比融合后的 stateEstimate.z 少一层滤波延迟）
            判断是否真的贴地：连续 TOUCHDOWN_CONFIRM_S 都低于 TOUCHDOWN_ZRANGE_MM 才认为
            落地。max_wait_s 是兜底上限，避免测距异常时无限悬停耗电。"""
            t0 = time.monotonic()
            below_since = None
            while time.monotonic() - t0 < max_wait_s:
                current_target_h[0] = LAND_HEIGHT_M
                cf.commander.send_position_setpoint(state["x0"], state["y0"], LAND_HEIGHT_M, state["yaw0"])
                print_status(LAND_HEIGHT_M)
                if not watchdog_ok():
                    raise FlightAbort()

                zrange = state["zrange_mm"]
                now = time.monotonic()
                if zrange is not None and zrange <= TOUCHDOWN_ZRANGE_MM:
                    if below_since is None:
                        below_since = now
                    elif now - below_since >= TOUCHDOWN_CONFIRM_S:
                        return
                else:
                    below_since = None
                time.sleep(SEND_PERIOD)

            print(f"警告：等待确认落地超过 {max_wait_s:.1f}s 上限，强制停桨。", flush=True)

        def ramp_down_thrust(duration_s):
            """停桨前的最后一段：把原始 thrust setpoint 从当时实测的 stabilizer.thrust
            线性斜坡降到 0（roll/pitch/yawrate=0，同 t4b/t4c 用的 cf.commander.send_setpoint
            接口），代替之前"直接调 send_stop_setpoint 瞬间归零"——那一刻控制器算出来的推力
            大概率还不是 0，瞬间砍掉就是最后一小段自由落体的来源。这台机没标定过真实悬停推力，
            起点必须用实测值，不能瞎猜。"""
            start_thrust = state["thrust_est"]
            if start_thrust is None or start_thrust <= 0:
                print("警告：未获取到有效的 stabilizer.thrust 读数，跳过推力斜坡，直接停桨。", flush=True)
                return
            start_thrust = min(start_thrust, THRUST_RAMP_START_CAP)
            print(f"推力斜坡：从当前 thrust≈{start_thrust:.0f} 平滑降到 0（{duration_s:.1f}s）...", flush=True)
            t0 = time.monotonic()
            try:
                while True:
                    elapsed = time.monotonic() - t0
                    frac = min(1.0, elapsed / duration_s) if duration_s > 0 else 1.0
                    thrust = start_thrust * (1.0 - frac)
                    cf.commander.send_setpoint(0, 0, 0, int(thrust))
                    if frac >= 1.0:
                        break
                    time.sleep(SEND_PERIOD)
            except KeyboardInterrupt:
                pass  # 已经在斜坡降推力了，再按一次 Ctrl+C 就直接跳到下面的硬停桨

        try:
            print(f"阶段1/3：起飞爬升到 {TARGET_HEIGHT_M:.2f}m（{TAKEOFF_TIME_S:.1f}s）...", flush=True)
            send_ramp(LIFTOFF_HEIGHT_M, TARGET_HEIGHT_M, TAKEOFF_TIME_S)

            print(f"阶段2/3：定高悬停 {HOVER_TIME_S:.1f}s（当前 stateEstimate.z={state['z_est']}）...", flush=True)
            hold(TARGET_HEIGHT_M, HOVER_TIME_S)

            print(f"阶段3/3：降落到地面（{LAND_TIME_S:.1f}s）...", flush=True)
            send_ramp(TARGET_HEIGHT_M, LAND_HEIGHT_M, LAND_TIME_S)
            wait_for_touchdown(TOUCHDOWN_MAX_WAIT_S)

        except (FlightAbort, KeyboardInterrupt):
            print(
                f"触发紧急下降：从当前目标高度 {current_target_h[0]:.2f}m 快速降到地面"
                f"（{EMERGENCY_LAND_TIME_S:.1f}s）...",
                flush=True,
            )
            try:
                send_ramp(current_target_h[0], LAND_HEIGHT_M, EMERGENCY_LAND_TIME_S)
            except (FlightAbort, KeyboardInterrupt):
                pass  # 已经在尽力下降了，watchdog 再触发也不再重入，直接走到下面的停桨

        ramp_down_thrust(THRUST_RAMP_DOWN_TIME_S)

        print("停桨。", flush=True)
        for _ in range(15):
            cf.commander.send_stop_setpoint()
            time.sleep(0.02)

        print("飞行结束。", flush=True)
    finally:
        aux_lg.stop()
        cf.close_link()


if __name__ == "__main__":
    main()
