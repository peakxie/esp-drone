#!/usr/bin/env python3
# T6：多命令飞行序列执行器——参考 t5_hover_land.py 已验证过的安全模型（20Hz 发送、日志新鲜度
# 看门狗、硬性总时长上限、Ctrl+C/看门狗触发走斜坡降落、range.zrange 触地确认），改成可以在
# 一次连接内按顺序执行任意多条命令（takeoff/hover/goto/land），而不是每种测试单独写一个脚本。
#
# 命令列表在下面的 FLIGHT_PLAN 里声明，每条是 (命令名, *参数) 元组：
#   ("takeoff", height_m[, duration_s])   起飞爬升到绝对高度 height_m，x/y 钉在起飞点
#   ("hover", duration_s)                 保持当前目标不变，悬停 duration_s
#   ("goto", dx, dy, h, duration_s)       目标线性斜坡过渡到 (起飞点+dx, 起飞点+dy, 绝对高度h)
#                                          —— dx=dy=0 时就是单纯改变高度（"定高"）
#   ("land"[, duration_s])                斜坡降到地面 + range.zrange 触地确认 + 停桨
#
# yaw 全程固定为起飞时的 yaw0，不作为参数暴露——这个项目还没有验证过 yaw 控制。
# 不使用固件的 HighLevelCommander（crtp_commander_high_level.c）：固件确实编译了这部分代码，
# 但从未做过飞行验证，继续沿用 t5 已验证的 send_position_setpoint 路线。
#
# 安全约定（同 t5_hover_land.py）：
#   - 全程以 SEND_PERIOD 周期性发送 setpoint，避免 commander watchdog 超时进入 fallback。
#   - 用高度日志做"链路存活"看门狗：超过 LOG_STALE_TIMEOUT_S 没收到新帧，立即认为链路/主控
#     异常，跳过剩余命令直接执行紧急下降。
#   - Ctrl+C 不会瞬间切电机——从当前目标做一次快速但连续的下降斜坡再停桨。
#   - 全程有一个 MAX_FLIGHT_TIME_S 硬上限（覆盖整个命令列表，不是每条命令单独计时），超时
#     无条件进入紧急下降。
#   - 命令列表如果没有以 land 结尾，执行完所有命令后自动补一次完整 land，不允许悬在半空
#     直接结束脚本。
#   - 连接前对命令列表做纯本地静态校验（命令名/参数个数/取值范围），不合法直接拒绝执行，
#     不会尝试连接/起飞。
#
# 首次测试建议：室内、地面平整、四周留够 1m 净空、旁边有人随时准备断电。
#
# 重要：本脚本本身尚未做过实机飞行验证（对应实现计划里的 Task 3 尚未执行）。上面列的安全
# 约定是从 t5_hover_land.py 继承并泛化的，那些具体机制（看门狗、触地判定、紧急下降节奏）
# 已经过 t5 的多次实机验证，但 t6 把它们接到新代码路径上这件事本身还没有飞过。
# 另外，goto 命令里的非零 dx/dy（水平方向移动）是本项目第一次真正尝试主动的水平位置指令
# ——t5_hover_land.py 出于保守考虑，全程把 x/y 钉死在起飞点不变，从未真正指挥飞机水平移动。
# 首次测试请先用只含 takeoff/hover/land、不含 goto 的最小 FLIGHT_PLAN（同下面 Task 3 Step 1
# 的建议），确认基本行为正常后再逐步加入 goto。

import time

from config import URI, connect_with_timeout

ESTIMATOR_NAMES = {0: "any", 1: "complementary", 2: "kalman"}

LIFTOFF_HEIGHT_M = 0.03    # 起飞斜坡的起点目标高度，避免第一帧就是 0（等同于"还没起飞"）
LAND_HEIGHT_M = 0.0        # 降落斜坡的终点目标高度

TAKEOFF_TIME_S = 2.0       # takeoff 命令未指定 duration_s 时的默认值
LAND_TIME_S = 2.0          # land 命令未指定 duration_s 时的默认值

TOUCHDOWN_ZRANGE_MM = 90   # 判定"已经进入地面效应气垫、可以停桨"的测距阈值（同 t5 的实测结论）
TOUCHDOWN_CONFIRM_S = 0.3  # 连续这么久测距都在阈值以下，才认为已经稳定卡进气垫
TOUCHDOWN_MAX_WAIT_S = 5.0 # 兜底上限：一直没等到"确认落地"也最多等这么久就强制停桨
CLOSE_TO_GROUND_LOG_ZRANGE_MM = 150  # 进入这个高度以内，print_status 不再受节流限制，每帧打印

SEND_PERIOD = 0.05         # 20Hz 发送 setpoint
STATUS_PRINT_PERIOD_S = 0.3
EMERGENCY_LAND_TIME_S = 1.0

LOG_WAIT_TIMEOUT_S = 2.0
LOG_STALE_TIMEOUT_S = 0.3
MAX_FLIGHT_TIME_S = 30.0   # 覆盖整个 FLIGHT_PLAN 执行期的硬上限，按实际命令列表总时长调整

RANGE_SANE_MAX_MM = 4000   # 不设下限，理由同 t5：VL53L1X 贴近量程下限读数本身偏随机，不是故障

VX_KI_OVERRIDE = 2.0
VY_KI_OVERRIDE = 2.0

MAX_XY_OFFSET_M = 1.0      # goto 的 dx/dy 绝对值上限，防止参数手误导致意外大位移
MAX_HEIGHT_M = 1.0         # takeoff/goto 的高度上限，防止参数手误导致意外高高度

# 命令名 -> (最少参数个数, 最多参数个数)
COMMAND_ARG_SPECS = {
    "takeoff": (1, 2),  # (height_m,) 或 (height_m, duration_s)
    "hover": (1, 1),    # (duration_s,)
    "goto": (4, 4),     # (dx, dy, h, duration_s)
    "land": (0, 1),     # () 或 (duration_s,)
}

# 默认示例飞行计划：起飞到 0.5m -> 悬停 3s -> 前移 0.3m 同时保持 0.5m -> 悬停 2s -> 降到 0.3m -> 降落
FLIGHT_PLAN = [
    ("takeoff", 0.5),
    ("hover", 3.0),
    ("goto", 0.3, 0.0, 0.5, 2.0),
    ("hover", 2.0),
    ("goto", 0.0, 0.0, 0.3, 1.5),
    ("land",),
]


class FlightAbort(Exception):
    """内部信号：立即停止当前阶段，转入紧急下降。"""


def validate_flight_plan(plan, max_xy_offset_m=MAX_XY_OFFSET_M, max_height_m=MAX_HEIGHT_M, max_flight_time_s=MAX_FLIGHT_TIME_S):
    """对命令列表做连接前的纯本地校验（不依赖飞机/cflib），返回错误信息列表；
    空列表表示合法。第一条命令必须是 takeoff，否则后续命令的前提条件不成立。
    除了逐条检查参数是否合法，还检查整体"形状"：只能有一次 takeoff（在第一条）、
    land 只能出现一次且必须是最后一条、命令列表的最坏情况总耗时不能超过
    MAX_FLIGHT_TIME_S——这三条都是运行时 main() 隐含假设的前提条件，必须在
    连接飞机之前就拒绝违反它们的计划，而不是等到飞到一半才出问题。"""
    errors = []
    if not plan:
        errors.append("命令列表为空，至少需要一条 takeoff 命令。")
        return errors

    first_entry = plan[0]
    first_name = first_entry[0] if isinstance(first_entry, tuple) and len(first_entry) > 0 else None
    if first_name != "takeoff":
        errors.append(f"第一条命令必须是 takeoff，实际是 {first_entry!r}。")

    takeoff_count = 0
    land_indices = []
    total_duration_s = 0.0

    for idx, entry in enumerate(plan):
        if not isinstance(entry, tuple) or len(entry) == 0:
            errors.append(f"第 {idx} 条命令格式不对（应为非空元组）：{entry!r}")
            continue
        name = entry[0]
        args = entry[1:]
        if name not in COMMAND_ARG_SPECS:
            errors.append(f"第 {idx} 条命令名未知：{name!r}（合法命令：{sorted(COMMAND_ARG_SPECS)}）")
            continue
        min_args, max_args = COMMAND_ARG_SPECS[name]
        if not (min_args <= len(args) <= max_args):
            errors.append(
                f"第 {idx} 条命令 {name!r} 参数个数为 {len(args)}，应在 [{min_args}, {max_args}] 之间：{entry!r}"
            )
            continue

        if name == "takeoff":
            takeoff_count += 1
            height_m = args[0]
            if not (0.0 < height_m <= max_height_m):
                errors.append(f"第 {idx} 条 takeoff 高度 {height_m} 超出合理范围 (0, {max_height_m}]m：{entry!r}")
            duration_s = args[1] if len(args) == 2 else TAKEOFF_TIME_S
            if duration_s <= 0:
                errors.append(f"第 {idx} 条 takeoff duration_s={duration_s} 必须 > 0：{entry!r}")
            total_duration_s += duration_s
        elif name == "goto":
            dx, dy, h, duration_s = args
            if abs(dx) > max_xy_offset_m or abs(dy) > max_xy_offset_m:
                errors.append(f"第 {idx} 条 goto 偏移 dx={dx} dy={dy} 超出 ±{max_xy_offset_m}m 范围：{entry!r}")
            if not (0.0 <= h <= max_height_m):
                errors.append(f"第 {idx} 条 goto 高度 h={h} 超出 [0, {max_height_m}]m 范围：{entry!r}")
            if duration_s <= 0:
                errors.append(f"第 {idx} 条 goto duration_s={duration_s} 必须 > 0：{entry!r}")
            total_duration_s += duration_s
        elif name == "hover":
            (duration_s,) = args
            if duration_s <= 0:
                errors.append(f"第 {idx} 条 hover duration_s={duration_s} 必须 > 0：{entry!r}")
            total_duration_s += duration_s
        elif name == "land":
            land_indices.append(idx)
            duration_s = args[0] if args else LAND_TIME_S
            if duration_s <= 0:
                errors.append(f"第 {idx} 条 land duration_s={duration_s} 必须 > 0：{entry!r}")
            total_duration_s += duration_s + TOUCHDOWN_MAX_WAIT_S

    if takeoff_count > 1:
        errors.append(f"命令列表里出现了 {takeoff_count} 次 takeoff，只能有一次（且必须是第一条）：{plan!r}")

    if len(land_indices) > 1:
        errors.append(f"命令列表里出现了 {len(land_indices)} 次 land，最多只能有一次，且必须是最后一条：{plan!r}")
    elif land_indices and land_indices[-1] != len(plan) - 1:
        errors.append(f"land 命令（第 {land_indices[-1]} 条）后面还有其他命令，land 只能是最后一条：{plan!r}")

    if not land_indices:
        total_duration_s += LAND_TIME_S + TOUCHDOWN_MAX_WAIT_S  # 没写 land 会被 main() 自动补一次，也要算进预算

    if total_duration_s > max_flight_time_s:
        errors.append(
            f"命令列表预计总耗时 {total_duration_s:.1f}s（含 land 触地确认的最坏情况等待）超过 "
            f"max_flight_time_s={max_flight_time_s:.1f}s，请精简命令或调大 MAX_FLIGHT_TIME_S：{plan!r}"
        )

    return errors


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
    # cflib 只在这里 import：让本模块（尤其是 validate_flight_plan）在没有装 cflib 的机器上
    # 也能被 import 和单测，只有真正执行 main() 飞行时才需要 cflib。
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig

    errors = validate_flight_plan(FLIGHT_PLAN)
    if errors:
        print("命令列表校验失败，拒绝执行：", flush=True)
        for err in errors:
            print(f"  - {err}", flush=True)
        return

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

        print(f"起飞前地面测距 = {zrange0}mm，遥测正常，开始执行 {len(FLIGHT_PLAN)} 条命令。", flush=True)

        flight_deadline = time.monotonic() + MAX_FLIGHT_TIME_S
        current_target = {"dx": 0.0, "dy": 0.0, "h": 0.0}
        last_status_print = [0.0]

        def watchdog_ok():
            if time.monotonic() > flight_deadline:
                print("警告：飞行总时长超过硬上限，强制转入紧急下降。", flush=True)
                return False
            if state["last_log_t"] is None or (time.monotonic() - state["last_log_t"]) > LOG_STALE_TIMEOUT_S:
                print("警告：高度日志已停止刷新，链路/主控可能异常，强制转入紧急下降。", flush=True)
                return False
            return True

        def print_status(force=False):
            now = time.monotonic()
            if not force and now - last_status_print[0] < STATUS_PRINT_PERIOD_S:
                return
            last_status_print[0] = now
            x0, y0 = state["x0"], state["y0"]
            dx = (state["x_est"] - x0) if (x0 is not None and state["x_est"] is not None) else None
            dy = (state["y_est"] - y0) if (y0 is not None and state["y_est"] is not None) else None
            print(
                f"    target=(dx={current_target['dx']:.2f}, dy={current_target['dy']:.2f}, "
                f"h={current_target['h']:.2f})  z={state['z_est']}  zrange={state['zrange_mm']}mm  "
                f"thrust={state['thrust_est']}  x={state['x_est']}(drift={dx})  y={state['y_est']}(drift={dy})",
                flush=True,
            )

        def send_target():
            cf.commander.send_position_setpoint(
                state["x0"] + current_target["dx"],
                state["y0"] + current_target["dy"],
                current_target["h"],
                state["yaw0"],
            )

        def ramp_to(dx, dy, h, duration_s):
            """把目标从 current_target 的当前值线性斜坡过渡到 (dx, dy, h)，全程以
            SEND_PERIOD 周期发送 send_position_setpoint。takeoff/goto/land/紧急下降
            全部复用这一个函数（同 t5_hover_land.py 的 send_ramp，泛化到三个轴）。"""
            start_dx, start_dy, start_h = current_target["dx"], current_target["dy"], current_target["h"]
            t0 = time.monotonic()
            while True:
                now = time.monotonic()
                elapsed = now - t0
                frac = min(1.0, elapsed / duration_s) if duration_s > 0 else 1.0
                current_target["dx"] = start_dx + (dx - start_dx) * frac
                current_target["dy"] = start_dy + (dy - start_dy) * frac
                current_target["h"] = start_h + (h - start_h) * frac

                send_target()
                print_status()

                if not watchdog_ok():
                    raise FlightAbort()

                if frac >= 1.0:
                    break
                time.sleep(SEND_PERIOD)

        def hold(duration_s):
            t0 = time.monotonic()
            while time.monotonic() - t0 < duration_s:
                send_target()
                print_status()
                if not watchdog_ok():
                    raise FlightAbort()
                time.sleep(SEND_PERIOD)

        def wait_for_touchdown(max_wait_s):
            """持续发送 LAND_HEIGHT_M 目标，用 range.zrange（比融合后的 stateEstimate.z 少一层
            滤波延迟）判断是否已经稳定卡进地面效应气垫：连续 TOUCHDOWN_CONFIRM_S 都低于
            TOUCHDOWN_ZRANGE_MM 才认为可以停桨（同 t5_hover_land.py 的实测结论）。"""
            current_target["h"] = LAND_HEIGHT_M
            t0 = time.monotonic()
            below_since = None
            while time.monotonic() - t0 < max_wait_s:
                send_target()
                zrange = state["zrange_mm"]
                close_to_ground = zrange is not None and zrange <= CLOSE_TO_GROUND_LOG_ZRANGE_MM
                print_status(force=close_to_ground)
                if not watchdog_ok():
                    raise FlightAbort()

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

        def stop_motors():
            print("停桨。", flush=True)
            for _ in range(15):
                cf.commander.send_stop_setpoint()
                time.sleep(0.02)

        def cmd_takeoff(height_m, duration_s=TAKEOFF_TIME_S):
            print(f"命令 takeoff：爬升到 {height_m:.2f}m（{duration_s:.1f}s）...", flush=True)
            current_target["dx"] = 0.0
            current_target["dy"] = 0.0
            current_target["h"] = LIFTOFF_HEIGHT_M
            ramp_to(0.0, 0.0, height_m, duration_s)

        def cmd_hover(duration_s):
            print(f"命令 hover：悬停 {duration_s:.1f}s...", flush=True)
            hold(duration_s)

        def cmd_goto(dx, dy, h, duration_s):
            print(f"命令 goto：过渡到 dx={dx:.2f} dy={dy:.2f} h={h:.2f}（{duration_s:.1f}s）...", flush=True)
            ramp_to(dx, dy, h, duration_s)

        def cmd_land(duration_s=LAND_TIME_S):
            print(f"命令 land：降落（{duration_s:.1f}s）...", flush=True)
            ramp_to(current_target["dx"], current_target["dy"], LAND_HEIGHT_M, duration_s)
            wait_for_touchdown(TOUCHDOWN_MAX_WAIT_S)
            stop_motors()

        command_handlers = {
            "takeoff": cmd_takeoff,
            "hover": cmd_hover,
            "goto": cmd_goto,
            "land": cmd_land,
        }

        executed_land = False
        try:
            for name, *args in FLIGHT_PLAN:
                command_handlers[name](*args)
                if name == "land":
                    executed_land = True

            if not executed_land:
                print("命令列表未以 land 结尾，自动补一次降落。", flush=True)
                cmd_land()

        except (KeyboardInterrupt, Exception):
            print(
                f"触发紧急下降：从当前目标 dx={current_target['dx']:.2f} dy={current_target['dy']:.2f} "
                f"h={current_target['h']:.2f} 快速降到地面（{EMERGENCY_LAND_TIME_S:.1f}s）...",
                flush=True,
            )
            # 紧急下降给自己单独续一段时间预算：如果刚才是 MAX_FLIGHT_TIME_S 到期触发的中止，
            # flight_deadline 是一个已经过去的固定时间点，不重新往后推的话 watchdog_ok() 在紧急
            # 斜坡的第一帧就会再次判定超时，导致下面的 ramp_to 立刻又被打断——紧急下降会退化
            # 成跟直接停桨一样的瞬间掉高度，而不是期望中的连续斜坡下降。
            flight_deadline = time.monotonic() + EMERGENCY_LAND_TIME_S
            try:
                ramp_to(current_target["dx"], current_target["dy"], LAND_HEIGHT_M, EMERGENCY_LAND_TIME_S)
            except (KeyboardInterrupt, Exception):
                pass  # 已经在尽力下降了，任何异常都不再重入，直接走到下面的停桨
            stop_motors()

        print("飞行结束。", flush=True)
    finally:
        aux_lg.stop()
        cf.close_link()


if __name__ == "__main__":
    main()
