#!/usr/bin/env python3
# T5b：隔离测试——只测"定高环+纯水平配平"，不让光流的水平速度闭环参与，用来判断
# t5_hover_land.py 里观察到的"悬停时一直往左偏"到底是光流方向/融合有问题，还是机身配平/
# 机械不对称本身就会漂（跟光流无关）。
#
# 原理（读 controller_pid.c 得到的结论，不是猜的）：
#   固件的水平控制分两层——position_controller_pid.c 的 positionController()/
#   velocityController() 是"闭环"层，会用 state->velocity（EKF 融合光流后的水平速度估计）
#   去纠正 roll/pitch；但 controller_pid.c 第 84~90 行：
#       if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
#         attitudeDesired.roll = setpoint->attitude.roll;
#         attitudeDesired.pitch = setpoint->attitude.pitch;
#       }
#   只要 setpoint->mode.x 或 mode.y 是 modeDisable（stabilizer_types.h 里 modeDisable=0，
#   也是 setpoint 结构体清零后的默认值），闭环层算出来的 roll/pitch 会被直接丢弃，改用
#   setpoint 里原始的 attitude.roll/pitch。
#   cflib 的 send_zdistance_setpoint(roll, pitch, yawrate, zdistance) 对应固件
#   crtp_commander_generic.c 里的 zDistanceType：这个包只设置 mode.z=modeAbs（走闭环定高）
#   和 mode.roll/pitch=modeAbs（把 roll/pitch 直接设成传进来的值），从来没碰 mode.x/mode.y，
#   它们保持默认的 modeDisable。也就是说发 send_zdistance_setpoint(0,0,0,h) 会得到：
#     - z 方向：闭环定高，跟 t5 一样用 range.zrange/EKF 的高度估计。
#     - x/y 方向：完全开环——roll/pitch 恒为 0（水平配平），光流算出来的水平速度闭环修正
#       会被算出来但直接丢弃，根本不会送到电机。
#
# 怎么解读结果：
#   - 如果这次依然稳定地往左漂、漂移速率跟 t5 差不多：说明跟光流没关系，是机身配平/
#     螺旋桨/电机不对称等机械因素——去测光流没用，该去查配平（PITCH_CALIB/ROLL_CALIB）
#     或者检查桨叶/电机是否对称。
#   - 如果这次基本不漂（或者漂移明显更小、方向不固定）：说明 t5 里的左偏主要来自闭环层，
#     大概率就是光流方向/EKF 融合的符号问题——回去用更明确的方式核对
#     flowdeck_v1v2.c 里 dpixelx=-deltaY, dpixely=-deltaX 这次翻转是否符合这块板子
#     PMW3901 的真实贴装方向。
#
# 安全约定跟 t5_hover_land.py 完全一致：20Hz 发 setpoint、日志新鲜度看门狗、硬性总时长上限、
# Ctrl+C/看门狗触发走紧急下降斜坡而不是瞬间切电机。

import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

from config import URI, connect_with_timeout

ESTIMATOR_NAMES = {0: "any", 1: "complementary", 2: "kalman"}

TARGET_HEIGHT_M = 0.50
LIFTOFF_HEIGHT_M = 0.03
LAND_HEIGHT_M = 0.0

TAKEOFF_TIME_S = 2.0
HOVER_TIME_S = 1.0
LAND_TIME_S = 2.0
TOUCHDOWN_SETTLE_S = 0.5

SEND_PERIOD = 0.05
STATUS_PRINT_PERIOD_S = 0.3
EMERGENCY_LAND_TIME_S = 1.0

LOG_WAIT_TIMEOUT_S = 2.0
LOG_STALE_TIMEOUT_S = 0.3
MAX_FLIGHT_TIME_S = 15.0

RANGE_SANE_MIN_MM = 20
RANGE_SANE_MAX_MM = 4000


class FlightAbort(Exception):
    """内部信号：立即停止当前阶段，转入紧急下降。"""


def read_current_estimator(cf, timeout_s=2.0):
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
        "x0": None,
        "y0": None,
    }

    def aux_cb(_timestamp, data, _logconf):
        state["zrange_mm"] = data["range.zrange"]
        state["z_est"] = data["stateEstimate.z"]
        state["x_est"] = data["stateEstimate.x"]
        state["y_est"] = data["stateEstimate.y"]
        if state["x0"] is None:
            state["x0"] = state["x_est"]
            state["y0"] = state["y_est"]
        state["last_log_t"] = time.monotonic()

    aux_lg = LogConfig(name="aux", period_in_ms=50)
    aux_lg.add_variable("range.zrange", "uint16_t")
    aux_lg.add_variable("stateEstimate.z", "float")
    aux_lg.add_variable("stateEstimate.x", "float")
    aux_lg.add_variable("stateEstimate.y", "float")
    cf.log.add_config(aux_lg)
    aux_lg.data_received_cb.add_callback(aux_cb)
    aux_lg.start()

    try:
        read_current_estimator(cf)
        print(
            "本脚本用 send_zdistance_setpoint（roll=pitch=0），水平方向是开环配平，"
            "不吃光流闭环修正——只用来对照 t5_hover_land.py 的漂移是否来自光流闭环。",
            flush=True,
        )

        deadline = time.monotonic() + LOG_WAIT_TIMEOUT_S
        while state["last_log_t"] is None and time.monotonic() < deadline:
            time.sleep(0.02)
        if state["last_log_t"] is None:
            print("错误：等不到 range.zrange/stateEstimate 日志，遥测未连通，放弃起飞。", flush=True)
            return

        zrange0 = state["zrange_mm"]
        if zrange0 is None or not (RANGE_SANE_MIN_MM <= zrange0 <= RANGE_SANE_MAX_MM):
            print(
                f"错误：起飞前 range.zrange={zrange0}mm 超出合理范围"
                f"[{RANGE_SANE_MIN_MM},{RANGE_SANE_MAX_MM}]mm，放弃起飞。",
                flush=True,
            )
            return

        print(f"起飞前地面测距 = {zrange0}mm，遥测正常，开始起飞。", flush=True)

        flight_deadline = time.monotonic() + MAX_FLIGHT_TIME_S
        current_target_h = [0.0]
        last_status_print = [0.0]

        def watchdog_ok():
            if time.monotonic() > flight_deadline:
                print("警告：飞行总时长超过硬上限，强制转入紧急下降。", flush=True)
                return False
            if state["last_log_t"] is None or (time.monotonic() - state["last_log_t"]) > LOG_STALE_TIMEOUT_S:
                print("警告：高度日志已停止刷新，链路/主控可能异常，强制转入紧急下降。", flush=True)
                return False
            return True

        def print_status(target_h):
            now = time.monotonic()
            if now - last_status_print[0] < STATUS_PRINT_PERIOD_S:
                return
            last_status_print[0] = now
            x0, y0 = state["x0"], state["y0"]
            dx = (state["x_est"] - x0) if (x0 is not None and state["x_est"] is not None) else None
            dy = (state["y_est"] - y0) if (y0 is not None and state["y_est"] is not None) else None
            print(
                f"    target_h={target_h:.2f}m  z={state['z_est']}  "
                f"x={state['x_est']}(drift={dx})  y={state['y_est']}(drift={dy})",
                flush=True,
            )

        def send_ramp(start_h, end_h, duration_s):
            t0 = time.monotonic()
            while True:
                now = time.monotonic()
                elapsed = now - t0
                frac = min(1.0, elapsed / duration_s) if duration_s > 0 else 1.0
                target_h = start_h + (end_h - start_h) * frac
                current_target_h[0] = target_h

                cf.commander.send_zdistance_setpoint(0.0, 0.0, 0.0, target_h)
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
                cf.commander.send_zdistance_setpoint(0.0, 0.0, 0.0, height_m)
                print_status(height_m)
                if not watchdog_ok():
                    raise FlightAbort()
                time.sleep(SEND_PERIOD)

        try:
            print(f"阶段1/3：起飞爬升到 {TARGET_HEIGHT_M:.2f}m（{TAKEOFF_TIME_S:.1f}s）...", flush=True)
            send_ramp(LIFTOFF_HEIGHT_M, TARGET_HEIGHT_M, TAKEOFF_TIME_S)

            print(f"阶段2/3：定高悬停 {HOVER_TIME_S:.1f}s...", flush=True)
            hold(TARGET_HEIGHT_M, HOVER_TIME_S)

            print(f"阶段3/3：降落到地面（{LAND_TIME_S:.1f}s）...", flush=True)
            send_ramp(TARGET_HEIGHT_M, LAND_HEIGHT_M, LAND_TIME_S)
            hold(LAND_HEIGHT_M, TOUCHDOWN_SETTLE_S)

        except (FlightAbort, KeyboardInterrupt):
            print(
                f"触发紧急下降：从当前目标高度 {current_target_h[0]:.2f}m 快速降到地面"
                f"（{EMERGENCY_LAND_TIME_S:.1f}s）...",
                flush=True,
            )
            try:
                send_ramp(current_target_h[0], LAND_HEIGHT_M, EMERGENCY_LAND_TIME_S)
            except (FlightAbort, KeyboardInterrupt):
                pass

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
