#!/usr/bin/env python3
# T4h：通电稳态姿态偏置检查（机身固定在支架上，桨叶已拆除）。
#
# 动机：t5b（开环，roll/pitch 强制 0）比 t5（闭环）漂得更快更狠，说明真实飞行时水平方向存在一个
# 持续的偏置；但用 t4_sensor_readings_check.py 做的静态（不通电）转向测试，测出的残余倾角很小
# （<0.6°）且合成幅值不随朝向变化——更像是测试台面没放平，不是机身/CALIB 固定偏置。这次要测的是
# t5b/t5 里唯一还没排除的变量："电机转起来（有振动、电流负载）之后，是不是才会暴露一个静态测不
# 出来的偏置"。
#
# 核心陷阱——为什么不能像之前那样看 stabilizer.roll/pitch：
#   本脚本会送 roll=pitch=0（角度模式）让姿态闭环（controller_pid.c 的 attitudeController）主动
#   把机身"稳"在水平。闭环的定义就是让被控量（这里是姿态估计 stabilizer.roll/pitch）趋于设定值
#   0——不管背后有没有真实偏置，闭环工作正常的话 stabilizer.roll/pitch 都会被拉回接近 0，所以这两
#   个数完全看不出偏置存在与否，跟 t5b 那次"开环强制 0 才暴露漂移"是同一个道理反过来看。
#   真正能看出"闭环在悄悄用力纠正什么"的信号是**闭环的输出**：controller.cmd_roll/cmd_pitch
#   （controller_pid.c 里的 control->roll/control->pitch，直接喂给 power_distribution_stock.c 的
#   电机混控）。如果 roll=pitch=0 的设定下这两个值稳定卡在一个非零常数（不是围绕 0 抖动的噪声），
#   说明闭环一直在使劲纠正一个恒定的物理倾斜/不对称，这就是我们要找的偏置。
#   交叉验证：power_distribution_stock.c 的 QUAD_FORMATION_X 混控公式
#     m1 = thrust - roll/2 + pitch/2 + yaw      m2 = thrust - roll/2 - pitch/2 - yaw
#     m3 = thrust + roll/2 - pitch/2 + yaw      m4 = thrust + roll/2 + pitch/2 - yaw
#   如果 cmd_roll 稳定非零，(m3+m4)-(m1+m2) 应该跟着稳定偏向同一侧；cmd_pitch 同理对应
#   (m1+m4)-(m2+m3)。两边应该互相印证，对不上说明测试本身有问题（比如支架没固定牢）。
#
# 安全约定（复用 t4c_rpy_step_response_check.py 已经验证过的基础设施，不重新发明）：
#   桨叶必须已拆除、机身必须固定在支架上转不动、全程有人盯着电机；thrust 用一个低速基线，斜坡
#   上升避免电流冲击；日志新鲜度看门狗 + 电机 PWM 硬保护线；Ctrl+C 或异常都走零推力收尾。
# 陷阱提醒（跟 t4c 一样）：光流模块在线时固件会自动切到 POSHOLD_MODE，这时候 send_setpoint()
# 发的原始 attitude 指令会被 poshold 的前馈覆盖，所以要先关掉 flightmode.althold/poshold。
# 同样，kalman 在这种静态台架场景下姿态四元数缺乏可靠的加速度计倾角修正，所以强制切到
# complementary——complementary 是逐拍直接用 acc.x/y/z（已经过 CONFIG_ROLL_CALIB/PITCH_CALIB
# 旋转）做倾角修正，正是检验 CALIB 残余偏置最直接的路径。

import signal
import statistics
import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

from config import URI, connect_with_timeout

ESTIMATOR_NAMES = {0: "any", 1: "complementary", 2: "kalman"}

BASE_THRUST = 15000     # 低速基线，同 t4c，只是为了在 idleThrust 之上留出可观察的差量
RAMP_TIME_S = 1.0       # 从 0 斜坡爬升到 BASE_THRUST 的时长，避免电流冲击
SETTLE_TIME_S = 1.0     # 爬升完到开始统计之间的停留，确认基线稳定
STATS_SKIP_S = 0.5      # 进入统计窗口后再跳过这么久（等 PID 输出彻底收敛），才开始真正采样
HOLD_TIME_S = 6.0       # 稳态保持总时长（含 STATS_SKIP_S），越长统计越稳
SEND_PERIOD = 0.05      # 20Hz 发送 setpoint，避免 commander watchdog 超时进入 fallback

LOG_WAIT_TIMEOUT_S = 2.0    # 等待第一帧 motor 日志的超时：等不到就说明遥测没通，绝不能盲发推力
LOG_STALE_TIMEOUT_S = 0.3   # 运行中超过这么久没收到新日志帧，视为链路/主控可能已经异常
ZERO_BURST_COUNT = 15       # 收尾/异常时连续发送零推力的次数，抵御偶发丢包
ZERO_BURST_PERIOD = 0.02

MOTOR_ABORT_THRESHOLD = 55000  # 电机 PWM 逼近满量程(65535)的硬保护线


def send_zero_burst(cf, times=ZERO_BURST_COUNT, period=ZERO_BURST_PERIOD):
    """连续发送零推力 setpoint，且屏蔽期间的二次 Ctrl+C，防止收尾指令被打断。"""
    old_handler = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        for _ in range(times):
            cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(period)
    finally:
        signal.signal(signal.SIGINT, old_handler)


def read_current_estimator(cf, timeout_s=2.0):
    """读取 stabilizer.estimator 参数，返回当前值（1=complementary, 2=kalman），超时返回 None。"""
    result = {}
    got_value = threading.Event()

    def estimator_cb(_name, value):
        try:
            val = int(value)
        except (TypeError, ValueError):
            val = None
        label = ESTIMATOR_NAMES.get(val, f"未知({value})")
        print(f"当前 stabilizer.estimator = {value} ({label})", flush=True)
        result["value"] = val
        got_value.set()

    cf.param.add_update_callback(group="stabilizer", name="estimator", cb=estimator_cb)
    cf.param.request_param_update("stabilizer.estimator")

    if not got_value.wait(timeout=timeout_s):
        print("警告：读取 stabilizer.estimator 超时，未确认当前估计器。", flush=True)
        return None
    return result["value"]


def ensure_complementary_estimator(cf):
    """若当前是 kalman(2)，强制切成 complementary(1)。理由见文件头部注释。"""
    current = read_current_estimator(cf)
    if current == 2:
        print("检测到当前是 kalman，强制切换为 complementary...", flush=True)
        cf.param.set_value("stabilizer.estimator", "1")
        time.sleep(0.5)
        read_current_estimator(cf)


def read_flightmode_flags(cf, timeout_s=2.0):
    """读取 flightmode.althold / flightmode.poshold，返回 {"althold": 0/1, "poshold": 0/1}。"""
    result = {}
    got_values = threading.Event()

    def make_cb(key):
        def cb(_name, value):
            try:
                result[key] = int(value)
            except (TypeError, ValueError):
                result[key] = None
            if "althold" in result and "poshold" in result:
                got_values.set()
        return cb

    cf.param.add_update_callback(group="flightmode", name="althold", cb=make_cb("althold"))
    cf.param.add_update_callback(group="flightmode", name="poshold", cb=make_cb("poshold"))
    cf.param.request_param_update("flightmode.althold")
    cf.param.request_param_update("flightmode.poshold")

    if not got_values.wait(timeout=timeout_s):
        print("警告：读取 flightmode.althold/poshold 超时。", flush=True)
    print(
        f"当前 flightmode.althold={result.get('althold')}  "
        f"flightmode.poshold={result.get('poshold')}",
        flush=True,
    )
    return result


def ensure_raw_commander_mode(cf):
    """若 althold/poshold 开着，强制关掉，改用原始 thrust/roll/pitch/yaw setpoint。"""
    flags = read_flightmode_flags(cf)
    if flags.get("althold") or flags.get("poshold"):
        print("检测到 althold/poshold 开启，强制关闭，改用原始 thrust/roll/pitch/yaw...", flush=True)
        cf.param.set_value("flightmode.althold", "0")
        cf.param.set_value("flightmode.poshold", "0")
        time.sleep(0.5)
        read_flightmode_flags(cf)


def main():
    cflib.crtp.init_drivers()

    print("再次确认：桨叶已拆除、飞机已固定在支架上（转不动）、你会全程盯着电机。")
    if input("确认无误请输入 yes 继续: ").strip().lower() != "yes":
        print("已取消。")
        return

    cf = Crazyflie()

    link_lost = threading.Event()

    def on_connection_lost(uri, msg):
        print(f"\n[链路] connection_lost: {msg}", flush=True)
        link_lost.set()

    def on_disconnected(uri):
        print("\n[链路] disconnected", flush=True)
        link_lost.set()

    cf.connection_lost.add_callback(on_connection_lost)
    cf.disconnected.add_callback(on_disconnected)

    if not connect_with_timeout(cf, URI):
        return

    try:
        ensure_complementary_estimator(cf)
        ensure_raw_commander_mode(cf)

        stop_event = threading.Event()

        # 确保不是 motorPowerSet 覆盖模式，走真实闭环
        cf.param.set_value("motorPowerSet.enable", "0")
        time.sleep(0.1)

        log_state = {"last": None}
        diag_state = {
            "controller.cmd_roll": None,
            "controller.cmd_pitch": None,
            "controller.cmd_thrust": None,
            "stabilizer.roll": None,
            "stabilizer.pitch": None,
        }
        collecting = {"on": False}
        samples = {"cmd_roll": [], "cmd_pitch": [], "m1": [], "m2": [], "m3": [], "m4": []}

        lg_motor = LogConfig(name="motor", period_in_ms=50)
        for v in ("motor.m1", "motor.m2", "motor.m3", "motor.m4"):
            lg_motor.add_variable(v, "uint32_t")
        cf.log.add_config(lg_motor)

        lg_ctrl = LogConfig(name="ctrl", period_in_ms=50)
        lg_ctrl.add_variable("controller.cmd_roll", "float")
        lg_ctrl.add_variable("controller.cmd_pitch", "float")
        lg_ctrl.add_variable("controller.cmd_thrust", "float")
        cf.log.add_config(lg_ctrl)

        lg_att = LogConfig(name="att", period_in_ms=100)
        lg_att.add_variable("stabilizer.roll", "float")
        lg_att.add_variable("stabilizer.pitch", "float")
        cf.log.add_config(lg_att)

        def check_auto_abort(m1, m2, m3, m4):
            if stop_event.is_set():
                return
            for name, value in (("m1", m1), ("m2", m2), ("m3", m3), ("m4", m4)):
                if value >= MOTOR_ABORT_THRESHOLD:
                    print(
                        f"\n警告：{name}={value} 已逼近满量程（硬保护线 {MOTOR_ABORT_THRESHOLD}），"
                        "自动停止并归零！",
                        flush=True,
                    )
                    stop_event.set()
                    return

        def log_motor_cb(timestamp, data, logconf):
            log_state["last"] = time.monotonic()
            m1, m2, m3, m4 = data["motor.m1"], data["motor.m2"], data["motor.m3"], data["motor.m4"]
            cr = diag_state["controller.cmd_roll"]
            cp = diag_state["controller.cmd_pitch"]
            er = diag_state["stabilizer.roll"]
            ep = diag_state["stabilizer.pitch"]
            cr_s = f"{cr:+7.1f}" if cr is not None else "    n/a"
            cp_s = f"{cp:+7.1f}" if cp is not None else "    n/a"
            er_s = f"{er:+6.2f}" if er is not None else "  n/a"
            ep_s = f"{ep:+6.2f}" if ep is not None else "  n/a"
            print(
                f"t={timestamp:>8}  m1={m1:6d}  m2={m2:6d}  m3={m3:6d}  m4={m4:6d}  |  "
                f"cmd(roll,pitch)=({cr_s},{cp_s})  |  est(roll,pitch)=({er_s},{ep_s})"
                f"{'  [采样中]' if collecting['on'] else ''}",
                flush=True,
            )
            if collecting["on"]:
                samples["m1"].append(m1)
                samples["m2"].append(m2)
                samples["m3"].append(m3)
                samples["m4"].append(m4)
            check_auto_abort(m1, m2, m3, m4)

        def log_ctrl_cb(_timestamp, data, _logconf):
            diag_state["controller.cmd_roll"] = data["controller.cmd_roll"]
            diag_state["controller.cmd_pitch"] = data["controller.cmd_pitch"]
            diag_state["controller.cmd_thrust"] = data["controller.cmd_thrust"]
            if collecting["on"]:
                samples["cmd_roll"].append(data["controller.cmd_roll"])
                samples["cmd_pitch"].append(data["controller.cmd_pitch"])

        def log_att_cb(_timestamp, data, _logconf):
            diag_state["stabilizer.roll"] = data["stabilizer.roll"]
            diag_state["stabilizer.pitch"] = data["stabilizer.pitch"]

        lg_motor.data_received_cb.add_callback(log_motor_cb)
        lg_ctrl.data_received_cb.add_callback(log_ctrl_cb)
        lg_att.data_received_cb.add_callback(log_att_cb)
        lg_motor.start()
        lg_ctrl.start()
        lg_att.start()

        print("等待第一帧 motor 日志，确认遥测通路正常...", flush=True)
        wait_deadline = time.monotonic() + LOG_WAIT_TIMEOUT_S
        while log_state["last"] is None and time.monotonic() < wait_deadline and not link_lost.is_set():
            time.sleep(0.05)

        if log_state["last"] is None:
            print(
                "超时仍未收到 motor 日志，遥测通路不通或已断连，为安全起见不会发送任何推力。"
                "请检查连接后重试；若怀疑是主控复位，可运行 t0_reset_reason.py 排查。",
                flush=True,
            )
            lg_motor.stop()
            lg_ctrl.stop()
            lg_att.stop()
            return

        print("日志已确认在收，即将开始发送推力基线（斜坡爬升，避免电流冲击）。", flush=True)

        def log_is_stale():
            last = log_state["last"]
            if last is None:
                return False
            return (time.monotonic() - last) > LOG_STALE_TIMEOUT_S

        def send_for(thrust, duration_s):
            """roll=pitch=yawrate=0，thrust 固定，按 SEND_PERIOD 发送直到 duration_s 用完。
            返回 False 表示需要中止（stop/link_lost/日志过期），调用方应立即停止。"""
            deadline = time.monotonic() + duration_s
            while time.monotonic() < deadline:
                if stop_event.is_set() or link_lost.is_set():
                    return False
                if log_is_stale():
                    print(
                        f"\n警告：超过 {LOG_STALE_TIMEOUT_S}s 未收到 motor 日志，"
                        "链路或主控可能已异常，立即停止！",
                        flush=True,
                    )
                    stop_event.set()
                    return False
                cf.commander.send_setpoint(0, 0, 0, thrust)
                time.sleep(SEND_PERIOD)
            return True

        def send_ramp(from_thrust, to_thrust, duration_s):
            """thrust 从 from_thrust 斜坡到 to_thrust，roll/pitch/yawrate 始终为 0。"""
            ramp_steps = max(1, int(duration_s / SEND_PERIOD))
            for i in range(1, ramp_steps + 1):
                if stop_event.is_set() or link_lost.is_set():
                    return False
                if log_is_stale():
                    print(
                        f"\n警告：斜坡阶段超过 {LOG_STALE_TIMEOUT_S}s 未收到 motor 日志，"
                        "链路或主控可能已异常，立即停止加推力！",
                        flush=True,
                    )
                    stop_event.set()
                    return False
                thrust = int(from_thrust + (to_thrust - from_thrust) * i / ramp_steps)
                cf.commander.send_setpoint(0, 0, 0, thrust)
                time.sleep(SEND_PERIOD)
            return True

        def setpoint_loop():
            cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.1)

            if not send_ramp(0, BASE_THRUST, RAMP_TIME_S):
                return

            print(f"基线已到 {BASE_THRUST}，停留 {SETTLE_TIME_S:.1f}s 确认稳定...", flush=True)
            if not send_for(BASE_THRUST, SETTLE_TIME_S):
                return

            print(
                f"进入 {HOLD_TIME_S:.1f}s 稳态保持，先跳过前 {STATS_SKIP_S:.1f}s 等 PID 输出彻底收敛，"
                "再开始采样 cmd_roll/cmd_pitch/motor...",
                flush=True,
            )
            if not send_for(BASE_THRUST, STATS_SKIP_S):
                return

            collecting["on"] = True
            ok = send_for(BASE_THRUST, HOLD_TIME_S - STATS_SKIP_S)
            collecting["on"] = False
            if not ok:
                return

            print("\n稳态保持结束，正常收尾。", flush=True)
            stop_event.set()

        sp_thread = threading.Thread(target=setpoint_loop, daemon=True)

        print("\n即将开始：thrust 从 0 斜坡爬升到低速基线，稳定后送 roll=pitch=0 保持不动，")
        print("采样窗口内统计 controller.cmd_roll/cmd_pitch 是否稳定偏离 0（见文件头部注释的判读方法）。")
        print("按 Ctrl+C 可随时提前结束测试。\n")

        sp_thread.start()

        try:
            while not stop_event.is_set() and not link_lost.is_set():
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        finally:
            print("\n正在停止并归零...", flush=True)
            stop_event.set()
            sp_thread.join(timeout=2)
            if link_lost.is_set():
                print(
                    "链路已断开：脚本发送的零推力指令很可能到不了飞机。"
                    "固件自身有 2 秒的 commander watchdog，会在断链后自动把推力清零；"
                    "如果远超这个时间电机仍不停，请立刻断电，别指望脚本能救回。",
                    flush=True,
                )
            else:
                send_zero_burst(cf)
            lg_motor.stop()
            lg_ctrl.stop()
            lg_att.stop()
            print("测试结束。", flush=True)

        if samples["cmd_roll"]:
            n = len(samples["cmd_roll"])
            cr_mean = statistics.mean(samples["cmd_roll"])
            cr_std = statistics.pstdev(samples["cmd_roll"]) if n > 1 else 0.0
            cp_mean = statistics.mean(samples["cmd_pitch"])
            cp_std = statistics.pstdev(samples["cmd_pitch"]) if n > 1 else 0.0
            m_means = {k: statistics.mean(samples[k]) for k in ("m1", "m2", "m3", "m4")}
            roll_motor_indicator = (m_means["m3"] + m_means["m4"]) - (m_means["m1"] + m_means["m2"])
            pitch_motor_indicator = (m_means["m1"] + m_means["m4"]) - (m_means["m2"] + m_means["m3"])

            print(f"\n===== 采样窗口统计（n={n} 帧）=====")
            print(f"cmd_roll   均值={cr_mean:+8.2f}  标准差={cr_std:7.2f}")
            print(f"cmd_pitch  均值={cp_mean:+8.2f}  标准差={cp_std:7.2f}")
            print(
                f"motor 均值  m1={m_means['m1']:.0f}  m2={m_means['m2']:.0f}  "
                f"m3={m_means['m3']:.0f}  m4={m_means['m4']:.0f}"
            )
            print(
                f"交叉验证：(m3+m4)-(m1+m2)={roll_motor_indicator:+.0f}（应与 cmd_roll 符号一致）  "
                f"(m1+m4)-(m2+m3)={pitch_motor_indicator:+.0f}（应与 cmd_pitch 符号一致）"
            )
            print(
                "\n判读：cmd_roll/cmd_pitch 的均值如果明显超过标准差的好几倍（不是围绕0的噪声），"
                "说明 roll=pitch=0 的设定下闭环一直在稳定地朝一个方向用力纠正——这就是电机通电/振动"
                "之后才暴露出来的偏置，而且方向应该和上面的 motor 交叉验证对得上。\n"
                "如果均值和标准差量级差不多、看不出稳定方向，说明这条路径本身没有明显偏置，"
                "之前飞行时的水平漂移主因更可能在别处（比如光流在贴地阶段的比例因子误差）。"
            )
    finally:
        cf.close_link()


if __name__ == "__main__":
    main()
