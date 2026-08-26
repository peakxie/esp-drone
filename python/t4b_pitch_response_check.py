#!/usr/bin/env python3
# Step B: 闭环下人工制造 pitch/roll 干扰，看 motor.m1~m4 响应方向对不对。
# 保持水平设定点 (0,0,0)，用手倾斜机身；PID 会尝试把它纠正回水平。
# 前提：桨叶已拆除，飞机已固定，全程有人盯着；thrust 只用一个很低的基线值。
#
# 注意：4 个电机同时起转（哪怕基线很低）比 t3 的单电机爬坡电流冲击大得多，
# 姿态 PID 在裸机（无桨气动阻力）场景下也容易瞬间打满某个电机。如果测试中
# 出现"电机狂转、日志彻底不刷新、Ctrl+C 也止不住"，大概率不是这个脚本能救的
# 软件问题，而是电流冲击把主控拉到 brownout/看门狗复位了——复位瞬间 PWM 外设
# 状态不受固件控制，脚本发什么指令都没用。出现这种情况请立刻断电，然后在
# 保持供电、能重连的前提下跑一次 t0_reset_reason.py，看 sys.resetReason 是否
# 为 9（brownout）或 5/6/7（看门狗复位），以确认/排除这个假设。

import signal
import threading
import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from config import URI

ESTIMATOR_NAMES = {0: "any", 1: "complementary", 2: "kalman"}

BASE_THRUST = 15000     # 低速基线，只是为了在 idleThrust 之上留出可观察的差量
RAMP_TIME_S = 1.0       # 从 0 斜坡爬升到 BASE_THRUST 的时长，避免 4 电机同时起转的电流冲击
SEND_PERIOD = 0.05      # 20Hz 发送 setpoint，避免 commander watchdog 超时进入 fallback
LOG_WAIT_TIMEOUT_S = 2.0    # 等待第一帧 motor 日志的超时：等不到就说明遥测没通，绝不能盲发推力
LOG_STALE_TIMEOUT_S = 0.3   # 运行中超过这么久没收到新日志帧，视为链路/主控可能已经异常
ZERO_BURST_COUNT = 15       # 收尾/异常时连续发送零推力的次数，抵御偶发丢包
ZERO_BURST_PERIOD = 0.02

# 实测发现过"机身没放平+夹具固定住转不动"会让姿态角环积分一路 windup，把某个电机顶向满转，
# 且过程是单调爬升、没有 D 项尖峰特征（见 pid_rate.pitch_outD 基本正常）。这两条阈值分别是
# "电机快到硬件上限了，不管什么原因先停"和"角速度环 P 项已经大到基本能断定是积分饱和"。
MOTOR_ABORT_THRESHOLD = 55000       # 电机 PWM 逼近满量程(65535)的硬保护线
RATE_OUTP_ABORT_THRESHOLD = 15000.0  # pid_rate.pitch_outP 绝对值超过这个数，基本可判定为姿态环积分饱和


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
    """若当前是 kalman(2)，强制切成 complementary(1)。本脚本是台架测试（桨叶已拆、机身固定，
    不是真实飞行），光流模块一在线固件就会自动锁定 kalman，但 kalman 的姿态四元数在这种静态
    场景下没有加速度计做倾角修正，观测到的姿态角可能不可信，不适合用来判断 PID 响应方向对不对。"""
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
    """若 althold/poshold 开着，强制关掉。光流模块在线时固件开机会自动 setCommandermode(POSHOLD_MODE)，
    把 althold 和 poshold 都打开，导致本脚本发的原始 thrust/roll/pitch 被 RPYT 解码器丢弃、改用定高/定点
    速度 PID 的输出（带一个很大的固定前馈量，且没有上限），跟脚本"缓慢斜坡爬升"的意图完全不是一回事，
    会在电机上瞬间叠加出远超预期的输出。切估计器不会动这两个开关，必须单独关掉。"""
    flags = read_flightmode_flags(cf)
    if flags.get("althold") or flags.get("poshold"):
        print("检测到 althold/poshold 开启，强制关闭，改用原始 thrust/roll/pitch...", flush=True)
        cf.param.set_value("flightmode.althold", "0")
        cf.param.set_value("flightmode.poshold", "0")
        time.sleep(0.5)
        read_flightmode_flags(cf)


def main():
    cflib.crtp.init_drivers()

    print("再次确认：桨叶已拆除、飞机已固定、你会全程盯着电机。")
    if input("确认无误请输入 yes 继续: ").strip().lower() != "yes":
        print("已取消。")
        return

    with SyncCrazyflie(URI) as scf:
        cf = scf.cf
        print("已连接。", flush=True)

        link_lost = threading.Event()

        def on_connection_lost(uri, msg):
            print(f"\n[链路] connection_lost: {msg}", flush=True)
            link_lost.set()

        def on_disconnected(uri):
            print("\n[链路] disconnected", flush=True)
            link_lost.set()

        cf.connection_lost.add_callback(on_connection_lost)
        cf.disconnected.add_callback(on_disconnected)

        ensure_complementary_estimator(cf)
        ensure_raw_commander_mode(cf)

        # 提前建好 stop_event：既用于 setpoint 发送线程的退出信号，也给下面的 log 回调里的
        # 自动保护复用，两边共享同一把停止开关。
        stop_event = threading.Event()

        # 确保不是 motorPowerSet 覆盖模式，走真实闭环
        cf.param.set_value("motorPowerSet.enable", "0")
        time.sleep(0.1)

        log_state = {"last": None}
        # 姿态/角速度 PID 分量 + 实际姿态角，用于坐实"电机尖峰是不是角速度环 D 项打出来的"、
        # "手动掰 pitch 时是不是也带了 roll 耦合"，跟 motor 日志分开一路，避免单个 log block
        # 超出 CRTP payload 限制。数值可能比 motor 那一路晚最多一个周期，人工读数够用。
        diag_state = {
            "pid_rate.pitch_outP": None,
            "pid_rate.pitch_outD": None,
            "stateEstimate.roll": None,
            "stateEstimate.pitch": None,
            "gyro.y": None,
            "controller.pitchRate": None,
        }

        lg = LogConfig(name="motor", period_in_ms=50)
        lg.add_variable("motor.m1", "uint32_t")
        lg.add_variable("motor.m2", "uint32_t")
        lg.add_variable("motor.m3", "uint32_t")
        lg.add_variable("motor.m4", "uint32_t")
        cf.log.add_config(lg)

        lg_diag = LogConfig(name="pid_diag", period_in_ms=50)
        lg_diag.add_variable("pid_rate.pitch_outP", "float")
        lg_diag.add_variable("pid_rate.pitch_outD", "float")
        lg_diag.add_variable("stateEstimate.roll", "float")
        lg_diag.add_variable("stateEstimate.pitch", "float")
        cf.log.add_config(lg_diag)

        # 单独一路，验证"outP 变大是不是因为你手速带来的真实角速度、而不是位置误差积分"：
        # gyro.y 是原始陀螺读数，controller.pitchRate（=rateDesired.pitch，attitudeControllerCorrectRatePID
        # 的期望角速度输入）是外环给内环的"期望角速度"。attitudeControllerCorrectRatePID 调用时传入的实际角速度
        # 是 -gyro.y（controller_pid.c 里 -sensors->gyro.y），是为了跟 stateEstimate.pitch 统一"抬头为正"的符号，
        # 所以这里打印时也换算成 -gyro.y，直接跟 pitchRate（期望角速度）同符号可比：如果抬头时 -gyro.y
        # 是一个远大于 pitchRate 的正数，说明 outP 主要是在对抗你手的转动速度，不是角度误差积分。
        lg_rate = LogConfig(name="rate_diag", period_in_ms=50)
        lg_rate.add_variable("gyro.y", "float")
        lg_rate.add_variable("controller.pitchRate", "float")
        cf.log.add_config(lg_rate)

        def check_auto_abort(m1, m2, m3, m4):
            """硬保护线 + windup 早期预警。任何一条触发就置位 stop_event，交给已有的收尾逻辑归零。"""
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
            outp = diag_state["pid_rate.pitch_outP"]
            if outp is not None and abs(outp) >= RATE_OUTP_ABORT_THRESHOLD:
                print(
                    f"\n警告：pid_rate.pitch_outP={outp:.0f} 超过 {RATE_OUTP_ABORT_THRESHOLD:.0f}，"
                    "疑似姿态角环积分饱和(windup)——机身没放平或被夹具固定住转不动，"
                    "自动停止并归零！请把机身调平后再测。",
                    flush=True,
                )
                stop_event.set()

        def log_cb(timestamp, data, logconf):
            log_state["last"] = time.monotonic()
            outp = diag_state["pid_rate.pitch_outP"]
            outd = diag_state["pid_rate.pitch_outD"]
            roll = diag_state["stateEstimate.roll"]
            pitch = diag_state["stateEstimate.pitch"]
            gyro_y = diag_state["gyro.y"]
            rate_desired = diag_state["controller.pitchRate"]
            outp_s = f"{outp:8.1f}" if outp is not None else "    n/a"
            outd_s = f"{outd:8.1f}" if outd is not None else "    n/a"
            roll_s = f"{roll:6.1f}" if roll is not None else "   n/a"
            pitch_s = f"{pitch:6.1f}" if pitch is not None else "   n/a"
            rate_actual_s = f"{-gyro_y:8.1f}" if gyro_y is not None else "     n/a"
            rate_desired_s = f"{rate_desired:8.1f}" if rate_desired is not None else "     n/a"
            print(
                f"t={timestamp:>8}  m1={data['motor.m1']:6d}  m2={data['motor.m2']:6d}  "
                f"m3={data['motor.m3']:6d}  m4={data['motor.m4']:6d}  |  "
                f"rateP={outp_s}  rateD={outd_s}  roll={roll_s}  pitch={pitch_s}  |  "
                f"rateDesired={rate_desired_s}  rateActual(-gyro.y)={rate_actual_s}",
                flush=True,
            )
            check_auto_abort(data["motor.m1"], data["motor.m2"], data["motor.m3"], data["motor.m4"])

        def log_diag_cb(timestamp, data, logconf):
            diag_state["pid_rate.pitch_outP"] = data["pid_rate.pitch_outP"]
            diag_state["pid_rate.pitch_outD"] = data["pid_rate.pitch_outD"]
            diag_state["stateEstimate.roll"] = data["stateEstimate.roll"]
            diag_state["stateEstimate.pitch"] = data["stateEstimate.pitch"]

        def log_rate_cb(timestamp, data, logconf):
            diag_state["gyro.y"] = data["gyro.y"]
            diag_state["controller.pitchRate"] = data["controller.pitchRate"]

        lg.data_received_cb.add_callback(log_cb)
        lg_diag.data_received_cb.add_callback(log_diag_cb)
        lg_rate.data_received_cb.add_callback(log_rate_cb)
        lg.start()
        lg_diag.start()
        lg_rate.start()

        # 先确认遥测通路活着，再考虑给电机上电，绝不盲发推力
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
            lg.stop()
            lg_diag.stop()
            lg_rate.stop()
            return

        print("日志已确认在收，即将开始发送推力基线（斜坡爬升，避免电流冲击）。", flush=True)

        def log_is_stale():
            last = log_state["last"]
            if last is None:
                return False
            return (time.monotonic() - last) > LOG_STALE_TIMEOUT_S

        def setpoint_loop():
            # 先发一次 thrust=0 解锁 thrust lock
            cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.1)

            ramp_steps = max(1, int(RAMP_TIME_S / SEND_PERIOD))
            for i in range(1, ramp_steps + 1):
                if stop_event.is_set() or link_lost.is_set():
                    return
                if log_is_stale():
                    print(
                        f"\n警告：斜坡爬升阶段超过 {LOG_STALE_TIMEOUT_S}s 未收到 motor 日志，"
                        "链路或主控可能已异常，立即停止加推力！",
                        flush=True,
                    )
                    stop_event.set()
                    return
                thrust = int(BASE_THRUST * i / ramp_steps)
                cf.commander.send_setpoint(0, 0, 0, thrust)
                time.sleep(SEND_PERIOD)

            while not stop_event.is_set() and not link_lost.is_set():
                if log_is_stale():
                    print(
                        f"\n警告：超过 {LOG_STALE_TIMEOUT_S}s 未收到 motor 日志，"
                        "链路或主控可能已异常，立即停止加推力！",
                        flush=True,
                    )
                    stop_event.set()
                    break
                cf.commander.send_setpoint(0, 0, 0, BASE_THRUST)
                time.sleep(SEND_PERIOD)

        sp_thread = threading.Thread(target=setpoint_loop, daemon=True)

        print("\n即将开始：thrust 从 0 斜坡爬升到低速基线，pitch/roll 设定点始终为 0（水平）。")
        print("电机转动、稳定几秒后，请缓慢抬起机头，观察打印：")
        print("  期望：m1、m4（前）变小，m2、m3（后）变大")
        print("  再缓慢低头，期望反过来：m1、m4 变大，m2、m3 变小")
        print("新增的 rateP/rateD 是角速度环 pitch 通道的 P/D 分量，roll/pitch 是实际估计姿态角：")
        print("  如果电机尖峰时 |rateD| 远大于 |rateP|，说明是未滤波的 D 项被陀螺噪声/震动打出来的尖峰")
        print("  如果你在掰 pitch 时 roll 也明显跟着变，说明手动动作带了耦合，m1m4/m2m3 规律会被 roll 项打乱")
        print("rateDesired 是外环给的期望角速度，rateActual(-gyro.y) 是换算成'抬头为正'后的实际角速度：")
        print("  重点看抬头这一下：如果 rateActual 出现一个远大于 rateDesired 的正值，说明 rateP 变负")
        print("  主要是角速度环在对抗你手的转动速度本身，不是角度误差积分——这是要验证的关键现象")
        print("按 Ctrl+C 结束测试。\n")

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
            lg.stop()
            lg_diag.stop()
            lg_rate.stop()
            print("测试结束。", flush=True)


if __name__ == "__main__":
    main()
