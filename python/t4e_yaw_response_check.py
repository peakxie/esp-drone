#!/usr/bin/env python3
# Step E：闭环下人工制造 yaw 干扰，看 motor.m1~m4 响应方向对不对，专门验证 yaw 通道。
# 保持水平/定头设定点 (0,0,yawrate=0)，用手扭转机头；PID 会尝试把它纠正回原来的朝向。
# 前提：桨叶已拆除、飞机已固定、全程有人盯着；thrust 只用一个很低的基线值。
#
# 为什么单独测 yaw：roll/pitch 的符号已经用 t4b/t4c 逐项核对过（pitch 那次还真揪出过
# power_distribution_stock.c 里 CONFIG_PITCH_DISTRIBUTION_INVERTED 配错导致的正反馈）。
# 但 yaw 从来没有拿真实的电机顺逆时针事实去对过——t4c_rpy_step_response_check.py 里 yaw
# 的 "expect" 字符串，注释写的是直接从混控公式反推出来的，等于拿代码的假设去验证代码自己，
# 不是独立证据，跟当年验证 pitch 时踩过的坑是同一个坑。
#
# 已知的怀疑点：controller_pid.c 里 attitudeControllerGetActuatorOutput() 之后紧跟着一行
#   control->yaw = -control->yaw;
# 这行是初次移植（2020）就带进来的原始代码，从未针对这块板子的真实电机转向验证过。结合已确认
# 的事实——M1、M3（对角线）俯视顺时针，MPU6050 正面朝上贴装、Z 轴跟机身 Z 轴同向（代码对
# gyro.z 没做任何取负）——把"角速度环 error=desired-measured"（pid.c）→ 这行取负 → 混控
# m1/m3 +yaw、m2/m4 -yaw 这条链路整个推一遍，结论是：当前这行会让 yaw 内环变成正反馈，
# 跟当年 pitch 的 bug 是同一类问题，只是藏在这一行，不在混控矩阵里。这个脚本就是用实测数据
# 坐实（或推翻）这个推导，测完再决定要不要动代码。
#
# 关键就看两个量的符号关系（同一个 log 周期内对照）：
#   pid_rate.yaw_outP  —— 角速度环内部原始输出（取负*之前*，attitude_pid_controller.c 里的 yawOutput）
#   controller.cmd_yaw —— 送进混控矩阵的最终值（取负*之后*，跟上面这个应该正好相反号）
# 如果这两个数确实一直是相反号，说明第116行那次取负真实存在、在起作用（这一步毋庸置疑，
# 只是读代码就知道）；真正要看的是 controller.cmd_yaw 和电机 m1/m3 变化方向的关系，
# 结合 M1/M3=顺时针这个物理事实来判断最终合力矩方向对不对。
#
# 操作方法：等基线转速稳定后，*缓慢、持续地* 把机头往左扭（俯视从上往下看，机头从正前方
# 转向左边，即 CCW/逆时针转动机身——不是让它转一下就停，是你主动、持续地转，跟 t4b 里
# "缓慢抬起机头"是同一种手法）。同时读 gyro.z：正值就代表你在往 CCW 方向转，跟预期一致，
# 说明这一步至少没有拿错方向。
#
#   正确（负反馈）应该看到：controller.cmd_yaw 变负，m1、m3（顺时针电机）转速*降低*，
#     m2、m4（逆时针电机）转速*升高*——即系统在用"减少顺时针电机反作用力矩"的方式产生
#     顺时针纠正力矩，把被你转歪的机头拉回去。
#   如果是 bug（正反馈）会看到：controller.cmd_yaw 变正，m1、m3 转速*升高*、m2、m4
#     转速*降低*——顺时针电机转速升高会让机体反作用力矩更偏向 CCW，等于在帮你把机头往
#     你转的方向继续推，而不是拉回来。
#
# 注意：4 个电机同时起转（哪怕基线很低）比单电机爬坡电流冲击大得多。如果测试中出现
# "电机狂转、日志彻底不刷新、Ctrl+C 也止不住"，大概率是电流冲击把主控拉到 brownout/
# 看门狗复位了，请立刻断电，然后跑一次 t0_reset_reason.py 排查。

import signal
import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

from config import URI, connect_with_timeout

ESTIMATOR_NAMES = {0: "any", 1: "complementary", 2: "kalman"}

BASE_THRUST = 15000     # 低速基线，只是为了在 idleThrust 之上留出可观察的差量
RAMP_TIME_S = 1.0       # 从 0 斜坡爬升到 BASE_THRUST 的时长，避免 4 电机同时起转的电流冲击
SEND_PERIOD = 0.05      # 20Hz 发送 setpoint，避免 commander watchdog 超时进入 fallback
LOG_WAIT_TIMEOUT_S = 2.0    # 等待第一帧 motor 日志的超时：等不到就说明遥测没通，绝不能盲发推力
LOG_STALE_TIMEOUT_S = 0.3   # 运行中超过这么久没收到新日志帧，视为链路/主控可能已经异常
ZERO_BURST_COUNT = 15       # 收尾/异常时连续发送零推力的次数，抵御偶发丢包
ZERO_BURST_PERIOD = 0.02

# 阈值含义同 t4b：MOTOR_ABORT_THRESHOLD 是电机 PWM 逼近满量程(65535)的硬保护线；
# YAW_RATE_OUTP_ABORT_THRESHOLD 是角速度环 yaw 通道 outP 绝对值过大、基本可判定为
# 积分/角度环饱和的预警线（机身固定夹死、转不动导致误差一直存在也会触发，不一定是 bug，
# 但先停下来看数据更安全）。
MOTOR_ABORT_THRESHOLD = 55000
YAW_RATE_OUTP_ABORT_THRESHOLD = 15000.0


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
    """若当前是 kalman(2)，强制切成 complementary(1)。理由同 t4b：本脚本是台架测试，
    静态场景下 kalman 的姿态四元数没有真实速度观测做修正，不适合用来判断响应方向对不对。"""
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
    """若 althold/poshold 开着，强制关掉，改用原始 thrust/roll/pitch/yaw setpoint。原因同 t4b/t4c：
    光流模块在线时固件会自动切到 POSHOLD_MODE，脚本发的指令会被丢弃、改用带固定前馈量的
    定高/定点速度 PID 输出，跟本脚本"发送已知 yawrate=0、手动施加干扰"的意图完全不是一回事。"""
    flags = read_flightmode_flags(cf)
    if flags.get("althold") or flags.get("poshold"):
        print("检测到 althold/poshold 开启，强制关闭，改用原始 thrust/roll/pitch/yaw...", flush=True)
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
            "pid_rate.yaw_outP": None,
            "pid_rate.yaw_outI": None,
            "pid_rate.yaw_outD": None,
            "gyro.z": None,
            "controller.yawRate": None,
            "stateEstimate.yaw": None,
            "controller.cmd_yaw": None,
            "stateEstimate.roll": None,
            "stateEstimate.pitch": None,
        }

        lg = LogConfig(name="motor", period_in_ms=50)
        lg.add_variable("motor.m1", "uint32_t")
        lg.add_variable("motor.m2", "uint32_t")
        lg.add_variable("motor.m3", "uint32_t")
        lg.add_variable("motor.m4", "uint32_t")
        cf.log.add_config(lg)

        # 角速度环 yaw 通道的 P/I/D（取负*之前*）+ 陀螺 z 原始值 + 外环给的期望角速度。
        lg_diag = LogConfig(name="yaw_diag", period_in_ms=50)
        lg_diag.add_variable("pid_rate.yaw_outP", "float")
        lg_diag.add_variable("pid_rate.yaw_outI", "float")
        lg_diag.add_variable("pid_rate.yaw_outD", "float")
        lg_diag.add_variable("gyro.z", "float")
        lg_diag.add_variable("controller.yawRate", "float")
        cf.log.add_config(lg_diag)

        # stateEstimate.yaw 看朝向有没有真的在往你转的方向跑飘；cmd_yaw 是取负*之后*、
        # 送进混控矩阵的最终值——跟上面 pid_rate.yaw_outP 对照，符号差异就是那一行取负
        # 在起作用；roll/pitch 顺便看有没有被手动扭 yaw 的动作带出耦合。
        lg_att = LogConfig(name="yaw_att", period_in_ms=50)
        lg_att.add_variable("stateEstimate.yaw", "float")
        lg_att.add_variable("controller.cmd_yaw", "float")
        lg_att.add_variable("stateEstimate.roll", "float")
        lg_att.add_variable("stateEstimate.pitch", "float")
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
            outp = diag_state["pid_rate.yaw_outP"]
            if outp is not None and abs(outp) >= YAW_RATE_OUTP_ABORT_THRESHOLD:
                print(
                    f"\n警告：pid_rate.yaw_outP={outp:.0f} 超过 {YAW_RATE_OUTP_ABORT_THRESHOLD:.0f}，"
                    "疑似 yaw 角速度环积分饱和(windup)——机身被固定住转不动导致误差一直存在，"
                    "自动停止并归零！",
                    flush=True,
                )
                stop_event.set()

        def _fmt(value, width=8, prec=1):
            return f"{value:{width}.{prec}f}" if value is not None else f"{'n/a':>{width}}"

        def log_cb(timestamp, data, logconf):
            log_state["last"] = time.monotonic()
            outp = diag_state["pid_rate.yaw_outP"]
            outi = diag_state["pid_rate.yaw_outI"]
            outd = diag_state["pid_rate.yaw_outD"]
            gz = diag_state["gyro.z"]
            rate_desired = diag_state["controller.yawRate"]
            yaw_est = diag_state["stateEstimate.yaw"]
            cmd_yaw = diag_state["controller.cmd_yaw"]
            roll_est = diag_state["stateEstimate.roll"]
            pitch_est = diag_state["stateEstimate.pitch"]
            print(
                f"t={timestamp:>8}  m1={data['motor.m1']:6d}  m2={data['motor.m2']:6d}  "
                f"m3={data['motor.m3']:6d}  m4={data['motor.m4']:6d}  |  "
                f"yawOutP(取负前)={_fmt(outp)}  cmd_yaw(取负后)={_fmt(cmd_yaw)}  |  "
                f"gyro.z(正=CCW/左转)={_fmt(gz)}  rateDesired={_fmt(rate_desired)}",
                flush=True,
            )
            print(
                f"          yawOutI={_fmt(outi)}  yawOutD={_fmt(outd)}  |  "
                f"yaw={_fmt(yaw_est, 6)}  roll={_fmt(roll_est, 6)}  pitch={_fmt(pitch_est, 6)}",
                flush=True,
            )
            check_auto_abort(data["motor.m1"], data["motor.m2"], data["motor.m3"], data["motor.m4"])

        def log_diag_cb(timestamp, data, logconf):
            diag_state["pid_rate.yaw_outP"] = data["pid_rate.yaw_outP"]
            diag_state["pid_rate.yaw_outI"] = data["pid_rate.yaw_outI"]
            diag_state["pid_rate.yaw_outD"] = data["pid_rate.yaw_outD"]
            diag_state["gyro.z"] = data["gyro.z"]
            diag_state["controller.yawRate"] = data["controller.yawRate"]

        def log_att_cb(timestamp, data, logconf):
            diag_state["stateEstimate.yaw"] = data["stateEstimate.yaw"]
            diag_state["controller.cmd_yaw"] = data["controller.cmd_yaw"]
            diag_state["stateEstimate.roll"] = data["stateEstimate.roll"]
            diag_state["stateEstimate.pitch"] = data["stateEstimate.pitch"]

        lg.data_received_cb.add_callback(log_cb)
        lg_diag.data_received_cb.add_callback(log_diag_cb)
        lg_att.data_received_cb.add_callback(log_att_cb)
        lg.start()
        lg_diag.start()
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
            lg.stop()
            lg_diag.stop()
            lg_att.stop()
            return

        print("日志已确认在收，即将开始发送推力基线（斜坡爬升，避免电流冲击）。", flush=True)

        def log_is_stale():
            last = log_state["last"]
            if last is None:
                return False
            return (time.monotonic() - last) > LOG_STALE_TIMEOUT_S

        def setpoint_loop():
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
                # roll=pitch=0（水平），yawrate=0（定头，不主动转），thrust=基线；
                # 干扰完全靠你用手扭转机头产生，不是脚本发的。
                cf.commander.send_setpoint(0, 0, 0, BASE_THRUST)
                time.sleep(SEND_PERIOD)

        sp_thread = threading.Thread(target=setpoint_loop, daemon=True)

        print("\n即将开始：thrust 从 0 斜坡爬升到低速基线，roll/pitch/yawrate 设定点始终为 0（定头水平）。")
        print("电机转动、稳定几秒后，请缓慢、持续地把机头往左扭（俯视看机头从正前方转向左边，即 CCW）。")
        print("gyro.z 应该读到正值，代表你确实在往 CCW 方向转，先确认这个再看电机响应。")
        print("正确（负反馈）应该看到：cmd_yaw 变负，m1、m3（顺时针电机）转速降低，m2、m4 转速升高——")
        print("  即用减少顺时针电机反作用力矩的方式，产生把机头拉回去的顺时针纠正力矩。")
        print("如果是 bug（正反馈）会看到：cmd_yaw 变正，m1、m3 转速升高，m2、m4 转速降低——")
        print("  等于在帮你把机头往你转的方向继续推，而不是拉回来。")
        print("yawOutP(取负前) 和 cmd_yaw(取负后) 应该始终是相反号，这个不用看，代码里那行取负决定的。")
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
            lg_att.stop()
            print("测试结束。", flush=True)
    finally:
        cf.close_link()


if __name__ == "__main__":
    main()
