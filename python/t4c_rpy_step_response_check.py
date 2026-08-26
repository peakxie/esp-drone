#!/usr/bin/env python3
# Step C: 机身固定在支架上，脚本主动发送小幅 roll/pitch/yaw 阶跃（脉冲）指令，
# 看 motor.m1~m4 加减速的方向对不对。
#
# 和 t4b 的区别：t4b 是人手去掰机身、setpoint 一直保持水平 (0,0,0)，考察 PID 把
# 机身纠正回水平的响应方向；这里反过来，机身完全固定不动，脚本主动把 setpoint
# 阶跃成一个小角度/角速度，考察 mixer + PID 输出到 4 个电机的响应方向。两者可以
# 互相印证：如果结论对不上，说明至少一个环节（手动测试的动作方向 or 脚本的
# setpoint 符号）理解错了。
#
# 前提：桨叶已拆除，飞机已固定在支架上（转不动），全程有人盯着；thrust 只用一个
# 很低的基线值。
#
# 关键风险：roll/pitch 是角度模式（modeAbs），机身被支架卡住转不动，阶跃期间角度
# 误差会一直存在、不会像真实飞行那样收敛；yaw 是角速度模式，同理角速度误差也会
# 一直存在。误差持续存在会让角速度环积分一路 windup，跟 t4b 文件头注释里那次
# "夹具固定住转不动"的经历是同一个问题。
#
# 光靠"短脉冲+回中停留"并不够：固件只有在 control->thrust 精确等于 0 时才会调用
# attitudeControllerResetAllPID()（见 controller_pid.c），只要回中时 thrust 还保持
# 在 BASE_THRUST（非零），积分项就完全不会清零，上一个阶跃的残留会一直带进下一个
# 阶跃，实测已经验证过这会污染读数（尤其是链式跑完 roll 再跑 pitch 的时候）。所以
# 每个阶跃结束后，回中动作会真的把 thrust 打到 0 一瞬间（RESET_PULSE_S）触发硬件
# 复位积分，再斜坡爬回 BASE_THRUST（RESET_RAMP_S），最后才停留 STEP_REST_S
# 确认稳定——保证每个阶跃都是从干净的零积分状态起跑，互相不污染。
# MOTOR_ABORT_THRESHOLD / RATE_OUTP_ABORT_THRESHOLD 仍然作为最后一道硬保护。
#
# 注意：4 个电机同时起转（哪怕基线很低）比 t3 的单电机爬坡电流冲击大得多。如果
# 测试中出现"电机狂转、日志彻底不刷新、Ctrl+C 也止不住"，大概率是电流冲击把主控
# 拉到 brownout/看门狗复位了，请立刻断电，然后跑一次 t0_reset_reason.py 排查。

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
SETTLE_TIME_S = 1.0     # 爬升完到开始第一个阶跃之间的停留，确认基线稳定
SEND_PERIOD = 0.05      # 20Hz 发送 setpoint，避免 commander watchdog 超时进入 fallback

STEP_HOLD_S = 0.3       # 每个阶跃保持的时长：短脉冲，避免支架限位下的积分饱和
                        # （实测 8°/0.6s 单次阶跃内、纯外环积分爬升就能在 ~0.5s 撞到
                        #  RATE_OUTP_ABORT_THRESHOLD，跟跨阶跃污染无关，方向在前 1~2
                        #  个 tick 就已经看得很清楚，不需要保持这么久）
RESET_PULSE_S = 0.15    # 回中时把 thrust 打到 0 的时长，触发固件清零姿态/角速度环积分
RESET_RAMP_S = 0.3      # 从 0 斜坡爬回 BASE_THRUST 的时长，避免复位后再次阶跃电流冲击
STEP_REST_S = 0.5       # 爬回基线后再停留确认稳定的时长（此时积分已清零，不需要很长）
REPEAT_COUNT = 1        # 整套阶跃序列重复几遍，想多看几次响应可以调大

ROLL_STEP_DEG = 5.0     # roll 阶跃角度（角度模式）
PITCH_STEP_DEG = 5.0    # pitch 阶跃角度（角度模式）
YAW_STEP_DEGPS = 25.0   # yaw 阶跃角速度（速度模式）

LOG_WAIT_TIMEOUT_S = 2.0    # 等待第一帧 motor 日志的超时：等不到就说明遥测没通，绝不能盲发推力
LOG_STALE_TIMEOUT_S = 0.3   # 运行中超过这么久没收到新日志帧，视为链路/主控可能已经异常
ZERO_BURST_COUNT = 15       # 收尾/异常时连续发送零推力的次数，抵御偶发丢包
ZERO_BURST_PERIOD = 0.02

# 阈值含义同 t4b：MOTOR_ABORT_THRESHOLD 是电机 PWM 逼近满量程(65535)的硬保护线；
# RATE_OUTP_ABORT_THRESHOLD 是角速度环 outP 绝对值过大、基本可判定为积分饱和的预警线。
# 这里三个轴（roll/pitch/yaw）共用同一条预警线，任意一个轴触发都会自动收尾。
MOTOR_ABORT_THRESHOLD = 55000
RATE_OUTP_ABORT_THRESHOLD = 15000.0

# 期望方向来自 power_distribution_stock.c 的 QUAD_FORMATION_X 混控公式：
#   m1 = thrust - roll/2 + pitch/2 + yaw
#   m2 = thrust - roll/2 - pitch/2 - yaw
#   m3 = thrust + roll/2 - pitch/2 + yaw
#   m4 = thrust + roll/2 + pitch/2 - yaw
# （CONFIG_PITCH_DISTRIBUTION_INVERTED 未设置，pitch 未取反）
STEPS = [
    {
        "label": f"ROLL  +{ROLL_STEP_DEG:.0f}deg",
        "roll": ROLL_STEP_DEG, "pitch": 0.0, "yawrate": 0.0,
        "expect": "期望 m1,m2 变小；m3,m4 变大",
    },
    {
        "label": f"ROLL  -{ROLL_STEP_DEG:.0f}deg",
        "roll": -ROLL_STEP_DEG, "pitch": 0.0, "yawrate": 0.0,
        "expect": "期望 m1,m2 变大；m3,m4 变小",
    },
    {
        "label": f"PITCH +{PITCH_STEP_DEG:.0f}deg",
        "roll": 0.0, "pitch": PITCH_STEP_DEG, "yawrate": 0.0,
        "expect": "期望 m1,m4 变大；m2,m3 变小",
    },
    {
        "label": f"PITCH -{PITCH_STEP_DEG:.0f}deg",
        "roll": 0.0, "pitch": -PITCH_STEP_DEG, "yawrate": 0.0,
        "expect": "期望 m1,m4 变小；m2,m3 变大",
    },
    {
        "label": f"YAW   +{YAW_STEP_DEGPS:.0f}deg/s",
        "roll": 0.0, "pitch": 0.0, "yawrate": YAW_STEP_DEGPS,
        "expect": "期望 m1,m3 变大；m2,m4 变小",
    },
    {
        "label": f"YAW   -{YAW_STEP_DEGPS:.0f}deg/s",
        "roll": 0.0, "pitch": 0.0, "yawrate": -YAW_STEP_DEGPS,
        "expect": "期望 m1,m3 变小；m2,m4 变大",
    },
]


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
    """若 althold/poshold 开着，强制关掉，改用原始 thrust/roll/pitch/yaw setpoint。原因见 t4b：
    光流模块在线时固件会自动切到 POSHOLD_MODE，脚本发的阶跃指令会被丢弃、改用带固定前馈量
    的定高/定点速度 PID 输出，跟这里"发一个已知小角度阶跃"的意图完全不是一回事。"""
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

        # 提前建好 stop_event：给 setpoint 发送线程的退出信号、log 回调里的自动保护、
        # 以及阶跃序列跑完后的正常收尾复用，三边共享同一把停止开关。
        stop_event = threading.Event()

        # 确保不是 motorPowerSet 覆盖模式，走真实闭环
        cf.param.set_value("motorPowerSet.enable", "0")
        time.sleep(0.1)

        log_state = {"last": None}
        # 三个轴的角速度环 outP + 三个轴的实际姿态角，用于坐实"电机变化是不是阶跃直接打出来的"、
        # 以及给自动保护判断积分饱和用。跟 motor 日志分开一路，避免单个 log block 超出 CRTP payload 限制。
        diag_state = {
            "pid_rate.roll_outP": None,
            "pid_rate.pitch_outP": None,
            "pid_rate.yaw_outP": None,
            "stateEstimate.roll": None,
            "stateEstimate.pitch": None,
            "stateEstimate.yaw": None,
        }

        lg = LogConfig(name="motor", period_in_ms=50)
        lg.add_variable("motor.m1", "uint32_t")
        lg.add_variable("motor.m2", "uint32_t")
        lg.add_variable("motor.m3", "uint32_t")
        lg.add_variable("motor.m4", "uint32_t")
        cf.log.add_config(lg)

        lg_diag = LogConfig(name="pid_diag", period_in_ms=50)
        lg_diag.add_variable("pid_rate.roll_outP", "float")
        lg_diag.add_variable("pid_rate.pitch_outP", "float")
        lg_diag.add_variable("pid_rate.yaw_outP", "float")
        lg_diag.add_variable("stateEstimate.roll", "float")
        lg_diag.add_variable("stateEstimate.pitch", "float")
        lg_diag.add_variable("stateEstimate.yaw", "float")
        cf.log.add_config(lg_diag)

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
            for axis in ("roll", "pitch", "yaw"):
                outp = diag_state[f"pid_rate.{axis}_outP"]
                if outp is not None and abs(outp) >= RATE_OUTP_ABORT_THRESHOLD:
                    print(
                        f"\n警告：pid_rate.{axis}_outP={outp:.0f} 超过 {RATE_OUTP_ABORT_THRESHOLD:.0f}，"
                        f"疑似 {axis} 轴角速度环积分饱和(windup)——机身被支架卡住转不动导致误差一直"
                        "存在，自动停止并归零！即使每步都会回中复位积分，单次阶跃内仍可能因为支架"
                        "没放平/角度太大而饱和，请缩短 STEP_HOLD_S 或调小对应的 *_STEP_DEG(PS) 后再测。",
                        flush=True,
                    )
                    stop_event.set()
                    return

        def log_cb(timestamp, data, logconf):
            log_state["last"] = time.monotonic()
            rp = diag_state["pid_rate.roll_outP"]
            pp = diag_state["pid_rate.pitch_outP"]
            yp = diag_state["pid_rate.yaw_outP"]
            er = diag_state["stateEstimate.roll"]
            ep = diag_state["stateEstimate.pitch"]
            ey = diag_state["stateEstimate.yaw"]
            rp_s = f"{rp:7.0f}" if rp is not None else "    n/a"
            pp_s = f"{pp:7.0f}" if pp is not None else "    n/a"
            yp_s = f"{yp:7.0f}" if yp is not None else "    n/a"
            er_s = f"{er:6.2f}" if er is not None else "   n/a"
            ep_s = f"{ep:6.2f}" if ep is not None else "   n/a"
            ey_s = f"{ey:6.2f}" if ey is not None else "   n/a"
            print(
                f"t={timestamp:>8}  m1={data['motor.m1']:6d}  m2={data['motor.m2']:6d}  "
                f"m3={data['motor.m3']:6d}  m4={data['motor.m4']:6d}  |  "
                f"rateP(r,p,y)=({rp_s},{pp_s},{yp_s})  |  est(r,p,y)=({er_s},{ep_s},{ey_s})",
                flush=True,
            )
            check_auto_abort(data["motor.m1"], data["motor.m2"], data["motor.m3"], data["motor.m4"])

        def log_diag_cb(timestamp, data, logconf):
            diag_state["pid_rate.roll_outP"] = data["pid_rate.roll_outP"]
            diag_state["pid_rate.pitch_outP"] = data["pid_rate.pitch_outP"]
            diag_state["pid_rate.yaw_outP"] = data["pid_rate.yaw_outP"]
            diag_state["stateEstimate.roll"] = data["stateEstimate.roll"]
            diag_state["stateEstimate.pitch"] = data["stateEstimate.pitch"]
            diag_state["stateEstimate.yaw"] = data["stateEstimate.yaw"]

        lg.data_received_cb.add_callback(log_cb)
        lg_diag.data_received_cb.add_callback(log_diag_cb)
        lg.start()
        lg_diag.start()

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
            return

        print("日志已确认在收，即将开始发送推力基线（斜坡爬升，避免电流冲击）。", flush=True)

        def log_is_stale():
            last = log_state["last"]
            if last is None:
                return False
            return (time.monotonic() - last) > LOG_STALE_TIMEOUT_S

        def send_for(roll, pitch, yawrate, thrust, duration_s):
            """按 SEND_PERIOD 周期发送同一个 setpoint，直到 duration_s 用完或需要中止。
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
                cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
                time.sleep(SEND_PERIOD)
            return True

        def send_ramp(from_thrust, to_thrust, duration_s):
            """thrust 从 from_thrust 斜坡到 to_thrust，roll/pitch/yawrate 始终为 0，
            避免电机推力突变造成的电流冲击。返回 False 表示需要中止。"""
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

        def recenter_with_reset():
            """回中：真的把 thrust 打到 0 一瞬间，触发固件清零姿态/角速度环积分
            (controller_pid.c: control->thrust == 0 才会调用 attitudeControllerResetAllPID())，
            再斜坡爬回 BASE_THRUST，避免上一个阶跃的残留积分污染下一个阶跃的读数。"""
            print(f"    回中：thrust 打 0 触发积分复位（{RESET_PULSE_S:.2f}s）...", flush=True)
            if not send_for(0, 0, 0, 0, RESET_PULSE_S):
                return False
            if not send_ramp(0, BASE_THRUST, RESET_RAMP_S):
                return False
            print(f"    已回到基线 {BASE_THRUST}，停留 {STEP_REST_S:.1f}s 确认稳定...", flush=True)
            return send_for(0, 0, 0, BASE_THRUST, STEP_REST_S)

        def setpoint_loop():
            # 先发一次 thrust=0 解锁 thrust lock
            cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.1)

            if not send_ramp(0, BASE_THRUST, RAMP_TIME_S):
                return

            print(f"基线已到 {BASE_THRUST}，停留 {SETTLE_TIME_S:.1f}s 确认稳定...", flush=True)
            if not send_for(0, 0, 0, BASE_THRUST, SETTLE_TIME_S):
                return

            for round_idx in range(1, REPEAT_COUNT + 1):
                for step in STEPS:
                    print(
                        f"\n>>> [第{round_idx}轮] {step['label']}  保持 {STEP_HOLD_S:.1f}s  "
                        f"{step['expect']}",
                        flush=True,
                    )
                    if not send_for(step["roll"], step["pitch"], step["yawrate"], BASE_THRUST, STEP_HOLD_S):
                        return
                    if not recenter_with_reset():
                        return

            print("\n全部阶跃已跑完，正常结束测试。", flush=True)
            stop_event.set()

        sp_thread = threading.Thread(target=setpoint_loop, daemon=True)

        print("\n即将开始：thrust 从 0 斜坡爬升到低速基线，稳定后依次对 roll/pitch/yaw 各发一正一负")
        print("的短脉冲阶跃，机身应保持固定不动，只看 motor.m1~m4 的加减速方向对不对。")
        print("每个阶跃结束后会真的把 thrust 打到 0 一瞬间再爬回基线，触发固件清零积分，")
        print("避免上一个阶跃的残留污染下一个阶跃的读数（这个过程中电机会短暂停一下，是预期行为）。")
        print("rateP(r,p,y) 是三个轴角速度环的 P 分量，用来判断是不是积分饱和触发了自动保护。")
        print("est(r,p,y) 是姿态估计的实际角度：机身固定在支架上应该基本不动（接近阶跃前的基线），")
        print("如果阶跃时 est 也跟着明显偏离，说明支架不是刚性固定、有间隙/形变，会影响结论。")
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
            lg.stop()
            lg_diag.stop()
            print("测试结束。", flush=True)


if __name__ == "__main__":
    main()
