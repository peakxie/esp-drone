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

URI = "udp://192.168.43.42:2390"  # 换成你能连上的地址

BASE_THRUST = 15000     # 低速基线，只是为了在 idleThrust 之上留出可观察的差量
RAMP_TIME_S = 1.0       # 从 0 斜坡爬升到 BASE_THRUST 的时长，避免 4 电机同时起转的电流冲击
SEND_PERIOD = 0.05      # 20Hz 发送 setpoint，避免 commander watchdog 超时进入 fallback
LOG_WAIT_TIMEOUT_S = 2.0    # 等待第一帧 motor 日志的超时：等不到就说明遥测没通，绝不能盲发推力
LOG_STALE_TIMEOUT_S = 0.3   # 运行中超过这么久没收到新日志帧，视为链路/主控可能已经异常
ZERO_BURST_COUNT = 15       # 收尾/异常时连续发送零推力的次数，抵御偶发丢包
ZERO_BURST_PERIOD = 0.02


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

        # 确保不是 motorPowerSet 覆盖模式，走真实闭环
        cf.param.set_value("motorPowerSet.enable", "0")
        time.sleep(0.1)

        log_state = {"last": None}

        lg = LogConfig(name="motor", period_in_ms=50)
        lg.add_variable("motor.m1", "uint32_t")
        lg.add_variable("motor.m2", "uint32_t")
        lg.add_variable("motor.m3", "uint32_t")
        lg.add_variable("motor.m4", "uint32_t")
        cf.log.add_config(lg)

        def log_cb(timestamp, data, logconf):
            log_state["last"] = time.monotonic()
            print(
                f"t={timestamp:>8}  m1={data['motor.m1']:6d}  m2={data['motor.m2']:6d}  "
                f"m3={data['motor.m3']:6d}  m4={data['motor.m4']:6d}",
                flush=True,
            )

        lg.data_received_cb.add_callback(log_cb)
        lg.start()

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
            return

        print("日志已确认在收，即将开始发送推力基线（斜坡爬升，避免电流冲击）。", flush=True)

        stop_event = threading.Event()

        def setpoint_loop():
            # 先发一次 thrust=0 解锁 thrust lock
            cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.1)

            ramp_steps = max(1, int(RAMP_TIME_S / SEND_PERIOD))
            for i in range(1, ramp_steps + 1):
                if stop_event.is_set() or link_lost.is_set():
                    return
                thrust = int(BASE_THRUST * i / ramp_steps)
                cf.commander.send_setpoint(0, 0, 0, thrust)
                time.sleep(SEND_PERIOD)

            while not stop_event.is_set() and not link_lost.is_set():
                last = log_state["last"]
                if last is not None and (time.monotonic() - last) > LOG_STALE_TIMEOUT_S:
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
            print("测试结束。", flush=True)


if __name__ == "__main__":
    main()
