#!/usr/bin/env python3
# Step B: 闭环下人工制造 pitch/roll 干扰，看 motor.m1~m4 响应方向对不对。
# 保持水平设定点 (0,0,0)，用手倾斜机身；PID 会尝试把它纠正回水平。
# 前提：桨叶已拆除，飞机已固定，全程有人盯着；thrust 只用一个很低的基线值。

import threading
import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = "udp://192.168.43.42:2390"  # 换成你能连上的地址

BASE_THRUST = 15000  # 低速基线，只是为了在 idleThrust 之上留出可观察的差量
SEND_PERIOD = 0.05   # 20Hz 发送 setpoint，避免 commander watchdog 超时进入 fallback


def main():
    cflib.crtp.init_drivers()

    print("再次确认：桨叶已拆除、飞机已固定、你会全程盯着电机。")
    if input("确认无误请输入 yes 继续: ").strip().lower() != "yes":
        print("已取消。")
        return

    with SyncCrazyflie(URI) as scf:
        cf = scf.cf
        print("已连接。")

        # 确保不是 motorPowerSet 覆盖模式，走真实闭环
        cf.param.set_value("motorPowerSet.enable", "0")
        time.sleep(0.1)

        lg = LogConfig(name="motor", period_in_ms=50)
        lg.add_variable("motor.m1", "uint32_t")
        lg.add_variable("motor.m2", "uint32_t")
        lg.add_variable("motor.m3", "uint32_t")
        lg.add_variable("motor.m4", "uint32_t")
        cf.log.add_config(lg)

        def log_cb(timestamp, data, logconf):
            print(
                f"t={timestamp:>8}  m1={data['motor.m1']:6d}  m2={data['motor.m2']:6d}  "
                f"m3={data['motor.m3']:6d}  m4={data['motor.m4']:6d}"
            )

        lg.data_received_cb.add_callback(log_cb)
        lg.start()

        stop_event = threading.Event()

        def setpoint_loop():
            # 先发一次 thrust=0 解锁 thrust lock
            cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.1)
            while not stop_event.is_set():
                cf.commander.send_setpoint(0, 0, 0, BASE_THRUST)
                time.sleep(SEND_PERIOD)
            # 结束前多发几次 0 thrust，确保电机停下并重新锁定
            for _ in range(5):
                cf.commander.send_setpoint(0, 0, 0, 0)
                time.sleep(SEND_PERIOD)

        sp_thread = threading.Thread(target=setpoint_loop, daemon=True)

        print("\n即将开始：thrust 保持低速基线，pitch/roll 设定点始终为 0（水平）。")
        print("电机转动、稳定几秒后，请缓慢抬起机头，观察打印：")
        print("  期望：m1、m4（前）变小，m2、m3（后）变大")
        print("  再缓慢低头，期望反过来：m1、m4 变大，m2、m3 变小")
        print("按 Ctrl+C 结束测试。\n")

        sp_thread.start()

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            print("\n正在停止并归零...")
            stop_event.set()
            sp_thread.join(timeout=2)
            lg.stop()
            print("测试结束。")


if __name__ == "__main__":
    main()
