#!/usr/bin/env python3
# T3 电机映射测试：逐个给单个电机加速，人工确认物理位置。
# 前提：桨叶已拆除，飞机已固定，全程有人盯着。

import time

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = "udp://192.168.43.42:2390"  # 换成你能正常连接上的地址

TEST_VALUE = 10000  # 0~65535，先用一个明显能看到转动但不会太猛的值，可按需调整
MOTORS = ["m1", "m2", "m3", "m4"]


def set_motor(cf, motor, value):
    cf.param.set_value(f"motorPowerSet.{motor}", str(value))


def zero_all(cf):
    for m in MOTORS:
        set_motor(cf, m, 0)


def main():
    cflib.crtp.init_drivers()

    print("再次确认：桨叶已拆除、飞机已固定、你会全程盯着电机。")
    if input("确认无误请输入 yes 继续: ").strip().lower() != "yes":
        print("已取消，未连接。")
        return

    with SyncCrazyflie(URI) as scf:
        cf = scf.cf
        print("已连接。")

        try:
            cf.param.set_value("motorPowerSet.enable", "1")
            time.sleep(0.2)
            print("motorPowerSet.enable = 1，已绕开飞控，接下来逐个测试电机。\n")

            for motor in MOTORS:
                input(f"按 Enter 开始测试 {motor}（其余电机保持 0）...")
                set_motor(cf, motor, TEST_VALUE)
                print(f"  -> {motor} 已设为 {TEST_VALUE}，请观察是哪个物理电机在转。")
                input("  观察完毕，按 Enter 停止该电机并进入下一个...")
                set_motor(cf, motor, 0)
                print(f"  -> {motor} 已归零。\n")

            print("四个电机测试完毕。")

        finally:
            print("正在归零所有电机，交还控制权给飞控...")
            try:
                zero_all(cf)
                cf.param.set_value("motorPowerSet.enable", "0")
                time.sleep(0.2)
                print("motorPowerSet.enable = 0，已交还飞控。")
            except Exception as exc:
                print(f"归零/交还飞控时出现异常，请立刻断电检查！错误: {exc}")


if __name__ == "__main__":
    main()
