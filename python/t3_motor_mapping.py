#!/usr/bin/env python3
# T3 电机映射测试：逐个给单个电机做 10000->60000->0 的匀速爬升/回落，
# 人工确认物理位置，同时回读固件实际下发的 PWM 占空比验证是否与设定同步变化。
# 前提：桨叶已拆除，飞机已固定，全程有人盯着。

import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

from config import URI, connect_with_timeout

TEST_MIN = 10000  # 0~65535，斜坡起点：明显能看到转动但不会太猛
TEST_MAX = 60000  # 斜坡终点
TEST_STEP = 10000  # 每级步长
STEP_HOLD_S = 1.0  # 每级停留时间，留出观察+回读窗口
MOTORS = ["m1", "m2", "m3", "m4"]

# 爬升到 TEST_MAX 后原路回落到 0，形成一次完整的速度扫描序列
_up = list(range(TEST_MIN, TEST_MAX + 1, TEST_STEP))
RAMP_SEQUENCE = _up + list(reversed(_up[:-1])) + [0]


def set_motor(cf, motor, value):
    cf.param.set_value(f"motorPowerSet.{motor}", str(value))


def zero_all(cf):
    for m in MOTORS:
        set_motor(cf, m, 0)


def ramp_motor_with_verify(cf, motor, sequence, hold_s):
    """按 sequence 逐级设定单个电机，并回读 pwm.{motor}_pwm 校验实际下发值。

    注意：不能用 motor.{m1..m4} 这组日志做回读——motorPowerSet.enable=1 时，
    固件的 powerDistribution() 直接绕过飞控输出走 motorsSetRatio()，不会刷新
    motorPower.m1..m4，那组日志会停留在旧的飞控值上。真正反映“最终下发给
    电机驱动”的是 motors.c 里的 motor_ratios[]，对应日志变量是 pwm.{m}_pwm。
    """
    var = f"pwm.{motor}_pwm"
    actual = {"value": None}

    lg = LogConfig(name=f"mmap_{motor}", period_in_ms=50)
    lg.add_variable(var, "uint32_t")

    def cb(timestamp, data, logconf):
        actual["value"] = data[var]

    lg.data_received_cb.add_callback(cb)
    cf.log.add_config(lg)
    lg.start()
    time.sleep(0.1)  # 等第一帧日志回来，避免第一级读到 None

    try:
        for value in sequence:
            set_motor(cf, motor, value)
            time.sleep(hold_s)
            got = actual["value"]
            mark = "OK" if got == value else "!! 不一致，检查电机/PWM链路"
            print(f"    设定={value:>5}  回读实际PWM={got!s:>5}  {mark}")
    finally:
        set_motor(cf, motor, 0)
        time.sleep(0.1)
        lg.stop()


def main():
    cflib.crtp.init_drivers()

    print("再次确认：桨叶已拆除、飞机已固定、你会全程盯着电机。")
    if input("确认无误请输入 yes 继续: ").strip().lower() != "yes":
        print("已取消，未连接。")
        return

    cf = Crazyflie()
    if not connect_with_timeout(cf, URI):
        return

    try:
        try:
            cf.param.set_value("motorPowerSet.enable", "1")
            time.sleep(0.2)
            print("motorPowerSet.enable = 1，已绕开飞控，接下来逐个测试电机。\n")

            for motor in MOTORS:
                input(f"按 Enter 开始测试 {motor}（其余电机保持 0）...")
                print(
                    f"  -> {motor} 将从 {TEST_MIN} 匀速升到 {TEST_MAX} 再降回 0，"
                    f"请观察对应物理电机转速是否随下方设定值同步变化："
                )
                ramp_motor_with_verify(cf, motor, RAMP_SEQUENCE, STEP_HOLD_S)
                print(f"  -> {motor} 测试完毕，已归零。\n")

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
    finally:
        cf.close_link()


if __name__ == "__main__":
    main()
