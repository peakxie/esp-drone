#!/usr/bin/env python3
# Step A / T4 前置检查：只读姿态估计（stabilizer.pitch/roll/yaw），
# 手动倾斜机身，看轴向和符号对不对。
# 不碰 motorPowerSet、不发 commander，电机完全不需要转，零风险。

import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

from config import URI, connect_with_timeout

ESTIMATOR_NAMES = {0: "any", 1: "complementary", 2: "kalman"}


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
        print(f"当前 stabilizer.estimator = {value} ({label})")
        result["value"] = val
        got_value.set()

    cf.param.add_update_callback(group="stabilizer", name="estimator", cb=estimator_cb)
    cf.param.request_param_update("stabilizer.estimator")

    if not got_value.wait(timeout=timeout_s):
        print("警告：读取 stabilizer.estimator 超时，未确认当前估计器。")
        return None
    return result["value"]


def ensure_complementary_estimator(cf):
    """若当前是 kalman(2)，强制切成 complementary(1)，避免光流模块把估计器锁死成 kalman。"""
    current = read_current_estimator(cf)
    if current == 2:
        print("检测到当前是 kalman，强制切换为 complementary...")
        cf.param.set_value("stabilizer.estimator", "1")
        time.sleep(0.5)
        read_current_estimator(cf)


def main():
    cflib.crtp.init_drivers()

    cf = Crazyflie()
    if not connect_with_timeout(cf, URI):
        return

    try:
        ensure_complementary_estimator(cf)

        lg = LogConfig(name="attitude", period_in_ms=100)
        lg.add_variable("stabilizer.roll", "float")
        lg.add_variable("stabilizer.pitch", "float")
        lg.add_variable("stabilizer.yaw", "float")

        cf.log.add_config(lg)

        def log_cb(timestamp, data, logconf):
            print(
                f"t={timestamp:>8}  roll={data['stabilizer.roll']:7.2f}  "
                f"pitch={data['stabilizer.pitch']:7.2f}  yaw={data['stabilizer.yaw']:7.2f}"
            )

        lg.data_received_cb.add_callback(log_cb)
        lg.start()

        print("\n开始打印姿态估计，按 Ctrl+C 结束。建议依次做：")
        print("  1. 水平静置几秒，记录基线（应接近 0,0）")
        print("  2. 只做抬头/低头动作（绕 pitch 轴倾斜），看 pitch 是否变化、roll 是否基本不动")
        print("  3. 回正后只做左右倾斜（绕 roll 轴），看 roll 是否变化、pitch 是否基本不动")
        print("  4. 记录：抬头时 pitch 是正还是负；机身右倾时 roll 是正还是负\n")

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            lg.stop()
            print("\n测试结束。")
    finally:
        cf.close_link()


if __name__ == "__main__":
    main()
