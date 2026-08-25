#!/usr/bin/env python3
# Step A / T4 前置检查：只读姿态估计（stabilizer.pitch/roll/yaw），
# 手动倾斜机身，看轴向和符号对不对。
# 不碰 motorPowerSet、不发 commander，电机完全不需要转，零风险。

import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from config import URI


def main():
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI) as scf:
        cf = scf.cf
        print("已连接。")

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


if __name__ == "__main__":
    main()
