#!/usr/bin/env python3
# T4：传感器读数检查（纯读，不碰电机/commander，零风险）
# 订阅 acc/gyro/mag/baro/range/motion 全部打印出来，方便对照预期现象手动验证。

import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = "udp://192.168.43.42:2390"  # 换成你能连上的地址


def main():
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI) as scf:
        cf = scf.cf
        print("已连接。\n")

        # CRTP 单个 LOG 包 payload 有限（约 26 字节），全部变量分三组订阅。
        imu_lg = LogConfig(name="imu", period_in_ms=100)
        for v in ("acc.x", "acc.y", "acc.z", "gyro.x", "gyro.y", "gyro.z"):
            imu_lg.add_variable(v, "float")

        env_lg = LogConfig(name="env", period_in_ms=200)
        for v in ("mag.x", "mag.y", "mag.z"):
            env_lg.add_variable(v, "float")
        env_lg.add_variable("baro.asl", "float")

        aux_lg = LogConfig(name="aux", period_in_ms=100)
        aux_lg.add_variable("range.zrange", "uint16_t")
        aux_lg.add_variable("motion.deltaX", "int16_t")
        aux_lg.add_variable("motion.deltaY", "int16_t")

        for lg in (imu_lg, env_lg, aux_lg):
            cf.log.add_config(lg)

        def imu_cb(timestamp, data, logconf):
            print(
                f"[imu] t={timestamp:>8}  "
                f"acc=({data['acc.x']:+.3f},{data['acc.y']:+.3f},{data['acc.z']:+.3f})g  "
                f"gyro=({data['gyro.x']:+7.2f},{data['gyro.y']:+7.2f},{data['gyro.z']:+7.2f})deg/s"
            )

        def env_cb(timestamp, data, logconf):
            print(
                f"[env] t={timestamp:>8}  "
                f"mag=({data['mag.x']:+.3f},{data['mag.y']:+.3f},{data['mag.z']:+.3f})  "
                f"baro.asl={data['baro.asl']:.2f}m"
            )

        def aux_cb(timestamp, data, logconf):
            print(
                f"[aux] t={timestamp:>8}  "
                f"range.zrange={data['range.zrange']:5d}mm  "
                f"motion=({data['motion.deltaX']:+5d},{data['motion.deltaY']:+5d})"
            )

        imu_lg.data_received_cb.add_callback(imu_cb)
        env_lg.data_received_cb.add_callback(env_cb)
        aux_lg.data_received_cb.add_callback(aux_cb)

        imu_lg.start()
        env_lg.start()
        aux_lg.start()

        print("开始打印，按 Ctrl+C 结束。建议依次做：")
        print("  1. 水平桌面静置：acc.z 应 ~1.0g，acc.x/y 应 ~0；偏差用 PITCH_CALIB/ROLL_CALIB 校正")
        print("  2. 绕某轴缓慢转动机身：对应 gyro 轴应有明显变化，其余两轴基本不动")
        print("  3. 缓慢抬高整机：baro.asl 应单调增大")
        print("  4. 手掌从远到近伸到机身下方：range.zrange 应单调减小到几十 mm 量级")
        print("  5. 手托机身做水平平移：motion.deltaX/deltaY 应有脉冲，方向要和平移方向对应")
        print("     （如果反了，是光流贴装朝向问题，flowdeck_v1v2.c 已做过一次 -deltaX/-deltaY，")
        print("      不要在这里再加一次翻转，先去核对贴装方向）\n")

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            imu_lg.stop()
            env_lg.stop()
            aux_lg.stop()
            print("\n测试结束。")


if __name__ == "__main__":
    main()
