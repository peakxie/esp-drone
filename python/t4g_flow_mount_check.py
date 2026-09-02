#!/usr/bin/env python3
# T4g：光流贴装方向核对（交互式打点版，取代 t4f 里"连续刷屏、看不清对应哪一段推动"的问题）。
# 纯读 motion.* 日志，不碰电机/commander，零风险。
#
# 和 t4f_flow_range_check.py 的区别：
#   t4f 是持续按时间窗自动累加打印，脚本和人的动作没有精确对齐，很难确定"这个数到底对应
#   我刚才推的哪一下"。这里改成人工打点：你按一下 Enter 开始计一段，做一个动作，再按一次
#   Enter 结束，脚本只打印这一段里的累加值，一段一个动作，界限清清楚楚。
#
# 这里只看"原始"读数（motion.deltaX / motion.deltaY，PMW3901 寄存器坐标，翻转前），不看
# flowdeck_v1v2.c 里为了套用 EKF 而做的那次 dpixelx=-deltaY, dpixely=-deltaX 变换——先把
# "传感器实际怎么装的"这件事本身测清楚，再去核对代码里那次变换对不对，两件事分开做，
# 不要混在一起看，否则更容易看花。
#
# 强烈建议这样做，而不是徒手悬空移动：
#   把飞机整体放在一块有明显纹理的桌面/地面上方几厘米（贴着能贴的高度，不要托着悬空——
#   悬空的手部晃动会同时引入旋转，混进 omegax_b/omegay_b 补偿项，污染平移这一路的信号），
#   保持水平姿态和高度基本不变，只做纯平移滑动。每次移动方向、速度尽量保持一致，
#   移动幅度大一点（20~30cm）、匀速推过去，这样单段的累加值明显大于噪声。
#
# 操作：
#   1. 先给这一段取个名字（比如 "forward" "back" "left" "right"，名字你自己定义，只要
#      每次同一个动作用同一个名字，前后一致就行），回车开始计这一段。
#   2. 做动作（推/拉）。
#   3. 再回车一次结束这一段，脚本打印这一段的 raw (deltaX, deltaY) 累加值。
#   4. 重复步骤 1~3，同一个方向至少做正反两次（比如 forward 和 back）互相印证——
#      如果真的是同一个物理方向的往复，两次的累加值符号应该正好相反；如果对不上，
#      说明本次操作里混入了别的干扰（旋转/高度变化/手抖），这一段数据不可信，重做。
#   5. 输入 q 并回车结束整个脚本，会打印一张汇总表，把所有段放在一起对比。

import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

from config import URI, connect_with_timeout

VALID_MOTION_BYTE = 0xB0  # 跟固件 flowdeck_v1v2.c 判断"这一帧可以喂给 EKF"的条件完全一致


def main():
    cflib.crtp.init_drivers()

    cf = Crazyflie()
    if not connect_with_timeout(cf, URI):
        return

    lock = threading.Lock()
    accum = {"dx": 0, "dy": 0, "valid": 0, "invalid": 0, "last_squal": None}

    def flow_cb(_timestamp, data, _logconf):
        with lock:
            accum["last_squal"] = data["motion.squal"]
            if data["motion.motion"] == VALID_MOTION_BYTE:
                accum["valid"] += 1
                accum["dx"] += data["motion.deltaX"]
                accum["dy"] += data["motion.deltaY"]
            else:
                accum["invalid"] += 1

    flow_lg = LogConfig(name="flow", period_in_ms=30)
    for v, t in (("motion.motion", "uint8_t"), ("motion.deltaX", "int16_t"),
                 ("motion.deltaY", "int16_t"), ("motion.squal", "uint8_t")):
        flow_lg.add_variable(v, t)
    cf.log.add_config(flow_lg)
    flow_lg.data_received_cb.add_callback(flow_cb)

    results = []

    try:
        flow_lg.start()
        print("\n已开始订阅光流日志。操作方法见脚本头部注释。\n", flush=True)

        while True:
            label = input("给这一段动作取个名字（q 结束）: ").strip()
            if label.lower() == "q":
                break
            if not label:
                print("名字不能为空，重新输入。")
                continue

            with lock:
                accum["dx"] = accum["dy"] = 0
                accum["valid"] = accum["invalid"] = 0

            input(f"[{label}] 回车开始计这一段，然后开始做动作...")
            t0 = time.monotonic()

            input(f"[{label}] 动作中，做完后回车结束这一段...")
            elapsed = time.monotonic() - t0

            with lock:
                dx, dy = accum["dx"], accum["dy"]
                valid, invalid = accum["valid"], accum["invalid"]
                squal = accum["last_squal"]

            print(
                f"  -> [{label}] 用时{elapsed:.1f}s  raw累加=(deltaX={dx:+d}, deltaY={dy:+d})  "
                f"valid/invalid帧数={valid}/{invalid}  squal={squal}\n"
            )
            results.append((label, dx, dy, valid, invalid))

    except KeyboardInterrupt:
        pass
    finally:
        flow_lg.stop()
        cf.close_link()

    if results:
        print("\n===== 汇总（原始 deltaX/deltaY，未做任何翻转） =====")
        print(f"{'label':<16}{'deltaX累加':>12}{'deltaY累加':>12}{'valid':>8}{'invalid':>8}")
        for label, dx, dy, valid, invalid in results:
            print(f"{label:<16}{dx:>12d}{dy:>12d}{valid:>8d}{invalid:>8d}")
        print(
            "\n核对方法：同一物理方向的正反两次动作（比如 forward/back），deltaX 或 deltaY "
            "（应该是同一个分量在变化，另一个分量应该接近 0）符号应该正好相反。确定了哪个分量"
            "对应哪个物理方向、正方向是哪边之后，再去跟 flowdeck_v1v2.c 里\n"
            "  dpixelx = -deltaY   (对应 EKF 的机体 X/前进方向 KC_STATE_PX)\n"
            "  dpixely = -deltaX   (对应 EKF 的机体 Y/侧向 KC_STATE_PY)\n"
            "这次翻转核对：如果测出来的物理方向和这次翻转对不上，就是这两行需要改的地方。"
        )


if __name__ == "__main__":
    main()
