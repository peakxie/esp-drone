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
#
# 除了原始 deltaX/deltaY，每一段还会额外记录 stateEstimate.vx/vy 在这一段里的峰值（带符号）。
# 这是比手算 dpixelx/dpixely 更直接、更值得信的证据：vx/vy 是 EKF 融合光流之后真正算出来的
# 机体速度估计，也是 position_controller_pid.c 里直接拿去做闭环反馈的那个数——不用我们手动
# 套 dpixelx=-deltaY 这层公式再去猜前后层的坐标系翻转，直接看"推一下、EKF 是不是也认为自己在
# 往对的方向动、动多快"，这才是最终结果，跳过了中间所有可能算错的环节。
# 前提：这部分只有 stabilizer.estimator==2（kalman）时才有意义，本脚本会在开始时检查并提示。

import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

from config import URI, connect_with_timeout

VALID_MOTION_BYTE = 0xB0  # 跟固件 flowdeck_v1v2.c 判断"这一帧可以喂给 EKF"的条件完全一致
ESTIMATOR_NAMES = {0: "any", 1: "complementary", 2: "kalman"}


def read_current_estimator(cf, timeout_s=2.0):
    got_value = threading.Event()
    holder = {"value": None}

    def estimator_cb(_name, value):
        holder["value"] = int(value)
        got_value.set()

    cf.param.add_update_callback(group="stabilizer", name="estimator", cb=estimator_cb)
    cf.param.request_param_update("stabilizer.estimator")
    got_value.wait(timeout=timeout_s)

    if holder["value"] is None:
        print("警告：读取 stabilizer.estimator 超时，未确认当前估计器。", flush=True)
    else:
        label = ESTIMATOR_NAMES.get(holder["value"], "unknown")
        print(f"当前 stabilizer.estimator = {holder['value']} ({label})", flush=True)

    return holder["value"]


def main():
    cflib.crtp.init_drivers()

    cf = Crazyflie()
    if not connect_with_timeout(cf, URI):
        return

    lock = threading.Lock()
    accum = {
        "dx": 0, "dy": 0, "valid": 0, "invalid": 0, "last_squal": None,
        "vx_peak": 0.0, "vy_peak": 0.0,  # 这一段里绝对值最大的 vx/vy（带符号）
    }

    def flow_cb(_timestamp, data, _logconf):
        with lock:
            accum["last_squal"] = data["motion.squal"]
            if data["motion.motion"] == VALID_MOTION_BYTE:
                accum["valid"] += 1
                accum["dx"] += data["motion.deltaX"]
                accum["dy"] += data["motion.deltaY"]
            else:
                accum["invalid"] += 1

    def vel_cb(_timestamp, data, _logconf):
        vx, vy = data["stateEstimate.vx"], data["stateEstimate.vy"]
        with lock:
            if abs(vx) > abs(accum["vx_peak"]):
                accum["vx_peak"] = vx
            if abs(vy) > abs(accum["vy_peak"]):
                accum["vy_peak"] = vy

    flow_lg = LogConfig(name="flow", period_in_ms=30)
    for v, t in (("motion.motion", "uint8_t"), ("motion.deltaX", "int16_t"),
                 ("motion.deltaY", "int16_t"), ("motion.squal", "uint8_t")):
        flow_lg.add_variable(v, t)
    cf.log.add_config(flow_lg)
    flow_lg.data_received_cb.add_callback(flow_cb)

    vel_lg = LogConfig(name="vel", period_in_ms=30)
    vel_lg.add_variable("stateEstimate.vx", "float")
    vel_lg.add_variable("stateEstimate.vy", "float")
    cf.log.add_config(vel_lg)
    vel_lg.data_received_cb.add_callback(vel_cb)

    results = []
    show_vel = False

    try:
        estimator = read_current_estimator(cf)
        show_vel = (estimator == 2)
        if not show_vel:
            print(
                "提示：当前不是 kalman，stateEstimate.vx/vy 不会反映光流的效果，下面只记录"
                "原始 deltaX/deltaY，vx/vy 那部分跳过。",
                flush=True,
            )

        flow_lg.start()
        if show_vel:
            vel_lg.start()
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
                accum["vx_peak"] = accum["vy_peak"] = 0.0

            input(f"[{label}] 回车开始计这一段，然后开始做动作...")
            t0 = time.monotonic()

            input(f"[{label}] 动作中，做完后回车结束这一段...")
            elapsed = time.monotonic() - t0

            with lock:
                dx, dy = accum["dx"], accum["dy"]
                valid, invalid = accum["valid"], accum["invalid"]
                squal = accum["last_squal"]
                vx_peak, vy_peak = accum["vx_peak"], accum["vy_peak"]

            print(
                f"  -> [{label}] 用时{elapsed:.1f}s  raw累加=(deltaX={dx:+d}, deltaY={dy:+d})  "
                f"valid/invalid帧数={valid}/{invalid}  squal={squal}"
            )
            if show_vel:
                print(f"     stateEstimate 峰值：vx={vx_peak:+.3f}m/s  vy={vy_peak:+.3f}m/s\n")
            else:
                print("")
            results.append((label, dx, dy, valid, invalid, vx_peak, vy_peak))

    except KeyboardInterrupt:
        pass
    finally:
        flow_lg.stop()
        if show_vel:
            vel_lg.stop()
        cf.close_link()

    if results:
        print("\n===== 汇总 =====")
        print(f"{'label':<16}{'deltaX累加':>12}{'deltaY累加':>12}{'vx峰值':>10}{'vy峰值':>10}{'valid':>8}{'invalid':>8}")
        for label, dx, dy, valid, invalid, vx_peak, vy_peak in results:
            print(f"{label:<16}{dx:>12d}{dy:>12d}{vx_peak:>10.3f}{vy_peak:>10.3f}{valid:>8d}{invalid:>8d}")
        print(
            "\n核对方法（两步，第 2 步更可信）：\n"
            "1. 同一物理方向的正反两次动作（比如 forward/back），deltaX 或 deltaY（应该是同一个"
            "分量在变化，另一个分量接近 0）符号应该正好相反；对不上说明这段数据混入了干扰，重做。\n"
            "2. 直接看 vx/vy 峰值：你朝哪个方向推，对应的那个峰值符号应该和你推的方向一致——\n"
            "   比如你确定是朝机头方向（前）推，vx 应该报正还是报负，取决于固件对'前'的定义，"
            "但至少方向感要跟你脑子里认为的'前进=正'或'前进=负'保持前后一致、可预测；\n"
            "   如果峰值符号和你预期的方向反过来、或者悬停不推它自己也会朝某个方向持续报出非零"
            "速度（虚假速度），那就是光流方向/融合有问题，对应去改 flowdeck_v1v2.c 里\n"
            "   dpixelx = -deltaY   (喂给 EKF 机体 X 方向 KC_STATE_PX 的那路)\n"
            "   dpixely = -deltaX   (喂给 EKF 机体 Y 方向 KC_STATE_PY 的那路)\n"
            "   这两行的符号或者对应关系。"
        )


if __name__ == "__main__":
    main()
