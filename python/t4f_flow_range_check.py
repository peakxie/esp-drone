#!/usr/bin/env python3
# T4f：光流 + 测距传感器读数检查（纯读，不碰电机/commander，零风险，手持即可）。
#
# 目的：在真正靠光流+测距做定高/定点悬停（t5_hover_land.py）之前，先用真实数据核对两件事：
#   1. range.zrange 的高度方向对不对（抬高应该变大，靠近地面应该变小）。
#   2. 光流的方向对不对——而且是核对"喂给 EKF 的那个值"的方向，不是传感器原始寄存器的方向。
#
# 关于第 2 点，光流方向不能只看 motion.deltaX/deltaY 原始值：
#   flowdeck_v1v2.c 里 flowdeckTask() 读到 currentMotion.deltaX/deltaY 之后，会先做一次
#   accpx = -currentMotion.deltaY; accpy = -currentMotion.deltaX;
#   （"Flip motion information to comply with sensor mounting"），然后才把 accpx/accpy
#   包成 dpixelx/dpixely 送进 estimatorEnqueueFlow() 喂给 EKF（kalmanCoreUpdateWithFlow）。
#   这次坐标交换+取负只发生在 flowdeckTask() 的局部变量里，LOG_GROUP(motion) 里能读到的
#   deltaX/deltaY 是翻转*之前*的原始寄存器值，两者方向可能不一样。
#   本脚本里的"原始 deltaX/deltaY"和"EKF 实际看到的 dpixelx/dpixely"分开打印，就是为了不要
#   把"看着原始值顺眼"误判成"喂给 EKF 的方向也对"——这正是 t4b/t4c/t4e 系列反复强调的教训
#   （拿代码推导出的方向去验证代码自己，等于没验证；只信真实测出来的信号）。
#
# 光流的瞬时读数在慢速移动时噪声占比很大，逐帧打印很难看出方向，所以本脚本按
# ACCUM_WINDOW_S 时间窗把 dpixelx/dpixely（以及原始 deltaX/deltaY）累加求和再打印：
# 操作手法是"先静止 ~1 个窗口，再朝一个方向连续推动 ~1~2 个窗口，再静止"，看哪个窗口的
# 累加值明显偏离 0、偏向哪个符号，比看瞬时值可靠得多。累加只统计 motion.motion == 0xB0
# 的帧——这跟固件 flowdeckTask() 里真正推给 EKF 时的有效性判断条件完全一致（无效帧固件也
# 不会用，这里跟着一起丢弃，否则累加值会被无效帧的噪声污染）。
#
# 建议操作顺序（每步之间停顿一下，方便对照打印出的窗口边界）：
#   1. 手持飞机，水平桌面上方几十厘米静止不动：观察 range.zrange 是否稳定（噪声应该很小），
#      dpixelx/dpixely 窗口累加值应接近 0。
#   2. 缓慢竖直抬高整机（不要水平移动）：range.zrange 应单调增大（离地更远）；
#      stateEstimate.z 如果在动（说明 kalman 在用这个高度更新状态）应该同步增大。
#      缓慢下降则反过来，range.zrange 应单调减小。
#   3. 保持高度不变，机头方向不变，将整机沿机头朝向的正前方水平推动一段距离：记下这个窗口
#      dpixelx/dpixely 的符号，跟你推动的方向对应起来（不用猜该是哪个符号对——先测出来，
#      往回推（负方向）应该正好翻转符号，重复验证比单次更可信）。
#   4. 同样方式测左右方向的水平推动。
#   5. 如果当前 stabilizer.estimator == 2（kalman，正常在有光流的情况下应该是这个），
#      本脚本还会打印 stateEstimate.x/y 相对起始点的累计偏移——这是光流最终落到状态估计的
#      结果，可以和步骤 3/4 里的推动方向再做一次交叉印证（两者应该指向同一个结论）。
#      如果不是 kalman，说明光流没有真正参与状态估计，这部分数字没有参考意义，脚本会提示。

import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

from config import URI, connect_with_timeout

ESTIMATOR_NAMES = {0: "any", 1: "complementary", 2: "kalman"}

ACCUM_WINDOW_S = 1.0     # 光流累加窗口时长：越长越平滑但方向切换的分辨率越低
VALID_MOTION_BYTE = 0xB0  # 跟 flowdeck_v1v2.c 里判断"这一帧可以喂给 EKF"的条件完全一致


def read_current_estimator(cf, timeout_s=2.0):
    """读取 stabilizer.estimator 参数，返回当前值（1=complementary, 2=kalman），超时返回 None。"""
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
        "dx_raw": 0, "dy_raw": 0,      # 原始 deltaX/deltaY 累加（传感器寄存器坐标）
        "dpx_ekf": 0, "dpy_ekf": 0,    # 翻转后累加：dpixelx=-deltaY, dpixely=-deltaX（EKF 实际看到的）
        "valid_frames": 0, "invalid_frames": 0,
        "last_squal": None,
        "last_outlier_count": None,
    }
    height = {"zrange_mm": None, "z_est": None, "last_t": None}
    pos = {"x": None, "y": None, "x0": None, "y0": None, "last_t": None}

    def flow_cb(_timestamp, data, _logconf):
        motion_byte = data["motion.motion"]
        dx = data["motion.deltaX"]
        dy = data["motion.deltaY"]
        with lock:
            accum["last_squal"] = data["motion.squal"]
            accum["last_outlier_count"] = data["motion.outlierCount"]
            if motion_byte == VALID_MOTION_BYTE:
                accum["valid_frames"] += 1
                accum["dx_raw"] += dx
                accum["dy_raw"] += dy
                accum["dpx_ekf"] += -dy
                accum["dpy_ekf"] += -dx
            else:
                accum["invalid_frames"] += 1

    def height_cb(_timestamp, data, _logconf):
        height["zrange_mm"] = data["range.zrange"]
        height["z_est"] = data["stateEstimate.z"]
        height["last_t"] = time.monotonic()

    def pos_cb(_timestamp, data, _logconf):
        pos["x"] = data["stateEstimate.x"]
        pos["y"] = data["stateEstimate.y"]
        if pos["x0"] is None:
            pos["x0"] = pos["x"]
            pos["y0"] = pos["y"]
        pos["last_t"] = time.monotonic()

    flow_lg = LogConfig(name="flow", period_in_ms=50)
    for v, t in (("motion.motion", "uint8_t"), ("motion.deltaX", "int16_t"),
                 ("motion.deltaY", "int16_t"), ("motion.squal", "uint8_t"),
                 ("motion.outlierCount", "uint8_t")):
        flow_lg.add_variable(v, t)

    height_lg = LogConfig(name="height", period_in_ms=100)
    height_lg.add_variable("range.zrange", "uint16_t")
    height_lg.add_variable("stateEstimate.z", "float")

    pos_lg = LogConfig(name="pos", period_in_ms=100)
    pos_lg.add_variable("stateEstimate.x", "float")
    pos_lg.add_variable("stateEstimate.y", "float")

    for lg in (flow_lg, height_lg, pos_lg):
        cf.log.add_config(lg)

    flow_lg.data_received_cb.add_callback(flow_cb)
    height_lg.data_received_cb.add_callback(height_cb)
    pos_lg.data_received_cb.add_callback(pos_cb)

    try:
        estimator = read_current_estimator(cf)
        show_pos = (estimator == 2)
        if not show_pos:
            print(
                "提示：当前不是 kalman，stateEstimate.x/y 不会反映光流的效果，下面只打印高度和"
                "光流本身的窗口累加值，位置交叉印证部分会跳过。",
                flush=True,
            )

        flow_lg.start()
        height_lg.start()
        pos_lg.start()

        print("\n开始测量，按 Ctrl+C 结束。建议操作顺序见脚本头部注释。\n", flush=True)
        print(
            f"每 {ACCUM_WINDOW_S:.1f}s 打印一次光流窗口累加值："
            "raw=(原始deltaX,deltaY累加)  ekf=(喂给EKF的dpixelx,dpixely累加，"
            "即 (-deltaY累加, -deltaX累加))\n",
            flush=True,
        )

        while True:
            time.sleep(ACCUM_WINDOW_S)

            with lock:
                dx_raw, dy_raw = accum["dx_raw"], accum["dy_raw"]
                dpx_ekf, dpy_ekf = accum["dpx_ekf"], accum["dpy_ekf"]
                valid, invalid = accum["valid_frames"], accum["invalid_frames"]
                squal, outlier_count = accum["last_squal"], accum["last_outlier_count"]
                accum["dx_raw"] = accum["dy_raw"] = 0
                accum["dpx_ekf"] = accum["dpy_ekf"] = 0
                accum["valid_frames"] = accum["invalid_frames"] = 0

            zrange = height["zrange_mm"]
            z_est = height["z_est"]

            print(
                f"[flow]   raw=({dx_raw:+6d},{dy_raw:+6d})px  "
                f"ekf=({dpx_ekf:+6d},{dpy_ekf:+6d})px  "
                f"valid/invalid={valid}/{invalid}  squal={squal}  outlierCount(累计)={outlier_count}"
            )
            print(f"[height] range.zrange={zrange}mm  stateEstimate.z={z_est}")

            if show_pos and pos["x"] is not None:
                dx_pos = pos["x"] - pos["x0"]
                dy_pos = pos["y"] - pos["y0"]
                print(
                    f"[pos]    stateEstimate.(x,y)=({pos['x']:+.3f},{pos['y']:+.3f})m  "
                    f"相对起点偏移=({dx_pos:+.3f},{dy_pos:+.3f})m"
                )
            print("", flush=True)

    except KeyboardInterrupt:
        pass
    finally:
        flow_lg.stop()
        height_lg.stop()
        pos_lg.stop()
        print("测量结束。", flush=True)
        cf.close_link()


if __name__ == "__main__":
    main()
