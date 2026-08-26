#!/usr/bin/env python3
# Step D：只读传感器/姿态日志，电机完全不转，零风险。
#
# 目的：验证 processAccGyroMeasurements()（sensors_mpu6050_hm5883L_ms5611.c）里的换轴/取负表
# 对不对——之前 t4c 在 CONFIG_PITCH_DISTRIBUTION_INVERTED 开/关两种情况下都测出"roll 对、
# pitch 反、不随窗口/积分变化"，已经排除了混控公式和积分饱和，也验证过 complementary/kalman
# 两个估计器的 pitch 符号数学上等价（不是估计器不一致）。剩下最直接的候选就是芯片贴装方向跟
# 代码假设的换轴/取负表不匹配，而这只能靠"手动倾斜 + 对照原始寄存器读数"来坐实，不能再靠代码推导。
#
# 打印的五层数据，x/y/z 三个轴都打，从"最原始"到"最终"：
#   gyro.xRaw/yRaw/zRaw    —— 芯片寄存器换轴之后、取负和换算之前的原始整数（int16 counts）
#   accRaw.xRaw/yRaw/zRaw  —— 同上，加速度计这一路的原始整数（int16 counts）
#   gyro.x/y/z             —— gyro.*Raw 再经过换算（取负+乘 SENSORS_DEG_PER_LSB_CFG）之后的角速度(deg/s)
#   acc.x/y/z              —— accRaw.*Raw 换轴+取负之后的读数（g），姿态融合用它来修正倾角
#   stabilizer.roll/pitch/yaw —— 最终融合出的姿态角(deg)，PID 真正用的就是这三个
#
# accRaw.*Raw 这一路日志目前固件里没有导出，这次改动同步在
# sensors_mpu6050_hm5883L_ms5611.c 里加了 LOG_GROUP_START(accRaw)（跟已有的 gyro.*Raw 对称），
# 并把原来被注释掉的 GYRO_ADD_RAW_AND_VARIANCE_LOG_VALUES 打开——没打开的话 gyro.*Raw
# 也是不存在的，之前几版脚本其实连不上这几个变量，这次已经一起改了，跑之前记得重新编译刷机。
#
# 分成四路 log block（gyro 6 个变量、accRaw 3 个、acc 3 个、姿态 3 个），避免单个 block 超出 CRTP payload 限制。
#
# 依次做纯 pitch（抬头/低头）、纯 roll（左右倾斜）、纯 yaw（原地扭转机头）动作，观察：
#   1. 做纯 pitch 时，roll/yaw 相关的列（gyro.xRaw/zRaw、gyro.x/z、stabilizer.roll/yaw）应该
#      基本不动；如果也跟着明显变化，说明有串轴，问题在换轴表本身（哪个寄存器映射到哪个轴错了）。
#   2. 做纯 pitch 时，只看 gyro.yRaw 这一列（还没被代码取负/取正处理过）随手的抬头/低头动作是
#      什么符号；再看 gyro.y 和 stabilizer.pitch 是不是跟 gyro.yRaw "该有的换算关系"一致。
#      如果 gyro.yRaw 的原始符号跟你直觉预期的物理方向对不上，说明贴装方向跟换轴表假设的不一致，
#      该改的是 processAccGyroMeasurements() 里对应那一行的取负，不是动力分配。
#   3. roll、yaw 同理，分别看 gyro.xRaw/stabilizer.roll、gyro.zRaw/stabilizer.yaw。
#   4. acc 这一路是静态测试（重力分量），抬头/低头时 accRaw.xRaw 应该有对应的单调变化，
#      同样对照 acc.x 和 stabilizer.pitch 的换算关系是否自洽；roll/yaw 同理看 accRaw.yRaw/zRaw。

import threading
import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from config import URI

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
        print(f"当前 stabilizer.estimator = {value} ({label})", flush=True)
        result["value"] = val
        got_value.set()

    cf.param.add_update_callback(group="stabilizer", name="estimator", cb=estimator_cb)
    cf.param.request_param_update("stabilizer.estimator")

    if not got_value.wait(timeout=timeout_s):
        print("警告：读取 stabilizer.estimator 超时，未确认当前估计器。", flush=True)
        return None
    return result["value"]


def ensure_complementary_estimator(cf):
    """若当前是 kalman(2)，强制切成 complementary(1)，避免光流模块把估计器锁死成 kalman
    （静态台架测试没有真实加速度动态，kalman 的倾角修正在这种场景下不可信，见 t4b 同名函数）。"""
    current = read_current_estimator(cf)
    if current == 2:
        print("检测到当前是 kalman，强制切换为 complementary...", flush=True)
        cf.param.set_value("stabilizer.estimator", "1")
        time.sleep(0.5)
        read_current_estimator(cf)


def main():
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI) as scf:
        cf = scf.cf
        print("已连接。", flush=True)

        ensure_complementary_estimator(cf)

        state = {
            "gyro.xRaw": None,
            "gyro.yRaw": None,
            "gyro.zRaw": None,
            "gyro.x": None,
            "gyro.y": None,
            "gyro.z": None,
            "accRaw.xRaw": None,
            "accRaw.yRaw": None,
            "accRaw.zRaw": None,
            "acc.x": None,
            "acc.y": None,
            "acc.z": None,
        }

        # 四路拆开，避免单个 CRTP log block 超出 payload 限制
        lg_gyro = LogConfig(name="gyro_axis", period_in_ms=50)
        lg_gyro.add_variable("gyro.xRaw", "int16_t")
        lg_gyro.add_variable("gyro.yRaw", "int16_t")
        lg_gyro.add_variable("gyro.zRaw", "int16_t")
        lg_gyro.add_variable("gyro.x", "float")
        lg_gyro.add_variable("gyro.y", "float")
        lg_gyro.add_variable("gyro.z", "float")
        cf.log.add_config(lg_gyro)

        lg_acc_raw = LogConfig(name="acc_raw_axis", period_in_ms=50)
        lg_acc_raw.add_variable("accRaw.xRaw", "int16_t")
        lg_acc_raw.add_variable("accRaw.yRaw", "int16_t")
        lg_acc_raw.add_variable("accRaw.zRaw", "int16_t")
        cf.log.add_config(lg_acc_raw)

        lg_acc = LogConfig(name="acc_axis", period_in_ms=100)
        lg_acc.add_variable("acc.x", "float")
        lg_acc.add_variable("acc.y", "float")
        lg_acc.add_variable("acc.z", "float")
        cf.log.add_config(lg_acc)

        lg_att = LogConfig(name="att_axis", period_in_ms=100)
        lg_att.add_variable("stabilizer.roll", "float")
        lg_att.add_variable("stabilizer.pitch", "float")
        lg_att.add_variable("stabilizer.yaw", "float")
        cf.log.add_config(lg_att)

        def log_gyro_cb(timestamp, data, logconf):
            state["gyro.xRaw"] = data["gyro.xRaw"]
            state["gyro.yRaw"] = data["gyro.yRaw"]
            state["gyro.zRaw"] = data["gyro.zRaw"]
            state["gyro.x"] = data["gyro.x"]
            state["gyro.y"] = data["gyro.y"]
            state["gyro.z"] = data["gyro.z"]

        def log_acc_raw_cb(timestamp, data, logconf):
            state["accRaw.xRaw"] = data["accRaw.xRaw"]
            state["accRaw.yRaw"] = data["accRaw.yRaw"]
            state["accRaw.zRaw"] = data["accRaw.zRaw"]

        def log_acc_cb(timestamp, data, logconf):
            state["acc.x"] = data["acc.x"]
            state["acc.y"] = data["acc.y"]
            state["acc.z"] = data["acc.z"]

        def log_att_cb(timestamp, data, logconf):
            roll = data["stabilizer.roll"]
            pitch = data["stabilizer.pitch"]
            yaw = data["stabilizer.yaw"]

            def fmt_i(v, width=6):
                return f"{v:{width}d}" if v is not None else "n/a".rjust(width)

            def fmt_f(v, width=7, prec=1):
                return f"{v:{width}.{prec}f}" if v is not None else "n/a".rjust(width)

            print(
                f"t={timestamp:>8}  "
                f"gyroRaw(x,y,z)=({fmt_i(state['gyro.xRaw'])},{fmt_i(state['gyro.yRaw'])},"
                f"{fmt_i(state['gyro.zRaw'])})  |  "
                f"gyro(x,y,z)=({fmt_f(state['gyro.x'])},{fmt_f(state['gyro.y'])},"
                f"{fmt_f(state['gyro.z'])})  |  "
                f"accRaw(x,y,z)=({fmt_i(state['accRaw.xRaw'])},{fmt_i(state['accRaw.yRaw'])},"
                f"{fmt_i(state['accRaw.zRaw'])})  |  "
                f"acc(x,y,z)=({fmt_f(state['acc.x'], 6, 3)},{fmt_f(state['acc.y'], 6, 3)},"
                f"{fmt_f(state['acc.z'], 6, 3)})  |  "
                f"roll={roll:6.2f} pitch={pitch:6.2f} yaw={yaw:6.2f}",
                flush=True,
            )

        lg_gyro.data_received_cb.add_callback(log_gyro_cb)
        lg_acc_raw.data_received_cb.add_callback(log_acc_raw_cb)
        lg_acc.data_received_cb.add_callback(log_acc_cb)
        lg_att.data_received_cb.add_callback(log_att_cb)
        lg_gyro.start()
        lg_acc_raw.start()
        lg_acc.start()
        lg_att.start()

        print("\n开始打印，电机完全不转、零风险。按 Ctrl+C 结束。建议依次做：")
        print("  1. 水平静置几秒，记录基线（gyro*Raw/accRaw*Raw 应接近 0，roll/pitch/yaw 应接近 0）")
        print("  2. 只做抬头/低头（绕 pitch 轴），观察 gyro.yRaw、accRaw.xRaw 抬头时是正是负，")
        print("     同时确认 gyro.xRaw/zRaw、accRaw.yRaw/zRaw、roll/yaw 基本不动（没有串轴）")
        print("  3. 回正后只做左右倾斜（绕 roll 轴），观察 gyro.xRaw、accRaw.yRaw 右倾时是正是负，")
        print("     同时确认 gyro.yRaw/zRaw、accRaw.xRaw/zRaw、pitch/yaw 基本不动")
        print("  4. 回正后只做原地扭转机头（绕 yaw 轴），观察 gyro.zRaw 符号，")
        print("     同时确认 gyro.xRaw/yRaw、roll/pitch 基本不动（yaw 是角速度，静态重力分量测不出来）")
        print("  5. 记录：抬头对应 gyro.yRaw/accRaw.xRaw 的符号、右倾对应 gyro.xRaw/accRaw.yRaw 的符号、")
        print("     机头右转对应 gyro.zRaw 的符号\n")

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            lg_gyro.stop()
            lg_acc_raw.stop()
            lg_acc.stop()
            lg_att.stop()
            print("\n测试结束。", flush=True)


if __name__ == "__main__":
    main()
