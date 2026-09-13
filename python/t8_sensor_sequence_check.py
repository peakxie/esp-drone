#!/usr/bin/env python3
# T8：验证固件新增的传感器确认式飞行序列（sequencer.c）——TAKEOFF_SENSOR/DELAY/
# LAND_SENSOR 由固件在专职任务里自主执行，完成判定基于 VL53L1 原始测距
# （range.zrange），不是时钟/duration。设计文档：
# docs/superpowers/specs/2026-09-13-sensor-gated-sequencer-design.md
#
# 协议（跟固件 sequencer.h / crtp_commander_high_level.c 的新 opcode 一致）：
#   复用 CRTP_PORT_SETPOINT_HL（0x08）端口。SEQ_CLEAR=11（无载荷）/
#   SEQ_ADD_STEP=12（1字节类型 + 7个float，共29字节）/ SEQ_START=13（无载荷）/
#   SEQ_CANCEL=14（无载荷）。STOP 复用现有 opcode=3——cflib 的
#   cf.high_level_commander.stop() 已经会发这个包，直接复用，不需要手工构造。

import struct

CRTP_PORT_SETPOINT_HL = 0x08

SEQ_CLEAR = 11
SEQ_ADD_STEP = 12
SEQ_START = 13
SEQ_CANCEL = 14

STEP_TYPE_CODES = {
    "TAKEOFF_SENSOR": 0,
    "DELAY": 1,
    "LAND_SENSOR": 2,
}

MAX_STEPS = 16
MAX_TAKEOFF_TARGET_M = 2.0
MAX_LAND_TARGET_M = 2.0

ESTIMATOR_NAMES = {0: "any", 1: "complementary", 2: "kalman"}

# 最小三步示例（本轮只实现这三种命令）：
#   TAKEOFF_SENSOR(target=0.3m, stable_hold=0.5s, timeout=3.0s)
#   DELAY(hold_time=1.0s)
#   LAND_SENSOR(target=0, duration=2.0s)
DEFAULT_SEQUENCE = [
    ("TAKEOFF_SENSOR", 0.3, 0.5, 3.0),
    ("DELAY", 1.0),
    ("LAND_SENSOR", 0.0, 2.0),
]


def pack_step(step_name, params):
    """按固件 sequencer.h 的 sequencerStep_t 打包成 29 字节：1字节类型 + 7个float
    （不足 7 个的补 0）。float 用小端序（ESP32/STM32 都是小端，固件里 struct 直接
    从 CRTP 载荷原样解释，不需要额外转换）。"""
    if step_name not in STEP_TYPE_CODES:
        raise ValueError(f"未知的步骤类型：{step_name!r}（合法值：{sorted(STEP_TYPE_CODES)}）")
    if len(params) > 7:
        raise ValueError(f"最多 7 个参数，{step_name} 收到了 {len(params)} 个：{params!r}")
    padded = list(params) + [0.0] * (7 - len(params))
    return bytes([STEP_TYPE_CODES[step_name]]) + struct.pack("<7f", *padded)


def validate_sequence(steps, max_steps=MAX_STEPS):
    """连接飞机前的纯本地校验，规则跟固件 sequencer.c 的 sequencerAddStep()/
    sequencerStart() 保持一致（见设计文档第6节）——本地校验不能替代固件校验，只是
    提前把明显非法的序列挡在连接飞机之前，跟 t6_flight_sequence.py 的
    validate_flight_plan 是同一个思路。"""
    errors = []
    if not steps:
        errors.append("序列为空，至少需要一个 LAND_SENSOR 结尾。")
        return errors
    if len(steps) > max_steps:
        errors.append(f"序列有 {len(steps)} 步，超过固件缓冲区上限 {max_steps} 步。")

    last_name = steps[-1][0]
    if last_name != "LAND_SENSOR":
        errors.append(f"最后一步必须是 LAND_SENSOR，实际是 {last_name!r}。")

    for idx, entry in enumerate(steps):
        name, *params = entry
        if name == "TAKEOFF_SENSOR":
            if len(params) != 3:
                errors.append(
                    f"第 {idx} 步 TAKEOFF_SENSOR 需要 3 个参数 (target_m, stable_hold_s, timeout_s)，"
                    f"收到 {len(params)} 个：{entry!r}"
                )
                continue
            target_m, stable_hold_s, timeout_s = params
            if not (0.0 < target_m <= MAX_TAKEOFF_TARGET_M):
                errors.append(f"第 {idx} 步 TAKEOFF_SENSOR target={target_m} 超出 (0, {MAX_TAKEOFF_TARGET_M}]m：{entry!r}")
            if stable_hold_s < 0.0:
                errors.append(f"第 {idx} 步 TAKEOFF_SENSOR stable_hold={stable_hold_s} 必须 >= 0：{entry!r}")
            if timeout_s <= stable_hold_s:
                errors.append(
                    f"第 {idx} 步 TAKEOFF_SENSOR timeout={timeout_s} 必须大于 stable_hold={stable_hold_s}：{entry!r}"
                )
        elif name == "DELAY":
            if len(params) != 1:
                errors.append(f"第 {idx} 步 DELAY 需要 1 个参数 (hold_time_s)，收到 {len(params)} 个：{entry!r}")
                continue
            (hold_time_s,) = params
            if hold_time_s <= 0.0:
                errors.append(f"第 {idx} 步 DELAY hold_time={hold_time_s} 必须 > 0：{entry!r}")
        elif name == "LAND_SENSOR":
            if len(params) != 2:
                errors.append(f"第 {idx} 步 LAND_SENSOR 需要 2 个参数 (target_m, duration_s)，收到 {len(params)} 个：{entry!r}")
                continue
            target_m, duration_s = params
            if not (0.0 <= target_m <= MAX_LAND_TARGET_M):
                errors.append(f"第 {idx} 步 LAND_SENSOR target={target_m} 超出 [0, {MAX_LAND_TARGET_M}]m：{entry!r}")
            if duration_s <= 0.0:
                errors.append(f"第 {idx} 步 LAND_SENSOR duration={duration_s} 必须 > 0：{entry!r}")
        else:
            errors.append(f"第 {idx} 步命令名未知：{name!r}（合法命令：{sorted(STEP_TYPE_CODES)}）")

    return errors


if __name__ == "__main__":
    print("这个文件到 Task 4 才有 main()，现在只能被 import 用于单元测试。")
