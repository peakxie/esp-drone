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
import time

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
RANGE_SANE_MAX_MM = 4000  # 起飞前地面测距合理性上限，理由同 t5/t6/t7：不设下限，VL53L1X 贴近量程
                          # 下限时读数本身偏随机，起飞前贴地见到几十 mm 以内的低读数是正常现象

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


def read_current_estimator(cf, timeout_s=2.0):
    """读取 stabilizer.estimator 参数，返回当前值（1=complementary, 2=kalman），
    超时返回 None。跟 t6/t7 完全一致的实现。"""
    import threading

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


def set_and_verify_param(cf, group, name, value, timeout_s=2.0):
    """设置 group.name = value，然后通过回调等待固件确认新值生效。成功返回 True；
    回读超时或者回读到的值跟期望值不一致都返回 False。跟 t7 完全一致的实现。"""
    import threading

    full_name = f"{group}.{name}"
    got_value = threading.Event()
    holder = {"value": None}

    def value_cb(_name, new_value):
        holder["value"] = new_value
        got_value.set()

    cf.param.add_update_callback(group=group, name=name, cb=value_cb)
    cf.param.set_value(full_name, str(value))
    got_value.wait(timeout=timeout_s)

    if not got_value.is_set():
        print(f"警告：设置 {full_name}={value} 后回读超时（{timeout_s:.1f}s 内没有收到确认）。", flush=True)
        return False

    if str(holder["value"]) != str(value):
        print(f"警告：设置 {full_name}={value} 后回读到的值是 {holder['value']!r}，与期望值不一致。", flush=True)
        return False

    print(f"已确认 {full_name} = {holder['value']}。", flush=True)
    return True


def _send_sequencer_packet(cf, opcode, payload=b""):
    """cflib 不认识本次新增的 opcode（SEQ_CLEAR/ADD_STEP/START/CANCEL），手工构造
    CRTPPacket 发到跟高层规划器同一个端口。STOP 复用 cflib 已有的
    cf.high_level_commander.stop()，不走这个函数。"""
    from cflib.crtp.crtpstack import CRTPPacket

    pk = CRTPPacket()
    pk.port = CRTP_PORT_SETPOINT_HL
    pk.data = bytes([opcode]) + payload
    cf.send_packet(pk)


def upload_and_start_sequence(cf, steps):
    """SEQ_CLEAR -> SEQ_ADD_STEP * N -> SEQ_START。固件的 sequencerAddStep()/
    sequencerStart() 会做自己的硬校验（见设计文档第6节），这里只负责按协议把包
    发出去，不假设一定会被接受——调用方需要之后轮询 seq.state 确认序列真的开始跑
    了（main() 里就是这么做的）。"""
    _send_sequencer_packet(cf, SEQ_CLEAR)
    time.sleep(0.05)
    for name, *params in steps:
        _send_sequencer_packet(cf, SEQ_ADD_STEP, pack_step(name, params))
        time.sleep(0.05)
    _send_sequencer_packet(cf, SEQ_START)


def main():
    # cflib 只在这里 import：让本模块的纯逻辑函数（pack_step、validate_sequence）
    # 在没有装 cflib 的机器上也能被 import 和单测，只有真正执行 main() 飞行时才
    # 需要 cflib。跟 t6/t7 完全一致的约定。
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig

    from config import URI, connect_with_timeout

    LOG_WAIT_TIMEOUT_S = 2.0
    LOG_STALE_TIMEOUT_S = 0.3
    MAX_FLIGHT_TIME_S = 20.0
    STATUS_PRINT_PERIOD_S = 0.3

    errors = validate_sequence(DEFAULT_SEQUENCE)
    if errors:
        print("序列校验失败，拒绝执行：", flush=True)
        for err in errors:
            print(f"  - {err}", flush=True)
        return

    cflib.crtp.init_drivers()

    cf = Crazyflie()
    if not connect_with_timeout(cf, URI):
        return

    state = {
        "zrange_mm": None,
        "last_log_t": None,
        "z_est": None,
        "seq_state": None,
        "seq_step_idx": None,
        "seq_step_type": None,
        "seq_elapsed_ms": None,
    }

    def aux_cb(_timestamp, data, _logconf):
        state["zrange_mm"] = data["range.zrange"]
        state["z_est"] = data["stateEstimate.z"]
        state["seq_state"] = data["seq.state"]
        state["seq_step_idx"] = data["seq.stepIdx"]
        state["seq_step_type"] = data["seq.stepType"]
        state["seq_elapsed_ms"] = data["seq.elapsedMs"]
        state["last_log_t"] = time.monotonic()

    aux_lg = LogConfig(name="aux", period_in_ms=50)
    aux_lg.add_variable("range.zrange", "uint16_t")
    aux_lg.add_variable("stateEstimate.z", "float")
    aux_lg.add_variable("seq.state", "uint8_t")
    aux_lg.add_variable("seq.stepIdx", "uint8_t")
    aux_lg.add_variable("seq.stepType", "uint8_t")
    aux_lg.add_variable("seq.elapsedMs", "uint32_t")
    cf.log.add_config(aux_lg)
    aux_lg.data_received_cb.add_callback(aux_cb)
    aux_lg.start()

    try:
        estimator = read_current_estimator(cf)
        if estimator != 2:
            print(
                "警告：当前不是 kalman 估计器——本机应装有 PMW3901 光流模块，期望是 "
                "kalman（=2）。非 kalman 时 x/y 只是加速度计双积分，不代表真实位置。",
                flush=True,
            )

        if not set_and_verify_param(cf, "commander", "enHighLevel", 1):
            print("错误：commander.enHighLevel 设置/回读失败，拒绝执行序列。", flush=True)
            return

        deadline = time.monotonic() + LOG_WAIT_TIMEOUT_S
        while state["last_log_t"] is None and time.monotonic() < deadline:
            time.sleep(0.02)
        if state["last_log_t"] is None:
            print("错误：等不到遥测，放弃执行。", flush=True)
            return

        zrange0 = state["zrange_mm"]
        if zrange0 is None or zrange0 > RANGE_SANE_MAX_MM:
            print(
                f"错误：起飞前 range.zrange={zrange0}mm 超出合理范围（上限 {RANGE_SANE_MAX_MM}mm），"
                "怀疑测距传感器读数异常，放弃执行。请确认飞机放在平整地面、传感器朝下且未被遮挡。",
                flush=True,
            )
            return
        print(f"起飞前地面测距 = {zrange0}mm，遥测正常。", flush=True)

        print(f"开始上传并启动序列（{len(DEFAULT_SEQUENCE)} 步）...", flush=True)
        upload_and_start_sequence(cf, DEFAULT_SEQUENCE)

        # SEQ_START 有可能被固件拒绝（参数越界、最后一步不是 LAND_SENSOR、
        # 或者 state->position.z 还没被喂过一次），拒绝时 seq.state 会一直停在
        # 0（IDLE）。不确认这一点就直接进入下面的监控循环，会在 20s 硬上限
        # 超时之前一直打印看起来"正常"的状态行，把"固件拒绝了"跟"序列正在
        # 执行"混为一谈。这里用一个远短于硬上限的独立超时提前发现拒绝。
        START_CONFIRM_TIMEOUT_S = 0.5
        start_deadline = time.monotonic() + START_CONFIRM_TIMEOUT_S
        while state["seq_state"] in (None, 0) and time.monotonic() < start_deadline:
            time.sleep(0.02)
        if state["seq_state"] in (None, 0):
            print(
                f"错误：SEQ_START 之后 {START_CONFIRM_TIMEOUT_S:.1f}s 内 seq.state 仍是 "
                f"{state['seq_state']!r}（IDLE），序列大概率被固件拒绝，不是在正常执行。"
                "放弃等待，不再进入下面的监控循环干等硬上限超时。",
                flush=True,
            )
            return
        print(f"已确认序列开始执行（seq.state={state['seq_state']}）。", flush=True)

        flight_deadline = time.monotonic() + MAX_FLIGHT_TIME_S
        last_status_print = [0.0]

        def emergency_stop(reason):
            print(reason, flush=True)
            cf.high_level_commander.stop()

        def watchdog_ok():
            now = time.monotonic()
            if now > flight_deadline:
                emergency_stop("警告：总时长超过硬上限，触发急停。")
                return False
            if state["last_log_t"] is None or (now - state["last_log_t"]) > LOG_STALE_TIMEOUT_S:
                emergency_stop("警告：遥测已停止刷新，触发急停。")
                return False
            return True

        def print_status():
            now = time.monotonic()
            if now - last_status_print[0] < STATUS_PRINT_PERIOD_S:
                return
            last_status_print[0] = now
            print(
                f"    seq.state={state['seq_state']} stepIdx={state['seq_step_idx']} "
                f"stepType={state['seq_step_type']} elapsedMs={state['seq_elapsed_ms']} "
                f"zrange={state['zrange_mm']}mm z={state['z_est']}",
                flush=True,
            )

        print("序列执行中，Ctrl+C 触发 CANCEL（转入 LAND_SENSOR）...", flush=True)
        started_running = False
        while True:
            print_status()
            if not watchdog_ok():
                break
            if state["seq_state"] in (1, 2):
                started_running = True
            elif started_running and state["seq_state"] == 0:
                print("序列已回到 IDLE，视为执行完成。", flush=True)
                break
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("收到 Ctrl+C，发送 SEQ_CANCEL（转入 LAND_SENSOR）...", flush=True)
        try:
            _send_sequencer_packet(cf, SEQ_CANCEL)
        except Exception as exc:
            print(f"SEQ_CANCEL 发送失败（{exc!r}），退化为 STOP 急停。", flush=True)
            cf.high_level_commander.stop()
    finally:
        aux_lg.stop()
        cf.close_link()


if __name__ == "__main__":
    main()
