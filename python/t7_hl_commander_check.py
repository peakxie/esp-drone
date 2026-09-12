#!/usr/bin/env python3
# T7：验证固件的 HighLevelCommander（CRTP 端口 0x08，crtp_commander_high_level.c）能否正常起降。
# t5_hover_land.py / t6_flight_sequence.py 走的是另一条固件路径——Python 侧用 send_position_setpoint
# 周期发送 setpoint，固件 position_controller_pid.c 的位置外环闭环控制，这条路径已经过多次实机验证。
# HighLevelCommander 是完全不同的固件代码分支：固件确实编译了 crtp_commander_high_level.c
# （takeoff2/land2/stop/go_to 均已实现，调研见 docs/superpowers/specs/2026-09-12-t7-hl-commander-
# check-design.md），但从未做过飞行验证。本脚本就是第一次验证：用官方 cflib.positioning.
# position_hl_commander.PositionHlCommander 做一次最小的起飞→悬停→降落，不测水平移动（go_to），
# 把风险面压到最低。
#
# 起飞前必须确认的固件前提（否则起飞命令会被静默忽略，电机不会响应）：
#   commander.c 里 enableHighLevel 默认 false，commanderGetSetpoint() 只有这个 param 为真时才会把
#   setpoint 交给高层规划器。本脚本起飞前会显式设置 commander.enHighLevel=1 并回读确认，失败则
#   拒绝起飞（不会尝试 take_off）。
#
# 本脚本必须是纯高层命令路径：commander.c 里任何一次低层 send_position_setpoint 都会把高层规划器
# 强制打回 idle（crtpCommanderHighLevelStop()），所以本脚本全程不调用 cf.commander.send_*，只用
# cf.high_level_commander / PositionHlCommander。
#
# 安全模型（跟 t5/t6 的"每帧插断斜坡"本质不同——PositionHlCommander.take_off()/land() 是阻塞调用，
# 轨迹插值完全在固件规划器里完成，Python 侧没有逐帧介入点）：
#   - 用 with PositionHlCommander(...) 语法：正常悬停结束、悬停中抛异常、Ctrl+C，__exit__ 都会
#     调一次 land()（land() 内部会再调 stop() 停桨）。
#   - __enter__（也就是 take_off()）执行期间发生的 Ctrl+C/异常，__exit__ 不会被调用（Python 的
#     with 语义：__enter__ 抛异常时不会进入 __exit__），所以额外用 try/except 包住整个 with 块，
#     异常时直接调 cf.high_level_commander.land()+stop() 兜底。
#   - 后台看门狗线程：日志新鲜度（LOG_STALE_TIMEOUT_S）+ 总时长硬上限（MAX_FLIGHT_TIME_S），
#     触发时直接调用 cf.high_level_commander.stop()（立即停桨，不是斜坡——PositionHlCommander
#     不提供"从任意状态平滑降落"的原语，且 HighLevelCommander.go_to()/land() 的文档明确警告过
#     不要叠加/打断正在执行的轨迹）。起飞高度只有 0.3m，直接停桨掉落的风险可接受。
#
# 已知行为差异（跟 t5/t6 不同，不是 bug）：cflib 的 HighLevelCommander.takeoff()/land() 默认把
# yaw 目标钉在绝对 0 弧度（yaw=0.0, useCurrentYaw=False），而不是像 t5/t6 那样保持起飞时的
# yaw0。PositionHlCommander.take_off()/land() 没有暴露 yaw 参数，无法覆盖这一行为。如果飞机
# 通电时朝向不在 0 弧度附近，起飞过程中可能会看到轻微自转——这是预期行为，不代表异常，靠
# print_status 里的 yaw 读数确认即可。
#
# 首次测试建议：室内、地面平整、四周留够 1m 净空、旁边有人随时准备断电/接住飞机。

import threading
import time

from config import URI, connect_with_timeout

ESTIMATOR_NAMES = {0: "any", 1: "complementary", 2: "kalman"}

TAKEOFF_HEIGHT_M = 0.3         # 起飞绝对高度，沿用 t5/t6 首次测试的保守高度
DEFAULT_VELOCITY_MPS = 0.5     # PositionHlCommander 的默认起降速度，不做覆盖
HOVER_TIME_S = 3.0             # 起飞完成后悬停时长

LOG_WAIT_TIMEOUT_S = 2.0       # 起飞前等待第一帧遥测的超时
LOG_STALE_TIMEOUT_S = 0.3      # 看门狗：日志新鲜度阈值
MAX_FLIGHT_TIME_S = 15.0       # 看门狗：覆盖起飞+悬停+降落全程的硬上限
STATUS_PRINT_PERIOD_S = 0.3    # 状态打印节流周期
PARAM_SET_TIMEOUT_S = 2.0      # commander.enHighLevel 设置后回读确认的超时
EMERGENCY_LAND_TIME_S = 1.0    # __enter__（take_off）期间异常时，兜底 land 的时长
LAND_HEIGHT_M = 0.0

RANGE_SANE_MAX_MM = 4000       # 起飞前地面测距合理性上限，不设下限（同 t5/t6）


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


def set_and_verify_param(cf, group, name, value, timeout_s=PARAM_SET_TIMEOUT_S):
    """设置 group.name = value，然后通过回调等待固件确认新值生效。成功返回 True；
    回读超时或者回读到的值跟期望值不一致都返回 False（调用方应据此拒绝起飞，而不是
    假设设置一定成功——commander.enHighLevel 这种"设置失败=静默忽略"的 param 尤其需要
    这一步，否则 take_off() 会看起来正常执行但固件其实完全没反应）。"""
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


if __name__ == "__main__":
    pass  # main() 由 Task 2 补上
