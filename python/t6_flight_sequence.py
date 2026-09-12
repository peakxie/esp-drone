#!/usr/bin/env python3
# T6：多命令飞行序列执行器——参考 t5_hover_land.py 已验证过的安全模型（20Hz 发送、日志新鲜度
# 看门狗、硬性总时长上限、Ctrl+C/看门狗触发走斜坡降落、range.zrange 触地确认），改成可以在
# 一次连接内按顺序执行任意多条命令（takeoff/hover/goto/land），而不是每种测试单独写一个脚本。
#
# 命令列表在下面的 FLIGHT_PLAN 里声明，每条是 (命令名, *参数) 元组：
#   ("takeoff", height_m[, duration_s])   起飞爬升到绝对高度 height_m，x/y 钉在起飞点
#   ("hover", duration_s)                 保持当前目标不变，悬停 duration_s
#   ("goto", dx, dy, h, duration_s)       目标线性斜坡过渡到 (起飞点+dx, 起飞点+dy, 绝对高度h)
#                                          —— dx=dy=0 时就是单纯改变高度（"定高"）
#   ("land"[, duration_s])                斜坡降到地面 + range.zrange 触地确认 + 停桨
#
# yaw 全程固定为起飞时的 yaw0，不作为参数暴露——这个项目还没有验证过 yaw 控制。
# 不使用固件的 HighLevelCommander（crtp_commander_high_level.c）：固件确实编译了这部分代码，
# 但从未做过飞行验证，继续沿用 t5 已验证的 send_position_setpoint 路线。
#
# 安全约定（同 t5_hover_land.py）：
#   - 全程以 SEND_PERIOD 周期性发送 setpoint，避免 commander watchdog 超时进入 fallback。
#   - 用高度日志做"链路存活"看门狗：超过 LOG_STALE_TIMEOUT_S 没收到新帧，立即认为链路/主控
#     异常，跳过剩余命令直接执行紧急下降。
#   - Ctrl+C 不会瞬间切电机——从当前目标做一次快速但连续的下降斜坡再停桨。
#   - 全程有一个 MAX_FLIGHT_TIME_S 硬上限（覆盖整个命令列表，不是每条命令单独计时），超时
#     无条件进入紧急下降。
#   - 命令列表如果没有以 land 结尾，执行完所有命令后自动补一次完整 land，不允许悬在半空
#     直接结束脚本。
#   - 连接前对命令列表做纯本地静态校验（命令名/参数个数/取值范围），不合法直接拒绝执行，
#     不会尝试连接/起飞。
#
# 首次测试建议：室内、地面平整、四周留够 1m 净空、旁边有人随时准备断电。

import time

from config import URI, connect_with_timeout

ESTIMATOR_NAMES = {0: "any", 1: "complementary", 2: "kalman"}

LIFTOFF_HEIGHT_M = 0.03    # 起飞斜坡的起点目标高度，避免第一帧就是 0（等同于"还没起飞"）
LAND_HEIGHT_M = 0.0        # 降落斜坡的终点目标高度

TAKEOFF_TIME_S = 2.0       # takeoff 命令未指定 duration_s 时的默认值
LAND_TIME_S = 2.0          # land 命令未指定 duration_s 时的默认值

TOUCHDOWN_ZRANGE_MM = 90   # 判定"已经进入地面效应气垫、可以停桨"的测距阈值（同 t5 的实测结论）
TOUCHDOWN_CONFIRM_S = 0.3  # 连续这么久测距都在阈值以下，才认为已经稳定卡进气垫
TOUCHDOWN_MAX_WAIT_S = 5.0 # 兜底上限：一直没等到"确认落地"也最多等这么久就强制停桨
CLOSE_TO_GROUND_LOG_ZRANGE_MM = 150  # 进入这个高度以内，print_status 不再受节流限制，每帧打印

SEND_PERIOD = 0.05         # 20Hz 发送 setpoint
STATUS_PRINT_PERIOD_S = 0.3
EMERGENCY_LAND_TIME_S = 1.0

LOG_WAIT_TIMEOUT_S = 2.0
LOG_STALE_TIMEOUT_S = 0.3
MAX_FLIGHT_TIME_S = 30.0   # 覆盖整个 FLIGHT_PLAN 执行期的硬上限，按实际命令列表总时长调整

RANGE_SANE_MAX_MM = 4000   # 不设下限，理由同 t5：VL53L1X 贴近量程下限读数本身偏随机，不是故障

VX_KI_OVERRIDE = 2.0
VY_KI_OVERRIDE = 2.0

MAX_XY_OFFSET_M = 1.0      # goto 的 dx/dy 绝对值上限，防止参数手误导致意外大位移
MAX_HEIGHT_M = 1.0         # takeoff/goto 的高度上限，防止参数手误导致意外高高度

# 命令名 -> (最少参数个数, 最多参数个数)
COMMAND_ARG_SPECS = {
    "takeoff": (1, 2),  # (height_m,) 或 (height_m, duration_s)
    "hover": (1, 1),    # (duration_s,)
    "goto": (4, 4),     # (dx, dy, h, duration_s)
    "land": (0, 1),     # () 或 (duration_s,)
}

# 默认示例飞行计划：起飞到 0.5m -> 悬停 3s -> 前移 0.3m 同时保持 0.5m -> 悬停 2s -> 降到 0.3m -> 降落
FLIGHT_PLAN = [
    ("takeoff", 0.5),
    ("hover", 3.0),
    ("goto", 0.3, 0.0, 0.5, 2.0),
    ("hover", 2.0),
    ("goto", 0.0, 0.0, 0.3, 1.5),
    ("land",),
]


class FlightAbort(Exception):
    """内部信号：立即停止当前阶段，转入紧急下降。"""


def validate_flight_plan(plan, max_xy_offset_m=MAX_XY_OFFSET_M, max_height_m=MAX_HEIGHT_M):
    """对命令列表做连接前的纯本地校验（不依赖飞机/cflib），返回错误信息列表；
    空列表表示合法。第一条命令必须是 takeoff，否则后续命令的前提条件不成立。"""
    errors = []
    if not plan:
        errors.append("命令列表为空，至少需要一条 takeoff 命令。")
        return errors

    first_entry = plan[0]
    first_name = first_entry[0] if isinstance(first_entry, tuple) and len(first_entry) > 0 else None
    if first_name != "takeoff":
        errors.append(f"第一条命令必须是 takeoff，实际是 {first_entry!r}。")

    for idx, entry in enumerate(plan):
        if not isinstance(entry, tuple) or len(entry) == 0:
            errors.append(f"第 {idx} 条命令格式不对（应为非空元组）：{entry!r}")
            continue
        name = entry[0]
        args = entry[1:]
        if name not in COMMAND_ARG_SPECS:
            errors.append(f"第 {idx} 条命令名未知：{name!r}（合法命令：{sorted(COMMAND_ARG_SPECS)}）")
            continue
        min_args, max_args = COMMAND_ARG_SPECS[name]
        if not (min_args <= len(args) <= max_args):
            errors.append(
                f"第 {idx} 条命令 {name!r} 参数个数为 {len(args)}，应在 [{min_args}, {max_args}] 之间：{entry!r}"
            )
            continue

        if name == "takeoff":
            height_m = args[0]
            if not (0.0 < height_m <= max_height_m):
                errors.append(f"第 {idx} 条 takeoff 高度 {height_m} 超出合理范围 (0, {max_height_m}]m：{entry!r}")
            if len(args) == 2 and args[1] <= 0:
                errors.append(f"第 {idx} 条 takeoff duration_s={args[1]} 必须 > 0：{entry!r}")
        elif name == "goto":
            dx, dy, h, duration_s = args
            if abs(dx) > max_xy_offset_m or abs(dy) > max_xy_offset_m:
                errors.append(f"第 {idx} 条 goto 偏移 dx={dx} dy={dy} 超出 ±{max_xy_offset_m}m 范围：{entry!r}")
            if not (0.0 <= h <= max_height_m):
                errors.append(f"第 {idx} 条 goto 高度 h={h} 超出 [0, {max_height_m}]m 范围：{entry!r}")
            if duration_s <= 0:
                errors.append(f"第 {idx} 条 goto duration_s={duration_s} 必须 > 0：{entry!r}")
        elif name == "hover":
            (duration_s,) = args
            if duration_s <= 0:
                errors.append(f"第 {idx} 条 hover duration_s={duration_s} 必须 > 0：{entry!r}")
        elif name == "land":
            if args and args[0] <= 0:
                errors.append(f"第 {idx} 条 land duration_s={args[0]} 必须 > 0：{entry!r}")

    return errors


if __name__ == "__main__":
    pass
