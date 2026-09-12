# t6_flight_sequence.py Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `python/t6_flight_sequence.py`, a multi-command flight-test script that executes a Python-list "flight plan" (`takeoff`/`hover`/`goto`/`land`) in one connection, reusing the safety model already validated in `python/t5_hover_land.py`.

**Architecture:** Single self-contained script (matching the existing `t5_hover_land.py`/`t5b_zdistance_hold.py` convention), split internally into a pure, hardware-free validation function (`validate_flight_plan`) and a hardware-driving `main()` that lazily imports `cflib` so the pure logic stays unit-testable without cflib installed. A generic `ramp_to(dx, dy, h, duration_s)` linear-interpolation primitive (generalizing t5's height-only `send_ramp`) backs every command (`takeoff`, `goto`, `land`, emergency descent).

**Tech Stack:** Python 3, `cflib` (Crazyflie Python library, hardware-only, not installed in this dev sandbox), stdlib `unittest` for the one pure-logic unit (no new dependency needed).

## Global Constraints

- Reference spec: `docs/superpowers/specs/2026-09-12-t6-flight-sequence-design.md`
- Do NOT use the firmware `HighLevelCommander` (`crtp_commander_high_level.c`) — unverified in flight on this firmware; keep using `send_position_setpoint` as t5 does.
- No `yaw` parameter anywhere — yaw is fixed to `yaw0` (the yaw at takeoff) for the whole flight.
- Flight-plan format is a Python list of `(command_name, *args)` tuples, hardcoded at module scope — no config file, no CLI args.
- Commands: `takeoff(height_m[, duration_s])`, `hover(duration_s)`, `goto(dx, dy, h, duration_s)` (`dx`/`dy` relative to takeoff point, `h` absolute), `land([duration_s])`.
- `goto`/`takeoff` bounds: `abs(dx), abs(dy) <= MAX_XY_OFFSET_M` (default `1.0`), `0 < height_m <= MAX_HEIGHT_M` / `0 <= h <= MAX_HEIGHT_M` (default `1.0`).
- First command in the flight plan MUST be `takeoff`; validated before connecting to hardware.
- If the flight plan does not end with an executed `land`, one is appended automatically after the plan finishes.
- `MAX_FLIGHT_TIME_S` is a single deadline covering the entire plan execution, not per-command.
- On `FlightAbort` (watchdog failure) or `KeyboardInterrupt` at any point: ramp from the current target straight to `LAND_HEIGHT_M` over `EMERGENCY_LAND_TIME_S`, then unconditionally stop motors (15x `send_stop_setpoint`) — no touchdown wait in the emergency path (matches t5).
- `cflib` imports must NOT be at module scope in `t6_flight_sequence.py` — they must be local to `main()` so the module (and `validate_flight_plan`) can be imported and unit-tested on a machine without `cflib` installed.

---

### Task 1: Flight-plan validation (pure logic, unit-tested)

**Files:**
- Create: `python/t6_flight_sequence.py` (header comment, constants, `FlightAbort`, `COMMAND_ARG_SPECS`, `validate_flight_plan`, default `FLIGHT_PLAN`)
- Create: `python/test_t6_flight_sequence.py`

**Interfaces:**
- Produces: `validate_flight_plan(plan, max_xy_offset_m=MAX_XY_OFFSET_M, max_height_m=MAX_HEIGHT_M) -> list[str]` — returns a list of Chinese error-message strings; empty list means the plan is valid. Later tasks (Task 2's `main()`) call this before opening any connection.
- Produces module constants: `LIFTOFF_HEIGHT_M=0.03`, `LAND_HEIGHT_M=0.0`, `TAKEOFF_TIME_S=2.0`, `LAND_TIME_S=2.0`, `TOUCHDOWN_ZRANGE_MM=90`, `TOUCHDOWN_CONFIRM_S=0.3`, `TOUCHDOWN_MAX_WAIT_S=5.0`, `CLOSE_TO_GROUND_LOG_ZRANGE_MM=150`, `SEND_PERIOD=0.05`, `STATUS_PRINT_PERIOD_S=0.3`, `EMERGENCY_LAND_TIME_S=1.0`, `LOG_WAIT_TIMEOUT_S=2.0`, `LOG_STALE_TIMEOUT_S=0.3`, `MAX_FLIGHT_TIME_S=30.0`, `RANGE_SANE_MAX_MM=4000`, `VX_KI_OVERRIDE=2.0`, `VY_KI_OVERRIDE=2.0`, `MAX_XY_OFFSET_M=1.0`, `MAX_HEIGHT_M=1.0`, `COMMAND_ARG_SPECS` (dict), `FLIGHT_PLAN` (default example list), `ESTIMATOR_NAMES`, exception class `FlightAbort`.

- [ ] **Step 1: Write the failing test file**

Create `python/test_t6_flight_sequence.py`:

```python
#!/usr/bin/env python3
import unittest

from t6_flight_sequence import FLIGHT_PLAN, MAX_HEIGHT_M, MAX_XY_OFFSET_M, validate_flight_plan


class ValidateFlightPlanTests(unittest.TestCase):
    def test_default_flight_plan_is_valid(self):
        self.assertEqual(validate_flight_plan(FLIGHT_PLAN), [])

    def test_minimal_valid_plan_returns_no_errors(self):
        plan = [("takeoff", 0.3), ("land",)]
        self.assertEqual(validate_flight_plan(plan), [])

    def test_empty_plan_returns_error(self):
        errors = validate_flight_plan([])
        self.assertEqual(len(errors), 1)
        self.assertIn("命令列表为空", errors[0])

    def test_first_command_not_takeoff_returns_error(self):
        plan = [("hover", 1.0), ("takeoff", 0.3), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("第一条命令必须是 takeoff" in e for e in errors))

    def test_unknown_command_name_returns_error(self):
        plan = [("takeoff", 0.3), ("spin", 1.0), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("命令名未知" in e for e in errors))

    def test_wrong_arg_count_returns_error(self):
        plan = [("takeoff", 0.3, 1.0, 99.0), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("参数个数为" in e for e in errors))

    def test_goto_xy_offset_out_of_bounds_returns_error(self):
        plan = [("takeoff", 0.3), ("goto", MAX_XY_OFFSET_M + 0.5, 0.0, 0.3, 2.0), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("超出 ±" in e for e in errors))

    def test_goto_height_out_of_bounds_returns_error(self):
        plan = [("takeoff", 0.3), ("goto", 0.0, 0.0, MAX_HEIGHT_M + 0.5, 2.0), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("高度 h=" in e for e in errors))

    def test_goto_nonpositive_duration_returns_error(self):
        plan = [("takeoff", 0.3), ("goto", 0.1, 0.0, 0.3, 0.0), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("goto duration_s=" in e for e in errors))

    def test_takeoff_height_out_of_bounds_returns_error(self):
        plan = [("takeoff", MAX_HEIGHT_M + 1.0), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("takeoff 高度" in e for e in errors))

    def test_hover_nonpositive_duration_returns_error(self):
        plan = [("takeoff", 0.3), ("hover", 0.0), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("hover duration_s=" in e for e in errors))

    def test_land_optional_duration_is_valid(self):
        plan = [("takeoff", 0.3), ("land", 1.5)]
        self.assertEqual(validate_flight_plan(plan), [])

    def test_land_nonpositive_duration_returns_error(self):
        plan = [("takeoff", 0.3), ("land", 0.0)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("land duration_s=" in e for e in errors))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the test to verify it fails (module doesn't exist yet)**

Run: `cd /data/project/source/peakxie/esp-drone/python && python3 -m unittest test_t6_flight_sequence -v`
Expected: FAIL/ERROR with `ModuleNotFoundError: No module named 't6_flight_sequence'`

- [ ] **Step 3: Create `python/t6_flight_sequence.py` with constants and `validate_flight_plan`**

```python
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
```

Note: `from config import URI, connect_with_timeout` is unused until Task 2's `main()`, which is expected — do not remove it in this step, Task 2 uses it. The `if __name__ == "__main__": pass` placeholder is replaced in Task 2 with a real `main()` call.

- [ ] **Step 4: Run the test to verify it passes**

Run: `cd /data/project/source/peakxie/esp-drone/python && python3 -m unittest test_t6_flight_sequence -v`
Expected: `OK` with 13 tests passing (0 failures, 0 errors).

- [ ] **Step 5: Commit**

```bash
cd /data/project/source/peakxie/esp-drone
git add python/t6_flight_sequence.py python/test_t6_flight_sequence.py
git commit -m "$(cat <<'EOF'
feat: add t6_flight_sequence flight-plan validation

Pure, hardware-free validate_flight_plan() checks command names, arg
counts, and dx/dy/height bounds before any connection is attempted.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 2: Flight execution (`main()`, ramp/hold/touchdown, command dispatch)

**Files:**
- Modify: `python/t6_flight_sequence.py` (append `read_current_estimator`, replace the `if __name__ == "__main__": pass` placeholder with `main()` + real entrypoint)

**Interfaces:**
- Consumes: `validate_flight_plan(plan) -> list[str]`, `FlightAbort`, `FLIGHT_PLAN`, and all constants from Task 1 (`LIFTOFF_HEIGHT_M`, `LAND_HEIGHT_M`, `TAKEOFF_TIME_S`, `LAND_TIME_S`, `TOUCHDOWN_ZRANGE_MM`, `TOUCHDOWN_CONFIRM_S`, `TOUCHDOWN_MAX_WAIT_S`, `CLOSE_TO_GROUND_LOG_ZRANGE_MM`, `SEND_PERIOD`, `STATUS_PRINT_PERIOD_S`, `EMERGENCY_LAND_TIME_S`, `LOG_WAIT_TIMEOUT_S`, `LOG_STALE_TIMEOUT_S`, `MAX_FLIGHT_TIME_S`, `RANGE_SANE_MAX_MM`, `VX_KI_OVERRIDE`, `VY_KI_OVERRIDE`, `ESTIMATOR_NAMES`), `URI`/`connect_with_timeout` from `config.py`.
- Produces: `main()` (no args, no return) as the script entrypoint — nothing else in this task is consumed by later tasks (Task 3 is a manual hardware run of this same `main()`).
- Constraint carried over from Task 1: `cflib` imports (`cflib.crtp`, `Crazyflie`, `LogConfig`) MUST be local to `main()`, not at module scope — this is what Step 2 below checks.

- [ ] **Step 1: Append `read_current_estimator` and `main()` to `python/t6_flight_sequence.py`**

Replace the trailing `if __name__ == "__main__": pass` block with:

```python
def read_current_estimator(cf, timeout_s=2.0):
    """读取 stabilizer.estimator 参数，返回当前值（1=complementary, 2=kalman），超时返回 None。"""
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


def main():
    # cflib 只在这里 import：让本模块（尤其是 validate_flight_plan）在没有装 cflib 的机器上
    # 也能被 import 和单测，只有真正执行 main() 飞行时才需要 cflib。
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig

    errors = validate_flight_plan(FLIGHT_PLAN)
    if errors:
        print("命令列表校验失败，拒绝执行：", flush=True)
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
        "x_est": None,
        "y_est": None,
        "yaw_est": None,
        "thrust_est": None,
        "x0": None,
        "y0": None,
        "yaw0": None,
    }

    def aux_cb(_timestamp, data, _logconf):
        state["zrange_mm"] = data["range.zrange"]
        state["z_est"] = data["stateEstimate.z"]
        state["x_est"] = data["stateEstimate.x"]
        state["y_est"] = data["stateEstimate.y"]
        state["yaw_est"] = data["stabilizer.yaw"]
        state["thrust_est"] = data["stabilizer.thrust"]
        if state["x0"] is None:
            state["x0"] = state["x_est"]
            state["y0"] = state["y_est"]
            state["yaw0"] = state["yaw_est"]
        state["last_log_t"] = time.monotonic()

    aux_lg = LogConfig(name="aux", period_in_ms=50)
    aux_lg.add_variable("range.zrange", "uint16_t")
    aux_lg.add_variable("stateEstimate.z", "float")
    aux_lg.add_variable("stateEstimate.x", "float")
    aux_lg.add_variable("stateEstimate.y", "float")
    aux_lg.add_variable("stabilizer.yaw", "float")
    aux_lg.add_variable("stabilizer.thrust", "float")
    cf.log.add_config(aux_lg)
    aux_lg.data_received_cb.add_callback(aux_cb)
    aux_lg.start()

    try:
        estimator = read_current_estimator(cf)
        if estimator != 2:
            print(
                "警告：当前不是 kalman 估计器——没有检测到光流 deck，或者 "
                "CONFIG_SENSORS_ENABLE_DECK 没有开。定高仍会依赖测距/气压工作，但没有水平位置"
                "修正，飞机可能会缓慢漂移，请留意周围净空。",
                flush=True,
            )

        cf.param.set_value("velCtlPid.vxKi", str(VX_KI_OVERRIDE))
        cf.param.set_value("velCtlPid.vyKi", str(VY_KI_OVERRIDE))
        time.sleep(0.2)
        print(
            f"已将 velCtlPid.vxKi/vyKi 覆盖为 {VX_KI_OVERRIDE}/{VY_KI_OVERRIDE}（默认 1.0，"
            "断电重启会恢复默认）。",
            flush=True,
        )

        deadline = time.monotonic() + LOG_WAIT_TIMEOUT_S
        while state["last_log_t"] is None and time.monotonic() < deadline:
            time.sleep(0.02)
        if state["last_log_t"] is None:
            print("错误：等不到 range.zrange/stateEstimate.z 日志，遥测未连通，放弃起飞。", flush=True)
            return

        zrange0 = state["zrange_mm"]
        if zrange0 is None or zrange0 > RANGE_SANE_MAX_MM:
            print(
                f"错误：起飞前 range.zrange={zrange0}mm 超出合理范围（上限 {RANGE_SANE_MAX_MM}mm），"
                "怀疑测距传感器读数异常，放弃起飞。"
                " 请确认飞机放在平整地面、传感器朝下且未被遮挡。",
                flush=True,
            )
            return

        print(f"起飞前地面测距 = {zrange0}mm，遥测正常，开始执行 {len(FLIGHT_PLAN)} 条命令。", flush=True)

        flight_deadline = time.monotonic() + MAX_FLIGHT_TIME_S
        current_target = {"dx": 0.0, "dy": 0.0, "h": 0.0}
        last_status_print = [0.0]

        def watchdog_ok():
            if time.monotonic() > flight_deadline:
                print("警告：飞行总时长超过硬上限，强制转入紧急下降。", flush=True)
                return False
            if state["last_log_t"] is None or (time.monotonic() - state["last_log_t"]) > LOG_STALE_TIMEOUT_S:
                print("警告：高度日志已停止刷新，链路/主控可能异常，强制转入紧急下降。", flush=True)
                return False
            return True

        def print_status(force=False):
            now = time.monotonic()
            if not force and now - last_status_print[0] < STATUS_PRINT_PERIOD_S:
                return
            last_status_print[0] = now
            x0, y0 = state["x0"], state["y0"]
            dx = (state["x_est"] - x0) if (x0 is not None and state["x_est"] is not None) else None
            dy = (state["y_est"] - y0) if (y0 is not None and state["y_est"] is not None) else None
            print(
                f"    target=(dx={current_target['dx']:.2f}, dy={current_target['dy']:.2f}, "
                f"h={current_target['h']:.2f})  z={state['z_est']}  zrange={state['zrange_mm']}mm  "
                f"thrust={state['thrust_est']}  x={state['x_est']}(drift={dx})  y={state['y_est']}(drift={dy})",
                flush=True,
            )

        def send_target():
            cf.commander.send_position_setpoint(
                state["x0"] + current_target["dx"],
                state["y0"] + current_target["dy"],
                current_target["h"],
                state["yaw0"],
            )

        def ramp_to(dx, dy, h, duration_s):
            """把目标从 current_target 的当前值线性斜坡过渡到 (dx, dy, h)，全程以
            SEND_PERIOD 周期发送 send_position_setpoint。takeoff/goto/land/紧急下降
            全部复用这一个函数（同 t5_hover_land.py 的 send_ramp，泛化到三个轴）。"""
            start_dx, start_dy, start_h = current_target["dx"], current_target["dy"], current_target["h"]
            t0 = time.monotonic()
            while True:
                now = time.monotonic()
                elapsed = now - t0
                frac = min(1.0, elapsed / duration_s) if duration_s > 0 else 1.0
                current_target["dx"] = start_dx + (dx - start_dx) * frac
                current_target["dy"] = start_dy + (dy - start_dy) * frac
                current_target["h"] = start_h + (h - start_h) * frac

                send_target()
                print_status()

                if not watchdog_ok():
                    raise FlightAbort()

                if frac >= 1.0:
                    break
                time.sleep(SEND_PERIOD)

        def hold(duration_s):
            t0 = time.monotonic()
            while time.monotonic() - t0 < duration_s:
                send_target()
                print_status()
                if not watchdog_ok():
                    raise FlightAbort()
                time.sleep(SEND_PERIOD)

        def wait_for_touchdown(max_wait_s):
            """持续发送 LAND_HEIGHT_M 目标，用 range.zrange（比融合后的 stateEstimate.z 少一层
            滤波延迟）判断是否已经稳定卡进地面效应气垫：连续 TOUCHDOWN_CONFIRM_S 都低于
            TOUCHDOWN_ZRANGE_MM 才认为可以停桨（同 t5_hover_land.py 的实测结论）。"""
            current_target["h"] = LAND_HEIGHT_M
            t0 = time.monotonic()
            below_since = None
            while time.monotonic() - t0 < max_wait_s:
                send_target()
                zrange = state["zrange_mm"]
                close_to_ground = zrange is not None and zrange <= CLOSE_TO_GROUND_LOG_ZRANGE_MM
                print_status(force=close_to_ground)
                if not watchdog_ok():
                    raise FlightAbort()

                now = time.monotonic()
                if zrange is not None and zrange <= TOUCHDOWN_ZRANGE_MM:
                    if below_since is None:
                        below_since = now
                    elif now - below_since >= TOUCHDOWN_CONFIRM_S:
                        return
                else:
                    below_since = None
                time.sleep(SEND_PERIOD)

            print(f"警告：等待确认落地超过 {max_wait_s:.1f}s 上限，强制停桨。", flush=True)

        def stop_motors():
            print("停桨。", flush=True)
            for _ in range(15):
                cf.commander.send_stop_setpoint()
                time.sleep(0.02)

        def cmd_takeoff(height_m, duration_s=TAKEOFF_TIME_S):
            print(f"命令 takeoff：爬升到 {height_m:.2f}m（{duration_s:.1f}s）...", flush=True)
            current_target["dx"] = 0.0
            current_target["dy"] = 0.0
            current_target["h"] = LIFTOFF_HEIGHT_M
            ramp_to(0.0, 0.0, height_m, duration_s)

        def cmd_hover(duration_s):
            print(f"命令 hover：悬停 {duration_s:.1f}s...", flush=True)
            hold(duration_s)

        def cmd_goto(dx, dy, h, duration_s):
            print(f"命令 goto：过渡到 dx={dx:.2f} dy={dy:.2f} h={h:.2f}（{duration_s:.1f}s）...", flush=True)
            ramp_to(dx, dy, h, duration_s)

        def cmd_land(duration_s=LAND_TIME_S):
            print(f"命令 land：降落（{duration_s:.1f}s）...", flush=True)
            ramp_to(current_target["dx"], current_target["dy"], LAND_HEIGHT_M, duration_s)
            wait_for_touchdown(TOUCHDOWN_MAX_WAIT_S)
            stop_motors()

        command_handlers = {
            "takeoff": cmd_takeoff,
            "hover": cmd_hover,
            "goto": cmd_goto,
            "land": cmd_land,
        }

        executed_land = False
        try:
            for name, *args in FLIGHT_PLAN:
                command_handlers[name](*args)
                if name == "land":
                    executed_land = True

            if not executed_land:
                print("命令列表未以 land 结尾，自动补一次降落。", flush=True)
                cmd_land()

        except (FlightAbort, KeyboardInterrupt):
            print(
                f"触发紧急下降：从当前目标 dx={current_target['dx']:.2f} dy={current_target['dy']:.2f} "
                f"h={current_target['h']:.2f} 快速降到地面（{EMERGENCY_LAND_TIME_S:.1f}s）...",
                flush=True,
            )
            try:
                ramp_to(current_target["dx"], current_target["dy"], LAND_HEIGHT_M, EMERGENCY_LAND_TIME_S)
            except (FlightAbort, KeyboardInterrupt):
                pass  # 已经在尽力下降了，watchdog 再触发也不再重入，直接走到下面的停桨
            stop_motors()

        print("飞行结束。", flush=True)
    finally:
        aux_lg.stop()
        cf.close_link()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Verify no top-level `cflib` import was introduced (regression check for testability)**

Run: `cd /data/project/source/peakxie/esp-drone/python && grep -n "^import cflib\|^from cflib" t6_flight_sequence.py`
Expected: no output (empty) — a match here means `cflib` leaked to module scope and Task 1's tests will break on a machine without `cflib`.

- [ ] **Step 3: Run the Task 1 test suite again to confirm no regression**

Run: `cd /data/project/source/peakxie/esp-drone/python && python3 -m unittest test_t6_flight_sequence -v`
Expected: `OK`, same 13 passing tests as Task 1 — this proves the module still imports cleanly (and `validate_flight_plan` still behaves correctly) on a machine without `cflib` installed, even after adding `main()`.

- [ ] **Step 4: Byte-compile check**

Run: `cd /data/project/source/peakxie/esp-drone/python && python3 -m py_compile t6_flight_sequence.py`
Expected: no output, exit code 0 (confirms no syntax errors in the appended code).

- [ ] **Step 5: Commit**

```bash
cd /data/project/source/peakxie/esp-drone
git add python/t6_flight_sequence.py
git commit -m "$(cat <<'EOF'
feat: implement t6_flight_sequence main() flight execution

Adds ramp_to/hold/wait_for_touchdown primitives and takeoff/hover/goto
/land command dispatch, reusing t5_hover_land.py's watchdog, touchdown
detection, and emergency-descent safety model. cflib imports are local
to main() so validate_flight_plan stays unit-testable without cflib.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 3: Manual real-hardware flight verification

**This task must be run by a human, on-site, with the physical drone. Do not run this task unattended or let an autonomous agent execute it without a human present and ready to cut power.**

**Files:** none (no code changes — this is a verification pass on the artifact from Tasks 1–2)

**Interfaces:**
- Consumes: `python/t6_flight_sequence.py`'s `main()` entrypoint and its module-level `FLIGHT_PLAN` constant (edit this constant directly in the file to choose what to test, same workflow as editing `t5_hover_land.py`).

- [ ] **Step 1: Minimal smoke flight — confirm parity with t5's behavior**

Edit `FLIGHT_PLAN` in `python/t6_flight_sequence.py` to:

```python
FLIGHT_PLAN = [
    ("takeoff", 0.3),
    ("hover", 2.0),
    ("land",),
]
```

Run: `cd /data/project/source/peakxie/esp-drone/python && python3 t6_flight_sequence.py`
Expected (manual observation, same as a `t5_hover_land.py` run): clean vertical takeoff to ~0.3m, stable hover for ~2s with no violent drift, controlled descent, touchdown detected before motors stop (no free-fall from a few cm up), motors stop within ~2s of `land` starting. Console prints `命令 takeoff：...` → `命令 hover：...` → `命令 land：...` → `停桨。` → `飞行结束。` in order.

- [ ] **Step 2: Multi-command sequence with `goto`**

Restore `FLIGHT_PLAN` to the full default (takeoff 0.5m → hover 3s → goto forward 0.3m at 0.5m → hover 2s → goto back to origin at 0.3m → land), or use the exact default already in the file.

Run: `cd /data/project/source/peakxie/esp-drone/python && python3 t6_flight_sequence.py`
Expected (manual observation): each `goto` produces a smooth, controlled horizontal move over the given `duration_s` (no sudden jump/lurch), console prints a `命令 goto：...` line per waypoint with the target `dx/dy/h` before each transition, final `land` behaves the same as Step 1.

- [ ] **Step 3: Ctrl+C emergency-descent check**

Run: `cd /data/project/source/peakxie/esp-drone/python && python3 t6_flight_sequence.py`, then press Ctrl+C once while the drone is mid-`hover` or mid-`goto` (comfortably above `LIFTOFF_HEIGHT_M`, e.g. after it reaches 0.5m).
Expected (manual observation): console prints `触发紧急下降：...`, drone descends on a fast but continuous ramp (not an instant motor cutoff), then `停桨。` once close to the ground.

- [ ] **Step 4: Record the outcome**

If all three steps behave as expected, the implementation is verified. If any step deviates (drift, hard landing, failure to detect touchdown, etc.), do not mark this task complete — capture the observed `target=(...) z=... zrange=...` console lines around the deviation (same diagnostic fields t5_hover_land.py prints) for follow-up debugging, matching the iterative process already used to harden `t5_hover_land.py`.
