# t7_hl_commander_check.py Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `python/t7_hl_commander_check.py`, a minimal real-flight test script that verifies the ESP-Drone firmware's never-flown `HighLevelCommander` path (CRTP port 0x08, `crtp_commander_high_level.c`) can perform a basic take_off → hover → land using cflib's official `PositionHlCommander`.

**Architecture:** Two layers in one file, same convention as `t6_flight_sequence.py`: (1) pure-logic helpers (`read_current_estimator`, `set_and_verify_param`) that take a duck-typed `cf` object and are unit-testable without cflib installed; (2) `main()`, which lazily imports cflib and drives the actual flight — pre-flight checks (telemetry, range sanity, estimator, and the new mandatory `commander.enHighLevel=1` param) followed by a `with PositionHlCommander(...)` block wrapped in exception handling, plus a background watchdog thread that calls `cf.high_level_commander.stop()` on staleness/timeout.

**Tech Stack:** Python 3, cflib (`cflib.positioning.position_hl_commander.PositionHlCommander`, `cflib.crazyflie.Crazyflie`), `unittest` (stdlib) for tests, existing `python/config.py` for connection.

## Global Constraints

- Must not call `cf.commander.send_*` anywhere in this script — any low-level setpoint forces the firmware's high-level planner back to idle (`commander.c:80-88`). This script is pure high-level-commander only.
- Must set and verify `commander.enHighLevel=1` before calling `PositionHlCommander.take_off()` / entering the `with` block — the firmware silently ignores high-level commands otherwise (`commander.c:48,104-116`).
- Must not use `go_to`/`spiral`/`start_trajectory` — out of scope per the design spec (`docs/superpowers/specs/2026-09-12-t7-hl-commander-check-design.md`).
- `main()` must lazily import `cflib` inside the function body (not at module top level), so the module's helper functions remain importable/testable on a machine without cflib installed (this repo's dev environment has no cflib installed — confirmed during design).
- Takeoff height fixed at `TAKEOFF_HEIGHT_M = 0.3`, hover `HOVER_TIME_S = 3.0`, using `PositionHlCommander`'s library-default velocity (`0.5 m/s`) — no CLI args, no config file, matching the t5/t6/t5b convention of "change the test, change the constants."

---

### Task 1: Pure-logic helpers (`read_current_estimator`, `set_and_verify_param`) with unit tests

**Files:**
- Create: `python/t7_hl_commander_check.py`
- Create: `python/test_t7_hl_commander_check.py`

**Interfaces:**
- Produces: `read_current_estimator(cf, timeout_s=2.0) -> int | None` — identical behavior/signature to the copy in `t6_flight_sequence.py` (returns 1=complementary, 2=kalman, `None` on timeout).
- Produces: `set_and_verify_param(cf, group, name, value, timeout_s=PARAM_SET_TIMEOUT_S) -> bool` — sets `f"{group}.{name}"` via `cf.param.set_value`, waits for the update callback, returns `True` only if a value arrives within `timeout_s` **and** it string-equals `value`; otherwise prints a warning and returns `False`.
- Produces module constants used by Task 2: `TAKEOFF_HEIGHT_M`, `DEFAULT_VELOCITY_MPS`, `HOVER_TIME_S`, `LOG_WAIT_TIMEOUT_S`, `LOG_STALE_TIMEOUT_S`, `MAX_FLIGHT_TIME_S`, `STATUS_PRINT_PERIOD_S`, `PARAM_SET_TIMEOUT_S`, `EMERGENCY_LAND_TIME_S`, `LAND_HEIGHT_M`, `RANGE_SANE_MAX_MM`, `ESTIMATOR_NAMES`.
- Consumes: `python/config.py`'s `URI`, `connect_with_timeout` (imported but only used by Task 2's `main()`; import it in Task 1 so the module is complete, even though nothing in Task 1 calls it yet).

- [ ] **Step 1: Write the failing test file**

Create `python/test_t7_hl_commander_check.py`:

```python
#!/usr/bin/env python3
import unittest

from t7_hl_commander_check import PARAM_SET_TIMEOUT_S, set_and_verify_param


class FakeParam:
    """Stands in for cflib's cf.param. respond=False simulates a callback
    that never fires (timeout). response=None means "echo back whatever
    set_value was called with" (simulates a successful readback)."""

    def __init__(self, response=None, respond=True):
        self._response = response
        self._respond = respond
        self.set_calls = []
        self._cb = None

    def add_update_callback(self, group, name, cb):
        self._cb = cb

    def set_value(self, full_name, value_str):
        self.set_calls.append((full_name, value_str))
        if self._respond:
            response = value_str if self._response is None else self._response
            self._cb(full_name, response)


class FakeCf:
    def __init__(self, response=None, respond=True):
        self.param = FakeParam(response=response, respond=respond)


class SetAndVerifyParamTests(unittest.TestCase):
    def test_success_when_readback_matches(self):
        cf = FakeCf()
        ok = set_and_verify_param(cf, "commander", "enHighLevel", 1, timeout_s=0.05)
        self.assertTrue(ok)
        self.assertEqual(cf.param.set_calls, [("commander.enHighLevel", "1")])

    def test_timeout_when_no_callback_fires(self):
        cf = FakeCf(respond=False)
        ok = set_and_verify_param(cf, "commander", "enHighLevel", 1, timeout_s=0.05)
        self.assertFalse(ok)

    def test_failure_when_readback_value_mismatches(self):
        cf = FakeCf(response="0")
        ok = set_and_verify_param(cf, "commander", "enHighLevel", 1, timeout_s=0.05)
        self.assertFalse(ok)

    def test_default_timeout_constant_is_positive(self):
        self.assertGreater(PARAM_SET_TIMEOUT_S, 0)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the test to verify it fails**

Run (from repo root):
```bash
cd python && python3 -m unittest test_t7_hl_commander_check -v
```
Expected: FAIL — `ModuleNotFoundError: No module named 't7_hl_commander_check'` (the module doesn't exist yet).

- [ ] **Step 3: Create `python/t7_hl_commander_check.py` with constants and the two helpers**

```python
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
```

- [ ] **Step 4: Run the test to verify it passes**

Run:
```bash
cd python && python3 -m unittest test_t7_hl_commander_check -v
```
Expected: `OK` with 4 tests passing:
```
test_default_timeout_constant_is_positive ... ok
test_failure_when_readback_value_mismatches ... ok
test_success_when_readback_matches ... ok
test_timeout_when_no_callback_fires ... ok

----------------------------------------------------------------------
Ran 4 tests in ...s

OK
```

- [ ] **Step 5: Commit**

```bash
git add python/t7_hl_commander_check.py python/test_t7_hl_commander_check.py
git commit -m "$(cat <<'EOF'
feat: add t7_hl_commander_check helpers with unit tests

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 2: `main()` — pre-flight checks, PositionHlCommander flight, watchdog

**Files:**
- Modify: `python/t7_hl_commander_check.py` (replace the `if __name__ == "__main__": pass` placeholder from Task 1 with a real `main()`)

**Interfaces:**
- Consumes from Task 1: `read_current_estimator(cf, timeout_s=2.0) -> int | None`, `set_and_verify_param(cf, group, name, value, timeout_s) -> bool`, and all module constants listed in Task 1's Interfaces.
- Consumes from `config.py`: `URI`, `connect_with_timeout(cf, uri, timeout_s) -> bool`.
- Produces: `main()` — no return value, no parameters; entry point run via `if __name__ == "__main__": main()`.

This task has no hardware-independent unit test (flying is not unit-testable — same as `t6_flight_sequence.py`'s `main()`). Verification here is: (a) the module still imports and the existing Task 1 tests still pass without cflib installed, proving the lazy-import convention was followed correctly, and (b) a manual code-review checklist against the design spec's safety model.

- [ ] **Step 1: Replace the placeholder with `main()`**

In `python/t7_hl_commander_check.py`, replace:

```python
if __name__ == "__main__":
    pass  # main() 由 Task 2 补上
```

with:

```python
def main():
    # cflib 只在这里 import：让本模块的纯逻辑函数（set_and_verify_param、read_current_estimator）
    # 在没有装 cflib 的机器上也能被 import 和单测。
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.positioning.position_hl_commander import PositionHlCommander

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
    }

    def aux_cb(_timestamp, data, _logconf):
        state["zrange_mm"] = data["range.zrange"]
        state["z_est"] = data["stateEstimate.z"]
        state["x_est"] = data["stateEstimate.x"]
        state["y_est"] = data["stateEstimate.y"]
        state["yaw_est"] = data["stabilizer.yaw"]
        state["thrust_est"] = data["stabilizer.thrust"]
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
                "CONFIG_SENSORS_ENABLE_DECK 没有开。本次只测垂直起降，没有水平位置修正时"
                "飞机可能会缓慢漂移，请留意周围净空。",
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

        if not set_and_verify_param(cf, "commander", "enHighLevel", 1, timeout_s=PARAM_SET_TIMEOUT_S):
            print(
                "错误：commander.enHighLevel 设置/回读失败，拒绝起飞——高层命令固件端不会被"
                "处理（commander.c 的 commanderGetSetpoint() 只有这个 param 为真时才会把 setpoint "
                "交给高层规划器）。",
                flush=True,
            )
            return

        print(f"起飞前地面测距 = {zrange0}mm，commander.enHighLevel 已确认，开始起飞。", flush=True)

        stop_event = threading.Event()
        flight_deadline = time.monotonic() + MAX_FLIGHT_TIME_S
        last_status_print = [0.0]

        def watchdog_loop():
            while not stop_event.is_set():
                now = time.monotonic()
                if now > flight_deadline:
                    print("警告：飞行总时长超过硬上限，触发立即停桨。", flush=True)
                    cf.high_level_commander.stop()
                    stop_event.set()
                    break
                if state["last_log_t"] is None or (now - state["last_log_t"]) > LOG_STALE_TIMEOUT_S:
                    print("警告：高度日志已停止刷新，链路/主控可能异常，触发立即停桨。", flush=True)
                    cf.high_level_commander.stop()
                    stop_event.set()
                    break
                if now - last_status_print[0] >= STATUS_PRINT_PERIOD_S:
                    last_status_print[0] = now
                    print(
                        f"    z={state['z_est']}  zrange={state['zrange_mm']}mm  "
                        f"thrust={state['thrust_est']}  x={state['x_est']}  y={state['y_est']}  "
                        f"yaw={state['yaw_est']}",
                        flush=True,
                    )
                time.sleep(0.05)

        watchdog_thread = threading.Thread(target=watchdog_loop, daemon=True)
        watchdog_thread.start()

        try:
            with PositionHlCommander(
                cf,
                default_height=TAKEOFF_HEIGHT_M,
                default_velocity=DEFAULT_VELOCITY_MPS,
            ):
                print(f"已起飞到 {TAKEOFF_HEIGHT_M:.2f}m，悬停 {HOVER_TIME_S:.1f}s...", flush=True)
                t0 = time.monotonic()
                while time.monotonic() - t0 < HOVER_TIME_S and not stop_event.is_set():
                    time.sleep(0.1)
            # 注意：如果看门狗线程在悬停期间已经调用过 stop()，上面 with 块退出时 __exit__
            # 仍会调用一次 land()——对已经停桨的飞机再发一次 land 命令是已知的、可接受的
            # 冗余动作（land() 内部只是发 LAND_2 + 再次 stop()），不会造成新的风险。
            print("飞行结束（已降落）。", flush=True)
        except (KeyboardInterrupt, Exception):
            # __enter__（也就是 take_off()）执行期间的异常/Ctrl+C 不会触发
            # PositionHlCommander.__exit__（Python 的 with 语义：__enter__ 抛异常时不会进入
            # __exit__），这里手动兜底一次 land+stop。悬停期间的异常/Ctrl+C 已经在上面的
            # with 块里被 __exit__ 处理过，这里再兜底一次 stop() 也是安全的。
            print(
                f"触发紧急处理：向固件发送 land+stop（{EMERGENCY_LAND_TIME_S:.1f}s）...",
                flush=True,
            )
            cf.high_level_commander.land(LAND_HEIGHT_M, EMERGENCY_LAND_TIME_S)
            time.sleep(EMERGENCY_LAND_TIME_S)
            cf.high_level_commander.stop()
        finally:
            stop_event.set()
            watchdog_thread.join(timeout=1.0)

    finally:
        aux_lg.stop()
        cf.close_link()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Byte-compile to catch syntax errors**

Run:
```bash
cd python && python3 -m py_compile t7_hl_commander_check.py
```
Expected: no output, exit code 0.

- [ ] **Step 3: Re-run Task 1's tests to confirm the lazy-import convention holds**

Run:
```bash
cd python && python3 -m unittest test_t7_hl_commander_check -v
```
Expected: same 4 tests still pass (`OK`) — this confirms `import cflib...` inside `main()` did not get hoisted to module level and that the module remains importable without cflib installed (this dev machine has no cflib, confirmed earlier: `ModuleNotFoundError: No module named 'cflib'`).

- [ ] **Step 4: Manual review checklist against the design spec's safety model**

Read through `python/t7_hl_commander_check.py` and confirm each of these (no code changes if all pass — this is a review gate, not a coding step):
- [ ] No call to `cf.commander.send_*` anywhere in the file.
- [ ] `commander.enHighLevel` is set and verified via `set_and_verify_param` before the `PositionHlCommander` block, and `main()` returns early (no flight attempt) if it fails.
- [ ] The `with PositionHlCommander(...)` block is wrapped in `try/except (KeyboardInterrupt, Exception)` with a manual `land()+stop()` fallback, covering the `__enter__`-time gap.
- [ ] The watchdog thread checks both `LOG_STALE_TIMEOUT_S` and `MAX_FLIGHT_TIME_S`, and its only action on trigger is `cf.high_level_commander.stop()`.
- [ ] `aux_lg.stop()` and `cf.close_link()` run in the outermost `finally`, so they run even if pre-flight checks return early.
- [ ] Module docstring/header comment at the top of the file states this path has never been flight-verified and lists the "室内、地面平整、四周留够 1m 净空、旁边有人随时准备断电/接住飞机" precaution.
- [ ] `stabilizer.yaw` is included in the telemetry log and printed by the watchdog's status line, so an unexpected yaw rotation during takeoff (cflib's default `yaw=0.0, useCurrentYaw=False` behavior) is visible rather than silent.

- [ ] **Step 5: Commit**

```bash
git add python/t7_hl_commander_check.py
git commit -m "$(cat <<'EOF'
feat: implement t7_hl_commander_check main() flight execution

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Post-plan note (not a task — do not automate)

This plan produces a script that is ready for a **real, physical flight test**. Task 2's manual review checklist is a code-review gate, not a substitute for the actual flight test described in the design spec's "验证方式" section. Running the drone is a physical-world action with real risk (first flight of a never-verified firmware path) and must be done deliberately by the user, in person, with the precautions listed in the script header — not triggered automatically by an agent as part of "finishing the plan."
