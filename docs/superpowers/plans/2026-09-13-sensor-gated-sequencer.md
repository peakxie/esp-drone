# 传感器确认式飞行序列（Sequencer）Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在固件里新增一个自主执行的传感器确认式飞行序列能力（`TAKEOFF_SENSOR`/`DELAY`/`LAND_SENSOR` + `CANCEL`/`STOP`），并交付对应的 Python 验证脚本。

**Architecture:** 新增独立模块 `sequencer.c/.h`，用一个专职 FreeRTOS 任务跑状态机，通过 CRTP 复用现有高层命令端口上传/启动/取消序列；`crtp_commander_high_level.c` 只做最小的钩子接入（新增 opcode 分发、`GetSetpoint`/`IsStopped` 两处判断），不改动老的 `planner.c`/`pptraj` 时间轴规划器。完成判定全部基于 VL53L1 原始测距（`rangeGet(rangeDown)`），不基于时钟。

**Tech Stack:** ESP-IDF (FreeRTOS, C99)、现有 CRTP/PARAM/LOG 框架；Python 侧 `cflib` + `unittest`。

**规范来源：** `docs/superpowers/specs/2026-09-13-sensor-gated-sequencer-design.md`（以下简称"设计文档"）。本计划中任何数值/行为都以该文档为准；如本计划的实现细节与设计文档冲突，以设计文档的意图为准，本计划里注明的是具体落地方式。

## Global Constraints

- **本沙箱没有 ESP-IDF 工具链**（`idf.py` 不在 PATH 里，`IDF_PATH` 未设置）。所有固件 C 代码任务**无法在本会话内编译验证**——每个 C 任务的验证步骤是人工审查清单，不是编译/运行。真正的 `idf.py build` + 烧录 + 试飞验证是 Task 5，需要在用户自己配好 ESP-IDF 的机器上执行。
- **CRTP 单包载荷上限 30 字节**（`CRTP_MAX_DATA_SIZE`，见 `crtp.h:33`）。`COMMAND_SEQ_ADD_STEP` 的载荷（1字节类型 + 7个float）正好是 29 字节，加 1 字节 opcode＝30，不能再加字段。
- **本机已装 PMW3901 光流模块**，预期估计器是 kalman（`stabilizer.estimator==2`）；Python 脚本应检测并在非 kalman 时警告（不阻止执行）。
- **Python 侧不安装 cflib**（本沙箱环境验证过 `import cflib` 会失败）。凡是要在本沙箱里跑单元测试的函数，必须在模块顶层不依赖 `cflib`，仅在 `main()` 内部 `import`——跟 `python/t6_flight_sequence.py`、`t7_hl_commander_check.py` 的既有约定完全一致。
- **本次只实现 `TAKEOFF_SENSOR`/`DELAY`/`LAND_SENSOR` 三种步骤**，`GOTO_SENSOR` 的协议位置预留但上传即拒绝（`ENOEXEC`），不实现执行逻辑。
- **x/y/yaw 全程固定为 0**（modeAbs 闭环，见设计文档第4节），本次任何步骤都不改变它们。
- **不改动 `position_controller_pid.c`**（前馈缺失是已知的独立事项，见设计文档第8节，记在项目记忆里，本次不修）。
- 新代码注释延续本仓库现有 C 文件的中文注释风格（对照 `range.c`、`crtp_commander_high_level.c`）。

---

## Task 1: 新增 `sequencer` 模块（状态机核心）

**Files:**
- Modify: `components/config/include/config.h`（新增任务常量）
- Create: `components/core/crazyflie/modules/interface/sequencer.h`
- Create: `components/core/crazyflie/modules/src/sequencer.c`
- Modify: `components/core/crazyflie/CMakeLists.txt`（注册新源文件）

**Interfaces:**
- Produces（Task 2 会直接调用这些符号，签名必须完全一致）：
  - `void sequencerInit(void);`
  - `int sequencerClear(void);`
  - `int sequencerAddStep(const sequencerStep_t* step);`
  - `int sequencerStart(bool oldPlannerIsStopped);`
  - `int sequencerCancel(void);`
  - `void sequencerAbortToIdle(void);`
  - `bool sequencerIsIdle(void);`
  - `bool sequencerIsActive(void);`
  - `void sequencerGetSetpoint(setpoint_t* setpoint);`
  - 类型：`sequencerStep_t { uint8_t type; float params[7]; } __attribute__((packed))`（29 字节），`sequencerStepType_t` 枚举（`SEQUENCER_STEP_TAKEOFF_SENSOR=0`/`SEQUENCER_STEP_DELAY=1`/`SEQUENCER_STEP_LAND_SENSOR=2`/`SEQUENCER_STEP_GOTO_SENSOR=3`预留）。
- Consumes：`range.h` 的 `rangeGet(rangeDown)`（现有，返回 mm 的 float）、`static_mem.h` 的 `STATIC_MEM_TASK_ALLOC`/`STATIC_MEM_TASK_CREATE_PINNED`（现有）、`config.h` 的 `FLIGHT_CTRL_TASK_CORE`（现有）、`stm32_legacy.h` 的 `usecTimestamp()`/`M2T()`（现有）。

### 并发模型说明（写进 sequencer.c 顶部注释，供后续维护者理解，不是可选项）

`sequencerTask`（专职任务，20ms 周期）是所有可变状态机字段（`state`/`currentStepIdx`/`currentStepType`/`commandedZM`/`activeTakeoff`/`activeDelay`/`activeLand`/`stepBuffer`/`stepCount`）在 `state != SEQ_IDLE` 期间的唯一写者。CRTP 命令处理任务（`crtpCommanderHighLevelTask`）只在 `state == SEQ_IDLE` 时写 `stepBuffer`/`stepCount`（`sequencerClear`/`sequencerAddStep`），只有 `sequencerTask` 自己会把 `state` 从非 IDLE 变回 IDLE——所以不存在"两个任务同时以为自己独占 IDLE→非IDLE 转换权"的竞争。`sequencerCancel()`（从 CRTP 任务调用）在 `state==SEQ_RUNNING` 时会跨任务写 `activeLand`/`currentStepIdx`/`state`：跟 `sequencerTask` 里同一 tick 的写入之间存在一个有限的竞态窗口，最坏情况是 `commandedZM` 被多写入一次（至多 20ms）过时的爬升/悬停值后才被下一次 `tickLanding()` 覆盖——对物理飞行（20ms 内最多几毫米高度指令误差）可忽略，因此不加互斥量，用 `volatile` 保证跨任务读写不被编译器优化掉即可。这个理由必须原样保留在代码注释里，不要被后续review"补"成加锁。

- [ ] **Step 1: 在 `config.h` 里新增任务常量**

在 `components/config/include/config.h` 做三处插入（照抄现有 `CMD_HIGH_LEVEL_TASK_*` 系列的写法，紧跟在它们后面）：

第 102 行 `#define CMD_HIGH_LEVEL_TASK_PRI 3` 之后插入：
```c
#define SEQUENCER_TASK_PRI      4
```

第 136 行 `#define CMD_HIGH_LEVEL_TASK_NAME "CMDHL"` 之后插入：
```c
#define SEQUENCER_TASK_NAME     "SEQUENCER"
```

第 162 行 `#define CMD_HIGH_LEVEL_TASK_STACKSIZE (2 * configBASE_STACK_SIZE)` 之后插入：
```c
#define SEQUENCER_TASK_STACKSIZE      (2 * configBASE_STACK_SIZE)
```

- [ ] **Step 2: 创建 `sequencer.h`**

写入 `components/core/crazyflie/modules/interface/sequencer.h`：

```c
#ifndef SEQUENCER_H_
#define SEQUENCER_H_

#include <stdbool.h>
#include <stdint.h>

#include "stabilizer_types.h"

#define SEQUENCER_MAX_STEPS 16
#define SEQUENCER_STEP_PARAM_COUNT 7

typedef enum {
  SEQUENCER_STEP_TAKEOFF_SENSOR = 0,
  SEQUENCER_STEP_DELAY          = 1,
  SEQUENCER_STEP_LAND_SENSOR    = 2,
  SEQUENCER_STEP_GOTO_SENSOR    = 3, // 预留，本次上传直接拒绝，不实现执行逻辑
} sequencerStepType_t;

// 线格式固定 1 + 7*4 = 29 字节，直接从 CRTP 载荷 memcpy 过来（见
// crtp_commander_high_level.c 的 COMMAND_SEQ_ADD_STEP），不能再加字段。
typedef struct {
  uint8_t type; // sequencerStepType_t 之一
  float params[SEQUENCER_STEP_PARAM_COUNT];
} __attribute__((packed)) sequencerStep_t;

// params[] 含义按 type 区分（未用到的槽位忽略）：
//   TAKEOFF_SENSOR : params[0]=target_m, params[1]=stable_hold_s, params[2]=timeout_s
//   DELAY          : params[0]=hold_time_s
//   LAND_SENSOR    : params[0]=target_m, params[1]=duration_s

void sequencerInit(void);

// 由 crtp_commander_high_level.c 的 handleCommand() 调用。成功返回 0，
// 拒绝时返回 errno 风格的错误码（本次统一用 ENOEXEC）。
int sequencerClear(void);
int sequencerAddStep(const sequencerStep_t* step);
int sequencerStart(bool oldPlannerIsStopped);
int sequencerCancel(void);

// 由现有 COMMAND_STOP 的 stop() 处理函数调用：立即把序列打回 IDLE，
// 不经过降落斜坡（真正的急停由此后 commander.c 的空 setpoint 兜底实现，
// 见 crtpCommanderHighLevelIsStopped() 的改动）。
void sequencerAbortToIdle(void);

// 供 crtpCommanderHighLevelIsStopped() 调用：只有序列也处于 IDLE 时才为 true。
bool sequencerIsIdle(void);

// 供 crtpCommanderHighLevelGetSetpoint() 调用：序列正在跑（含降落阶段）时为 true。
bool sequencerIsActive(void);

// 只有在 sequencerIsActive() 为 true 时调用才有意义；填充当前应下发的 setpoint。
void sequencerGetSetpoint(setpoint_t* setpoint);

#endif /* SEQUENCER_H_ */
```

- [ ] **Step 3: 创建 `sequencer.c`（数据结构 + 参数/日志 + 起飞子状态机）**

写入 `components/core/crazyflie/modules/src/sequencer.c` 的前半部分：

```c
/*
sequencer.c: 板载、传感器确认式的飞行步骤序列执行器。

一旦 SEQ_START，整套 TAKEOFF_SENSOR / DELAY / LAND_SENSOR 步骤完全在固件的
专职任务里自主跑完：每一步是否"完成"，判据是 VL53L1 原始测距
（rangeGet(rangeDown)，mm）是否连续达标，不是时钟/duration——具体原因见
docs/superpowers/specs/2026-09-13-sensor-gated-sequencer-design.md 第1节。

x/y/yaw 全程固定为 0（modeAbs 闭环，见设计文档第4节）：本次三种步骤都不会
水平移动或转向。

并发模型：见本文件顶部以外、实现计划 docs/superpowers/plans/
2026-09-13-sensor-gated-sequencer.md 里"并发模型说明"一节的完整论证。
简要结论：state!=SEQ_IDLE 期间，sequencerTask 是所有可变字段的唯一写者；
sequencerCancel() 从另一个任务跨线写入时，最坏情况是 commandedZM 被多写入
一次至多 20ms 的过时值，物理上可忽略，因此不加互斥量，只用 volatile。
*/

#include <errno.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "log.h"
#include "param.h"
#include "range.h"
#include "sequencer.h"
#include "static_mem.h"
#include "stm32_legacy.h"
#include "system.h"

#define SEQUENCER_TICK_MS 20

typedef enum {
  SEQ_IDLE    = 0,
  SEQ_RUNNING = 1,
  SEQ_LANDING = 2,
} sequencerState_t;

// seq.stepIdx 遥测里，"降落阶段是被 CANCEL/超时中止合成出来的，不是序列里真实
// 上传过的某一步" 用这个哨兵值区分。
#define SEQUENCER_ABORT_STEP_IDX 0xFEu
#define SEQUENCER_IDLE_STEP_IDX  0xFFu

static bool isInit = false;

static sequencerStep_t stepBuffer[SEQUENCER_MAX_STEPS];
static uint8_t stepCount = 0;

static volatile sequencerState_t state = SEQ_IDLE;
static volatile uint8_t currentStepIdx = SEQUENCER_IDLE_STEP_IDX;
static volatile uint8_t currentStepType = 0;
static volatile float commandedZM = 0.0f;

STATIC_MEM_TASK_ALLOC(sequencerTask, SEQUENCER_TASK_STACKSIZE);

typedef struct {
  float targetM;
  float stableHoldS;
  float timeoutS;
  float rampStartM;
  float stepStartTimeS;
  float inToleranceSinceS; // < 0 表示"当前不在容差内"
} takeoffState_t;
static takeoffState_t activeTakeoff;

typedef struct {
  float holdZM;
  float stepStartTimeS;
  float holdTimeS;
} delayState_t;
static delayState_t activeDelay;

typedef struct {
  float targetM;
  float durationS;
  float rampStartM;
  float phaseStartTimeS;
  float belowSinceS; // < 0 表示"当前未连续低于触地阈值"
} landingState_t;
static landingState_t activeLand;

// 可调参数，默认值取自已经实机调出来的 Python 脚本经验值（见设计文档第7节）。
static float seqTakeoffVelMps       = 0.15f;
static float seqTolTakeoffMm        = 30.0f;
static float seqTouchdownMm         = 80.0f;
static float seqTouchdownConfirmS   = 0.3f;
static float seqLandMaxWaitS        = 5.0f;
static float seqCancelLandDurationS = 2.0f;
static float seqRangeSaneMaxMm      = 4000.0f;

// 遥测镜像，每个 tick 结束时统一刷新一次。
static uint8_t seqStateLog;
static uint8_t seqStepIdxLog;
static uint8_t seqStepTypeLog;
static uint32_t seqElapsedMsLog;

static void finishSequence(void)
{
  state = SEQ_IDLE;
  currentStepIdx = SEQUENCER_IDLE_STEP_IDX;
  stepCount = 0;
}

static void enterLanding(float now, float fromZM, float targetM, float durationS, uint8_t stepIdxForTelemetry)
{
  activeLand.targetM = targetM;
  activeLand.durationS = durationS;
  activeLand.rampStartM = fromZM;
  activeLand.phaseStartTimeS = now;
  activeLand.belowSinceS = -1.0f;
  currentStepIdx = stepIdxForTelemetry;
  currentStepType = SEQUENCER_STEP_LAND_SENSOR;
  state = SEQ_LANDING;
}

static void enterTakeoff(float now, float fromZM, const sequencerStep_t* step)
{
  activeTakeoff.targetM = step->params[0];
  activeTakeoff.stableHoldS = step->params[1];
  activeTakeoff.timeoutS = step->params[2];
  activeTakeoff.rampStartM = fromZM;
  activeTakeoff.stepStartTimeS = now;
  activeTakeoff.inToleranceSinceS = -1.0f;
  currentStepType = SEQUENCER_STEP_TAKEOFF_SENSOR;
}

static void enterDelay(float now, float fromZM, const sequencerStep_t* step)
{
  activeDelay.holdZM = fromZM;
  activeDelay.stepStartTimeS = now;
  activeDelay.holdTimeS = step->params[0];
  currentStepType = SEQUENCER_STEP_DELAY;
}
```

- [ ] **Step 4: 追加推进/tick 逻辑**

在 Step 3 写的内容后面继续追加到同一个文件：

```c
static void advanceToNextStep(float now)
{
  float fromZM = commandedZM;
  uint8_t nextIdx = currentStepIdx + 1;

  // sequencerStart() 已经要求最后一步必须是 LAND_SENSOR，而 LAND_SENSOR
  // 永远通过 finishSequence() 结束、不会调用 advanceToNextStep()——所以正常
  // 情况下这里 nextIdx 一定 < stepCount。这个越界检查是防御性兜底，不是
  // 期望路径。
  if (nextIdx >= stepCount) {
    finishSequence();
    return;
  }

  currentStepIdx = nextIdx;
  const sequencerStep_t* step = &stepBuffer[nextIdx];
  switch (step->type) {
    case SEQUENCER_STEP_TAKEOFF_SENSOR:
      enterTakeoff(now, fromZM, step);
      break;
    case SEQUENCER_STEP_DELAY:
      enterDelay(now, fromZM, step);
      break;
    case SEQUENCER_STEP_LAND_SENSOR:
      enterLanding(now, fromZM, step->params[0], step->params[1], nextIdx);
      break;
    default:
      // 不可达：sequencerAddStep() 已经拒绝了未知类型和 GOTO_SENSOR。
      finishSequence();
      break;
  }
}

static void tickTakeoff(float now, float rangeMm)
{
  float elapsed = now - activeTakeoff.stepStartTimeS;
  float direction = (activeTakeoff.targetM >= activeTakeoff.rampStartM) ? 1.0f : -1.0f;
  float rampZ = activeTakeoff.rampStartM + direction * seqTakeoffVelMps * elapsed;
  if ((direction > 0.0f && rampZ > activeTakeoff.targetM) ||
      (direction < 0.0f && rampZ < activeTakeoff.targetM)) {
    rampZ = activeTakeoff.targetM;
  }
  commandedZM = rampZ;

  bool sensorValid = rangeMm <= seqRangeSaneMaxMm;
  bool inTolerance = sensorValid && fabsf(rangeMm - activeTakeoff.targetM * 1000.0f) <= seqTolTakeoffMm;
  if (inTolerance) {
    if (activeTakeoff.inToleranceSinceS < 0.0f) {
      activeTakeoff.inToleranceSinceS = now;
    }
    if (now - activeTakeoff.inToleranceSinceS >= activeTakeoff.stableHoldS) {
      advanceToNextStep(now);
      return;
    }
  } else {
    activeTakeoff.inToleranceSinceS = -1.0f;
  }

  if (elapsed >= activeTakeoff.timeoutS) {
    // 已确认的策略：超时不是"当作完成继续走"，是中止整个序列去执行 LAND_SENSOR。
    enterLanding(now, commandedZM, 0.0f, seqCancelLandDurationS, SEQUENCER_ABORT_STEP_IDX);
  }
}

static void tickDelay(float now)
{
  commandedZM = activeDelay.holdZM;
  if (now - activeDelay.stepStartTimeS >= activeDelay.holdTimeS) {
    advanceToNextStep(now);
  }
}

static void tickLanding(float now, float rangeMm)
{
  float elapsed = now - activeLand.phaseStartTimeS;
  float frac = activeLand.durationS > 0.0f ? fminf(1.0f, elapsed / activeLand.durationS) : 1.0f;
  commandedZM = activeLand.rampStartM + (activeLand.targetM - activeLand.rampStartM) * frac;

  bool sensorValid = rangeMm <= seqRangeSaneMaxMm;
  bool touchedDown = sensorValid && rangeMm <= seqTouchdownMm;
  if (touchedDown) {
    if (activeLand.belowSinceS < 0.0f) {
      activeLand.belowSinceS = now;
    }
    if (now - activeLand.belowSinceS >= seqTouchdownConfirmS) {
      finishSequence();
      return;
    }
  } else {
    activeLand.belowSinceS = -1.0f;
  }

  if (elapsed >= activeLand.durationS + seqLandMaxWaitS) {
    // 硬性安全上限：就算没确认触地也要切电机。
    finishSequence();
  }
}

static float elapsedInCurrentPhaseS(float now)
{
  switch (currentStepType) {
    case SEQUENCER_STEP_TAKEOFF_SENSOR:
      return now - activeTakeoff.stepStartTimeS;
    case SEQUENCER_STEP_DELAY:
      return now - activeDelay.stepStartTimeS;
    case SEQUENCER_STEP_LAND_SENSOR:
      return now - activeLand.phaseStartTimeS;
    default:
      return 0.0f;
  }
}

static void sequencerTick(float now, float rangeMm)
{
  switch (state) {
    case SEQ_RUNNING: {
      const sequencerStep_t* step = &stepBuffer[currentStepIdx];
      if (step->type == SEQUENCER_STEP_TAKEOFF_SENSOR) {
        tickTakeoff(now, rangeMm);
      } else if (step->type == SEQUENCER_STEP_DELAY) {
        tickDelay(now);
      }
      // LAND_SENSOR 不会在这里被派发到：advanceToNextStep() 进入它时总是
      // 调用 enterLanding()，会先把 state 切到 SEQ_LANDING。
      break;
    }
    case SEQ_LANDING:
      tickLanding(now, rangeMm);
      break;
    default:
      break; // SEQ_IDLE：什么都不做
  }

  seqStateLog = (uint8_t)state;
  seqStepIdxLog = currentStepIdx;
  seqStepTypeLog = currentStepType;
  seqElapsedMsLog = (state == SEQ_IDLE) ? 0 : (uint32_t)(elapsedInCurrentPhaseS(now) * 1000.0f);
}

static void sequencerTask(void* param)
{
  TickType_t lastWakeTime;

  systemWaitStart();
  lastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&lastWakeTime, M2T(SEQUENCER_TICK_MS));
    if (state == SEQ_IDLE) {
      continue;
    }
    float now = usecTimestamp() / 1e6f;
    float rangeMm = rangeGet(rangeDown);
    sequencerTick(now, rangeMm);
  }
}
```

- [ ] **Step 5: 追加公开 API（CRTP 命令处理函数）+ PARAM/LOG 分组**

继续追加到同一个文件末尾：

```c
void sequencerInit(void)
{
  if (isInit) {
    return;
  }

  state = SEQ_IDLE;
  stepCount = 0;
  currentStepIdx = SEQUENCER_IDLE_STEP_IDX;
  currentStepType = 0;
  commandedZM = 0.0f;

  STATIC_MEM_TASK_CREATE_PINNED(
      sequencerTask, sequencerTask, SEQUENCER_TASK_NAME, NULL, SEQUENCER_TASK_PRI, FLIGHT_CTRL_TASK_CORE);

  isInit = true;
}

int sequencerClear(void)
{
  if (state != SEQ_IDLE) {
    return ENOEXEC;
  }
  stepCount = 0;
  return 0;
}

int sequencerAddStep(const sequencerStep_t* newStep)
{
  if (state != SEQ_IDLE) {
    return ENOEXEC;
  }
  if (stepCount >= SEQUENCER_MAX_STEPS) {
    return ENOEXEC;
  }

  switch (newStep->type) {
    case SEQUENCER_STEP_TAKEOFF_SENSOR: {
      float targetM = newStep->params[0];
      float stableHoldS = newStep->params[1];
      float timeoutS = newStep->params[2];
      if (!(targetM > 0.0f && targetM <= 2.0f)) return ENOEXEC;
      if (!(stableHoldS >= 0.0f)) return ENOEXEC;
      if (!(timeoutS > stableHoldS)) return ENOEXEC;
      break;
    }
    case SEQUENCER_STEP_DELAY: {
      float holdTimeS = newStep->params[0];
      if (!(holdTimeS > 0.0f)) return ENOEXEC;
      break;
    }
    case SEQUENCER_STEP_LAND_SENSOR: {
      float targetM = newStep->params[0];
      float durationS = newStep->params[1];
      if (!(targetM >= 0.0f && targetM <= 2.0f)) return ENOEXEC;
      if (!(durationS > 0.0f)) return ENOEXEC;
      break;
    }
    default:
      // 未知类型，或者预留但本次未实现的 GOTO_SENSOR。
      return ENOEXEC;
  }

  stepBuffer[stepCount] = *newStep;
  stepCount++;
  return 0;
}

int sequencerStart(bool oldPlannerIsStopped)
{
  if (state != SEQ_IDLE) {
    return ENOEXEC;
  }
  if (!oldPlannerIsStopped) {
    return ENOEXEC;
  }
  if (stepCount == 0) {
    return ENOEXEC;
  }
  if (stepBuffer[stepCount - 1].type != SEQUENCER_STEP_LAND_SENSOR) {
    return ENOEXEC;
  }

  float now = usecTimestamp() / 1e6f;
  float startZM = rangeGet(rangeDown) / 1000.0f; // 原始 mm -> m，独立于 state_t，见设计文档第4节

  currentStepIdx = 0;
  commandedZM = startZM;
  state = SEQ_RUNNING;

  const sequencerStep_t* step = &stepBuffer[0];
  switch (step->type) {
    case SEQUENCER_STEP_TAKEOFF_SENSOR:
      enterTakeoff(now, startZM, step);
      break;
    case SEQUENCER_STEP_DELAY:
      enterDelay(now, startZM, step);
      break;
    case SEQUENCER_STEP_LAND_SENSOR:
      enterLanding(now, startZM, step->params[0], step->params[1], 0);
      break;
    default:
      break; // 不可达：sequencerAddStep() 已经拒绝了未知类型
  }

  return 0;
}

int sequencerCancel(void)
{
  if (state == SEQ_IDLE) {
    return ENOEXEC;
  }
  if (state == SEQ_RUNNING) {
    float now = usecTimestamp() / 1e6f;
    enterLanding(now, commandedZM, 0.0f, seqCancelLandDurationS, SEQUENCER_ABORT_STEP_IDX);
  }
  // state == SEQ_LANDING：已经在降落中（正常步骤或者之前的 cancel/超时触发），
  // 幂等 no-op，不算错误。
  return 0;
}

void sequencerAbortToIdle(void)
{
  state = SEQ_IDLE;
  currentStepIdx = SEQUENCER_IDLE_STEP_IDX;
  stepCount = 0;
}

bool sequencerIsIdle(void)
{
  return state == SEQ_IDLE;
}

bool sequencerIsActive(void)
{
  return state != SEQ_IDLE;
}

void sequencerGetSetpoint(setpoint_t* setpoint)
{
  setpoint->position.x = 0.0f;
  setpoint->position.y = 0.0f;
  setpoint->position.z = commandedZM;
  setpoint->velocity.x = 0.0f;
  setpoint->velocity.y = 0.0f;
  setpoint->velocity.z = 0.0f;
  setpoint->attitude.yaw = 0.0f;
  setpoint->attitudeRate.roll = 0.0f;
  setpoint->attitudeRate.pitch = 0.0f;
  setpoint->attitudeRate.yaw = 0.0f;
  setpoint->acceleration.x = 0.0f;
  setpoint->acceleration.y = 0.0f;
  setpoint->acceleration.z = 0.0f;

  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->mode.z = modeAbs;
  setpoint->mode.roll = modeDisable;
  setpoint->mode.pitch = modeDisable;
  setpoint->mode.yaw = modeAbs;
  setpoint->mode.quat = modeDisable;
}

PARAM_GROUP_START(seq)
PARAM_ADD(PARAM_FLOAT, takeoffVelMps, &seqTakeoffVelMps)
PARAM_ADD(PARAM_FLOAT, tolTakeoffMm, &seqTolTakeoffMm)
PARAM_ADD(PARAM_FLOAT, touchdownMm, &seqTouchdownMm)
PARAM_ADD(PARAM_FLOAT, touchdownConfirmS, &seqTouchdownConfirmS)
PARAM_ADD(PARAM_FLOAT, landMaxWaitS, &seqLandMaxWaitS)
PARAM_ADD(PARAM_FLOAT, cancelLandDurationS, &seqCancelLandDurationS)
PARAM_ADD(PARAM_FLOAT, rangeSaneMaxMm, &seqRangeSaneMaxMm)
PARAM_GROUP_STOP(seq)

LOG_GROUP_START(seq)
LOG_ADD(LOG_UINT8, state, &seqStateLog)
LOG_ADD(LOG_UINT8, stepIdx, &seqStepIdxLog)
LOG_ADD(LOG_UINT8, stepType, &seqStepTypeLog)
LOG_ADD(LOG_UINT32, elapsedMs, &seqElapsedMsLog)
LOG_GROUP_STOP(seq)
```

- [ ] **Step 6: 把 `sequencer.c` 注册进构建**

在 `components/core/crazyflie/CMakeLists.txt` 里，`"./modules/src/sensfusion6.c"` 那一行之后插入一行（保持文件里按字母序排列的习惯：`sensfusion6` < `sequencer` < `sitaw`）：

```cmake
                "./modules/src/sensfusion6.c"
                "./modules/src/sequencer.c"
                "./modules/src/sitaw.c"
```

- [ ] **Step 7: 人工审查清单（本沙箱无法编译，用这个清单代替编译验证）**

逐条确认，全部打勾才算这个 Task 完成：

1. `sequencer.h` 里 `sequencerStep_t` 是 `1 + 7*4 = 29` 字节（`uint8_t` 1 字节 + `float[7]` 28 字节），跟设计文档第5节的协议表一致。
2. `sequencer.c` 里每一个从 `sequencer.h` 声明的函数都有对应实现，签名（参数类型、返回类型）逐字符匹配。
3. `SEQUENCER_STEP_GOTO_SENSOR` 在 `sequencerAddStep()` 的 `switch` 里落到 `default` 分支被拒绝（`ENOEXEC`），没有被意外放行。
4. `finishSequence()`、`sequencerAbortToIdle()` 都会把 `stepCount` 清零——确认序列结束/被 STOP 之后，一个陈旧的 `stepBuffer` 不会被误当作"已经上传好可以直接 START"（`sequencerStart()` 本身也会检查 `stepCount==0` 拒绝，双重保险）。
5. `config.h` 新增的三个 `SEQUENCER_TASK_*` 宏名字跟 `sequencer.c` 里 `STATIC_MEM_TASK_ALLOC`/`STATIC_MEM_TASK_CREATE_PINNED` 引用的名字完全一致（`SEQUENCER_TASK_STACKSIZE`/`SEQUENCER_TASK_NAME`/`SEQUENCER_TASK_PRI`）。
6. `CMakeLists.txt` 新增的那一行路径拼写（`./modules/src/sequencer.c`）跟 Step 2/3/4/5 实际创建的文件路径完全一致。

- [ ] **Step 8: Commit**

```bash
git add components/config/include/config.h \
        components/core/crazyflie/modules/interface/sequencer.h \
        components/core/crazyflie/modules/src/sequencer.c \
        components/core/crazyflie/CMakeLists.txt
git commit -m "$(cat <<'EOF'
feat: add sensor-gated flight sequencer module

New sequencer.c/.h drives TAKEOFF_SENSOR/DELAY/LAND_SENSOR steps in a
dedicated task, gating step completion on raw VL53L1 range readings
instead of elapsed time. Not yet wired into the CRTP command dispatcher
(next commit).

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: 接入 `crtp_commander_high_level.c`

**Files:**
- Modify: `components/core/crazyflie/modules/src/crtp_commander_high_level.c`

**Interfaces:**
- Consumes：Task 1 产出的全部 `sequencer.h` 符号（见上面 Task 1 的 Produces 列表）。
- Produces：无新增公开符号；`crtp_commander_high_level.c` 原有的公开函数（`crtpCommanderHighLevelInit`/`crtpCommanderHighLevelIsStopped`/`crtpCommanderHighLevelGetSetpoint`）行为发生变化，供 `commander.c`（不改动）继续调用。

- [ ] **Step 1: 加 `#include "sequencer.h"`**

用 Edit 工具在 `components/core/crazyflie/modules/src/crtp_commander_high_level.c` 里找到：

```c
// Crazyswarm includes
#include "crtp.h"
#include "crtp_commander_high_level.h"
#include "planner.h"
#include "log.h"
```

替换为：

```c
// Crazyswarm includes
#include "crtp.h"
#include "crtp_commander_high_level.h"
#include "planner.h"
#include "sequencer.h"
#include "log.h"
```

- [ ] **Step 2: 新增 CRTP opcode**

找到：

```c
  COMMAND_TAKEOFF_WITH_VELOCITY   = 9,
  COMMAND_LAND_WITH_VELOCITY      = 10,
};
```

替换为：

```c
  COMMAND_TAKEOFF_WITH_VELOCITY   = 9,
  COMMAND_LAND_WITH_VELOCITY      = 10,
  COMMAND_SEQ_CLEAR               = 11,
  COMMAND_SEQ_ADD_STEP            = 12,
  COMMAND_SEQ_START               = 13,
  COMMAND_SEQ_CANCEL              = 14,
};
```

- [ ] **Step 3: `handleCommand()` 分发新 opcode**

找到：

```c
    case COMMAND_DEFINE_TRAJECTORY:
      ret = define_trajectory((const struct data_define_trajectory*)data);
      break;
    default:
      ret = ENOEXEC;
      break;
```

替换为：

```c
    case COMMAND_DEFINE_TRAJECTORY:
      ret = define_trajectory((const struct data_define_trajectory*)data);
      break;
    case COMMAND_SEQ_CLEAR:
      ret = sequencerClear();
      break;
    case COMMAND_SEQ_ADD_STEP:
      ret = sequencerAddStep((const sequencerStep_t*)data);
      break;
    case COMMAND_SEQ_START:
      ret = sequencerStart(plan_is_stopped(&planner));
      break;
    case COMMAND_SEQ_CANCEL:
      ret = sequencerCancel();
      break;
    default:
      ret = ENOEXEC;
      break;
```

- [ ] **Step 4: `crtpCommanderHighLevelInit()` 里初始化 sequencer**

找到：

```c
  memoryRegisterHandler(&memDef);
  plan_init(&planner);

  //Start the trajectory task
```

替换为：

```c
  memoryRegisterHandler(&memDef);
  plan_init(&planner);
  sequencerInit();

  //Start the trajectory task
```

- [ ] **Step 5: `crtpCommanderHighLevelIsStopped()` 合并 sequencer 的空闲状态**

找到：

```c
bool crtpCommanderHighLevelIsStopped()
{
  return plan_is_stopped(&planner);
}
```

替换为：

```c
bool crtpCommanderHighLevelIsStopped()
{
  // 老的（时间轴）规划器和新的传感器确认序列都必须空闲，commander.c 才会把
  // setpoint 清零——否则序列 RUNNING 时收到这个查询会被误判为"已停"，导致
  // commander.c 在序列还在跑的时候把 setpoint 清零，飞机永远飞不起来。见
  // docs/superpowers/specs/2026-09-13-sensor-gated-sequencer-design.md 第3节。
  return plan_is_stopped(&planner) && sequencerIsIdle();
}
```

- [ ] **Step 6: `crtpCommanderHighLevelGetSetpoint()` 优先走 sequencer**

找到：

```c
void crtpCommanderHighLevelGetSetpoint(setpoint_t* setpoint, const state_t *state)
{
  xSemaphoreTake(lockTraj, portMAX_DELAY);
  float t = usecTimestamp() / 1e6;
```

替换为：

```c
void crtpCommanderHighLevelGetSetpoint(setpoint_t* setpoint, const state_t *state)
{
  if (sequencerIsActive()) {
    // 传感器确认序列正在跑（含降落阶段）：由它直接决定 setpoint，完全不走
    // 下面的多项式规划器。
    sequencerGetSetpoint(setpoint);
    return;
  }

  xSemaphoreTake(lockTraj, portMAX_DELAY);
  float t = usecTimestamp() / 1e6;
```

- [ ] **Step 7: `stop()` 处理函数里同步中止 sequencer**

找到：

```c
int stop(const struct data_stop* data)
{
  int result = 0;
  if (isInGroup(data->groupMask)) {
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    plan_stop(&planner);
    xSemaphoreGive(lockTraj);
  }
  return result;
}
```

替换为：

```c
int stop(const struct data_stop* data)
{
  int result = 0;
  if (isInGroup(data->groupMask)) {
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    plan_stop(&planner);
    xSemaphoreGive(lockTraj);
    sequencerAbortToIdle();
  }
  return result;
}
```

- [ ] **Step 8: 人工审查清单**

1. 新增的 4 个 opcode 数值（11/12/13/14）跟原有的 0-10 没有重复。
2. `handleCommand()` 的 `switch` 里，新增的 4 个 `case` 都在 `default` 之前，且原有 11 个 `case` 一个没删/没改。
3. `sequencerStart(plan_is_stopped(&planner))` 这一行：确认 `planner` 是本文件已有的 `static struct planner planner;`（不是新引入的符号）。
4. `crtpCommanderHighLevelGetSetpoint()` 的 `sequencerIsActive()` 分支 `return` 之前，没有遗留任何会在 `sequencerGetSetpoint()` 之后又覆盖 `setpoint` 的代码。
5. 通读一遍确认没有改动 `takeoff`/`takeoff2`/`land`/`land2`/`go_to`/`start_trajectory`/`define_trajectory` 这些既有函数的函数体（本任务只加新代码和小范围替换，不应该有无关改动）。

- [ ] **Step 9: Commit**

```bash
git add components/core/crazyflie/modules/src/crtp_commander_high_level.c
git commit -m "$(cat <<'EOF'
feat: wire sensor-gated sequencer into the high-level CRTP commander

Adds SEQ_CLEAR/ADD_STEP/START/CANCEL opcodes on the existing
CRTP_PORT_SETPOINT_HL port, dispatching into sequencer.c. Reuses the
existing COMMAND_STOP path for emergency-stop (now also aborts an
active sequence) and merges sequencer idle-state into
crtpCommanderHighLevelIsStopped() so commander.c's null-setpoint
fallback still fires correctly. The old planner-based takeoff2/land2/
go_to commands are untouched.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: Python 纯函数 + 单元测试（可在本沙箱直接跑）

**Files:**
- Create: `python/t8_sensor_sequence_check.py`（本任务只写纯函数部分）
- Create: `python/test_t8_sensor_sequence_check.py`

**Interfaces:**
- Produces（Task 4 会在同一个 `t8_sensor_sequence_check.py` 文件里继续追加 `main()` 等硬件相关代码，依赖这里定义的符号）：
  - `CRTP_PORT_SETPOINT_HL = 0x08`
  - `SEQ_CLEAR = 11`, `SEQ_ADD_STEP = 12`, `SEQ_START = 13`, `SEQ_CANCEL = 14`
  - `STEP_TYPE_CODES: dict[str, int]`
  - `DEFAULT_SEQUENCE: list[tuple]`
  - `pack_step(step_name: str, params: list[float]) -> bytes`
  - `validate_sequence(steps: list[tuple], max_steps: int = MAX_STEPS) -> list[str]`

- [ ] **Step 1: 写测试（先写测试，再写实现）**

创建 `python/test_t8_sensor_sequence_check.py`：

```python
#!/usr/bin/env python3
import struct
import unittest

from t8_sensor_sequence_check import (
    DEFAULT_SEQUENCE,
    MAX_LAND_TARGET_M,
    MAX_TAKEOFF_TARGET_M,
    pack_step,
    validate_sequence,
)


class PackStepTests(unittest.TestCase):
    def test_pack_step_length_is_29_bytes(self):
        packed = pack_step("TAKEOFF_SENSOR", [0.3, 0.5, 3.0])
        self.assertEqual(len(packed), 29)

    def test_pack_step_type_byte(self):
        packed = pack_step("DELAY", [1.0])
        self.assertEqual(packed[0], 1)

    def test_pack_step_pads_unused_params_with_zero(self):
        packed = pack_step("DELAY", [1.0])
        floats = struct.unpack("<7f", packed[1:])
        self.assertEqual(floats, (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    def test_pack_step_land_sensor_params(self):
        packed = pack_step("LAND_SENSOR", [0.0, 2.0])
        floats = struct.unpack("<7f", packed[1:])
        self.assertEqual(floats[:2], (0.0, 2.0))

    def test_pack_step_unknown_type_raises(self):
        with self.assertRaises(ValueError):
            pack_step("SPIN", [1.0])

    def test_pack_step_too_many_params_raises(self):
        with self.assertRaises(ValueError):
            pack_step("DELAY", [1.0] * 8)


class ValidateSequenceTests(unittest.TestCase):
    def test_default_sequence_is_valid(self):
        self.assertEqual(validate_sequence(DEFAULT_SEQUENCE), [])

    def test_empty_sequence_returns_error(self):
        errors = validate_sequence([])
        self.assertTrue(any("序列为空" in e for e in errors))

    def test_last_step_not_land_sensor_returns_error(self):
        seq = [("TAKEOFF_SENSOR", 0.3, 0.5, 3.0), ("DELAY", 1.0)]
        errors = validate_sequence(seq)
        self.assertTrue(any("最后一步必须是 LAND_SENSOR" in e for e in errors))

    def test_takeoff_target_out_of_bounds_returns_error(self):
        seq = [("TAKEOFF_SENSOR", MAX_TAKEOFF_TARGET_M + 1.0, 0.5, 3.0), ("LAND_SENSOR", 0.0, 2.0)]
        errors = validate_sequence(seq)
        self.assertTrue(any("target=" in e for e in errors))

    def test_takeoff_timeout_not_greater_than_stable_hold_returns_error(self):
        seq = [("TAKEOFF_SENSOR", 0.3, 3.0, 3.0), ("LAND_SENSOR", 0.0, 2.0)]
        errors = validate_sequence(seq)
        self.assertTrue(any("必须大于 stable_hold" in e for e in errors))

    def test_delay_nonpositive_hold_time_returns_error(self):
        seq = [("TAKEOFF_SENSOR", 0.3, 0.5, 3.0), ("DELAY", 0.0), ("LAND_SENSOR", 0.0, 2.0)]
        errors = validate_sequence(seq)
        self.assertTrue(any("DELAY hold_time=" in e for e in errors))

    def test_land_target_out_of_bounds_returns_error(self):
        seq = [("TAKEOFF_SENSOR", 0.3, 0.5, 3.0), ("LAND_SENSOR", MAX_LAND_TARGET_M + 1.0, 2.0)]
        errors = validate_sequence(seq)
        self.assertTrue(any("LAND_SENSOR target=" in e for e in errors))

    def test_land_nonpositive_duration_returns_error(self):
        seq = [("TAKEOFF_SENSOR", 0.3, 0.5, 3.0), ("LAND_SENSOR", 0.0, 0.0)]
        errors = validate_sequence(seq)
        self.assertTrue(any("LAND_SENSOR duration=" in e for e in errors))

    def test_unknown_step_name_returns_error(self):
        seq = [("SPIN", 1.0), ("LAND_SENSOR", 0.0, 2.0)]
        errors = validate_sequence(seq)
        self.assertTrue(any("命令名未知" in e for e in errors))

    def test_too_many_steps_returns_error(self):
        seq = [("DELAY", 0.1)] * 20 + [("LAND_SENSOR", 0.0, 2.0)]
        errors = validate_sequence(seq, max_steps=16)
        self.assertTrue(any("超过固件缓冲区上限" in e for e in errors))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 跑测试，确认失败（`t8_sensor_sequence_check.py` 还不存在）**

Run: `cd python && python3 -m unittest test_t8_sensor_sequence_check -v`
Expected: FAIL，报 `ModuleNotFoundError: No module named 't8_sensor_sequence_check'`

- [ ] **Step 3: 写最小实现**

创建 `python/t8_sensor_sequence_check.py`（本步骤只写到 `validate_sequence` 为止，`main()` 等硬件逻辑留给 Task 4 追加）：

```python
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
```

- [ ] **Step 4: 跑测试，确认通过**

Run: `cd python && python3 -m unittest test_t8_sensor_sequence_check -v`
Expected: 全部 `PASS`（16 个测试用例，`OK`）

- [ ] **Step 5: Commit**

```bash
git add python/t8_sensor_sequence_check.py python/test_t8_sensor_sequence_check.py
git commit -m "$(cat <<'EOF'
test: add pure-function tests for the sensor sequence packet encoding

pack_step()/validate_sequence() mirror t6_flight_sequence.py's
validate_flight_plan() convention: importable and testable without
cflib installed. main() and any hardware I/O land in a follow-up
commit.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Python 硬件验证脚本（`main()`）

**Files:**
- Modify: `python/t8_sensor_sequence_check.py`（追加 `main()` 及硬件相关辅助函数）

**Interfaces:**
- Consumes：Task 3 产出的 `CRTP_PORT_SETPOINT_HL`/`SEQ_CLEAR`/`SEQ_ADD_STEP`/`SEQ_START`/`SEQ_CANCEL`/`STEP_TYPE_CODES`/`DEFAULT_SEQUENCE`/`pack_step`/`validate_sequence`/`ESTIMATOR_NAMES`；`python/config.py` 的 `URI`/`connect_with_timeout`（现有，不改动）。
- Produces：无（`main()` 是脚本入口，不被其他模块导入）。

本任务的代码依赖真实飞机连接，**不能在本沙箱里跑通**（没有 cflib、没有硬件），验证方式是人工审查 + 最终 Task 5 的实机飞行。

- [ ] **Step 1: 替换文件末尾的占位 `if __name__ == "__main__":` 块，追加完整 `main()`**

用 Edit 工具，把 Task 3 Step 3 写的：

```python
if __name__ == "__main__":
    print("这个文件到 Task 4 才有 main()，现在只能被 import 用于单元测试。")
```

替换为：

```python
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

        print(f"开始上传并启动序列（{len(DEFAULT_SEQUENCE)} 步）...", flush=True)
        upload_and_start_sequence(cf, DEFAULT_SEQUENCE)

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
```

注意：文件顶部的 `import struct` 之后还需要加 `import time`（`main()` 及其辅助函数用到），Step 2 处理。

- [ ] **Step 2: 补上缺的 `import time`**

用 Edit 工具，在 `python/t8_sensor_sequence_check.py` 顶部找到：

```python
import struct
```

替换为：

```python
import struct
import time
```

- [ ] **Step 3: 确认 Task 3 的单元测试仍然全部通过（追加 `main()` 不应该影响纯函数）**

Run: `cd python && python3 -m unittest test_t8_sensor_sequence_check -v`
Expected: 仍然全部 `PASS`（`main()`/`_send_sequencer_packet` 等新函数在模块顶层不需要 cflib，只有真正调用 `main()` 时才 `import cflib`，不影响测试收集/运行）

- [ ] **Step 4: 语法检查（本沙箱能做到的最接近"编译"的验证）**

Run: `cd python && python3 -m py_compile t8_sensor_sequence_check.py`
Expected: 无输出，退出码 0（确认没有语法错误；不代表 cflib 相关调用在真实硬件上一定正确，那是 Task 5 的事）

- [ ] **Step 5: 人工审查清单**

1. `main()` 里所有 `state["..."]` 的 key 在 `aux_cb` 里都有对应赋值，没有拼写不一致（比如日志变量名 `seq.stepIdx` vs 代码里的 `"seq_step_idx"`）。
2. `_send_sequencer_packet` 用的 `SEQ_CLEAR`/`SEQ_ADD_STEP`/`SEQ_START`/`SEQ_CANCEL` 数值（11/12/13/14）跟 Task 2 里 `crtp_commander_high_level.c` 新增的 `COMMAND_SEQ_*` 枚举值逐一对应。
3. `pack_step()` 的调用点（`upload_and_start_sequence` 里）参数顺序是 `(name, *params)`，跟 `DEFAULT_SEQUENCE` 元组的 `(name, p0, p1, ...)` 形状一致。
4. `KeyboardInterrupt` 分支不会在 `cf`/`aux_lg` 还没定义时被触发（确认 `try` 块从 `connect_with_timeout` 判断失败 `return` 之后才开始，`except KeyboardInterrupt` 包住的范围里 `cf` 已经赋值）。
5. **未经验证的假设**：本沙箱没装 `cflib`（`pip show cflib` 会失败），`_send_sequencer_packet` 里 `pk.port = CRTP_PORT_SETPOINT_HL` / `pk.data = bytes([opcode]) + payload` 这两行是按 cflib 公开 API 的一般写法编的，没有在真实 `cflib` 上跑过——`CRTPPacket.data` 具体接受 `bytes` 还是要求 `list[int]`/`bytearray`，需要在 Task 5 第一次实际 `import cflib` 时现场确认，如果类型不对当场按 cflib 的报错改成对应类型即可，不是协议设计问题。

- [ ] **Step 6: Commit**

```bash
git add python/t8_sensor_sequence_check.py
git commit -m "$(cat <<'EOF'
feat: add main() flight runner for the sensor sequence check script

Connects, verifies commander.enHighLevel and the kalman estimator,
uploads the default TAKEOFF_SENSOR/DELAY/LAND_SENSOR sequence via raw
CRTP packets, polls seq.state/stepIdx for progress, and falls back to
the existing high_level_commander.stop() e-stop on watchdog trip or
Ctrl+C (via SEQ_CANCEL first). Not yet flight-tested -- see Task 5.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: 真机构建 + 试飞验收（用户在自己的机器上执行）

本任务不产出代码，是一份验收 runbook——本沙箱没有 ESP-IDF 工具链和真实硬件，前四个 Task 的固件代码到目前为止只经过人工审查，**从未编译过**。这份清单跟 `t5`/`t6`/`t7` 系列脚本的既有惯例一致：功能在真正飞过一次之前都算"未验证"。

- [ ] **Step 1: 编译固件**

在装好 ESP-IDF 的机器上：

```bash
idf.py build
```

Expected: 编译成功，无 `sequencer.c`/`crtp_commander_high_level.c` 相关的错误或警告。如果失败，先检查 Task 1/2 的人工审查清单有没有漏掉的签名不一致。

- [ ] **Step 2: 烧录并确认 PARAM/LOG 分组注册成功**

```bash
idf.py flash monitor
```

用 `cfclient` 或 Python 连接后确认能看到 `seq` PARAM 组（`seq.takeoffVelMps` 等 8 个参数，含最终 review 阶段新增的 `seq.rangeStaleMs`）和 `seq` LOG 组（`seq.state`/`seq.stepIdx`/`seq.stepType`/`seq.elapsedMs`）。

- [ ] **Step 3: 地面（不通电机）协议冒烟测试**

飞机放在地面、螺旋桨可以先摘掉或者确认安全的情况下：连接、设置 `commander.enHighLevel=1`、发 `SEQ_CLEAR`/`SEQ_ADD_STEP`×3/`SEQ_START`，观察 `seq.state` 是否从 0 变成 1（TAKEOFF_SENSOR 开始），`seq.stepIdx`/`seq.stepType` 是否符合预期。确认 `seq.elapsedMs` 在增长。

- [ ] **Step 4: 首次实机试飞**

按 `python/t8_sensor_sequence_check.py` 默认的最小序列（`TAKEOFF_SENSOR(0.3m) → DELAY(1.0s) → LAND_SENSOR(0, 2.0s)`），室内、地面平整、四周留够 1m 净空、旁边有人随时准备断电：

```bash
cd python && python3 t8_sensor_sequence_check.py
```

观察打印的 `seq.state`/`stepIdx`/`stepType`/`elapsedMs`/`zrange`/`z` 轨迹，重点确认：
1. TAKEOFF_SENSOR 阶段 `zrange` 确实爬升到目标附近并停留（不是像旧 bug 那样全程贴地）。
2. LAND_SENSOR 阶段电机切断的时刻，`zrange` 已经足够接近地面（不是像旧 bug 那样离地几厘米就切）。
3. `seq.touchdownMm`（默认 80mm）是否需要根据这次实测调整——同 t5/t6/t7 的调参历史，预计需要迭代。

- [ ] **Step 5: CANCEL / STOP 验收**

分别测试：序列执行中 Ctrl+C（应触发 `SEQ_CANCEL`，观察飞机转入降落而不是硬摔或悬停不动）；序列执行中另开一个连接发送现有的 `cf.high_level_commander.stop()`（应立即切电机，观察下落是否符合"急停"预期而不是继续尝试降落斜坡）。

- [ ] **Step 6: 记录调参结果**

如果 Step 4/5 暴露出 `seq.touchdownMm`/`seq.tolTakeoffMm`/`seq.takeoffVelMps` 等默认值不合适，在 `sequencer.c` 里更新默认值（同时说明是第几次试飞发现的、原始值和新值），提交一条新的 commit，注明是参数调优而非行为变更。
