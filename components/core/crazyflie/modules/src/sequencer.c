/*
sequencer.c: 板载、传感器确认式的飞行步骤序列执行器。

一旦 SEQ_START，整套 TAKEOFF_SENSOR / DELAY / LAND_SENSOR 步骤完全在固件的
专职任务里自主跑完：每一步是否"完成"，判据是 VL53L1 原始测距
（rangeGet(rangeDown)，mm）是否连续达标，不是时钟/duration——具体原因见
docs/superpowers/specs/2026-09-13-sensor-gated-sequencer-design.md 第1节。

x/y/yaw 全程固定为 0（modeAbs 闭环，见设计文档第4节）：本次三种步骤都不会
水平移动或转向。

并发模型：state!=SEQ_IDLE 期间，sequencerTask 是所有可变字段的唯一写者，
但有四个例外，均从 CRTP 命令处理任务跨任务写入：sequencerClear()/
sequencerAddStep()（仅在 state==SEQ_IDLE 时写 stepBuffer/stepCount）、
sequencerStart()（IDLE->RUNNING/LANDING 的启动写入，见该函数注释）、
sequencerCancel()（state==SEQ_RUNNING 时跨任务调用 enterLanding()）、
sequencerAbortToIdle()（任意状态下可能被 STOP 命令调用，跨任务清空
state/currentStepIdx/stepCount）。这些跨任务写入之间不加互斥量，只用
volatile 保证不被编译器优化掉——因为 state 和 currentStepIdx 是两个独立
变量，读者有可能读到"旧 state + 新 currentStepIdx"这种不一致组合（比如
currentStepIdx 已经被改写成 abort 哨兵值，或者 stepCount 已经被清成 0，
但这一次读到的 state 还是旧值 SEQ_RUNNING）。真正防止这类不一致导致越界
读的，是 sequencerTick() 里对 stepBuffer[currentStepIdx] 解引用前的
currentStepIdx < stepCount 边界检查（见该函数），不是"跨任务写入足够
罕见"这类假设——不一致的一 tick（至多 20ms）会被这个检查安全地跳过，
下一 tick 一定能读到一致的新状态，物理上无实际影响。
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
      // currentStepIdx 是另一个可能被 sequencerCancel()/sequencerAbortToIdle()
      // 跨任务并发改写的 volatile 字段，跟这里读到的 state 不是同一次原子
      // 快照：有可能读到"state 还是旧的 SEQ_RUNNING，但 currentStepIdx 已经
      // 被改写成 abort 哨兵值，或 stepCount 已经被清成 0"这种不一致组合。
      // 越界就跳过这一 tick 的派发，下一个 20ms tick 一定能读到一致的新
      // 状态——不会有正确性损失，只是最多晚一个 tick 反应。
      if (currentStepIdx < stepCount) {
        const sequencerStep_t* step = &stepBuffer[currentStepIdx];
        if (step->type == SEQUENCER_STEP_TAKEOFF_SENSOR) {
          tickTakeoff(now, rangeMm);
        } else if (step->type == SEQUENCER_STEP_DELAY) {
          tickDelay(now);
        }
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

  commandedZM = startZM;

  // 注意顺序：state 必须在对应 enterXxx() 把 activeTakeoff/activeDelay/
  // currentStepType 都填好之后才置为非 IDLE——sequencerTask 只看 state 是否
  // 非 IDLE 就会开始读这些字段，如果 state 提前变成 SEQ_RUNNING，
  // sequencerTask 有可能在 activeTakeoff/activeDelay 还是上一次序列的残留值
  // 时就跑起来，导致虚假的立即超时。LAND_SENSOR 分支不用在这里单独设置
  // state：enterLanding() 内部已经保证了同样"数据先备好、state 最后写"的
  // 顺序。
  const sequencerStep_t* step = &stepBuffer[0];
  switch (step->type) {
    case SEQUENCER_STEP_TAKEOFF_SENSOR:
      enterTakeoff(now, startZM, step);
      currentStepIdx = 0;
      state = SEQ_RUNNING;
      break;
    case SEQUENCER_STEP_DELAY:
      enterDelay(now, startZM, step);
      currentStepIdx = 0;
      state = SEQ_RUNNING;
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
