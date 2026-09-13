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
