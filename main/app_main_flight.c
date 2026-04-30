/**
 * app_main_flight.c - 自动飞行序列
 *
 * 运行流程 (受 config.h 里 MOTOR_OUTPUT_DISABLE 控制):
 *   Phase 2a (MOTOR_OUTPUT_DISABLE 定义): 切 Kalman -> 等收敛 -> 打印 state.position
 *                                         让人手推飞机验证位置估计方向, 不起飞
 *   Phase 2b (MOTOR_OUTPUT_DISABLE 注释): 切 Kalman -> 等收敛 -> high-level takeoff 0.5m
 */

#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include "config.h"
#include "crtp_commander_high_level.h"
#include "log.h"
#include "param.h"
#include "stabilizer.h"
#include "stabilizer_types.h"
#include "stm32_legacy.h"

#define DEBUG_MODULE "APP_FLIGHT"
#include "debug_cf.h"

/* kalmanEstimator = 2 (见 estimator.h: anyEstimator=0, complementary=1, kalman=2) */
#define ESTIMATOR_KALMAN  2

void appMain(void) {
  DEBUG_PRINT("APP started!\n");

  /* 切到 Kalman 估计器. stabilizer 任务会在下个 tick 检测到并切换,
   * 切换时会重新 init Kalman, 清所有状态为 0. */
  paramVarId_t idEst = paramGetVarId("stabilizer", "estimator");
  paramSetInt(idEst, ESTIMATOR_KALMAN);
  DEBUG_PRINT("Requested Kalman estimator\n");
  vTaskDelay(M2T(500));

  /* 等 10 秒: 陀螺仪偏置校准 + Kalman 静置收敛 (飞机必须水平放稳) */
  DEBUG_PRINT("Waiting 10s for Kalman to converge (KEEP STILL)...\n");
  vTaskDelay(M2T(10000));

#ifdef MOTOR_OUTPUT_DISABLE
  /* === Phase 2a: 电机锁死, 手推验证 Kalman 位置估计方向 ===
   * 期望: 手推飞机往前 -> state.position.x 增加
   *       手推飞机往右 -> state.position.y 减小 (Crazyflie FLU: +Y=左)
   * 每秒打印一次, 持续 30 秒. */
  DEBUG_PRINT("=== Phase 2a: MOTOR DISABLED, push drone to verify Kalman ===\n");
  DEBUG_PRINT("Expect: forward -> x+, right -> y-, up -> z+\n");
  state_t s;
  for (int i = 0; i < 30; ++i) {
    stabilizerGetState(&s);
    DEBUG_PRINT("t=%2ds pos[x=%+6.2f y=%+6.2f z=%+5.2f]m vel[%+5.2f %+5.2f %+5.2f] att[r=%+5.1f p=%+5.1f y=%+5.1f]\n",
                i,
                (double)s.position.x, (double)s.position.y, (double)s.position.z,
                (double)s.velocity.x, (double)s.velocity.y, (double)s.velocity.z,
                (double)s.attitude.roll, (double)s.attitude.pitch, (double)s.attitude.yaw);
    vTaskDelay(M2T(1000));
  }
  DEBUG_PRINT("=== Phase 2a window closed, motors still disabled, bye ===\n");
  return;
#else
  /* === Phase 2b: 真起飞 === */
  paramVarId_t idHL = paramGetVarId("commander", "enHighLevel");
  paramSetInt(idHL, 1);
  vTaskDelay(M2T(500));

  DEBUG_PRINT("Takeoff 0.5m in 3s...\n");
  crtpCommanderHighLevelTakeoff(0.5f, 3.0f);
  vTaskDelay(M2T(3500));  /* 爬升完 + 0.5s 裕量 */

  DEBUG_PRINT("Hover 3s...\n");
  /* 悬停期间每秒打印一次状态, 便于观察漂移 */
  state_t s;
  for (int i = 0; i < 3; ++i) {
    stabilizerGetState(&s);
    DEBUG_PRINT("hover t=%d pos[x=%+5.2f y=%+5.2f z=%+5.2f] att[r=%+5.1f p=%+5.1f]\n",
                i,
                (double)s.position.x, (double)s.position.y, (double)s.position.z,
                (double)s.attitude.roll, (double)s.attitude.pitch);
    vTaskDelay(M2T(1000));
  }

  DEBUG_PRINT("Land in 3s...\n");
  crtpCommanderHighLevelLand(0.0f, 3.0f);
  vTaskDelay(M2T(3500));

  DEBUG_PRINT("Done.\n");
  crtpCommanderHighLevelStop();
#endif
}
