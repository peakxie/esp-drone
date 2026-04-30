/**
 * app_main_flight.c - 自动飞行序列
 *
 * 运行流程 (受 config.h 里 MOTOR_OUTPUT_DISABLE 控制):
 *   Phase 2a (MOTOR_OUTPUT_DISABLE 定义): 切 Kalman -> 等收敛 -> 打印 state.position
 *                                         让人手推飞机验证位置估计方向, 不起飞
 *   Phase 2b (MOTOR_OUTPUT_DISABLE 注释): 切 Kalman -> 等收敛 -> high-level takeoff 0.5m
 *
 * 日志通道: 使用 DEBUG_PRINTI (走 CRTP console), 飞行时通过 WiFi UDP
 * 在 cfclient 的 Console tab 能实时看到. 前提是 config.h 里定义了
 * DEBUG_PRINT_ON_CONSOLE.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include "config.h"
#include "crtp_commander_high_level.h"
#include "log.h"
#include "param.h"
#include "sensors.h"
#include "stabilizer.h"
#include "stabilizer_types.h"
#include "stm32_legacy.h"

#define DEBUG_MODULE "APP_FLIGHT"
#include "debug_cf.h"

/* kalmanEstimator = 2 (见 estimator.h: anyEstimator=0, complementary=1, kalman=2) */
#define ESTIMATOR_KALMAN  2

void appMain(void) {
  DEBUG_PRINTI("APP started!\n");

  /* [1] 先等陀螺仪零偏校准完成.
   *
   * 顺序非常关键: 如果在 gyro 还带偏置的时候切 Kalman, Kalman 会用
   * 带偏 gyro 数据积累 N 秒, 起飞瞬间必翻. 必须先让 sensors 层把 gyro bias
   * 算出来, 再切 Kalman, Kalman 拿到的就是无偏数据.
   *
   * sensorsAreCalibrated() 的实现: 收集 SENSORS_BIAS_SAMPLES 个静置样本
   * 算均值作为 bias. 飞机必须水平静置, 通常 ~2 秒内完成. */
  DEBUG_PRINTI("Waiting for gyro bias calibration (KEEP STILL)...\n");
  uint32_t waited_ms = 0;
  while (!sensorsAreCalibrated()) {
    vTaskDelay(M2T(100));
    waited_ms += 100;
    if (waited_ms % 1000 == 0) {
      DEBUG_PRINTI("  ... still calibrating (%u ms)\n", (unsigned)waited_ms);
    }
    if (waited_ms > 30000) {
      DEBUG_PRINTI("ERROR: gyro calibration timeout, abort\n");
      return;
    }
  }
  DEBUG_PRINTI("Gyro calibrated after %u ms\n", (unsigned)waited_ms);

  /* [2] 再切 Kalman 估计器. 此时 gyro 已无偏, Kalman 用干净数据起步. */
  paramVarId_t idEst = paramGetVarId("stabilizer", "estimator");
  paramSetInt(idEst, ESTIMATOR_KALMAN);
  DEBUG_PRINTI("Requested Kalman estimator\n");
  vTaskDelay(M2T(500));

  /* [3] Kalman 收敛: 用干净 gyro + acc + 光流/ToF 收敛到 (0,0,0).
   * 以前要 10 秒是因为在吃含偏数据, 需要反复纠正. 现在 3 秒够了. */
  DEBUG_PRINTI("Waiting 3s for Kalman to converge (KEEP STILL)...\n");
  vTaskDelay(M2T(3000));

#ifdef MOTOR_OUTPUT_DISABLE
  /* === Phase 2a: 电机锁死, 手推验证 Kalman 位置估计方向 ===
   * 期望: 手推飞机往前 -> state.position.x 增加
   *       手推飞机往右 -> state.position.y 减小 (Crazyflie FLU: +Y=左)
   * 每秒打印一次, 持续 30 秒. */
  DEBUG_PRINTI("=== Phase 2a: MOTOR DISABLED, push drone to verify Kalman ===\n");
  DEBUG_PRINTI("Expect: forward -> x+, right -> y-, up -> z+\n");
  state_t s;
  for (int i = 0; i < 30; ++i) {
    stabilizerGetState(&s);
    DEBUG_PRINTI("t=%2ds pos[x=%+6.2f y=%+6.2f z=%+5.2f]m vel[%+5.2f %+5.2f %+5.2f] att[r=%+5.1f p=%+5.1f y=%+5.1f]\n",
                i,
                (double)s.position.x, (double)s.position.y, (double)s.position.z,
                (double)s.velocity.x, (double)s.velocity.y, (double)s.velocity.z,
                (double)s.attitude.roll, (double)s.attitude.pitch, (double)s.attitude.yaw);
    vTaskDelay(M2T(1000));
  }
  DEBUG_PRINTI("=== Phase 2a window closed, motors still disabled, bye ===\n");
  return;
#else
  /* === Phase 2b: 真起飞 ===
   * 首次 PID 调试用 0.3m 高度 + 5 秒悬停.
   * 起飞/降落 duration=3s 是 Crazyflie 经验值, 更快飞机跟不上, 更慢反而在过渡期
   * 积累位置误差. 5s hover 够观察稳态振荡/漂移. */
  paramVarId_t idHL = paramGetVarId("commander", "enHighLevel");
  paramSetInt(idHL, 1);
  vTaskDelay(M2T(500));

  const float takeoff_height = 0.3f;
  /* takeoff duration 8s: pydrone 起飞瞬间电池压降 0.6V (4.1V -> 3.5V), 怀疑
   * 主电容不够 + 电源去耦不足. 软件层缓解: 延长爬升时间, thrust 斜率更平, 避免
   * 瞬时大电流导致 3.3V LDO 掉压, 进而导致 IMU 读数毛刺 -> Kalman 姿态发散.
   * 根治需要硬件改进 (检查主电容 / 电机 MOSFET 走线粗细).
   * 稳定后可以缩回 3-5s. */
  DEBUG_PRINTI("Takeoff %.2fm in 8s...\n", (double)takeoff_height);
  crtpCommanderHighLevelTakeoff(takeoff_height, 8.0f);
  vTaskDelay(M2T(8500));  /* 爬升完 + 0.5s 裕量 */

  DEBUG_PRINTI("Hover 5s...\n");
  /* 悬停期间每秒打印一次状态, 便于观察漂移.
   * 注: 同样数据用 cfclient Plotter 看波形更直观, 这里的 DEBUG_PRINTI
   * 只是在没连 cfclient 时备用. */
  state_t s;
  for (int i = 0; i < 5; ++i) {
    stabilizerGetState(&s);
    DEBUG_PRINTI("hover t=%d pos[x=%+5.2f y=%+5.2f z=%+5.2f] att[r=%+5.1f p=%+5.1f]\n",
                i,
                (double)s.position.x, (double)s.position.y, (double)s.position.z,
                (double)s.attitude.roll, (double)s.attitude.pitch);
    vTaskDelay(M2T(1000));
  }

  DEBUG_PRINTI("Land in 3s...\n");
  crtpCommanderHighLevelLand(0.0f, 3.0f);
  vTaskDelay(M2T(3500));

  DEBUG_PRINTI("Done.\n");
  crtpCommanderHighLevelStop();
#endif
}
