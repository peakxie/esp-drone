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
#include "commander.h"
#include "config.h"
#include "crtp_commander_high_level.h"
#include "log.h"
#include "param.h"
#include "sensors.h"
#include "stabilizer.h"
#include "stabilizer_types.h"
#include "stm32_legacy.h"
#include <string.h>

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

  /* [2b] 强制 reset Kalman estimation, 清掉任何可能残留的 pos/vel 状态.
   * 切 Kalman 时内部已经调 estimatorKalmanInit, 这里再 reset 一次确保干净. */
  paramVarId_t idReset = paramGetVarId("kalman", "resetEstimation");
  paramSetInt(idReset, 1);
  vTaskDelay(M2T(200));
  DEBUG_PRINTI("Kalman state reset\n");

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

  /* [2c] TAKEOFF 前最后一刻再 reset 一次 Kalman + 同步 HL commander.
   *
   * 起因 (2026-05-03 栓绳试飞翻机分析): [2b] 第一次 reset 到这里还有 ~3.7s
   * 空窗 (3s 收敛 + 0.5s enHighLevel + 其它同步延迟), Kalman 这段时间在吃
   * 光流+acc 噪声积分, 位置会漂到 (2, -3.76, 0) m 级别. 接着 takeoff2()
   * (crtp_commander_high_level.c:436) 会把这个漂移值作为起飞原点锁住,
   * 整场飞行追幽灵坐标 -> 姿态指令饱和 ±8° -> 翻机.
   *
   * 修复: takeoff 前最后一刻再 reset 一次. kalmanCoreInit 会把状态直接
   * 设为 (initialX, initialY, initialZ) = (0,0,0) 并重置 P 矩阵, 同时清
   * flowDataQueue/tofDataQueue.
   *
   * [2026-05-05 再调] 100ms -> 300ms. 100ms 够让 Kalman 处理完 reset 指令,
   * 但不够它通过 gravity correction 吸收飞机 1~3° 的安装倾斜 (Kalman reset
   * 时姿态被初始化成 identity quaternion, 实际飞机如果有轻微倾斜, Kalman
   * 会把 sin(倾斜) * g 当成真实水平加速度, 积分出来就是空中持续漂移).
   * 300ms 让 acc update 跑 30 次, 足够姿态收敛到真实水平. 这 300ms 内
   * 位置还是会有一点漂移, 但下面 TellState 会把位置强制同步掉, 姿态收敛
   * 反而更重要 (位置漂移可以用定速 hover 规避, 姿态错了就是炸机).
   *
   * 单独调 crtpCommanderHighLevelTellState 是因为 HL commander 有自己内部
   * 的 pos 缓存 (crtp_commander_high_level.c:305), takeoff2 持 lockTraj
   * 读它时不保证 stabilizer 主循环已经跑过 GetSetpoint 的兜底同步分支.
   * 显式 TellState 把这个窗口堵死, 保证 takeoff 看到的起飞原点是 (0,0,0). */
  DEBUG_PRINTI("Final Kalman reset before takeoff...\n");
  paramSetInt(idReset, 1);
  vTaskDelay(M2T(300));

  state_t s_zero;
  stabilizerGetState(&s_zero);
  DEBUG_PRINTI("Pre-takeoff state: pos[%+5.2f %+5.2f %+5.2f] vel[%+5.2f %+5.2f %+5.2f]\n",
              (double)s_zero.position.x, (double)s_zero.position.y, (double)s_zero.position.z,
              (double)s_zero.velocity.x, (double)s_zero.velocity.y, (double)s_zero.velocity.z);
  crtpCommanderHighLevelTellState(&s_zero);

  const float takeoff_height = 0.15f;
  /* [2026-04-30 re-tune] takeoff 8s -> 5s.
   * 前一版用 8s 是担心 vbat 压降, 但实测前 1-2s VL53L1 在盲区 (<50mm 返回
   * 无效), Kalman 只能靠 acc 积分估高度, 时间越长积分漂移越多. 5s 是
   * takeoff 常见值, 电池压降风险小幅增加但 Kalman 高度更稳.
   *
   * [2026-05-05 再调] 0.3m -> 0.15m. 上次修好幽灵目标后飞机起飞没翻,
   * 但推重比不够 + 电池塌压 (vbat 从 4.0V -> 2.7V), 油门一直饱和也只爬到
   * 0.13~0.20m; VL53L1 在 <50mm 也是盲区. 0.15m 是 "刚好出盲区, 油门不
   * 饱和, 电池不会瞬间崩" 的最低可行点. 等硬件 (电池/电容) 升级后再加高. */
  DEBUG_PRINTI("Takeoff %.2fm in 5s...\n", (double)takeoff_height);
  crtpCommanderHighLevelTakeoff(takeoff_height, 5.0f);
  vTaskDelay(M2T(5500));  /* 爬升完 + 0.5s 裕量 */

  DEBUG_PRINTI("Hover 5s (velocity-hold mode, NOT position-hold)...\n");
  /* [2026-05-05] 悬停从 "HL position hold" 改成 "手动 velocity = 0 hold".
   *
   * 为什么: HL takeoff 结束后默认进入 position hold, 会用起飞原点 (0,0,z)
   * 作为目标. 但 pydrone 硬件存在系统性问题:
   *   - 姿态有 1~3° 安装倾斜 -> Kalman 认为有水平 acc -> 位置估计持续漂
   *   - 光流 squal 在低空经常 < 80 (PMW3901 需要纹理地面才准)
   *   - ToF 在 < 50mm 盲区, 起飞初期高度也不准
   * 结果: state 一直漂, 位置环拼命纠偏 -> 姿态指令 ±5° 抖 -> 越纠偏越飘.
   *
   * 改成 velocity hold: setpoint 直接给 vx=vy=vz=0, 控制器目标是 "速度为零"
   * 而不是 "位置不变". 飞机位置会被风/漂移慢慢带跑, 但不会因为位置估计
   * 漂移就拉大角度. 姿态稳 >> 位置稳 (栓绳试飞阶段飞机被栓着, 漂一点没
   * 关系, 但翻机就完蛋).
   *
   * commanderSetSetpoint 的副作用: 会调 crtpCommanderHighLevelStop() 把
   * HL planner 踢回 idle (commander.c:87), 之后 HL 不会再往 setpoint
   * 里塞东西, 完全由我们这里每 20ms 的 setpoint 主导.
   *
   * WDT 要求: commander 的 watchdog 超时 500ms 就进入稳定化模式, 2000ms
   * 切到 null setpoint (commander.h:35-36). 20ms 一次的刷新远远够. */
  state_t s;
  setpoint_t sp;
  for (int i = 0; i < 250; ++i) {  /* 250 * 20ms = 5s */
    memset(&sp, 0, sizeof(sp));
    sp.mode.x = modeVelocity;
    sp.mode.y = modeVelocity;
    sp.mode.z = modeVelocity;
    sp.mode.yaw = modeAbs;
    sp.velocity.x = 0.0f;
    sp.velocity.y = 0.0f;
    sp.velocity.z = 0.0f;
    sp.attitude.yaw = 0.0f;
    sp.velocity_body = false;  /* world frame */
    commanderSetSetpoint(&sp, COMMANDER_PRIORITY_EXTRX);

    /* 每 1s 打印一次状态 */
    if (i % 50 == 0) {
      stabilizerGetState(&s);
      DEBUG_PRINTI("hover t=%ds pos[x=%+5.2f y=%+5.2f z=%+5.2f] vel[%+5.2f %+5.2f %+5.2f] att[r=%+5.1f p=%+5.1f]\n",
                  i / 50,
                  (double)s.position.x, (double)s.position.y, (double)s.position.z,
                  (double)s.velocity.x, (double)s.velocity.y, (double)s.velocity.z,
                  (double)s.attitude.roll, (double)s.attitude.pitch);
    }
    vTaskDelay(M2T(20));
  }

  /* 降落前先把控制权还给 HL (land 是 HL 指令), 并把 HL 内部 pos 同步到
   * 当前真实位置 - 否则 land() 会用 hover 开始时的 stale pos 作为降落
   * 起点 (crtp_commander_high_level.c:473), 降落轨迹会跳变. */
  DEBUG_PRINTI("Land in 3s...\n");
  stabilizerGetState(&s);
  crtpCommanderHighLevelTellState(&s);
  vTaskDelay(M2T(50));
  crtpCommanderHighLevelLand(0.0f, 3.0f);
  vTaskDelay(M2T(3500));

  DEBUG_PRINTI("Done.\n");
  crtpCommanderHighLevelStop();
#endif
}
