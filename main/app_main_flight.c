/**
 * app_main_flight.c - 自动飞行序列
 *
 * 开机校准后等待 30s → 起飞到 20cm → 悬停 3s → 缓慢降落
 *
 * 前置条件:
 *   - config.h 中定义 APP_ENABLED
 *   - VL53L1X + PMW3901 传感器已连接且初始化 OK
 *   - Kalman 估计器由 PMW3901 自动注册
 */

#include "FreeRTOS.h"
#include "app.h"
#include "config.h"
#include "crtp_commander_high_level.h"
#include "log.h"
#include "param.h"
#include "stm32_legacy.h"
#include "task.h"

#define DEBUG_MODULE "APP_FLIGHT"
#include "debug_cf.h"

void appMain(void) {
  /* 等待 30 秒: 陀螺仪校准 + 传感器稳定 */
  DEBUG_PRINT("Waiting 30s for calibration...\n");
  vTaskDelay(M2T(30000));
  DEBUG_PRINT("Calibration wait done.\n");

  /* 启用 high-level commander */
  paramVarId_t idHL = paramGetVarId("commander", "enHighLevel");
  DEBUG_PRINT("enHighLevel paramId: %d.%d\n", idHL.id, idHL.index);
  paramSetInt(idHL, 1);
  vTaskDelay(M2T(500));

  /* 确认 enableHighLevel 已设为 1 */
  int hlVal = paramGetInt(idHL);
  DEBUG_PRINT("enHighLevel = %d\n", hlVal);

  /* 起飞到 0.2m, 用 2 秒缓慢上升 */
  DEBUG_PRINT("Takeoff to 0.2m...\n");
  int ret = crtpCommanderHighLevelTakeoff(0.2f, 2.0f);
  DEBUG_PRINT("Takeoff ret = %d\n", ret);
  vTaskDelay(M2T(3000)); /* 等起飞完成, 多留 1 秒余量 */

  /* 悬停 3 秒 */
  DEBUG_PRINT("Hovering 3s...\n");
  vTaskDelay(M2T(3000));

  /* 降落到 0m, 用 3 秒缓慢下降 */
  DEBUG_PRINT("Landing...\n");
  ret = crtpCommanderHighLevelLand(0.0f, 3.0f);
  DEBUG_PRINT("Land ret = %d\n", ret);
  vTaskDelay(M2T(4000)); /* 等降落完成, 多留 1 秒余量 */

  /* 停止 */
  DEBUG_PRINT("Flight complete.\n");
  crtpCommanderHighLevelStop();
}
