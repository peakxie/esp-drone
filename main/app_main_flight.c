/**
 * app_main_flight.c - 自动飞行序列
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
  DEBUG_PRINT("APP started!\n");

  /* 等待 10 秒: 陀螺仪校准 + 传感器稳定 */
  DEBUG_PRINT("Waiting 10s...\n");
  vTaskDelay(M2T(10000));

  /* 启用 high-level commander */
  paramVarId_t idHL = paramGetVarId("commander", "enHighLevel");
  paramSetInt(idHL, 1);
  vTaskDelay(M2T(500));

  /* 起飞到 0.5m, 5 秒上升 */
  DEBUG_PRINT("Takeoff 0.5m...\n");
  crtpCommanderHighLevelTakeoff(0.5f, 5.0f);
  vTaskDelay(M2T(1500));

  /* 悬停 2 秒 */
  // DEBUG_PRINT("Hover 2s...\n");
  // vTaskDelay(M2T(2000));

  /* 降落, 3 秒下降 */
  DEBUG_PRINT("Land...\n");
  crtpCommanderHighLevelLand(0.0f, 3.0f);
  vTaskDelay(M2T(2000));

  DEBUG_PRINT("Done.\n");
  crtpCommanderHighLevelStop();
}
