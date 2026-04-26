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

  /* 第一步: 只等待, 确认不崩溃 */
  DEBUG_PRINT("Waiting 30s...\n");
  vTaskDelay(M2T(30000));
  DEBUG_PRINT("Wait done. Enabling HL commander...\n");

  /* 第二步: 启用 high-level commander */
  paramVarId_t idHL = paramGetVarId("commander", "enHighLevel");
  paramSetInt(idHL, 1);
  vTaskDelay(M2T(500));
  DEBUG_PRINT("enHighLevel = %d\n", paramGetInt(idHL));

  /* 第三步: 起飞 */
  DEBUG_PRINT("Takeoff...\n");
  int ret = crtpCommanderHighLevelTakeoff(0.5f, 2.0f);
  DEBUG_PRINT("Takeoff ret=%d\n", ret);
  vTaskDelay(M2T(3000));

  /* 第四步: 悬停 */
  DEBUG_PRINT("Hovering 3s...\n");
  vTaskDelay(M2T(3000));

  /* 第五步: 降落 */
  DEBUG_PRINT("Landing...\n");
  ret = crtpCommanderHighLevelLand(0.0f, 3.0f);
  DEBUG_PRINT("Land ret=%d\n", ret);
  vTaskDelay(M2T(4000));

  DEBUG_PRINT("Flight complete.\n");
  crtpCommanderHighLevelStop();
}
