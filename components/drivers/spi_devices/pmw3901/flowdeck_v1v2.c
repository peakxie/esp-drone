/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2017, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* flowdeck.c: Flow deck driver */
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pmw3901.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "range.h"
#include "sleepus.h"
#include "config.h"
#include "stabilizer_types.h"
#include "estimator.h"
#include "cf_math.h"
#define DEBUG_MODULE "FLOW"
#include "debug_cf.h"

#define AVERAGE_HISTORY_LENGTH 4
#define OULIER_LIMIT 100
#define LP_CONSTANT 0.8f
//#define USE_LP_FILTER
//#define USE_MA_SMOOTHING

#if defined(USE_MA_SMOOTHING)
static struct {
    float32_t averageX[AVERAGE_HISTORY_LENGTH];
    float32_t averageY[AVERAGE_HISTORY_LENGTH];
    size_t ptr;
} pixelAverages;
#endif

float dpixelx_previous = 0;
float dpixely_previous = 0;

static uint8_t outlierCount = 0;
static float stdFlow = 2.0f;

static bool isInit1 = false;
static bool isInit2 = false;

motionBurst_t currentMotion;

// Disables pushing the flow measurement in the EKF
static bool useFlowDisabled = false;

// Turn on adaptive standard deviation for the kalman filter
static bool useAdaptiveStd = true;

// Set standard deviation flow 
// (will not work if useAdaptiveStd is on)
static float flowStdFixed = 2.0f;

#define NCS_PIN CONFIG_SPI_PIN_CS0


static void flowdeckTask(void *param)
{
    systemWaitStart();
    initUsecTimer();
    uint64_t lastTime  = usecTimestamp();

    while (1) {
// if task watchdog triggered,flow frequency should set lower
#if CONFIG_FREERTOS_UNICORE
        vTaskDelay(10);
#else
        vTaskDelay(5);
#endif

        pmw3901ReadMotion(NCS_PIN, &currentMotion);

#ifdef DEBUG_SENSOR_EXT
        {
          static uint32_t s_flowDbgCnt = 0;
          static int32_t s_sumDx = 0, s_sumDy = 0;
          s_sumDx += currentMotion.deltaX;
          s_sumDy += currentMotion.deltaY;
          if (++s_flowDbgCnt >= 100) {  /* 累积 ~1 秒 */
            s_flowDbgCnt = 0;
            DEBUG_PRINT_LOCAL("FLOW sumDx=%+6d sumDy=%+6d squal=%3u\n",
              (int)s_sumDx, (int)s_sumDy, (unsigned)currentMotion.squal);
            s_sumDx = 0;
            s_sumDy = 0;
          }
        }
#endif

        // Flip motion information to comply with sensor mounting
        // [2026-04-26] pydrone 实测:
        //   飞机往前推 → deltaY=+正, 飞机往右推 → deltaX=-负
        //   Crazyflie 坐标系: 前=+X, 右=+Y
        //   光流看地面反向: 前推→地面后移→accpx 应为负(给估计器), 估计器内部再反转
        //   原版 accpx=-deltaY 在 Crazyflie 上正确, 但 pydrone 传感器安装旋转了 90°
        //   实测修正: 去掉负号
        int16_t accpx = currentMotion.deltaY;
        int16_t accpy = currentMotion.deltaX;

        // Outlier removal
        if (abs(accpx) < OULIER_LIMIT && abs(accpy) < OULIER_LIMIT) {
        if (useAdaptiveStd)
        {
        // The standard deviation is fitted by measurements flying over low and high texture 
        //   and looking at the shutter time
        float shutter_f = (float)currentMotion.shutter;
        stdFlow=0.0007984f *shutter_f + 0.4335f;


        // The formula with the amount of features instead
        /*float squal_f = (float)currentMotion.squal;
        stdFlow =  -0.01257f * squal_f + 4.406f; */
        if (stdFlow < 0.1f) stdFlow=0.1f;
        } else {
        stdFlow = flowStdFixed;
        }
            // Form flow measurement struct and push into the EKF
            flowMeasurement_t flowData;
            flowData.stdDevX = stdFlow;    // [pixels] should perhaps be made larger?
            flowData.stdDevY = stdFlow;    // [pixels] should perhaps be made larger?

// if task watchdog triggered,flow frequency should set lower
#if CONFIG_FREERTOS_UNICORE
            flowData.dt = 0.01;
#else
            flowData.dt = 0.005;
#endif

#if defined(USE_MA_SMOOTHING)
            // Use MA Smoothing
            pixelAverages.averageX[pixelAverages.ptr] = (float32_t)accpx;
            pixelAverages.averageY[pixelAverages.ptr] = (float32_t)accpy;

            float32_t meanX;
            float32_t meanY;

            xtensa_mean_f32(pixelAverages.averageX, AVERAGE_HISTORY_LENGTH, &meanX);
            xtensa_mean_f32(pixelAverages.averageY, AVERAGE_HISTORY_LENGTH, &meanY);

            pixelAverages.ptr = (pixelAverages.ptr + 1) % AVERAGE_HISTORY_LENGTH;

            flowData.dpixelx = (float)meanX;   // [pixels]
            flowData.dpixely = (float)meanY;   // [pixels]
#elif defined(USE_LP_FILTER)
            // Use LP filter measurements
            flowData.dpixelx = LP_CONSTANT * dpixelx_previous + (1.0f - LP_CONSTANT) * (float)accpx;
            flowData.dpixely = LP_CONSTANT * dpixely_previous + (1.0f - LP_CONSTANT) * (float)accpy;
            dpixelx_previous = flowData.dpixelx;
            dpixely_previous = flowData.dpixely;
#else
            // Use raw measurements
            flowData.dpixelx = (float)accpx;
            flowData.dpixely = (float)accpy;
#endif

            // Push measurements into the estimator
            /* [2026-04-30] 低空禁用光流 enqueue.
             * PMW3901 有效工作范围 ~80mm 起, 飞机在 <8cm 时光流看不清地面
             * (视距太近 + 桨叶气流扰动 + 视野被自身遮挡). 低于 8cm 时
             * Kalman 拿到的光流是纯噪声 -> pos.x/y 乱跳 -> 位置环命令飞机
             * 大倾斜 -> 起飞瞬间就被带飞走.
             * 用 rangeDown 而不是 Kalman 的 state.z, 因为这个判断要在
             * Kalman 收敛前也能正确工作 (state.z 需要光流+VL53L1 融合,
             * 起飞瞬间不可靠). VL53L1 直读高度足够准. */
            float rangeDown_m = rangeGet(rangeDown);
            const float FLOW_MIN_HEIGHT = 0.08f;  // PMW3901 最小有效高度
            bool flowTooLow = (rangeDown_m > 0.001f && rangeDown_m < FLOW_MIN_HEIGHT);

            if (!useFlowDisabled && !flowTooLow && currentMotion.motion == 0xB0) {
                flowData.dt = (float)(usecTimestamp()-lastTime)/1000000.0f;
                lastTime = usecTimestamp();
                estimatorEnqueueFlow(&flowData);
            }
        } else {
            outlierCount++;
        }
    }
}

// static void flowdeck1Init()
// {
//   if (isInit1 || isInit2) {
//     return;
//   }

//   // Initialize the VL53L0 sensor using the zRanger deck driver
//   const DeckDriver *zRanger = deckFindDriverByName("bcZRanger");
//   zRanger->init(NULL);

//   if (pmw3901Init(NCS_PIN))
//   {
//     xTaskCreate(flowdeckTask, FLOW_TASK_NAME, FLOW_TASK_STACKSIZE, NULL,
//                 FLOW_TASK_PRI, NULL);

//     isInit1 = true;
//   }
// }

// static bool flowdeck1Test()
// {
//   if (!isInit1) {
//     DEBUG_PRINTD("Error while initializing the PMW3901 sensor\n");
//   }

//   // Test the VL53L0 driver
//   const DeckDriver *zRanger = deckFindDriverByName("bcZRanger");

//   return zRanger->test();
// }

void flowdeck2Init()
{
	if (isInit1 || isInit2) {
        return;
    }

    // Initialize the VL53L1 sensor using the zRanger deck driver
    // const DeckDriver *zRanger = deckFindDriverByName("bcZRanger2");
    // zRanger->init(NULL);

    if (pmw3901Init(NCS_PIN)) {
        xTaskCreate(flowdeckTask, FLOW_TASK_NAME, FLOW_TASK_STACKSIZE, NULL, FLOW_TASK_PRI, NULL);

        isInit2 = true;
    }
}

bool flowdeck2Test()
{
    if (!isInit2) {
        DEBUG_PRINTD("Error while initializing the PMW3901 sensor\n");
    }

    // Test the VL53L1 driver
    //const DeckDriver *zRanger = deckFindDriverByName("bcZRanger2");

    return isInit2;//zRanger->test();
}

LOG_GROUP_START(motion)
LOG_ADD(LOG_UINT8, motion, &currentMotion.motion)
LOG_ADD(LOG_INT16, deltaX, &currentMotion.deltaX)
LOG_ADD(LOG_INT16, deltaY, &currentMotion.deltaY)
LOG_ADD(LOG_UINT16, shutter, &currentMotion.shutter)
LOG_ADD(LOG_UINT8, maxRaw, &currentMotion.maxRawData)
LOG_ADD(LOG_UINT8, minRaw, &currentMotion.minRawData)
LOG_ADD(LOG_UINT8, Rawsum, &currentMotion.rawDataSum)
LOG_ADD(LOG_UINT8, outlierCount, &outlierCount)
LOG_ADD(LOG_UINT8, squal, &currentMotion.squal)
LOG_ADD(LOG_FLOAT, std, &stdFlow)
LOG_GROUP_STOP(motion)

PARAM_GROUP_START(motion)
PARAM_ADD(PARAM_UINT8, disable, &useFlowDisabled)
PARAM_ADD(PARAM_UINT8, adaptive, &useAdaptiveStd)
PARAM_ADD(PARAM_FLOAT, flowStdFixed, &flowStdFixed)
PARAM_GROUP_STOP(motion)

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcFlow, &isInit1)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcFlow2, &isInit2)
PARAM_GROUP_STOP(deck)
