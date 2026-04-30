
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "range.h"
#include "sensfusion6.h"
#include "position_controller.h"
#include "controller_pid.h"

#include "log.h"
#include "debug_cf.h"
#include "param.h"
#include "math3d.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static bool tiltCompensationEnabled = false;

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

void controllerPidInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
}

bool controllerPidTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();
  DEBUG_PRINTD("controller_Pid_Test = %d", pass);

  return pass;
}

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}

void controllerPid(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {
       attitudeDesired.yaw += setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT;
    } else {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    }

    attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
  }

  if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
    positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable) {
      actuatorThrust = setpoint->thrust;
    }
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      attitudeDesired.roll = setpoint->attitude.roll;
      attitudeDesired.pitch = setpoint->attitude.pitch;
    }

    /* [2026-04-30] 低空水平位置环禁用 (PMW3901 光流失效区).
     * PMW3901 工作范围 ~80mm-3m, 起飞瞬间 z=0~8cm 光流读数是噪声,
     * Kalman 被误导让 pos.y 乱跳到 ±1m, position controller 命令大倾斜
     * 导致飞机真的被带飞走. 解决: 低空时 attitudeDesired.roll/pitch
     * 强制为 0, 飞机只做姿态自稳不追位置.
     *
     * 判断来源用 rangeGet(rangeDown) 而不是 state.position.z:
     * state.z 在起飞早期 VL53L1 还没报数 (<50mm 不推 Kalman) 时只靠 acc
     * 积分, 不可靠. rangeDown 直接是 VL53L1 最新有效读数 (>50mm 才更新),
     * 如果 <50mm 期间 rangeDown 保持 0, 那我们就认为飞机还贴地, 锁 rp=0;
     * 一旦 VL53L1 报出 >=50mm 的读数, rangeDown 更新到真实高度,
     * 再比较 >= 0.15m 时解锁位置环.
     *
     * 阈值 0.15m: PMW3901 在 80mm 开始有效, 加 70mm 余量避开近场干扰. */
    #define POS_CTRL_MIN_HEIGHT  0.15f
    float rangeDown_m = rangeGet(rangeDown);
    if (rangeDown_m < POS_CTRL_MIN_HEIGHT) {
      attitudeDesired.roll = 0.0f;
      attitudeDesired.pitch = 0.0f;
    }

    /* [2026-04-26] Roll 轴符号修正:
     *   sensor 层 acc.y 取反后, state.attitude.roll 的符号约定与混控矩阵相反.
     *   在 PID 输入端反转 state.roll, 而不是在输出端反转 control->roll,
     *   否则遥控指令方向也会被一起反转.
     *   反转 state.roll 后: PID error = desired - (-actual) = desired + actual,
     *   自稳和遥控方向都正确. */
    attitudeControllerCorrectAttitudePID(-state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity) {
      rateDesired.roll = setpoint->attitudeRate.roll;
      attitudeControllerResetRollAttitudePID();
    }
    if (setpoint->mode.pitch == modeVelocity) {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudePID();
    }

    // TODO: Investigate possibility to subtract gyro drift.
    /* -gyro.y 是数学正确的: Mahony 中 d(pitch)/dt ∝ -gy (因为 gravX = 2*(qx*qz - qw*qy)),
     * rate PID 需要与姿态变化率符号一致, 所以传 -gyro.y.
     * 注意: 这与 sensor 层的轴映射无关, 是四元数微分方程的固有性质.
     * [2026-04-26] 实测确认: pitch 全程稳定不发散 ✓
     * gyro.x 取反: 与 attitude PID 中 -state.roll 配套, 保持 rate 反馈方向一致. */
    attitudeControllerCorrectRatePID(-sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

    /* roll 取反已移到 PID 输入端 (-state.roll, -gyro.x), 输出端不再需要 */

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
    accelz = sensors->acc.z;

    /* === 调试: 打印姿态解算 + setpoint + desired ===
     * 打开方式: config.h 中取消注释 #define DEBUG_CTRL_DBG */
#ifdef DEBUG_CTRL_DBG
    static uint32_t s_ctrlDbgCnt = 0;
    if (++s_ctrlDbgCnt >= DEBUG_PRINT_INTERVAL) {
      s_ctrlDbgCnt = 0;
      DEBUG_PRINT_LOCAL("CTRL_DBG st[r=%+6.1f p=%+6.1f y=%+6.1f] sp[r=%+6.1f p=%+6.1f y=%+6.1f(mode=%d) rateY=%+6.1f] desY=%+6.1f thr=%5d\n",
        (double)state->attitude.roll, (double)state->attitude.pitch, (double)state->attitude.yaw,
        (double)setpoint->attitude.roll, (double)setpoint->attitude.pitch, (double)setpoint->attitude.yaw,
        (int)setpoint->mode.yaw, (double)setpoint->attitudeRate.yaw,
        (double)attitudeDesired.yaw, (int)setpoint->thrust);
    }
#endif
  }

  if (tiltCompensationEnabled)
  {
    control->thrust = actuatorThrust / sensfusion6GetInvThrustCompensationForTilt();
  }
  else
  {
    control->thrust = actuatorThrust;
  }

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    attitudeControllerResetAllPID();
    positionControllerResetAllPID();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}


LOG_GROUP_START(controller)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
LOG_ADD(LOG_FLOAT, accelz, &accelz)
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)

PARAM_GROUP_START(controller)
PARAM_ADD(PARAM_UINT8, tiltComp, &tiltCompensationEnabled)
PARAM_GROUP_STOP(controller)
