# ESP-Drone (pydrone 硬件) 项目上下文

## 硬件配置

- 主控: ESP32 系列 (非 V1，非 S2_DRONE_V1_2，走 else 分支)
- IMU: MPU6050
- 电机: 有刷电机, PWM 驱动
- 构型: X 四轴

## 电机布局

```
      机头
M4(左前,CCW) ↘   ↙ M1(右前,CW)
              X
M3(左后,CW)  ↗   ↖ M2(右后,CCW)
      机尾
```

## 关键文件索引

| 功能 | 文件路径 |
|------|---------|
| 电机混控(power distribution) | `components/core/crazyflie/modules/src/power_distribution_stock.c` |
| 电机驱动(PWM) | `components/drivers/general/motors/motors.c` |
| 电机引脚/ID 定义 | `components/drivers/general/motors/include/motors.h` |
| PID 姿态控制器 | `components/core/crazyflie/modules/src/controller_pid.c` |
| PID attitude 内环/外环 | `components/core/crazyflie/modules/src/attitude_pid_controller.c` |
| PID 参数定义 | `components/core/crazyflie/modules/interface/pid.h` |
| IMU 读取 & 轴映射 | `components/core/crazyflie/hal/src/sensors_mpu6050_hm5883L_ms5611.c` |
| 姿态融合(Mahony/Madgwick) | `components/core/crazyflie/modules/src/sensfusion6.c` |
| 互补滤波估计器 | `components/core/crazyflie/modules/src/estimator_complementary.c` |
| 稳定器主循环 | `components/core/crazyflie/modules/src/stabilizer.c` |
| 遥控指令解析 | `components/core/crazyflie/modules/src/crtp_commander_rpyt.c` |
| 全局配置(QUAD_FORMATION_X等) | `components/config/include/config.h` |
| 电机 GPIO Kconfig | `main/Kconfig.projbuild` (行 307-349) |

## 信号链路 (IMU → 电机)

```
MPU6050 寄存器
  ↓ processAccGyroMeasurements()   [sensors_mpu6050_hm5883L_ms5611.c]
  ↓ (X/Y 互换, 取反适配硬件贴片方向)
sensorData.gyro / sensorData.acc
  ↓ estimatorComplementary()        [estimator_complementary.c]
  ↓ sensfusion6UpdateQ() → sensfusion6GetEulerRPY()  [sensfusion6.c]
state.attitude.roll / pitch / yaw
  ↓ controllerPid()                 [controller_pid.c]
  ↓ attitudeControllerCorrectAttitudePID() → 外环 PID
  ↓ attitudeControllerCorrectRatePID()     → 内环 Rate PID (注意: pitch 用 -gyro.y)
control.roll / pitch / yaw / thrust
  ↓ powerDistribution()             [power_distribution_stock.c]
  ↓ X 构型混控矩阵
motorPower.m1~m4
  ↓ motorsSetRatio()                [motors.c]
  ↓ LEDC PWM
物理电机 M1~M4
```

## IMU 轴映射现状 (pydrone 硬件, 非 V1 分支)

### sensors_mpu6050_hm5883L_ms5611.c 中的映射

```c
// 原始寄存器 → accelRaw/gyroRaw (X/Y 互换)
accelRaw.y = MPU_ACCEL_XOUT;   accelRaw.x = MPU_ACCEL_YOUT;
gyroRaw.y  = MPU_GYRO_XOUT;    gyroRaw.x  = MPU_GYRO_YOUT;

// 转物理量时的符号:
sensorData.gyro.x = -(gyroRaw.x - bias) * DEG_PER_LSB;   // 取反
sensorData.gyro.y =  (gyroRaw.y - bias) * DEG_PER_LSB;   // [AXIS-FIX] 不取反
accScaled.x = -(accelRaw.x) * G_PER_LSB / scale;          // 取反
accScaled.y = -(accelRaw.y) * G_PER_LSB / scale;          // [AXIS-FIX] 取反
```

### controller_pid.c 中的额外处理

```c
// Rate PID 输入: pitch 轴用 -gyro.y (Crazyflie 原版约定)
attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z, ...);

// Roll 和 Pitch 输出取反 (实测确认必需)
control->roll  = -control->roll;
control->pitch = -control->pitch;
```

## 已知问题 & 调试历史

### [2026-04-26] 起飞左侧翻 (详见 docs/analysis_left_flip_bug.md)

**根因**: Roll 和 Pitch 轴符号都与混控矩阵约定不一致 — sensor 层轴映射后的符号需要在 controller 层取反。04-25 版本只取反了 roll 未取反 pitch，pitch 正反馈发散导致侧翻。

**修复方案**: `controller_pid.c` 中同时取反 roll 和 pitch:
```c
control->roll  = -control->roll;   // 保留
control->pitch = -control->pitch;  // 新增
```

**状态**: roll+pitch 取反已应用, 待手持验证 + 栓绳试飞

**待验证项**:
1. gyro.x 方向一致性 (右滚时应为正值)
2. pitch 轴纠偏方向
3. GPIO 引脚与实物接线对应关系

### [2026-04-25] 地面调试工具 (已关闭)

以下宏已全部注释，进入飞行阶段：
- `MOTOR_OUTPUT_DISABLE` (power_distribution_stock.c) — 强制电机输出为 0
- `GROUND_TEST_MODE` (attitude_pid_controller.c) — 关闭所有 Ki
- `GROUND_TEST_ZERO_RPY` (crtp_commander_rpyt.c) — 清零遥控 RPY 指令

## 电机混控公式 (X 构型, QUAD_FORMATION_X)

```c
r = control->roll / 2;
p = control->pitch / 2;
M1(右前,CW)  = thrust - r + p + yaw;
M2(右后,CCW) = thrust - r - p - yaw;
M3(左后,CW)  = thrust + r - p + yaw;
M4(左前,CCW) = thrust + r + p - yaw;
```

- `+r` → 左侧(M3,M4)加油  → 纠正右翼下沉
- `+p` → 前侧(M1,M4)加油  → 纠正机头下压
- `+yaw` → CW 电机(M1,M3)加油 → 产生 CCW 力矩(机头左转)
