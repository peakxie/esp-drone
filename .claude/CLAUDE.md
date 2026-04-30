# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# ESP-Drone (pydrone 硬件) 项目上下文

这是一个基于 **ESP-IDF** 的四旋翼无人机固件, 控制代码移植自 **Crazyflie 2021.01** (GPL-3.0)。
当前仓库针对 **pydrone 自制硬件** (ESP32 + MPU6050 + 有刷电机) 做了适配与调试。

## 构建 & 烧录 (ESP-IDF)

必须使用 **ESP-IDF release/v5.0** (CI 亦测试 v4.4)。默认 target 是 `esp32`；也支持 `esp32s2` / `esp32s3` (有对应 `sdkconfig.defaults.*`)。

```bash
# 首次配置 / 切换芯片 (会重写 sdkconfig)
idf.py set-target esp32          # 本项目 pydrone 用这个
# idf.py set-target esp32s2
# idf.py set-target esp32s3

idf.py menuconfig                # 进 ESP-Drone Config 改硬件/引脚/target 类型
idf.py build                     # 编译
idf.py -p /dev/ttyUSB0 flash     # 烧录
idf.py -p /dev/ttyUSB0 monitor   # 串口监视 (Ctrl+]) 退出
idf.py -p /dev/ttyUSB0 flash monitor
idf.py fullclean                 # 切 target 前先跑这个
```

本项目没有 host 单元测试套件, 验证靠: 编译通过 + `monitor` 日志 + 地面 PID 响应方向测试 (见 `config.h` 中的 `GROUND_TEST_*` 宏) + 栓绳试飞。

## 高层架构

`main/main.c` 里的 `app_main()` 只做两件事: `platformInit()` → `systemLaunch()`。
之后一切都跑在 FreeRTOS 任务里, 由 `components/core/crazyflie/hal/src/system.c` 启动。
`main/app_main_flight.c` 里的 `appMain()` 是可选的自动飞行脚本 (当 `APP_ENABLED` 定义时被 `system` 任务调用)。

### 组件布局

- `components/core/crazyflie/` — **移植自 Crazyflie 2021.01**, 是所有飞控逻辑的所在地
  - `modules/` — 控制器 (PID/Mellinger/INDI)、估计器 (complementary/Kalman)、stabilizer 主循环、commander、power distribution
  - `hal/` — 传感器 (MPU6050/HMC5883L/MS5611)、平台抽象、system 启动
  - `utils/` — CRTP 协议、log/param 系统、数学/滤波
- `components/drivers/` — ESP32 专属驱动: `general/motors/` (LEDC PWM), `i2c_bus/`, `i2c_devices/` (VL53L1X ToF), `spi_devices/` (PMW3901 光流)
- `components/config/include/config.h` — **所有手动调参/调试开关集中在这里** (见下)
- `components/platform/` — ESP32 平台初始化
- `components/lib/dsp_lib/` — STM32 DSP 库的移植 (Mellinger/Kalman 依赖)
- `main/Kconfig.projbuild` — 电机 GPIO / 硬件选型的 Kconfig (行 307-349)

### 任务模型 (FreeRTOS)

稳定化走的是一条经典的 sensor→estimator→controller→mixer 流水线, 由 **stabilizer 任务 (优先级 7, 最高)** 驱动, **1kHz** 主循环 (`RATE_MAIN_LOOP = RATE_1000_HZ`, 见 `stabilizer_types.h`):

- `SENSORS_TASK` (prio 6) 把 MPU6050 数据 push 到队列
- `STABILIZER_TASK` (prio 7) 被 `sensorsWaitDataReady()` 解锁后:
  `stateEstimator()` → `commanderGetSetpoint()` → `controller()` → `powerDistribution()`
- 通讯层 (UDP/CRTP/WiFi) 跑在 prio 2, 不抢占飞控
- Kalman 在单核 SoC 上降到 prio 1 防饿死飞控

任务优先级/栈大小全在 `config.h` 第 147-232 行。

### 参数/日志系统 (CRTP)

Crazyflie 的 `PARAM_GROUP`/`LOG_GROUP` 宏让 C 代码里的变量能被 cfclient/APP 通过 CRTP 协议远程读写, 不需要改 firmware。加新调参变量时用这个机制, 不要 hard-code。

### 目标硬件判别

`config.h` 用 Kconfig 选项区分三套硬件:
- `CONFIG_TARGET_ESPLANE_V1` (ESP32)
- `CONFIG_TARGET_ESPLANE_V2_S2` (ESP32-S2)
- `CONFIG_TARGET_ESP32_S2_DRONE_V1_2` (ESP32-S2/S3)
- **本仓库 pydrone 走 `CONFIG_TARGET_ESP32_S2_DRONE_V1_2` 分支** (sdkconfig: `CONFIG_TARGET_ESP32_S2_DRONE_V1_2=y` + `CONFIG_MOTOR_BRUSHED_715=y`)
- 修改传感器/电机代码时用 `#if defined(CONFIG_TARGET_ESP32_S2_DRONE_V1_2)` 条件分支,不要动 V1 / V2_S2 / else 其他分支的代码

## 硬件配置

- 主控: ESP32-S3 (走 `CONFIG_TARGET_ESP32_S2_DRONE_V1_2` 分支, sdkconfig `CONFIG_IDF_TARGET_ESP32S3=y`)
- 电机类型: `CONFIG_MOTOR_BRUSHED_715=y` (有刷 715 空心杯)
- IMU: MPU6050 (I2C)
- 高度/位置: VL53L1X (ToF) + PMW3901 (光流)
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
| 自动飞行脚本 | `main/app_main_flight.c` (`appMain()`) |

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

## IMU 轴映射现状 (pydrone 硬件, CONFIG_TARGET_ESP32_S2_DRONE_V1_2 分支)

### sensors_mpu6050_hm5883L_ms5611.c 中的映射

```c
// 原始寄存器 → accelRaw/gyroRaw (X/Y 互换)
accelRaw.y = MPU_ACCEL_XOUT;   accelRaw.x = MPU_ACCEL_YOUT;
gyroRaw.y  = MPU_GYRO_XOUT;    gyroRaw.x  = MPU_GYRO_YOUT;

// 转物理量时的符号:
sensorData.gyro.x =  (gyroRaw.x - bias) * DEG_PER_LSB;   // [2026-04-26] 去掉取反, 与 acc.y 一致
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

### [2026-04-26] 起飞侧翻 (详见 docs/analysis_left_flip_bug.md)

**根因**: 两个问题叠加:
1. `controller_pid.c` 缺少 roll 取反 → roll 纠偏方向反 → 导致左侧翻
2. `sensors` 层 `gyro.x` 多余取反 → 与 acc.y 方向不一致 → Mahony 滤波器 gyro/acc 打架 → 姿态估计发散

**修复 (已验证)**:
- `controller_pid.c`: `control->roll = -control->roll;` (保留)
- `sensors_mpu6050_hm5883L_ms5611.c`: gyro.x 去掉取反, 改为 `(gyroRaw.x - gyroBias.x) * DEG_PER_LSB` (与 V1 分支一致)
- pitch 实测不需取反, 全程稳定

**验证结果**:
- 右翼朝下: r≈-28000, M1/M2 加油 ✓
- 放回水平: r 收敛回 ≈-4000 ✓ (不再发散)
- pitch 全程 ≈-1900 稳定 ✓

**状态**: 方向验证通过, 栓绳试飞已起飞, 调优中

**已验证完毕**:
1. gyro.x 去掉取反 → Mahony roll 轴收敛 ✓
2. pitch 轴: `-gyro.y` 在 rate PID 中是数学正确的, 实测稳定不发散 ✓
3. roll 取反(`control->roll = -control->roll`) 实测确认必需 ✓
4. IMU trim: roll=-3.9°, pitch=+1.78° 已补偿 ✓

### [2026-04-26] 慢推油门斜飞

**根因**: `DEFAULT_IDLE_THRUST=0` → 低油门时 PID 修正量 > 油门量 → 部分电机 clip 到 0 → 不对称推力
**修复**: `config.h` 中 `#define DEFAULT_IDLE_THRUST 6000`

### [2026-04-30] WiFi/cfclient 调试基建

调试 auto takeoff 时发现飞机只能收到 ~20 个 log packet 就停推,cfclient 几秒后 Flight Control 数据停止更新。

**根因 1** (cfclient 卡住的原因): `wifilink.c:65` 的 `wifilinkIsConnected()` 沿用 Crazyflie NRF radio 假设——"1s 没收到 client 包 = 断开",但 UDP 是 full-duplex, client 订阅完 log 后纯被动收不再发包,约 2s 后触发 `log.c:863` 的 `logReset()+crtpReset()`, 永久停止推送。
**修复**: `wifilink.c` 的 `wifilinkIsConnected()` 改成永远 `return true`, UDP 没有"断开"概念, 断连由应用层 timeout 处理。

**根因 2** (起飞侧翻的新发现): `appMain()` 原先先切 Kalman 再 `vTaskDelay(10000)`,但切 Kalman 瞬间陀螺仪零偏还没校准完,Kalman 用带偏 gyro 数据积累 10 秒,起飞瞬间姿态已错,必翻。
**修复**: appMain 里先 `while (!sensorsAreCalibrated()) vTaskDelay(100);` 等 sensors 校准完成,再切 Kalman,再等 3s Kalman 收敛。

**副发现 - pm.vbat 读数错**: pydrone 硬件用 40k+10k 分压 (1/5),但 `pm_esplane.c:120` 的 multiplier 是 Crazyflie 原版的 2 (1/2 分压)。正确值应该是 5 (用万用表实测 ADC 脚 vs 电池端电压比确认)。不影响飞行, 只影响 vbat 读数。

### [2026-04-30] 调试工具链 (cfclient over WiFi)

| 工具 | 位置 | 用途 |
|------|------|------|
| `/data/project/source/peakxie/crazyflie-clients-python` | Linux 源码镜像 | cfclient leeebo fork, 有 udp driver |
| `D:\work\peakxie\crazyflie-clients-python` | Windows 实际运行 | 同上 Windows 端 editable install |
| `D:\work\flight_recorder.py` | Windows 诊断脚本 | 50Hz 订阅 14 变量, 存 CSV 到 `D:\work\flight_logs\` |
| `D:\work\drone_diag.py` | Windows 诊断脚本 | 监测 log packet 推送频率, 卡死时用 |

**WiFi 配置** (`idf.py menuconfig → ESPDrone Config → wireless config`):
- APSTA 混合模式已实现 (`WIFI_STA_ENABLE=y`), 飞机可同时当 AP 和连外部路由
- mDNS 已实现 (`MDNS_ENABLE=y`), 可用 `udp://esp-drone.local` 连接

**cfclient 连接 URI**: 
- AP 模式: `udp://192.168.43.42:2390` (cfclient 默认)
- STA 模式: `udp://<路由分配的IP>:2390` 或 `udp://esp-drone.local:2390`
- cfclient 源码 `main.py:foundInterfaces()` 已改,用 config.json 里的 `link_uri` 优先

### [2026-04-25] 地面调试工具 (集中在 config.h)

以下宏全部在 `config.h` 管理：
- `MOTOR_OUTPUT_DISABLE` — 强制电机输出为 0
- `GROUND_TEST_MODE` — 关闭所有 Ki
- `GROUND_TEST_ZERO_RPY` — 清零遥控 RPY 指令

**正式飞行前务必全部注释掉** (`config.h` 第 82-97 行的安全开关 + 打印开关)。

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
