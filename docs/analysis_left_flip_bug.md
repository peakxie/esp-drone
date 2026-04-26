# 起飞左侧翻问题分析

## 问题现象

加大油门起飞时，飞机迅速朝左侧侧翻（左翼下沉）。

## 电机布局 (X 构型)

```
      机头
M4(左前,CCW) ↘   ↙ M1(右前,CW)
              X
M3(左后,CW)  ↗   ↖ M2(右后,CCW)
      机尾
```

## 根因：Roll 轴双重取反导致纠偏方向反转

### 完整信号链追踪

#### 第1步：IMU 原始数据 → 物理量转换

**文件**: `components/core/crazyflie/hal/src/sensors_mpu6050_hm5883L_ms5611.c`

```c
// 非 V1 分支（pydrone 走这里），X/Y 轴互换：
accelRaw.y = buffer[0:1];  // MPU 寄存器 ACCEL_XOUT
accelRaw.x = buffer[2:3];  // MPU 寄存器 ACCEL_YOUT
gyroRaw.y  = buffer[8:9];  // MPU 寄存器 GYRO_XOUT
gyroRaw.x  = buffer[10:11];// MPU 寄存器 GYRO_YOUT

// 行 380: gyro.x 取反（非 V1 硬件）
sensorData.gyro.x = -(gyroRaw.x - gyroBias.x) * DEG_PER_LSB;
// 行 387: [AXIS-FIX] gyro.y 去掉了取反
sensorData.gyro.y =  (gyroRaw.y - gyroBias.y) * DEG_PER_LSB;

// 行 395: acc.x 取反（非 V1 硬件）
accScaled.x = -(accelRaw.x) * G_PER_LSB / accScale;
// 行 400: [AXIS-FIX] acc.y 增加了取反 ← 修正 #1
accScaled.y = -(accelRaw.y) * G_PER_LSB / accScale;
```

#### 第2步：姿态解算 (Mahony/Madgwick)

**文件**: `components/core/crazyflie/modules/src/sensfusion6.c` 行 263-274

```c
void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw)
{
  *pitch = asinf(gravX) * 180 / M_PI_F;
  *roll  = atan2f(gravY, gravZ) * 180 / M_PI_F;  // ← roll 由 gravY 决定
}
```

**`[AXIS-FIX]` acc.y 取反后的效果**：
- 右翼下低 → `accScaled.y > 0` → `gravY > 0` → `roll > 0`（正值）
- 左翼下低 → `accScaled.y < 0` → `gravY < 0` → `roll < 0`（负值）

这符合标准航空惯例（右翼下 = 正 roll），**acc.y 取反本身是正确的**。

#### 第3步：PID 控制器

**文件**: `components/core/crazyflie/modules/src/controller_pid.c` 行 115-125

```c
attitudeControllerGetActuatorOutput(&control->roll, &control->pitch, &control->yaw);

// 行 125: [2026-04-25] 增加了 roll 取反 ← 修正 #2
control->roll = -control->roll;
```

PID 内部逻辑：`output ≈ Kp × (desired - actual)`

#### 第4步：电机混控 (X 构型)

**文件**: `components/core/crazyflie/modules/src/power_distribution_stock.c` 行 106-111

```c
int16_t r = control->roll / 2.0f;
motorPower.m1 = thrust - r + p + yaw;  // 右前 (CW)
motorPower.m2 = thrust - r - p - yaw;  // 右后 (CCW)
motorPower.m3 = thrust + r - p + yaw;  // 左后 (CW)
motorPower.m4 = thrust + r + p - yaw;  // 左前 (CCW)
```

混控矩阵含义：
- **`+r` → M3、M4（左侧）加油，M1、M2（右侧）减油** → 左侧升力增大
- **`-r` → M1、M2（右侧）加油，M3、M4（左侧）减油** → 右侧升力增大

### 信号链全链推演：起飞时左翼下沉

| 步骤 | 环节 | 值 | 说明 |
|------|------|-----|------|
| 1 | 左翼下沉 | 物理现象 | 起飞扰动 |
| 2 | `state.attitude.roll` | **< 0**（负值） | acc.y 取反后，左翼下 = 负 roll ✓ |
| 3 | PID attitude error | `0 - (负) = 正` | 期望回正 |
| 4 | `rateDesired` | **> 0**（正值） | 期望向右滚转 |
| 5 | Rate PID output | **> 0**（正值） | 陀螺仪≈0，全量输出 |
| 6 | `control->roll`（取反前） | **> 0**（正值） | PID 正确输出 |
| 7 | **`control->roll = -control->roll`** | **< 0**（负值） | ⚠️ 取反后变负！ |
| 8 | `r = control->roll / 2` | **< 0**（负值） | |
| 9 | M1、M2（右侧） | `thrust - r` = 加油 | 右侧升力增大 |
| 10 | M3、M4（左侧） | `thrust + r` = 减油 | 左侧升力减小 |
| 11 | **结果** | **左侧升力更小** | ⚠️ 左翼继续下沉 → 侧翻！ |

### 根本原因

在 2026-04-25 的调试中，同时做了两个修正：

1. **修正 #1**（sensors 层）：`accScaled.y = -(accelRaw.y) * ...`
   - 修正了 roll 角度的符号约定，使 `+roll = 右翼下低`
   - **这是正确的修正**

2. **修正 #2**（controller 层）：`control->roll = -control->roll`
   - 基于手持测试观察到 roll 方向不对而添加
   - **但手持测试可能是在 acc.y 取反之前做的，或两个修正同时添加未验证合体效果**

**两个取反叠加后互相抵消**，等效于没有任何修正，roll 纠偏方向仍然是反的：
- 修正 #1 把 roll 角度符号修正了 → PID 输出方向变正确
- 修正 #2 又把 PID 输出反过来 → 最终送到电机的方向又变错了

## 修复方案

### 方案 A（推荐）：删除 controller_pid.c 中的 roll 取反

**文件**: `components/core/crazyflie/modules/src/controller_pid.c`

删除第 125 行：
```diff
    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

-   control->roll = -control->roll;
-   // control->yaw  = -control->yaw;
```

**理由**：保留 sensor 层的轴向修正（更根本），移除 controller 层的补偿（不再需要）。
这样信号链变为：

| 步骤 | 值 | 结果 |
|------|-----|------|
| 左翼下沉 → roll < 0 | PID output > 0 | → `r > 0` |
| M3、M4（左侧） | `thrust + r` = **加油** | ✓ 左侧升力增大 |
| M1、M2（右侧） | `thrust - r` = **减油** | ✓ 右侧升力减小 |
| **结果** | 左翼被抬起 | ✓ **纠偏正确** |

### 方案 B（备选）：回滚 sensor 层的 acc.y 取反

**文件**: `components/core/crazyflie/hal/src/sensors_mpu6050_hm5883L_ms5611.c`

```diff
-   accScaled.y = -(accelRaw.y) * SENSORS_G_PER_LSB_CFG / accScale;
+   accScaled.y =  (accelRaw.y) * SENSORS_G_PER_LSB_CFG / accScale;
```

**不推荐**：虽然也能让混控方向正确，但 roll 角度符号会与航空惯例相反，不利于后续调试。

## 修复后仍需验证的事项

### 1. Gyro X 轴一致性

当前 `sensorData.gyro.x = -(gyroRaw.x - gyroBias.x) * DEG_PER_LSB`（取反）。

需验证：手持飞机向右滚转时，串口 `IMU_DBG` 打印的 `gyro.x` 是否为正值。

- 如果 `gyro.x > 0`（右翼下沉时）→ gyro 与 acc 一致 ✓
- 如果 `gyro.x < 0`（右翼下沉时）→ gyro 取反是错的，需同时移除取反：
  ```diff
  - sensorData.gyro.x = -(gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
  + sensorData.gyro.x =  (gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
  ```
  如果 gyro.x 方向不对，会导致 Mahony 滤波器在 roll 轴上陀螺积分与加速度计修正互相打架，造成姿态估计延迟或振荡。

### 2. Pitch 轴方向 (-gyro.y in rate PID)

**文件**: `controller_pid.c` 行 112

```c
attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z, ...)
```

`[AXIS-FIX]` 去掉了 sensor 层的 `gyro.y` 取反。由于 rate PID 调用处仍有 `-sensors->gyro.y`，需验证：
机头下压时，pitch 纠偏方向是否正确（M1+M4 前侧加油抬机头）。

### 3. GPIO 引脚实物验证

确认 `sdkconfig` 中 `CONFIG_MOTOR01_PIN` ~ `CONFIG_MOTOR04_PIN` 对应的 GPIO 号，
与 pydrone 硬件上 M1~M4 的物理接线一致：

| 软件电机 | 位置 | 旋向 | GPIO（需确认） |
|---------|------|------|---------------|
| M1 | 右前 | CW | CONFIG_MOTOR01_PIN |
| M2 | 右后 | CCW | CONFIG_MOTOR02_PIN |
| M3 | 左后 | CW | CONFIG_MOTOR03_PIN |
| M4 | 左前 | CCW | CONFIG_MOTOR04_PIN |

### 4. 低油门逐步测试

修复后建议：
1. **栓绳/固定测试**：低油门（不离地），观察手压某侧时对应侧电机是否加速
2. **手持倾斜测试**：装桨但不飞，观察 `PWR_DBG` 打印
   - 右翼下压 → M1+M2 输出应增大
   - 左翼下压 → M3+M4 输出应增大
   - 机头下压 → M1+M4 输出应增大
3. **栓绳悬停**：油门缓慢加到离地，确认不侧翻后再解绳

---

*分析日期: 2026-04-26*
*关键文件:*
- `components/core/crazyflie/modules/src/controller_pid.c` (第 125 行)
- `components/core/crazyflie/hal/src/sensors_mpu6050_hm5883L_ms5611.c` (第 395-400 行)
- `components/core/crazyflie/modules/src/power_distribution_stock.c` (第 105-111 行)
- `components/core/crazyflie/modules/src/sensfusion6.c` (第 263-274 行)
