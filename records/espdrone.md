# flight

## 背景介绍

使用pydrone的硬件（自带的micropython固件https://github.com/01studio-lab/micropython.git，目录ports/esp32/py-drone），移植https://github.com/espressif/esp-drone.git固件
搜集硬件电路图资料比对差异，主要结合固件差异，给出移植方案。

claude --resume 68bebf86-e92d-49ea-a13c-426c9f279fc3
方案已写入 espdrone.md。核心结论和几个必须先知道的点：

  最重要的发现

  pyDrone 自带的 py-drone 固件本身就是 esp-drone 的一个分支。 mpu6050.c 只有 21 行差异、i2cdev 11 行、motors.c 67 行且多数是注释掉的
  ASSERT/LOG，GPL 头都还在。01studio 做的事是：保留驱动层，把 esp-drone 的 CRTP + Kalman + Crazyflie 模块层换成「匿名(ANO)协议 +
  互补滤波 + 简化 PID」，再包成 MicroPython 的 drone.DRONE。

  所以这不是从零适配，而是把上层控制栈搬回去。驱动层不用重写。

  硬件差异（已核对原理图 netlist）

  从 01studio-lab/pyDrone 仓库拿到了 hardware/pyDrone_Sch.pdf（v1.0, 2022-07-14），提取出完整网表。关键差异：

  - 主控 ESP32-S3 N8R8（Octal PSRAM，占 GPIO35/36/37）
  - 气压计 SPL06-001 ≠ MS5611
  - 磁力计 QMC5883L (0x0D) ≠ HMC5883L (0x1E) —— 地址、字节序、轴顺序、灵敏度全不同，必须整体换驱动
  - MPU6050 相同，且气压/磁力计同样挂在 MPU6050 的 AUX 总线上，拓扑一致，sensorsSetupSlaveRead() 框架可直接复用

  一条必须在首次烧写前改掉的：esp-drone 默认 deck I²C 是 GPIO40/41，而 pyDrone 这两脚直接驱动 M3/M4 电机 MOS
  管。不改就上电，i2cdrvInit(&deckBus) 会把电机脚配成 I²C 开漏并挂内部上拉，可能让两个电机在初始化阶段意外转动。

  两个需要你决定的地方

  定高会丢。 ESP-Drone 主线里气压计和磁力计默认是 #define 注释掉的 —— 它的定高/定点靠扩展板的 VL53L1X + PMW3901 光流。pyDrone
  没有扩展板，SPI 脚还被 Octal PSRAM 占了装不上。直接编译只能得到自稳模式。要恢复定高得把 py-drone 的 SPL06 通路移植进去（好消息：这活
  01studio 已经做完了，直接抄）。

  MicroPython 和 esp-drone 的 IDF 版本不重叠：esp-drone 要 v5.0，01studio 的 S3 端口要 v4.4/master。我推荐路线 A（独立 IDF 工程，放弃
  MicroPython），路线 B 的要点也写了，工作量约 2–3 倍，主要坑在 log.c/param.c 依赖 linker_fragment.lf 的自定义链接段，而 MicroPython 的
  idf_component_register 没有 LDFRAGMENTS。

  一处我没有下结论的地方

  两版 power_distribution_stock.c 里 roll 和 yaw 分配完全一致，只有 pitch 整体反号。我在文档里标为必须台架实测（T3）而不是照抄 ——
  因为无法从代码判断这是电机编号差异还是 IMU 贴装朝向差异，两者要改的文件不同。装反的后果是俯仰通道正反馈，一推杆就翻。同理 PID
  参数：716 电机 + 46mm 桨 + 400mAh 与 esp-drone 的 720 整机惯量不同，默认增益大概率要重调。

  验证顺序 T0–T7 已列，T1–T4 请务必拆桨执行；T0 是备份原厂固件，否则难回退。

---

# pyDrone 移植 ESP-Drone 固件方案

## 0. 结论摘要

**关键发现：pyDrone 自带的 `py-drone` 固件本身就是 ESP-Drone 的一个分支**（保留了 GPL 头、`i2cdev`/`mpu6050`/`motors`/`pm_esplane`/`adc_esp32` 几乎逐字相同），但它把 ESP-Drone 的 **CRTP + Kalman + Crazyflie 模块层整体换成了「匿名(ANO)上位机协议 + 互补滤波 + 简化 PID」的控制栈**，并作为 MicroPython 的一个 C 模块（`drone.DRONE`）运行。

所以「移植」不是从零适配，而是**把 ESP-Drone 上层控制/通信栈搬回 pyDrone 硬件**。有两条路线：

| 路线 | 做法 | 工作量 | 结果 |
|---|---|---|---|
| **A. 独立 IDF 工程**（推荐） | 直接编译 esp-drone，新增 `TARGET_PYDRONE_S3` 板型，改引脚 + 换气压计/磁力计驱动 | 中 | 完整 ESP-Drone（CRTP、cfclient、手机 APP），**失去 MicroPython** |
| **B. 保留 MicroPython** | 在现有 `py-drone` 目录内，用 esp-drone 的 `crtp/commander/estimator` 替换 `anop/commander` | 大 | 两者兼得，但需自行处理 IDF 版本与内存冲突 |

推荐 **路线 A**：ESP-Drone 是完整 IDF 工程，pyDrone 硬件与 ESP32-S2-Drone-V1.2 高度同构（同 MPU6050、同 4 路 LEDC 有刷电机、同 UDP:2390），改动集中在引脚表和两颗 I²C 从设备驱动上。下文以路线 A 为主线，第 5 节给出路线 B 要点。

---

## 1. 硬件对比（已核对原理图）

数据来源：
- pyDrone：`01studio-lab/pyDrone` 仓库 `hardware/pyDrone_Sch.pdf`（v1.0, 2022-07-14）+ `pyDrone_Resource_1.png` + `boards/PYDRONE/mpconfigboard.h`
- ESP-Drone：`hardware/ESP32_S2_Drone_V1_2/SCH_Mainboard_ESP32_S2_Drone_V1_2.pdf` + `main/Kconfig.projbuild`

### 1.1 总体

| 项目 | ESP-Drone (ESP32-S2-Drone V1.2) | pyDrone v1.0 | 影响 |
|---|---|---|---|
| 主控 | ESP32-S2-WROVER | **ESP32-S3-WROOM-1 N8R8** | S3 已被 esp-drone 支持（Kconfig 有 S3 分支）✅ |
| Flash / PSRAM | 4MB / 2MB Quad | **8MB / 8MB Octal** | Octal PSRAM 占用 GPIO35/36/37 ⚠️ |
| IMU | MPU-6050 (0x68) | **MPU6050（相同）** | 驱动可直接复用 ✅ |
| 气压计 | MS5611（原理图有，固件默认**未启用**） | **SPL06-001 (0x76)** | 需换驱动 ⚠️ |
| 磁力计 | HMC5883L (0x1E) | **QMC5883L (0x0D)** | 芯片不同、寄存器不同 ⚠️ |
| 气压/磁 挂载 | MPU6050 AUX 总线 | **MPU6050 AUX 总线（相同）** | 拓扑一致 ✅ |
| 电机 | 4× 有刷 720，LEDC | **4× 有刷 716，LEDC**（SI2302 N-MOS） | 驱动相同 ✅ |
| 蜂鸣器 | 有（GPIO38/39） | **无** | 需关闭 ⚠️ |
| USB | CP2102N USB-UART 桥 | **无桥，GPIO19/20 原生 USB** | 需用 USB-Serial-JTAG ⚠️ |
| 摄像头 | OV2640 FPC-24P | OV2640 FPC-24P | esp-drone 不用 ✅ |
| 扩展板 | Flow Deck（PMW3901+VL53L1X） | **无，仅 2×8P 2.0mm 排母** | 定点模式不可用 ⚠️ |
| LED | 蓝/红/绿 3 路可控 | **蓝(46)/绿(42) 仅 2 路可控**；红=电源硬连、黄=充电STAT | 需改 LED 表 ⚠️ |
| 电池 | 锂电 + ADC 分压 | 400mAh + **40.2k/10k 分压（÷5.02）** | 分压比不同 ⚠️ |

### 1.2 引脚对照表（**移植核心**）

| 功能 | esp-drone 默认 (S2/S3) | pyDrone 实际 | 是否冲突 |
|---|---|---|---|
| I2C0_SCL（传感器） | 10 | **15** | 改 |
| I2C0_SDA（传感器） | 11 | **16** | 改 |
| I2C1_SCL（deck） | 41 | **1**（P3-6） | ⚠️ **41 是 pyDrone 电机 M4！** |
| I2C1_SDA（deck） | 40 | **6**（P3-5） | ⚠️ **40 是 pyDrone 电机 M3！** |
| MPU_PIN_INT | 12 | **7** | 改 |
| MOTOR01 | 5 | **4** | 改 |
| MOTOR02 | 6 | **5** | 改 |
| MOTOR03 | 3 | **40** | 改 |
| MOTOR04 | 4 | **41** | 改 |
| LED_BLUE | 7 | **46** | 改 |
| LED_GREEN | 9 | **42** | 改 |
| LED_RED | 8 | **无可用 GPIO** | 复用/裁掉 |
| ADC1_PIN（电池） | 2 | **2**（同）| ✅ 但分压比不同 |
| BUZ1/BUZ2 | 39 / 38 | **无蜂鸣器**（38/39=摄像头D1/D2 + P2） | 关闭 |
| SPI MISO/MOSI/CLK/CS0 | 37/35/36/34 | **35/36/37 被 Octal PSRAM 占用，且未引出** | Flow Deck 不可用 |

> **最危险的一条**：esp-drone 默认 deck I²C 用 GPIO40/41，而 pyDrone 这两个脚直接驱动 M3/M4 电机 MOS 管。若不改就上电，`i2cdrvInit(&deckBus)` 会把两路电机脚配成 I²C 开漏并挂上内部上拉 —— **可能导致两个电机在初始化阶段意外转动**。必须在第一次烧写前改掉。

### 1.3 pyDrone 传感器总线拓扑（原理图 netlist 实证）

```
ESP32-S3 ──I2C0(SCL=15, SDA=16)──> MPU6050 (U3, 0x68)
                                     │  AUX_CL(pin7) / AUX_DA(pin6)
                                     └──AUX 总线──┬── SPL06-001  (U6, 0x76)
                                                  └── QMC5883L   (U7, 0x0D)
```
`AUX_SCL/AUX_SDA` 网络上有 R28/R29 4.7k 上拉。气压计与磁力计**不在主 I²C 上**，只能靠 MPU6050 的 I²C-Master（slave0~3 自动读）或 Bypass 模式访问 —— 与 ESP-Drone 的做法完全一致，这是可以直接复用 `sensorsSetupSlaveRead()` 框架的原因。

---

## 2. 固件架构差异

### 2.1 目录/模块对照

| 层次 | esp-drone | py-drone (01studio) | 复用性 |
|---|---|---|---|
| 入口 | `main/main.c` → `platformInit()` → `systemLaunch()` | MicroPython `drone.DRONE()` → `systemInit()` | ✗ |
| 通信 | `crtp.c` + `wifilink.c` + `crtp_commander_rpyt.c`（CRTP over UDP:2390） | `anop.c` + `mod_wifllink.c`（匿名协议 over UDP:2390） | ✗ 协议不同 |
| 指令 | `commander.c`（setpoint/`crtpCommanderRpytDecodeSetpoint`） | `commander.c`（`ctrlVal_t`/一键起飞降落） | ✗ 语义不同 |
| 姿态估计 | `estimator_kalman.c`（EKF）+ `estimator_complementary.c` | `state_estimator.c` + `sensfusion6.c`（互补滤波） | ✗ |
| 控制 | `controller_pid.c` / `attitude_pid_controller.c` / `position_controller_pid.c` | `state_control.c` / `attitude_pid.c` / `position_pid.c` | 部分 |
| 动力分配 | `power_distribution_stock.c` | 同名文件，**pitch 符号相反** | ⚠️ 见 2.3 |
| 参数/日志 | `param.c` / `log.c` / `mem.c`（cfclient 可调） | `config_param.c`（精简，NVS） | ✗ |
| 传感器 | `sensors_mpu6050_hm5883L_ms5611.c` | `sensors_mpu6050_spl06.c` | ⚠️ 见 2.2 |
| 驱动 | `i2cdev`/`i2c_drv`/`mpu6050`/`motors`/`led`/`pm_esplane`/`adc_esp32` | **同名同源，仅日志宏与 DeInit 差异** | ✅ 可直接用 |
| DSP | `components/lib/dsp_lib` | `py-drone/dsp_lib`（同源） | ✅ |

驱动层实测差异极小（`mpu6050.c` 仅 21 行不同、`i2cdev` 11 行、`motors.c` 67 行且多为注释掉 ASSERT/LOG）——**这印证了 py-drone 是 esp-drone 的下游分支**，硬件驱动无需重写。

### 2.2 传感器栈：这是最实质的工作量

ESP-Drone 主线里 **磁力计和气压计默认是关掉的**：

```c
// components/core/crazyflie/hal/src/sensors_mpu6050_hm5883L_ms5611.c:82-86
// #define SENSORS_ENABLE_MAG_HM5883L
// #define SENSORS_ENABLE_PRESSURE_MS5611
#define SENSORS_ENABLE_RANGE_VL53L1X
#define SENSORS_ENABLE_FLOW_PMW3901
```

ESP-Drone 的定高/定点靠 **扩展板的 VL53L1X 激光测距 + PMW3901 光流**，不靠气压计。而 pyDrone **没有扩展板**，且 SPI 引脚被 PSRAM 占用装不了 Flow Deck。

**推论：直接编译 esp-drone 到 pyDrone 上，只能得到「自稳模式」，没有定高。** 要恢复定高，必须把 py-drone 的 SPL06 气压计通路移植进 esp-drone —— 而 py-drone 已经把这件事做完了，直接抄：

- `py-drone/drivers/i2c_devices/spl06/`（248 行）→ 整体拷入 esp-drone 作为新组件
- `py-drone/drivers/i2c_devices/hmc5883l/`（QMC5883L 版，341 行）→ 覆盖 esp-drone 的 HMC5883L 驱动
- `sensors_mpu6050_spl06.c` 的 `sensorsSetupSlaveRead()` slave2/3 配置、`processBarometerMeasurements()`、`processMagnetometerMeasurements()` → 移植进 esp-drone 的 `sensors_mpu6050_hm5883L_ms5611.c`

QMC5883L 与 HMC5883L 的关键差异（已核对）：

| | HMC5883L | QMC5883L |
|---|---|---|
| I²C 地址 | 0x1E | **0x0D** |
| ID 校验 | 读 `RA_ID_A(0x0A)` 得 'H','4','3' | 读 `0x0D` 得 **0xFF** |
| 数据寄存器 | 0x03 起，**大端 X,Z,Y** | **0x00 起，小端 X,Y,Z** |
| 状态位 | `RA_STATUS` bit0 RDY | `0x06` bit0 DRDY |
| 初始化 | CONFIG_A/B + MODE | 写 `0x0A=0x00`，`0x0B=0x01`，`0x0A=0x40`，`0x09=模式` |
| 灵敏度 | 660 LSB/Gauss | **12000 LSB/Gauss** |

字节序和轴顺序都变了，这块必须整体替换而非改地址。

### 2.3 动力分配的 pitch 符号相反（必须确认）

```c
// esp-drone
m1 = thrust - r + p + yaw;    m2 = thrust - r - p - yaw;
m3 = thrust + r - p + yaw;    m4 = thrust + r + p - yaw;

// py-drone
m1 = thrust - r - p + yaw;    m2 = thrust - r + p - yaw;
m3 = thrust + r + p + yaw;    m4 = thrust + r - p - yaw;
```

roll 与 yaw 的分配完全一致，**只有 pitch 整体反号**。原因是 pyDrone 的电机编号/机头方向与 ESP-Drone 板不同：按 `pyDrone_Resource_1.png`，机头指向图示左方，M1 左上、M2 右上、M3 右下、M4 左下。

**这一项不要凭代码推断，必须上台架实测**（见第 4 节 T3）。装反的后果是俯仰通道正反馈 —— 一推杆就翻。

### 2.4 构建环境冲突（路线 B 的主要障碍）

- esp-drone：要求 **ESP-IDF v5.0**，独立 IDF 工程（`project(ESPDrone)`）
- 01studio micropython：ESP32-S3 需 IDF **v4.4/master**，`README` 明示支持 v4.0.2/v4.1.1/v4.2

两者 IDF 版本不重叠，且 esp-drone 大量使用 `esp_adc_cal`（v5.0 中已 deprecated 但仍可用）、`LOG_GROUP/PARAM_GROUP` 链接段（依赖 `linker_fragment.lf`）。路线 B 需要把 esp-drone 的 `log/param/mem` 链接段机制搬进 MicroPython 的 `idf_component_register`，这是主要坑点。**这也是推荐路线 A 的核心原因。**

---

## 3. 移植方案（路线 A：独立 IDF 工程）

### 步骤 1 — 环境

```bash
git clone -b release/v5.0 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32s3 && . ./export.sh
git clone https://github.com/espressif/esp-drone.git && cd esp-drone
idf.py set-target esp32s3
```

### 步骤 2 — 新增板型 `TARGET_PYDRONE_S3`

`main/Kconfig.projbuild`，在 choice 里加一项，并为所有引脚 config 补 pyDrone 默认值：

```kconfig
config TARGET_PYDRONE_S3
    bool "01Studio pyDrone with ESP32-S3 onboard"
```

引脚默认值（每个 config 都要加 `default X if TARGET_PYDRONE_S3`，range 放宽到 `0 48`）：

```
I2C0_PIN_SCL  = 15      I2C0_PIN_SDA  = 16
I2C1_PIN_SCL  = 1       I2C1_PIN_SDA  = 6      # 绝不能留 40/41
MPU_PIN_INT   = 7
MOTOR01_PIN   = 4       MOTOR02_PIN   = 5
MOTOR03_PIN   = 40      MOTOR04_PIN   = 41
LED_PIN_BLUE  = 46      LED_PIN_GREEN = 42     LED_PIN_RED = 46（复用蓝）
ADC1_PIN      = 2
BUZZER_ON     = n                              # pyDrone 无蜂鸣器
```

`components/config/include/config.h` 加板型/芯片一致性校验：

```c
#elif defined(CONFIG_TARGET_PYDRONE_S3)
#ifndef CONFIG_IDF_TARGET_ESP32S3
#error "pyDrone hardware with ESP32-S3 onboard"
#endif
```

`components/platform/platform_cf2.c` 增加设备条目（`deviceType`/`sensorImplementation`）。

### 步骤 3 — sdkconfig（S3 N8R8）

```
CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y          # N8R8 是 Octal，务必设对
CONFIG_ESP32S3_DEFAULT_CPU_FREQ_240=y
CONFIG_FREERTOS_HZ=1000           # 与 esp-drone 默认一致
CONFIG_FREERTOS_UNICORE=n
CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y   # 无 UART 桥，必须走原生 USB
```
> Octal PSRAM 会占用 GPIO35/36/37。这三脚在 pyDrone 上未引出（原理图 netlist 中 `NLGPIO35/36/37` 只连到模块引脚，无其他去向），所以**不要**给 SPI/Flow Deck 分配这些脚。

### 步骤 4 — LED 表

`components/drivers/general/led/include/led.h`：pyDrone 只有 2 路可控 LED（蓝46/绿42），红色是电源硬连、黄色是充电芯片 STAT，均不可编程。

```c
#define LED_NUM 2
typedef enum {LED_BLUE = 0, LED_GREEN} led_t;
#define CHG_LED LED_BLUE      // 原 LED_RED 的引用全部改指向现有 LED
#define LOWBAT_LED LED_BLUE
#define ERR_LED1 LED_GREEN
#define ERR_LED2 LED_GREEN
```
极性：原理图为 GPIO→1k→LED 阳极，阴极接 GND，即**高电平点亮**，保持 `LED_POL_POS` 不变。

### 步骤 5 — 电池 ADC（数值必须改，否则电量判断全错）

pyDrone 分压：R4 40.2k / R7 10k → 比值 **(40.2+10)/10 = 5.02**

```c
// components/core/crazyflie/hal/src/pm_esplane.c
pmEnableExtBatteryVoltMeasuring(CONFIG_ADC1_PIN, 5.02f);   // 原为 2
```
```c
// components/drivers/general/adc/adc_esp32.c
static const adc_atten_t atten = ADC_ATTEN_DB_0;   // 原为 3(11dB)
```
理由：VBAT 4.2V ÷ 5.02 = 0.836V，落在 0dB 衰减（满量程 ~1.1V）范围内；沿用 11dB 会严重损失分辨率。py-drone 用的就是 `ADC_ATTEN_DB_0` + 倍数 5，与原理图一致，可交叉验证。

### 步骤 6 — 气压计：新增 SPL06 组件

```bash
cp -r <micropython>/ports/esp32/py-drone/drivers/i2c_devices/spl06 \
      esp-drone/components/drivers/i2c_devices/spl06
```
补 `CMakeLists.txt`（仿 ms5611 的写法），然后在 `sensors_mpu6050_hm5883L_ms5611.c`：

```c
#define SENSORS_ENABLE_PRESSURE_SPL06     // 替代 MS5611
#define SENSORS_ENABLE_MAG_HM5883L        // 打开（实为 QMC5883L）

#define SENSORS_BARO_STATUS_LEN   1
#define SENSORS_BARO_DATA_LEN     6
#define SENSORS_BARO_BUFF_S_P_LEN SENSORS_BARO_STATUS_LEN
#define SENSORS_BARO_BUFF_T_LEN   SENSORS_BARO_DATA_LEN
```
`sensorsSetupSlaveRead()` 的 slave2/3 按 py-drone 配置（slave2 读 `SPL06_MODE_CFG_REG(0x08)` 1 字节状态，slave3 读 `SPL06_PRESSURE_MSB_REG(0x00)` 6 字节），`processBarometerMeasurements()` 换成 `spl0601_get_temperature/get_pressure`。这些函数从 `py-drone/port/sensors_mpu6050_spl06.c` 直接搬。

### 步骤 7 — 磁力计：换成 QMC5883L

用 py-drone 版 `hmc5883l.c/h`（文件名保留、内部实为 QMC5883L）覆盖 esp-drone 同名文件，关键点：地址 0x0D、ID 读 0x0D 得 0xFF、数据 0x00 起小端 X/Y/Z、`MAG_GAUSS_PER_LSB = 12000`、状态位 `QMC5883L_STATUS_DRDY_BIT`。`SENSORS_MAG_BUFF_LEN` 由 8 改为 **7**（1 状态 + 6 数据）。

### 步骤 8 — 裁掉不存在的外设

```c
// CONFIG_BUZZER_ON=n 后确认 buzzerInit/soundInit 空实现或条件编译掉
// 关闭 Flow Deck / ToF（无扩展板、SPI 脚被 PSRAM 占）
// sensors_mpu6050_hm5883L_ms5611.c
// #define SENSORS_ENABLE_RANGE_VL53L1X
// #define SENSORS_ENABLE_FLOW_PMW3901
```
EEPROM：pyDrone 无 24Cxx。ESP-Drone 的 `eeprom.c` 里 `eepromTest()`/`eepromTestConnection()` **已被改成无条件 `return true`**，`configblockInit()` 在校验失败时会回落到默认配置，所以自检不会卡住 —— 但配置无法持久化。若需要持久化，改用 `configblockflash.c`（仓库已有）或走 NVS。

### 步骤 9 — 定高策略

无 ToF/光流，`estimator_kalman` 的 Z 通道缺观测量。两个选择：

1. **先只做自稳**：`ENABLE_POSITION_HOLD_MODE=n`，把 `estimator` 设为 `complementary`，先保证能飞。
2. **移植气压定高**：把 py-drone 的 `position_estimator.c`（气压→高度互补滤波）+ `fastAdjustPosZ()` 逻辑接进 esp-drone 的 `estimator_complementary`，或给 Kalman 加气压高度观测。工作量明显大于第 1 步，建议自稳飞通后再做。

### 步骤 10 — 编译烧写

```bash
idf.py menuconfig      # ESPDrone Config → 选 pyDrone 板型
idf.py build
idf.py -p /dev/ttyACM0 flash monitor     # USB-Serial-JTAG 是 ACM 设备
```
> 烧写前请先备份原厂 MicroPython 固件：`esptool.py -p /dev/ttyACM0 read_flash 0 0x800000 pydrone_backup.bin`，否则难以回退。

---

## 4. 验证顺序（**务必拆桨执行 T1–T4**）

| 编号 | 项目 | 通过标准 |
|---|---|---|
| **T0** | 备份原厂固件 | `pydrone_backup.bin` 可回刷 |
| **T1** | 上电不转桨 | **拆桨**。串口无 `i2c driver install` 报错；M3/M4（GPIO40/41）无抽动 → 验证 deck I²C 已改离 40/41 |
| **T2** | I²C 扫描 | 日志出现 MPU6050 `[OK]`、SPL06 `[OK]`、QMC5883L ID=0xFF |
| **T3** | 电机映射 + 方向 | **拆桨**。用 `param` 逐个 `motorPowerSet.m1..m4`，确认序号与 M1左上/M2右上/M3右下/M4左下 一致；再验 pitch 符号（见 2.3），装反必翻机 |
| **T4** | 传感器读数 | 静止时 acc.z≈1g；绕各轴转动 gyro 符号符合右手系；气压高度随抬升单调下降 |
| **T5** | 电池电压 | cfclient `pm.vbat` 与万用表实测误差 <0.1V → 验证 5.02 分压比与 0dB 衰减 |
| **T6** | 通信 | 连 AP `ESP-DRONE`，cfclient 能连上、参数树可读 |
| **T7** | 系留悬停 | 用绳系住，小油门自稳；确认无发散再自由飞 |

---

## 5. 路线 B 要点（保留 MicroPython）

若必须保留 `drone.DRONE` Python API，不要替换整个栈，建议**只替换通信与控制层**，保留 py-drone 的 `system_int.c` 启动流程：

1. 硬件驱动层（`i2cdev`/`mpu6050`/`motors`/`led`/`pm`/`adc`/SPL06/QMC5883L）**完全不动** —— 已经是对的。
2. 把 esp-drone 的 `crtp.c`、`crtpservice.c`、`crtp_commander_rpyt.c`、`wifilink.c`、`param.c`、`log.c` 加入 `DRONE_SRCS`，与现有 `anop.c` 并存（UDP 端口区分或按首字节分流 —— 两者都用 2390）。
3. **难点**：`log.c`/`param.c` 依赖 `linker_fragment.lf` 的自定义链接段来收集 `LOG_GROUP`/`PARAM_GROUP`。MicroPython 的 `idf_component_register` 没有 `LDFRAGMENTS`，需手动补 `target_link_libraries(... LDFRAGMENTS ...)` 或改用显式注册表。
4. **IDF 版本**：需先把 01studio 的 MicroPython 移到 IDF v5.0，或把 esp-drone 代码降级到 v4.4（`esp_adc_cal`、`i2c` 旧 API、`gpio_pad_select_gpio` 等在两版间有差异）。
5. 内存：MicroPython heap + PSRAM + EKF 三者叠加，S3 8MB PSRAM 够用，但 `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL` 需调，避免飞控任务栈落到 PSRAM 上（有延迟抖动风险）。

工作量估计约为路线 A 的 2–3 倍，且调试面更大。**建议先按路线 A 把飞控跑通、参数调好，再决定是否回接 MicroPython。**

---

## 6. 待确认事项（无法仅凭代码/原理图定论）

1. **IMU 贴装方向**：py-drone 与 esp-drone 的 `processAccGyroMeasurements()` 在非 ESPLANE_V1 分支下完全相同（交换 X/Y、X 取反），但 py-drone 把 `sensorsAccAlignToGravity()` 用 `#if 0` 关掉了。pyDrone PCB 丝印有 X/Y/Z 标记，需按 T4 实测确认是否需要额外旋转。
2. **pitch 反号的真实来源**：是电机编号差异还是 IMU 朝向差异，会影响该改 `power_distribution` 还是改传感器轴映射。台架实测（T3）才能定论。
3. **PID 参数**：pyDrone 是 716 电机 + 46mm 桨 + 400mAh，与 ESP-Drone 的 720 电机整机惯量/推重比不同，esp-drone 默认 PID 大概率需重调（`ROLL_CALIB`/`PITCH_CALIB` 与 `controller_pid.c` 增益）。
4. **官方原理图版本**：本文基于 `pyDrone_Sch.pdf` v1.0 (2022-07-14)。若手上是后续改版，需重新核对引脚。
