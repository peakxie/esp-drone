列出需要还需要哪些任务，方便后续实现                                                                                                  

● 我先修掉自己写的一个拼写错误，并核对分区表/剩余引用点，以便任务清单准确。

● Update(sdkconfig.defaults.pydrone)
  ⎿  Added 1 line, removed 1 line
      20  CONFIG_SPIRAM_MODE_OCT=y
      21  CONFIG_SPIRAM_SPEED_80M=y
      22  # Keep task stacks and DMA-capable buffers in internal RAM: the flight control
      23 -# tasks are latency sensitive and PSUAM access jitters.                                                                
      23 +# tasks are latency sensitive and PSRAM access jitters.                                                                
      24  CONFIG_SPIRAM_USE_MALLOC=y
      25  CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096
      26  

  必须做（否则无法烧写/上天）
  1. 编译验证 — 本次按你要求未编译。本地 IDF 是 v5.4（计划假设 v5.0），esp_adc_cal 在 v5.4 已进 deprecated/ 目录，adc_esp32.c
  能否直接过需实测。
  2. T0 备份原厂固件 — esptool.py read_flash 0 0x800000，否则无法回退 MicroPython。
  3. T1–T7 台架验证 — 全部需要真机，T1–T4 务必拆桨。

  需实测后回填代码的（我故意没定值）
  4. pitch 符号（T3） — 实测后设 CONFIG_PITCH_DISTRIBUTION_INVERTED。若查明是 IMU 贴装朝向而非电机编号，则应改
  processAccGyroMeasurements() 的轴映射（sensors_mpu6050_hm5883L_ms5611.c:429-475）而不是动力分配。
  5. IMU 轴向确认（T4） — py-drone 把 sensorsAccAlignToGravity() 用 #if 0 关掉了，我这边保留启用；若实测有静态倾角偏差，用
  PITCH_CALIB/ROLL_CALIB 校正。
  6. PID 重调 — 716 电机 + 46mm 桨 + 400mAh 惯量与 720 不同。我已把电机类型设为 MOTOR_BRUSHED_715，这会让 position_controller_pid.c:143
  走 715 分支（thrustBase 24000→36000），但 pid.h:33 的姿态环增益仍是 S2_DRONE_V1_2 专属值，pyDrone 会落到 #else 默认组 ——
  需要确认是否要给 pyDrone 单独一组增益。

  可选/待定
  7. 配置持久化 — 现在 EEPROM 自检恒真、回落默认配置，不会卡启动但配置存不下来。若需要持久化，configblockflash.c 不是即插即用（缺
  configblockGetRadioAddress、CONFIG_BLOCK_ADDRESS 是 STM32 地址），得改走 NVS。
  8. 分区表 — 目前用 IDF 默认单 app 表；已声明 8MB flash 但没自定义 CSV，若固件超默认 app 分区需要加 partitions.csv。
  9. CI — .idf_build_apps.toml 扫全仓库，新板型不会自动被构建覆盖；如需 CI 验证 pyDrone 配置，要加 .build-test-rules.yml 条目。
  10. 文档/原理图 — docs/{en,zh_CN}/rst/hardware.rst 未提 pyDrone，hardware/ 下也没有对应目录。

  一处提醒：asl 我保留米为单位（py-drone 用的是厘米），因为 esp-drone 的 position_estimator_altitude.c:96 和 Kalman 气压更新都按米处理
  —— 照抄 py-drone 的 * 100.f 会让定高增益差 100 倍。
