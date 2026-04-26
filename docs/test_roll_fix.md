# Roll 修复验证测试步骤

## 当前代码状态

| 开关 | 状态 | 文件 |
|------|------|------|
| `MOTOR_OUTPUT_DISABLE` | **打开** (电机不转) | `power_distribution_stock.c` |
| `DEBUG_PWR_DBG` | **打开** | `config.h` |
| `DEBUG_CTRL_DBG` | 关闭 | `config.h` |
| `DEBUG_IMU_DBG` | 关闭 | `config.h` |
| `control->roll = -control->roll` | **已注释** (本次修复) | `controller_pid.c` |

串口只输出 PWR_DBG 一路，每 2 秒一条，格式：
```
PWR_DBG ctrl[thr=XXXXX r=+XXXX p=+XXXX y=+XXXX] M[XXXX XXXX XXXX XXXX]
```
其中 `M[M1 M2 M3 M4]` 对应 右前 右后 左后 左前。

---

## 第一步：手持验证方向 (不装桨, 电机不转)

编译烧录后，App 连接并推一点油门（让 thrust > 0 使 PID 工作），手持飞机做以下动作：

### 测试 1: Roll 轴 (本次修复重点)

**右翼下压** (飞机向右倾斜):
```
期望: M1 M2 增大, M3 M4 减小  (右侧加油抬右翼)
      M[大 大 小 小]
```

**左翼下压** (飞机向左倾斜):
```
期望: M3 M4 增大, M1 M2 减小  (左侧加油抬左翼)
      M[小 小 大 大]
```

> 如果相反 → 修复失败，需要再排查信号链。

### 测试 2: Pitch 轴 (顺便验证)

**机头下压**:
```
期望: M1 M4 增大, M2 M3 减小  (前侧加油抬机头)
      M[大 小 小 大]
```

**机尾下压**:
```
期望: M2 M3 增大, M1 M4 减小  (后侧加油抬机尾)
      M[小 大 大 小]
```

### 测试 3: Yaw 轴 (顺便验证)

**顺时针旋转飞机** (俯视, 机头右转):
```
期望: M1 M3 (CW电机) 增大  → 产生反扭矩阻止右转
      M[大 小 大 小]
```

---

## 第二步：验证通过后，进入栓绳试飞

需要改回的开关:

1. `power_distribution_stock.c` → 注释掉 `#define MOTOR_OUTPUT_DISABLE`
2. `config.h` → 按需选择保留哪些打印 (建议只留 `DEBUG_CTRL_DBG`)

然后:
1. 装桨，栓绳固定
2. 缓慢推油门到刚离地
3. 观察是否稳定悬停，无侧翻趋势
4. 确认无问题后解绳自由飞行
