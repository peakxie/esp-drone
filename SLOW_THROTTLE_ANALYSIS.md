# ESP-Drone Slow Throttle Ramp-Up Analysis: Root Causes of Sideways Sliding

## Executive Summary
The sideways sliding during slow throttle ramp-up is caused by a **motor clipping problem** in the power distribution system combined with **integral windup in the attitude PID controllers**. When thrust is ramping up slowly but PID corrections are relatively large, individual motor outputs can be clipped to zero, asymmetrically disabling motors and causing the drone to slide sideways instead of lifting off.

---

## 1. Power Distribution: Motor Clipping Issue

### File: `/data/project/source/peakxie/esp-drone/components/core/crazyflie/modules/src/power_distribution_stock.c`

#### Problem #1: limitThrust Hard Clips Negative Values to Zero
**Lines 81-82 & 85-96:**
```c
#define limitThrust(VAL) limitUint16(VAL)

void powerDistribution(const control_t *control)
{
  #ifdef QUAD_FORMATION_X
    int16_t r = control->roll / 2.0f;
    int16_t p = control->pitch / 2.0f;
    motorPower.m1 = limitThrust(control->thrust - r + p + control->yaw);  // Line 96
    motorPower.m2 = limitThrust(control->thrust - r - p - control->yaw);  // Line 97
    motorPower.m3 = limitThrust(control->thrust + r - p + control->yaw);  // Line 98
    motorPower.m4 = limitThrust(control->thrust + r + p - control->yaw);  // Line 99
  #else // QUAD_FORMATION_NORMAL
    motorPower.m1 = limitThrust(control->thrust + control->pitch + control->yaw);     // Line 101-102
    motorPower.m2 = limitThrust(control->thrust - control->roll - control->yaw);      // Line 103-104
    motorPower.m3 = limitThrust(control->thrust - control->pitch + control->yaw);     // Line 105-106
    motorPower.m4 = limitThrust(control->thrust + control->roll - control->yaw);      // Line 107-108
  #endif
}
```

**The Issue:**
- The mixing formula subtracts PID corrections from thrust to distribute control
- When thrust is LOW (e.g., 100 units) but PID corrections are LARGE (e.g., 120 units for roll/pitch correction), the formula produces **negative values**
- Example: `thrust=100 - roll_correction=120 = -20`
- The `limitThrust()` macro calls `limitUint16()` which clips negative values to **0**

#### Implementation: `/data/project/source/peakxie/esp-drone/components/core/crazyflie/utils/src/num.c` (Lines 85-97)
```c
uint16_t limitUint16(int32_t value)
{
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  else if(value < 0)          // ← HARD CLIPS NEGATIVE TO ZERO!
  {
    value = 0;
  }
  return (uint16_t)value;
}
```

### Root Cause of Sideways Sliding:
1. User slowly ramps throttle: 0 → 50 → 100 → 150 → 200
2. Drone has attitude error (e.g., tilted right)
3. PID generates roll correction: -150 (to stop rightward tilt)
4. Motor formula: M2 = thrust - roll = 100 - 150 = **-50** 
5. `limitThrust(-50)` → **0** (motor M2 completely stops!)
6. Asymmetric thrust: M1=100, M2=0, M3=100, M4=100
7. **Result: Drone can't lift (thrust 300/4=75 avg < weight), and asymmetry causes sideways roll**

### Problem #2: idleThrust Only Applies AFTER limitThrust
**Lines 142-159:**
```c
if (motorPower.m1 < idleThrust) {
  motorPower.m1 = idleThrust;                    // Line 143
}
// ... similar for M2, M3, M4
```

**The Issue:**
- The `idleThrust` safety check comes AFTER the hard clipping
- Even if `idleThrust` is set to 10, the sequence is:
  1. **First:** `motorPower.m1 = limitThrust(negative_value)` → 0 (hard clipped)
  2. **Then:** Check if `0 < idleThrust(10)` → Yes, so set to 10
  
- **This only partially masks the asymmetry problem** because:
  - If `idleThrust=10` and one motor is clipped to 0, it's bumped to 10
  - But multiple motors may be clipped, causing different final values
  - The actual thrust deficit is still there

### Default Idle Thrust Configuration:
**Lines 58-62:**
```c
#ifndef DEFAULT_IDLE_THRUST
#define DEFAULT_IDLE_THRUST 0                    // ← DEFAULT IS ZERO!
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;
```

**Impact:**
- By default, `DEFAULT_IDLE_THRUST = 0`, meaning **NO minimum motor speed**
- Individual motors can be stopped completely via clipping
- This is the primary enabler of the sideways sliding issue

---

## 2. PID Controller: Integral Windup Problem

### File: `/data/project/source/peakxie/esp-drone/components/core/crazyflie/modules/src/pid.c`

#### Problem #3: PID Integral Keeps Accumulating During Low Thrust
**Lines 56-102:**
```c
float pidUpdate(PidObject* pid, const float measured, const bool updateError)
{
    float output = 0.0f;

    if (updateError)
    {
        pid->error = pid->desired - measured;
    }

    pid->outP = pid->kp * pid->error;
    output += pid->outP;

    // ... Derivative term ...

    pid->integ += pid->error * pid->dt;           // ← Line 81: ALWAYS ACCUMULATES!

    // Integral limit is only on the integral itself, not on the total output
    if(pid->iLimit != 0)
    {
    	pid->integ = constrain(pid->integ, -pid->iLimit, pid->iLimit);  // Line 86
    }

    pid->outI = pid->ki * pid->integ;
    output += pid->outI;

    // Output limit applies AFTER all terms combined
    if(pid->outputLimit != 0)
    {
      output = constrain(output, -pid->outputLimit, pid->outputLimit);  // Line 95
    }

    pid->prevError = pid->error;
    return output;
}
```

**The Issue:**
- The integral term **accumulates continuously** regardless of thrust level
- During slow ramp-up, the drone sits at a low thrust (below weight), maintaining attitude error
- The attitude error keeps building up in the integral: `integ += error * dt`
- By the time thrust reaches sufficient levels for flight, the integral is **saturated**
- This large integral term forces big PID corrections that exceed available thrust when distributed to motors

### PID Configuration with Large Integral Gains:
**File: `/data/project/source/peakxie/esp-drone/components/core/crazyflie/modules/interface/pid.h` (Lines 34-66)**

For ESP32-S2 Drone:
```c
#define PID_ROLL_RATE_KI  440.0           // ← VERY HIGH!
#define PID_PITCH_RATE_KI  440.0          // ← VERY HIGH!
#define PID_ROLL_KI  2.5
#define PID_PITCH_KI  2.5
#define PID_ROLL_RATE_INTEGRATION_LIMIT    33.3
#define PID_PITCH_RATE_INTEGRATION_LIMIT   33.3
#define PID_ROLL_INTEGRATION_LIMIT    20.0
#define PID_PITCH_INTEGRATION_LIMIT   20.0
```

The rate PID has 440.0 integral gain! This accumulates errors very quickly.

### Integral Limits Don't Prevent Motor Clipping:
**Lines 84-87 of pid.c:**
```c
if(pid->iLimit != 0)
{
    pid->integ = constrain(pid->integ, -pid->iLimit, pid->iLimit);
}
```

The integral is limited to ±33.3, so max rate PID output from I term = 440 × 33.3 ≈ **14,652**!
This is applied as a **correction value** that's subtracted from thrust in the mixing equation, easily exceeding available thrust during slow ramp-up.

---

## 3. PID Reset Logic: Only Resets at Zero Thrust

### File: `/data/project/source/peakxie/esp-drone/components/core/crazyflie/modules/src/controller_pid.c`

#### Problem #4: PID NOT Reset During Slow Ramp-Up
**Lines 157-174:**
```c
if (control->thrust == 0)                        // ← Line 157: ONLY resets when thrust = ZERO
{
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    attitudeControllerResetAllPID();              // ← Line 169: Calls this function
    positionControllerResetAllPID();

    attitudeDesired.yaw = state->attitude.yaw;
}
```

**The Issue:**
- PID resets ONLY when `thrust == 0` (not flying)
- During slow ramp-up from 0 to flight thrust, the integral never resets
- The drone accumulates attitude error for the entire ramp period
- By the time thrust is high enough to fly, the integral is saturated
- When the drone finally gets off the ground, the huge PID corrections cause immediate sideways sliding

### Sequence of Events During Slow Ramp-Up:

```
Time=0ms:   thrust=0,    integral=0,    drone on ground
Time=100ms: thrust=50,   error_accumulating,    integral growing
Time=200ms: thrust=100,  error_accumulating,    integral=100×2.5×0.2s=50
Time=300ms: thrust=150,  error_accumulating,    integral=150×2.5×0.2s=75
Time=400ms: thrust=200,  integral_saturated,    integral=-33.3 (if tilted left) or +33.3 (if tilted right)
Time=500ms: thrust≈230,  LIFTOFF OCCURS
           → Motor clipping happens because integral × 440 ≈ 14,652 correction >> 230 available thrust
           → Asymmetric motors stop → SLIDE SIDEWAYS!
```

---

## 4. Attitude Controller: No Anti-Windup During Flight Readiness

### File: `/data/project/source/peakxie/esp-drone/components/core/crazyflie/modules/src/attitude_pid_controller.c`

#### Problem #5: Rate PID Integral Keeps Growing Without Saturation Signal
**Lines 113-125:**
```c
void attitudeControllerCorrectRatePID(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired)
{
  pidSetDesired(&pidRollRate, rollRateDesired);
  rollOutput = saturateSignedInt16(pidUpdate(&pidRollRate, rollRateActual, true));

  pidSetDesired(&pidPitchRate, pitchRateDesired);
  pitchOutput = saturateSignedInt16(pidUpdate(&pidPitchRate, pitchRateActual, true));

  pidSetDesired(&pidYawRate, yawRateDesired);
  yawOutput = saturateSignedInt16(pidUpdate(&pidYawRate, yawRateActual, true));
}
```

**The Issue:**
- The output is saturated to int16 range via `saturateSignedInt16()` (**Lines 43-52**)
- But this saturation happens AFTER the PID calculation
- The integral in the PID object continues accumulating even though output is saturated
- **This is classic integral windup**: the integrator doesn't know it's saturating

**Anti-windup is Missing:**
- No mechanism to stop integral accumulation when output is saturated
- No back-calculation to unwind the integral
- No conditional integral reset based on thrust availability

---

## 5. Integration of Root Causes: The Perfect Storm

### Sequence Summary:

1. **Low Thrust Phase (0-500ms):**
   - Throttle ramps slowly: 0 → 50 → 100 → 150 → 200 → 230
   - Drone sits on ground with slight attitude error (e.g., tilted right)
   - **Rate PID integral accumulates continuously** (no reset, no anti-windup)
   - By t=400ms: Integral is saturated at ±33.3, output × 440 = 14,652 correction needed

2. **Liftoff Phase (500ms+):**
   - Thrust crosses weight threshold, drone starts lifting
   - **Motor mixing happens:** Each motor = thrust - corrections
   - Example: thrust=230, roll_correction=14,652
   - M1 = 230 - 14,652 = -14,422 → **Clipped to 0** via limitThrust()
   - M2 = 230 + 14,652 = 14,882 → **Clipped to UINT16_MAX** via limitThrust()
   - M3 = 230 - 14,652 = -14,422 → **Clipped to 0**
   - M4 = 230 + 14,652 = 14,882 → **Clipped to UINT16_MAX**

3. **Asymmetric Motor Failure:**
   - Motors 1&3 stop (clipped to 0), Motors 2&4 max out
   - Thrust becomes extremely asymmetric
   - **Result: Drone tilts hard right (M1&3 unpowered) and slides sideways**

### Why Slow Ramp-Up Makes It Worse Than Fast Throttle:
- **Slow ramp-up:** Integral accumulates for longer time before thrust reaches liftoff
- **Fast throttle:** Less time for integral to wind up before liftoff occurs
- **Fast throttle:** Any initial clipping happens, then drone catches up quickly and PID stabilizes

---

## 6. Summary of Exact Problem Locations

| # | Problem | File | Line | Impact |
|---|---------|------|------|--------|
| **1** | `limitThrust()` hard clips negative motors to 0 | `power_distribution_stock.c` | 81, 85-96 | Motor asymmetry during low thrust |
| **2** | `limitUint16()` clips negatives without minimum motor floor | `num.c` | 85-97 | No minimum idle thrust enforcement |
| **3** | `DEFAULT_IDLE_THRUST = 0` (default) | `power_distribution_stock.c` | 59 | Motors can stop completely |
| **4** | `idleThrust` check comes AFTER clipping | `power_distribution_stock.c` | 142-159 | Doesn't prevent hard clipping |
| **5** | PID integral always accumulates | `pid.c` | 81 | Integral windup during low thrust |
| **6** | No integral reset during slow ramp-up | `controller_pid.c` | 157 | Saturated integral causes large corrections |
| **7** | No anti-windup in rate PID | `attitude_pid_controller.c` | 113-125 | Integral keeps growing when output saturates |
| **8** | Large integral gains (KI=440) | `pid.h` | 35-36 | Big corrections = big clipping |

---

## 7. Why This Appears as "Sideways Sliding"

The asymmetric motor clipping in the X-formation creates a **roll oscillation that manifests as sideways sliding**:

- **Quad-X formation:** Motors are at 45° angles
- **Motor 1 & 3 on one diagonal stop** (clipped to 0)
- **Motor 2 & 4 on other diagonal max out**
- **Net effect:** Huge roll moment about the longitudinal axis
- **Drone tilts and slides sideways** instead of lifting smoothly

---

## Recommended Fixes (In Priority Order)

1. **Implement Integral Anti-Windup:** Stop accumulating integral when output saturates (at motor mixing stage)
2. **Add Minimum Motor Floor:** Enforce `idleThrust` BEFORE motor clipping, not after
3. **Reset Integral on Liftoff:** Detect when drone transitions from ground to flight and reset PID
4. **Slow Ramp-Up Limiter:** Limit throttle ramp rate to match maximum rate PID can safely correct
5. **Reduce Integral Gains:** Lower `KI` values for rate PIDs (currently 440 is very aggressive)

---

