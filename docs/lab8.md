# Lab 8: Stunts - Task B: Drift

## Overview

The robot starts about 4 meters from the wall, drives forward using linear PID control. and initiates a 180° orientation-PID-controlled spin when it is about a meter from the wall, before returning past the starting line. 

## Design

My initial plan was to use the Kalman Filter from Lab 7 suring the appproach phase for faster distance estimation and drive at full raw PWM. However, several design decisions changed during implementation: 

- I did not use KF: the KF parameters (d,m) were determined using PWM 150 during Lab 7. At PWM 190, the model was worng and so the KF estimate diverged. I opted for using raw ToF.
- Linear PID replaced raw PWM: using full speed caused the car to crash into the wall before triggering. Linear PID from Lab 5 was a more reliable approach

The drift runs inside `drift_step()`, called in every iteration of the main `loop()`:

- STATE 0 - initialize: capture starting yaw, reset all PID state, enable linear PID
- STATE 1 - approach: linear pID drives toward wall, triggers when ToF < 1500 mm
- STATE 2 - setup spin: compute `yaw_start + 180°` target wth angle wrapping, enable orientation PID.
- STATE 3 - spin: orientation PID rotates car to target, exists when yaw error < 10° (after minimum 500ms)
- STATE 4 - return: motorsForward(200) for 1 seconds

```c
void drift_step() {
  if (!drift_active) return;
  unsigned long now = millis();
  
  // STATE 0: initialize
  if (drift_state_num == 0) {
    yaw_start = global_yaw;
    if (tof_curr_dist <= 0) return;
    pid_pos_target = drift_trigger_dist;
    flag_pid_pos = true;
    drift_state_num = 1;
  }
  // STATE 1: approach
  else if (drift_state_num == 1) {
    if (dist > 0 && dist <= drift_trigger_dist) {
      flag_pid_pos = false;
      motorsStop();
      drift_state_num = 2;
    }
  }
  // STATE 2: setup spin
  else if (drift_state_num == 2) {
    float target_yaw = yaw_start + 180.0;
    if (target_yaw > 180.0) target_yaw -= 360.0;
    if (target_yaw < -180.0) target_yaw += 360.0;
    orient_setpoint = target_yaw;
    flag_pid_orient = true;
    drift_state_num = 3;
  }
  // STATE 3: spin
  else if (drift_state_num == 3) {
    if (now - drift_state_start < 500) return;
    float yaw_err = fabs(global_yaw - orient_setpoint);
    if (yaw_err > 180.0) yaw_err = 360.0 - yaw_err;
    if (yaw_err < 10.0) {
      flag_pid_orient = false;
      motorsStop();
      delay(200);
      drift_state_num = 4;
    }
  }
  // STATE 4: return
  else if (drift_state_num == 4) {
    motorsForward(200);
    if (now - drift_state_start > 2000) {
      motorsStop();
      drift_active = false;
    }
  }
}
```
### The Return Phase

After the 180° spin, the ToF sensor faces away from the wall and cannot measure return distance. I used a timed `motorsForward()` call at 200 PWM for 1 second. 

### PID Gains

- Linear PID: Kp = ; Ki = ; Kd =
- Orientation PID: Kp = ; Ki = ; Kd =

## Debugging Issues 

This lab was supposed to be quick and easy and I spent countless hours debugging the following issues: 

1. **Car crashing into wall:** the ToF was mounted slightly downward, and it was reading the floor. This meant the trigger condition fired immediately regardless of wall distance, causing the car to jump straight to STATE 3 (spin) without approaching.
2. **Orient PID gains:** with `Kp = 0.5`, the PID output was only `0.05 * 180 = 9`, below the minimum threshold in `motorsOrient()`. The car never spun, it would just stop 3 feet from the wall. Raisin Kp to XXXXXX fixed this.

## Results 

VIDEO 1 
PLOT 1

VIDEO 2
PLOT 2

## Conclusion

The drift stint succesfully demonstrates combined use of linear PID, orientation PID, and ToF sensing. The main challenges were managing DMP FIFO reads, tuning the trigger distance for high-speed spproach, and maintaining BLE connectivity during the stunt. 

## References

I used XXX page for code reference. I used Claude 
