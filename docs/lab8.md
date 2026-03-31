# Lab 8 

## The Plan 

I will have 4 states:
1. `APPROACH`: full speed forward, KF running, waiting for dist < 914mm
2. `SPIN`: motors stop forward, orientation PID spins 180 degrees, waiting for yae settled.
3. `RETURN`: full speed backward (away from wall = forward in new direction), fixed duration or until BLE stop
4. `DONE`: stop everything, log data

## Design Decisions: 
- During `APPROACH`, KF predicts between ToF readings exactly like Lab 7, but input `u` is the fixed forward PWM (not a PID output)
- The spin target yaw = `current_yaw + 180` (with wrap-around), handed off to my existing `computeOrientPID`
- For `RETURN`, just raw PWM for a fixed duration, no need for PID
- Data logging reuses mu existing `kf_*` arrays

## New Additions:
- `START_DRIFT` command + enum entry
- A `SEND_DRIFT_DATA` command (can reuse `SEND_KF_DATA` format)
- A `drift_state` enum
- A `drift_speed` PWM parameter sent from Python
- 
