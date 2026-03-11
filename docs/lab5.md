# Prelab

## Clearly describe how you handle sending and receiving data over Bluetooth
## Consider adding code snippets as necessary to showcase how you implemented this on Arduino and Python

I implemented a BLE command system that allows me to start/stop PID control, update gains, and retrieve logged data wirelessly from Python. 

I added the following new commands to my `CommandTypes` enum:

- `SET_PID_GAINS`: sends Kp, Ki, Kd floats over BLE
- `ENABLE_MOTORS`/`DESABLE_MOTORS`: toggle motor output
- `START_PID_POS_WITH_DATA`: begins pID control and data logging with a target distance
- `STOP_PID_POS_WITH_DATA`: ends pID control and stops motors
- `SEND_PID_DATA`: transmits timestamped ToF and motor output arrays back to Python

The Python workflow works like this: 

````
ble.send_command(CMD.SET_PID_GAINS, "0.5|0.0|0.1")
ble.send_command(CMD.ENABLE_MOTORS, "")
ble.send_command(CMD.START_PID_POS_WITH_DATA, "304")  # 304mm = 1ft
# wait for maneuver
ble.send_command(CMD.STOP_PID_POS_WITH_DATA, "")
ble.send_command(CMD.SEND_PID_DATA,"")
````
PID runs non-blocking inside the main `while (central.connected())` loop, checking `checkForDataReady()` on each iteration rather than blocking with a `while(!ready) wait`. Data is stored in timestamped arrays (`pid_time_data`, `pid_tod_data`, `pid_motor_data`) and sent over BLE after the maneuver completes. 

For safety, I implemented a hard stop that triggered when BLE disconnects:

````
flag_pid_pos = false;
    flag_record_pid = false;
    motors_enabled = false;
    motorsStop();
    distanceSensor1.stopRanging();
    distanceSensor2.stopRanging();
````
This ensures the car stops if the Bluetooth connection drops mid-run. 

# Lab Tasks

## P/I/D Discussion 
My task was to drive the car from a starting distance of ~1.2m and stop it exactly 204mm (1 foot) from the wall using closed-loop PID position with a VL53L1X ToF sensor.

## Controller selection
I implemented a full PID controller. Starting with P-only control gave reasonable behavior as the car approached and settled near 304mm. However, the car had a small steady-state error that I wanted to eliminate. Adding the I term corrected this. The D term helped reduce overshoot during the fast approach. 

**Final gains:** `Kp = 0.07, Ki = 0.002, Kd = 0.005`

A speed scale factor `pid_speed_scale` multiplies the PID output before clamping, allowing testing at 50%, 80%, and 100% of full speed without changing the gains. 

**Motor deadband:** Motors stop when the error is within ±20mm of the target, preventing chatter near the setpoint. Forward motor PWM is floored at 0 (no floor needed since proportional output is already large far from the wall), and backward PWM is floored at 55 to overcome static friction. 

## P/I/D discussion (Kp/Ki/Kd values chosen, why you chose a combination of controllers, etc.)

## Task 1: Frequency: Determine the frequency at which the ToF sensor is returning new data.

## Task 1: Frequency 
The ToF sampling frequency is dependent on the timing budget. Because the goal here is to make a fast PID controller that can stop the car from running into walls, I opted for a 20ms timing budget, which is the fastest supported by our ToF sensors in long mode and yields a frequency of 50 Hz. To cofirm this, I printed times to the serial monitor as fast as possible while conditionally reading ToF data, like in lab 3: 

````
if (flag_pid_pos) {
        static int pid_count = 0;
        static int tof_count_local = 0;

        // Update ToF reading only when new data is ready
        if (distanceSensor2.checkForDataReady()) {
          tof_count_local++;
          float new_dist = distanceSensor2.getDistance();
          distanceSensor2.clearInterrupt();
````
Testing revealed that the default timing bidget gave ~100ms intervals (10 Hz), which was far too slow for a responsive pID controller. After setting `setTimingBudgetInMs(20)`, the intervals dropped to ~20 ms, a 50 Hz sampling rate and a 5x improvement. 

Since the car needs to start beyond 1.3m to have room to reach full speed, short mode's range limit was also a problem. I switched to long mode, which supports up to 4m. Together with the 20ms timing budget, the sensors trade some accuracy for speed and range, which is acceptable for this task. 

## Range and Sampling Time Discussion 
The VL53L1X ToF sensor was configured as follows:
````
distanceSensor2.setDistanceModeLong();
distanceSensor2.setTimingBudgetInMs(20);
distanceSensor2.setIntermeasurementPeriod(20);
````
This gave a ToF **sampling rate of ~50 Hz**. The PID control loop, however, runs at **~800 Hz** (about 16x faster than the sensor). To bridge this gap, I implemented **distance extrapolation:** between ToF readings, the controller estimates the current distance using the last measured velocity (slope):
````
float elapsed = (millis() - tof_curr_time) / 1000.0;
float extrap_dist = tof_curr_dist + tof_slope * elapsed;
````
The extrapolated distance is fed into `computePID()` every loop interation, while `tof_slope` is updated only when a new sensor reading arrives. This decouples the control rate from the sensor rate and allows the PID loop to respond faster than the sensor can provide data. 

## Task 2: PID Results

A PID controller combines proportional, integral, and derivative control terms to form the motor input u(t):

![equation](assets/lab5/equation.png)

where e(t) is the error between the current ToF distance and the 304mm target. 

A P controller is a good starting point, however it often experiences steady-state error. A PI controller can fix this by pushing the motors harder until the error is eliminated. A PID controller improves further by damping the motor input in response to quick rapid changes in sensor readings, like when the car closes in on the wall quickly. I built up the controller incrementally, starting from P and adding terms one by one. 

### P Control
I started with the proportional term alone. The error is the difference between the current ToF reading and the 304mm setpoint. With `Kp = 0.07`, an error of ~1000mm produces an utput of 70, which is well within the 55-150PWM operating range, while an error of 50mm produces an output of 3.5, which falls below the motor threhold and causes the car to stop. This gives a natural deadband behavior near the target. 

````
float computePID(float current_dist) {
    unsigned long now = millis();
    float dt = (now - pid_prev_time) / 1000.0;
    if (dt <= 0) dt = 0.001;
    pid_prev_time = now;

    float error = current_dist - pid_pos_target;
    pid_prev_error = error;

    return (pid_Kp * error);
}
````
![P control](assets/lab5/Pcontrol.png)

The car approaches and settles near 304mm with one overshoot, but a small steady-state error persists. 

### PI Control 
To eliminate the steady-state error, I added the integral term. The integral accumulates error over time and provides a growing corrective push even when the proportional output has become very small. I kept track of `dt` between PID iterations to properly integrate

````
float error = current_dist - pid_pos_target;
pid_integral += error * dt;

float I_term = pid_Ki * pid_integral;

return (pid_Kp * error) + I_term;
````
![PI control](assets/lab5/PIcontrol.png)

## Task 3: PID Loop Rate: How fast is the PID control loop running? Compare this rate to ToF sensor rate.

## Task 4: Distance Extrapolation 

## Task 5: PID Control

## Task 6: Speed

## (5000) Wind-Up Protection


# Discussion

# References
