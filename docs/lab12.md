# Lab 12: Inverted Pendulum

## Objective

For this final lab, I decided to pursue the inverted pendulum challenge using closed-loop PID control. This problem requires fast, accurate angle estimation and a quick response from the motors. 

## Approach Overview

The inverted pendulum is an unstable system. Any small perturbation from the upright position will cause the car to fall unless the controller actively compensates by driving the wheels in the direction of the fall, like a Segway or hoverboard. My implementation uses a PD controller running on the Artemis, reading pitch anfle from the ICM-20948 DMP, and driving the motors to maintain balance. 

## IMU Angle Detection

The first step was to figure out which IMU axis and what angle reading corresponded to the upright position. I added a debug print to Serial to output roll, pitch, and yaw while manually tilting the car. 

I found the following: 
- Flat on the ground: pitch ~ 0°
- Upright on rear wheels: pitch ~ -83°
- Tipping forward: pitch < -83° (more negative)
- Tipping forward: pitch > 83° (more positive)

This told me pitch was the correct axis, and that I needed to shift the reference so uright reads as 0°. I applied a fixed offset 

```c
float angle = global_pitch + 83.0;
```

With this correction: 
- Upright = 0°
- Tipping forward = positive error
- Tipping backward = negative error 

At some point, I think the IMU moved a little because when I the perfectly upright position, the angle read -1.4° rather than 0°. I decided to tune this setpoint parameter whenever necessary by sending a python command rather than hardcoding it. 
```python
ble.send_command(CMD.START_PENDULUM, "-3.5")  # lean setpoint 3.5° forward
```

## Controller Design

I implemented a PD controller. I ommited Ki on purpose as for an inverted pendulum the integral term accumulates error over time and can cause wind-up and instability making the tuning much harded. 

```c
float angle = global_pitch + 83.0;
float error = angle - pen_setpoint;

float P_term = pen_Kp * error;
float raw_D = pen_Kd * (error - pen_prev_error) / dt;
pen_d_filtered = PEN_D_LPF_ALPHA * raw_D + (1.0 - PEN_D_LPF_ALPHA) * pen_d_filtered;

float output = P_term + pen_d_filtered;
```

The D term is filtered with a low-pass filter (\alpha = 0.1) to reduce noise amplification from the derivative calculation. A safety cutoff stops the motors if the angle exceeds ±30° from vertical, since recovery is not possible beyond that point. 

Motor direction mapping: 
```c
if (output > 0) {
    motorsBackward(constrain((int)abs(output), 85, 255));
} else if (output < 0) {
    motorsForward(constrain((int)abs(output), 85, 255));
}
```

## The DMP Speed Problem

Early attempts showed the car responding to ti[[ing but never catching up with the fall. I first assumed that this was a gains problem and spent significant time tuning Kp and Kd across a wide range. Videos of these attempts are included below. No matter how aggressively I tuned, the car should just fall before the motors could response in time. 

The real issue turned out to be the DMP output rate. My initialize_DMP() had:

```c
myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 2);  // ~26Hz
```

At 26Hz, the controller only received a new angle reading every ~38ms. For a fast-falling inverted pendulum, this is far too slow — the car can tip several degrees between updates. I changed the rate setting to 0, which runs the DMP at its maximum rate of ~225Hz:

```c
myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0);  // ~225Hz
```

This change made an immediate dramatic difference. Now the car could stay upright for a few seconds. 

To improve speed, I also added a `continue` statement in the main loop to skip all non-pendulum processing (ToF, drift, orientation PID) while the pendulum was balancing. 

```c
if (flag_pid_pendulum && dmp_ready) {
    if (get_yaw()) {
        pidPendulum();
    }
    continue;
}
```

## Gain Tuning

All gains were hand-tuned iteratively using BLE commands from Python so I could change gains without reflashing:

```python
ble.send_command(CMD.SET_PENDULUM_GAINS, "4.5|0.5")
ble.send_command(CMD.START_PENDULUM, "-1.4")
```

Although more gain combinations were tested, for the purpose of this writeup, here is a chart that outlines 3 combinations, each shown in a video below: 

