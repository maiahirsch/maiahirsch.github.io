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

| Kp   | Kd   | result |
| ---  | ---- | ---- |
| 1.5  | 0.1  |Too slow — car falls backward consistently|
| 2.0  | 0.2  |Too slow — car falls backward consistently|
| 3.0  | 0.3  |Oscillates — overcorrects in both directions|
| 4.5  | 0.05 |Too slow — car falls backward consistently|
| 4.5  | 0.5  |Best — car stays upright up to 7 seconds|
| 6.0  | 0.5  |Too slow — car falls backward consistently|

The general pattern I observed: too low Kp meant the motors were too slow to catch up. Too high Kp caused too many oscillations and the car overcorrected back and fourth. Kd helped damp these oscillations, but if Kd was too high the car also did not have time to catch up. 

### Results 

**Video 1 — Kp=1.5, Kd=0.1:** Car responds but falls consistently backward. Not enough proportional gain to overcome gravity.

<iframe width="560" height="315" src="https://www.youtube.com/embed/jbLX7fHOpy0?si=65y1vA2DDOOyZ4U_" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

**Video 2 — Kp=2.0, Kd=0.2:** Visible oscillation — car rocks back and forth before falling. Getting closer but Kd needs more tuning.

<iframe width="560" height="315" src="https://www.youtube.com/embed/_APt6M0QBYo?si=q_GceCyUb_jTlNOL" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

**Video 3 — Kp=3.0, Kd=0.3:** Visible oscillation — car rocks back and forth before falling. Getting closer but Kd needs more tuning.

<iframe width="560" height="315" src="https://www.youtube.com/embed/h_yLxKQAwrM?si=bxYdYt8vFkZ1_pn8" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

**Video 4 — Kp=4.5, Kd=0.05:**

<iframe width="560" height="315" src="https://www.youtube.com/embed/OZajaiyc-mY?si=uCzROWjwydy_Re38" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

**Video 5 — Kp=4.5, Kd=0.5:** Best result. Car maintains balance for up to 7 seconds before drifting off. I was very happy when it worked after days of debugging and trial and error. 

<iframe width="560" height="315" src="https://www.youtube.com/embed/h1Ry-R3_PE8?si=2UXt4eG-NvjaH90w" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

**Video 6 — Kp=6.0, Kd=0.5:**

<iframe width="560" height="315" src="https://www.youtube.com/embed/MYkF0nMDdws?si=-4UeRElp49V2-UGC" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

**Reaction video of the first time it worked** 📍 Olin Library

<iframe width="560" height="315" src="https://www.youtube.com/embed/zcUG7D7CMiU?si=wwl0-WXM_jUMBLtT" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Failed attempts when DMP was too slow




## Conclusion

The inverted pendulum challenge required solving two main problems: getting the angle measurement right, and making the control loop fast enough. Determining the offset gave the controller an accurate error signal. Increasing the DMP output rate from 26Hz to 225 Hz was the single most impactful change. The final PD controller with Kp = 4.5 and Kd = 0.5 achieves up to 7 seconds of sustained balance, which I'm quite happy about. 
