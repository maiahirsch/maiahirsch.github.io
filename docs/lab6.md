# Prelab

# Lab Tasks

As done in lab 5, I will be using a PID controller. A PID controller combines proportional, integral, and derivative control terms to form the motor input u(t):
![equation](assets/lab5/equation.png)

but in this case, e(t) is the error between the angular difference between the current orientation and the desired target orientation. 

## Task 1: PID Input Signal

**You should integrate your gyroscope to get an estimate for the orientation of the robot.**
To begin this week's lab, I needed to integrate readings from the IMU's gyroscope in order to estimate the orientation of my robot around its z-axis (yaw).

**Are there any problems that digital integration might lead to over time? Are there ways to minimize these problems?**

Integrating the gyroscope to estimate yaw works by accumulating `yaw += gyrZ * dt` each timestep. The main problem with this approach is gyroscope bias — even when stationary, the sensor outputs a small nonzero value that gets integrated continuously, causing the yaw estimate to drift over time.

To address this, the ICM-20948's onboard Digital Motion Processor (DMP) was used instead of raw gyro integration. The DMP fuses accelerometer and gyroscope data onboard to produce a quaternion-based orientation estimate with significantly reduced drift, which is then converted to a yaw angle in degrees for the PID input.

**Does your sensor have any bias, and are there ways to fix this? How fast does your error grow as a result of this bias? Consider using the onboard digital motion processor (DMP) built into your IMU to minimize yaw drift.** 

The ICM-20948 gyroscope does exhibit bias — a small nonzero output even when the robot is stationary. This bias directly sets the rate of yaw drift: at a typical bias of ~0.3 °/s, the estimate drifts roughly 3° over 10 seconds, which is significant for a tight orientation controller. One way to correct for this manually is to sample the gyro at startup while the robot is still, average the readings, and subtract that constant offset from every future measurement. However, the cleaner solution is to use the IMU's onboard DMP, which performs continuous bias calibration automatically as part of its sensor fusion algorithm, keeping drift minimal without any manual correction.

![DMP Setup](assets/lab6/dmpsetup.png)

![Fixed DMP Setup](assets/lab6/fixeddmpsetup.png)

**Are there limitations on the sensor itself to be aware of? What is the maximum rotational velocity that the gyroscope can read (look at spec sheets and code documentation on github). Is this sufficient for our applications, and is there was to configure this parameter?**

The ICM-20948 gyroscope has a maximum measurable rotational velocity of 2000 dps (degrees per second), as specified in the datasheet. This is configurable via the `GYRO_FR_FSEL` register, and when the DMP is enabled, the SparkFun library sets the full-scale range to 2000 dps by default. For this application, 2000 dps is more than sufficient — even during aggressive in-place spins, the robot is unlikely to exceed a few hundred degrees per second, leaving a large margin before the sensor saturates.

## Task 2: Derivative Term

**Does it make sense to take the derivative of a signal that is the integral of another signal.**

Yes, it actually makes sense in this case. The PID input is yaw angle, which is the integral of angular velocity (from the gyro). Taking the derivative of yaw recovers angular velocity — a clean, physically meaningful signal. So the D term is effectively just feeding back angular velocity, which is exactly what we want to dampen oscillations during a spin.

**Think about derivative kick. Does changing your setpoint while the robot is running cause problems with your implementation of the PID controller?**

Derivative kick is a concern when the setpoint is changed while the controller is running. In a naive implementation where the derivative is computed as the change in error, a sudden setpoint change causes an instantaneous jump in error, which when divided by a small dt produces a large derivative spike that kicks the motors hard. To avoid this, the derivative term should be computed on the yaw measurement directly rather than on the error, `d(yaw)/dt instead` of `d(error)/dt`. Since the yaw signal only changes when the robot actually moves, updating the setpoint mid-run only affects the P and I terms, and the D term remains smooth.

**Is a lowpass filter needed before your derivative term?**

Since the DMP already performs onboard sensor fusion, the yaw signal it outputs is significantly smoother than a raw gyro integration or ToF reading. As a result, a lowpass filter on the derivative term is less critical than in Lab 5. However, I still included it in `computeOrientPID()` as it helps reduce any residual noise in the derivative signal and prevents occasional spikes that would cause sudden motor commands: 

```c
  orient_d_filtered = D_LPF_ALPHA * derivative + (1.0 - D_LPF_ALPHA) * orient_d_filtered;
```
Where `D_LPF_ALPHA` = 0.015. 

## Task 3: Programming Implementation

The yaw signal from the DMP was sampled at approximately 20Hz (one reading every ~50ms), measured by logging timestamps whenever get_yaw() returned true. This rate is bottlenecked by the DMP's FIFO output rate, set to ~26Hz via setDMPODRrate(DMP_ODR_Reg_Quat6, 2). The usable range of the yaw signal is -180° to +180°.


**Have you implemented your code in such a way that you can continue sending an processing Bluetooth commands while your controller is running**
This is essential for being able to tune the PID gains quickly.
This is also essential for being able to change the setpoint while the robot is running.

**Think about future applications of your PID controller with regards to navigation or stunts. Will you need to be able to update the setpoint in real time?**

**Can you control the orientation while the robot is driving forward or backward? This is not required for this lab, but consider how this might be implemented in the future and what steps you can take now to make adding this functionality simple.**

Include graphs of all appropriate measurements needed to debug your PID controller. Below is an example the set point, angle and motor offset plotted as a function of time. Observe the overshoot and settling time of the angle and the response of the motor values.

## 5000-level Task: Integrator Windup

Implement wind-up protection for your integrator. Argue for why this is necessary (you may for example demonstrate how your controller works reasonably independent of floor surface).

Integrator wind-up occurs when the integral term accumulates unboundedly during periods where the controller output is saturated — for example, when the robot is far from its setpoint and the motors are already running at maximum speed. During this time, the error continues to be integrated even though the motor output cannot increase further. When the robot finally approaches the setpoint, the wound-up integrator produces a large overshoot because it takes time to "unwind" the accumulated integral before the output drops back into a reasonable range.
This is particularly problematic for orientation control on different floor surfaces. On a high-friction surface like carpet, the robot may be unable to move at all for several seconds while the error is large, causing the integrator to wind up significantly. When the robot is then placed on a low-friction surface, the same wound-up integrator would drive the motors far past the setpoint before the integral decays.
To prevent this, the integral term is clamped to ±200 on every timestep:
cpporient_integral = constrain(orient_integral, -ORIENT_INTEGRAL_MAX, ORIENT_INTEGRAL_MAX);
This ensures the integrator's contribution to the motor output never exceeds a bounded value regardless of how long the robot has been away from its setpoint, making the controller's behavior more consistent across different surfaces and starting conditions.
