# Prelab

# Lab Tasks

## Task 1: PID Input Signal

You should integrate your gyroscope to get an estimate for the orientation of the robot.
Are there any problems that digital integration might lead to over time? Are there ways to minimize these problems?
Does your sensor have any bias, and are there ways to fix this? How fast does your error grow as a result of this bias? Consider using the onboard digital motion processor (DMP) built into your IMU to minimize yaw drift.
Are there limitations on the sensor itself to be aware of? What is the maximum rotational velocity that the gyroscope can read (look at spec sheets and code documentation on github). Is this sufficient for our applications, and is there was to configure this parameter?

## Task 2: Derivative Term

Does it make sense to take the derivative of a signal that is the integral of another signal.
Think about derivative kick. Does changing your setpoint while the robot is running cause problems with your implementation of the PID controller?
Is a lowpass filter needed before your derivative term?

## Task 3: Programming Implementation

Have you implemented your code in such a way that you can continue sending an processing Bluetooth commands while your controller is running?
This is essential for being able to tune the PID gains quickly.
This is also essential for being able to change the setpoint while the robot is running.
Think about future applications of your PID controller with regards to navigation or stunts. Will you need to be able to update the setpoint in real time?
Can you control the orientation while the robot is driving forward or backward? This is not required for this lab, but consider how this might be implemented in the future and what steps you can take now to make adding this functionality simple.

Include graphs of all appropriate measurements needed to debug your PID controller. Below is an example the set point, angle and motor offset plotted as a function of time. Observe the overshoot and settling time of the angle and the response of the motor values.

## 5000-level Task

Implement wind-up protection for your integrator. Argue for why this is necessary (you may for example demonstrate how your controller works reasonably independent of floor surface).
