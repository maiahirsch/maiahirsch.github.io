# Lab 2

## Prelab

Read and got familiar with the IMU we will be using. The IMU was connected to the Artemis board using the QWIIC connector. I installed the ICM-20948 library. 



## Lab Tasks

### Setup the IMU 

I installed the “SparkFun 9DOF IMU Breakout_ICM 20948_Arduino Library” during lecture.

I ran the “..\Arduino\libraries\SparkFun_ICM-20948\SparkFun_ICM-20948_ArduinoLibrary-master\examples\Arduino\Example1_Basics”. 

[INSERT VIDEO]

From the [data sheet](https://learn.sparkfun.com/tutorials/sparkfun-9dof-imu-icm-20948-breakout-hookup-guide/all) of the IMU, AD0_VAL is the value of the last bit of the I2C address. The default is 1, and when the ADR jumper is closed the value becomes 0. 

I added a blink code to see when the board is running. 

[LAB2BLINK](assets/lab2/lab2blinkcode.png)


### Accelerometer 

I included the math.h library in Arduino. Then using the class slide, I added this conversion 

[[INSERT LAB2CLASSSLIDE]]

I kept getting similar values for pitch and roll. So I decided to rewrite my roll formula as. 

[[INSERT ROLL PITCH CODE]]
[[INSERT ACCELEROMETER VIDEO]]
[[INSERT ROLL PITCH VIDEO]]

The accelerometer is quite accuracy if it is held steady at an angle. But still the sensor readings keep changing. A fourier transform will assist with filtering out the noise from the sensor readings.

I followed the Fourier transform for Python tutorial provided. Still confused about implementation I asked Chat for advise. 

I added Serial.print(millis()); to my loop, so I could have a time stamp for each reading to plot. 

### The Stunt!



## Discussion 

## References


