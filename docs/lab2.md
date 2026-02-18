# Lab 2

## Prelab

Read and got familiar with the IMU we will be using. The IMU was connected to the Artemis board using the QWIIC connector. I installed the ICM-20948 library. 



## Lab Tasks

### Setup the IMU 

I installed the “SparkFun 9DOF IMU Breakout_ICM 20948_Arduino Library” during lecture.

I ran the “..\Arduino\libraries\SparkFun_ICM-20948\SparkFun_ICM-20948_ArduinoLibrary-master\examples\Arduino\Example1_Basics”. 

[INSERT VIDEO]

From the [data sheet](https://learn.sparkfun.com/tutorials/sparkfun-9dof-imu-icm-20948-breakout-hookup-guide/all) of the IMU, AD0_VAL is the value of the last bit of the I2C address. The default is 1, and when the ADR jumper is closed the value becomes 0. 

When rotating the board, the accelerometer showed gravity shifting between the axes. I added a blink code to see when the board is running. 

[LAB2BLINK](assets/lab2/lab2blinkcode.png)

### Accelerometer 

I included the math.h library in Arduino. Then using the class slide, I added this conversion 

[LAB2CLASSSLIDE](assets/lab2/lab2classslide.png)

[ROLLPITCHFORMULAS)(assets/lab2/rollpitchformulas.png)

[video -90 0 90] 

The accelerometer is quite accuracy if it is held steady at an angle. But still the sensor readings keep changing. A fourier transform will assist with filtering out the noise from the sensor readings.

I followed the Fourier transform for Python tutorial provided. 

To analize noise, I collected accelerometer data keeping the IMU still and tapping on the table to induce vobration. Shown below is the FFT signal for both cases. 

FFT](assets/lab2/FFT.png)

The FFT shows that noise is very low in both cases as the ICM-20948 already has a low pass filter on board. Even under vibration there is no significant high frequency spikes. A cuttoff frequency of 5 Hz was chosen for the low pass filter. 

The low-pass filter implemented was as follows: 

[LPF](assets/lab2/lowpassfilter1.png)
[LPF](assets/lab2/lowpassfilter2.png)

[raw vs filtered](assets/lab2/rawvsfiltered.png)
### The Stunt!



## Discussion 

## References


