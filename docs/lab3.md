Write-up
Word limit: < 1000 words

This is not a strict requirement, but may be helpful in understanding what should be included in your webpage. It also helps with the flow of your report to show your understanding to the lab graders.

# Prelab

## I2C Address

The VL53L1X ToF sensor has a default 12C address of 0x29 (7-bit), which corresponds to  0 x 52 in the 8-bit addressing convention used in the datasheet. The Arduino Wire library uses 7-bit addressing, so **0x29** is the address I will use in the code. 

## Two Sensor Approach 

Both sensors share the same default a dress and this is problematic as they have to be modified to be used simultaneously. I used the XSHUT pin approach: on startup, the XSHUT pin of sensor 2 is pulled LOW to disable it, then sensor 1's address is changed to 0x30. Sensor 2 is then re-enabled via XSHUT and boots at the default 0x29. Both sensors can now be addressed independently on the same I2C bus at the same time. 

## Sensor Placement 

For the final robot, I plan to mount one sensor facing forward and one facing on the side. The forward sensor handles the primary obstacle detection for the stop-before-wall task. The side sensor helps detect lateral obstacles during turns. Obstacles will be missed if objects are very low to the ground, below the sensor's field of view. Objects approaching from behind and objects outside the angular sensitivity cone of each sensor will also be missed, although I do not expect any objects approaching from behind. 

## Wiring Diagram

![wiring](assets/lab3/lab3schematic.png)

Both sensors connect to the QWIIC breakout board via QWIIC cables (SDA, SCL, VCC, GND). Sensor 2 has an additional wire from its XSHIT pin to GPIO pin A0 on the Artemis for shutdown control. 

# Lab Tasks

## ToF sensor connected to QWIIC breakout board

![connectedQWIIC](assets/lab3/connectedQWIIC.png)
![soldered](assets/lab3/soldered.png)

## I2C Scan

![i2c](assets/lab3/i2c.png)

The I2C scan detected the ToF sensor at address 0x29, matching the expected 7-bit address. The IMU was also detected at 0x69. The seconf I2C port showed no devices since all sensors are connected to the same bus. 

## Distance Mode Selection 

I tested both Short and Long distance modes between 100mm and 1000mm, averaging 256 readings at each positions. The results are plotted below: 

![longvsshort](assets/lab3/longshort.png)

Interestingly, Long mode tracked the reference line more smoothly throughout the range, while Short mode showed irregularities, particularly a flat region around 500 - 600mm where readings plateaued before catching up. Despite this, I chose Short mode for the final robot for two reasons. 

1. Short mode's ranging time is nearly half that of Long mode (~56ms vs ~101ms), meaning the robot can sample twice as fast and this is critical for last-minute stops.
2. The robot only required reliable detection within ~1.3m, which is well within Short mode's specified range.

![4cases](assets/lab3/4cases.png)

## 2 ToF sensors and the IMU: Discussion and screenshot/video of sensors working in parallel

![dualTOF](assets/lab3/dualTOF.png)

Tof sensor speed: Discussion on speed and limiting factor; include code snippet of how you do this

Time v Distance: Include graph of data sent over bluetooth (2 sensors)

Time v Angle: Include graph of data sent over bluetooth

(5000) Discussion on infrared transmission based sensors

(5000) Sensitivity of sensors to colors and textures

Please also include code snippets (consider using GitHub Gists) in appropriate sections if you included any written code. Do not copy and paste all your code. Include only relevant functions used for each task.

Include screenshots of relevant results.
