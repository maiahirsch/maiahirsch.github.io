# Lab 9: Mapping

## Objective 

The purpose of this lab is to map out the little world built in the lab lab using ToF distance readings collected from four marked positions of the world. The robot spins in place at each position while recording distances, and the resulting readings are merged using transformation matrices to produce a line-based map for use in a simulation and future labs. 

## Control 

### Orientation control

I chose orientation PID control using the DMP chip for yaw estimation, the same controller I tuned in Lab 6. I collect 24 orientation data points in 15° increments, completing a full 360° rotation. At each step, the orientation PID runs until the yaw error falls within 4° and remains there for 500ms. Once settled, 5 ToF readings are collected from each sensor and averaged before mocing to the next setpoint. 

```c


```

#### Tuning the PID orientation controller

My orientation PID gains are: 
- Kp = 3.5
- Ki = 0.0
- Kd = 0.3


#### Graphs to document PID controller works well + upload video showing robot turning on axis 

The controller is the same one I used in lab 6. 



<iframe width="560" height="315" src="https://www.youtube.com/embed/4kSM23w-bDc?si=N8ZT9cjNbkfqguF-" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

#### Given any potential drift in your sensor, the size and accuracy of your increments, and how reliably your robot turns on axis, reason about the average and maximum error of your map if you were to do an on-axis turn in the middle of a 4x4m square, empty room.

## Read out Distances

### 1. execute each turn on marked positions. more locations to improve map. 
Consider whether your robot behavior is reliable enough to assume that the readings are spaced equally in angular space, or if you are better off trusting the orientation from gyroscope values.

I executed the mapping scan at each of the four marked positions: (-3,-2), (5,3), (0,3), and (5,-3). The robot was placed at each mark facing the same direction. 

### 2. Sanity check individual turns by plotting them in polar coordinate plot. Do the measurement match up what you expect? 

### 3. rotate twice or more to see how precise the scans are. 

## Merge and Plot your readings 

### 1. compute transformation matrices. describe matrices

### 2. plot all of your tof sensor readings in a single plot. assign different colors to data sets acquired from each turn. 

## Convert to Line-Based Map

### manually estimate where the actual walls/obstacles are based on your scatter plot. draw lines on top of these, and save two lists containing the end points of these lines: (x_start, y_start) and (x_end, y_end)

Feel free to correct slight errors found discovered during post processing in this step, but be sure to explain what caused them and how/why you correct them.






