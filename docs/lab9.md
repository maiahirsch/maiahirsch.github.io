# Lab 9: Mapping

## Objective 

The purpose of this lab is to map out the little world built in the lab lab using ToF distance readings collected from four marked positions of the world. The robot spins in place at each position while recording distances, and the resulting readings are merged using transformation matrices to produce a line-based map for use in a simulation and future labs. 

## Control 

### Orientation control

I chose orientation PID control using the DMP chip for yaw estimation, the same controller I tuned in Lab 6. I collect 36 orientation data points in 10° increments, completing a full 360° rotation. At each step, the orientation PID runs until the yaw error falls within 4° and remains there for 500ms. Once settled, 5 ToF readings are collected from each sensor and averaged before mocing to the next setpoint. 

```c
void run_mapping_scan(float start_yaw) {

  Serial.print("run_mapping_scan start_yaw=");
  Serial.println(start_yaw);
  map_data_index = 0;
  distanceSensor1.startRanging();
  distanceSensor2.startRanging();
  delay(200);

  for (int step = 0; step < MAPPING_STEPS; step++) {
    // 1. Compute target yaw 
    float target = start_yaw + step * MAPPING_INCREMENT;
    while (target > 180.0) target -= 360.0;
    while (target < -180.0) target += 360.0;

    // 2. Run orientation PID until settled 
    orient_setpoint = target;
    orient_integral = 0.0;
    orient_d_filtered = 0.0;
    orient_prev_time = millis();
    orient_prev_yaw = global_yaw;

    unsigned long step_start = millis();
    unsigned long settled_at = 0;
    bool settled = false;

    while (millis() - step_start < 3000) {
      if (get_yaw()) {
        float output = computeOrientPID(global_yaw);
        output = constrain(output, -200, 200);

        float err = global_yaw - orient_setpoint;
        while (err > 180.0) err -= 360.0;
        while (err < -180.0) err += 360.0;

        if (abs(err) < MAPPING_DEADBAND) {
          motorsStop();
          if (!settled) {
            settled = true;
            settled_at = millis();
          }
          if (millis() - settled_at > MAPPING_SETTLE_MS) break;
        } else {
          settled = false;
          motorsOrient((int)-output);
        }
      }
    }
    motorsStop();
    delay(50);

    // 3. Record actual yaw 
    get_yaw();
    map_yaw_data[map_data_index] = global_yaw;

    // 4. Average MAPPING_READINGS from each sensor
    long sum1 = 0, sum2 = 0;
    int cnt1 = 0, cnt2 = 0;
    unsigned long tof_start = millis();

    while ((cnt1 < MAPPING_READINGS || cnt2 < MAPPING_READINGS)
           && millis() - tof_start < 1000) {
      if (cnt1 < MAPPING_READINGS && distanceSensor1.checkForDataReady()) {
        sum1 += distanceSensor1.getDistance();
        distanceSensor1.clearInterrupt();
        cnt1++;
      }
      if (cnt2 < MAPPING_READINGS && distanceSensor2.checkForDataReady()) {
        sum2 += distanceSensor2.getDistance();
        distanceSensor2.clearInterrupt();
        cnt2++;
      }
    }

```

#### Tuning the PID orientation controller

My orientation PID gains are: 
- Kp = 3.5
- Ki = 0.0
- Kd = 0.3

The controller is the same one I used in lab 6. The plot blow shows the yaw tracking a 90° setpoint cleanly, confirming the controller is well-tuned. 

![setpoint](assets/lab6/setpoint.png)

<iframe width="560" height="315" src="https://www.youtube.com/embed/4kSM23w-bDc?si=N8ZT9cjNbkfqguF-" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

#### Error Analysis

From Serial Monitor output, the per-step yaw error was consistently within 1–3°. In the center of a 4×4m empty room, walls are 2m away in every direction. A 3° angular error at 2m produces a positional error of 2 × sin(3°) ≈ 105mm at the wall. On average with ~1.5° error this is closer to 52mm. The robot also exhibited slight translational drift (~50mm) during full scans due to imperfect on-axis spinning, giving a combined maximum map error of approximately 150mm.

## Read out Distances

I executed the mapping scan at each of the four marked positions: (-3,-2), (5,3), (0,3), and (5,-3).

![polar sanoty check](assets/lab9/polarsanitycheck.png)

The polar plot shows reasonable shaped for each position, with distances that reflect what we expect from room geometry at each location. Here it is clear that sensor 1 is pointing at the floor. 

![consistency check](assets/lab9/consistencycheck.png)

I performed two full rotations on my desk to assess scan repeatability. The front sensor (blue) shows good consistency between the two rotations — the large spike toward 45°–60° (pointing toward the open room interior) and the readings toward 0° and 315° (pointing toward the right and bottom walls) appear in both rotations at similar distances. The orange (right sensor) forms a consistent small ring close to the robot in both rotations, which as discussed above is due to the sensor pointing downward and hitting the floor rather than the walls. The front sensor shape is reproducible enough to trust for mapping purposes, with the main variation occurring at angles where the sensor hits open space or the robot's chassis — both of which produce inherently noisy readings that are filtered out in post-processing.

## Merge and Plot your readings 

### Transformation Matrices

To convert each ToF reading from the robot's local frame into the global room frame, I applied a 3×3 homogeneous transformation matrix at each angular step:

$$T(\theta, x_r, y_r) = \begin{bmatrix} \cos\theta & -\sin\theta & x_r \\ \sin\theta & \cos\theta & y_r \\ 0 & 0 & 1 \end{bmatrix}$$

where $\theta$ is the robot's DMP yaw at that step and $(x_r, y_r)$ is the robot's known position in the room in mm.

Sensor 2 (front-facing) has its measurement along the robot's $+x$ axis:

$$P_{room} = T \cdot \begin{bmatrix} d_2 + \text{dist} \\ 0 \\ 1 \end{bmatrix}$$

Sensor 1 (right-facing) has its measurement along the robot's $+y$ axis:

$$P_{room} = T \cdot \begin{bmatrix} 0 \\ d_1 + \text{dist} \\ 1 \end{bmatrix}$$

where $d_2 = 90$ mm (sensor 2 forward offset) and $d_1 = 40$ mm (sensor 1 rightward offset).


### A Note on Sensor 1

During testing, sensor 1 (right-side mounted) produced readings that formed a tight circular ring around the robot position rather than reaching out to the walls. After investigation, this was found to be caused by the sensor's physical mounting angle — sensor 1 was pointing slightly downward toward the floor rather than perfectly horizontal. As the robot rotated, the sensor consistently hit the floor at a short, nearly constant distance regardless of heading, producing the characteristic radius-around-robot pattern visible in the right-side scatter plot. This is a hardware mounting issue, not a software or calibration problem. Because of this, sensor 1's data is less reliable for wall detection and the final map is primarily informed by sensor 2 (front-facing), which had clear unobstructed line of sight to the walls in most directions.

### Angle Corrections
After plotting the combined scatter, I found that two of the four scan positions required a 180° angular correction to align their data with the room's coordinate frame. Specifically, the scans at (0, 3) and (5, -3) required a 180° correction, while (-3, -2) and (5, 3) required none. Since all four scans were performed within the same BLE session — meaning the DMP never re-initialized and its yaw reference remained consistent throughout — the offset is most likely due to a slight inconsistency in how the robot was placed at those two positions. Despite my best efforts to orient the robot the same way at each mark, the 180° discrepancy suggests the robot was inadvertently placed facing the opposite direction at (0, 3) and (5, -3). The correction was applied as a post-processing angular offset added to all yaw readings from those positions, rotating the entire dataset by 180° to realign it with the room's coordinate system. The resulting point clusters from all four positions align consistently with the known wall geometry, confirming the corrections are appropriate.

![front vs right](assets/lab9/frontvsright.png)

## Convert to Line-Based Map

I manually drew estimated wall segments through the point clusters in the scatter plot, ignoring outliers caused by the sensor pointing toward open space or the robot chassis at certain angles. The estimated edges were shifted slightly inward from the true edges in some areas, reflecting the tendency of the front sensor to read slightly long when hitting walls at oblique angles.

```python
# ─────────────────────────────────────────────────────────────────────────────
# TRUE EDGES 
# ─────────────────────────────────────────────────────────────────────────────
true_walls_ft = np.array([
    # Outer boundary (L-shaped room)
    [-6.5, -4.5,  6.5, -4.5],   # bottom
    [ 6.5, -4.5,  6.5,  4.5],   # right
    [ 6.5,  4.5, -2.0,  4.5],   # top right
    [-2.0,  4.5, -2.0,  0.5],   # step down
    [-2.0,  0.5, -6.5,  0.5],   # top left
    [-6.5,  0.5, -6.5, -4.5],   # left

    # Interior box obstacle
    [ 2.5,  0.5,  2.5,  2.5],   # left side of box
    [ 2.5,  2.5,  4.5,  2.5],   # top of box
    [ 4.5,  2.5,  4.5,  0.5],   # right side of box
    [ 4.5,  0.5,  2.5,  0.5],   # bottom of box

    # Bottom center obstacle
    [-1.0, -4.5, -1.0, -2.5],   # left side
    [-1.0, -2.5,  1.0, -2.5],   # top
    [ 1.0, -2.5,  1.0, -4.5],   # right side
])

# ─────────────────────────────────────────────────────────────────────────────
# ESTIMATED EDGES — draw these based on where your scatter points cluster
# ─────────────────────────────────────────────────────────────────────────────
estimated_walls_ft = np.array([
    # Outer boundary
    [-5.5, -4.5,  6.5, -4.5],   # bottom
    [ 6.5, -4.5,  6.5,  4.5],   # right
    [ 6.5,  4.5, -1.5,  5.0],   # top right
    [-1.5,  5.0, -1.5,  0.5],   # step down
    [-1.5,  0.5, -5.5,  0.5],   # top left
    [-5.5,  0.5, -5.5, -4.5],   # left

    # Interior box obstacle
    [ 2.0,  0.5,  2.0,  2.5],
    [ 2.0,  2.5,  4.0,  2.5],
    [ 4.0,  2.5,  4.0,  0.5],
    [ 4.0,  0.5,  2.0,  0.5],

    # Bottom center obstacle
    [ 0.0, -4.5,  0.0, -2.5],
    [ 0.0, -2.5,  2.0, -2.5],
    [ 2.0, -2.5,  2.0, -4.5],
])
```
![estimated path](assets/lab9/estimatedpath.png)

Feel free to correct slight errors found discovered during post processing in this step, but be sure to explain what caused them and how/why you correct them.

```c
starts = [(-1981.2, -1371.6), (1981.2, -1371.6), (1981.2, 1371.6), (-609.6, 1371.6), (-609.6, 152.4), (-1981.2, 152.4), (762.0, 152.4), (762.0, 762.0), (1371.6, 762.0), (1371.6, 152.4), (-304.8, -1371.6), (-304.8, -762.0), (304.8, -762.0)]
ends   = [(1981.2, -1371.6), (1981.2, 1371.6), (-609.6, 1371.6), (-609.6, 152.4), (-1981.2, 152.4), (-1981.2, -1371.6), (762.0, 762.0), (1371.6, 762.0), (1371.6, 152.4), (762.0, 152.4), (-304.8, -762.0), (304.8, -762.0), (304.8, -1371.6)]

```

### Wall Offset Due to Drift

Some of the estimated wall positions are slightly shifted from the true edges — most noticeably the left wall and parts of the top boundary. This is caused by translational drift during the mapping scan. Despite using orientation PID to control heading, the robot does not spin perfectly on-axis — each incremental turn causes a small sideways displacement of the robot's physical center. Over the course of a full 36-step scan, these small displacements accumulate, meaning the robot's actual position at the time of each reading is slightly different from the marked position it started at. Since the transformation assumes the robot remains stationary at its starting coordinates throughout the entire scan, any translational drift introduces a systematic offset in the transformed wall positions. The direction and magnitude of the shift depends on which way the robot drifted — typically 1–3 inches over a full rotation. Rather than attempting to model or correct for this drift algorithmically, I manually adjusted the estimated edge positions during post-processing to best fit the visible point clusters, accepting that the estimated map will have small but bounded positional errors on the order of 50–80mm.

### Outliers Beyond Room Boundary

Several scatter points land outside the known room boundary, particularly from scan positions that are far from certain walls. This is likely caused by sensor 2 being mounted at a slight upward angle. When the sensor points toward a distant wall, the upward tilt means the beam grazes above the wall rather than hitting it squarely, returning either a very long reading or missing the wall entirely and hitting a distant ceiling fixture or nothing at all. This effect is most pronounced at scan positions far from the wall being measured — for example, readings from (0,3) pointing toward the top wall.

## Discussion 

This lab successfully produced a line-based map of the lab space using orientation-PID-controlled incremental scanning. The main challenges were sensor 1's downward mounting angle — which caused it to read the floor instead of walls, producing a circular ring pattern in the scatter plot — and translational drift during spins, which shifted some wall clusters 50–80mm from their true positions and required manual correction in post-processing. In future iterations, verifying sensor mounting angle before scanning and marking a reference direction at each scan position would eliminate both issues entirely.

## References
I used Stephan Wagner's page for guidance on a couple of plots and the manual tracing. I attended office hours and Lucca was extremely helpful helping me realize the sensor facing downward problem. I gave my car to Hayden Webb for a day so he could run his code. Claude helped me make very pretty plots. 


