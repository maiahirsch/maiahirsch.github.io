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
This ensures the car stops if the Bluetooth connection drops mid-run

# Lab Tasks

## P/I/D discussion (Kp/Ki/Kd values chosen, why you chose a combination of controllers, etc.)
## Range/Sampling time discussion
## Graphs, code, videos, images, discussion of reaching task goal
## Graph data should include Tof vs time and Motor input vs time (and whatever helps with debugging)
## (5000) Wind-up implementation and discussion

# Discussion

# References
