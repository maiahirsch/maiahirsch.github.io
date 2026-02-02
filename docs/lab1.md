# Lab 1A

## Blink

<iframe width="560" height="315" src="https://www.youtube.com/embed/nM_cUpNXuFU?si=UiGdOjgL0qS2NQb6" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Serial

<iframe width="560" height="315" src="https://www.youtube.com/embed/dYNelFfFumU?si=Cj7t4ho5kk9GpRc9" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Temperature Sensor

<iframe width="560" height="315" src="https://www.youtube.com/embed/emRVZet1o7o?si=VkVn_TA0kB1KX25i" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Microphone

<iframe width="560" height="315" src="https://www.youtube.com/embed/gg5GWId_iys?si=lU0Z6rz9q5LlY1yW" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Electronic Tuner (5000-level)

# Lab 1B: 

## Prelab

## Task 1:Echo 

The <mark>ECHO</mark> command was used to verify that the communication between the computer and the board was functioning correctly. This command instructs the board to return the same string that it receives from the computer. To test this, I sent the string <mark>“HiHello”</mark> to the board. The board responded with the augmented message <mark>“Robot says → HiHello :)”</mark>, which I observed in my notebook, confirming that the string was correctly received and processed by the board. The returned string was also successfully received by the computer, verifying bidirectional communication.

![Lab 1B Task 1](assets/lab1btask1.png)
![Lab 1B Task 1](assets/lab1btask1code.png)

## Task 2: SEND_THREE_FLOATS

The <mark>SEND_THREE_FLOATS</mark> command instructs the board to transmit three floating-point values to the computer. This command was implemented as an extension of the existing <mark>SEND_TWO_INTS</mark> command that was already provided. I modified the original implementation to support floating-point values instead of integers. The three floating-point values are transmitted and printed to the serial monitor for verification.

![Lab 1B Task 2](assets/lab1btask2answer2.png)
![Lab 1B Task 2](assets/lab1btask2code.png)

## Task 3:GET_TIME_MILLIS

I added a new command, <mark>GET_TIME_MILLIS</mark>, that makes the robot respond with the current time (in milliseconds) formatted as a string.
In the C code, I added `GET_TIME_MILLIS` to the `CommandTypes` enum and implemented a new `case GET_TIME_MILLIS:` in the command handler. In the Python code, I added the corresponding command entry in `cmd_types.py` so I could send the command from my notebook.

I used `millis()` to get the current time since the board started running, converted it into a string with a `"T:"` prefix, and wrote it to the **string TX characteristic** (and also printed it to the serial monitor for debugging). On the computer side, I sent the command and verified that the received string matched the expected format.

![Lab 1B Task 3](assets/lab1btask3code.png)
![Lab 1B Task 3](assets/lab1btask3result.png)
![Lab 1B Task 3](assets/lab1btask3update.png)
![Lab 1B Task 3](assets/lab1btask3update2python.png)

I ran into a lot of errors with GET_TIME_MILLIS missing from CMD and fixed it by restarting the Kernel. 

Reference: https://akinfelami.github.io/fastrobots-2025/artemis-and-bluetooth and https://rga47-lab.github.io/lab1.html

## Task 4: Notification Handler 

To improve communication efficiency between the computer and the robot, I implemented a notification-based approach for receiving string data from the board. Instead of explicitly calling a receive function every time a response was needed, I set up a notification handler that automatically processes incoming data from the RX string characteristic.

On the Python side, I defined a custom notification handler that converts the received byte array into a string and parses the message. Since some responses include multiple tokens (e.g., sample index and timestamp), the handler checks the length of the parsed string before printing the appropriate output. I then enabled notifications on the RX string characteristic. 
![Lab 1B Task 4](assets/lab1btask4usingthis.png)

```c
def notification_handler(uuid, byte_array):
    s = ble.bytearray_to_string(byte_array)
    sarray = s.split(" ")
    if len(sarray) > 2:
        print(s)
    else:
        print(sarray[1])
```

Once notifications were enabled, I sent the GET_TIME_MILLIS command from the computer. The board responds by sending back the current timestamp (in milliseconds) formatted as a string (e.g., T:387094.000). This response is automatically received and printed by the notification handler.

![Lab 1B Task 4](assets/lab1btask4newntimestamps.png)
![Lab 1B Task 4](assets/lab1btask4result.png)

Reference: https://rga47-lab.github.io/lab1.html

## Task 5: GET_TIME_MILLIS_LOOP & Data Transfer Rate

To measure how fast the robot can stream data to my laptop using BLE notifications, I implemented a new command called **GET_TIME_MILLIS_LOOP**. This command continuously sends timestamp messages for a fixed duration of **3 seconds**. Each message contains a sample counter and the current time in milliseconds (from `millis()`), formatted as:

`Sample: X, T: Y`

```c
case GET_TIME_MILLIS_LOOP: {

    int sample = 0; 
    unsigned long startTime = millis();

    // create a loop to retrieve data for 3 seconds 
    while(millis() - startTime < 3000) {
        tx_estring_value.clear();
        tx_estring_value.append("Sample: ");
        tx_estring_value.append(sample);
        tx_estring_value.append(", T: ");
        tx_estring_value.append((float)millis());
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        sample++;
    }
    Serial.println("sent all");
    break;
}
```
On the laptop, I enabled notifications on the RX string characteristic and then sent the command:

```
ble.start_notify(ble.uuid['RX_STRING'], notification_handler)
ble.send_command(CMD.GET_TIME_MILLIS_LOOP, '')
```

The notification handler automatically printed each incoming message:
```c
Sample: 0, T: 40292
Sample: 1, T: 40292
Sample: 2, T: 40292
Sample: 3, T: 40293
Sample: 4, T: 40319
Sample: 5, T: 40355
Sample: 6, T: 40355
Sample: 7, T: 40386
Sample: 8, T: 40386
Sample: 9, T: 40412
Sample: 10, T: 40444
Sample: 11, T: 40444
Sample: 12, T: 40471
Sample: 13, T: 40471
Sample: 14, T: 40506
Sample: 15, T: 40531
Sample: 16, T: 40531
Sample: 17, T: 40564
Sample: 18, T: 40564
Sample: 19, T: 40591
Sample: 20, T: 40621
Sample: 21, T: 40621
Sample: 22, T: 40655
Sample: 23, T: 40655
Sample: 24, T: 40680
Sample: 25, T: 40710
Sample: 26, T: 40710
Sample: 27, T: 40743
Sample: 28, T: 40743
Sample: 29, T: 40771
Sample: 30, T: 40801
Sample: 31, T: 40801
Sample: 32, T: 40831
Sample: 33, T: 40831
Sample: 34, T: 40858
Sample: 35, T: 40889
Sample: 36, T: 40889
Sample: 37, T: 40924
Sample: 38, T: 40924
Sample: 39, T: 40981
Sample: 40, T: 41012
Sample: 41, T: 41012
Sample: 42, T: 41040
Sample: 43, T: 41040
Sample: 44, T: 41067
Sample: 45, T: 41098
Sample: 46, T: 41098
Sample: 47, T: 41133
Sample: 48, T: 41133
Sample: 49, T: 41161
Sample: 50, T: 41193
Sample: 51, T: 41193
Sample: 52, T: 41219
Sample: 53, T: 41219
Sample: 54, T: 41252
Sample: 55, T: 41280
Sample: 56, T: 41280
Sample: 57, T: 41312
Sample: 58, T: 41312
Sample: 59, T: 41337
Sample: 60, T: 41369
Sample: 61, T: 41369
Sample: 62, T: 41399
Sample: 63, T: 41399
Sample: 64, T: 41428
Sample: 65, T: 41464
Sample: 66, T: 41464
Sample: 67, T: 41493
Sample: 68, T: 41493
Sample: 69, T: 41521
Sample: 70, T: 41548
Sample: 71, T: 41548
Sample: 72, T: 41584
Sample: 73, T: 41584
Sample: 74, T: 41608
Sample: 75, T: 41640
Sample: 76, T: 41640
Sample: 77, T: 41666
Sample: 78, T: 41672
Sample: 79, T: 41700
Sample: 80, T: 41731
Sample: 81, T: 41731
Sample: 82, T: 41731
Sample: 83, T: 41756
Sample: 84, T: 41787
Sample: 85, T: 41817
Sample: 86, T: 41817
Sample: 87, T: 41852
Sample: 88, T: 41852
Sample: 89, T: 41877
Sample: 90, T: 41910
Sample: 91, T: 41910
Sample: 92, T: 41942
Sample: 93, T: 41942
Sample: 94, T: 41970
Sample: 95, T: 42000
Sample: 96, T: 42000
Sample: 97, T: 42029
Sample: 98, T: 42029
Sample: 99, T: 42057
Sample: 100, T: 42089
Sample: 101, T: 42089
Sample: 102, T: 42115
Sample: 103, T: 42121
Sample: 104, T: 42151
Sample: 105, T: 42180
Sample: 106, T: 42180
Sample: 107, T: 42180
Sample: 108, T: 42207
Sample: 109, T: 42235
Sample: 110, T: 42273
Sample: 111, T: 42273
Sample: 112, T: 42302
Sample: 113, T: 42302
Sample: 114, T: 42329
Sample: 115, T: 42360
Sample: 116, T: 42360
Sample: 117, T: 42387
Sample: 118, T: 42387
Sample: 119, T: 42422
Sample: 120, T: 42446
Sample: 121, T: 42446
Sample: 122, T: 42478
Sample: 123, T: 42478
Sample: 124, T: 42510
Sample: 125, T: 42537
Sample: 126, T: 42537
Sample: 127, T: 42564
Sample: 128, T: 42574
Sample: 129, T: 42595
Sample: 130, T: 42631
Sample: 131, T: 42654
Sample: 132, T: 42654
Sample: 133, T: 42687
Sample: 134, T: 42716
Sample: 135, T: 42746
Sample: 136, T: 42778
Sample: 137, T: 42778
Sample: 138, T: 42804
Sample: 139, T: 42838
Sample: 140, T: 42867
Sample: 141, T: 42900
Sample: 142, T: 42900
Sample: 143, T: 42926
Sample: 144, T: 42960
Sample: 145, T: 42983
Sample: 146, T: 43019
Sample: 147, T: 43046
Sample: 148, T: 43077
Sample: 149, T: 43104
Sample: 150, T: 43132
Sample: 151, T: 43162
Sample: 152, T: 43196
Sample: 153, T: 43225
Sample: 154, T: 43253
Sample: 155, T: 43284
```

First timestamp: T = 40292 (Sample 0)
Last timestamp: T = 43284 (Sample 155)

Elapsed time: Δt = 43284 − 40292 = 2992 ms = 2.992 s

Number of messages received: 156 messages

Messages per second ≈ 156 / 2.992 ≈ 52.1 messages/s
Average time per message ≈ 2.992 / 156 ≈ 19.2 ms/message
Each message is approximately "Sample: X, T: Y" which is ~20 characters ≈ 20 bytes.
**Effective data transfer rate:** Data rate ≈ 52.1 × 20 ≈ 1040 bytes/s


6. Now create an array that can store time stamps. This array should be defined globally so that other functions can access it if need be. In the loop, rather than send each time stamp, place each time stamp into the array. (Note: you’ll need some extra logic to determine when your array is full so you don’t “over fill” the array.) Then add a command SEND_TIME_DATA which loops the array and sends each data point as a string to your laptop to be processed. (You can store these values in a list in python to determine if all the data was sent over.)

reference: https://sgb1443.github.io/ece4160/Lab1B/

7. Add a second array that is the same size as the time stamp array. Use this array to store temperature readings. Each element in both arrays should correspond, e.e., the first time stamp was recorded at the same time as the first temperature reading. Then add a command GET_TEMP_READINGS that loops through both arrays concurrently and sends each temperature reading with a time stamp. The notification handler should parse these strings and add populate the data into two lists.


