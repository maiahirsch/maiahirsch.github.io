

Lab 1 B: 

1. Send a string value from the computer to the Artemis board using the ECHO command. The computer should then receive and print an augmented string.
For example, the computer sends the string value “HiHello” to the Artemis board using the ECHO command, and the computer receives the augmented string “Robot says -> HiHello :)” from a read GATT characteristic.

![Lab 1B Tast 1](assets/lab1btask1.png)

2. Send three floats to the Artemis board using the SEND_THREE_FLOATS command and extract the three float values in the Arduino sketch.

3. Add a command GET_TIME_MILLIS which makes the robot reply write a string such as “T:123456” to the string characteristic.

using as reference: https://akinfelami.github.io/fastrobots-2025/artemis-and-bluetooth

updated CommandTypes

using: https://rga47-lab.github.io/lab1.html

was running into a lot of errors with GET_TIME_MILLIS missing from CMD and fixed it by restarting the Kernel

4. Setup a notification handler in Python to receive the string value (the BLEStringCharactersitic in Arduino) from the Artemis board. In the callback function, extract the time from the string.

reference: https://rga47-lab.github.io/lab1.html

using extract from chart (screenshot)

5. Write a loop that gets the current time in milliseconds and sends it to your laptop to be received and processed by the notification handler. Collect these values for a few seconds and use the time stamps to determine how fast messages can be sent. What is the effective data transfer rate of this method?

using as reference: https://akinfelami.github.io/fastrobots-2025/artemis-and-bluetooth

I then used the following methodology to determine the effective data transfer rate of this method after all 1000 time stamps were received.

Taking the last time stamp T:95213.000 minus the first time stamp T:93824.000. The message rate is 100 over the difference between time staps (1.389). There are approximately 720 messages being sent / sec. My message has 12 bytes, one per character. meaning 8640 bytes/sec


chsnged it to new notification handler. updated time stamps screenshot. 117 time stamps. need to do math again 

6. Now create an array that can store time stamps. This array should be defined globally so that other functions can access it if need be. In the loop, rather than send each time stamp, place each time stamp into the array. (Note: you’ll need some extra logic to determine when your array is full so you don’t “over fill” the array.) Then add a command SEND_TIME_DATA which loops the array and sends each data point as a string to your laptop to be processed. (You can store these values in a list in python to determine if all the data was sent over.)

reference: https://sgb1443.github.io/ece4160/Lab1B/

7. Add a second array that is the same size as the time stamp array. Use this array to store temperature readings. Each element in both arrays should correspond, e.e., the first time stamp was recorded at the same time as the first temperature reading. Then add a command GET_TEMP_READINGS that loops through both arrays concurrently and sends each temperature reading with a time stamp. The notification handler should parse these strings and add populate the data into two lists.


