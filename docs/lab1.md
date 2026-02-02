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


