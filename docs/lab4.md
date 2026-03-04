# Prelab

## Diagram with your intended connections between the motor drivers, Artemis, and battery

To drive the motors, I used pina A3, A14, A15, and A16 as these are PWM capable and analog. Also, these pins are conveniently placed on the opposite side of the Artemis QWIIC port so there would be minimal damage if compoenents move in the car and the pins are closer to the motors allowing for shorter wires avoiding EMI.  I followed the diagram shown in class as shown below: 

![diagrm](assets/lab4/lab4diagram.png)

## Battery 

The Artemis and motors are powered from separate batteries. The Artemis from a 650 mAh battery and the motors from a 850mAh battery. This is important because motors draw large amounts of current when starting, which causes voltage drops on a shared power supply. If both were on the same battery, the voltage could drop below the Artemis's operational range, causes resets or BLE disconnects. Keeping them separate ensures stable operation of the microcontroller regardless of motor load. 

# Lab Tasks

## Picture of your setup with power supply and oscilloscope hookup

I started by soldering the motor driver to the Artemis and tested it could output PWM signals. I connected an oscilloscope to the output signal of one motor driver and grounded it. I powered the driver using a 3.7volt expernal power supply, equivalent to using a battery. I sent a 50% duty cicle PWM signal for both drivers. I tested each separately and obtained the following results: 

## Image of your oscilloscope

I obtained a chopped rising edge result instead of a perfect square wave for both drivers as follows: 

![chopped1](assets/lab4/chopped1.png)

I resoldered the drivers and kept getting the same result. 

![chopped1](assets/lab4/chopped2.png)

Even though this is not normal PWM behavior and does not match the expected results, I continued the lab, paying attention to any anomalies. In the end, the drivers worked just fine. 

As part of the debugging process for the chopped edge I was able to determine that this was an isolated issue regarding the motor driver and not the Artemis. The Artemis was outputting a perfect square wave as shown: 

![chopped1](assets/lab4/square.png)

## Power supply setting

The external power supply was set to 3.7V to match the nominal voltage of the 850mAh LiPo battery, with a current limit arounf 1A to protect the motor drivers during initial testing. 

## Wiring to car + Testing motor drivers

I removed the circuit board on the car and the attatched LEDs to it. I soldered the motor connections to the driver output pins. To test the motors, I kept using the external power supply and ran the following code that would make my wheels go forward for 5 seconds, and backwards for 5 seconds: 

````
// forward backward test
//forward
  analogWrite(3, 150);
  analogWrite(16, 150);
  analogWrite(14, 0);
  analogWrite(15, 0);
  delay(5000);
//backward
  analogWrite(14, 150);
  analogWrite(15, 150);
  analogWrite(3, 0);
  analogWrite(16, 0);
  delay(5000);
````

Here is a video of the code running: 

## Short video of wheels spinning as expected (including code snippet it’s running on)

<iframe width="560" height="315" src="https://www.youtube.com/embed/n6jmWBIV9c4?si=LcekCE5srbo_04N4" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

I also tested running the car using the 850mAh battery. I soldered the battery to the motor driver. See the following video for results: 

## Short video of both wheels spinning (with battery driving the motor drivers)

<iframe width="560" height="315" src="https://www.youtube.com/embed/gb6jZ4B1DHs?si=qXSFO98zxbwUQuQl" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Looking back, I made the mistake to solder the battery directly onto the motor drivers. I did not notice the connector we were given and assumed I had to cut the existing connector of the battery. I realized my mistake when the battery depleated and my car was not working. I am hoping to get a replacement during my next lab session 🥹. But for now, I borrowed a friend's. 

## Picture of all the components secured in the car
## Consider labeling your picture if you can’t see all the components

I fixed all the components in the car with tape. The motor drivers were placed far away from the Artemis and IMU to avoid EMF interference. The ToF sensors were placed on the walls outside of the car to detect objects. As seen in the following picture, I did not secure the Artemis board yet as I kept contantly plugging it to my computer to debug and download code. 

![artemis](assets/lab4/Artemis.png)

## Lower limit PWM value discussion

I tested the minimum PWM values the motors would operate under by incrementally changing the values provided to analoWrite from 0 to 255. I found that an analogWrite value of **35** was sufficient to make the car go forward and backward. This corresponds to a duty cycle of approximately 13.7%

For on-axis turns, a value of 140 was necessary to get both wheels running, although the turn was sluggish at this value. Increasing to 150 ensured the car would immediately and smoothly spin on axis. The videos below show the difference between turning at 140 and 150. 

<iframe width="560" height="315" src="https://www.youtube.com/embed/dtmJ3wMXpCQ?si=A96E5sIl4lVn0pdU" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

<iframe width="560" height="315" src="https://www.youtube.com/embed/WXuU-dE0o2g?si=f0t0W6pLrWeTlHjN" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Calibration demonstration (discussion, video, code, pictures as needed)
## Open loop code and video
## (5000) analogWrite frequency discussion (include screenshots and code)
## (5000) Lowest PWM value speed (once in motion) discussion (include videos where appropriate)
