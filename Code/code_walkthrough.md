# Segbot Code Walkthrough
This file will be going through the steps and logic to how the Segbot code works, mostly glossing over how different peripherals of the microcontroller are initialized. The [code](https://github.com/monk200/Segbot/blob/main/Code/segbot/segbot_main.c) itself is well commented and written in a way that most of the peripherals I know how to implement are set up, making it easy to change the code for other functions. Because of this, however, if this code is run on a barebones system just intended to be a Segbot then it will have issues (because certain parts on my PCB and written into my code aren't directly for the Segbot). To learn more about how to interface with the TMS320F28379D or the other parts, I recommend looking through the [datasheets provided](https://github.com/monk200/Segbot/blob/main/Parts_Lists/Revised_Parts_List.md).  

These sections are in the order necessary for the next step in the logic, not necessarily the order they appear in the [code](https://github.com/monk200/Segbot/blob/main/Code/segbot/segbot_main.c). The line numbers will be mentioned at certain points to make it easier to follow and reference the text.  

## Initialize ADCA to push MPU-9250 data onto SPI queue
### Initialize SPIB to read from the MPU-9250
Lines [267](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L267), [824-1001](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L824)  

>Prof. Block provided this [condensed SPI datasheet](http://coecsl.ece.illinois.edu/me461/Labs/SPICondensed_TechRef.pdf) with skeleton code for initializing the SPI to communicate with the IMU. The datasheet includes descriptions of what each bit in each bitfield of the SPI corresponds to. All of the comments in <code>setupSpib()</code> has comments describing what code needed to be written and it can be used in conjunciton with the datasheet to see why. The execption is the code on lines [917-995](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L917), which was entirely provided by Prof. Block.  

### Initialize EPWM5 to trigger ADCA
Lines [307](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L307), [723-738](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L723)  

>EPWM5 is set up as a timer that triggers the ADCA sample every 1ms. Similar to before, Prof. Block provided pseudocode for how to set up an EPWM register as an ADC trigger and provided a [technical reference](http://coecsl.ece.illinois.edu/me461/Labs/EPWM_Peripheral.pdf) for the EPWM peripheral.  

Lines [308](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L308), [741-809](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L741)  

>A large chunk of this function is commented out because it is skeleton code for setting up the other ADC channels. ADCA is set up to SOC0 and SOC1 trigger ADCA2 and ADCA3, respectively. This specific sampling exists because one of the extra features on my board is a joystick, however this is not used in the final Segbot (but it could be!). Regardless of what is being sampled, it is necessary to sample the ADC every 1ms to trigger the next step. Lines [769](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L769) and [772](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L772) are where EPWM5 is selected as the timer. More information on the choen values for the other parts of the ADC setup can be found in Prof. Block's [ADC technical reference](http://coecsl.ece.illinois.edu/me461/Labs/ADC_Peripheral.pdf).  

### Use ADCA interrupt to transmit MPU-9250 data
Lines [452-479](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L452)  

>As stated above, the channels being read of the ADC aren't actually used in the final Segbot so the first few lines of this funciton can be ignored. However, from line [462-473](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L462), The SPI transmit buffer reads off all of the accelerometer and gyroscope readings (including a few junk values). The choices here can be explained more by reading the [MPU-9250 technical reference](http://coecsl.ece.illinois.edu/me461/Labs/MPU-9250-Addendum.pdf). Essentially, the bits corresponding to the desired signals are put onto a queue that the SPI will read from later.  

## Setup EPWM6A and EPWM6B to control the left and right motors
### Setup Quadrature Encoder
Lines [268](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L268), [679-720](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L679)  

>The code to setup the eQEP peripherals was mostly provided by Prof. Block, but he also commented that this simple use of the eQEP was almost entirely provided by sample code that came with the microcontroller. Nonetheless, it is absolutely required before the next step in the process.  

### Create functions to read from the encoders
Lines [390-410](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L390)  

>Prof. Block provided all but the last line in each function. The goal of these functions are to return the radians that the wheel is currently at (relative to their position at startup). To determine this, I was given the information that the gear ratio is 18.7, meaning that 18.7 rotations of the motor cause one rotation of the wheel. I was also told that the eQEP creates a square wave based on when one of the 20 North/South magnetic poles spaced around the wheel passes by a Hall Effect sensor. Since the eQEP is set to be in quadrature count mode, the number of counts per revolution is 4 * 20 = 80. This can be combined with the gear ratio to find the conversion factor from eQEP counts to radians. This is done with the equation EQep1Regs.QPOSCNT * (TWOPI/(80 * 18.7)), and the polarity would be negative for the left motor and positive for the right.  

### Initialize EPWM6A and EPWM6B
Lines [351-372](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L351)  

>Similarly to [setting up EPWM5](https://github.com/monk200/Segbot/new/main/Code#initialize-epwm5-to-trigger-adca), the [EPWM technical reference](http://coecsl.ece.illinois.edu/me461/Labs/EPWM_Peripheral.pdf) can be used to set up EPWM6A and EPWM6B to control the motor speeds. Their period needs to be set to a 20 kHz carrier frequency and their duty cycle will be used to control their speed.  

### Create functions to set the speed of the motors
Lines [412-435](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L412)  

>These functions take in a <code>controleffort</code> variable. They need to start by saturating the control to be between -10 and 10, corresponding to the minimum and maximum reasonable operating speeds for the motors. After that, either GPIO29 or GPIO32 (depending on which motor) is set to a binary value representing if the motor is moving forward, which is true if the control effort is positive. The last line of each function scales the control effort to the period of the register and sets that value as the duty cycle. One difference in my code from what most people's code should look like is that right wheel encoder had an inconsistency in wiring so the polarity is opposite of what it should be. This is compensated for in my code but any recreations of this project should be careful to test that the motors are moving in the direction the user intends them to be before moving forward with the control algorithms.  

## Use the SPIB interrupt to calibrate MPU-9250 and gather filtered values over time
Lines [482-581](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L482)  

>The first thing the SPIB interrupt does when it is called every 1ms is read the MPU-9250 values off the SPI queue. These raw values are converted so that the accelerometer readings range from -4g to 4g and the gyroscope readings range from -250 degrees/sec to 250 degrees/sec. Just before entering the calibration stage, it also reads the current position of each wheel (again accounting for the pinout inconsistency on my encoder).  

### Calibration
Lines [509-533](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L509)  

>The calibration of the MPU-9250 starts by just taking two seconds to let any electrical noise caused by startup calm down (calibration_state = 0). After those two seconds, it spends another two seconds just summing the reading for each parameter. At the end of this state, it takes the average over that two second period and sets it as the parameter's corresponding offset (calibration_state = 1). This process happens upon startup so it is important to keep the Segbot steady for at least 4 seconds (there will be a blue LED that blinks during the calibration process, [line 572](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L572)).  

### Kalman Filter
Lines [534-569](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L534)  

>A Kalman filter is used to combine the accelerometer reading and the integral of the gyroscope. The accelerometer provides a tilt reading but it is slow and therefore not responsive enough to let the control algorithm balance the Segbot. The integral of the gyroscope provides a tilt rate much faster, but its downside being that it starts drifting eventually because of electrical noise. Using the Kalman filter to combine the two allows for an accurate and fast reading of the tilt. At the end of this function, since the SPI is called every 1ms and the control algorithm is only called every 4ms, the average of the last 4ms of data is updated and passed onto the control algorithm.  

## Use software interrupt to implement control algorithms
### Balancing control
Lines [584-597](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L584)  

>The balancing control starts by applying a low-pass filter in discrete time to estimate the velocity of each wheel. Magnetic encoders have a low amplitude and the noise in the position variable is amplified when differentiated, so blocking out higher frequencies lead to more accurate estimates of the velocity. In my implementation, I applied a low-pass filter with a cutoff frequency of 125 rad/s, the bode plot of which can be seen below. It it worthwhile to note here that the cutoff frequency is a tunable parameter and the Segbot performance can be changed by changing it. Also below is a snippet of the Matlab code used to take the derivative of the filter and then convert it to discrete time using the Tustin rule (aka bilinear transformation aka trapezoidal rule) at a 4ms sample rate. This final equation, <code>vel_pos</code>, can be converted so that the <code>z</code> is to the power of -1, making it refer to a unit delay. This is necessary in real-time application because at any given moment we only know the past states. The final form of the equation can be seen in the code on [lines 592 and 593](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L592).  

<p align="center">
  <img src="https://github.com/monk200/Segbot/blob/main/Code/Figures/wheelVelFilter%20Bode.PNG" alt="Bode Plot" width="500"/>
  <img src="https://github.com/monk200/Segbot/blob/main/Code/Figures/wheelVelFilter%20discrete.PNG" alt="Matlab Calculations" width="400"/>
  <img src="https://github.com/monk200/Segbot/blob/main/Code/Figures/wheelVelFilter%20expanded.PNG" alt="Expanded Equation" width="400"/>
</p>  

>After estimating the current velocity of the left and right wheels, the actual balancing control needs to be added. For this, I used a full state feedback control where x<sub>1</sub> is tilt, x<sub>2</sub> is tilt rate, and x<sub>3</sub> is the average of the two (now filtered) wheel velocities. This portion of the equations were derived by [Yorihisa Yamamoto's Segobt paper](https://www.mathworks.com/matlabcentral/fileexchange/19147-nxtway-gs-self-balancing-two-wheeled-robot-controller-design) and can be found in the code for the [Matlab model](https://github.com/monk200/Segbot/tree/main/Matlab_Simulation) used previously. Ultimately, the control effort u = -K<sub>1</sub>x<sub>1</sub> - K<sub>2</sub>x<sub>2</sub> - K<sub>3</sub>x<sub>3</sub>, where each variable K is a tunable gain. The implementation of this in the code is on [line 596](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L596). As far as tuning the gains, the [Matlab simulation](https://github.com/monk200/Segbot/tree/main/Matlab_Simulation/Reworked%20SImulation%20Files) was used as a starting point (and those gains need to be scaled up to to real-world Segbot) but because of discrepencies between the simulated model and the real model, the values may need to be tuned further on the real robot. For my robot, the gains K<sub>1</sub> = -30, K<sub>2</sub> = -2.8, K<sub>3</sub> = -1.0 worked sufficiently but can still be improved to reduce the small oscillations happening in the video below (which are slightly caused by the cable dragging anyway). The gains I had used in my simulation ([-34.3159, -3.6375, -3.3290]) also balanced but seemed to be slightly less stable. The parameter that actually seemed to have more of an impact when tuned was [the angle the body of the Segbot tries to balance at](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L77).  

[![Balancing Segbot](https://i9.ytimg.com/vi/_FPlLJUeUD4/mq2.jpg?sqp=CICPmoUG&rs=AOn4CLCNEwNHM3WrOVnD_qe3GA9vNIED5g)](https://youtu.be/_FPlLJUeUD4)  

### Steering control
Lines [600-622](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L600)  

>The process of starting the steering control starts similarly to calculating wheel velocity. The steering control essentially commands the Segbot to turn at a certain rate by aiming to a achieve a certain difference between the left and right wheel velocities. To do this, I started by using a low-pass filter with a cutoff frequency of 250 rad/s and converting it to discrete time at a 4ms sample rate again. The steps from the last filter are taken and shown below, to finally produce the code in [line 602](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L602).  

<p align="center">
  <img src="https://github.com/monk200/Segbot/blob/main/Code/Figures/velDifFilter%20Bode.PNG" alt="Bode Plot" width="500"/>
  <img src="https://github.com/monk200/Segbot/blob/main/Code/Figures/velDifFilter%20discrete.PNG" alt="Matlab Calculations" width="400"/>
  <img src="https://github.com/monk200/Segbot/blob/main/Code/Figures/velDifFilter%20expanded.PNG" alt="Expanded Equation" width="400"/>
</p>  

>Next, the error between the desired angle to turn to and the difference between the two wheel positions in radians is integrated using the trapezoidal rule ([line 605](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L605)). Doing these calculations produces the necessary variables to perform PID control of the turn. The turn command ends up being K<sub>p</sub> * turnError + K<sub>i</sub> * turnErrorIntegral - K<sub>d</sub> * diffInWheelVels ([line 606](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L606)).  

>Before applying the turn command determined by the PID algorithm, it needs to both account for integral windup and be saturated so that it doesn't overpower the balance control. Integral windup happens when for some reason the error term in the algorithm keeps growing, often caused when the motor can't meet the desired control effort. We demonstrated this in the lab by holding onto one of the wheels and then when you release it, it goes extremely fast because it is trying to correct for a huge percieved error. A more stable system doesn't attempt to correct itself so aggressively because it can damage the motor or throw the Segbot off balance. In this case, the code will stop the error integral from accumulating if the turn command is near its maximum value. When the turn command is near the maximum value, there is no reason to keep accumulating the error integral because the error is already right at the maximum error the Segbot can account for. As far as saturting the control, the code keeps the turn command within [-4, 4] so that it never overpowers the balancing control that ranges from [-10, 10]. A block diagram of the turning algorithm can be seen below: 

<p align="center"><img src="https://github.com/monk200/Segbot/blob/main/Code/Figures/Turn%20Control%20Block%20Diagram.PNG" alt="Turn PID" width="1000"/></p>  

>Finally, on [lines 614-622](https://github.com/monk200/Segbot/blob/675fc0f092b4d83fd8ebe73bdf923763511bc55d/Code/segbot/segbot_main.c#L614), the balacing control effort, turning control effort, and direction commands are combined, saturated to be between -10 and 10, and sent to the left and right motor control functions. The last portion of the software interrupt just updates all of the past states for the next time the software enters the interrupt to calculate a new control effort. A video of me controlling the Segbot's motion using keyboard input can be seen below:  

[![Controlling Segbot](https://i9.ytimg.com/vi/ZbytxEKrEEE/mq2.jpg?sqp=CIykmoUG&rs=AOn4CLBqXdg6dqMmESX1mFsb2rz_ZUEcKA&retry=5)](https://youtu.be/ZbytxEKrEEE)  

## Sources
https://doc.synapticon.com/software/42/motion_control/advanced_control_options/filtering/low-pass-filter/index.html  
https://www.electrical4u.com/cutoff-frequency/  
https://www.allaboutcircuits.com/technical-articles/deriving-and-plotting-a-low-pass-transfer-function-on-matlab/  
https://control.com/technical-articles/intergral-windup-method-in-pid-control/  
