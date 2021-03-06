# Segbot
<img align="right" height="350" src="https://github.com/monk200/Segbot/blob/main/Demo%20Picture.PNG">

This repository contains files such as code, CAD files, Matlab simulations, and brainstorming documents from my work designing and developing a two-wheeled balancing robot. The project can be split into two main parts: designing the system and developing the real-world version of the system. The original design was done in my [Mechatronics](http://coecsl.ece.illinois.edu/ge423/) course as a final project for Spring 2020 but the real robot was built and programmed in the course [Computer Control of Mechanical Systems](http://coecsl.ece.illinois.edu/me461/) in Fall 2020. Both courses were led by UIUC Prof. Dan Block, the university's Control Systems Lab Manager. Prof. Block created a guide to building a Segbot at [this link](http://coecsl.ece.illinois.edu/segbot/segbot.html) and his guidance and expertise was absolutely essential in all steps of this project.  

## Designing the System
Because of the COVID-19 pandemic, Prof. Block restructured the final project of Mechatronics to be to design a Segbot from scratch. At the time, the [Mechatronics course used the MPS430G2553 microcontroller](https://github.com/monk200/Mechatronics_with_MSP430G2553) so all of the initial design choices and considerations were made keeping that in mind. I compiled a list of parts that were essential for desiging a PCB that would interface with the microcontroller. I was also tasked with creating a CAD model of the robot body and using its parameters, as well as the parameters of the motors, to tune a simulation to balance the Segbot. The last portion of the designing process was to create pseudocode for how the robot needed to behave to balance, solving the inverted pendulum problem. Below you can find a list of each step in the process:  
* [Creating a parts list](https://github.com/monk200/Segbot/blob/main/Parts_Lists/Initial_Parts_List.md): This document includes a list of each part, a link to an online retailor to buy the part, and a link to its datasheet. Most parts also include commentary on the thought process behind choosing the specified part.  
* [Initial wiring configuration](https://github.com/monk200/Segbot/blob/main/Wiring/Initial%20Design%20Wiring%20Diagram.png): This image was created and iterated upon when planning out how the system needs to be connected electronically. This was done as an intermediate step before desiging the PCB in EagleCAD because the color-coding made it quick and easy to troubleshoot and discuss.  
* [PCB design](https://github.com/monk200/Segbot/tree/main/Wiring/Initial%20EagleCAD%20Files): EagleCAD was used to design a PCB that the MSP430 would sit onto. Some parts needed to be created from scratch but otherwise it is a very minimal board.  
* [CAD model](https://github.com/monk200/Segbot/tree/main/Frame/Initial%20Frame%20Design): Autodesk Fusion 360 was used to design a body for the Segbot that would ideally be 3D-printed in UIUC's ISE design lab.  
* [Matlab model](https://github.com/monk200/Segbot/tree/main/Matlab_Simulation/Initial%20Simulation%20Files): A simulation animation and framework were provided by Prof. Block. The parameters from the previous stages of the design process were input into the model and it was manually tuned to balance. This step is useful as a proof-of-concept and as an excersie for students to get experience tuning a control system.  

## Developing the System
While Prof. Block signed off on the original design as feasible, since the second part of this project was done as part of a different course it didn't use the exact same design. Prof. Block upgraded to using the [TMS320F28379D microcontroller](https://github.com/monk200/Mechatronics_with_TMS320F28379D), added a variety of peripherals to the PCB, and simplified the body of the Segbot. For the sake of being thorough, the points below include information on each stage of the design process and how they've changed:  
* [Parts list](https://github.com/monk200/Segbot/blob/main/Parts_Lists/Revised_Parts_List.md): This document lists the essential parts used in the new Segbot design and how they compare to the specs of the previously chosen parts.  
* [Wiring configuration](https://github.com/monk200/Segbot/blob/main/Wiring/Reworked%20Wiring%20Diagram.png): This image is again a color-coded wiring schematic to show a quick layout of how the new components are connected.  
* [PCB design](https://github.com/monk200/Segbot/tree/main/Wiring/Reworked%20EagleCAD%20Files): This folder contains the new EagleCAD files for the PCB. As stated earlier, this is a much more complicated design because a lot of peripherals were added to allow the board to be used for exploring other applications.  
* [CAD model](https://github.com/monk200/Segbot/tree/main/Frame/Reworked%20Frame%20Design): A much simpler design, this design is only two 3D-printed legs to be attached to the PCB itself.  

With a new physical design, the Matlab model needed to be redone but this time since the physical parts were present the tuning was much more reliable. Throughout the [Computer Control of Mechanical Systems](https://github.com/monk200/Mechatronics_with_TMS320F28379D) course, the new microcontroller was explolred and experimented with. Near the end of the semester, I programmed the board to implement control algorithms to solve the inverted pendulum problem and control the robot's motion. The links below lead to details on each step of this process:  
* [Matlab model](https://github.com/monk200/Segbot/tree/main/Matlab_Simulation/Reworked%20SImulation%20Files): Using the same simulation as before but new, more accurate, parameters, I manually tuned the Segbot to balance.  
* [Programming](https://github.com/monk200/Segbot/tree/main/Code): This link goes to the code for the project with extensive details on the processes used and how to set it up on the hardware.  

## Final Product
Below are links to several videos demonstrating different portions of the Segbot's performance:  

### Controlling the Segbot with the Keyboard
[![Controlling Segbot](https://i9.ytimg.com/vi/ZbytxEKrEEE/mq2.jpg?sqp=CIykmoUG&rs=AOn4CLBqXdg6dqMmESX1mFsb2rz_ZUEcKA&retry=5)](https://youtu.be/ZbytxEKrEEE)  
[Github Video Link](https://github.com/monk200/Segbot/blob/main/Working%20Demo%20Video.mp4)  

### Balancing Segbot
[![Balancing Segbot](https://i9.ytimg.com/vi/_FPlLJUeUD4/mq2.jpg?sqp=CICPmoUG&rs=AOn4CLCNEwNHM3WrOVnD_qe3GA9vNIED5g)](https://youtu.be/_FPlLJUeUD4)  

### The Stable Simulation
[![Stable Simulation](https://i9.ytimg.com/vi/3cc0C4fP93k/mq2.jpg?sqp=COivmoUG&rs=AOn4CLCJ8H2jLm_IbXASKFX4jyyljRI18w)](https://youtu.be/3cc0C4fP93k)  
