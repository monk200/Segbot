# Initial Frame Design
Fusion 360 was used to design the frame, create drawing files, and render images and videos. Since the plan was to 3D print the frame, I wanted to make the dimensions fit inside the ISE product design lab’s Stratasys Dimension SST 1200 3D printer. This means the design had to be smaller than 30x25x25 cm. It also needed to house a battery, the PCB, and account for the motors and wheels. The design ended up being a fairly simple box with pockets inside to be able to set the battery and board into, as well as having front-facing windows to be able to see and quickly access the hardware. The sides of the frame have circular cutouts to fit the motors through and the front and back of the frame have kickstands to keep the Segbot from damaging itself if it fails to balance. The kickstands were highly recommended by Prof. Block because not only do they minimize the damage from the Segbot going off balance but they also provide a stable, repeatable position to calibrate the sensors on startup.  

<p align="center"><img src="https://github.com/monk200/Segbot/blob/main/Frame/Initial%20Frame%20Design/render1.png" alt="Render 1" width="1000"/></p>  

<p align="center"><img src="https://github.com/monk200/Segbot/blob/main/Frame/Initial%20Frame%20Design/render2.png" alt="Render 2" width="1000"/></p>  

## Model Parameters
For the next step of simulating the Segbot in Matlab, the parameters of the physical model will be needed. Fusion provides these and I listed them in [Frame Physical Properties.txt](https://github.com/monk200/Segbot/blob/main/Frame/Initial%20Frame%20Design/Frame%20Physical%20Properites.txt). Below is a summary of the properties:  

* Material:	ABS Plastic
* Mass	1535.286 g
* Center of Mass	-10.00 cm, 13.805 cm, 3.897 cm
* Body Width: 0.20 m
* Body Depth: 0.20 m
* Body Height: 0.29 m
* Distance of the Center of Mass from the Wheel Axis: 11.86 m  

More calculations and their equations will be explained in the [Matlab Model](https://github.com/monk200/Segbot/tree/main/Matlab_Simulation/Initial%20Simulation%20Files) section.  
