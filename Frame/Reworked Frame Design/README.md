# Reworked Frame Design
Prof. Block simplified the frame design so much so that there isn't really a "frame" left. Instead of designing a protective casing for the hardware, he only designed front-facing legs that the motor subassembly fastens into and then both attach to the PCB. Minimizing the size of this part reduces 3D printer filament needed and makes the Segbot overall more condensed. The battery pack is attached to the back of the PCB by wrapping around the whole thing with a rubber band, which is very secure and manages to not be in the way. Minimizing the design like this also, however, removes the safety of allowing the Segbot to fall backwards when trying to tune the poles. This ended up not being a problem while I was running the program because I always kept a watchful eye on it and used it on smooth, predictable surfaces.  

<p align="center"><img src="https://github.com/monk200/Segbot/blob/main/Frame/Reworked%20Frame%20Design/render1.png" alt="Render 1" width="1000"/></p>  

<p align="center"><img src="https://github.com/monk200/Segbot/blob/main/Frame/Reworked%20Frame%20Design/render2.png" alt="Render 2" width="1000"/></p>  

## Model Parameters
For the next step of simulating the Segbot in Matlab, the parameters of the physical model will be needed. Fusion provides these and I listed them in [Frame Physical Properties.txt](https://github.com/monk200/Segbot/blob/main/Frame/Reworked%20Frame%20Design/Frame%20Physical%20Properites.txt). Below is a summary of the properties:  

* Material:	ABS Plastic
* Mass	2.134E+04 g
* Center of Mass	150.00 mm, 319.389 mm, 255.088 mm  

The important parameters shown here are less detailed than in my initial design because of how much smaller this "frame" design is. The two legs on the PCB are going to have much less of an overall effect on the physics of the Segbot than when the bulk of the Segbot's mass before was due to the frame. Also, because this time there were real physical parts to investigate, the Matlab parameters can be measured more directly and are therefore more reliable.  
