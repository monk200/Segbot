# PCB for Reworked Design
Prof. Block used EagleCAD to design the circuit board for the reworked design. The circuit schematic can be seen to connect each part in the way my [reworked wiring diagram](https://github.com/monk200/Segbot/blob/main/Wiring/Initial%20Design%20Wiring%20Diagram.png) does, but my diagram only has the essential peripherials to make the Segbot work. The board was fabricated at https://www.4pcb.com/ and my professor uses its [free file checker](https://www.4pcb.com/free-pcb-file-check/index.html).  

## Wiring Considerations
This board design is significantly more complex because the board was designed with the intent of using it for any number of personal projects. My initial design minimized complexity because I was focusing on the project at hand and I am less experienced than my Professor. When looking through his design, he added a switch to power the motor subassembly on and off and a potentiometer for each motor connected to the H-bridge's VREF pin. This allows the user to regulate the maximum motor current. Other major changes from my design include that Prof. Block's design needs to have an external real-time clock but does not need external quadrature encoders. The most complex difference between my design and Prof. Block's design is the power system. Prof. Block designed this PCB to be able to be powered by an AC outlet or by a DC battery pack. Since the Segbot is a mobile project, I only focused on deconstructing the DC power system.  

## EagleCAD Design
<p align="center"><img src="https://github.com/monk200/Segbot/blob/main/Wiring/Reworked%20EagleCAD%20Files/brd%20picture.PNG" alt="BRD" width="800"/></p>  

<p align="center"><img src="https://github.com/monk200/Segbot/blob/main/Wiring/Reworked%20EagleCAD%20Files/sch%20picture.PNG" alt="SCH" width="1100"/></p>  

<p align="center"><img src="https://github.com/monk200/Segbot/blob/main/Wiring/Reworked%20EagleCAD%20Files/headers%20picture.PNG" alt="SCH" width="600"/></p>  

