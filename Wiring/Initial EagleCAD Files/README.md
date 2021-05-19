# PCB for Initial Design
EagleCAD was used to design the circuit board for the initial design. The circuit schematic can be seen to very closely resemble the [initial wiring diagram](https://github.com/monk200/Segbot/blob/main/Wiring/Initial%20Design%20Wiring%20Diagram.png). The board was intened to be fabricated at https://www.4pcb.com/ because students get a discount that their boards only cost $33 and they have their own [free file checker](https://www.4pcb.com/free-pcb-file-check/index.html).  

## Wiring Considerations
I tethered the AD0 pin on the MPU-6050 to ground because it is used to toggle between two slave addresses, which is unneccesary in this case. By tethering it to ground, the slave address is set to 0x68 and one less pin on the MSP-430 needs to be occupied. I also connected PMODE on both DRV8874 chips to ground to select the control mode that takes an enable input and a phase. The capacitors and resistors connected directly to the DRV8874 chips are based on the diagram from [page 9 of the datasheet](https://www.ti.com/lit/ds/symlink/drv8874.pdf?ts=1621215513855&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FDRV8874). Lastly, since the MPU-6050 is already using UCB for its clock and reading data, the LS73366R chips are using UCA instead of leaving UCA for connecting to a terminal emulator. This is slightly inconvenient for programming and debugging purposees but is a necesssary sacrifice to connect the required circuitry.  

## Custom Parts
The only custom part that I needed to create was the MPU-6050 ([MPU6050_MODULE in SE423EagleLibrary.lbr](https://github.com/monk200/Segbot/blob/main/Wiring/Initial%20EagleCAD%20Files/SE423EagleLibrary.lbr)). The module chosen didn't list the exact information on the pin spacing so some assumptions were made that since it is technically an Arduino module, the pins would be spaced the same distance apart. Other than that, the circuit design was fairly simple to work through.  

## 4PCB File Check
Running my design through the free file checker produced no major issues and it automatically fixed a recurring line width error I made. The specific error message output was "Insufficient Silkscreen Line Width (1543 violations)," and, as I noted, the file checker automatically fixed the error.  

## EagleCAD Design
<p align="center"><img src="https://github.com/monk200/Segbot/blob/main/Wiring/Initial%20EagleCAD%20Files/brd%20picture.PNG" alt="BRD" width="800"/></p>  

<p align="center"><img src="https://github.com/monk200/Segbot/blob/main/Wiring/Initial%20EagleCAD%20Files/sch%20picture.PNG" alt="SCH" width="800"/></p>  

