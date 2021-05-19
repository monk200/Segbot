# Parts List for the Revised Design
The revised design was made by Prof. Block and I've deconstructed it here and in the [wiring diagram](https://github.com/monk200/Segbot/blob/main/Wiring/Reworked%20Wiring%20Diagram.png) to isolate the essential parts needed just for the Segbot project. There are many similarities between my initial design and Prof. Block's design with the only major changes being to the microcontroller and the frame. Below I go through each part used in Prof. Block's design and I compare his choices to mine with commentary on how these changes improve the Segbot.  

## Microcontroller
The biggest change in the new design is that it is using the TMS320F28379D microcontroller (on the TI development board). Both the MSP430 and the TMS320F28379D are part of TI's C2000 series but this board has a lot more pins and peripherals, making it a good upgrade for leaning about embedded systems and making more complex projects. One of the challenges with the initial design was that there were so few pins that wiring had to be carefully planned out. Some other chips in the last system had some of their pins either completely disconnected or tied to ground because there wasn't the luxury of extra pins. During conversations about the initial design I even mentioned potentially making the control wireless or adding a camera, both of which would have been very difficult to manage with the three pins left over after connecting the essential elements.  

The TMS320F28379D has two 32-bit CPUs, 204 KB SRAM, 50 I/O pins, between 12-24 ADC channels at either a 16 or 12 bit resolution, and three built-in quadrature encoders. Prof. Block compiled a cheat-sheet for navigating the differnt uses of each pin, which can be found [here](https://github.com/monk200/Segbot/blob/main/Wiring/TMS320F28379D%20Pinout%20Info/PinMuxTableF28379DLaunchPad.pdf). Below are a couple images of the layout of the launchpad board used:  
<p align="center"><img src="https://github.com/monk200/Segbot/blob/main/Wiring/TMS320F28379D%20Pinout%20Info/Launchpad%20pinout.PNG" alt="TMS320F28379D Pinout" width="1000"/></p>  
<p align="center"><img src="https://github.com/monk200/Segbot/blob/main/Wiring/TMS320F28379D%20Pinout%20Info/Launchpad%20overview.PNG" alt="Launchpad Layout" width="600"/></p>  

[Texas Instruments Page](https://www.ti.com/tool/LAUNCHXL-F28379D)  
[Chip Datasheet](https://www.ti.com/lit/ds/sprs880o/sprs880o.pdf?ts=1621420796409)  
[Launchpad Datasheet](https://www.ti.com/lit/an/sprt720/sprt720.pdf?ts=1621438664717&ref_url=https%253A%252F%252Fwww.ti.com%252Ftool%252FLAUNCHXL-F28379D%253FbaseDir%253DC%253A%252Fti%252FcontrolSUITE%252F%257Ecs_desktop%252FcontrolSUITE)  
[Store page](https://www.digikey.com/en/products/detail/texas-instruments/LAUNCHXL-F28379D/7219341?utm_adgroup=Texas%20Instruments&utm_source=google&utm_medium=cpc&utm_campaign=Smart%20Shopping_Supplier_Texas%20Instruments&utm_term=&utm_content=Texas%20Instruments&gclid=Cj0KCQjw7pKFBhDUARIsAFUoMDao6VYeKDVkAEsnp9LdM7cmuvV7M230qgGeLHqIusd2_IxUyIF2nUcaApuHEALw_wcB)  

Cost: $41 on digikey.com, $0 since I recieved it from the course  

## Motor Control
The motor control was done very similarly: a geared DC motor with an attached optical encoder was paired with the same DRV8874 H-bridge. The motor chosen did not also come with the wheel assembly like the initial design but it used the same (or at least visually identical) wheels. The main difference in the motor assembly then is that the motor is 6V instaed of 12V. In my [initial parts list] I mentioned how the reason I chose a 12V motor was because I was concerned that the size of the Segbot body would make it heavy enough that a more powerful motor would be needed. Since the body is redesigned to be much more minimalistic and lightweight, a 6V motor easily suffices. The specific motor chosen was the GM25-370 DC 6V geared motor.  

<p align="center"><img src="https://imgaz1.staticbg.com/thumb/large/oaupload/banggood/images/51/C2/35f730c1-307d-4351-bcfc-5fbc0eedebf5.jpg.webp" alt="GM25-370 Motor" width="300"/></p>  

[Store Page](https://www.banggood.com/CHIHAI-MOTOR-GM25-370-DC-Gear-Motor-6V-100210300RPM-Encoder-Motor-p-1016183.html?cur_warehouse=CN&ID=519629)  
Cost: $27 for two motors + cost of two wheels  

Same H-bridge: DRV8874  
[Chip Datasheet](https://www.ti.com/lit/ds/symlink/drv8874.pdf?ts=1621215513855&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FDRV8874)  
[Texas Instruments Page](https://www.ti.com/product/DRV8874)  
Cost: $2.50 for two H-bridges  

An added feature to this design is a switch before the motor subassembly that will turn them on or off entirely. A potentiometer was also used in conjunction with each side of the motor subassembly, wired to the VREF pin of the H-bridge. Doing this allows me to regulate the maximum motor current. Other than some small changes because of these additions, my initial circuit design surrounding the H-bridge was fairly similar.  

## Sensors
A very similar Arduino module combining the accelerometer and gyroscope sensors was chosen. The main difference on this switch from using the MPU-6050 to now using the MPU-9250 is the the 9250 is a 9-axis sensor instead of a 6-axis sensor. The new module combines an accelerometer, gyroscope, and a magnometer which, similarly to upgrading the microcontroller, adds potential to do more advanced projects with the same hardware in the future. Otherwise, for this specific project the change doesn't affect performance on the Segbot.  

<p align="center"><img src="https://images-na.ssl-images-amazon.com/images/I/61j4fPRgkhL._AC_UL1010_.jpg" alt="Accelerometer, Gyroscope, and Magnometer" width="200"/></p>  

[Chip Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)  
[Store Page](https://www.amazon.com/HiLetgo-Gyroscope-Acceleration-Accelerator-Magnetometer/dp/B01I1J0Z7Y)  
Cost: $10  

## Battery
The power supply subsystem is probably the most complex change happening in the revised design (and I am not particularly skilled in designing power circuits). Prof. Block designed the PCB to be capable of being powered both by an AC outlet or a DC battery pack. On top of that, a 3V Lithium button cell battery (CR2032) was added to power a real-time clock that will be discussed in the next section. The primary power supply is run through a DC regulator (the TL1963A-33KTTR) to bring it to 5V, which is used to power the microcontroller. From there, the microcontroller regulates and distributes the power to the other parts of the system when they need either a 5V input or a 3.3V input.  

[TL1963A-33KTTR Datasheet](https://www.ti.com/lit/ds/symlink/tl1963a.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1621389533941)  
[Store Page](https://www.digikey.com/en/products/detail/texas-instruments/TL1963A-33KTTR/1905593)  
Cost: $3

Since the AC power option is for working on a stationary board and the Segbot is mobile, I'm only focusing on the DC inputs. A battery pack of four AA batteries supplies 1.5V of current to the board's DC regulator. This input voltage is much smaller than my initial design's 11V battery, but we also need to consider that the motors require 6V instead of 12V and that my lack of experience with power circuits probably led me to overestimate. My initial DC/DC converter was still designed to bring the input 11V down to the needed 3.3V, and the DC regulator in this design is capable of brining down a voltage as high as 20V so it might still have been able to work with the old battery pack. Regardless, using a smaller battery pack is ideal because it allows for rechargable betteries and is ultimately cheaper.  

<p align="center"><img src="https://m.media-amazon.com/images/I/61CktcC0ZSL._AC_SS450_.jpg" alt="Main Power Supply" width="200"/></p>  

[Main Power Supply Store Page](https://www.amazon.com/gp/product/B07QV8GYV3/ref=ppx_yo_dt_b_asin_title_o05_s02?ie=UTF8&psc=1)  
Cost: $23  

[CR2032 Store Page](https://www.amazon.com/gp/product/B071D4DKTZ/ref=ppx_yo_dt_b_asin_title_o05_s00?ie=UTF8&psc=1)  
Cost: $6 for 10 (only 1 is needed)  

## Frame
Similar to the initial design, the frame is designed to be 3D printed for free on campus. More details can be found in the [CAD model](https://github.com/monk200/Segbot/tree/main/Frame/Reworked%20Frame%20Design) section.  

Cost: $0  

## Other Parts
As mentioned in the section on the new microcontroller, it has three quadrature encoders built in so the LS7366R is no longer needed. However, Prof. Block designed the PCB to have an external real-time clock (the BQ32000) connected to an external 32.768 kHz crystal oscillator. I don't have the specific part number of the cyrstal oscillator used but it does need to be one with two through-hole leads such as the ECS-.327-12.5-8X. The CR2032 battery directly powers the BQ32000.  

[BQ32000 Datasheet](https://www.ti.com/lit/ds/symlink/bq32000.pdf?HQS=dis-mous-null-mousermode-dsf-pf-null-wwe&ts=1621397103494&ref_url=https%253A%252F%252Fwww.mouser.com%252F)  
[BQ32000 Store Page](https://www.mouser.com/ProductDetail/Texas-Instruments/BQ32000DR?qs=IK5e5L0zOXhHwF090YgGcg%3D%3D&gclid=CjwKCAjwqIiFBhAHEiwANg9szpL4yjimNMDo_NLrpCCS9bDkUOz3TQVJnjl3wzBTYj0lHHZXMJg7xhoC9sAQAvD_BwE)  
Cost: $2  

[ECS-.327-12.5-8X Datasheet](https://ecsxtal.com/store/pdf/ECS-3x8X%202x6X%201X5X.pdf)  
[ECS-.327-12.5-8X Store Page](https://www.digikey.com/en/products/detail/ecs-inc/ECS-327-12-5-8X/827614)  
Cost: $0.30  
