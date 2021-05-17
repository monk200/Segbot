# Parts List for the Initial Design
The first portion of this project initially allowed for the flexibility to design a Segbot of any size, but because I was intending to build it eventually I went in trying to pick cost-effective hobbyist parts. Below are details on the process of selecting each part.   

## Microcontroller
The original design was intended to use the MSP430G2553 microcontroller (on the TI development board) because it was used earlier in the course for homework projects and tinkering so it was a part I already owned and was familiar with. The MSP430 controller is a low-power, high-performance, 16-bit general purpose controller. This particular MSP430 chip has 512 bytes SRAM, 24 I/O pins, 8 ADC channels with 10 bit resoultion, and two 16-bit timers. Below is a simple pinout of the launchpad board used:  
<p align="center"><img src="http://embedded-lab.com/blog/wp-content/uploads/2017/08/MSP430G2553-Pinmap.png" alt="MSP430 Pinout" width="800"/></p>  

[Texas Instruments Page](https://www.ti.com/product/MSP430G2553#product-details##params)  
[Chip Datasheet](https://www.ti.com/lit/ds/symlink/msp430g2553.pdf)  
[Launchpad Datasheet](https://www.ti.com/lit/ug/slau772a/slau772a.pdf?ts=1621210205139&ref_url=https%253A%252F%252Fwww.google.com%252F)  
[Store page](https://www.digikey.com/en/products/detail/texas-instruments/MSP-EXP430G2ET/9608004?utm_adgroup=Texas%20Instruments&utm_source=google&utm_medium=cpc&utm_campaign=Smart%20Shopping_Supplier_Texas%20Instruments&utm_term=&utm_content=Texas%20Instruments&gclid=CjwKCAjwhYOFBhBkEiwASF3KGaN2kOassUfhETN19iC_mOvuDlhFtcEphUyQ9jJh-qUR7a-CiZiPixoCr_EQAvD_BwE)  

Cost: $12 on digikey.com, $0 since I recieved it from the course  

## Motor Control
This was the first time I had experience choosing a motor and battery to begin creating a project (as opposed to having a starting kit). From the start, I knew I was searching for a 6-12V DC hobby motor because that was the kind of motors I've used before for robot car projects. Choosing a DC motor over an AC motor here is a no-brainer because AC motors need to be driven by AC current and tend to be used in stationary, high-load situations. Generally on my other robot cars, a 6V motor was sufficient but because of the added weight of the Segbot body it might be necessary to increase the voltage. Further calculations for this can be seen below, using estimated parameters from the rest of the Segbot build. Exploring other robot car kits and projects also led me to belive that the exact type of motor needed should be a micro gear reduction motor. Further research into why showed that often times DC motors rotate significantly faster than necessary for most applications and difficult to control, so adding the gearbox reduces the speed to a reasonable level and increases the motor's torque. Another consideration when choosing the motor is its compatibility with an optical encoder and wheels. Installing an optical encoder to a motor adds a more intensive task with notable room for error so immediately I took the luxury of not considering any motors that did not come pre-installed with an optical encoder. While browsing the remaining options, it was clear that I could also take the luxury of choosing a motor that also came with its own wheel. This continued to simplify my work because I didn't need to match up shaft sizes or be concerned about the width of the motor. Ultimately, the approach of getting the whole subassmbly together did make the final choice cheaper than the alternatives. The chosen motor subassembly was the Machifit 25GA370 DC 12V Micro Gear Reduction Motor and its details can be found below.  

<p align="center">
<img src="https://imgaz2.staticbg.com/thumb/large/oaupload/ser1/banggood/images/1B/45/d25b0465-7069-46fe-aa34-dc4d4eb2a95e.JPG.webp" alt="Motor Subassembly" width="200"/>
<img src="https://raw.githubusercontent.com/Arduinolibrary/DFRobot_12V_DC_Motor_350RPM_with_Encoder/master/Motor_interface.png" alt="Encoder Pinout" width="400"/>
</p>  

[Store Page](https://www.banggood.com/Machifit-25GA370-DC-12V-Micro-Gear-Reduction-Encoder-Motor-with-Mounting-Bracket-and-Wheel-p-1532242.html?rmmds=search&ID=6157425&cur_warehouse=CN)  
Cost: $40 for two motors  

An H-bridge is essential to be able to control the direction of the motor. The stall current of the chosen motor corresponds to the maximum continuous current the H-bridge needs to produce. The chosen motor's stall current is 1.8A so the H-bridge needs to have a peak output current that is sufficient. Searching [TI's selection of motor drivers](https://www.ti.com/motor-drivers/brushed-dc-bdc-drivers/overview.html) led to many seemingly suitable options. Because most students in the course were struggling selecting an H-bridge they could be confident would work, Prof. Block suggested a few options that he was familiar with and knew would work for different Segbot sizes. In my hobby-sized Segbot, he recommended the DRV8874.

[Chip Datasheet](https://www.ti.com/lit/ds/symlink/drv8874.pdf?ts=1621215513855&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FDRV8874)  
[Texas Instruments Page](https://www.ti.com/product/DRV8874)  
Cost: $2.50 for two H-bridges  

## Sensors
Aside from the motor encoders, the only other sensors required for the balancing control are an accelerometer and gyroscope. This part was actually extremely easy to find because there is a popular Arduino module that combines both sensors, and since it's made for the Arduino it will also easily be able to be used with the MSP430. The module chosen was the MPU-6050:  

<p align="center"><img src="https://imgaz1.staticbg.com/thumb/large/upload/2012/jiangjunchao/SKU080242h.JPG.webp" alt="Accelerometer and Gyroscope" width="200"/></p>  

[Chip Datasheet](http://myosuploads3.banggood.com/products/20190227/20190227200210MPU-6050InvenSense.pdf)  
[Store page](https://www.banggood.com/Geekcreit-6DOF-MPU-6050-3-Axis-Gyro-With-Accelerometer-Sensor-Module-p-80862.html?akmClientCountry=America&rmmds=search&cur_warehouse=CN)  
Cost: $4  

## Battery
Again since I didn't have much experience determining what kind of battery a system needed, I was guided by Prof. Block to choose a lithium ion battery. He advised that it should have 2-3 cells and be rated for 1500-2500 mAh. The LiPo battery is ideal because it is lightweight and rechargable, even though it is important to be very careful to not short the leads. The battery I ended up choosing was specifically for hobby applications and its details are shown below:  

[Store page](https://www.amazon.com/Zeee-3000mAh-Connector-Airplane-Helicopter/dp/B07VLR5ZW9/ref=sr_1_4_sspa?dchild=1&keywords=3S+Lipo&qid=1587735417&sr=8-4-spons&psc=1&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUExTkZCN09SVk9HOU1MJmVuY3J5cHRlZElkPUEwMjUzNjMwT1hHU1FCSEU5UFJXJmVuY3J5cHRlZEFkSWQ9QTA3NTY5NzgxRzdXSkRVNktWWVhMJndpZGdldE5hbWU9c3BfYXRmJmFjdGlvbj1jbGlja1JlZGlyZWN0JmRvTm90TG9nQ2xpY2s9dHJ1ZQ==)  
Cost: $40 for two, though only one is needed  

Lastly, a DC-DC converter would be needed to scale the battery power down to power the MSP430. Since the battery chosen provides 11.1V and the MSP430 needs a 3.3V input, the chosen converter needs to fit that range. It was easy enough to find the OKI-78SR, with up to a 36V input and a 3.3V output:  

[Store page](https://www.digikey.com/en/products/detail/murata-power-solutions-inc/OKI-78SR-3.3-1.5-W36-C/2259780)  
[Chip Datasheet](https://www.murata.com/products/productdata/8807037992990/oki-78sr.pdf?1583754815000)  
Cost: $4.50  

## Frame
Since I had free access to a 3D printer while in school, the body of the Segbot was planned to be 3D printed. This firstly creates an opportunity to work on my CAD skills and learn to use a 3D printer, but also allows the flexibility to create a custom-shaped body. More details can be found in the [CAD model]() section.  

Cost: $0

## Other Parts
A few parts needed for the assembly were provided by Prof. Block without detailed explanation. The first is the LS7366R Quadrature Counter, which is used to communicate with the MSP430's SPI. The other part is a crystal oscillator whose pinout was provided but specific part number was not. The connections between these parts and the controller's SPI peripheral can be seen on the last page of [this link](http://coecsl.ece.illinois.edu/ge423/LS7366CodeHandout.pdf). The first three pages contain the code needed to initialize the SPI on the microcontroller.  

[LS7366R Datasheet](https://lsicsi.com/datasheets/LS7366R.pdf)  

## Sources
https://www.powerelectric.com/motor-resources/motors101/what-are-dc-motors-usually-gear-motors  
https://www.engineersgarage.com/tech-articles/choosing-motor-for-robots/  
https://rogershobbycenter.com/lipoguide  
