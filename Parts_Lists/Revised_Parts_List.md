# Parts List for the Revised Design
TBA 

## Microcontroller TBA
The original design was intended to use the MSP430G2553 microcontroller (on the TI development board) because it was used earlier in the course for homework projects and tinkering so it was a part I already owned and was familiar with. The MSP430 controller is a low-power, high-performance, 16-bit general purpose controller. This particular MSP430 chip has 512 bytes SRAM, 24 I/O pins, 8 ADC channels with 10 bit resoultion, and two 16-bit timers. Below is a simple pinout of the launchpad board used:  
<p align="center"><img src="http://embedded-lab.com/blog/wp-content/uploads/2017/08/MSP430G2553-Pinmap.png" alt="MSP430 Pinout" width="800"/></p>  

[Texas Instruments Page](https://www.ti.com/product/MSP430G2553#product-details##params)  
[Chip Datasheet](https://www.ti.com/lit/ds/symlink/msp430g2553.pdf)  
[Launchpad Datasheet](https://www.ti.com/lit/ug/slau772a/slau772a.pdf?ts=1621210205139&ref_url=https%253A%252F%252Fwww.google.com%252F)  
[Store page](https://www.digikey.com/en/products/detail/texas-instruments/MSP-EXP430G2ET/9608004?utm_adgroup=Texas%20Instruments&utm_source=google&utm_medium=cpc&utm_campaign=Smart%20Shopping_Supplier_Texas%20Instruments&utm_term=&utm_content=Texas%20Instruments&gclid=CjwKCAjwhYOFBhBkEiwASF3KGaN2kOassUfhETN19iC_mOvuDlhFtcEphUyQ9jJh-qUR7a-CiZiPixoCr_EQAvD_BwE)  

Cost: $12 on digikey.com, $0 since I recieved it from the course  

## Motor Control TBA
GM25-370: 6V gear motor, 250 RPM + encoder

Same H-bridge DRV8874.

[Chip Datasheet](https://www.ti.com/lit/ds/symlink/drv8874.pdf?ts=1621215513855&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FDRV8874)  
[Texas Instruments Page](https://www.ti.com/product/DRV8874)  
Cost: $2.50 for two H-bridges  

## Sensors
MPU-9250 instead

<p align="center"><img src="https://imgaz1.staticbg.com/thumb/large/upload/2012/jiangjunchao/SKU080242h.JPG.webp" alt="Accelerometer and Gyroscope" width="200"/></p>  

[Chip Datasheet](http://myosuploads3.banggood.com/products/20190227/20190227200210MPU-6050InvenSense.pdf)  
[Store page](https://www.banggood.com/Geekcreit-6DOF-MPU-6050-3-Axis-Gyro-With-Accelerometer-Sensor-Module-p-80862.html?akmClientCountry=America&rmmds=search&cur_warehouse=CN)  
Cost: $4  

## Battery
battery pack from amazon

[DC regulator](https://datasheetz.com/data/Integrated%20Circuits%20(ICs)/Voltage%20Regulators%20-%20Linear/TL1963A-33KTTR-datasheetz.html)  

## Frame TBA


## Other Parts
A few parts needed for the assembly were provided by Prof. Block without detailed explanation. The first is the LS7366R Quadrature Counter, which is used to communicate with the MSP430's SPI. The other part is a crystal oscillator whose pinout was provided but specific part number was not. The connections between these parts and the controller's SPI peripheral can be seen on the last page of [this link](http://coecsl.ece.illinois.edu/ge423/LS7366CodeHandout.pdf). The first three pages contain the code needed to initialize the SPI on the microcontroller.  

[LS7366R Datasheet](https://lsicsi.com/datasheets/LS7366R.pdf)  

32.768 kHz crystal oscillator  
relay https://pdf.voron.ua/files/pdf/relay/JQC-3F(T73).pdf  
[real time clock](https://www.mouser.com/ProductDetail/Texas-Instruments/BQ32000DR?qs=IK5e5L0zOXhHwF090YgGcg%3D%3D&gclid=CjwKCAjwqIiFBhAHEiwANg9szpL4yjimNMDo_NLrpCCS9bDkUOz3TQVJnjl3wzBTYj0lHHZXMJg7xhoC9sAQAvD_BwE)  
