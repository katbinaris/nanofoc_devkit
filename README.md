![](/Images/NanoFOC_GH.jpg?raw=true)

#### Product Description:
Starting your first Field Oriented Control (FOC) project might be daunting, especially if you're new to the field.

As makers, we frequently want to jump right into the action and begin writing the firmware without the hassle/need of selecting the right Microcontroller, Magnetic Sensor, and Motor Driver, then configuring and calibrating it with an abundance of cables that renders your projects too bulky to use in your creations.


The NanoFOC is little enough to attach on the back of most small BLDC gimbal motors (Even easier with Universal BLDC Motor Adapter Kit), making it an excellent choice for compact projects requiring maximum efficiency and precision.

Purchase Link: https://store.binaris.io/products/nanofoc-devkit

### Features:
- Espressif ESP32 S3 Microcontroller 4MB Flash, 2MB PSRAM 
- Trinamic TMC6300 low power 6PWM BLDC Driver
- MPS MagAlpha MA710 or MAQ430 rotary magnetic position sensor
- BiCMOS 3.3V LDO
- 11 Programmable GPIO's (SPI, PWM, ADC, RTC etc.)
- I2C & UART pins available
- USB Powered (Compatible with USB PD 2.0 - 5V @ 3A)
- USB OTG
- AUX Power via VIN pads 

### Pinout Cheat Sheet:
|  Pin | Function  | On GPIO Matrix? |
| ------------ | ------------ | ----------- |
| GPIO 01 | I2C SCL | YES |
| GPIO 02 |  I2C SDA | YES |
| GPIO 04 |  SPI CS -MAGNETIC SESNOR | NO |
| GPIO 05 |  SPI MOSI -MAGNETIC SESNOR | NO |
| GPIO 06 |  SPI SCLK -MAGNETIC SESNOR | NO |
| GPIO 07 |  SPI MISO -MAGNETIC SESNOR | NO |
| GPIO 08 |  I/O  | YES |
| GPIO 09 |  I/O  | YES |
| GPIO 10 |  VL - PhaseB Low  | NO |
| GPIO 11 |  WL - PhaseC Low  | NO |
| GPIO 12 |  UL - PhaseA Low  | NO |
| GPIO 13 |  WH - PhaseC High  | NO |
| GPIO 14 |  VH - PhaseB High  | NO |
| GPIO 17 |  I/O  | YES |
| GPIO 18 |  I/O  | YES |
| GPIO 19 |  USB D- | NO |
| GPIO 20 |  USB D+ | NO |
| GPIO 21 |  UH PhaseA High | NO |
| GPIO 38 |  I/O  | YES |
| GPIO 39 |  I/O  | YES |
| GPIO 40 |  I/O  | YES |
| GPIO 41 |  I/O  | YES |
| GPIO 42 |  I/O  | YES |
| GPIO 43 | UART0 TX or I/O  | YES |
| GPIO 44 | UART0 RX or I/O | YES |
| GPIO 43 | UART0 TX  | YES |
| GPIO 44 | UART0 RX | YES |
| GPIO 47 | I/O | YES |
| GPIO 48 | I/O | YES |

### Board Layout:

![](/Images/PCB_LAYOUT.jpg)

### Bill Of Materials

| Component | References | Footprint | Quantity | Link | Price |
| -------------- | -------------- | -------------- | -------------- | -------------- | -------------- | 
| 100nF | C1, C2, C4, C8, C12 | 0603 | 5 | https://eu.mouser.com/ProductDetail/KEMET/C0603C104K8RAC | 0,55 € |
| 4.7uF | C5, C6 | 0805 | 2 | https://eu.mouser.com/ProductDetail/TDK/C2012X7R1A475K125AC | 0,46 € |
| 10uF | C7, C13 | 0603 | 2 | https://eu.mouser.com/ProductDetail/Samsung-Electro-Mechanics/CL10A106KP8NNWC | 0,24 € |
| 22uf | C9 | 0805 | 1 | https://eu.mouser.com/ProductDetail/Murata-Electronics/GRM21BC81C226ME44L | 0,39 € |
| 22uf | C11 | 0603 | 1 | https://eu.mouser.com/ProductDetail/Murata-Electronics/GRM188R61A226ME15J | 0,17 € |
| 100K | R1-R6 | 0603 | 6 | https://eu.mouser.com/ProductDetail/Bourns/CMP0603-FX-1003ELF | 1,20 € |
| 300R | R7, R8 | 0603 | 2 | https://eu.mouser.com/ProductDetail/Panasonic/ERJ-UP3F3000V | 0,48 € |
| 150m | R9, R10 | 0603 | 2 | https://eu.mouser.com/ProductDetail/Susumu/KRL0816D-C-R150-F-T5 | 1,20 € |
| 340R | R14 | 0603 | 1 | https://eu.mouser.com/ProductDetail/Panasonic/ERJ-3EKF3400V | 0,09 €|
| 5.1K | R13 | 0603 | 2 | https://eu.mouser.com/ProductDetail/Panasonic/ERJ-P03F5101V | 0,70 € |
| 10K | R15 | 0603 | 1 | https://eu.mouser.com/ProductDetail/ROHM-Semiconductor/SFR03EZPF1002 | 0,13 € |
| 100Mhz 400Ohm 1.5A | L1 | 0805 |  1 | https://eu.mouser.com/ProductDetail/Laird-Performance-Materials/MI0805K400R-10 | 0,19 €  |
| NUF2042XV6T1G | D3 | SOT-563 | 1 | https://www.mouser.com/ProductDetail/onsemi/NUF2042XV6T1G | 0,44 € |
| CUS15S30 | D4 | SOD-323 | 1 | https://www.mouser.com/ProductDetail/Toshiba/CUS15S30H3F | 0,33 € |
| Yellow/Green Mono LED | D5, DRV | 0603 | 2 | https://eu.mouser.com/ProductDetail/ROHM-Semiconductor/SML-D12M8WT86 | 0,68 € |
| ESP32-S3 N4R2 1U | U1 | ESP32-S3-WROOM-1U | 1 | https://www.mouser.com/ProductDetail/Espressif-Systems/ESP32-S3-WROOM-1U-N8R2 | 3,38 € |
| TMC6300 LA | U2 | QFN20 | 1 | https://eu.mouser.com/ProductDetail/ADI-Trinamic/TMC6300-LA-T | 2,13 € |
| MPS MA710| U3 | QFN16 | 1 | https://www.mouser.com/ProductDetail/Monolithic-Power-Systems-MPS/MA710GQ-Z | 6,15 € |
| Alps SKTDLDE010 | SW1, SW2 | SKTDLDE010 | 2 | https://eu.mouser.com/ProductDetail/Alps-Alpine/SKTDLDE010 | 1,46 € |
| Amber Mono LED | ERR | 0603 | 1 | https://eu.mouser.com/ProductDetail/Wurth-Elektronik/150060AS75000 | 0,14 € |
| ST1L05CPU33R LDO | IC1 | DFN6 | 1 | https://www.mouser.com/ProductDetail/STMicroelectronics/ST1L05CPU33R | 1,18 € |
| G-Switch_GT-USB-7010ASV | P1 | 7010ASV | 1 | https://www.mouser.com/ProductDetail/GCT/USB4105-GF-A | 0,75 € | 
| | | | **38** | | **22,44 € / 23.50 $** |

 
### Power Options:
Device is designed to run of 5V @ 1.5A Max.
There are two ways you can power the board 

1. USB Type C - Power delivery is capped at 5V3A with usage of 5.1K resistors on CC1 and CC2 pads. It is recommended to use USB that is PD2.0 compatible.
It is possible to power device off 500mA capable USB port - however this is not recommended.

2. External VIN  - This connection bypass L1 and  D4 connecting directly to Driver and LDO VS pin. The recommended input voltage is 5V and maximum allowed is 6V - which is restricted by max VIN of ST1L05C LDO.

### First Time Setup
Development Kit comes with no Firmware.
Since ESP32-S3 comes with USB-OTG/USB-Serial integrated into MCU the assumption is that this would be a preferred method of code upload.
If you are connecting your device to computer for the first time it is critical to put device into DFU mode.

**In order to do that you need to:**
1. Connect Development Board to USB port - *On windows machines you may notice that USB detection sound goes on and off in loop - Please ignore that for now*
2. Press and hold **BOOT** switch
3. While holding **BOOT** press and hold **RESET** switch for about **2 seconds**
4. Release **RESET**
5. Release **BOOT**
6. You are now in **DFU mode** and you can use Arduino IDE or PlatformIO to upload your first sketch.

Alternatively you may use UART pins which are available on GPIO Matrix to upload your code the legacy way.
In order to do that you need external TTL Converter.

### Projects using NanoFOC:

SmartKnob by Scott Bezek -> https://github.com/scottbez1/smartknob#nanofoc-3rd-party

