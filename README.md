![](https://github.com/katbinaris/nanofoc_devkit/blob/main/Images/IMG_20230418_120543-01.jpeg?raw=true)
![](https://github.com/katbinaris/nanofoc_devkit/blob/main/Images/IMG_20230418_120543-01.jpg?raw=true)

#### Product Description:
Starting your first Field Oriented Control (FOC) project might be daunting, especially if you're new to the field.

As makers, we frequently want to jump right into the action and begin writing the firmware without the hassle/need of selecting the right Microcontroller, Magnetic Sensor, and Motor Driver, then configuring and calibrating it with an abundance of cables that renders your projects too bulky to use in your creations.


The NanoFOC is little enough to attach on the back of most small BLDC gimbal motors (Even easier with Universal BLDC Motor Adapter Kit), making it an excellent choice for compact projects requiring maximum efficiency and precision.


#### Features:
- Espressif ESP32 S3 Microcontroller 4MB Flash, 2MB PSRAM 
- Trinamic TMC6300 low power 6PWM BLDC Driver
- MPS MagAlpha MA710 or MAQ430 rotary magnetic position sensor
- BiCMOS 3.3V LDO
- 11 Programmable GPIO's (SPI, PWM, ADC, RTC etc.)
- I2C & UART pins available
- USB Powered (Compatible with USB PD 2.0 - 5V @ 3A)
- USB OTG
- AUX Power via VIN pads 

#### Pinout Cheat Sheet:
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


#### Power Options:
Device is designed to run of 5V @ 1.5A Max.
There are two ways you can power the board 

1. USB Type C - Power delivery is capped at 5V3A with usage of 5.1K resistors on CC1 and CC2 pads. It is recommended to use USB that is PD2.0 compatible.
It is possible to power device off 500mA capable USB port - however this is not recommended.

2. External VIN  - This connection bypass L1 and  D4 connecting directly to Driver and LDO VS pin. The recommended input voltage is 5V and maximum allowed is 6V - which is restricted by max VIN of ST1L05C LDO.


