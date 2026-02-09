Self-Balancing Robot
Elektor project 240134

Board: ESP32S3 Dev Module

!!! Compile with ESP32 Arduino Core version 2.0.17
!!! Version 3 and higher are not compatible with the Dabble Bluetooth controller library.
!!!
!!! GFX_Library_for_Arduino v1.4.6 is not included and must be installed with the Arduino IDE library manager.
!!! More recent versions have a problem with the ESP32 SPI library from core 2.0.17 and will not compile.


Required libraries
==================
DabbleESP32                     Included, adapted from https://github.com/STEMpedia/DabbleESP32
DFRobot_INA219                  Included, https://github.com/DFRobot/DFRobot_INA219
GFX_Library_for_Arduino v1.4.6  NOT included, https://github.com/moononournation/Arduino_GFX
I2Cdev                          Included, https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev
JPEGDEC                         Included, https://github.com/bitbank2/JPEGDEC
LMotorController                Included, adapted from https://github.com/rppelayo/arduino-self-balancing-robot
MPU6050                         Included, https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
PID_v1                          Included, https://github.com/br3ttb/Arduino-PID-Library
