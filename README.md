# Due32bit_GY521MPU_1_V4
/*
 Test By tinnakon kheowree  project 
 
 tinnakon_za@hotmail.com
 
 tinnakonza@gmail.com
 
 http://quad3d-tin.lnwshop.com/
 
 https://www.facebook.com/tinnakonza

 12/03/2561     write Due32bit_GY521MPU_1_V3  ,,2,3,4,5,6,7,8,9,10,11,// Roop time 1000Hz,,// Timer3 Interrupt handler
 
                                              ,velocity GPS Rotated Frame of arm gps
                                              
                                              ,Altitude control
                                              
 16/03/2561     write Due32bit_GY521MPU_1_V4  ,Observer_kalman_filter ,, Velocity_THR
 
support : Arduino 1.5.8   Arduino Due 32 bit  , MPU6050  MS5611

• Atmel SAM3X8E ARM Cortex-M3 CPU 32-bit a 84 MHz clock, ARM core microcontroller

• MPU6050 Gyro Accelerometer //I2C 400kHz nois gyro +-0.05 deg/s , acc +-0.04 m/s^2

• MS5611 Barometer//SPI

• HMC5883L Magnetometer //I2C_BYPASS ,I2C 400kHz

• GPS NEO-M8N //

Timer

timer 7 TC2-1  use read ppm

Quad-X

pin 9 FRONTL  M1CW        M2CCW  FRONTR pin 8

              \         / 
              
                \ --- /
                
                 |   |
                 
                / --- \
                
              /         \ 
              
pin 6 motor_BackL  M4 CCW      M3 CW  motor_BackR  pin 7

----------rx-----------  

A8 = PPM 8 CH
 */
