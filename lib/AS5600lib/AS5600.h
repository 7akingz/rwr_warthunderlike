#ifndef __AS5600_H
#define __AS5600_H

#include "main.h"
#include "stdio.h"
#define cpr  (float)(2.0f*PI) 
#define Slave_Address 0x36<<1 //AS5600的I2C地址，左移一位是因为HAL库需要8位地址
#define Angle_High_Register 0x0C //角度高字节寄存器地址
#define Angle_Low_Register 0x0D //角度低字节寄存器
unsigned char write_reg(unsigned char reg,unsigned char* value,unsigned short len);
unsigned char read_register(unsigned char reg, unsigned char* data, unsigned short len);
float i2c_AS5600_read_rawangle(void);
float getAngle_Without_track(void);
float getAngle(void);

#endif // __AS5600_H  