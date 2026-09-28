#ifndef __SR04_H
#define __SR04_H
#define Trig_GPIO_Port GPIOA
#define Trig_Pin GPIO_PIN_1
#include "main.h"

#include "stdio.h"
 
#define TRIG_H  HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_SET)
#define TRIG_L  HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_RESET)


void delay_us(uint32_t us);
void SR04_GetData(void);
 
#endif