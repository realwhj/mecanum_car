//本文件实现引脚和实际功能的对应
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "tim.h"
#include "usart.h"
#include "i2c.h"
#include "gpio.h"

//TIM
//音乐
#define MUSIC_PORT &htim5
#define MUSIC_CHANNEL TIM_CHANNEL_3
//编码器
#define Encoder_Port_1 &htim4
#define Encoder_Port_2 &htim2
#define Encoder_Port_3 &htim8
#define Encoder_Port_4 &htim3
#define Encode_CHANNEL TIM_CHANNEL_ALL
//控制电机输出
#define Motor_Port_1 &htim10
#define Motor_CHANNEL_1 TIM_CHANNEL_1
#define Motor_Port_2 &htim11
#define Motor_CHANNEL_2 TIM_CHANNEL_1
#define Motor_Port_3 &htim13
#define Motor_CHANNEL_3 TIM_CHANNEL_1
#define Motor_Port_4 &htim2
#define Motor_CHANNEL_4 TIM_CHANNEL_1


//USART
//蓝牙串口
#define BlueTooth_Port &huart2

//IIC
//oled
#define OLED_Port &hi2c1


//GPIO
//循迹
#define Patrol_Port_1 GPIOG
#define Patrol_Port_2 GPIOG
#define Patrol_Port_3 GPIOG
#define Patrol_Port_4 GPIOE
#define Patrol_Port_5 GPIOE
#define OUTPUT_R2 GPIO_PIN_14
#define OUTPUT_R1 GPIO_PIN_13
#define OUTPUT_M  GPIO_PIN_12
#define OUTPUT_L1 GPIO_PIN_11
#define OUTPUT_L2 GPIO_PIN_2





#endif
