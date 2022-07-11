#ifndef BLUETOOTH_H
#define	BLUETOOTH_H

#include "usart.h"


#define DataLength 9

typedef struct bluetoothdata
{
	char data[30];
	char lastdata[30];
	short x;				//左右摇杆值
	short y;				//前后遥感值
	short w;				//旋转值
}BlueToothdata;

void USER_DMA_Init(void);
void USER_IDLE_Rxcallback(UART_HandleTypeDef *huart);

//蓝牙对话模式
void BlueToothTalk(BlueToothdata*remotedata,UART_HandleTypeDef *huart);

//数据解析
void RomoteDataSolve(BlueToothdata*remotedata);

#endif
