#include "BlueTooth.h"
#include "string.h"
#include "robot.h"
#include "robot_def.h"

extern ROBOT_T*robot_t;


//手机发出指令，对比正确后传输
uint8_t cmd_information[]="information";
uint8_t cmd_speed[]="speed";
uint8_t cmd_motor_on[]="motoron";
uint8_t cmd_motor_off[]="motoroff";
uint8_t cmd_run_patrol[]="patrol";
uint8_t cmd_run_bluetooth[]="bluetooth";
uint8_t successfully[]="successfully\n";

uint8_t wrong_cmd[]="WRONG CMD!\nfor Motor and Run status:information\nfor speed:speed\nfor motor:motoron or motoroff\nfor runmode:patrol or bluetooth";



//蓝牙串口接受初始化
void USER_DMA_Init(void)
{
	//HAL库BUG，先deinit在init
	HAL_DMA_DeInit(*BlueTooth_Port.hdmarx);
	HAL_DMA_DeInit(*BlueTooth_Port.hdmatx);
	HAL_DMA_Init(*BlueTooth_Port.hdmarx);
	HAL_DMA_Init(*BlueTooth_Port.hdmatx);
	__HAL_UART_ENABLE_IT(BlueTooth_Port,UART_IT_IDLE);//设置为空闲中断
	HAL_UART_Receive_DMA(BlueTooth_Port,(uint8_t*)robot_t->remotedata,100u);	//文本接受数据
}


//自定义蓝牙串口通信数据接受（采用DMA+空闲中断）
void USER_IDLE_Rxcallback(UART_HandleTypeDef *huart)
{
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))	//判断是否接受完数据
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);					//清空标志位，防止一直中断
		USART1->SR;                                	//清空SR寄存器
		USART1->DR;                                 //清空DR寄存器
		HAL_UART_DMAStop(huart);									//关闭dma防止数据错乱
	}
	else return;
	
	//更据头尾帧判断是遥控数据还是对话数据
	if((int)robot_t->remotedata->data[0]==0xa5&&(int)robot_t->remotedata->data[DataLength-1]==0x5a)
	{
		RomoteDataSolve(robot_t->remotedata);
	}
	else BlueToothTalk(robot_t->remotedata,huart);
	
	//保留本次信息
	strcpy((char*)robot_t->remotedata->lastdata,(char*)robot_t->remotedata->data);
	memset(robot_t->remotedata,0,sizeof(BlueToothdata));
	HAL_UART_Receive_DMA(huart,(uint8_t*)robot_t->remotedata,100u);	//再次使能接受
}



//蓝牙对话模式
void BlueToothTalk(BlueToothdata*remotedata,UART_HandleTypeDef *huart)
{
	//通过拼接方式返回机器人信息，一次性传输
	if(strcmp((char*)remotedata,(char*)cmd_information)==0)
	{
		uint8_t Motor[30]="Motor:";
		uint8_t On[5]="on\n";
		uint8_t Off[6]="off\n";
		uint8_t Mode[30]=" RunMode:";
		uint8_t Mode_BT[12]="BlueTooth\n";
		uint8_t Mode_Pa[9]="Patrol\n";
		
		if(robot_t->MotorMode==MotorOff)		strcat((char*)Motor,(char*)Off);
		else		strcat((char*)Motor,(char*)On);
		if(robot_t->CtrMode==ModeBlueTooth)	strcat((char*)Mode,(char*)Mode_BT);
		else 		strcat((char*)Mode,(char*)Mode_Pa);
		strcat((char*)Motor,(char*)Mode);
		HAL_UART_Transmit(huart,Motor,sizeof(Motor),20);
	}
	
	//返回期望和反馈速度
	else if(strcmp(	(char*)remotedata,(char*)cmd_speed)==0)
	{
		uint8_t decimal_piont[]=".";
		uint8_t SpeedRef[30]="speed_ref:";
		uint8_t SpeedFdb[13]="\nspeed_fdb:";
		
		uint8_t speed_ref_integer_ASCII=(int)robot_t->robot_config->v_ref+48;
		uint8_t speed_ref_decimal_ASCII=(int)(robot_t->robot_config->v_ref*10)%10+48;
		uint8_t speed_fdb_integer_ASCII=(int)robot_t->robot_config->v_fdb+48;
		uint8_t speed_fdb_decimal_ASCII=(int)(robot_t->robot_config->v_fdb*10)%10+48;
		
		//期望速度
		strcat((char*)SpeedRef,(char*)(&speed_ref_integer_ASCII));
		strcat((char*)SpeedRef,(char*)decimal_piont);
		strcat((char*)SpeedRef,(char*)(&speed_ref_decimal_ASCII));

		//反馈速度
		strcat((char*)SpeedFdb,(char*)(&speed_fdb_integer_ASCII));
		strcat((char*)SpeedFdb,(char*)decimal_piont);
		strcat((char*)SpeedFdb,(char*)(&speed_fdb_decimal_ASCII));

		strcat((char*)SpeedRef,(char*)SpeedFdb);
		HAL_UART_Transmit(huart,SpeedRef,sizeof(SpeedRef),20);
	}

	//设置电机的开闭
	else if (strcmp((char*)remotedata,(char*)cmd_motor_on)==0)
	{
		robot_t->MotorMode=MotorOn;
		HAL_UART_Transmit(huart,successfully,sizeof(successfully),20);
	}
	else if(strcmp((char*)remotedata,(char*)cmd_motor_off)==0)
	{
		robot_t->MotorMode=MotorOff;
		HAL_UART_Transmit(huart,successfully,sizeof(successfully),20);
	}
	
	//设置运动控制的模式
	else if (strcmp((char*)remotedata,(char*)cmd_run_patrol)==0)
	{
		robot_t->CtrMode=ModePatrol;
		HAL_UART_Transmit(huart,successfully,sizeof(successfully),20);
	}
	else if (strcmp((char*)remotedata,(char*)cmd_run_bluetooth)==0)
	{
		robot_t->CtrMode=ModeBlueTooth;
		HAL_UART_Transmit(huart,successfully,sizeof(successfully),20);
	}
	//错误控制信息
	else
	{
		HAL_UART_Transmit(huart,wrong_cmd,sizeof(wrong_cmd),20);
	}
}


void RomoteDataSolve(BlueToothdata*remotedata)
{
	//判断是否接受的正确数据
	int receive_flag=0;

	//校验检查
	int sum=0;
	for(int i=1;i<DataLength-2;i++)
	{
		sum+=(int)remotedata->data[i];
	}
	sum=sum&0xff;
	if(sum==(int)remotedata->data[DataLength-2])
	{
		receive_flag=1;
	}
	//数据正常，进行解算
	if(receive_flag)
	{
		remotedata->x=(remotedata->data[2]<<8)|(remotedata->data[1]);
		remotedata->y=(remotedata->data[4]<<8)|(remotedata->data[3]);
		remotedata->w=(remotedata->data[6]<<8)|(remotedata->data[5]);
		
		//如果打开电机且赋值且为蓝牙控制模式，则使用遥控数据
		if(robot_t->MotorMode==MotorOn&&robot_t->CtrMode==ModeBlueTooth)
		{
			robot_t->robot_config->vx=(remotedata->x-1000)/320.0f;
			robot_t->robot_config->vy=(remotedata->y-1000)/320.0f;
			robot_t->robot_config->w= (remotedata->w-1000)/100.0f;
		}
	}
}
