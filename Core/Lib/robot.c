/*各引脚：
循迹：right2：PG14	right1：PG13	middle：PG12	left1：PG11	left2：PE2

电机：（TIM10:PF6 TIM11:PF7	TIM13:PF8	TIM14:PA5）
	驱动1:（3.3v:+/GND:GND/AB相(电机1)：PD13/PD12（tim4））
	左侧：STBY：3.3v	 GND:GND  PWMA:PF6  AIN1:PF11 AIN2:PB1			PWMB:PF7  BIN1:PD7	BIN2:PD6
	右侧:上下的VM/GND:电源+/-	VCC:3.3V	AO1/AO2:电机+/-(左侧接了gnd，右侧可不接了)
	驱动2:(3.3v:+/GND:GND/AB相：)
	左侧：STBY：3.3v	 GND:GND  PWMA:PF8  AIN1:PC10 AIN2:PG15			PWMB:PF9  BIN1:PD2	BIN2:PC11
	右侧：上下的VM/GND:电源+/-	VCC:3.3V	AO1/AO2:电机+/-(左侧接了gnd，右侧可不接了)
其他：
	TIM5(PA2):输出音乐

*/

#include "robot.h"
#include "stdlib.h"
#include "tim.h"
#include "OLED.h"
#include "music.h"
#include <string.h>


//为robot创建对象（采用链表）
ROBOT_T*RobotCreat(void)
{
	ROBOT_T*obj=(ROBOT_T*)malloc(sizeof(ROBOT_T));
	if(obj==NULL)
	{
		return NULL;
	}
	
	obj->robot_config=RobotConfigCreat();
	
	obj->remotedata=RemoteDataCreat();
	
	obj->motor1=MotorCreat();
	obj->motor2=MotorCreat();
	obj->motor3=MotorCreat();
	obj->motor4=MotorCreat();
	RobotParamInit(obj);
	return obj;
}

//为motor创建对象
MOTOR_T*MotorCreat(void)
{
	MOTOR_T* obj = (MOTOR_T*)malloc(sizeof(MOTOR_T));
	if(obj==NULL)	//创建失败，退出
	{
		return	NULL;
	}
	obj->pid_t=PidCreat();
	return obj;
}

//机器人基本参数创建
RobotConfig_T*RobotConfigCreat(void)
{
	RobotConfig_T*obj=(RobotConfig_T*)malloc(sizeof(RobotConfig_T));
	if(obj==NULL)
	{
		return NULL;
	}
	return obj;
}

BlueToothdata*RemoteDataCreat(void)
{
	BlueToothdata*obj=(BlueToothdata*)malloc(sizeof(BlueToothdata));
	if(obj==NULL)
	{
		return NULL;
	}
	return obj;
}

//机器人结构体初始化
void RobotParamInit(ROBOT_T*robot_t)
{
	robot_t->patrol_state=stop;
	robot_t->MotorMode=MotorOff;
	robot_t->CtrMode=ModeBlueTooth;

	RobotConfigInit(robot_t->robot_config);

	RemoteDataParamInit(robot_t->remotedata);
	
	MotorParamInit(robot_t->motor1);
	MotorParamInit(robot_t->motor2);
	MotorParamInit(robot_t->motor3);
	MotorParamInit(robot_t->motor4);
}


//电机参数初始化
void MotorParamInit(MOTOR_T*motor)
{
	static int cnt=1;
	motor->num=cnt;
	cnt++;
	
	motor->direction=2;
	motor->speed_r_ref=0;
	motor->speed_r_fdb=0;
	motor->vw_ref=0;
	motor->vw_fdb=0;
	PID_Init(motor->pid_t,6,0,0,2000,3000);
}

//机器人参数初始化
void RobotConfigInit(RobotConfig_T*robot_config)
{
	robot_config->a=0.16;
	robot_config->b=0.08;
	
	robot_config->vx=0;
	robot_config->vy=0;
	robot_config->w=0;
}

void RemoteDataParamInit(BlueToothdata*remotedata)
{
	memset(remotedata,0,sizeof(BlueToothdata));
}

//检测创建的成功or失败
void CreatMonitor(ROBOT_T*robot_t)
{
	if(robot_t==NULL)	//创建失败强制退出
	{
		exit(0);
	}
	else
	{
		InitShow();
		Music();
	}
}


//微秒级别的延时
void delay_us(uint16_t us)
{
	for(; us > 0; us--)
 {
		for(uint8_t i = 50; i > 0; i--)
		{
			;
		}
 }
}
