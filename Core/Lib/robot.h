#ifndef ROBOT_H
#define ROBOT_H

#include "pid.h"
#include "line_patrol.h"
#include "BlueTooth.h"
#include "usart.h"


//电机开关
typedef enum motormode
{
	MotorOff=0,
	MotorOn,
}MotorMode_E;

//控制模式选择
typedef enum	ctrmode
{
	ModeBlueTooth=0,	//蓝牙打错了，不想改了。。。。
	ModePatrol,
}CtrMode_E;

//电机相关信息结构体
typedef struct motor
{
	int num;				//该电机的序号
	int direction;			//设0为正转，1为反转，2为停止
	float speed_r_ref;		//期望轮子转速(rad/s,弧度值)
	float speed_r_fdb;  	//反馈轮子转速
	float vw_ref;			//期望轮子的线速
	float vw_fdb;			//反馈轮子的线速
	PID_T*pid_t;			//PID结构体
}MOTOR_T;

//机器人相关参数
typedef struct robotconfig
{
	float vx;			//一系列期望速度
	float vy;
	float w;
	float v_ref;		
	float vy_fdb;		//y方向反馈速度
	float vx_fdb;		//x方向反馈速度
	float w_fdb;		//w方向反馈速度
	float v_fdb;		//反馈速度
	float a;			//车长
	float b;			//车宽
}RobotConfig_T;

//机器人结构体
typedef struct robot
{
	RUN_STATE_E patrol_state;
	MotorMode_E MotorMode;
	CtrMode_E CtrMode;
	RobotConfig_T*robot_config;
	BlueToothdata*remotedata;
	MOTOR_T*motor1;
	MOTOR_T*motor2;
	MOTOR_T*motor3;
	MOTOR_T*motor4;
}ROBOT_T;


ROBOT_T*RobotCreat(void);
MOTOR_T*MotorCreat(void);
RobotConfig_T*RobotConfigCreat(void);
MotorMode_E*MotorModeCreat(void);
CtrMode_E*CtrModeCreat(void);
BlueToothdata*RemoteDataCreat(void);

void CreatMonitor(ROBOT_T*robot_t);

void RobotConfigInit(RobotConfig_T*robot_config);
void RobotParamInit(ROBOT_T*robot_t);
void MotorParamInit(MOTOR_T*motor);
void RemoteDataParamInit(BlueToothdata*remotedata);

void delay_us(uint16_t us);


#endif
