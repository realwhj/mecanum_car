#include "line_patrol.h"
#include "gpio.h"
#include "robot.h"
#include "robot_def.h"

int right2,right1,middle,left1,left2;
extern ROBOT_T*robot_t;


//白色返回0，黑色返回1
void LinePatrolGet(void)
{
	right2=HAL_GPIO_ReadPin(Patrol_Port_1,OUTPUT_R2);
	right1=HAL_GPIO_ReadPin(Patrol_Port_2,OUTPUT_R1);
	middle=HAL_GPIO_ReadPin(Patrol_Port_3,OUTPUT_M);
	left1=HAL_GPIO_ReadPin(Patrol_Port_4,OUTPUT_L1);
	left2=HAL_GPIO_ReadPin(Patrol_Port_5,OUTPUT_L2);	
}
//运动状态确定直行，左右移，左右转
void RunStateGet(void)
{
	LinePatrolGet();
	if(left2&&left1&&middle&&right1&&right2)
	{
		robot_t->patrol_state=straghit;				//十字路口直行
	}
	if(middle&&left1&&right1&&!right2&&!left2)
	{
		robot_t->patrol_state=straghit;		//普通道路直行
		return;
	}
	if(!left1&&right2)
	{
		robot_t->patrol_state=right;		//左偏，右移
		return;
	}
	if(!right1&&left2)
	{
		robot_t->patrol_state=left;			//右偏，左移
		return;
	}
	if(!left2&&!left1&&middle&&right1&&!right2)
	{
		robot_t->patrol_state=turn_benright;  	//右转曲弯
	}
	if(!left2&&left1&&middle&&!right1&&!right2)
	{
		robot_t->patrol_state=turn_benleft;		//左转曲弯
	}
	if(right1&&right2&&middle&&left1&&!left2)
	{
		robot_t->patrol_state=turn_strright;			//右转直弯
		return;
	}
	if(left2&&left1&&middle&&right1&&!right2)
	{
		robot_t->patrol_state=turn_strleft;				//左转直弯		
		return;
	}
	else 
		robot_t->patrol_state=stop;				//其他情况停止下来
	
}

