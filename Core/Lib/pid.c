#include "pid.h"
#include "math.h"
#include "tim.h"
#include "robot.h"
#include "stdlib.h"


extern MOTOR_T*robot_t;

//为config指针创建对象
CONFIG_T*ConfigCreat(void)
{
	CONFIG_T*obj=(CONFIG_T*)malloc(sizeof(CONFIG_T));
	if(obj==NULL)
	{
		return NULL;
	}
	return obj;
}

//为pid指针创建对象
PID_T*PidCreat(void)
{
	PID_T*obj=(PID_T*)malloc(sizeof(PID_T));
	if(obj==NULL)
	{
		return	NULL;
	}
	obj->config=ConfigCreat();
	return obj;
}


//PID参数初始化
void PID_Init(PID_T*pid,float kp,float ki,float kd,float bias_max,int16_t output_max)
{
	pid->config->kp=kp;
	pid->config->ki=ki;
	pid->config->kd=kd;

	pid->config->output=0;
	pid->config->output_max=output_max;	

	pid->config->bias_sum=0;
	pid->config->bias_max=bias_max;
	
	pid->pulse_fdb=0;
	pid->pulse_ref=100;
}


//利用增量式PID实现闭环控制电机输出
void PID_Calculate(PID_T*pid_t)
{
	//误差数据更新
	pid_t->config->bias[2]=pid_t->config->bias[1];
	pid_t->config->bias[1]=pid_t->config->bias[0];
	pid_t->config->bias[0]=pid_t->pulse_ref-pid_t->pulse_fdb;

	//积分上限判断
	pid_t->config->bias_sum+=pid_t->config->bias[0];		
	if(pid_t->config->bias_sum>pid_t->config->bias_max)		pid_t->config->bias_sum=pid_t->config->bias_max;
	if(pid_t->config->bias_sum<-pid_t->config->bias_max)	pid_t->config->bias_sum=-pid_t->config->bias_max;

	//pid计算
	pid_t->config->output=pid_t->config->kp*pid_t->config->bias[0]+pid_t->config->bias_sum*
								pid_t->config->ki+pid_t->config->kd*(pid_t->config->bias[1]-pid_t->config->bias[0]);
	//防止输出过大
	if(pid_t->config->output>pid_t->config->output_max)		pid_t->config->output=pid_t->config->output_max;
	if(pid_t->config->output<-pid_t->config->output_max)	pid_t->config->output=-pid_t->config->output_max;
}

