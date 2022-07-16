#include "motor.h"
#include "pid.h"
#include "robot.h"
#include "math.h"
#include "stdlib.h"
#include "oled.h"
#include "robot_def.h"


extern ROBOT_T*robot_t;

float speed_fdb=0;

//机器人运动
void RobotRun(ROBOT_T*robot_t)
{
	if(robot_t->MotorMode==MotorOn)	//打开电机
	{
    RunMode(robot_t);
	}
  //停止电机转动
	else{
    RobotStop();
	}
}

//小车运动的方向
void RunMode(ROBOT_T*robot_t)
{
	robot_t->patrol_state=straghit;
	//根据巡线结果赋值三个自由度
	if(robot_t->CtrMode==ModePatrol)	//打开电机且为循迹模式否则进入蓝牙
	{
	 switch (robot_t->patrol_state)
    {
      case stop:
        RobotStop();
        break;
      case straghit:
				robot_t->robot_config->vy=3;
				robot_t->robot_config->vx=0;
				robot_t->robot_config->w=0;
        break;
      case left:
				robot_t->robot_config->vy=0;
				robot_t->robot_config->vx=3;
				robot_t->robot_config->w=0;
        break;
      case right:
				robot_t->robot_config->vy=0;
				robot_t->robot_config->vx=-3;
				robot_t->robot_config->w=0;
        break;
			case turn_strleft:
				robot_t->robot_config->vy=1;
        robot_t->robot_config->vx=0;
        robot_t->robot_config->w=2;
        break;
			case turn_strright:
        robot_t->robot_config->vy=1;
        robot_t->robot_config->vx=0;
        robot_t->robot_config->w=-2;
				break;
			case turn_benleft:
				robot_t->robot_config->vy=1;
        robot_t->robot_config->vx=0;
        robot_t->robot_config->w=4;
        break;
			case turn_benright:
        robot_t->robot_config->vy=1;
        robot_t->robot_config->vx=0;
        robot_t->robot_config->w=-4;
				break;
		}
	}

	
	MecanumCal(robot_t);
  DirectionGet(robot_t);
	//MotorCal(robot_t);
  MotorRun(robot_t);
}



//根据给出的各方向速度计算出各轮子的输出和各期望值(艹,结构体创多了。。。。。)
void MecanumCal(ROBOT_T*robot_t)
{
	//期望速度，忽略w方向
  robot_t->robot_config->v_ref=(float)sqrt(robot_t->robot_config->vy * robot_t->robot_config->vy + robot_t->robot_config->vx * robot_t->robot_config->vx);

	//麦克纳姆方程解算各轮子速度1、2、3、4分别为rf、lf、rb、lb
//  robot_t->motor1->vw_ref = robot_t->robot_config->vy + robot_t->robot_config->vx + robot_t->robot_config->w*(robot_t->robot_config->a+robot_t->robot_config->b);
//  robot_t->motor2->vw_ref = robot_t->robot_config->vy - robot_t->robot_config->vx - robot_t->robot_config->w*(robot_t->robot_config->a+robot_t->robot_config->b);
//  robot_t->motor3->vw_ref = robot_t->robot_config->vy - robot_t->robot_config->vx + robot_t->robot_config->w*(robot_t->robot_config->a+robot_t->robot_config->b);
//  robot_t->motor4->vw_ref = robot_t->robot_config->vy + robot_t->robot_config->vx - robot_t->robot_config->w*(robot_t->robot_config->a+robot_t->robot_config->b);
	robot_t->motor1->vw_ref = robot_t->robot_config->vy - robot_t->robot_config->vx - robot_t->robot_config->w*(robot_t->robot_config->a+robot_t->robot_config->b);
  robot_t->motor2->vw_ref = robot_t->robot_config->vy + robot_t->robot_config->vx + robot_t->robot_config->w*(robot_t->robot_config->a+robot_t->robot_config->b);
  robot_t->motor3->vw_ref = robot_t->robot_config->vy + robot_t->robot_config->vx - robot_t->robot_config->w*(robot_t->robot_config->a+robot_t->robot_config->b);
  robot_t->motor4->vw_ref = robot_t->robot_config->vy - robot_t->robot_config->vx + robot_t->robot_config->w*(robot_t->robot_config->a+robot_t->robot_config->b);


  robot_t->motor1->speed_r_ref = robot_t->motor1->vw_ref/(0.11*2*3.15149);      //转速（弧度值)
  robot_t->motor2->speed_r_ref = robot_t->motor2->vw_ref/(0.11*2*3.15149);
  robot_t->motor3->speed_r_ref = robot_t->motor3->vw_ref/(0.11*2*3.15149);
  robot_t->motor4->speed_r_ref = robot_t->motor4->vw_ref/(0.11*2*3.15149);

  robot_t->motor1->pid_t->pulse_ref=robot_t->motor1->speed_r_ref  * 390;  //将v（m/s）转化为相关的脉冲数
  robot_t->motor2->pid_t->pulse_ref=robot_t->motor2->speed_r_ref  * 390;
  robot_t->motor3->pid_t->pulse_ref=robot_t->motor3->speed_r_ref  * 390;
  robot_t->motor4->pid_t->pulse_ref=robot_t->motor4->speed_r_ref  * 390;
}

//根据计算的速度正负，为电机direction赋值
void DirectionGet(ROBOT_T*robot_t)
{
  if(robot_t->motor1->vw_ref>0)   robot_t->motor1->direction=0;
  else if(robot_t->motor1->vw_ref<0)  robot_t->motor1->direction=1;
  else robot_t->motor1->direction=2;

  if(robot_t->motor2->vw_ref>0)   robot_t->motor2->direction=0;
  else if(robot_t->motor2->vw_ref<0)  robot_t->motor2->direction=1;
  else robot_t->motor2->direction=2;

  if(robot_t->motor3->vw_ref>0)   robot_t->motor3->direction=0;
  else if(robot_t->motor3->vw_ref<0)  robot_t->motor3->direction=1;
  else robot_t->motor3->direction=2;

  if(robot_t->motor4->vw_ref>0)   robot_t->motor4->direction=0;
  else if(robot_t->motor4->vw_ref<0)  robot_t->motor4->direction=1;
  else robot_t->motor4->direction=2;
}

////编码器采集脉冲与圈速转化  引脚PD12,PD13
//int pulse1,pulse2,pulse3,pulse4;
//void SpeedCalculate(ROBOT_T*robot_t)
//{
//	//4倍频检测脉冲数(转化为每s，和pulse_ref对应)
////	int pulse1,pulse2,pulse3,pulse4;
//  //获取脉冲数
//  pulse1 = __HAL_TIM_GET_COUNTER(Encoder_Port_1);
//  if(pulse1 > 30000) pulse1 = 65536 - pulse1;
//  pulse2 = __HAL_TIM_GET_COUNTER(Encoder_Port_2);
//  if(pulse2 > 30000) pulse2 = 65536 - pulse2;
//  pulse3 = __HAL_TIM_GET_COUNTER(Encoder_Port_3);
//  if(pulse3 > 30000) pulse3 = 65536 - pulse3;
//  pulse4 = __HAL_TIM_GET_COUNTER(Encoder_Port_4);
//  if(pulse4 > 30000) pulse4 = 65536 - pulse4;
//	
//  //每秒等效脉冲数
//	robot_t->motor1->pid_t->pulse_fdb=pulse1*1000;
//	robot_t->motor2->pid_t->pulse_fdb=pulse2*1000;
//	robot_t->motor3->pid_t->pulse_fdb=pulse3*1000;
//	robot_t->motor4->pid_t->pulse_fdb=pulse4*1000;

//	//利用脉冲计算电机速度的反馈值
//	robot_t->motor1->speed_r_fdb=robot_t->motor1->pid_t->pulse_fdb/(390.0000*4);	//转速=脉冲/（每圈发出脉冲*4倍频）
//	robot_t->motor1->vw_fdb=0.12*2*3.14159*robot_t->motor1->speed_r_fdb;																		//线速=轮子半径*转速
//	robot_t->motor2->speed_r_fdb=robot_t->motor2->pid_t->pulse_fdb/(390.0000*4);
//	robot_t->motor2->vw_fdb=0.12*2*3.14159*robot_t->motor2->speed_r_fdb;
//	robot_t->motor3->speed_r_fdb=robot_t->motor3->pid_t->pulse_fdb/(390.0000*4);
//	robot_t->motor3->vw_fdb=0.12*2*3.14159*robot_t->motor3->speed_r_fdb;
//	robot_t->motor4->speed_r_fdb=robot_t->motor4->pid_t->pulse_fdb/(390.0000*4);
//	robot_t->motor4->vw_fdb=0.12*2*3.14159*robot_t->motor4->speed_r_fdb;

//  //通过计算得出反馈的vy，vx与v_fdb
//	robot_t->robot_config->vy_fdb=(robot_t->motor1->vw_fdb+robot_t->motor2->vw_fdb)/2.0f;
//	robot_t->robot_config->vx_fdb=(robot_t->motor1->vw_fdb-robot_t->motor3->vw_fdb)/2.0f;
//	robot_t->robot_config->v_fdb=(float)sqrt(robot_t->robot_config->vy_fdb*robot_t->robot_config->vy_fdb+robot_t->robot_config->vx_fdb*robot_t->robot_config->vx_fdb);

//	//脉冲归零
//	__HAL_TIM_SetCounter(Encoder_Port_1,0);
//  __HAL_TIM_SetCounter(Encoder_Port_2,0);
//  __HAL_TIM_SetCounter(Encoder_Port_3,0);
//  __HAL_TIM_SetCounter(Encoder_Port_4,0);

//}

////pid计算
//void MotorCal(ROBOT_T*robot_t)
//{
//	PID_Calculate(robot_t->motor1->pid_t);
//	PID_Calculate(robot_t->motor2->pid_t);
//	PID_Calculate(robot_t->motor3->pid_t);
//	PID_Calculate(robot_t->motor4->pid_t);
//}

//驱动各个电机
void MotorRun(ROBOT_T*robot_t)
{
  //电机1驱动rf
  if(robot_t->motor1->direction==0)//正转
  {
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_11,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
  }
  else if(robot_t->motor1->direction==1){   //反转
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_11,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
  }
	else{     //停止
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_11,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
	}
  TIM10->ARR=3000;
  TIM10->CCR1=2*abs(robot_t->motor1->pid_t->pulse_ref);     //电机输出
  HAL_TIM_PWM_Start(Motor_Port_1,Motor_CHANNEL_1);

  //电机2驱动lf
  if(robot_t->motor2->direction==0)
  {
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_15,GPIO_PIN_RESET);
  }
  else if(robot_t->motor2->direction==1){
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_15,GPIO_PIN_SET);
  }
	else{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_15,GPIO_PIN_RESET);
	}
  TIM11->ARR=3000;
  TIM11->CCR1=2*abs(robot_t->motor2->pid_t->pulse_ref);
  HAL_TIM_PWM_Start(Motor_Port_2,Motor_CHANNEL_2);

  //电机3驱动rb
  if(robot_t->motor3->direction==0)
  {
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);
  }
  else if(robot_t->motor3->direction==1){
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);
  }
	else{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);
	}
  TIM13->ARR=3000;
  TIM13->CCR1=2*abs(robot_t->motor2->pid_t->pulse_ref);
  HAL_TIM_PWM_Start(Motor_Port_3,Motor_CHANNEL_3);

  //电机4驱动lb
  if(robot_t->motor1->direction==0)
  {
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_RESET);
  }
  else if(robot_t->motor1->direction==1){
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET);
  }
	else{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_RESET);
	}
  TIM2->ARR=3000;
  TIM2->CCR1=2*abs(robot_t->motor1->pid_t->pulse_ref);
  HAL_TIM_PWM_Start(Motor_Port_4,Motor_CHANNEL_4);
}

void RobotStop(void)
{
  HAL_TIM_PWM_Stop(Motor_Port_1,Motor_CHANNEL_1);
	HAL_TIM_PWM_Stop(Motor_Port_2,Motor_CHANNEL_2);
  HAL_TIM_PWM_Stop(Motor_Port_3,Motor_CHANNEL_3);
  HAL_TIM_PWM_Stop(Motor_Port_4,Motor_CHANNEL_4);
}

