#ifndef LINE_PATROL_H
#define  LINE_PATROL_H


//机器人运动方式枚举类型
//直行，左右移，左右转直角，左右转弯道，偏离左右修正
typedef enum run_state
{
	stop=0,
	straghit,
	left,
	right,
	turn_strleft,
	turn_strright,
	turn_benleft,
	turn_benright,
}RUN_STATE_E;



void LinePatrolGet(void);
void RunStateGet(void);




#endif
