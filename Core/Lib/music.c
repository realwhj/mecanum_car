#include "music.h"
#include "robot_def.h"
	
	
	
int music_hgd[14]={M_Mi,M_La,M_Mi,M_Do,L_La,0,L_La,M_Mi,L_La,M_Mi,M_So,M_La,M_Xi,M_So};

int music_twotiger[63]={M_Do, 0,    M_Do, M_Mi, M_Mi, M_Mi, M_Mi, 0,    M_Do,
                       0,    M_Do, M_Mi, M_Mi, M_Mi, M_Mi, 0,    M_La, 0,
                       M_La, 0,    M_So, 0,    M_La, 0,    M_So, M_Do, 0,
                       M_Mi, M_Mi, 0,    0,    H_Do, 0,    M_La, M_La, M_So,
                       0,    M_La, 0,    0,    M_So, 0,    M_Do, M_Re, M_Re,
                       0,    0,    M_Xi, M_Xi, M_Xi, M_Xi, M_Xi, 0,    M_Xi,
                       0,    M_So, M_Mi, 0,    M_So, M_So, M_So};
	
int music1[62] = {H_Do, 0,    M_Xi, 0,    M_So, 0,    M_So, 0,    M_La,
                       1145, M_La, 0,    M_So, 0,    M_So, 0,    M_La, 1145,
                       M_La, 0,    M_So, 0,    M_So, 0,    M_La, 1145, M_La,
                       0,    M_La, 0,    M_La, M_La, M_Xi, M_Xi, H_Do, H_Do,
                       H_Re, H_Re, M_Xi, M_Xi, M_Xi, M_Xi, M_So, M_So, M_So,
                       M_So, H_Fa, H_Fa, H_Fa, H_Fa, H_Re, H_Re, H_Re, 0,
                       H_Re, H_Re, 0,    H_Re, H_Re, H_Re, H_Mi, H_Mi};
int  music6[56] = {
    H_Do, H_Do, H_Do, 0,    H_Do, 0,    M_Xi, H_Do, H_Do, 0,    H_Re, 0,
    H_Mi, 0,    H_Fa, 0,    H_Mi, H_Mi, 0,    H_Mi, H_Mi, 0,    M_Xi, 0,
    M_Xi, M_Xi, M_Xi, 0,    0,    M_La, M_La, M_La, 0,    M_La, 0,    M_So,
    M_La, M_La, 0,    H_Fa, 0,    H_Mi, 0,    H_Re, 0,    H_Re, H_Re, 0,
    H_Do, H_Do, 0,    H_Re, 0,    H_Mi, H_Mi, H_Mi};
	
int music_kaqiusha[] =
{
	L_La,0,L_Xi,0,M_Do,0,L_La,M_Do,0,L_Xi,L_La,0,L_Xi,L_Mi,0,L_Xi,0,M_Do,M_Re,0,L_Xi,
	M_Re,0,M_Re,0,M_Do,0,L_Xi,L_La,0,L_La,M_Mi,0,M_La,0,M_So,0,M_La,M_So,0,M_Fa,0,M_Mi,M_Re,0,
	M_Mi,0,L_La,0,0,M_Fa,0,M_Re,0,M_Mi,0,M_Do,0,L_Xi,L_Mi,0,M_Do,L_Xi,0,L_La,L_La
};
		
		
  void Music(void)
{
	static int cnt=0;
	while(1)
	{
		if(cnt<sizeof(MUSIC)/sizeof(int))
		{
			if(MUSIC[cnt]!=0)
			{
				__HAL_TIM_SetCompare(MUSIC_PORT,MUSIC_CHANNEL,MUSIC[cnt]>>1);
				__HAL_TIM_SetAutoreload(MUSIC_PORT,MUSIC[cnt]);
			}
			else
			{
				__HAL_TIM_SetCompare(MUSIC_PORT,MUSIC_CHANNEL,3000);
				__HAL_TIM_SetAutoreload(MUSIC_PORT,3000);	//低电平触发，全输出高电平
			}
			cnt++;
		}
		else
		{
			HAL_TIM_PWM_Stop(MUSIC_PORT,MUSIC_CHANNEL);	
			break;
		}
		HAL_Delay(175);	//延时100ms，使音乐播放连续
	}
}
