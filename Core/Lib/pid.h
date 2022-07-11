#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct config_t
{
    float kp;
    float ki;
    float kd;
    int16_t bias[3];
    int16_t bias_sum;
    int16_t bias_max;
    float output;
    float output_max;
}CONFIG_T;

typedef struct pid
{
    CONFIG_T*config;
    int pulse_ref;
    int pulse_fdb;
}PID_T;

PID_T*PidCreat(void);
CONFIG_T*ConfigCreat(void);


void PID_Init(PID_T*pid,float kp,float ki,float kd,float bias_max,int16_t output_max);
void PID_Calculate(PID_T*pid_t);

#endif
