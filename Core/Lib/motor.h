#ifndef MOTOR_H
#define MOTOR_H

#include "robot.h"

void RobotRun(ROBOT_T*robot_t);
void RobotStop(void);

void MecanumCal(ROBOT_T*robot_t);
void DirectionGet(ROBOT_T*robot_t);
void MotorRun(ROBOT_T*robot_t);
void MotorCal(ROBOT_T*robot_t);
void SpeedCalculate(ROBOT_T*robot_t);

void RunMode(ROBOT_T*robot_t);
#endif
