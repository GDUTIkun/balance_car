#ifndef __PID_H
#define __PID_H

#include "main.h"
#include "Motor.h"

extern int32_t pwm_balance;
extern float kp_balance;
extern float ki_balance;
extern int32_t balance_out;

extern int32_t pwm_velocity;
extern float kp_velocity;
extern float ki_velocity;
extern float velocity_out;

extern int32_t pwm_turn;
extern float kp_turn;
extern float kd_turn;

extern int32_t  pwml;
extern int32_t  pwmr;

void Blance(float target, float roll, int16_t gx);
void Velocity(short encoderL, short encoderR, float pitch);
void Turn(float yaw, int16_t gz);
void PWM_Get(void);

#endif
