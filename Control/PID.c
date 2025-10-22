#include "PID.h"


#define pwm_max  5000
#define pwm_min  -5000


int32_t balance_out;
float mechanical_value = 0;
float kp_balance = -750*0.6; //-280//-250
float kd_balance = -1.5*0.6;//-1.4

float velocity_out;
float  kp_velocity = 1.6;//1.6
float  ki_velocity = 0.008;//0.008

int32_t pwm_turn;
float kp_turn;
float kd_turn;

int32_t  pwml;
int32_t  pwmr;



void Blance(float target, float roll, int16_t gx)
{
    float bias;
    bias = roll - target - mechanical_value;
    balance_out = bias*kp_balance + gx*kd_balance;
}


void Velocity(short encoderL, short encoderR, float roll)
{
    static int16_t bias_last = 0;
    int16_t bias = 0;
    static int16_t bias_integral = 0;
    
    bias = encoderL + encoderR - 0;
    bias = bias * 0.2 + bias_last*0.8;
    bias_last = bias;
    bias_integral += bias;

    if(bias_integral > 10000){bias_integral = 10000;}
    else if(bias_integral < -10000){bias_integral = -10000;}

    if(roll >40 | roll <-40){bias_integral = 0;}

    velocity_out = bias*kp_velocity + bias_integral*ki_velocity;
}


void Turn(float yaw, int16_t gz)
{
	int16_t bias;
	
	bias = yaw - 0;
	pwm_turn = bias*kp_turn + gz*kp_turn;
}


void PWM_Get(void)
{
	pwml = balance_out + pwm_turn ;
	pwmr = balance_out - pwm_turn ;
	if(pwml>pwm_max){pwml = pwm_max;}
	else if(pwml<pwm_min){pwml = pwm_min;}
	if(pwmr>pwm_max){pwmr = pwm_max;}
	else if(pwmr<pwm_min){pwmr = pwm_min;}
    
    Motor_SetSpeed(pwml, pwmr);
}
