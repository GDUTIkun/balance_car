#include "Encoder.h"
#include "tim.h"

short EncoderL;
short EncoderR;

void Encoder_SpeedGet (void)
{
	EncoderL = -__HAL_TIM_GetCounter(&htim2);
	__HAL_TIM_SetCounter(&htim2, 0);
	EncoderR = __HAL_TIM_GetCounter(&htim4);//自己测一下极性和对应轮子
	__HAL_TIM_SetCounter(&htim4, 0);
}
