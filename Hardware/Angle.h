#ifndef __ANGLE_H
#define __ANGLE_H

#include "main.h"
#include "MPU6050.h"
#include <math.h>

typedef struct imudata
{
    float ax, ay, az, gx, gy, gz;
}imudata;

extern int16_t ax, ay, az, gx, gy, gz;
extern imudata imu;
extern float roll, pitch, yaw;;
extern float k_roll, k_pitch;
extern float z_roll, z_pitch;


void Get_Angle(void);
void FK_GetAngle(void);

#endif
