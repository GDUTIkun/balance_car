#ifndef __MPU6050_H
#define __MPU6050_H

void MPU6050_Init(void);
void MPU6050_GetData(int16_t* accx, int16_t* accy, int16_t* accz,
                     int16_t* gyx, int16_t* gyy, int16_t* gyz);
void MPU6050_Caliberation(void);
void MPU6050_GetCaliberateData(int16_t* accx, int16_t* accy, int16_t* accz,
                     int16_t* gyx, int16_t* gyy, int16_t* gyz);

#endif
