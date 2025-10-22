#include "main.h"
#include "MyI2C.h"
#include "MPU6050_Reg.h"

int16_t data_bias[6] = {0XFCCA, 0X00A1, 0X009E, 0X0069, 0XFF70, 0X000A};
    
void MPU6050_Init(void)
{
    //复位
    I2C_WriteReg(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x80);
    HAL_Delay(100);
    //唤醒,选择时钟源为x轴陀螺仪
    I2C_WriteReg(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x01);
    //所有轴均不待机
    I2C_WriteReg(MPU6050_ADDRESS, MPU6050_PWR_MGMT_2, 0x00);
    //采样率
    I2C_WriteReg(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 0x04);
    //配置寄存器，配置低通滤波，满足采样率>=2*带宽
    I2C_WriteReg(MPU6050_ADDRESS, MPU6050_CONFIG, 0x04);
    //acc量程
    I2C_WriteReg(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0x00);
    //gyro量程
    I2C_WriteReg(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0x18);

}

void MPU6050_GetData(int16_t* accx, int16_t* accy, int16_t* accz,
                     int16_t* gyx, int16_t* gyy, int16_t* gyz)
{
    uint8_t DataH, DataL;							           
  
    DataH = I2C_ReadReg(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H);
    DataL = I2C_ReadReg(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_L);
    *accx = (DataH << 8) | DataL;						       

    DataH = I2C_ReadReg(MPU6050_ADDRESS, MPU6050_ACCEL_YOUT_H);
    DataL = I2C_ReadReg(MPU6050_ADDRESS, MPU6050_ACCEL_YOUT_L);
    *accy = (DataH << 8) | DataL;						       

    DataH = I2C_ReadReg(MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT_H);
    DataL = I2C_ReadReg(MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT_L);
    *accz = (DataH << 8) | DataL;						       

    DataH = I2C_ReadReg(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H);
    DataL = I2C_ReadReg(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_L);
    *gyx = (DataH << 8) | DataL;						       

    DataH = I2C_ReadReg(MPU6050_ADDRESS, MPU6050_GYRO_YOUT_H);
    DataL = I2C_ReadReg(MPU6050_ADDRESS, MPU6050_GYRO_YOUT_L);
    *gyy = (DataH << 8) | DataL;						       

    DataH = I2C_ReadReg(MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_H);
    DataL = I2C_ReadReg(MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_L);
    *gyz = (DataH << 8) | DataL;						       
}

void MPU6050_GetCaliberateData(int16_t* accx, int16_t* accy, int16_t* accz,
                     int16_t* gyx, int16_t* gyy, int16_t* gyz)
{
    int16_t data[6] = {0};
    MPU6050_GetData(&data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);
    *accx = data[0] + data_bias[0];
    *accy = data[1] + data_bias[1];
    *accz = data[2] + data_bias[2];
    *gyx = data[3] + data_bias[3];
    *gyy = data[4] + data_bias[4];
    *gyz = data[5] + data_bias[5];
}
