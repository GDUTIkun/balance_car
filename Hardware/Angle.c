#include "Angle.h"

//互补滤波解欧拉角
#define fliterKp           2.0f
#define fliterKi           0.00002f
#define sampletime         0.005f
#define halfsampletime     0.0025f

int16_t ax, ay, az, gx, gy, gz;

imudata imu;
float normalax, normalay, normalaz;
float q[4] = {1.0, 0.0, 0.0, 0.0};
float vx, vy, vz;
float biasx, biasy, biasz;
float biasx_intigral, biasy_intigral, biasz_intigral;
float roll, pitch, yaw;

//取平方根倒数函数
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f375a86 - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//单位转化
void imu_GetData(void)
{
    MPU6050_GetCaliberateData(&ax, &ay, &az, &gx, &gy, &gz);
    imu.ax = (float)ax / 16384;
    imu.ay = (float)ay / 16384;
    imu.az = (float)az / 16384;
    imu.gx = gx * 0.001065;
    imu.gy = gy * 0.001065;
    imu.gz = gz * 0.001065;
}


void Data_Correction(void)
{
    //刚体加速度归一化（只保留角度信息）
    float norm;
    float tq[4];

    imu_GetData();
    norm  = invSqrt(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az); 
    normalax = imu.ax * norm;
    normalay = imu.ay * norm;
    normalaz = imu.az * norm;
    
    //把地理系归一化的加速度转化到刚体系（用角速度表示）
    vx = 2*( q[1]*q[3] - q[0]*q[2] );
    vy = 2*( q[2]*q[3] + q[0]*q[1] );
    vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    
    //叉积，获取欧拉角误差
    biasx = normalay*vz - normalaz*vy;
    biasy = normalaz*vx - normalax*vz;
    biasz = normalax*vy - normalay*vx;
    
    //二阶滤波器互补滤波
    biasx_intigral += fliterKi*biasx;
    biasy_intigral += fliterKi*biasy;
    biasz_intigral += fliterKi*biasz;
    imu.gx += fliterKp*biasx + biasx_intigral;
    imu.gy += fliterKp*biasy + biasy_intigral;
    imu.gz += fliterKp*biasz + biasz_intigral;
    
    //四元数更新
    tq[0] = q[0] + (-imu.gx*q[1] - imu.gy*q[2] - imu.gz*q[3])*halfsampletime;
    tq[1] = q[1] + (imu.gx*q[0] - imu.gy*q[3] + imu.gz*q[2])*halfsampletime;
    tq[2] = q[2] + (imu.gx*q[3] + imu.gy*q[0] - imu.gz*q[1])*halfsampletime;
    tq[3] = q[3] + (-imu.gx*q[2] + imu.gy*q[1] + imu.gz*q[0])*halfsampletime;
    
    //四元数重新归一化
    norm = invSqrt(tq[0]*tq[0] + tq[1]*tq[1] + tq[2]*tq[2] + tq[3]*tq[3]);
    q[0] = tq[0]*norm;
    q[1] = tq[1]*norm;
    q[2] = tq[2]*norm;
    q[3] = tq[3]*norm;
}

void Get_Angle(void)
{
    imu_GetData();
    Data_Correction();
    roll = atan2(2*(q[2]*q[3] + q[0]*q[1]), q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])* 57.296;
    pitch = -asin(2*(q[1]*q[3] - q[0]*q[2])) * 57.296;
    yaw += imu.gz * sampletime * 57.296;
}


imudata KF_imu;
float g_roll, g_pitch;
float k_roll = 0, k_pitch = 0;
float gx_bias, gy_bias;
float z_roll, z_pitch;
float roll_P[2][2], pitch_P[2][2] = {1,0,1,0};
float Q_angle = 0.0025, Q_bias = 0.0025;
float R = 0.3;
float roll_K[2], pitch_K[2];

void FK_GetAngle(void)
{
    //转角度
    int16_t ax, ay, az, gx, gy, gz;
    MPU6050_GetCaliberateData(&ax, &ay, &az, &gx, &gy, &gz);
    KF_imu.ax = (float)ax / 16384;
    KF_imu.ay = (float)ay / 16384;
    KF_imu.az = (float)az / 16384;
    KF_imu.gx = gx / 16.384;
    KF_imu.gy = gy / 16.384;
    KF_imu.gz = gz / 16.384;
    
    //耦合角速度求角度增量
    g_roll = (KF_imu.gx + (sin(k_pitch)*sin(k_roll)/cos(k_pitch))* KF_imu.gy 
            + (cos(k_roll)*sin(k_pitch)/cos(k_pitch))*KF_imu.gz)*sampletime;
    g_pitch = (cos(k_roll)*KF_imu.gy - sin(k_roll)*KF_imu.gz)*sampletime;
    
    //状态外推方程
    k_roll += g_roll - gx_bias*sampletime;
    k_pitch += g_pitch - gy_bias*sampletime;
    
    //观测方程
    z_roll = atan2(KF_imu.ay, KF_imu.az)* 57.296;
    z_pitch = atan2(KF_imu.ax, sqrt(KF_imu.ay*KF_imu.ay + KF_imu.az*KF_imu.az))* 57.296;
    
    //协方差外推方程
    roll_P[0][0] += Q_angle + (roll_P[1][1]*sampletime - roll_P[1][0] - roll_P[0][1])*sampletime;
    roll_P[0][1] -= roll_P[1][1]*sampletime;
    roll_P[1][0] -= roll_P[1][1]*sampletime;
    roll_P[1][1] += Q_bias;
    pitch_P[0][0] += Q_angle + (pitch_P[1][1]*sampletime - pitch_P[1][0] - pitch_P[0][1])*sampletime;
    pitch_P[0][1] -= pitch_P[1][1]*sampletime;
    pitch_P[1][0] -= pitch_P[1][1]*sampletime;
    pitch_P[1][1] += Q_bias;
    
    //卡尔曼增益
    roll_K[0] = roll_P[0][0]/(roll_P[0][0] + R);
    roll_K[1] = roll_P[1][0]/(roll_P[0][0] + R);
    pitch_K[0] = pitch_P[0][0]/(pitch_P[0][0] + R);
    pitch_K[1] = pitch_P[1][0]/(pitch_P[0][0] + R);
    
    //状态更新方程
    k_roll += roll_K[0]*(z_roll - k_roll);
    gx_bias += roll_K[1]*(z_roll - k_roll);
    k_pitch += pitch_K[0]*(z_pitch - k_pitch);
    gy_bias += pitch_K[1]*(z_pitch - k_pitch);
    
    //协方差更新方程
    roll_P[0][0] -= roll_K[0]*roll_P[0][0];
    roll_P[0][1] -= roll_K[0]*roll_P[0][1]; 
    roll_P[1][0] -= roll_K[1]*roll_P[0][0];
    roll_P[1][1] -= roll_K[1]*roll_P[0][1];
    pitch_P[0][0] -= pitch_K[0]*pitch_P[0][0];
    pitch_P[0][1] -= pitch_K[0]*pitch_P[0][1];
    pitch_P[1][0] -= pitch_K[1]*pitch_P[0][0];
    pitch_P[1][1] -= pitch_K[1]*pitch_P[0][1];
}




