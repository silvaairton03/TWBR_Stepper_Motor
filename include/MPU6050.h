#ifndef MPU6050_H
#define MPU6050_H
#include<Wire.h>

extern float gyroX, gyroY, gyroZ;
extern float gyroXOffset, gyroYOffset, gyroZOffset;
extern float GyroPitch, FilteredPitchAngle;
extern float y_hp[], x_hp[], y_lp[], x_lp[];
extern float const_a;
extern int calibrationNumber;
extern float AccX, AccY, AccZ;
extern float AngleRoll, AnglePitch, AngleYaw;
extern float YawAngle;
extern unsigned long lastTime;
extern bool calibrated;

void initMPU6050();
void setupMPU6050();
void calibrationGyro();
//void medidasAcelerometro();

#endif