#include "MeasuresIMU.h"

MeasuresIMU::MeasuresIMU(TwoWire& w) : MPU6050(w) {}

byte MeasuresIMU::beginWithLogging(int gyro_config_num, int acc_config_num, bool doOffsetCalibration) {
    byte status = MPU6050::begin(gyro_config_num, acc_config_num);

    if (status == 0) {
        Serial.println("MPU6050 initialized!");
        if (doOffsetCalibration) {
            Serial.println("Calibrating offsets...");
            this->calcOffsets(true, true);
            Serial.print("GyroY Offset: "); Serial.println(this->getGyroY(), 5);
        }
        this->update();
        prevFilteredAngle = this->getAngleY();
        prevTime = millis();
    } else {
        Serial.println("MPU6050 initialization failed!");
    }

    return status;
}

void MeasuresIMU::updateFilter(){
    this->update();
}

float MeasuresIMU::getIMUAngleY(){
    return this->getAngleY() * DEG_TO_RAD; 
}

float MeasuresIMU::getIMUGyroY(){
    return this->getGyroY() * DEG_TO_RAD;
}

float MeasuresIMU::getFusedRadSpeed()
{   
    const float EPSILON = 1e-5; 
    unsigned long actualTime = micros();
    float dt = (actualTime - prevTime) / 1e-6;
    if (dt < EPSILON) dt = EPSILON; 


    float currentAngle = this->getAccAngleY();
    float gyroYraw = this->getGyroY();

    float radSpeedEstimate = (currentAngle - prevFilteredAngle)/dt;

    const float alpha = 0.98;
    float fusedRate = alpha * gyroYraw + (1 - alpha) * radSpeedEstimate;

    prevFilteredAngle = currentAngle;
    prevTime = actualTime;
    prevFusedGyroY = fusedRate;

    return fusedRate * DEG_TO_RAD;
}

