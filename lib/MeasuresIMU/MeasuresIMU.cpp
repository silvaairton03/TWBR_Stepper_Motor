#include "MeasuresIMU.h"

MeasuresIMU::MeasuresIMU(TwoWire& w) : MPU6050(w) {}

byte MeasuresIMU::beginWithLogging(int gyro_config_num, int acc_config_num, bool doOffsetCalibration) {
    byte status = MPU6050::begin(gyro_config_num, acc_config_num);  // Call base begin()

    if (status != 0) {
        Serial.println(F("MPU6050 initialization failed!"));
        return status;
    }

    Serial.println(F("MPU6050 initialized successfully."));

    if (doOffsetCalibration) {
        Serial.println(F("Calculating offsets..."));
        this->calcOffsets(true, true);  // Uses the base class method
    }

    // Display the offsets using base class's getters
    Serial.print(F("Gyro Y Offsets : "));
    // Serial.print(this->getGyroXOffset()); Serial.print(", ");
    Serial.println(this->getGyroYOffset());
    // Serial.print(", ");
    // Serial.println(this->getGyroZOffset());

    Serial.print(F("Accel Offsets (X,Y,Z): "));
    Serial.print(this->getAccXOffset()); Serial.print(", ");
    Serial.print(this->getAccYOffset()); Serial.print(", ");
    Serial.println(this->getAccZOffset());

    return status;
}

float MeasuresIMU::getIMUAngleY(){
    return this->getAngleY() * DEG_TO_RAD; 
}

float MeasuresIMU::getIMUGyroY(){
    return this->getGyroY() * DEG_TO_RAD;
}
