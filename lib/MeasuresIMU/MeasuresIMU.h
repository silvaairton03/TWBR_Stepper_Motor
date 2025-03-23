#ifndef MEASURES_IMU_H
#define MEASURES_IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>

class MeasuresIMU : public MPU6050 {
public:
    // Construtor padrão com referência ao barramento I2C
    MeasuresIMU(TwoWire& w);

    // Método de inicialização com possibilidade de calibrar offsets e fazer log
    byte beginWithLogging(int gyro_config_num = 0, int acc_config_num = 0, bool doOffsetCalibration = true);

    void updateFilter();

    float getIMUAngleY();
    float getIMUGyroY();
    float getFusedRadSpeed();

private:
    float prevFilteredAngle = 0.0;
    unsigned long prevTime = 0.0;
    float prevFusedGyroY = 0.0;
};  

#endif
