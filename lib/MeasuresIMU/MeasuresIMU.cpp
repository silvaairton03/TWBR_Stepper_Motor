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

// float MeasuresIMU::getFusedRadSpeed()
// {   
//     const float EPSILON = 1e-5; 
//     unsigned long actualTime = micros();
//     float dt = (actualTime - prevTime) / 1e-6;
//     if (dt < EPSILON) dt = EPSILON; 


//     float currentAngle = this->getAccAngleY();
//     float gyroYraw = this->getGyroY();

//     float radSpeedEstimate = (currentAngle - prevFilteredAngle)/dt;

//     const float alpha = 0.98;
//     float fusedRate = alpha * gyroYraw + (1 - alpha) * radSpeedEstimate;

//     prevFilteredAngle = currentAngle;
//     prevTime = actualTime;
//     prevFusedGyroY = fusedRate;

//     return fusedRate * DEG_TO_RAD;
// }
float MeasuresIMU::getFusedRadSpeed()
{
    static float x_lp[2] = {0.0f, 0.0f}; // Acc-derived rate
    static float y_lp[2] = {0.0f, 0.0f};

    static float x_hp[2] = {0.0f, 0.0f}; // Gyro rate
    static float y_hp[2] = {0.0f, 0.0f};

    const float alpha = 0.95f;  // filtro LP para acelerômetro
    const float beta  = 0.95f;  // filtro HP para giroscópio

    unsigned long actualTime = micros();
    float dt = (actualTime - prevTime) / 1e6f;
    if (dt < 1e-5f) dt = 1e-5f;
    prevTime = actualTime;

    float currentAccAngle = this->getAccAngleY();
    float gyroYraw = this->getGyroY();

    // === Derivada da medida do acelerômetro ===
    float accRate = (currentAccAngle - prevFilteredAngle) / dt;
    prevFilteredAngle = currentAccAngle;

    // === Filtro Passa-Baixa para derivada do acelerômetro ===
    x_lp[0] = accRate;
    y_lp[0] = (1 - alpha) * x_lp[1] + alpha * y_lp[1];
    x_lp[1] = x_lp[0];
    y_lp[1] = y_lp[0];

    // === Filtro Passa-Alta para giroscópio ===
    x_hp[0] = gyroYraw;
    y_hp[0] = beta * (y_hp[1] + x_hp[0] - x_hp[1]);
    x_hp[1] = x_hp[0];
    y_hp[1] = y_hp[0];

    // === Combinação dos dois ===
    float fused = y_lp[0] + y_hp[0];

    return fused * DEG_TO_RAD;
}

