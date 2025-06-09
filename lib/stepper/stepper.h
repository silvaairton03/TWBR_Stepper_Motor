#ifndef STEPPER_H
#define STEPPER_H

#include<Arduino.h>
#include<AccelStepper.h>
#include<MPU6050_light.h>

class stepper {
public:
    stepper(uint8_t stepPinLeft, uint8_t dirPinLeft,
        uint8_t stepPinRight, uint8_t dirPinRight,
        float wheelRadius, int stepsPerRevolution);
    void enableMotors(uint8_t enPin);
    void attachMPU(MPU6050 &mpuRef);
    void update();
    void initMotors(float maxSpeed, float acceleration);
    void initTimers(void (*isrLeft)(), void (*isrRight)());
    void setMotorSpeed(float leftSpeed, float rightSpeed);
    void resetSteps();
    void runLeftMotor();
    void runRightMotor();
    float getRobotPosition() const;
    float getRobotVelocity() const;


private:
    AccelStepper* stepperLeft;
    AccelStepper* stepperRight;
    MPU6050 *mpu;
    float wheelRadius;
    float wheelBase;
    int stepsPerRevolution;
    float pendulumPosition;
    float pendulumVelocity;
    const float POSITION_DEADBAND = 1e-4;
    const float VELOCITY_DEADBAND = 1e-4;   

    uint8_t stepPinLeft, dirPinLeft;
    uint8_t stepPinRight, dirPinRight;
    hw_timer_t *timerLeft;
    hw_timer_t *timerRight;
};

#endif