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
    void runLeftMotor();
    void runRightMotor();
    float getRobotPosition() const;
    float getRobotVelocity() const;
    float getYawAngle() const;
    float getYawRate() const;


private:
    AccelStepper* stepperLeft;
    AccelStepper* stepperRight;
    MPU6050 *mpu;
    float wheelRadius;
    float wheelBase;
    int stepsPerRevolution;
    float pendulumPosition;
    float pendulumVelocity;
    float yawAngle;
    float yawRate;
    float lastYawOdom;
    const float VELOCITY_DEADBAND = 0.001;
    const float POSITION_DEADBAND = 0.005;

    uint8_t stepPinLeft, dirPinLeft;
    uint8_t stepPinRight, dirPinRight;
    hw_timer_t *timerLeft;
    hw_timer_t *timerRight;
};

#endif