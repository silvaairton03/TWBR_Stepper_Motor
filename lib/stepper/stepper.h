#ifndef STEPPER_H
#define STEPPER_H

#include<Arduino.h>
#include<AccelStepper.h>
#include<MPU6050_light.h>

class stepper {
public:
    stepper(AccelStepper &leftStepper, AccelStepper &rightStepper, float wheelRadius, int stepsPerRevolution);
    void attachMPU(MPU6050 &mpuRef);
    void update();
    float getRobotPosition() const;
    float getRobotVelocity() const;
    float getYawAngle() const;
    float getYawRate() const;

private:
    AccelStepper &stepperLeft;
    AccelStepper &stepperRight;
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
};

#endif