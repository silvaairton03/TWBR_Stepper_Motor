#ifndef STEPPER_H
#define STEPPER_H

#include<Arduino.h>
#include<AccelStepper.h>

class stepper {
public:
    stepper(AccelStepper &leftStepper, AccelStepper &rightStepper, float wheelRadius, int stepsPerRevolution);
    void update();
    float getRobotPosition() const;
    float getRobotVelocity() const;

private:
    AccelStepper &stepperLeft;
    AccelStepper &stepperRight;
    float wheelRadius;
    int stepsPerRevolution;
    float pendulumPosition;
    float pendulumVelocity;
    const float VELOCITY_DEADBAND = 0.001;
    const float POSITION_DEADBAND = 0.005;
};

#endif