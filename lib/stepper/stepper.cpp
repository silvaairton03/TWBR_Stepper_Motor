#include "stepper.h"

stepper::stepper(AccelStepper &leftStepper, AccelStepper &rightStepper, float wheelRadius, int stepsPerRevolution)
    : stepperLeft(leftStepper), stepperRight(rightStepper), wheelRadius(wheelRadius), stepsPerRevolution(stepsPerRevolution), 
    pendulumPosition(0.0), pendulumVelocity(0.0) {}

void stepper::update() {
    float stepperMotorSpeedLeft = stepperLeft.speed();
    float stepperMotorSpeedRight = stepperRight.speed();

    float stepperMotorRadLeft = stepperMotorSpeedLeft * 2 * PI / stepsPerRevolution;
    float stepperMotorRadRight = -stepperMotorSpeedRight * 2 * PI / stepsPerRevolution;

    float stepperPositionLeft = stepperLeft.currentPosition() * 2 * PI / stepsPerRevolution;
    float stepperPositionRight = -stepperRight.currentPosition() * 2 * PI / stepsPerRevolution;

    float rawPendulumVelocity = ((stepperMotorRadLeft + stepperMotorRadRight) * wheelRadius) / 2;
    float rawPendulumPosition = ((stepperPositionLeft + stepperPositionRight) * wheelRadius) / 2;

    // pendulumVelocity = (fabs(rawPendulumVelocity) < VELOCITY_DEADBAND) ? 0 : rawPendulumVelocity;
    // pendulumPosition = (fabs(rawPendulumPosition) < POSITION_DEADBAND) ? 0 : rawPendulumPosition;
    pendulumPosition = rawPendulumPosition;
    pendulumVelocity = rawPendulumVelocity;
}

float stepper::getRobotPosition() const {
    return pendulumPosition;
}

float stepper::getRobotVelocity() const {
    return pendulumVelocity;
}