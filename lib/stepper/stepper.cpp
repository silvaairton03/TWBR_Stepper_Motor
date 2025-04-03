#include "stepper.h"

stepper::stepper(AccelStepper &leftStepper, AccelStepper &rightStepper, float wheelRadius, int stepsPerRevolution)
    : stepperLeft(leftStepper), stepperRight(rightStepper), wheelRadius(wheelRadius), stepsPerRevolution(stepsPerRevolution), 
    pendulumPosition(0.0), pendulumVelocity(0.0),
    yawAngle(0.0), yawRate(0.0), lastYawOdom(0.0),
    wheelBase(0.210416){}

void stepper::attachMPU(MPU6050 &mpuRef)
{
     mpu = &mpuRef;
}

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

    float yawOdom = (wheelRadius / wheelBase) * (stepperPositionRight - stepperPositionLeft);
    float yawRateOdom = (wheelRadius / wheelBase) * (stepperMotorRadRight - stepperMotorRadLeft);

    float yawIMU = mpu->getAngleZ() * DEG_TO_RAD;     // Converte de graus para radianos
    float yawRateIMU = mpu->getGyroZ() * DEG_TO_RAD;

    const float alpha = 0.98;
    yawAngle = alpha * yawOdom + (1.0 - alpha) * yawIMU;
    yawRate = alpha * yawRateOdom + (1.0 - alpha) * yawRateIMU;

    lastYawOdom = yawOdom;
}

float stepper::getRobotPosition() const {
    return pendulumPosition;
}

float stepper::getRobotVelocity() const {
    return pendulumVelocity;
}

float stepper::getYawAngle() const {
    return yawAngle;
}

float stepper::getYawRate() const {
    return yawRate;
}