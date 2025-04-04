#include "stepper.h"

stepper::stepper(uint8_t stepPinLeft, uint8_t dirPinLeft,
                uint8_t stepPinRight, uint8_t dirPinRight,
                float wheelRadius, int stepsPerRevolution)
    :wheelRadius(wheelRadius), stepsPerRevolution(stepsPerRevolution),
    pendulumPosition(0.0), pendulumVelocity(0.0),
    yawAngle(0.0), yawRate(0.0), lastYawOdom(0.0),
    wheelBase(0.210416),
    stepPinLeft(stepPinLeft), dirPinLeft(dirPinLeft),
    stepPinRight(stepPinRight), dirPinRight(dirPinRight)
    {
        stepperLeft = new AccelStepper(1, stepPinLeft, dirPinLeft);
        stepperRight = new AccelStepper(1, stepPinRight, dirPinRight);
    }

void stepper::enableMotors(uint8_t enPin)
{
    pinMode(enPin, OUTPUT);
    digitalWrite(enPin, LOW);
}

void stepper::attachMPU(MPU6050 &mpuRef)
{
     mpu = &mpuRef;
}

void stepper::update() {
    float stepperMotorSpeedLeft = stepperLeft->speed();
    float stepperMotorSpeedRight = stepperRight->speed();

    float stepperMotorRadLeft = stepperMotorSpeedLeft * 2 * PI / stepsPerRevolution;
    float stepperMotorRadRight = -stepperMotorSpeedRight * 2 * PI / stepsPerRevolution;

    float stepperPositionLeft = stepperLeft->currentPosition() * 2 * PI / stepsPerRevolution;
    float stepperPositionRight = -stepperRight->currentPosition() * 2 * PI / stepsPerRevolution;

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

void stepper::initMotors(float maxSpeed, float acceleration)
{
    stepperLeft->setMaxSpeed(maxSpeed);
    stepperLeft->setAcceleration(acceleration);
    stepperLeft->setSpeed(0);

    stepperRight->setMaxSpeed(maxSpeed);
    stepperRight->setAcceleration(acceleration);
    stepperRight->setSpeed(0);

    Serial.println("Motores iniciliazdos.");
    delay(1000);
}

void stepper::setMotorSpeed(float leftSpeed, float rightSpeed)
{
    stepperLeft->setSpeed(leftSpeed);
    stepperRight->setSpeed(rightSpeed);
}

void stepper::runMotors() {
    stepperLeft->runSpeed();
    stepperRight->runSpeed();
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