#include "stepper.h"

stepper::stepper(uint8_t stepPinLeft, uint8_t dirPinLeft,
                uint8_t stepPinRight, uint8_t dirPinRight,
                float wheelRadius, int stepsPerRevolution)
    :wheelRadius(wheelRadius), stepsPerRevolution(stepsPerRevolution),
    pendulumPosition(0.0), pendulumVelocity(0.0),
    stepPinLeft(stepPinLeft), dirPinLeft(dirPinLeft),
    stepPinRight(stepPinRight), dirPinRight(dirPinRight),
    timerLeft(nullptr), timerRight(nullptr)
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

void stepper::initTimers(void (*isrLeft)(), void (*isrRight)())
{
    timerLeft = timerBegin(0, 80, true);
    timerAttachInterrupt(timerLeft, isrLeft, true);
    timerAlarmWrite(timerLeft, 100, true);
    timerAlarmEnable(timerLeft);

    timerRight = timerBegin(1, 80, true);
    timerAttachInterrupt(timerRight, isrRight, true);
    timerAlarmWrite(timerRight, 100, true);
    timerAlarmEnable(timerRight);

    Serial.println("Timer inicializados");

    delay(2000);
}

void stepper::setMotorSpeed(float leftSpeed, float rightSpeed)
{
    stepperLeft->setSpeed(leftSpeed);
    stepperRight->setSpeed(rightSpeed);
}

void stepper::resetSteps()
{
    stepperLeft->setCurrentPosition(0);
    stepperRight->setCurrentPosition(0);
    pendulumPosition = 0.0f;
    pendulumVelocity = 0.0f;
}

void stepper::runLeftMotor() {
    stepperLeft->runSpeed();
}

void stepper::runRightMotor(){
    stepperRight->runSpeed();
}

float stepper::getRobotPosition() const {
    return pendulumPosition;
}

float stepper::getRobotVelocity() const {
    return pendulumVelocity;
}
