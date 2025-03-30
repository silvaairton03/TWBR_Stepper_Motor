
#include <Arduino.h>
#include <Wire.h>
// #include "MPU6050_light.h"
#include"MeasuresIMU.h"
#include<AccelStepper.h>
#include"stepper.h"
#include"AS5600Sensor.h"
#include"freertos/FreeRTOS.h"
#include"freertos/task.h"

#define AS5600_ADDRESS 0x36
#define STEPS_PER_REVOLUTION 6400
#define SAFE_ANGLE 0.6f //rad
#define MAX_STEPS 8000
#define MICROSTEP_DIVISOR 32

MeasuresIMU imu(Wire);

volatile bool updateMotors = false;

float totalAngleLeftMotor = 0, totalAngleRightMotor = 0;

// void calculateRPM_position_Accel();

float yawAS5600(float leftAngle, float rightAngle);

const int EN = 32;
const int DIR = 23;
const int STEP = 19;

const int DIR_2 = 27;
const int STEP_2 = 18;

const float r = 0.0325;

AccelStepper stepperLeft(1, STEP, DIR);
AccelStepper stepperRight(1, STEP_2, DIR_2);
stepper stepperStates(stepperLeft, stepperRight, r, STEPS_PER_REVOLUTION);

hw_timer_t *timerLeft = NULL;
hw_timer_t *timerRight = NULL;

void IRAM_ATTR onTimerLeft();
void IRAM_ATTR onTimerRight();
void computeMembership(float theta, float &h1, float &h2, float &h3);
void updateStates();
float computeTScontrol();
void applyMotorCommand(float u);

float theta = 0.0, fusedThetaRate = 0.0;
float pendulumPosition = 0.0, pendulumVelocity = 0.0;
float lastVel = 0.0;
const float wheelRadius = 0.0325;


float k[4] = {-39.5113,   -3.0546,  -10.5095,   -7.4827}; //Ts = 0.008s

float K1[4] = {-39.7073,   -3.4122,  -10.6711,   -7.6583};
// float K2[4] = {-71.0149,   -6.2501,  -11.4300,   -8.2029};
// float K3[4] = {-71.0149,   -6.2501,  -11.4300,   -8.2029};
float K2[4] = { -49.6606,   -4.8163,  -14.4073,  -10.3396};
float K3[4] = { -49.6606,   -4.8163,  -14.4073,  -10.3396};
float Ts = 8;
float M = 0.208;

const float stepAngle = 0.05625; // 1/32
float n = 0.0;

float rpm_to_rad = 0.104719755;

unsigned long currentMillis;
unsigned long prevMillis = 0;

void setup(){
    pinMode(2, OUTPUT);
    Serial.begin(115200);

    Wire.begin(21,22,800000L);

    delay(1000);
    
    // sensorRight.begin();
    // sensorLeft.begin();

    // delay(2000);

    //------------------INICIALIZAÇÃO DOS MOTORES---------------//
    pinMode(EN, OUTPUT);
    pinMode(DIR, OUTPUT);
    pinMode(STEP, OUTPUT);
    pinMode(DIR_2, OUTPUT);
    pinMode(STEP_2, OUTPUT);
    digitalWrite(EN, LOW);

    // //MOTOR 1
    stepperLeft.setMaxSpeed(8000);
    stepperLeft.setAcceleration(10000);
    stepperLeft.setSpeed(0);
    
    // //MOTOR 2
    stepperRight.setMaxSpeed(8000);
    stepperRight.setAcceleration(10000);
    stepperRight.setSpeed(0);

    Serial.println("Motores iniciliazdos.");

    timerLeft = timerBegin(0, 80, true);    // Timer 0 for left motor
    timerAttachInterrupt(timerLeft, &onTimerLeft, true);
    timerAlarmWrite(timerLeft, 100, true);   // call every 100 µs (adjust as needed)
    timerAlarmEnable(timerLeft);
  
    timerRight = timerBegin(1, 80, true);   // Timer 1 for right motor
    timerAttachInterrupt(timerRight, &onTimerRight, true);
    timerAlarmWrite(timerRight, 100, true);  // call every 100 µs
    timerAlarmEnable(timerRight);

    Serial.println("Timer inicializado");

    delay(1000);
    // //------------------------------------------------------------//

    // //-----------------CONFIGURAÇÃO MPU6050-----------------------//
    if (imu.beginWithLogging(0, 0, true) != 0) {
      Serial.println("MPU6050 failed!");
      while (1);
    }

   //------------------------------------------------------------//
    
    delay(2000);
}

void loop() {
  currentMillis = millis();
  unsigned long timerdiff = currentMillis - prevMillis;
  imu.update();

  theta = imu.getIMUAngleY();
  float thetaRate = imu.getIMUGyroY();
  fusedThetaRate = imu.getFusedRadSpeed();
  pendulumPosition = stepperStates.getRobotPosition();
  pendulumVelocity = stepperStates.getRobotVelocity();

  float sigma = radians(20);
  float mf1 = exp(-pow((theta - radians(0)) / sigma, 2));
  float mf2 = exp(-pow((theta - radians(45)) / sigma, 2));
  float mf3 = exp(-pow((theta + radians(45)) / sigma, 2));

  float total = mf1 + mf2 + mf3;
  float h1 = mf1 / total;
  float h2 = mf2 / total;
  float h3 = mf3 / total;

  float x[4] = {theta, fusedThetaRate, pendulumPosition, pendulumVelocity};

  // Auto reset if robot is vertically balanced and stable
  if (fabs(theta) < 0.02 && fabs(fusedThetaRate) < 0.05) {
    pendulumPosition = 0;
    pendulumVelocity = 0;
  }

  float u = 0.0;
  if (timerdiff >= Ts) {
    stepperStates.update();

    // if (fabs(theta) < SAFE_ANGLE) {
    //   // Controle Fuzzy TS com 3 MF
    //   for (int i = 0; i < 4; i++) {
    //     float Ki = h1 * K1[i] + h2 * K2[i] + h3 * K3[i];
    //     u += -Ki * x[i];
    //   }

    //   float Fm = u / 2.0;
    //   float a = Fm / M;
    //   float vel = lastVel + a * (Ts / 1000.0);

    //   if (fabs(vel) > 1.0) vel = (vel > 0) ? 1.0 : -1.0;

    //   float controlSteps = (vel * STEPS_PER_REVOLUTION) / (2 * PI * wheelRadius);

    //   if (fabs(controlSteps) > MAX_STEPS) {
    //     controlSteps = (controlSteps > 0) ? MAX_STEPS : -MAX_STEPS;
    //   }

    //   stepperLeft.setSpeed(controlSteps);
    //   stepperRight.setSpeed(-controlSteps);

    //   lastVel = vel;
    // } else {
    //   stepperLeft.setSpeed(0);
    //   stepperRight.setSpeed(0);
    //   lastVel = 0;
    //   u = 0;
    // }

    prevMillis = currentMillis;
  }
  Serial.print(1.60); Serial.print(",");
  Serial.print(theta); Serial.print(",");
  Serial.print(thetaRate); Serial.print(",");
  Serial.println(-1.60);
  // Serial.print(",");
  // Serial.print(pendulumPosition); Serial.print(",");
  // Serial.print(pendulumVelocity); Serial.print(",");
  // Serial.println(u);
}


void IRAM_ATTR onTimerLeft() {
  stepperLeft.runSpeed();
}

void IRAM_ATTR onTimerRight() {
  stepperRight.runSpeed();
}

void computeMembership(float theta, float &h1, float &h2, float &h3)
{
  float sigma = radians(20);
  float mu1 = exp(-pow((theta - radians(0)) / sigma, 2));
  float mu2 = exp(-pow((theta - radians(45)) / sigma, 2));
  float mu3 = exp(-pow((theta + radians(45)) / sigma, 2));

  float total = mu1 + mu2 + mu3;
  h1 = mu1 / total;
  h2 = mu2 / total;
  h3 = mu3 / total;
}

void updateStates()
{
  imu.update();
  theta = imu.getIMUAngleY();
  fusedThetaRate = imu.getFusedRadSpeed();
  pendulumPosition = stepperStates.getRobotPosition();
  pendulumVelocity = stepperStates.getRobotVelocity();
}

float computeTScontrol()
{
  float h1, h2, h3;
  computeMembership(theta, h1, h2, h3);

  float state[4] = {theta, fusedThetaRate, pendulumPosition, pendulumVelocity};
  float u = 0.0;

  for (int i = 0; i < 4; i++) {
    float Ki = h1 * K1[i] + h2 * K2[i] + h3 * K3[i];
    u += -Ki * state[i];
  }
  return u;
}

float yawAS5600(float leftAngle, float rightAngle)
{
  const float wheelRadius = 0.0325;
  const float lenght = 0.210416;

  float yawMagEncoder = (wheelRadius/lenght) * (rightAngle - leftAngle);

  return yawMagEncoder;
}

void applyMotorCommand(float u) {
  float Fm = u / 2.0f;
  float a = Fm / M;
  float vel = lastVel + a * (Ts / 1000.0f);

  float controlSteps = (vel * STEPS_PER_REVOLUTION) / (2.0f * PI * wheelRadius);

  controlSteps = constrain(controlSteps, -MAX_STEPS, MAX_STEPS);

  stepperLeft.setSpeed(controlSteps);
  stepperRight.setSpeed(-controlSteps);

  lastVel = vel;
}
