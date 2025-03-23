
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
#define SAFE_ANGLE 0.6 //rad
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

float pitch, roll, angularVelocity;
float rollOffset;

float u = 0.0;
float omega = 0.0;

float theta = 0.0;
float thetaRate = 0.0, fusedThetaRate = 0.0;
const float wheelRadius = 0.0325;

// float state[4] = {0, 0, 0, 0};
// // float K[4] = {-0.5359, 0.1557};
// //float K[2] = {1.4941, 2.5642};
// //float k[4] = {0, -9.6591, -21.0637, -4.4535};
float k[4] = {-39.5113,   -3.0546,  -10.5095,   -7.4827}; //Ts = 0.008s
// //float k[4] = {-38.6303,   -2.9825,  -10.1185,   -7.2154}; //Ts = 0.01
// float k[4] = {-37.5724,  -3.8561,   -9.1731,   -6.8592}; //Ts = 0.02
float Ts = 8;
float M = 0.208;

const float stepAngle = 0.05625; // 1/32
float n = 0.0;

float stepperMotorSpeedLeft = 0.0;
float stepperMotorRadLeft = 0.0;
float stepperPositionLeft = 0.0;

float stepperMotorSpeedRight = 0.0;
float stepperMotorRadRight = 0.0;
float stepperPositionRight = 0.0;

float pendulumPosition = 0.0;
float pendulumVelocity = 0.0;

float rpm_to_rad = 0.104719755;

unsigned long currentMillis;
unsigned long prevMillis = 0;
float lastVel = 0;


void setup(){
    pinMode(2, OUTPUT);
    Serial.begin(115200);

    Wire.begin(21,22,800000L);

    delay(1000);
    
    // sensorRight.begin();
    // sensorLeft.begin();

    // delay(2000);

    //------------------INICIALIZAÇÃO DOS MOTORES---------------//
    // pinMode(EN, OUTPUT);
    // pinMode(DIR, OUTPUT);
    // pinMode(STEP, OUTPUT);
    // pinMode(DIR_2, OUTPUT);
    // pinMode(STEP_2, OUTPUT);
    // digitalWrite(EN, LOW);

    // // //MOTOR 1
    // stepperLeft.setMaxSpeed(8000);
    // stepperLeft.setAcceleration(10000);
    // stepperLeft.setSpeed(0);
    
    // // //MOTOR 2
    // stepperRight.setMaxSpeed(8000);
    // stepperRight.setAcceleration(10000);
    // stepperRight.setSpeed(0);

    // Serial.println("Motores iniciliazdos.");

    // timerLeft = timerBegin(0, 80, true);    // Timer 0 for left motor
    // timerAttachInterrupt(timerLeft, &onTimerLeft, true);
    // timerAlarmWrite(timerLeft, 100, true);   // call every 100 µs (adjust as needed)
    // timerAlarmEnable(timerLeft);
  
    // timerRight = timerBegin(1, 80, true);   // Timer 1 for right motor
    // timerAttachInterrupt(timerRight, &onTimerRight, true);
    // timerAlarmWrite(timerRight, 100, true);  // call every 100 µs
    // timerAlarmEnable(timerRight);

    // Serial.println("Timer inicializado");

    // delay(1000);
    // //------------------------------------------------------------//

    // //-----------------CONFIGURAÇÃO MPU6050-----------------------//
    if (imu.beginWithLogging(0, 0, true) != 0) {
      Serial.println("MPU6050 failed!");
      while (1);
    }

   //------------------------------------------------------------//
    
    delay(2000);
}

void loop(){
    currentMillis = millis();
    unsigned long timerdiff = currentMillis - prevMillis;
    imu.update();

    theta = imu.getIMUAngleY();
    thetaRate = imu.getIMUGyroY();
    fusedThetaRate = imu.getFusedRadSpeed();

    pendulumPosition = stepperStates.getRobotPosition();
    pendulumVelocity = stepperStates.getRobotVelocity();
    

    float Fm = 0.0;
    float a = 0.0;
    float vel = 0.0;
    float controlSteps = 0.0;

    if (timerdiff >= Ts){
      stepperStates.update();
        

      //-------------------CONTROLADOR LQR---------------------------//
      if (fabs(theta) < SAFE_ANGLE){
          u = -(k[0]*(theta) + k[1]*fusedThetaRate + k[2]*pendulumPosition + k[3]*pendulumVelocity);

      //     Fm = u / 2.0;
      //     a = Fm / M;

      //     vel = lastVel + a * (Ts/1000.0);

      //     controlSteps = (vel * STEPS_PER_REVOLUTION) / (2* PI * wheelRadius);


      //     if (fabs(controlSteps) > MAX_STEPS) {
      //         controlSteps = (controlSteps > 0) ? MAX_STEPS : -MAX_STEPS;
      //     }

      //     stepperLeft.setSpeed(controlSteps);
      //     stepperRight.setSpeed(-controlSteps);

      //     lastVel = vel;

      //   } else {
      //       stepperLeft.setSpeed(0);
      //       stepperRight.setSpeed(0);
      //       lastVel = 0; 
      }
      //-------------------------------------------------------------//
      prevMillis = currentMillis;
    }
    // Serial.print(theta * 57.2958); Serial.print(",");
    // Serial.println(thetaRate * 57.2958);
    Serial.print(theta); Serial.print(",");
    Serial.print(fusedThetaRate); Serial.print(",");
    Serial.println(u);
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


float yawAS5600(float leftAngle, float rightAngle)
{
  const float wheelRadius = 0.0325;
  const float lenght = 0.210416;

  float yawMagEncoder = (wheelRadius/lenght) * (rightAngle - leftAngle);

  return yawMagEncoder;
}
