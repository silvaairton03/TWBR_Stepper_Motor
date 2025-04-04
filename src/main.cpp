
#include <Arduino.h>
#include <Wire.h>
#include<WiFi.h>
#include<ESPAsyncWebServer.h>
#include<AsyncTCP.h>
#include "MeasuresIMU.h"
#include<AccelStepper.h>
#include"stepper.h"
#include"TS_Fuzzy.h"
#include"coupledController.h"

#define AS5600_ADDRESS 0x36
#define STEPS_PER_REVOLUTION 6400
#define SAFE_ANGLE 0.8 //rad
#define MAX_STEPS 8000
#define MICROSTEP_DIVISOR 32

MeasuresIMU imu(Wire);
coupledController controller;
TS_Fuzzy tsController;

volatile bool updateMotors = false;

const int EN = 32;
const int DIR = 23;
const int STEP = 19;

const int DIR_2 = 27;
const int STEP_2 = 18;

hw_timer_t *timerLeft = NULL;
hw_timer_t *timerRight = NULL;

void IRAM_ATTR onTimerLeft();
void IRAM_ATTR onTimerRight();

float u = 0.0, controlSteps = 0.0;
const float wheelRadius = 0.0325;
float Fm = 0.0, M = 0.208;
float a = 0.0;
float vel = 0.0;

stepper stepperStates(STEP, DIR, STEP_2, DIR_2, wheelRadius, STEPS_PER_REVOLUTION);

//Ganhos para modelo do pêndulo sobre o carro
float K1[4] = {-39.7073,   -3.4122,  -10.6711,   -7.6583};
float K2[4] = { -49.6606,   -4.8163,  -14.4073,  -10.3396}; //30 graus
float K3[4] = { -49.6606,   -4.8163,  -14.4073,  -10.3396}; //-30 graus
float K4[4] = {-71.0149,   -6.2501,  -11.4300,   -8.2029}; //45 graus
float K5[4] = {-71.0149,   -6.2501,  -11.4300,   -8.2029};//-45 gruas

// float Ktheta[4] = {-39.5113,   -3.0546,  -10.5095,   -7.4827}; //Ts = 0.008s
float Kdelta[2] = {5.7480,  1.3461};
// //float k[4] = {-38.6303,   -2.9825,  -10.1185,   -7.2154}; //Ts = 0.01
// float k[4] = {-37.5724,  -3.8561,   -9.1731,   -6.8592}; //Ts = 0.02
float Ts = 8;
// float M = 0.104;

float Ttheta = 0.0, Tdelta = 0.0;
float Tl = 0.0, Tr = 0.0;
float theta = 0.0, thetaRate = 0.0;
float pendulumPosition = 0.0, pendulumVelocity = 0.0;
float delta = 0.0,deltaRate  = 0.0;

unsigned long currentMillis;
unsigned long prevMillis = 0;
float lastVel = 0;

void setup(){
    pinMode(2, OUTPUT);
    Serial.begin(115200);

    Wire.begin(21,22,800000L);

    delay(1000);

    // Controlador
    controller.setFuzzyGains(K1, K2, K3, K4, K5);
    controller.setDeltaGains(Kdelta);

    //------------------INICIALIZAÇÃO DOS MOTORES---------------//
    stepperStates.enableMotors(EN);
    stepperStates.attachMPU(imu);
    stepperStates.initMotors(10000, 8000);
    
    timerLeft = timerBegin(0, 80, true);    // Timer 0 for left motor
    timerAttachInterrupt(timerLeft, &onTimerLeft, true);
    timerAlarmWrite(timerLeft, 100, true);   // call every 100 µs (adjust as needed)
    timerAlarmEnable(timerLeft);
  
    timerRight = timerBegin(1, 80, true);   // Timer 1 for right motor
    timerAttachInterrupt(timerRight, &onTimerRight, true);
    timerAlarmWrite(timerRight, 100, true);  // call every 100 µs
    timerAlarmEnable(timerRight);

    Serial.println("Timer inicializado");

    // //------------------------------------------------------------//

    //-----------------CONFIGURAÇÃO MPU6050-----------------------//
    byte status = imu.beginWithLogging(0, 0, true);
    if (status != 0) {
        Serial.println(F("MPU6050 initialization failed!"));
        while (true); // trava o sistema caso a inicialização falhe
    }
   //------------------------------------------------------------//

    delay(2000);
}

void loop(){
    currentMillis = millis();
    unsigned long timerdiff = currentMillis - prevMillis;
    
    stepperStates.update();

    theta = imu.getIMUAngleY();
    thetaRate = imu.getFusedRadSpeed();
    pendulumPosition = stepperStates.getRobotPosition();
    pendulumVelocity = stepperStates.getRobotVelocity();
    delta = stepperStates.getYawAngle();
    deltaRate = stepperStates.getYawRate();

    controller.updateStates(theta, thetaRate, pendulumPosition, pendulumVelocity, delta, deltaRate);
    controller.computeTorques(Ttheta, Tdelta);


    if (timerdiff >= Ts){
      // //-------------------CONTROLADOR LQR---------------------------//
      if (fabs(theta) < SAFE_ANGLE){
          // u = tsController.computeControl(theta, thetaRate, pendulumPosition, pendulumVelocity);

          float Tl = 0.5 * Ttheta + 0.5 * Tdelta;
          float Tr = 0.5 * Ttheta - 0.5 * Tdelta;

          Fm = (Tl+Tr)/2;
          a = Fm / M;
          vel = lastVel + a * (Ts / 1000.0);

          controlSteps = (vel * STEPS_PER_REVOLUTION) / (2* PI * wheelRadius);


          if (fabs(controlSteps) > MAX_STEPS) {
              controlSteps = (controlSteps > 0) ? MAX_STEPS : -MAX_STEPS;
          }

          stepperStates.setMotorSpeed(controlSteps, -controlSteps);

          lastVel = vel;

        } else {
            stepperStates.setMotorSpeed(0,0);
            lastVel = 0; 
        }
      // //-------------------------------------------------------------//
      prevMillis = currentMillis;
    }

    Serial.print(theta * 57.2958); Serial.print(",");
    Serial.print(thetaRate * 57.2958); Serial.print(",");
    Serial.print(pendulumPosition); Serial.print(",");
    Serial.print(pendulumVelocity); Serial.print(",");
    Serial.print(Ttheta); Serial.print(",");
    Serial.println(Tdelta);
}


void IRAM_ATTR onTimerLeft() {
  stepperStates.runMotors();
}

void IRAM_ATTR onTimerRight() {
  stepperStates.runMotors();
}
