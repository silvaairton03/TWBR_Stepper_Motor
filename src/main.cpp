
#include <Arduino.h>
#include <Wire.h>
#include "MPU6050_light.h"
#include<AccelStepper.h>
#include"stepper.h"
#include"TS_Fuzzy.h"
#include"coupledController.h"

#define AS5600_ADDRESS 0x36
#define STEPS_PER_REVOLUTION 6400
#define SAFE_ANGLE 0.8 //rad
#define MAX_STEPS 8000
#define MICROSTEP_DIVISOR 32

// AS5600Sensor sensorRight(&Wire, AS5600_ADDRESS);    // This one uses the default Wire bus.
// AS5600Sensor sensorLeft(&Wire1, AS5600_ADDRESS);
MPU6050 mpu(Wire);
coupledController controller;
TS_Fuzzy tsController;

volatile bool updateMotors = false;

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

float u = 0.0;
const float wheelRadius = 0.0325;

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
float M = 0.208;

const float stepAngle = 0.05625; // 1/32
float n = 0.0;

float Ttheta = 0.0, Tdelta = 0.0;
float Tl = 0.0, Tr = 0.0;
float theta = 0.0;
float thetaRate = 0.0;
float pendulumPosition = 0.0;
float pendulumVelocity = 0.0;
float delta = 0.0;
float deltaRate = 0.0;

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

    //-----------------CONFIGURAÇÃO MPU6050-----------------------//
    byte status = mpu.begin();
    stepperStates.attachMPU(mpu);
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status!=0){ } // stop everything if could not connect to MPU6050

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    //mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
    mpu.calcOffsets(); // gyro and accelero
    Serial.println("Done!\n");
   //------------------------------------------------------------//

    
    delay(2000);
}

void loop(){
    currentMillis = millis();
    unsigned long timerdiff = currentMillis - prevMillis;
    
    mpu.update();
    stepperStates.update();

    theta = mpu.getAngleY() * 0.0174533;
    thetaRate = mpu.getGyroY() * 0.0174533;

    pendulumPosition = stepperStates.getRobotPosition();
    pendulumVelocity = stepperStates.getRobotVelocity();
    delta = stepperStates.getYawAngle();
    deltaRate = stepperStates.getYawRate();

    controller.updateStates(theta, thetaRate, pendulumPosition, pendulumVelocity, delta, deltaRate);

    float Fm = 0.0;
    float a = 0.0;
    float vel = 0.0;
    float controlSteps = 0.0;
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

          stepperLeft.setSpeed(controlSteps);
          stepperRight.setSpeed(-controlSteps);

          lastVel = vel;

        } else {
            stepperLeft.setSpeed(0);
            stepperRight.setSpeed(0);
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
  stepperLeft.runSpeed();
}

void IRAM_ATTR onTimerRight() {
  stepperRight.runSpeed();
}
