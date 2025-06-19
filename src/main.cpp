
#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/queue.h>
#include "MeasuresIMU.h"
#include "stepper.h"
#include "TS_Fuzzy.h"

// #define AS5600_ADDRESS 0x36
#define SAFE_ANGLE 0.4 //rad
#define MAX_STEPS 8000
#define MICROSTEP_DIVISOR 32
#define STEPS_PER_REVOLUTION 6400
float wheelRadius = 0.0325;
float stepsPerRev = 6400;

const int EN = 32;
const int DIR = 23;
const int STEP = 19;
const int DIR_2 = 27;
const int STEP_2 = 18;

MeasuresIMU imu(Wire);
TS_Fuzzy tsController;
stepper stepperStates(STEP, DIR, STEP_2, DIR_2,
  wheelRadius, stepsPerRev);

void IRAM_ATTR onTimerLeft();
void IRAM_ATTR onTimerRight();
// void controlTask(void *parameter);

float computeLQRControl(float theta, float thetaRate, float position, float velocity);
const float K_LQR[4] = {-38.2034, -3.7559, -10.8308, -7.8431};

float controlSteps = 0.0;
float Fm = 0.0, M = 0.208;
float a = 0.0;
float vel = 0.0;

//Ganhos para modelo do pêndulo sobre o carro
float K1[4] = {-49.9657,   -4.7947,  -16.3911,  -11.3871};//-5 graus
float K2[4] = {-39.7073,   -3.4122,  -10.6711,   -7.6583};
float K3[4] = {-49.9657,   -4.7947,  -16.3911,  -11.3871};//-2 graus

float Ts = 8;

float Ttheta = 0.0;
float theta = 0.0, thetaRate = 0.0;
float pendulumPosition = 0.0, pendulumVelocity = 0.0;
// volatile float refPosition = 0.0f;
// bool referenceEnabled = false; //Flag para ativar a referência
// unsigned long referenceStartTime = 0; //Intervalo que o sinal de refeerência inicia
// float errorPosition = 0.0;


unsigned long currentMillis;
static unsigned long prevControlMillis = 0; 
static unsigned long prevSerialMillis = 0;
const unsigned long controlInterval = 8;
const unsigned long serialInterval = 8; 

float lastVel = 0;

void setup(){
  pinMode(2, OUTPUT);
  Serial.begin(115200);
  Wire.begin(21,22,800000L);

  delay(1000);
  
  // tsController.set5Gains(K1, K2, K3, K4, K5);
  // tsController.setSigmaDegrees(2);
  // tsController.set3Gains(K1, K2, K3);

  //------------------INICIALIZAÇÃO DOS MOTORES---------------//
  stepperStates.enableMotors(EN);
  stepperStates.resetSteps();
  stepperStates.initMotors(MAX_STEPS, 10000);
  stepperStates.initTimers(onTimerLeft, onTimerRight);
  //------------------------------------------------------------//

  //-----------------CONFIGURAÇÃO MPU6050-----------------------//
  byte status = imu.beginWithLogging(0, 0, true);
  if (status != 0) {
      Serial.println(F("MPU6050 initialization failed!"));
      while (true); // trava o sistema caso a inicialização falhe
  }
  //------------------------------------------------------------//

  delay(1000);

  prevControlMillis = millis();
}

void loop(){
  currentMillis = millis();

   if (currentMillis - prevControlMillis >= Ts){
    prevControlMillis = currentMillis;
    imu.update();
    imu.updateFilter();
    stepperStates.update();

    theta = imu.getIMUAngleY(); // Ângulo do pêndulo em radianos
    thetaRate = imu.getFusedRadSpeed(); // Velocidade angular do pêndulo em rad/s
    // stepperStates.setMotorSpeed(1000, -1000);
    pendulumPosition = stepperStates.getRobotPosition();
    pendulumVelocity = stepperStates.getRobotVelocity();

    if (fabs(theta) < SAFE_ANGLE){
      Ttheta = computeLQRControl(theta, thetaRate, pendulumPosition, pendulumVelocity);
      float Fm = Ttheta / 2.0; 
      float a = Fm / M; 
      vel = lastVel + a * (Ts / 1000.0);

      controlSteps = (vel * STEPS_PER_REVOLUTION) / (2 * PI * wheelRadius);
      controlSteps = constrain(controlSteps, -MAX_STEPS, MAX_STEPS);
      stepperStates.setMotorSpeed(controlSteps, -controlSteps);

      lastVel = vel; 
    } else{
      stepperStates.setMotorSpeed(0, 0);
      vel = 0;
      lastVel = 0;
    }
  }

  Serial.print(theta * RAD_TO_DEG); Serial.print(",");
  Serial.print(thetaRate * RAD_2_DEG); Serial.print(",");
  Serial.println(thetaRate * RAD_2_DEG); 
}


void IRAM_ATTR onTimerLeft() {
  stepperStates.runLeftMotor();
}

void IRAM_ATTR onTimerRight() {
  stepperStates.runRightMotor();
}


float computeLQRControl(float theta, float thetaRate, float position, float velocity) {
    float u = 0;
    u = -(K_LQR[0] * theta + 
          K_LQR[1] * thetaRate + 
          K_LQR[2] * position + 
          K_LQR[3] * velocity);
    return u;
} 