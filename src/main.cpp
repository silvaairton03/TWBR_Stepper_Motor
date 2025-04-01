
#include<Arduino.h>
#include<Wire.h>
#include<FastAccelStepper.h>
#include"MeasuresIMU.h"
#include"AS5600Sensor.h"


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
const int DIR_LEFT = 23;
const int STEP_LEFT = 19;

const int DIR_RIGHT = 27;
const int STEP_RIGHT = 18;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperLeft = NULL;
FastAccelStepper *stepperRight = NULL;

const float r = 0.0325;

float theta = 0.0, fusedThetaRate = 0.0;
float pendulumPosition = 0.0, pendulumVelocity = 0.0;
float lastVel = 0.0;
const float wheelRadius = 0.0325;


float k[4] = {-39.5113,   -3.0546,  -10.5095,   -7.4827}; //Ts = 0.008s

float K1[4] = {-39.7073,   -3.4122,  -10.6711,   -7.6583};
float K2[4] = { -49.6606,   -4.8163,  -14.4073,  -10.3396}; //30 graus
float K3[4] = { -49.6606,   -4.8163,  -14.4073,  -10.3396}; //-30 graus
float K4[4] = {-71.0149,   -6.2501,  -11.4300,   -8.2029}; //45 graus
float K5[4] = {-71.0149,   -6.2501,  -11.4300,   -8.2029};//-45 gruas

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

    engine.init();

    //CONFIGURAÇÃO MOTOR DA ESQUERDA
    stepperLeft = engine.stepperConnectToPin(STEP_LEFT);
    if (stepperLeft){
      stepperLeft->setDirectionPin(DIR_LEFT);
      stepperLeft->setEnablePin(EN);
      stepperLeft->setAutoEnable(true);
      stepperLeft->setAcceleration(1000);
    } else {
      Serial.println("Não foi possível conectar o motor da esquerda");
    }

    stepperRight = engine.stepperConnectToPin(STEP_RIGHT);
    if(stepperRight){
      stepperRight->setDirectionPin(DIR_RIGHT);
      stepperRight->setEnablePin(EN);
      stepperRight->setAutoEnable(true);
      stepperRight->setAcceleration(1000);
      } else {
        Serial.println("Não foi possível conectar o motor da direita");
    }
    
  // //-----------------CONFIGURAÇÃO MPU6050-----------------------//
  // if (imu.beginWithLogging(0, 0, true) != 0) {
  //   Serial.println("MPU6050 failed!");
  //   while (1);
  // }

  //------------------------------------------------------------//
}

void loop() {
}

float yawAS5600(float leftAngle, float rightAngle)
{
  const float wheelRadius = 0.0325;
  const float lenght = 0.210416;

  float yawMagEncoder = (wheelRadius/lenght) * (rightAngle - leftAngle);

  return yawMagEncoder;
}

