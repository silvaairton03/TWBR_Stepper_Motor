
#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "MeasuresIMU.h"
#include "stepper.h"
#include "TS_Fuzzy.h"
#include "coupledController.h"

// #define AS5600_ADDRESS 0x36
#define SAFE_ANGLE 0.8 //rad
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
coupledController controller;
TS_Fuzzy tsController;
stepper stepperStates(STEP, DIR, STEP_2, DIR_2,
  wheelRadius, stepsPerRev);

void IRAM_ATTR onTimerLeft();
void IRAM_ATTR onTimerRight();
void controlTask(void *parameter);
void commandTask(void *parameter);

float controlSteps = 0.0;
float Fm = 0.0, M = 0.208;
float a = 0.0;
float vel = 0.0;

//Ganhos para modelo do pêndulo sobre o carro
float K1[4] = {-39.7073,   -3.4122,  -10.6711,   -7.6583};
float K2[4] = { -49.6606,   -4.8163,  -14.4073,  -10.3396}; //30 graus
float K3[4] = { -49.6606,   -4.8163,  -14.4073,  -10.3396}; //-30 graus
float K4[4] = {-71.0149,   -6.2501,  -11.4300,   -8.2029}; //45 graus
float K5[4] = {-71.0149,   -6.2501,  -11.4300,   -8.2029};//-45 gruas

float Kdelta[2] = {5.7480,  1.3461};
float Ts = 8;

float Ttheta = 0.0, Tdelta = 0.0;
float Tl = 0.0, Tr = 0.0;
float theta = 0.0, thetaRate = 0.0;
float pendulumPosition = 0.0, pendulumVelocity = 0.0;
float refPosition = 5.0, errorPosition = 0.0;
float delta = 0.0,deltaRate  = 0.0;
volatile float thetaDeg = 0.0;
volatile float thetaRateDeg = 0.0;

unsigned long currentMillis;
static unsigned long prevMillis = 0;

float lastVel = 0;

typedef struct {
  float theta;
  float thetaRate;
} SensorData;

TaskHandle_t controlTaskHandle = NULL;
QueueHandle_t dataQueue;

void setup(){
    pinMode(2, OUTPUT);
    Serial.begin(115200);

    Wire.begin(21,22,800000L);

    delay(1000);

    dataQueue = xQueueCreate(100, sizeof(SensorData));
    if (dataQueue == NULL) {
      Serial.println("Erro ao criar a fila!");
      while (true);  // trava o sistema se falhar
    }

    // Controlador
    controller.setFuzzyGains(K1, K2, K3, K4, K5);
    controller.setDeltaGains(Kdelta);

    //------------------INICIALIZAÇÃO DOS MOTORES---------------//
    stepperStates.enableMotors(EN);
    stepperStates.attachMPU(imu);
    stepperStates.initMotors(10000, 8000);
    stepperStates.initTimers(onTimerLeft, onTimerRight);
    //------------------------------------------------------------//

    //-----------------CONFIGURAÇÃO MPU6050-----------------------//
    byte status = imu.beginWithLogging(0, 0, true);
    if (status != 0) {
        Serial.println(F("MPU6050 initialization failed!"));
        while (true); // trava o sistema caso a inicialização falhe
    }
   //------------------------------------------------------------//

    delay(2000);

    xTaskCreatePinnedToCore(
      controlTask,           // função da task
      "ControlTask",         // nome
      8192,                  // stack size
      NULL,                  // param
      1,                     // prioridade
      &controlTaskHandle,    // handle
      1                      // core 1 (para isolar do WiFi/loop)
    );

    // xTaskCreatePinnedToCore(
    //   commandTask,
    //   "CommandTask",
    //   4096,
    //   NULL,
    //   1,
    //   NULL,
    //   0  // Roda no core 0, separado do controle
    // );

    

    // xTaskCreatePinnedToCore(
    //   [] (void *param) {
    //     SensorData received;
    //     while (true) {
    //       if (xQueueReceive(dataQueue, &received, portMAX_DELAY)) {
    //         Serial.print(received.theta*RAD_2_DEG, 4);
    //         Serial.print(",");
    //         Serial.println(received.thetaRate*RAD_2_DEG, 4);
    //       }
    //       vTaskDelay(pdMS_TO_TICKS(8)); // impressão a cada 20ms
    //     }
    //   },
    //   "TransmitTask",
    //   4096,
    //   NULL,
    //   1,    // prioridade menor que a de controle
    //   NULL,
    //   0     // Core 0
    // );
}

void loop(){
  // static unsigned long lastPrint = 0;
  // if (millis() - lastPrint >= 20){
  //   Serial.print(thetaDeg);
  //   Serial.print(",");
  //   Serial.println(thetaRateDeg);
  // }
  // unsigned long timerdiff = currentMillis - prevMillis;
  
  // imu.update();
  // imu.updateFilter();
  // stepperStates.update();

  // theta = imu.getIMUAngleY();
  // thetaRate = imu.getFusedRadSpeed();
  // pendulumPosition = stepperStates.getRobotPosition();
  // pendulumVelocity = stepperStates.getRobotVelocity();
  // delta = stepperStates.getYawAngle();
  // deltaRate = stepperStates.getYawRate();

  // controller.updateStates(theta, thetaRate, pendulumPosition, pendulumVelocity, delta, deltaRate);
  // controller.computeTorques(Ttheta, Tdelta);


  // if (timerdiff >= Ts){
  //   // //-------------------CONTROLADOR LQR---------------------------//
  //   if (fabs(theta) < SAFE_ANGLE){
  //       // u = tsController.computeControl(theta, thetaRate, pendulumPosition, pendulumVelocity);

  //       float Tl = 0.5 * Ttheta + 0.5 * Tdelta;
  //       float Tr = 0.5 * Ttheta - 0.5 * Tdelta;

  //       Fm = (Tl+Tr)/2;
  //       a = Fm / M;
  //       vel = lastVel + a * (Ts / 1000.0);

  //       controlSteps = (vel * STEPS_PER_REVOLUTION) / (2* PI * wheelRadius);


  //       if (fabs(controlSteps) > MAX_STEPS) {
  //           controlSteps = (controlSteps > 0) ? MAX_STEPS : -MAX_STEPS;
  //       }

  //       stepperStates.setMotorSpeed(controlSteps, -controlSteps);

  //       lastVel = vel;

  //     } else {
  //         stepperStates.setMotorSpeed(0,0);
  //         lastVel = 0; 
  //     }
  //   // //-------------------------------------------------------------//
  //   prevMillis = currentMillis;
  // }

  // Serial.print(theta * 57.2958); Serial.print(",");
  // Serial.print(thetaRate * 57.2958); Serial.print(",");
  // Serial.println(Ttheta);
}


void IRAM_ATTR onTimerLeft() {
  stepperStates.runLeftMotor();
}

void IRAM_ATTR onTimerRight() {
  stepperStates.runRightMotor();
}

void controlTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(8); // 8ms

  while (true) {
    imu.update();
    imu.updateFilter();
    stepperStates.update();

    theta = imu.getIMUAngleY();
    thetaRate = imu.getFusedRadSpeed();
    pendulumPosition = stepperStates.getRobotPosition();
    errorPosition = (refPosition/100.0f) - pendulumPosition;
    pendulumVelocity = stepperStates.getRobotVelocity();
    delta = stepperStates.getYawAngle();
    deltaRate = stepperStates.getYawRate();

    controller.updateStates(theta, thetaRate, pendulumPosition, pendulumVelocity, delta, deltaRate);
    controller.computeTorques(Ttheta, Tdelta);

    if (fabs(theta) < SAFE_ANGLE){
        float Tl = 0.5 * Ttheta + 0.5 * Tdelta;
        float Tr = 0.5 * Ttheta - 0.5 * Tdelta;

        float Fm = (Tl+Tr)/2;
        float a = Fm / M;
        vel = lastVel + a * (8.0 / 1000.0); // Ts = 8ms

        controlSteps = (vel * STEPS_PER_REVOLUTION) / (2 * PI * wheelRadius);

        controlSteps = constrain(controlSteps, -MAX_STEPS, MAX_STEPS);

        stepperStates.setMotorSpeed(controlSteps, -controlSteps);

        lastVel = vel;
    } else {
        stepperStates.setMotorSpeed(0, 0);
        vel = 0;
    }

    // SensorData data = {theta, thetaRate};
    // xQueueSend(dataQueue, &data, 0);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// void commandTask(void *parameter) {
//   while (true) {
//     if (Serial.available()) {
//       String input = Serial.readStringUntil('\n');
//       input.trim();

//       if (input.startsWith("ref=")) {
//         String valueStr = input.substring(4);
//         float value = valueStr.toFloat();
//         refPosition = value / 100.0f;  // cm -> m

//         Serial.print("Nova referência de posição: ");
//         Serial.print(refPosition);
//         Serial.println(" m");
//       }
//     }
//     vTaskDelay(pdMS_TO_TICKS(10)); // pequena espera para não travar o sistema
//   }
// }


