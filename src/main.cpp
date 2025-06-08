
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <AccelStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

#include "webSocketServer.h"
#include "MeasuresIMU.h"
#include "stepper.h"
#include "TS_Fuzzy.h"

// #define AS5600_ADDRESS 0x36
#define SAFE_ANGLE 0.8 //rad
#define MAX_STEPS 10000
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

AsyncWebServer server(80);

void IRAM_ATTR onTimerLeft();
void IRAM_ATTR onTimerRight();
void controlTask(void *parameter);

float computeLQRControl(float theta, float thetaRate, float position, float velocity);

float controlSteps = 0.0;
float Fm = 0.0, M = 0.202;
float a = 0.0;
float vel = 0.0;

//Ganhos para modelo do pêndulo sobre o carro
float K1[4] = {-49.9657,   -4.7947,  -16.3911,  -11.3871};//-5 graus
float K2[4] = {-39.7073,   -3.4122,  -10.6711,   -7.6583};
// float K2[4] = { -49.6606,   -4.8163,  -14.4073,  -10.3396}; //30 graus
// float K3[4] = { -49.6606,   -4.8163,  -14.4073,  -10.3396}; //-30 graus
// float K4[4] = {-71.0149,   -6.2501,  -11.4300,   -8.2029}; //45 graus
// float K5[4] = {-71.0149,   -6.2501,  -11.4300,   -8.2029};//-45 graus
float K3[4] = {-49.9657,   -4.7947,  -16.3911,  -11.3871};//-2 graus

float Ts = 8;

float Ttheta = 0.0;
float theta = 0.0, thetaRate = 0.0;
float pendulumPosition = 0.0, pendulumVelocity = 0.0;
volatile float refPosition = 0.0f;
bool referenceEnabled = false; //Flag para ativar a referência
unsigned long referenceStartTime = 0; //Intervalo que o sinal de refeerência inicia
float errorPosition = 0.0;
volatile float thetaDeg = 0.0;
volatile float thetaRateDeg = 0.0;


unsigned long currentMillis;
static unsigned long prevMillis = 0;

float lastVel = 0;

typedef struct {
  float theta;
  float pendulumPosition;
  float Ttheta;
  float controlSteps;
  // float h1, h2, h3;
  // float refPosition;
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

    
    // tsController.set5Gains(K1, K2, K3, K4, K5);
    // tsController.setSigmaDegrees(2);
    // tsController.set3Gains(K1, K2, K3);

    //------------------INICIALIZAÇÃO DOS MOTORES---------------//
    stepperStates.enableMotors(EN);
    stepperStates.attachMPU(imu);
    stepperStates.initMotors(MAX_STEPS, 8000);
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

    xTaskCreatePinnedToCore(
      controlTask,           // função da task
      "ControlTask",         // nome
      8192,                  // stack size
      NULL,                  // param
      1,                     // prioridade
      &controlTaskHandle,    // handle
      1                      // core 1 (para isolar do WiFi/loop)
    );

    referenceStartTime = millis();

    xTaskCreatePinnedToCore(
      [](void *param) {
        SensorData received;
        while (true) {
          if (xQueueReceive(dataQueue, &received, portMAX_DELAY)) {
            // Serial.printf("theta=%.2f°, h1=%.2f, h2=%.2f, h3=%.2f\n",
            //               received.theta * RAD_TO_DEG);
                          // received.h1, received.h2, received.h3);

            // CSV-style output
            Serial.print(received.theta * RAD_TO_DEG); Serial.print(",");
            Serial.print(received.pendulumPosition * 100); Serial.print(",");
            Serial.print(received.controlSteps); Serial.print(",");
            Serial.println(received.Ttheta);
          }

          // Optional: monitor stack usage
          // Serial.printf("Stack: %d bytes\n", uxTaskGetStackHighWaterMark(NULL));

          vTaskDelay(pdMS_TO_TICKS(8));
        }
      },
      "TransmitTask",
      4096,
      NULL,
      1,
      NULL,
      0 // Core 0
    );

}

void loop(){
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
  const float dt = pdTICKS_TO_MS(xFrequency) / 1000.0f;

  while (true) {

    imu.update();
    imu.updateFilter();
    stepperStates.update();

    theta = imu.getIMUAngleY();
    thetaRate = imu.getIMUGyroY();
    pendulumPosition = stepperStates.getRobotPosition();
    pendulumVelocity = stepperStates.getRobotVelocity();

    // float h1, h2, h3;
    // tsController.compute3Memberships(theta, h1, h2, h3);
    // Serial.printf("theta=%.2f°, h1=%.2f, h2=%.2f, h3=%.2f\n", theta * RAD_TO_DEG, h1, h2, h3);

    // if (!referenceEnabled && millis() - referenceStartTime > 5000) {
    //     referenceEnabled = true;
    // }

    // if (referenceEnabled) {
    //     refPosition = 0.1f;  // Example step to 10 cm
    //     // Or use sinusoidal:
    //     // float t = (millis() - referenceStartTime) / 1000.0;
    //     // refPosition = 0.05f * sin(2 * PI * 0.1 * t);
    // } else {
    //     refPosition = 0.0f;
    // }
    // errorPosition = pendulumPosition - refPosition;

    if (fabs(theta) < SAFE_ANGLE){;
        Ttheta = computeLQRControl(theta, thetaRate, pendulumPosition, pendulumVelocity);
        float Fm = Ttheta/2;
        float a = Fm / M;
        vel = lastVel + a * dt; // Ts = 8ms

        controlSteps = (vel * STEPS_PER_REVOLUTION) / (2 * PI * wheelRadius);

        controlSteps = constrain(controlSteps, -MAX_STEPS, MAX_STEPS);

        stepperStates.setMotorSpeed(controlSteps, -controlSteps);

        lastVel = vel;
    } else {
        stepperStates.setMotorSpeed(0, 0);
        vel = 0;
        lastVel = 0;
    }

    SensorData data = {};
    data.theta = theta;
    data.pendulumPosition = pendulumPosition;
    data.Ttheta = Ttheta;
    data.controlSteps = controlSteps;
    // data.h1 = h1;
    // data.h2 = h2;
    // data.h3 = h3;
    if (xQueueSend(dataQueue, &data, 0) != pdPASS) {
      Serial.println("Queue full! Dropping data.");
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

float computeLQRControl(float theta, float thetaRate, float position, float velocity) {
    float u = 0;
    float K[4] = {-38.2034,   -3.7559,  -10.8308, -7.8431};
    u = -(K[0] * theta + 
          K[1] * thetaRate + 
          K[2] * position + 
          K[3] * velocity);
    return u;
}