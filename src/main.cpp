
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
#include "coupledController.h"

// #define AS5600_ADDRESS 0x36
#define SAFE_ANGLE 0.8 //rad
#define MAX_STEPS 9000
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

AsyncWebServer server(80);

void IRAM_ATTR onTimerLeft();
void IRAM_ATTR onTimerRight();
void controlTask(void *parameter);
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
  AwsEventType type, void *arg, uint8_t *data, size_t len);


float controlSteps = 0.0;
float Fm = 0.0, M = 0.208;
float a = 0.0;
float vel = 0.0;

//Ganhos para modelo do pêndulo sobre o carro
float K1[4] = {-39.7073,   -3.4122,  -10.6711,   -7.6583};
float K2[4] = { -49.6606,   -4.8163,  -14.4073,  -10.3396}; //30 graus
float K3[4] = { -49.6606,   -4.8163,  -14.4073,  -10.3396}; //-30 graus
float K4[4] = {-71.0149,   -6.2501,  -11.4300,   -8.2029}; //45 graus
float K5[4] = {-71.0149,   -6.2501,  -11.4300,   -8.2029};//-45 graus
float K6[4] = { -104.8055,  -10.5896,  -16.6980,  -11.9836};
float K7[4] = { -104.8055,  -10.5896,  -16.6980,  -11.9836};

float Ts = 8;

float Ttheta = 0.0;
float theta = 0.0, thetaRate = 0.0;
float pendulumPosition = 0.0, pendulumVelocity = 0.0;
volatile float refPosition = 0.0f;
float errorPosition = 0.0;
volatile float thetaDeg = 0.0;
volatile float thetaRateDeg = 0.0;

unsigned long currentMillis;
static unsigned long prevMillis = 0;

float lastVel = 0;
// typedef struct {
//   float theta;
//   float pendulumPosition;
// } SensorData;

TaskHandle_t controlTaskHandle = NULL;
// QueueHandle_t dataQueue;

const char* ssid = "Kenayvision2";
const char* password = "L59S70A95a97";

void setup(){
    pinMode(2, OUTPUT);
    Serial.begin(115200);
    Wire.begin(21,22,800000L);

    delay(1000);

    WiFi.begin(ssid, password);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\n[WiFi] Connected!");
    Serial.print("[WiFi] IP Address: ");
    Serial.println(WiFi.localIP());

    if (!SPIFFS.begin(true)) {
      Serial.println("SPIFFS mount failed");
      return;
    }

    // Serve static files
    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
    initWebSocket(server);
    server.begin();

    // dataQueue = xQueueCreate(100, sizeof(SensorData));
    // if (dataQueue == NULL) {
    //   Serial.println("Erro ao criar a fila!");
    //   while (true);  // trava o sistema se falhar
    // }

    tsController.set7Gains(K1, K2, K3, K4, K5, K6, K7);

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

    // xTaskCreatePinnedToCore(
    //   [] (void *param) {
    //     SensorData received;
    //     while (true) {
    //       if (xQueueReceive(dataQueue, &received, portMAX_DELAY)) {
    //         Serial.print(theta*RAD_2_DEG);
    //         Serial.print(",");
    //         Serial.println(pendulumPosition*100);
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
    pendulumVelocity = stepperStates.getRobotVelocity();
    errorPosition = pendulumPosition - refPosition;

    if (pendingCommand != NONE) {
      if (pendingCommand == FORWARD) {
        refPosition += 0.05f;
      } else if (pendingCommand == BACKWARD) {
        refPosition -= 0.05f;
      }
    
      refPosition = constrain(refPosition, -0.3f, 0.3f);
      pendingCommand = NONE;  // Reset flag
    }

    if (fabs(errorPosition) < 0.005f) {
      errorPosition = 0.0f;
    }

    if (fabs(theta) < SAFE_ANGLE){;
        Ttheta = tsController.computeControl7mf(theta, thetaRate, errorPosition, pendulumVelocity);

        float Fm = Ttheta/2;
        float a = Fm / M;
        vel = lastVel + a * (Ts / 1000.0); // Ts = 8ms

        controlSteps = (vel * STEPS_PER_REVOLUTION) / (2 * PI * wheelRadius);

        controlSteps = constrain(controlSteps, -MAX_STEPS, MAX_STEPS);

        stepperStates.setMotorSpeed(controlSteps, -controlSteps);

        lastVel = vel;
    } else {
        stepperStates.setMotorSpeed(0, 0);
        vel = 0;
    }

    // SensorData data = {theta, pendulumPosition};
    // xQueueSend(dataQueue, &data, 0);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

