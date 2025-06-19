
#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "MPU6050_light.h"
// #include "MeasuresIMU.h"
#include "stepper.h"
#include "TS_Fuzzy.h"

#define SAFE_ANGLE 0.6 //rad
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

// MeasuresIMU imu(Wire);
MPU6050 mpu(Wire);
TS_Fuzzy tsController;
stepper stepperStates(STEP, DIR, STEP_2, DIR_2,
  wheelRadius, stepsPerRev);

void IRAM_ATTR onTimerLeft();
void IRAM_ATTR onTimerRight();
void controlTask(void *parameter);
// void commandTask(void *parameter);

const float M = 0.208;
//Ganhos para modelo do pêndulo sobre o carro
float K1[4] = {-37.7319,   -3.7463,  -12.8162,   -8.5867};
float K2[4] = { -37.7155,   -3.7389,  -12.7855,   -8.5661};
float K3[4] = {-37.7319,   -3.7463,  -12.8162,   -8.5867};

// float Kdelta[2] = {5.7480,  1.3461};
float Ts = 8;

// float Ttheta = 0.0, Tdelta = 0.0;
// float Tl = 0.0, Tr = 0.0;
float theta = 0.0, thetaRate = 0.0;
float pendulumPosition = 0.0, pendulumVelocity = 0.0;

unsigned long currentMillis;
static unsigned long prevMillis = 0;
float lastVel = 0;

typedef struct {
  float theta;
  float pendulumPosition;
} SensorData;

TaskHandle_t controlTaskHandle = NULL;
QueueHandle_t dataQueue;

void setup(){
    pinMode(2, OUTPUT);
    Serial.begin(115200);

    Wire.begin(21,22,400000L);

    // delay(1000);

    dataQueue = xQueueCreate(100, sizeof(SensorData));
    if (dataQueue == NULL) {
      Serial.println("Erro ao criar a fila!");
      while (true);  // trava o sistema se falhar
    }

    // tsController.setSigmaDegrees(2.0F);
    // tsController.set3Gains(K1, K2, K3);
    // tsController.setMode(TS_Fuzzy::MF3);

    //------------------INICIALIZAÇÃO DOS MOTORES---------------//
    stepperStates.enableMotors(EN);
    stepperStates.initMotors(MAX_STEPS, 10000);
    stepperStates.initTimers(onTimerLeft, onTimerRight);
    //------------------------------------------------------------//

    //-----------------CONFIGURAÇÃO MPU6050-----------------------//
    byte status = mpu.begin();
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

    xTaskCreatePinnedToCore(
      controlTask,           // função da task
      "ControlTask",         // nome
      8192,                  // stack size
      NULL,                  // param
      2,                     // prioridade
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

    

    xTaskCreatePinnedToCore(
      [] (void *param) {
        SensorData received;
        while (true) {
          if (xQueueReceive(dataQueue, &received, portMAX_DELAY)) {
            Serial.print(received.theta * RAD_2_DEG);
            Serial.print(",");
            Serial.println(received.pendulumPosition * 100);
          }
          vTaskDelay(pdMS_TO_TICKS(8)); // impressão a cada 20ms
        }
      },
      "TransmitTask",
      4096,
      NULL,
      1,    // prioridade menor que a de controle
      NULL,
      0     // Core 0
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

  while (true) {
    stepperStates.update();
    // mpu.update();

    // theta = mpu.getAngleY() * DEG_TO_RAD;
    // thetaRate = mpu.getGyroY() * DEG_TO_RAD;
    theta = 0.0;
    thetaRate = 0.0;
    // pendulumPosition = stepperStates.getRobotPosition();
    // errorPosition = pendulumPosition - refPosition;
    // pendulumVelocity = stepperStates.getRobotVelocity();
    pendulumPosition = 0.0;
    pendulumVelocity = 0.0;


    float controlSteps = 0.0;
    float u = 0.0;
    float Fm = 0.0;
    float a = 0.0;
    float vel = 0.0;


    if (fabs(theta) < SAFE_ANGLE){
        // u = tsController.computeControl(theta, thetaRate, pendulumPosition, pendulumVelocity);
        // float Fm = u/2;
        // float a = Fm / M;
        // vel = lastVel + a * (Ts / 1000.0); // Ts = 8ms
        float Fm_local = u/2; // Mude o nome da variável para evitar conflito de escopo se Fm já foi declarada globalmente
        float a_local = Fm_local / M; // Mude o nome da variável
        vel = lastVel + a_local * (Ts / 1000.0); 

        // controlSteps = (vel * STEPS_PER_REVOLUTION) / (2 * PI * wheelRadius);

        // controlSteps = constrain(controlSteps, -MAX_STEPS, MAX_STEPS);

        // stepperStates.setMotorSpeed(controlSteps, -controlSteps);

        // lastVel = vel;
    } else {
        // stepperStates.setMotorSpeed(0, 0);
        // lastVel = 0;
    }

    SensorData data = {theta, pendulumPosition};
    xQueueSend(dataQueue, &data, 0);

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


