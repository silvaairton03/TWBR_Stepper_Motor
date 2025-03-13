#include "MPU6050.h"
#include<Wire.h>

float gyroX, gyroY, gyroZ;
float gyroXOffset, gyroYOffset, gyroZOffset;
float GyroPitch, FilteredPitchAngle;
// float y_hp[] = {0, 0}, x_hp[] = {0, 0}, y_lp[] = {0, 0}, x_lp[] = {0, 0};
// float const_a = 0.98;
int calibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float AngleYaw = 0.0;
unsigned long lastTime = 0;
bool calibrated = false;
float rad_to_deg = 180/3.141592654;

void initMPU6050(){
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void setupMPU6050(){
  //---------------Configuração do Giróscopio----------------------//
  Wire.beginTransmission(0x68);//Inicia comunicação I2C com MPU6050
  //Filtro Passa-baixas Digital
  Wire.write(0x1A);//Endereço de registrador
  Wire.write(0x05);//Largura de banda de 10Hz
  Wire.endTransmission();
  //Escala de sensibilidade do giroscópio
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);//Endereço do registrador para sensibilidade do giroscópio
  Wire.write(0x8);
  Wire.endTransmission();
  //Acesso aos registradores de medidas do giroscopio
  Wire.beginTransmission(0x68);
  Wire.write(0x43);//Endereço do registrador de medidas
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);//"Puxando informação dos seis registradores disponpiveis na MPU6050"
  //-----------Término da configuração do giroscópio---------------//

    //-----------------Medidas do girocópio--------------------------// 
  int16_t gyroXraw = Wire.read() << 8 | Wire.read();
  int16_t gyroYraw = Wire.read() << 8 | Wire.read();
  int16_t gyroZraw = Wire.read() << 8 | Wire.read();

  //Velocidade
  gyroX = (float)gyroXraw/65.5; //Conversão de medidas para graus/segundo
  gyroY = (float)gyroYraw/65.5;
  gyroZ= (float)gyroZraw/65.5;

  if (calibrated) {
    gyroX -= gyroXOffset;
    gyroY -= gyroYOffset;
    gyroZ -= gyroZOffset;
  }

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime)/1000.0;

  GyroPitch += gyroY*dt;

  AngleYaw += gyroZ*dt;

  lastTime = currentTime;
  //---------------------------------------------------------------//

    //----------------Configuração do Acelerômetro-------------------//
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);//Registrador do Acelerometro
  Wire.write(0x10);
  Wire.endTransmission();
  //Acesso aos registradores de medida do Acelerometro
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  //------------Término da configuração do Acelerômetro------------//

    //-----------------Medidas do Acelerômetro----------------------//
  int16_t AccXLSB = Wire.read()<<8 | 
    Wire.read();
  int16_t AccYLSB = Wire.read()<<8 | 
    Wire.read();
  int16_t AccZLSB = Wire.read()<<8 | 
    Wire.read();

  AccX = (float)AccXLSB/4096 - 4.27 + 0.32;
  AccY = (float)AccYLSB/4096;
  AccZ = (float)AccZLSB/4096 + 0.06;

  //AngleRoll = atan(AccY/sqrt(AccX*AccX + AccZ*AccZ))*rad_to_deg; //Dividir o resultado em rad por (3.142/180) para obter a resposta em graus
  AnglePitch = -atan(AccX/sqrt(AccY*AccY + AccZ*AccZ))*rad_to_deg;
  //---------------------------------------------------------------//
}  

void calibrationGyro() {
  gyroXOffset = 0;
  gyroYOffset = 0;
  gyroZOffset = 0;

  for (int i = 0; i < 2000; i++) {
    setupMPU6050();
    gyroXOffset += gyroX;
    gyroYOffset += gyroY;
    gyroZOffset += gyroZ;
    delay(1);  // S
  }
  
  gyroXOffset /= 2000;
  gyroYOffset /= 2000;
  gyroZOffset /= 2000;
  
  calibrated = true;
}

// void medidasAcelerometro(){
//   //----------------Configuração do Acelerômetro-------------------//
//   Wire.beginTransmission(0x68);//Inicia comunicação I2C com MPU6050
//   //Filtro Passa-baixas Digital
//   Wire.write(0x1A);//Endereço de registrador
//   Wire.write(0x05);//Largura de banda de 10Hz
//   Wire.endTransmission();

//   Wire.beginTransmission(0x68);
//   Wire.write(0x1C);//Registrador do Acelerometro
//   Wire.write(0x10);
//   Wire.endTransmission();
//   //Acesso aos registradores de medida do Acelerometro
//   Wire.beginTransmission(0x68);
//   Wire.write(0x3B);
//   Wire.endTransmission();
//   Wire.requestFrom(0x68,6);
//   //------------Término da configuração do Acelerômetro------------//

// }

