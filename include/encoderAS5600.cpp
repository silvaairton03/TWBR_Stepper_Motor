#include "encoderAS5600.h"

EncoderAS5600::EncoderAS5600(){}

EncoderAS5600::~EncoderAS5600(){}

void EncoderAS5600::setEncoder_AS5600(AS5600 &obj, int SDA_pin, int SCL_pin, int direction_pin, TwoWire &I2C_obj)
{
    I2C_obj.begin(SCL_pin, SDA_pin);
    obj.begin(direction_pin);  //  set direction pin.
    obj.setDirection(AS5600_COUNTERCLOCK_WISE);  // default, just be explicit.

    int b = obj.isConnected();
    Serial.print("Connect: ");
    Serial.println(b);
}
float EncoderAS5600::getRPM_AS5600(AS5600 &obj)
{
    this->rpmAS5600 = obj.getAngularSpeed(AS5600_MODE_RPM);
    return this->rpmAS5600;
};
