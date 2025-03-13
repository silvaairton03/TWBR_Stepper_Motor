#ifndef ENCODERAS5600_H
#define ENCODERAS5600_H

#include <Arduino.h>
#include <AS5600.h>
#include<Wire.h>

class EncoderAS5600
{
private:
    float rpmAS5600 = 0.0;

public:
    EncoderAS5600(/*args*/);
    ~EncoderAS5600();

    void setEncoder_AS5600(AS5600 &obj, int SDA_pin, int SCL_pin, int direction_pin, TwoWire &I2C_obj);
    float getRPM_AS5600(AS5600 &obj);
};
#endif // ENCODERAS5600_H