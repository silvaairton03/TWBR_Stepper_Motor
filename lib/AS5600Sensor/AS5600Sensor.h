#ifndef AS5600SENSOR_H
#define AS5600SENSOR_H

#include <Arduino.h>
#include <Wire.h>

// You may define your STEPS_CONSTANT here if needed (for example, 3200 steps per revolution)
#ifndef STEPS_CONSTANT
  #define STEPS_CONSTANT 6400
#endif

class AS5600Sensor {
  public:
    // Constructor accepts a pointer to a TwoWire object and a sensor address (default is 0x36)
    AS5600Sensor(TwoWire *wirePort, uint8_t addr = 0x36);
    
    // Initialize the sensor: check magnet presence and take an initial reading to zero the sensor.
    void begin();

    // Read the raw angle from the sensor over I²C
    void readRawAngle();
    
    // Subtract the initial angle to obtain a "tared" reading
    void correctAngle();
    
    void calibrate(uint16_t numSamples = 100, uint16_t delayMs = 5);
    // Determine in which quadrant the tared angle lies and update full-turn count
    void checkQuadrant();
    
    // Calculate RPM, angular speed (rad/s) and step rate based on the change in the sensor's absolute angle.
    void calculateRPM_noTime();
    
    float yawAS5600() const;
    // Getters for the computed values:
    float getLeftWheelAngle() const;
    float getRightWheelAngle() const;
    // Returns the full (multi-turn) angle in degrees.
    float getRPM() const;               // Returns the calculated RPM.
    float getRadSpeedLeft() const;
    float getRadSpeedRight() const; // Returns the angular speed in radians per second.
    float getStepRateFromRPM() const;    // Returns the motor step rate (steps per second).
  
  private:
    TwoWire *_i2c;         // Pointer to the I2C bus instance used by this sensor.
    uint8_t _address;      // Sensor I²C address.
    unsigned long _lastMeasurementTime;   // time when the last measurement window started
    float _lastMeasurementAngle;          // total angle at the start of the measurement window
    const unsigned long _measurementInterval = 200;
    // Sensor raw and processed values:
    int _lowbyte, _highbyte, _rawAngle;
    float _degAngle;       // Angle in degrees directly from the sensor.
    float _startAngle;     // Initial angle used to zero the sensor.
    float _correctedAngle; // Tared angle after subtracting _startAngle.
    int _quadrantNumber, _previousQuadrantNumber;
    float _numberOfTurns;  // Number of complete revolutions (can be negative).
    float _totalAngle;     // Absolute angle (turns * 360 + correctedAngle).
    
    // Variables for RPM/speed calculations:
    float _rpmValue;
    float _magEncoderRadSpeed;  // Angular speed (rad/s) calculated from the sensor.
    float _stepRateFromRPM;     // Converted step rate in steps/s.
    float _recentTotalAngle;    // Last total angle used for RPM calculation.
    unsigned long _lastTime;    // Time stamp (in ms) of the last calculation.
    int _taxaDeAmostragem;      // Sampling interval (in ms), set here to 20.
    
    // Helper function to check that the magnet is correctly positioned.
    void checkMagnetPresence();
};

#endif
