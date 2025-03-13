#include "AS5600Sensor.h"

AS5600Sensor::AS5600Sensor(TwoWire *wirePort, uint8_t addr)
  : _i2c(wirePort),
    _address(addr),
    _startAngle(0), _numberOfTurns(0), _previousQuadrantNumber(0),
    _totalAngle(0), _recentTotalAngle(0), _rpmValue(0),
    _magEncoderRadSpeed(0), _stepRateFromRPM(0),
    _taxaDeAmostragem(20), _lastTime(0),
    _lastMeasurementTime(0), _lastMeasurementAngle(0)
{
}

void AS5600Sensor::begin() {
  // Ensure the magnet is in range before continuing.
  checkMagnetPresence();
  // Take an initial raw angle reading.
  readRawAngle();
  // Set the start (zero) reference to the current angle.
  calibrate();
  // Optionally, print a confirmation message:
  Serial.print("AS5600 sensor at 0x");
  Serial.print(_address, HEX);
  Serial.println(" tared: total angle set to 0.0");
}


void AS5600Sensor::readRawAngle() {
  // Read low byte from register 0x0D.
  _i2c->beginTransmission(_address);
  _i2c->write(0x0D);
  _i2c->endTransmission();
  _i2c->requestFrom(_address, (uint8_t)1);
  while (_i2c->available() == 0);
  _lowbyte = _i2c->read();

  // Read high byte from register 0x0C.
  _i2c->beginTransmission(_address);
  _i2c->write(0x0C);
  _i2c->endTransmission();
  _i2c->requestFrom(_address, (uint8_t)1);
  while (_i2c->available() == 0);
  _highbyte = _i2c->read();

  // Combine the two bytes into a 12-bit value.
  _highbyte = _highbyte << 8;
  _rawAngle = _highbyte | _lowbyte;
  
  // Convert raw value to degrees (360° / 4096 steps).
  _degAngle = _rawAngle * 0.087890625;
}

// void AS5600Sensor::correctAngle()
// {
//   //recalculate angle
//   _correctedAngle = _degAngle - _startAngle; // this tares the position

//   if (_correctedAngle < 0) // if the calculated angle is negative, we need to "normalize" it
//   {
//     _correctedAngle = _correctedAngle + 360; // correction for negative numbers (i.e. -15 becomes +345)
//   }
// //  Serial.print("Corrected angle: ");
// //  Serial.println(correctedAngle, 2); //print the corrected/tared angle
// }

void AS5600Sensor::calibrate(uint16_t numSamples, uint16_t delayMs) {
  float sum = 0;
  for (uint16_t i = 0; i < numSamples; i++) {
    readRawAngle();
    sum += _degAngle;
    delay(delayMs); // short delay between samples
  }
  // Set the average as the tare.
  _startAngle = sum / numSamples;
  
  // Reset all integration counters.
  _numberOfTurns = 0;
  _totalAngle = 0.0;
  _recentTotalAngle = 0.0;
  _lastMeasurementAngle = 0.0;
  _lastMeasurementTime = millis();
  
  Serial.print("AS5600 sensor at 0x");
  Serial.print(_address, HEX);
  Serial.print(" calibrated: _startAngle = ");
  Serial.println(_startAngle, 2);
}


void AS5600Sensor::correctAngle() {
  float diff = _degAngle - _startAngle;
  // If the difference is very small (within 1 degree), force it to 0.
  if (fabs(diff) < 2.0) {
    _correctedAngle = 0;
  }
  else if (diff < 0) {
    _correctedAngle = diff + 360;
  }
  else {
    _correctedAngle = diff;
  }
  // Optionally, print _correctedAngle for debugging:
  // Serial.print("Corrected angle: ");
  // Serial.println(_correctedAngle, 2);
}



void AS5600Sensor::checkQuadrant() {
  // Determine the quadrant based on the corrected angle.
  if (_correctedAngle >= 0 && _correctedAngle <= 90)
    _quadrantNumber = 1;
  else if (_correctedAngle > 90 && _correctedAngle <= 180)
    _quadrantNumber = 2;
  else if (_correctedAngle > 180 && _correctedAngle <= 270)
    _quadrantNumber = 3;
  else if (_correctedAngle > 270 && _correctedAngle < 360)
    _quadrantNumber = 4;

  // If there is a change in quadrant, adjust the turn count.
  if (_quadrantNumber != _previousQuadrantNumber) {
    if (_quadrantNumber == 1 && _previousQuadrantNumber == 4)
      _numberOfTurns++;  // Clockwise rotation.
    else if (_quadrantNumber == 4 && _previousQuadrantNumber == 1)
      _numberOfTurns--;  // Counterclockwise rotation.
    _previousQuadrantNumber = _quadrantNumber;
  }
  // Compute the total (multi-turn) angle.
  _totalAngle = (_numberOfTurns * 360) + _correctedAngle;
}

void AS5600Sensor::checkMagnetPresence() {
  int magnetStatus = 0;
  unsigned long printMillis = millis();
  // Poll the status register (0x0B) until the MD bit (value 32) is set.
  while ((magnetStatus & 32) != 32) {
    magnetStatus = 0;
    _i2c->beginTransmission(_address);
    _i2c->write(0x0B);
    _i2c->endTransmission();
    _i2c->requestFrom(_address, (uint8_t)1);
    while (_i2c->available() == 0);
    magnetStatus = _i2c->read();

    if (millis() - printMillis >= 100) {
      Serial.print("Magnet status: ");
      Serial.println(magnetStatus, BIN);
      printMillis = millis();
    }
  }
  Serial.print("AS5600 sensor at 0x");
  Serial.print(_address, HEX);
  Serial.println(" - Magnet found, sensor ready!");
}

void AS5600Sensor::calculateRPM_noTime() {
  unsigned long currentTime = millis();
  
  // Check if the fixed interval has passed
  if (currentTime - _lastMeasurementTime >= _measurementInterval) {
    unsigned long dt = currentTime - _lastMeasurementTime; // This is your measurement window in ms.
    float deltaAngle = _totalAngle - _lastMeasurementAngle;   // Change in total angle over dt
    
    // Compute RPM: (60000 / dt) converts ms to minutes; then multiply by fraction of a revolution.
    _rpmValue = (60000.0 / dt) * (deltaAngle / 360.0);
    
    // Compute angular speed in rad/s (using a fixed _taxaDeAmostragem if desired, or dt)
    float newAngularSpeed = (1000.0 / dt) * (deltaAngle / 360.0) * 2 * PI; 
    const float alpha = 0.3;
    _magEncoderRadSpeed = alpha * newAngularSpeed + (1.0 - alpha) * _magEncoderRadSpeed;

    // _magEncoderRadSpeed = (1000.0 / _taxaDeAmostragem) * (deltaAngle / 360.0) * 2 * PI;
    
    // Convert RPM to step rate (steps per second); STEPS_CONSTANT is defined elsewhere.
    _stepRateFromRPM = _rpmValue * STEPS_CONSTANT / 60.0;
    
    // Update the measurement window start values:
    _lastMeasurementTime = currentTime;
    _lastMeasurementAngle = _totalAngle;
  }
  // Otherwise, do nothing—keep the last computed RPM value.
}


float AS5600Sensor::getLeftWheelAngle() const {
  // For the left wheel, we assume the sensor is mounted in the normal orientation.
  return - _totalAngle;
}

float AS5600Sensor::getRightWheelAngle() const {
  // For the right wheel (mirrored), return the mirror of the corrected angle.
  // For example, if the sensor reads 10°, the mirrored value becomes 360 - 10 = 350°.
  return _totalAngle;
}


float AS5600Sensor::getRPM() const {
  return _rpmValue;
}

float AS5600Sensor::getRadSpeedLeft() const {
  return -_magEncoderRadSpeed;
}

float AS5600Sensor::getRadSpeedRight() const {
  return _magEncoderRadSpeed;
}

float AS5600Sensor::getStepRateFromRPM() const {
  return _stepRateFromRPM;
}


float AS5600Sensor::yawAS5600() const
{
  const float wheelRadius = 0.0325;
  const float lenght = 0.210416;

  float angularSpeedLeftWheel = getRadSpeedLeft();
  float angularSpeedRightWheel = getRadSpeedRight();

  float yawMagEncoder = (wheelRadius/lenght) * (angularSpeedRightWheel - angularSpeedLeftWheel);

  return yawMagEncoder;
}