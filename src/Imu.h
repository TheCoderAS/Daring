#pragma once
#include <Arduino.h>
#include <Wire.h>

class Imu6050 {
 public:
  bool begin(uint8_t address = 0x68) {
    address_ = address;
    Wire.begin();
    Wire.beginTransmission(address_);
    Wire.write(0x6B);  // power management
    Wire.write(0);
    if (Wire.endTransmission() != 0) {
      return false;
    }
    lastUpdateMicros_ = micros();
    return true;
  }

  void update() {
    readRaw();
    const float ax = rawAx_ / 16384.0f;
    const float ay = rawAy_ / 16384.0f;
    const float az = rawAz_ / 16384.0f;
    const float gx = rawGx_ / 131.0f;  // dps
    const float gy = rawGy_ / 131.0f;
    const float gz = rawGz_ / 131.0f;

    unsigned long now = micros();
    const float dt = (now - lastUpdateMicros_) / 1e6f;
    lastUpdateMicros_ = now;

    // Complementary filter
    const float accRoll = atan2f(ay, az) * 57.2958f;
    const float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;

    roll_ = 0.98f * (roll_ + gx * dt) + 0.02f * accRoll;
    pitch_ = 0.98f * (pitch_ + gy * dt) + 0.02f * accPitch;
    yaw_ += gz * dt;
  }

  float pitch() const { return pitch_; }
  float roll() const { return roll_; }
  float yaw() const { return yaw_; }

 private:
  void readRaw() {
    Wire.beginTransmission(address_);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(address_, static_cast<uint8_t>(14));

    rawAx_ = Wire.read() << 8 | Wire.read();
    rawAy_ = Wire.read() << 8 | Wire.read();
    rawAz_ = Wire.read() << 8 | Wire.read();
    rawTemp_ = Wire.read() << 8 | Wire.read();
    rawGx_ = Wire.read() << 8 | Wire.read();
    rawGy_ = Wire.read() << 8 | Wire.read();
    rawGz_ = Wire.read() << 8 | Wire.read();
  }

  uint8_t address_ = 0x68;
  int16_t rawAx_ = 0, rawAy_ = 0, rawAz_ = 0, rawGx_ = 0, rawGy_ = 0, rawGz_ = 0, rawTemp_ = 0;
  float pitch_ = 0, roll_ = 0, yaw_ = 0;
  unsigned long lastUpdateMicros_ = 0;
};

