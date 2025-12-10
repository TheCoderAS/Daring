#pragma once
#include <Arduino.h>

struct MotorPins {
  uint8_t frontLeft;
  uint8_t frontRight;
  uint8_t rearLeft;
  uint8_t rearRight;
};

class MotorMixer {
 public:
  MotorMixer(const MotorPins &pins, uint16_t pwmFreq = 400, uint8_t resolution = 11)
      : pins_(pins), pwmFreq_(pwmFreq), resolution_(resolution) {}

  void begin() {
    ledcSetup(0, pwmFreq_, resolution_);
    ledcSetup(1, pwmFreq_, resolution_);
    ledcSetup(2, pwmFreq_, resolution_);
    ledcSetup(3, pwmFreq_, resolution_);
    ledcAttachPin(pins_.frontLeft, 0);
    ledcAttachPin(pins_.frontRight, 1);
    ledcAttachPin(pins_.rearLeft, 2);
    ledcAttachPin(pins_.rearRight, 3);
  }

  void write(float frontLeft, float frontRight, float rearLeft, float rearRight) {
    ledcWrite(0, constrain(frontLeft, 0.0f, maxDuty()));
    ledcWrite(1, constrain(frontRight, 0.0f, maxDuty()));
    ledcWrite(2, constrain(rearLeft, 0.0f, maxDuty()));
    ledcWrite(3, constrain(rearRight, 0.0f, maxDuty()));
  }

  void armEscs(uint16_t pulses = 200) { writeSameDuty(maxDuty() * 0.05f, pulses, 5); }

  void calibrateEscs(uint16_t highMs = 2000, uint16_t lowMs = 2000) {
    // Calibrates throttle range by sending max then min duty. Only run with props off.
    writeSameDuty(maxDuty(), highMs / 5, 5);
    writeSameDuty(maxDuty() * 0.05f, lowMs / 5, 5);
  }

  uint32_t maxDuty() const { return (1 << resolution_) - 1; }

 private:
  void writeSameDuty(float duty, uint16_t pulses, uint16_t delayMs) {
    for (uint16_t i = 0; i < pulses; ++i) {
      ledcWrite(0, duty);
      ledcWrite(1, duty);
      ledcWrite(2, duty);
      ledcWrite(3, duty);
      delay(delayMs);
    }
  }

  MotorPins pins_{};
  uint16_t pwmFreq_;
  uint8_t resolution_;
};

