#pragma once
#include <Arduino.h>

struct PidGains {
  float kp;
  float ki;
  float kd;
};

class PidController {
 public:
  explicit PidController(const PidGains &gains, float outMin = -400.0f, float outMax = 400.0f)
      : gains_(gains), outMin_(outMin), outMax_(outMax) {}

  void setGains(const PidGains &gains) { gains_ = gains; }

  float compute(float target, float measurement, float dt) {
    float error = target - measurement;
    integral_ += error * dt;
    float derivative = (dt > 0) ? (error - lastError_) / dt : 0.0f;
    lastError_ = error;

    float output = (gains_.kp * error) + (gains_.ki * integral_) + (gains_.kd * derivative);
    if (output > outMax_) output = outMax_;
    if (output < outMin_) output = outMin_;
    return output;
  }

  void reset() {
    integral_ = 0;
    lastError_ = 0;
  }

 private:
  PidGains gains_{};
  float integral_ = 0.0f;
  float lastError_ = 0.0f;
  float outMin_;
  float outMax_;
};

