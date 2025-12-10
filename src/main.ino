#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

#include "Imu.h"
#include "MotorMixer.h"
#include "Pid.h"

// Replace with your motor control pins
constexpr MotorPins kMotorPins{.frontLeft = 2, .frontRight = 1, .rearLeft = 42, .rearRight = 41};

// PID gains tuned for initial hover with 6x4.5 props, 2200kV motors on 4S
constexpr PidGains kPitchRollGains{.kp = 3.8f, .ki = 0.02f, .kd = 18.0f};
constexpr PidGains kYawGains{.kp = 1.0f, .ki = 0.01f, .kd = 4.0f};

// Optional: set true for an ESC throttle-range calibration pulse (props off, supervised)
constexpr bool kCalibrateEscsOnBoot = false;

// Network configuration
const char *kApSsid = "QuadHover-AP";
const char *kApPassword = "hover123";  // keep at least 8 chars for WPA2

// Globals shared between tasks
volatile float targetThrottlePercent = 0.0f;
portMUX_TYPE throttleMux = portMUX_INITIALIZER_UNLOCKED;

WebServer server(80);
Imu6050 imu;
MotorMixer motors(kMotorPins);
PidController pitchPid(kPitchRollGains);
PidController rollPid(kPitchRollGains);
PidController yawPid(kYawGains, -200.0f, 200.0f);

TaskHandle_t webTaskHandle = nullptr;
TaskHandle_t controlTaskHandle = nullptr;

String sliderPage() {
  return R"(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<style>
body { font-family: Arial, sans-serif; margin: 30px; }
.slider { width: 100%; height: 300px; writing-mode: bt-lr; -webkit-appearance: slider-vertical; }
label { font-size: 18px; }
</style>
</head>
<body>
<h2>Quad Hover Throttle</h2>
<label for="throttle">Throttle: <span id="val">0</span>%</label><br>
<input type="range" min="0" max="100" value="0" class="slider" id="throttle" orient="vertical">
<script>
const slider = document.getElementById('throttle');
const val = document.getElementById('val');
slider.oninput = function() {
  val.innerText = this.value;
  fetch('/set?throttle=' + this.value).catch(() => {});
}
</script>
</body>
</html>
  )";
}

void handleRoot() { server.send(200, "text/html", sliderPage()); }

void handleSet() {
  if (server.hasArg("throttle")) {
    float pct = server.arg("throttle").toFloat();
    pct = constrain(pct, 0.0f, 100.0f);
    portENTER_CRITICAL(&throttleMux);
    targetThrottlePercent = pct;
    portEXIT_CRITICAL(&throttleMux);
    server.send(200, "text/plain", String("Throttle set to ") + pct);
  } else {
    server.send(400, "text/plain", "Missing throttle param");
  }
}

void startWebServer() {
  WiFi.softAP(kApSsid, kApPassword);
  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.begin();
}

void webTask(void *param) {
  startWebServer();
  for (;;) {
    server.handleClient();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

float readThrottle() {
  portENTER_CRITICAL(&throttleMux);
  float pct = targetThrottlePercent;
  portEXIT_CRITICAL(&throttleMux);
  return pct;
}

void controlTask(void *param) {
  if (!imu.begin()) {
    Serial.println("Failed to init MPU6050");
    vTaskDelete(nullptr);
  }
  motors.begin();
  if (kCalibrateEscsOnBoot) {
    Serial.println("ESC calibration: sending max then min throttle. Props off!");
    motors.calibrateEscs();
  }
  motors.armEscs();

  unsigned long lastLoopMicros = micros();
  const float hoverDuty = motors.maxDuty() * 0.4f;  // ~40% base for hover starting point

  for (;;) {
    imu.update();
    unsigned long now = micros();
    float dt = (now - lastLoopMicros) / 1e6f;
    lastLoopMicros = now;

    float desiredThrottle = readThrottle();
    float throttleDuty = (desiredThrottle / 100.0f) * motors.maxDuty();

    float rollCorrection = rollPid.compute(0.0f, imu.roll(), dt);
    float pitchCorrection = pitchPid.compute(0.0f, imu.pitch(), dt);
    // Yaw hold disabled for now
    float yawCorrection = 0.0f;

    // X quad mixing
    float mFrontLeft = throttleDuty + pitchCorrection - rollCorrection - yawCorrection;
    float mFrontRight = throttleDuty + pitchCorrection + rollCorrection + yawCorrection;
    float mRearLeft = throttleDuty - pitchCorrection - rollCorrection + yawCorrection;
    float mRearRight = throttleDuty - pitchCorrection + rollCorrection - yawCorrection;

    // keep motors spinning lightly even at 0 throttle to improve stability
    const float idle = hoverDuty * 0.2f;
    motors.write(max(mFrontLeft, idle), max(mFrontRight, idle), max(mRearLeft, idle), max(mRearRight, idle));

    static unsigned long lastLogMs = 0;
    unsigned long nowMs = millis();
    if (nowMs - lastLogMs >= 50) {
      lastLogMs = nowMs;
      Serial.printf("thr=%.1f%% roll=%.2f pitch=%.2f yaw=%.2f\n", desiredThrottle, imu.roll(), imu.pitch(), imu.yaw());
    }

    vTaskDelay(4 / portTICK_PERIOD_MS);  // ~250 Hz loop
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  xTaskCreatePinnedToCore(webTask, "WebTask", 8192, nullptr, 1, &webTaskHandle, 1);
  xTaskCreatePinnedToCore(controlTask, "ControlTask", 8192, nullptr, 2, &controlTaskHandle, 0);
}

void loop() {
  // Not used. Work is split across two cores.
}

