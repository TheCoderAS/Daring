# ESP32-S3 + MPU6050 Quad Hover Controller

This sketch provides a dual-core ESP32-S3 flight controller that exposes a Wi-Fi hotspot with a simple throttle-only web UI (core 1) while core 0 runs the attitude loop, reads the MPU6050, and drives four BLDC ESCs.

## Hardware assumptions
- ESP32-S3 DevKit
- MPU6050 IMU on I2C
- 4 × 2200 kV motors with 6×4.5 props and 4S LiPo (approx. 250 g X-frame)
- ESCs driven by LEDC PWM (default pins are placeholders, update `kMotorPins`)

## Features
- SoftAP named `QuadHover-AP` (password `hover123`) serving a vertical throttle slider (0–100%).
- Flight loop pinned to core 0 at ~250 Hz using complementary-filter attitude estimation and PID stabilization. Raw IMU values are used so the craft always drives toward true zero attitude instead of a calibrated offset.
- Initial hover-oriented PID gains: pitch/roll `Kp=3.8`, `Ki=0.02`, `Kd=18.0`; yaw `Kp=1.0`, `Ki=0.01`, `Kd=4.0`.
- ESC arming pulse, optional throttle-range calibration helper, idle spin to maintain stability when throttle is near zero.

## File layout
- `src/main.ino` — task setup, web server, control loop, motor mixing.
- `src/Imu.h` — lightweight MPU6050 driver with complementary filter.
- `src/Pid.h` — reusable PID controller implementation.
- `src/MotorMixer.h` — LEDC PWM setup, ESC arming, motor writes.

## Usage
1. Adjust motor output pins in `kMotorPins` inside `src/main.ino` to match your wiring.
2. (Optional) To calibrate ESC throttle range on first power-up, set `kCalibrateEscsOnBoot` to `true`, remove props, and power the quad so the max-then-min pulse sequence can complete. Return it to `false` for normal use.
3. Flash the sketch with the Arduino ESP32 core installed.
4. Power the quad, connect to the `QuadHover-AP` Wi-Fi network, open `http://192.168.4.1/`, and move the slider to raise throttle.
5. Start low; tune the PID gains as needed to match your specific frame and prop/motor combination.

## Safety
This code is an initial hover-focused template. Test with props removed first, secure the airframe, and confirm correct motor direction/mixing before flight. Tune PIDs gradually to avoid oscillations.
