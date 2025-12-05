#include "Config.hpp"
#include "Motor.hpp"
#include "Encoder.hpp"
#include "Control.hpp"
#include "Sensor.hpp"
#include "Telemetry.hpp"

void setup() {
  Serial.begin(115200);
  // init drivers, motors, encoders, controller, telemetry
}

void loop() {
  static unsigned long lastControl = 0;
  unsigned long now = millis();
  if (now - lastControl >= cfg::CONTROL_PERIOD_MS) {
    lastControl = now;
    // 1) lire capteurs
    // 2) lire encoders -> convertir en vitesse
    // 3) decide target via Strategy
    // 4) controller.update(...)
    // 5) send PWM via Motor::setDuty
    // 6) telemetry.publishStatus(...)
  }
  // other lower-priority tasks (non-blocking)
}
