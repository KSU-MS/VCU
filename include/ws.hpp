#ifndef WS_HPP
#define WS_HPP

#include <Arduino.h>
#include <stdint.h>

class wheel_speed {
private:
  uint64_t *pulseCount;
  uint32_t lastMillis = 0;
  uint16_t rpm = 0;

  static constexpr uint16_t pulsesPerRevolution = 18;
  static constexpr uint32_t intervalMs = 20;

public:
  void init(uint8_t pulsePin, void (*isr)(void), uint64_t *pulseCounter) {
    pinMode(pulsePin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pulsePin), isr, RISING);

    pulseCount = pulseCounter;
    lastMillis = millis();
  }

  void update_rpms(uint32_t currentMillis) {
    if (currentMillis - lastMillis >= intervalMs) {
      uint16_t pulses;

      // Atomic read & reset
      noInterrupts();
      pulses = *pulseCount;
      *pulseCount = 0;
      interrupts();

      // RPM calculation:
      // RPM = (pulses * 60000) / (intervalMs * pulsesPerRevolution)
      rpm = (pulses * 60000UL) / (intervalMs * pulsesPerRevolution);

      lastMillis = currentMillis;
    }
  }

  uint16_t get_rpms(void) { return rpm; }
};

#endif
