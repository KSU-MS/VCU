#pragma once
#include <stdint.h>

// We want to catch when there isn't a ADC_CS pin defined globally
#ifndef ADC_CS
uint8_t ADC_CS = 10; // Typical default SPI CS pin
#endif

#ifndef DEFAULT_SPI_SPEED
#define DEFAULT_SPI_SPEED 1000000
#endif

#ifdef ARDUINO
#include <Arduino.h>
#include <SPI.h>

// Helper functions for the pointer
uint16_t avr_helper(uint8_t pin) { return (uint16_t)analogRead(pin); }

uint16_t mcp_helper(uint8_t pin) {
  // Gain control of the SPI port
  // and configure settings
  SPI.beginTransaction(SPISettings(DEFAULT_SPI_SPEED, MSBFIRST, SPI_MODE0));

  // Take the SS pin low to select the chip:
  digitalWrite(ADC_CS, LOW);

  // Set up channel
  byte b = B01100000;
  b |= ((pin << 2));

  // Send in the channel via SPI:
  SPI.transfer(b);

  // Read data from SPI
  byte result1 = SPI.transfer(0);
  byte result2 = SPI.transfer(0);

  // Take the SS pin high to de-select the chip:
  digitalWrite(ADC_CS, HIGH);

  // Release control of the SPI port
  SPI.endTransaction();

  return (result1 << 4) | (result2 >> 4);
}

void init_mcp() {
  pinMode(ADC_CS, OUTPUT);
  digitalWrite(ADC_CS, HIGH);

  SPI.begin();
}

void init_mcp(uint8_t cs_pin) {
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);

  SPI.begin();
}
#endif

enum adc_method {
  avr,
  mcp,
};

class adc {
private:
  uint8_t pin;
  uint16_t (*reader)(uint8_t);
  uint16_t rawval;
  double alpha;
  bool is_filtered = false;

  void set_helpers(adc_method guy) {
    switch (guy) {
#ifdef ARDUINO
    case avr:
      reader = &avr_helper;
      break;

    case mcp:
      init_mcp();
      reader = &mcp_helper;
      break;
#endif // ARDUINO

    default:
      break;
    }
  };

public:
  union {
    uint16_t in;
    uint8_t b[2];
  } value;

  adc(adc_method guy, uint8_t target_pin) {
    pin = target_pin;
    set_helpers(guy);
  }

  // Added an overload for when the MCP is on another CS pin
  adc(adc_method guy, uint8_t cs_pin, uint8_t target_pin) {
    pin = target_pin;
    ADC_CS = cs_pin;
    set_helpers(guy);
  }

  adc(adc_method guy, uint8_t target_pin, double alpha) {
    pin = target_pin;
    this->alpha = alpha;
    is_filtered = true;
    set_helpers(guy);
  }

  adc(adc_method guy, uint8_t cs_pin, uint8_t target_pin, double alpha) {
    pin = target_pin;
    ADC_CS = cs_pin;
    this->alpha = alpha;
    is_filtered = true;
    set_helpers(guy);
  }

  void update() {
    // Vars to hold some data
    if (is_filtered) {
      rawval = reader(pin);
      value.in = alpha * value.in + (1 - alpha) * rawval;
    } else {
      value.in = reader(pin);
    }
  }
};
