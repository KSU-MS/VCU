#pragma once
#include "core_pins.h"
#include <stdint.h>

// We want to catch when there isn't a CS_PIN pin defined globally
#ifndef CS_PIN
uint8_t CS_PIN = 10; // Typical default SPI CS pin
#endif

#ifndef DEFAULT_SPI_SPEED
#define DEFAULT_SPI_SPEED 1000000
#endif

// Configuration Register Base Values (Hex)
// Breakdown of bits:
// 15    : OS = 1 (Start Single-Shot Conversion)
// 14-12 : MUX = 100/101/110/111 (Single-Ended AIN0 to AIN3)
// 11-9  : PGA = 000 (+/- 6.144V, No amplification)
// 8     : MODE = 1 (Single-Shot Mode)
// 7-5   : DR = 111 (3300 Samples Per Second)
// 4     : TS_MODE = 0 (ADC Mode)
// 3     : PULL_UP_EN = 0 (Internal Pull-Up Disabled)
// 2-1   : NOP = 01 (Valid Configuration Write)
// 0     : RESERVED = 0
const uint16_t CONFIG_AIN0 = 0xC1E2; // 1100 0001 1110 0010
const uint16_t CONFIG_AIN1 = 0xD1E2; // 1101 0001 1110 0010
const uint16_t CONFIG_AIN2 = 0xE1E2; // 1110 0001 1110 0010
const uint16_t CONFIG_AIN3 = 0xF1E2; // 1111 0001 1110 0010

#ifdef ARDUINO
#include "ads1018.h"
#include <Arduino.h>
#include <SPI.h>

// ADS1018 SPI Configuration Settings
// Max SPI clock for ADS1018 is 4MHz. Mode 1 (CPOL=0, CPHA=1)
SPISettings ads1018Settings(DEFAULT_SPI_SPEED, MSBFIRST, SPI_MODE1);

// Helper functions for the pointer
uint16_t avr_helper(uint8_t pin) { return (uint16_t)analogRead(pin); }

uint16_t mcp_helper(uint8_t pin) {
  // Gain control of the SPI port
  // and configure settings
  SPI.beginTransaction(SPISettings(DEFAULT_SPI_SPEED, MSBFIRST, SPI_MODE0));

  // Take the SS pin low to select the chip:
  digitalWrite(CS_PIN, LOW);

  // Set up channel
  byte b = B01100000;
  b |= ((pin << 2));

  // Send in the channel via SPI:
  SPI.transfer(b);

  // Read data from SPI
  byte result1 = SPI.transfer(0);
  byte result2 = SPI.transfer(0);

  // Take the SS pin high to de-select the chip:
  digitalWrite(CS_PIN, HIGH);

  // Release control of the SPI port
  SPI.endTransaction();

  return (result1 << 4) | (result2 >> 4);
}

uint16_t ads_helper(uint8_t pin) {

  uint16_t configRegister = 0;

  // Select the appropriate configuration word based on the channel
  switch (pin) {
  case 0:
    configRegister = CONFIG_AIN0;
    break;
  case 1:
    configRegister = CONFIG_AIN1;
    break;
  case 2:
    configRegister = CONFIG_AIN2;
    break;
  case 3:
    configRegister = CONFIG_AIN3;
    break;
  default:
    return 0.0;
  }

  SPI.beginTransaction(ads1018Settings);

  // --- STEP 1: Send Configuration to Start Conversion ---
  digitalWrite(CS_PIN, LOW);
  SPI.transfer16(configRegister);
  digitalWrite(CS_PIN, HIGH);

  // --- STEP 2: Wait for the conversion to finish ---
  // At 3.3 ksps, a conversion takes ~303 microseconds.
  // We wait 350us to guarantee completion.
  delayMicroseconds(500);

  // --- STEP 3: Read the Conversion Result ---
  digitalWrite(CS_PIN, LOW);
  // We send the config again but clear the MSB (OS bit) to 0 so we don't
  // start a redundant conversion
  uint16_t rawData = SPI.transfer16(configRegister & 0x7FFF);
  digitalWrite(CS_PIN, HIGH);

  SPI.endTransaction();

  // --- STEP 4: Data Processing ---
  // The ADS1018 provides 12-bit data left-justified within the 16-bit word.
  // We cast to signed 16-bit to handle Two's Complement correctly, then
  // arithmetic shift right by 4 bits.
  int16_t signedData = (int16_t)rawData;
  signedData = signedData >> 4;

  // Clamp any negative noise/offset values to 0
  if (signedData < 0) {
    signedData = 0;
  }

  return signedData;
}

void init_mcp() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();
}

void init_mcp(uint8_t cs_pin) {
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);

  SPI.begin();
}

void init_ads() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // Set CS high initially (inactive state)

  SPI.begin();
}
#endif

enum adc_method {
  avr,
  mcp,
  ads,
  ads_tech,
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

    case ads:
      init_ads();
      reader = &ads_helper;
      break;

    case ads_tech:
      reader = &ads_helper;
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
    CS_PIN = cs_pin;
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
    cs_pin = cs_pin;
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
