#pragma once
#ifndef logger_hpp
#define logger_hpp

#include <Arduino.h>

enum logger_type {
  serial,
  teensy_microsd_numbered_log,
  teensy_microsd_date_log
};

typedef void (*print_fucntion)(String format);
// typedef void (*printf_fucntion)(string format, ...);

class Logger {
private:
  print_fucntion print;
  // printf_fucntion printf;

public:
  Logger(logger_type target_logger);

  inline void log(String input) { this->print(input); };

  // TODO: Implement printf
  // void logf(char *input, ...) {
  //   va_list args;
  //   va_start(args, input);
  //   printf(input, args);
  //   va_end(args);
  //   this->printf(input);
  // };

  void logln(String input) {
    this->print(input);
    this->print("\r\n");
  };
};

#ifdef ARDUINO
#include <Arduino.h>

inline void init_serial_port() { Serial.begin(9600); }
inline void serial_print(String output) { Serial.print(output); }
// inline void serial_printf(const char *output, ...) { Serial.printf(output); }
#endif

#ifdef ARDUINO_TEENSY41
#include <SD.h>
#include <TimeLib.h>

void init_microsd_log(bool is_date_time_named);
void sd_print(String output);
#endif

#endif // logger_hpp
