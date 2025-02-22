#pragma once
#include <stdarg.h>

enum logger_type {
  serial,
  teensy_microsd_numbered_log,
  teensy_microsd_date_log
};

typedef void (*print_fucntion)(const char *format);
typedef void (*printf_fucntion)(const char *format, ...);

class logger {
private:
  print_fucntion print;
  printf_fucntion printf;

public:
  void init(logger_type target_logger);

  inline void log(char *input) { this->print(input); };

  // TODO: Implement printf
  // void logf(char *input, ...) {
  //   va_list args;
  //   va_start(args, input);
  //   printf(input, args);
  //   va_end(args);
  //   this->printf(input);
  // };

  void logln(const char *input) {
    this->print(input);
    this->print("\r\n");
  };
};

#ifdef ARDUINO
#include <Arduino.h>

inline void init_serial_port() { Serial.begin(9600); }
inline void serial_print(const char *output) { Serial.print(output); }
// inline void serial_printf(const char *output, ...) { Serial.printf(output); }
#endif

#ifdef ARDUINO_TEENSY41
#include <SD.h>
#include <TimeLib.h>

File sd_logger;

void init_microsd_log(bool is_date_time_named);
inline void sd_print(const char *output) { sd_logger.print(output); }
#endif
