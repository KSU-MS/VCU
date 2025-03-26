#include "logger.hpp"

Logger::Logger(logger_type target_logger) {
  switch (target_logger) {
  case serial:
    init_serial_port();

    this->print = serial_print;
    // this->printf = serial_printf;
    break;

  case teensy_microsd_numbered_log:
    init_microsd_log(false);

    this->print = sd_print;
    break;

  case teensy_microsd_date_log:
    init_microsd_log(true);

    this->print = sd_print;
    break;
  }
}

#ifdef ARDUINO_TEENSY41
File sd_logger;

void init_microsd_log(bool is_date_time_named) {
  if (is_date_time_named) {
    SD.begin(BUILTIN_SDCARD);

    // Make name of current time
    int time = Teensy3Clock.get();
    const char *filename =
        ("MDY_" + String(month(time)) + "-" + String(day(time)) + "-" +
         String(year(time)) + "_HMS_" + String(hour(time)) + "-" +
         String(minute(time)) + "-" + String(second(time)) + ".csv")
            .c_str();

    if (!SD.exists(filename)) { // Open file for writing
      sd_logger = SD.open(filename, (uint8_t)O_WRITE | (uint8_t)O_CREAT);
    }
  } else {
    int index = 0;
    const char *filename = (String(index) + String(".csv")).c_str();

    while (SD.exists(filename)) { // Iterate until we find a file for writing
      index++;
      filename = (String(index) + String(".csv")).c_str();
    }

    sd_logger = SD.open(filename, (uint8_t)O_WRITE | (uint8_t)O_CREAT);
  }
}

inline void sd_print(String output) { sd_logger.print(output); };
#endif
