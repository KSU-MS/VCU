#include "vehicle.hpp"
#include <SD.h>

bool ParameterStore::sd_ready = false;

bool ParameterStore::begin() {
  sd_ready = SD.begin(SD_CS_PIN);
  return sd_ready;
}
