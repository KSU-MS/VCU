#include "main.hpp"
#include <car.h>
#include <core_pins.h>

void setup() {
  consol.logln("Booting...");

  state_machine.init_state_machine();

  // TODO: Get rid of these evil arduino calls for the buzzer
  pinMode(BUZZER, OUTPUT);

  // Pump fellas
  pinMode(LOWSIDE1, OUTPUT);
  pinMode(LOWSIDE2, OUTPUT);
  digitalWrite(LOWSIDE1, LOW);
  digitalWrite(LOWSIDE2, LOW);

  consol.logln("Booted");
}

void loop() {

  data_handler.data_handler_main_loop();
  pedals.pedal_main_loop();
  inverter.inverter_main_loop();
  accumulator.accumulator_main_loop();
  state_machine.state_machine_main_loop();

  if (timer_1s.check()) {

    inverter.inverter_1hz_loop();
    state_machine.state_machine_1hz_loop();
  }

  if (timer_20hz.check()) {

    consol.log("\n\rraw_apps1: ");
    consol.log(vehicle_data.pedals.raw_apps1);
    consol.log("\n\rraw_apps2: ");
    consol.log(vehicle_data.pedals.raw_apps2);
    consol.log("\n\rraw_brake: ");
    consol.log(vehicle_data.pedals.raw_brake);
  }

  if (timer_200hz.check()) {

    // data_handler.data_handler_200hz_loop();
    pedals.pedal_200hz_loop();
    inverter.inverter_200hz_loop();
    accumulator.accumulator_200hz_loop();
    state_machine.state_machine_200hz_loop();
  }
}
