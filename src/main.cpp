#include "main.hpp"
#include <car.h>

void setup() {
  consol.logln("Booting...");

  pedals = std::make_unique<Pedals>(
      MIN_BRAKE_PEDAL, START_BRAKE_PEDAL, END_BRAKE_PEDAL, MAX_BRAKE_PEDAL,
      MIN_APPS_PEDAL, START_ACCELERATOR_PEDAL_1, END_ACCELERATOR_PEDAL_1,
      START_ACCELERATOR_PEDAL_2, END_ACCELERATOR_PEDAL_2, &vehicle_data);

  inverter = std::make_unique<Inverter>(false, &params, &vehicle_data);
  accumulator = std::make_unique<Accumulator>(&params, &vehicle_data);
  state_machine =
      std::make_unique<StateMachine>(inverter.get(), &params, &vehicle_data);
  data_handler = std::make_unique<DataHandler>(&params, &vehicle_data);

  state_machine->init_state_machine();

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

  data_handler->data_handler_main_loop();
  if (pedals) {
    pedals->pedal_main_loop();
  }
  if (inverter) {
    inverter->inverter_main_loop();
  }
  if (accumulator) {
    accumulator->accumulator_main_loop();
  }
  if (state_machine) {
    state_machine->state_machine_main_loop();
  }

  if (timer_1s.check()) {

    if (inverter) {
      inverter->inverter_1hz_loop();
    }
    if (state_machine) {
      state_machine->state_machine_1hz_loop();
    }
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

    data_handler->data_handler_200hz_loop();
    if (pedals) {
      pedals->pedal_200hz_loop();
    }
    if (inverter) {
      inverter->inverter_200hz_loop();
    }
    if (accumulator) {
      accumulator->accumulator_200hz_loop();
    }
    if (state_machine) {
      state_machine->state_machine_200hz_loop();
    }
  }
}
