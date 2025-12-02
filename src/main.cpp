#include "main.hpp"
#include "car.h"
#include "core_pins.h"

void setup() {
  consol.logln("Booting...");

  vcu.init_state_machine();

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
  //
  //// ADC Stage
  apps1.update();
  apps2.update();
  bse.update();
  pedals.update_travel(apps1.value.in, apps2.value.in, bse.value.in);

  vsense_bspd.update();
  vcu.update_bspd(vsense_bspd.value.in, 0, 0);

  //
  //// CAN Stage
  data_handler_obj.process_acc_message();
  data_handler_obj.process_inv_message();
  data_handler_obj.process_daq_message();

  if (timer_1s.check()) {
    vcu.send_firmware_status_message();
    vcu.send_status_message();
    pedals.send_status_message();
    inverter.set_pid_parameters(params[INVERTER_TORQUE_KP].parameter_value,
                                params[INVERTER_TORQUE_KI].parameter_value,
                                params[INVERTER_TORQUE_KD].parameter_value);
  }

  if (timer_20hz.check()) {
    pedals.send_pedal_travel_message();
    pedals.send_pedal_raw_message(
        pedals.get_apps1_raw(), pedals.get_apps2_raw(), pedals.get_brake_raw());

    vcu.send_power_tracking_message();

    consol.log("\n\rraw_apps1: ");
    consol.log(vcu.pedals->get_apps1_raw());
    consol.log("\n\rraw_apps2: ");
    consol.log(vcu.pedals->get_apps2_raw());
    consol.log("\n\rraw_brake: ");
    consol.log(vcu.pedals->get_brake_raw());
  }

  //
  //// Math Stage
  accumulator.calculate_energy_consumed_wh(millis());
  inverter.calculate_power_output();
  inverter.calculate_motor_distance_M(millis());
  inverter.calculate_pid_loop();

  vcu.handle_state_machine();
}
