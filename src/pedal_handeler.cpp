#include "pedal_handeler.hpp"
#include "data_handler.hpp"

void Pedals::update_travel(uint16_t raw_apps1, uint16_t raw_apps2,
                           uint16_t raw_brake) {

  vehicle_data->pedals.raw_apps1 = raw_apps1;
  vehicle_data->pedals.raw_apps2 = raw_apps2;
  vehicle_data->pedals.raw_brake = raw_brake;

  // Get the pedal percentage in its throw from 0 to 1

  // Update 'travel' values only, do not check faults here
  vehicle_data->pedals.apps1_travel = (raw_apps1 - apps1_start) * apps1_ratio;
  vehicle_data->pedals.apps1_travel =
      std::clamp(vehicle_data->pedals.apps1_travel, 0.0, 1.0);

  vehicle_data->pedals.apps2_travel = (raw_apps2 - apps2_start) * apps2_ratio;
  vehicle_data->pedals.apps2_travel =
      std::clamp(vehicle_data->pedals.apps2_travel, 0.0, 1.0);

  vehicle_data->pedals.brake_travel = (raw_brake - brake_start) * brake_ratio;
  vehicle_data->pedals.brake_travel =
      std::clamp(vehicle_data->pedals.brake_travel, 0.0, 1.0);

  // Check APPS1 and APPS2 range, reset fault if it is outside of range  //
  // Check APPS1 and APPS2 range, reset fault if it is outside of range
  vehicle_data->pedals.apps_fault =
      vehicle_data->pedals.raw_apps1 < apps_low_fault ||
      vehicle_data->pedals.raw_apps2 < apps_low_fault;

  // Check Brake sensor encoder out of range fault, reset fault if it is outside
  // of range
  vehicle_data->pedals.bse_fault =
      vehicle_data->pedals.raw_brake > bse_high_fault ||
      vehicle_data->pedals.raw_brake < bse_low_fault;

  // Check that the driver isn't using both brake and throttle at once
  vehicle_data->pedals.apps_bse_fault =
      (vehicle_data->pedals.throttle_travel > 0.3) &&
      (vehicle_data->pedals.brake_travel > 0.3);

  // T.4.2.4
  // Check that there is no apps related faults
  if (vehicle_data->pedals.apps_fault == true ||
      vehicle_data->pedals.bse_fault == true ||
      vehicle_data->pedals.apps_bse_fault == true) {
    vehicle_data->pedals.throttle_travel = 0;
  } else {

    // Check that the pedals are reading within 10%
    if ((fabs(vehicle_data->pedals.apps1_travel -
              vehicle_data->pedals.apps2_travel) > 0.3)) {
      vehicle_data->pedals.apps_fault = true;
      vehicle_data->pedals.throttle_travel = 0;
    } else {
      vehicle_data->pedals.throttle_travel =
          (vehicle_data->pedals.apps1_travel +
           vehicle_data->pedals.apps2_travel) /
          2;

      // Check that the driver isn't using both pedals at once
      if ((vehicle_data->pedals.throttle_travel > 0.3) &&
          (vehicle_data->pedals.brake_travel > 0.3)) {
        vehicle_data->pedals.apps_bse_fault = true;
        vehicle_data->pedals.throttle_travel = 0;
      }
    }
  }
}

void Pedals::check_hard_faults() {

  // Check BSPD high side fault, do not reset fault if it is already set
  vehicle_data->pedals.bspd_ok_hs =
      // vsense_bspd.value.in > 500 || vehicle_data->pedals.bspd_ok_hs;
      0;
}

void Pedals::pedal_main_loop() {}

void Pedals::pedal_200hz_loop() {

  // apps1.update();
  // apps2.update();
  // bse.update();
  // vsense_bspd.update();

  check_hard_faults();

  // update_travel(apps1.value.in, apps2.value.in, bse.value.in);
  update_travel(0, 0, 0);
}

void Pedals::pedal_10hz_loop() {

  data_handler::send_pedal_travel_message(vehicle_data->pedals.apps1_travel,
                                          vehicle_data->pedals.apps2_travel,
                                          vehicle_data->pedals.brake_travel);
  data_handler::send_pedal_raw_message(vehicle_data->pedals.raw_apps1,
                                       vehicle_data->pedals.raw_apps2,
                                       vehicle_data->pedals.raw_brake);
}