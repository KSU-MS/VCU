#include "pedal_handeler.hpp"
#include "data_handler.hpp"
#include <cstdint>

void Pedals::update_travel(uint16_t raw_apps1, uint16_t raw_apps2,
                           uint16_t raw_brake) {
  if (vehicle_data == nullptr) {
    return;
  }

  auto &pedal_data = vehicle_data->pedals;

  pedal_data.raw_apps1 = raw_apps1;
  pedal_data.raw_apps2 = raw_apps2;
  pedal_data.raw_brake = raw_brake;

  // Get the pedal percentage in its throw from 0 to 1

  // Update 'travel' values only, do not check faults here
  pedal_data.apps1_travel = (raw_apps1 - apps1_start) * apps1_ratio;
  pedal_data.apps1_travel = std::clamp(pedal_data.apps1_travel, 0.0, 1.0);

  pedal_data.apps2_travel = (raw_apps2 - apps2_start) * apps2_ratio;
  pedal_data.apps2_travel = std::clamp(pedal_data.apps2_travel, 0.0, 1.0);

  pedal_data.brake_travel = (raw_brake - brake_start) * brake_ratio;
  pedal_data.brake_travel = std::clamp(pedal_data.brake_travel, 0.0, 1.0);

  // Check APPS1 and APPS2 range, reset fault if it is outside of range  //
  // Check APPS1 and APPS2 range, reset fault if it is outside of range
  pedal_data.apps_fault = pedal_data.raw_apps1 < apps_low_fault ||
                          pedal_data.raw_apps2 < apps_low_fault;

  // Check Brake sensor encoder out of range fault, reset fault if it is outside
  // of range
  pedal_data.bse_fault = pedal_data.raw_brake > bse_high_fault ||
                         pedal_data.raw_brake < bse_low_fault;

  // Check that the driver isn't using both brake and throttle at once
  pedal_data.apps_bse_fault =
      (pedal_data.throttle_travel > 0.3) && (pedal_data.brake_travel > 0.3);

  // T.4.2.4
  // Check that there is no apps related faults
  if (pedal_data.apps_fault == true || pedal_data.bse_fault == true ||
      pedal_data.apps_bse_fault == true) {
    pedal_data.throttle_travel = 0;
  } else {

    // Check that the pedals are reading within 10%
    if ((fabs(pedal_data.apps1_travel - pedal_data.apps2_travel) > 0.3)) {
      pedal_data.apps_fault = true;
      pedal_data.throttle_travel = 0;
    } else {
      pedal_data.throttle_travel =
          (pedal_data.apps1_travel + pedal_data.apps2_travel) / 2;

      // Check that the driver isn't using both pedals at once
      if ((pedal_data.throttle_travel > 0.3) &&
          (pedal_data.brake_travel > 0.3)) {
        pedal_data.apps_bse_fault = true;
        pedal_data.throttle_travel = 0;
      }
    }
  }
}

void Pedals::check_hard_faults() {
  if (vehicle_data == nullptr) {
    return;
  }

  auto &pedal_data = vehicle_data->pedals;

  // Check BSPD high side fault, do not reset fault if it is already set
  pedal_data.bspd_ok_hs = vsense_bspd.value.in > 500 || pedal_data.bspd_ok_hs;
}

void Pedals::pedal_main_loop() {}

void Pedals::pedal_200hz_loop() {
  if (vehicle_data == nullptr) {
    return;
  }

  apps1.update();
  apps2.update();
  bse.update();
  vsense_bspd.update();

  check_hard_faults();

  update_travel(apps1.value.in, apps2.value.in, bse.value.in);
}

void Pedals::pedal_10hz_loop() {
  if (vehicle_data == nullptr) {
    return;
  }

  auto &pedal_data = vehicle_data->pedals;

  data_handler::send_pedal_travel_message(pedal_data.apps1_travel,
                                          pedal_data.apps2_travel,
                                          pedal_data.brake_travel);
  data_handler::send_pedal_raw_message(
      pedal_data.raw_apps1, pedal_data.raw_apps2, pedal_data.raw_brake);
}