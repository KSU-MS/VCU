#include "inverter.hpp"
#include "data_handler.hpp"
#include "vehicle.hpp"

Inverter::Inverter(bool spin_direction, std::array<Parameter, 25> *params,
                   VehicleData *vehicle_data)
    : params(params), vehicle_data(vehicle_data) {
  vehicle_data->inverter.spin_forward = spin_direction;

  this->ping();

  this->set_inv_parameter(Motor_Overspeed_EEPROM_RPM,
                          this->params->at(MAX_RPM_LIMIT_ID).value);
  this->set_inv_parameter(Max_Speed_EEPROM_RPM,
                          this->params->at(SOFT_RPM_LIMIT_ID).value);
  this->set_inv_parameter(Break_Speed_EEPROM_RPM,
                          this->params->at(BRAKE_SPEED_LIMIT_ID).value);
  // this->set_inv_parameter(Speed_Rate_Limit_EEPROM_RPM_per_s,
  // SPEED_RATE_LIMIT_RPM_PER_S);

  this->torquepid =
      QuickPID(&torque_over_nm, &torque_adjustment, 0,
               double(torque_kp_x100) / 100, double(torque_ki_x100) / 100,
               double(torque_kd_x100) / 100, QuickPID::Action::direct);
  this->torquepid.SetSampleTimeUs(5000);
}

void Inverter::set_inverter_current_limits(uint16_t charge_limit,
                                           uint16_t discharge_limit) {
  DataHandler::send_inverter_current_limits(charge_limit, discharge_limit);
}

void Inverter::calculate_power_output() {
  // power output is calculated as bus_current * bus_voltage
  if (vehicle_data == nullptr) {
    return;
  }

  vehicle_data->inverter.power_output_w =
      vehicle_data->inverter.bus_current * vehicle_data->inverter.bus_voltage;
}

void Inverter::calculate_motor_distance_M(uint32_t time_msec) {

  uint32_t time_elaped_msec =
      time_msec - vehicle_data->inverter.last_distance_calc_timestamp_ms;

  double velocity_Msec =
      (double(vehicle_data->inverter.motor_rpm) / 60 / GEAR_RATIO) *
      WHEEL_CIRCUMFRANCE_M;

  vehicle_data->inverter.motor_distance_m +=
      (double(time_elaped_msec) / 1000) * velocity_Msec;

  vehicle_data->inverter.last_distance_calc_timestamp_ms = time_msec;
}

void Inverter::ping() { DataHandler::send_inverter_ping(); }

void Inverter::send_clear_faults() {
  DataHandler::send_inverter_clear_faults();
}

void Inverter::request_torque(double torque_request) {
  if (vehicle_data == nullptr) {
    return;
  }

  vehicle_data->inverter.torque_target_nm = torque_request;
}

void Inverter::command_torque() {
  double torque_target = vehicle_data->inverter.torque_target_nm;

  // EV. 1.4.4 A violation is defined as using more than the specified maximum
  // power OR exceeding the maximum voltage EITHER: a. Continuously for 100 ms
  // or more b. After a moving average over 500 ms is applied

  // electronic rev limiter

  // https://www.desmos.com/calculator/j8kydktjry

  // calculate excess power output
  double power_over_w = std::max(
      0.0, vehicle_data->inverter.power_output_w -
               (this->params->at(POWER_LIMIT_ID).value / 10.0) * 1000.0);

  if (power_over_w > 1e-6) {
    // calculate excess torque output from motor speed and excess power
    torque_over_nm =
        power_over_w /
        std::max(1e-6, (vehicle_data->inverter.motor_rpm / 60.0) * 2.0 * M_PI);

    // cap torque adjustment to 10% of max torque
    double torque_adjustment_capped =
        std::min(torque_over_nm, this->params->at(MAX_TORQUE_ID).value * 0.1);

    // ensure torque subtracted is not negative
    torque_adjustment_capped = std::max(0.0, torque_adjustment_capped);

    // adjust torque target
    torque_target -= torque_adjustment_capped;

    // ensure torque target is not negative
    torque_target = std::max(0.0, torque_target);
  } else {
    torque_over_nm = 0.0;
  }
  // else
  // {
  //   torque_I *= 0.98;
  //   // torque_target -= torque_kp * torque_over_nm + torque_I;
  // }
  // speed limiter
  // if (angular_vel_over_rad_s > 0)
  // {
  //   torque_over_nm += power_over_w / angular_vel_over_rad_s;
  // }
  // else
  // {
  //   speed_I *= 0.98;
  // }
  // double speed_err = speed_kp * angular_vel_over_rad_s + speed_I;
  // torque_target -= speed_err;

  // if (motor_rpm >= speed_limit && torque_target > 0)
  // {
  //   speed_I += speed_ki * ((motor_rpm - speed_limit) / speed_limit) * dt_s;
  //   torque_target *= 1 - (speed_kp * ((motor_rpm - speed_limit) /
  //   speed_limit) + speed_I);
  // }
  // else
  // {
  //   speed_I *= 0.98;
  // }

  // all of this should really be handled elsewhere, we call this function on a
  // 200hz interval

  DataHandler::send_inverter_torque_command(torque_target);
}

void Inverter::set_inv_parameter(uint16_t param_address, uint32_t param_data) {
  DataHandler::send_inverter_parameter(param_address, param_data);
}

void Inverter::inverter_main_loop() {
  this->calculate_pid_loop();
  this->calculate_power_output();
  this->calculate_motor_distance_M(millis());
}

void Inverter::inverter_200hz_loop() {
  if (vehicle_data->state_machine.current_state !=
      StateMachineData::state::READY_TO_DRIVE) {
    this->ping();
    vehicle_data->inverter.inverter_enable = false;
  } else {
    vehicle_data->inverter.inverter_enable = true;
    this->command_torque();
  }
}

void Inverter::inverter_10hz_loop() {}

void Inverter::inverter_1hz_loop() {
  this->set_pid_parameters(this->params->at(INVERTER_TORQUE_KP_X100_ID).value,
                           this->params->at(INVERTER_TORQUE_KI_X100_ID).value,
                           this->params->at(INVERTER_TORQUE_KD_X100_ID).value);
}