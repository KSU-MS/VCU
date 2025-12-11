#include "inverter.hpp"
#include "data_handler.hpp"
#include "parameters.hpp"

Inverter::Inverter(bool spin_direction, std::array<parameter, 25> *params,
                   VehicleData *vehicle_data) {
  this->spin_forward = spin_direction;

  this->params = params;
  this->vehicle_data = vehicle_data;

  this->ping();

  this->set_inv_parameter(
      Motor_Overspeed_EEPROM_RPM,
      static_cast<uint32_t>(params->at(MAX_RPM_LIMIT).parameter_value));
  this->set_inv_parameter(
      Max_Speed_EEPROM_RPM,
      static_cast<uint32_t>(params->at(SOFT_RPM_LIMIT).parameter_value));
  this->set_inv_parameter(
      Break_Speed_EEPROM_RPM,
      static_cast<uint32_t>(params->at(BRAKE_SPEED_LIMIT).parameter_value));
  // this->set_inv_parameter(Speed_Rate_Limit_EEPROM_RPM_per_s,
  // SPEED_RATE_LIMIT_RPM_PER_S);

  this->torquepid =
      QuickPID(&torque_over_nm, &torque_adjustment, 0,
               double(torque_kp_x100) / 100, double(torque_ki_x100) / 100,
               double(torque_kd_x100) / 100, QuickPID::Action::direct);
  this->torquepid.SetSampleTimeUs(5000);
}

void Inverter::set_current_limits(uint16_t charge_limit,
                                  uint16_t discharge_limit) {
  data_handler::send_inverter_current_limits(charge_limit, discharge_limit);
}

void Inverter::calculate_power_output() {
  // power output is calculated as bus_current * bus_voltage
  if (vehicle_data == nullptr) {
    return;
  }

  auto &inv_data = vehicle_data->inverter;
  inv_data.power_output_w = inv_data.bus_current * inv_data.bus_voltage;
}

void Inverter::calculate_motor_distance_M(uint32_t time_msec) {
  if (vehicle_data == nullptr) {
    return;
  }

  auto &inv_data = vehicle_data->inverter;
  uint32_t time_elaped_msec =
      time_msec - inv_data.last_distance_calc_timestamp_ms;

  double velocity_Msec =
      (double(inv_data.motor_rpm) / 60 / GEAR_RATIO) * WHEEL_CIRCUMFRANCE_M;

  inv_data.motor_distance_m +=
      (double(time_elaped_msec) / 1000) * velocity_Msec;

  inv_data.last_distance_calc_timestamp_ms = time_msec;
}

void Inverter::ping() {
  data_handler::send_inverter_ping(spin_forward, inverter_enable,
                                   inverter_discharge);
}

void Inverter::send_clear_faults() {
  data_handler::send_inverter_clear_faults();
}

void Inverter::command_torque(double torque_request) {
  if (vehicle_data == nullptr) {
    return;
  }

  auto &inv_data = vehicle_data->inverter;
  double torque_target = torque_request;

  // EV. 1.4.4 A violation is defined as using more than the specified maximum
  // power OR exceeding the maximum voltage EITHER: a. Continuously for 100 ms
  // or more b. After a moving average over 500 ms is applied

  // electronic rev limiter

  // https://www.desmos.com/calculator/j8kydktjry

  // calculate excess power output
  double power_over_w = std::max(
      0.0, inv_data.power_output_w -
               (((*params)[POWER_LIMIT].parameter_value / 10.0) * 1000.0));

  if (power_over_w > 1e-6) {
    // calculate excess torque output from motor speed and excess power
    torque_over_nm =
        power_over_w / std::max(1e-6, (inv_data.motor_rpm / 60.0) * 2.0 * M_PI);

    // cap torque adjustment to 10% of max torque
    double torque_adjustment_capped =
        std::min(torque_over_nm, (*params)[MAX_TORQUE].parameter_value * 0.1);

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

  data_handler::send_inverter_torque_command(torque_target);
}

void Inverter::command_speed(int16_t speed_request) // unused
{
  data_handler::send_inverter_speed_command(
      speed_request, spin_forward, speed_mode, inverter_enable,
      inverter_discharge, (*params)[MAX_TORQUE].parameter_value);
}

void Inverter::set_inv_parameter(uint16_t param_address, uint32_t param_data) {
  data_handler::send_inverter_parameter(param_address, param_data);
}
