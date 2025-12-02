#pragma once
#include "parameters.hpp"
#include <FlexCAN_T4.h>
#include <can_tools.hpp>
#include <car.h>

class VCU;
class Accumulator;

class data_handler {
private:
  canMan acc_can = canMan(TEENSY_CAN1, ACCUMULATOR_CAN_BAUD_RATE);
  canMan inv_can = canMan(TEENSY_CAN2, INVERTER_CAN_BAUD_RATE);
  canMan daq_can = canMan(TEENSY_CAN3, DAQ_CAN_BAUD_RATE);
  can_obj_car_h_t kms_can;
  VCU *vcu = nullptr;
  Accumulator *accumulator = nullptr;

public:
  data_handler(VCU *vcu, Accumulator *accumulator);
  void start_comms(void);
  void process_acc_message(void);
  void process_inv_message(void);
  void process_daq_message(void);
  void send_1hz_messages(void);
  void send_20hz_messages(void);
  void send_200hz_messages(void);
};
