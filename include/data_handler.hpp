#pragma once
#include "main.hpp"
#include <can_tools.hpp>
#include <car.h>
#include <stdint.h>

#include <FlexCAN_T4.h>

extern can_obj_car_h_t kms_can;

void start_comms(void);
void process_acc_message(void);
void process_inv_message(void);
void process_daq_message(void);
void send_1hz_messages(void);
void send_20hz_messages(void);
void send_200hz_messages(void);

canMan acc_can(TEENSY_CAN1, ACCUMULATOR_CAN_BAUD_RATE, process_acc_message);
canMan inv_can(TEENSY_CAN2, INVERTER_CAN_BAUD_RATE, process_inv_message);
canMan daq_can(TEENSY_CAN3, DAQ_CAN_BAUD_RATE, process_daq_message);
