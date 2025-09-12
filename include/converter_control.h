#pragma once

#include <SimpleFOC.h>
#include "converter.h"
#include "supply_sensor.h"

enum converter_control_state { waiting, going_up, going_down, locked };
struct converter_control {
  float last_voltage_reached;
  float input_voltage_estimate;
  float last_i;
  float last_i2;
  converter_control_state state;
  float target_voltage;
  LowPassFilter input_voltage_estimate_f;
};
typedef struct converter_control converter_control_t;

converter_control_t* converter_control_create();
void converter_control_do(converter_control_t* converter_control, converter_t* converter, supply_sensor_t* supply_sensor);
