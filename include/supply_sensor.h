#pragma once

#include <SimpleFOC.h>
#include <current_sense/hardware_specific/rp2040/rp2040_mcu.h>

struct supply_sensor {
  int* raw_i;
  long* raw_i2;
  int* raw_v;
  float i_battery;
  float i_battery_variance;
  float v_motor;
  float i_battery_raw;
  float i_battery_variance_raw;
  float i_battery_offset;
  int i_battery_offset_int;
  long i_battery_offset2_int;
  float i_battery_scale;
  float v_motor_offset;
  float v_motor_scale;
  LowPassFilter v_motor_f;
  LowPassFilter i_battery_f;
  LowPassFilter i_battery_variance_f;
};

typedef struct supply_sensor supply_sensor_t;

supply_sensor_t* supply_sensor_create(int* raw_i, long* raw_i2, int* raw_v);
void supply_sensor_update(supply_sensor_t* s);