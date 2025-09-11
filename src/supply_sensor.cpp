#include "supply_sensor.h"

supply_sensor_t* supply_sensor_create(int* i, long* i2, int* v) {
  supply_sensor_t* s = (supply_sensor_t*) malloc(sizeof(supply_sensor_t));
  s->raw_i = i;
  s->raw_i2 = i2;
  s->raw_v = v;
  s->i_battery = 0;
  s->i_battery_variance = 0;
  s->v_motor = 0;
  s->i_battery_offset = 1.65;
  s->i_battery_offset_int = s->i_battery_offset/3.3*0x1000;
  s->i_battery_offset2_int = s->i_battery_offset_int * s->i_battery_offset_int;
  // Our current sensors have nominal 100 mV/A.
  // But we run them at 3.3V instead of 5, so we need to apply a correction  
  s->i_battery_scale = 1/0.100 * 5.0/3.3;
  // we have a 22k vs 1k voltage divider, then we apply a measured correction factor
  s->v_motor_scale = 23.0;
  s->v_motor_f = LowPassFilter(.0001);
  // We want to average batty current over one or two PWM cycles.
  // We will get about 5 samples per PWM cycle (max).
  // One PWM cycle is 50 usecs. So 0.0001 corresponds to 2 PWM cycles.
  s->i_battery_f = LowPassFilter(.0001);
  s->i_battery_variance_f = LowPassFilter(.0001);
  return s;
}

void supply_sensor_update(supply_sensor_t* s) {
  s->i_battery = (*s->raw_i * 3.3f/4096.0f - s->i_battery_offset) * s->i_battery_scale;
  s->i_battery_variance = s->i_battery_variance_f((*s->raw_i2 - 2*s->i_battery_offset_int*(*s->raw_i) + s->i_battery_offset2_int)*3.3f*3.3f/0x1000000 * s->i_battery_scale * s->i_battery_scale);
  s->v_motor = s->v_motor_f(*s->raw_v * 3.3f/0x1000) * s->v_motor_scale;
}