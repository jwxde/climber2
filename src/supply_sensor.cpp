#include "supply_sensor.h"

supply_sensor_t* supply_sensor_create() {
  supply_sensor_t* s = (supply_sensor_t*) malloc(sizeof(supply_sensor_t));
  s->i_battery = 0;
  s->i_battery_variance = 0;
  s->i_battery_raw = 0;
  s->v_motor = 0;
  s->i_battery_offset = 1.65;
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

extern RP2040ADCEngine engine; 

void supply_sensor_update(supply_sensor_t* s) {
  ADCResults adcResults = engine.getLastResults();

  // We filter the raw values
  // Be aware that the channel numbers correspond to the hardware channel numbers
  // which do not match with the pin labels for all boards
  float i_battery_raw_now = adcResults.ch4 * 3.3f/256;
  s->i_battery_raw = s->i_battery_f(i_battery_raw_now);
  s->i_battery = (s->i_battery_raw - s->i_battery_offset) * s->i_battery_scale;
  s->v_motor = s->v_motor_f(adcResults.ch5 * 3.3f/256) * s->v_motor_scale;
  // For the battery current variance, we filter the computed values.
  float i_battery_deviation = (i_battery_raw_now - s->i_battery_raw) * s->i_battery_scale;
  s->i_battery_variance = s->i_battery_variance_f(i_battery_deviation*i_battery_deviation);
}