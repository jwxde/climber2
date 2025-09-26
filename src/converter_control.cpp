#include "converter_control.h"
#include "converter.h"
#include "supply_sensor.h"

#include <cstdlib>

converter_control_t* converter_control_create() {
    converter_control_t* c = (converter_control_t*) malloc(sizeof(converter_control_t));
    c->input_voltage_estimate_f = LowPassFilter(1.0);
    c->state = waiting;
    return c;
}



void converter_control_do(converter_control_t* converter_control, converter_t* converter, supply_sensor_t* supply_sensor) {

  #if false
  // Catch some power issues
  if(supply_sensor->v_motor > 30.0 || supply_sensor->v_motor < 10.0 && supply_sensor->i_battery > 1) {
    // Our regulation seems to fail, shut down motor
    motor->disable();
    Serial.println("Over or under voltage, motor disabled, shutdown");
    initOk = false;
  }
  if(false & abs(supply_sensor->i_battery) > 12.0) {
    motor->disable();
    converter_set_state(converter, off);
    Serial.println("Over current, motor and converter disabled");
    initOk = false;
  }
  #endif
  
  if(converter->state == off) {
    if (supply_sensor->v_motor < 26.0) {
        converter_control->target_voltage = 28.0;    
        converter_set_state(converter, consuming);
    }
    if(supply_sensor->v_motor > 29.0) {
        converter_control->target_voltage = 26.0;
        converter_set_state(converter, charging);
    }
  } else {
    if(converter->state == consuming && supply_sensor->v_motor >= converter_control->target_voltage
        || converter->state == charging && supply_sensor->v_motor <= converter_control->target_voltage
        || micros() - converter->last_activated > 1000) {
        converter_set_state(converter, off);
        // Take note of the voltage we reached as a basis for future adjustments
        converter_control->last_voltage_reached = supply_sensor->v_motor;
        converter_control->last_i = supply_sensor->i_battery;
        converter_control->last_i2 = supply_sensor->i_battery_variance;
        converter_control->input_voltage_estimate = converter_control->input_voltage_estimate_f(converter_control->last_voltage_reached/converter->control);

        // Section below only makes sense if we regulate the control variable
        #if false
        // In case there is no current flow through the coil, assume a power failure
        // and choose a safe starting point for power return instead of adjusting
        if(converter_control->last_i2 > 2.0 && converter_control->input_voltage_estimate > 0) {
          converter->control = 27.0/converter_control->input_voltage_estimate;
        } else {
          converter->control = 2;
        }
        // Note: changes will be applied next time the converter is switched on.
        #endif
    }
  }
}
