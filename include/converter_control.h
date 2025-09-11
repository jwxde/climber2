#include <SimpleFOC.h>

struct converter_control {
  float last_voltage_reached;
  float input_voltage_estimate;
  float last_i;
  float last_i2;
  LowPassFilter input_voltage_estimate_f;
};
typedef struct converter_control converter_control_t;

converter_control_t* converter_control_create();