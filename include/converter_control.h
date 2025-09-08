struct converter_control {
  float last_voltage_reached;
  float last_i;
  float last_i2;
};
typedef struct converter_control converter_control_t;

converter_control_t* converter_control_create();