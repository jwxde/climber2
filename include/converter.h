#pragma once
#include <stdint.h>

enum converter_state { off, consuming, charging};

typedef void (*converter_cycle_handler_t)(void);

struct converter {
  int pwm_pin;
  int enable_pin;
  converter_state state;
  float control;
  int level;
  uint32_t last_activated;
  converter_cycle_handler_t cycle_handler;
  void* implementation_data;
};
typedef struct converter converter_t;

converter_t* converter_create(int pwm_pin, int enable_pin);
bool converter_init(converter_t* converter);
void converter_enable(converter_t* converter);
void converter_disable(converter_t* converter);

void converter_set_state(converter_t *c, converter_state state);

void converter_set_cycle_handler(converter_t *c, converter_cycle_handler_t f);
