#include <stdint.h>
enum converter_state { off, consuming, charging};

struct converter {
  int pwm_pin;
  int enable_pin;
  int slice_num;
  converter_state state;
  float control;
  int level;
  uint32_t last_activated;
};
typedef struct converter converter_t;

converter_t* converter_create(int pwm_pin, int enable_pin);
void converter_init(converter_t* converter);
void converter_enable(converter_t* converter);
void converter_disable(converter_t* converter);

void converter_set_state(converter_t *c, converter_state state);