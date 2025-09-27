#if ARDUINO_NUCLEO_G474RE

#include "converter.h"
#include <cstdlib>
#include <Arduino.h>
#include <drivers/hardware_specific/stm32/stm32_searchtimers.h>
#include <drivers/hardware_specific/stm32/stm32_timerutils.h>

typedef struct converter_data {
  TIM_HandleTypeDef* handle_c;
  TIM_HandleTypeDef* handle_e;
  uint32_t channel_c;
  uint32_t channel_e;
  uint32_t ll_channel_c;
  uint32_t ll_channel_e;
  long pwm_frequency;
  uint8_t num_timers;
  TIM_HandleTypeDef* master_timer;
} converter_data_t;

static converter_t c1;
static converter_data_t cd1;

// We reuse some functions from the SimpleFOC driver implementation for STM32
TIM_HandleTypeDef* stm32_initPinPWM(uint32_t PWM_freq, PinMap* timer, uint32_t mode = TIM_OCMODE_PWM1, uint32_t polarity = TIM_OCPOLARITY_HIGH, uint32_t Npolarity = TIM_OCNPOLARITY_HIGH);
int stm32_checkTimerFrequency(long pwm_frequency, TIM_HandleTypeDef *timers[], uint8_t num_timers);

converter_t* converter_create(int pwm_pin, int enable_pin) {
  converter_t* converter = &c1;
  converter->pwm_pin = pwm_pin;
  converter->enable_pin = enable_pin;
  converter->implementation_data = &cd1;

  #if false
  converter->slice_num = pwm_gpio_to_slice_num(converter->pwm_pin);
  if(converter->slice_num != pwm_gpio_to_slice_num(converter->enable_pin)) {
    // BAD, TODO: DO something
  }
  #endif
  return converter;
}

/**
 * We want the control signal to be the inverse duty ratio.
 * When consuming, we use an inverted PWM output so that our PWM signal starts with a low phase.
 * Hence to get to duty ratio > 0.5 we need to put the level below 0.5*range.
 * When charging, we use a non inverted PWM output so that our PWM signal starts with a high phase.
 * Hence to get a duty ration > 0.5 wee need to put the level above 0.5*range.
 * BUT: When charging we want to reverse the relation anyways, so we end up using the same
 * formula in both cases.
 */
void converter_apply(converter_t *converter) {
  converter_data_t* cd = (converter_data_t*) converter->implementation_data;
  if(converter->control > 4.0) converter->control = 4.0;
  if(converter->control < 1.25) converter->control = 1.25;
  converter->level = (cd->handle_c->Instance->ARR + 1)*(1.0 - 1.0/converter->control);
  stm32_setPwm(cd->handle_c, cd->channel_c, converter->level);
}

void converter_set_state(converter_t *c, converter_state state) {
  converter_data_t* cd = (converter_data_t*) c->implementation_data;

  // We only support state transitions from off into one of the non off phases.
  // Why? We want to change the polarity of the PWM signal between consuming
  // and charging states. But the polarity becomes effective immediately
  // which we don't want unless we are coming from an off state.
  switch(c->state) {
    case off:
      switch(state) {
        case consuming:
          // TODO: pwm_set_output_polarity(c->slice_num, true, false);
          c->state = consuming;
          c->last_activated = micros();
          break;
        case charging:
          // TODO: pwm_set_output_polarity(c->slice_num, false, false);
          c->state = charging;
          c->last_activated = micros();
          break;
        case off:
          // nothing to do
          break;
      }
      break;
    default:
      switch(state) {
        case off:
          c->state = off;
          break;
        default:
          // Can't change anything now
          break;
      }
  }
  converter_apply(c);
  // TODO: Check that this is glitch free at 0 and full scale
  stm32_setPwm(cd->handle_e, cd->channel_e, c->state == converter_state::off ? 0 : cd->handle_e->Instance->ARR + 1);
}

void converter_disable(converter_t* converter) {
  // TODO: gpio_set_outover(converter->pwm_pin, GPIO_OVERRIDE_HIGH);
  // TODO: gpio_set_outover(converter->enable_pin, GPIO_OVERRIDE_LOW);
}

void converter_enable(converter_t* converter) {
  // TODO: gpio_set_outover(converter->pwm_pin, GPIO_OVERRIDE_NORMAL);
  // TODO: gpio_set_outover(converter->enable_pin, GPIO_OVERRIDE_NORMAL);
}

bool converter_init(converter_t* converter) {
  converter_data_t* cd = (converter_data_t*) converter->implementation_data;

  // It is crucial that we set up the PWM to a sane pulse width
  // (too much connection to ground will short our power source)
  // before enabling the half bridge by connecting the output pins
  
  long pwm_frequency = 25000;

  // Setup code in analogy to Arduino-FOC/src/current_sense/hardware_specific/stm32/stm32_mcu.cpp
  // The pins should be channel 1 and 2 of the same timer
  int pins[2] = { converter->pwm_pin, converter->enable_pin };
  PinMap* pinTimers[2] = { NULL, NULL };
  if (stm32_findBestTimerCombination(2, pins, pinTimers)<0) {
    SIMPLEFOC_DEBUG("STM32-CNV: converter pins are not on the same timer");
    return false;
  }
  TIM_HandleTypeDef* HT1 = stm32_initPinPWM(pwm_frequency, pinTimers[0], TIM_OCMODE_PWM1, (SIMPLEFOC_PWM_ACTIVE_HIGH)?TIM_OCPOLARITY_HIGH:TIM_OCPOLARITY_LOW);
  TIM_HandleTypeDef* HT2 = stm32_initPinPWM(pwm_frequency, pinTimers[1], TIM_OCMODE_PWM1, (SIMPLEFOC_PWM_ACTIVE_HIGH)?TIM_OCPOLARITY_HIGH:TIM_OCPOLARITY_LOW);

  TIM_HandleTypeDef *timers[2] = {HT1, HT2};
  stm32_checkTimerFrequency(pwm_frequency, timers, 2);

  uint32_t channel1 = STM_PIN_CHANNEL(pinTimers[0]->function);
  uint32_t channel2 = STM_PIN_CHANNEL(pinTimers[1]->function);

  cd->handle_c = HT1;
  cd->handle_e = HT2;
  cd->channel_c = channel1;
  cd->channel_e = channel2;
  cd->ll_channel_c = stm32_getLLChannel(pinTimers[0]) ;
  cd->ll_channel_e = stm32_getLLChannel(pinTimers[1]);
  cd->pwm_frequency = pwm_frequency;
  cd->num_timers = stm32_countTimers(timers, 2);
  cd->master_timer = stm32_alignTimers(timers, 2);

  // In order to eliminate all risk, we configure pull ups/downs
  // the way we want the default signal to be (half bridge not enabled, hi side connected)
  // and disable output. We need to set the enablement state after setting the pin function as
  // the pin function also sets the enablement state.
  // TODO: gpio_pull_up(converter->pwm_pin);
  // TODO: gpio_pull_down(converter->enable_pin);
  // TODO: gpio_set_function(converter->pwm_pin, GPIO_FUNC_PWM);
  // TODO: gpio_set_function(converter->enable_pin, GPIO_FUNC_PWM);
  converter_disable(converter);
 
  // Don't invert the enable pin signal so that we get an off signal
  // by default (for level 0).
  // TODO: pwm_set_output_polarity(converter->slice_num, false, false);
  
  // TODO: pwm_set_clkdiv_mode(converter->slice_num, PWM_DIV_FREE_RUNNING);
  // This will result in roughly 20kHz frequency
  // TODO: pwm_set_wrap(converter->slice_num, 4095);
  // TODO: pwm_set_clkdiv_int_frac4(converter->slice_num, 2, 0);
  // TODO: pwm_set_phase_correct(converter->slice_num, false);

  // Start out with 50% duty cycle so that we get a voltage that is about twice the battery voltage.
  // But do not enable the converter yet.
  converter->control = 2.0;
  converter_apply(converter);
  converter_set_state(converter, off);
  converter->last_activated = 0;

  // Kick off the PWM slice. The SimpleFOC PWM driver will later (re) enable all slices.
  // TODO: pwm_set_enabled(converter->slice_num, true);

  // And let the signal show up on the output pins
  converter_enable(converter);

  return true;
}

void converter_handle_irq(converter_t* converter) {
  // TODO: 
  //if(pwm_get_irq_status_mask() && (0x1 << converter->slice_num)) {
  //  pwm_clear_irq(converter->slice_num);
  //  converter->cycle_handler();
  //}
}

void converter_h1() {
  converter_handle_irq(&c1);
}

void converter_set_cycle_handler(converter_t *converter, converter_cycle_handler_t f) {
  
  converter->cycle_handler = f;

  // TODO: pwm_clear_irq(converter->slice_num);
  // TODO: pwm_set_irq_enabled(converter->slice_num, true);

  // TODO: If we ever have more than one converter, pick the right handler here
  // TODO: irq_add_shared_handler(PWM_DEFAULT_IRQ_NUM(), converter_h1, 128);
  // TODO: irq_set_enabled(PWM_DEFAULT_IRQ_NUM(), true);

}

#endif