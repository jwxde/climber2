#if STM32G474xx

#include "converter.h"
#include <cstdlib>
#include <Arduino.h>

static converter_t c1;

converter_t* converter_create(int pwm_pin, int enable_pin) {
  converter_t* converter = &c1;
  converter->pwm_pin = pwm_pin;
  // TODO
  #if false
  converter->slice_num = pwm_gpio_to_slice_num(converter->pwm_pin);
  converter->enable_pin = enable_pin;
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
  if(converter->control > 4.0) converter->control = 4.0;
  if(converter->control < 1.25) converter->control = 1.25;
  converter->level = 0x1000*(1.0 - 1.0/converter->control);
  // TODO pwm_set_chan_level(converter->slice_num, PWM_CHAN_A, converter->level);
}

void converter_set_state(converter_t *c, converter_state state) {
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
  // TODO: pwm_set_chan_level(c->slice_num, PWM_CHAN_B, c->state == converter_state::off ? 0 : 4096);
}

void converter_disable(converter_t* converter) {
  // TODO: gpio_set_outover(converter->pwm_pin, GPIO_OVERRIDE_HIGH);
  // TODO: gpio_set_outover(converter->enable_pin, GPIO_OVERRIDE_LOW);
}

void converter_enable(converter_t* converter) {
  // TODO: gpio_set_outover(converter->pwm_pin, GPIO_OVERRIDE_NORMAL);
  // TODO: gpio_set_outover(converter->enable_pin, GPIO_OVERRIDE_NORMAL);
}

void converter_init(converter_t* converter) {
  // It is crucial that we set up the PWM to a sane pulse width
  // (too much connection to ground will short our power source)
  // before enabling the half bridge by connecting the output pins
  
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