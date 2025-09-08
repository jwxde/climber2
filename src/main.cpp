#include <Arduino.h>
#include <SimpleFOC.h>
#include <MyCurrentSense.h>

#include "adc_engine.h"
#include "supply_sensor.h"

// For convverter

#include <hardware/pwm.h>

#include <current_sense/hardware_specific/rp2040/rp2040_mcu.h>

#define SIGNAL_PIN D22

// We construct these later so that we can debug output

BLDCDriver3PWM *driver;
BLDCMotor *motor;
MagneticSensorSPI *sensor;
MyCurrentSense *currentSense;
Commander *commander;

void doMotor(char* cmd) {
  commander->motor(motor, cmd);
}

float n_value;

void getN(char *cmd) {
  commander->scalar(&n_value, cmd);
}

int initFOC() {

  motor->torque_controller = TorqueControlType::foc_current;
  motor->voltage_limit = 12.0;
  motor->current_limit = 2.0;

  driver->voltage_power_supply = 12;
  driver->pwm_frequency = 20000;
  // initialize the driver
  driver->init();
  // link the motor and the driver
  motor->linkDriver(driver);
  
  // initialize the sensor
  // go a bit slower because of our messy wiring
  sensor->clock_speed = 500000;
  sensor->init();
  // link the motor and the sensor
  motor->linkSensor(sensor);

  // Don't link the driver before initialization, this might mess up zero setting  
  if(!currentSense->init()) {
    Serial.println("Current sense init error\n");
    return 0;
  }
  currentSense->linkDriver(driver);
  motor->linkCurrentSense(currentSense);

  // initialize the motor
  if(!motor->init()) {
    Serial.println("Motor init error\n");
    return 0;
  }

  // set motion control loop to be used
  motor->target = 0;
  motor->controller = MotionControlType::torque;
  motor->torque_controller = TorqueControlType::foc_current;

  motor->useMonitoring(Serial);
  motor->monitor_downsample = 5000; // set downsampling can be even more > 100
  motor->monitor_variables =  _MON_TARGET | _MON_VEL | _MON_VOLT_Q | _MON_VOLT_D | _MON_CURR_Q | _MON_CURR_D | _MON_ANGLE;
  // set monitoring of d and q currents _MON_TARGET | _MON_VEL | _MON_ANGLE |
  motor->monitor_decimals = 2; //!< monitor outputs decimal places
  
  motor->target = 0;

  // start motor FOC algorithm
  if(!motor->initFOC()) {
    Serial.println("FOC init error\n");
    return 0;
  }

  return 1;
}

int initOk = 0;

enum converter_state { off, consuming, charging};
struct converter {
  pin_size_t pwm_pin;
  pin_size_t enable_pin;
  int slice_num;
  converter_state state;
  float control;
  int level;
  float last_voltage_reached;
  float last_i;
  float last_i2;
};
typedef struct converter converter_t;

converter_t* converter_create() {
  converter_t* converter = (converter_t*) malloc(sizeof(converter_t));
  converter->pwm_pin = D0; // PWM 0A
  converter->slice_num = pwm_gpio_to_slice_num(converter->pwm_pin);
  converter->enable_pin = D1;
  return converter;
}

/**
 * We want the control signal to be the inverse duty ratio.
 * When consuming, we use an inverted PWM output so that our PWM signal starts with a low phase.
 * Hence to get to duty ratio > 0.5 we need to put the level below 0.5*range.
 * When charging, we use a non inverted PWM output so that our PWM signal starts with a high phase.
 * Hence to get a duty ration > 0.5 wee need to put the level above 0.5*range.
 */
void converter_apply(converter_t *converter) {
  if(converter->control > 4.0) converter->control = 4.0;
  if(converter->control < 1.25) converter->control = 1.25;
  switch(converter->state) {
    case converter_state::consuming:
      converter->level = 0x1000*(1.0 - 1.0/converter->control); break;
    case converter_state::charging:
    case converter_state::off:
      converter->level = 0x1000*(1.0/converter->control);
  }
  pwm_set_chan_level(converter->slice_num, PWM_CHAN_A, converter->level);
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
          pwm_set_output_polarity(c->slice_num, true, false);
          c->state = consuming;
          break;
        case charging:
          pwm_set_output_polarity(c->slice_num, false, false);
          c->state = charging;
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
  pwm_set_chan_level(c->slice_num, PWM_CHAN_B, c->state == converter_state::off ? 0 : 4096);
}

void converter_disable(converter_t* converter) {
  gpio_set_outover(converter->pwm_pin, GPIO_OVERRIDE_HIGH);
  gpio_set_outover(converter->enable_pin, GPIO_OVERRIDE_LOW);
}

void converter_enable(converter_t* converter) {
  gpio_set_outover(converter->pwm_pin, GPIO_OVERRIDE_NORMAL);
  gpio_set_outover(converter->enable_pin, GPIO_OVERRIDE_NORMAL);
}

void converter_init(converter_t* converter) {
  // It is crucial that we set up the PWM to a sane pulse width
  // (too much connection to ground will short our power source)
  // before enabling the half bridge by connecting the output pins
  
  // In order to eliminate all risk, we configure pull ups/downs
  // the way we want the default signal to be (half bridge not enabled, hi side connected)
  // and disable output. We need to set the enablement state after setting the pin function as
  // the pin function also sets the enablement state.
  gpio_pull_up(converter->pwm_pin);
  gpio_pull_down(converter->enable_pin);
  gpio_set_function(converter->pwm_pin, GPIO_FUNC_PWM);
  gpio_set_function(converter->enable_pin, GPIO_FUNC_PWM);
  converter_disable(converter);
 
  // Don't invert the enable pin signal so that we get an off signal
  // by default (for level 0).
  pwm_set_output_polarity(converter->slice_num, false, false);
  
  pwm_set_clkdiv_mode(converter->slice_num, PWM_DIV_FREE_RUNNING);
  // This will result in roughly 20kHz frequency
  pwm_set_wrap(converter->slice_num, 4095);
  pwm_set_clkdiv_int_frac4(converter->slice_num, 2, 0);
  pwm_set_phase_correct(converter->slice_num, false);

  // Start out with 50% duty cycle so that we get a voltage that is about twice the battery voltage.
  // But do not enable the converter yet.
  converter->control = 2.0;
  converter_apply(converter);
  converter_set_state(converter, off);

  // Kick off the PWM slice. The SimpleFOC PWM driver will later (re) enable all slices.
  pwm_set_enabled(converter->slice_num, true);

  // And let the signal show up on the output pins
  converter_enable(converter);
}

adc_engine_t* adc_engine;
converter_t* converter;
supply_sensor_t* supply_sensor;

void converterInfo(char *cmd) {
  if (strlen(cmd) > 0) {
    switch(cmd[0]) {
      case 'C': Serial.println(supply_sensor->i_battery); break;
      case 'U': Serial.println(supply_sensor->v_motor); break;
      default: Serial.print("Unknown converter command"); Serial.println(cmd); break;
    }
  } else {
    Serial.println("Converter needs a subcommand");
  }
}

void command_resistance_level(char *cmd);
void command_current_scale(char *cmd) { commander->scalar(&supply_sensor->i_battery_scale, cmd); }
void command_velocity_pid(char *cmd) { commander->pid(&motor->PID_velocity, cmd); }
void command_angle_pid(char *cmd) { commander->pid(&motor->P_angle, cmd); }

float angle_0 = 0;

// For each cycle of the converter PWM, we kick of a new ADC run.
// And we collect the data that resulted from the previous run.

void kickOffAdcConversion() {
  // TODO: Check that the previous run finished as expected.
  if(pwm_get_irq_status_mask() && (0x1 << converter->slice_num)) {
    pwm_clear_irq(converter->slice_num);
    adc_engine_run(adc_engine);
    adc_engine_collect(adc_engine);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) { sleep_ms(100); }

  Serial.printf("hi there\n");

  // Initialize SPI to fit our needs before FOC code does it
  // we are using SPI unit 1 on the RP2350 which confusingly is called SPI instead of SPI1.
  // Presumably because SPI unit 0 is attached to the ISP header.
  // See .platformio/packages/framework-arduinopico/variants/adafruit_metro_rp2350/pins_arduino.h

  SPI.setRX(D8); // MISO
  SPI.setCS(D9); // CS
  SPI.setSCK(D10); // SCK
  SPI.setTX(D11); // MOSI
  // Does SimpleFOC take care of handling CS?
  SPI.begin();

  // Set up cycle indicator signal

  gpio_init(SIGNAL_PIN);
  gpio_set_dir(SIGNAL_PIN, GPIO_OUT);

  // Set up power converter
  // On Adafruit Metro, be sure to put the RX/TX switch to RX=0, TX=1

  converter = converter_create();

  converter_init(converter);

  // Start out with the converter going full throttle
  // TODO: Change
  converter_set_state(converter, consuming);

  // Setup ADC engine
  adc_engine = adc_engine_create();
  adc_engine_init(adc_engine);

  supply_sensor = supply_sensor_create(&adc_engine->i_bat, &adc_engine->i_bat2, &adc_engine->v_mot);

  SimpleFOCDebug::enable(&Serial);

  // TODO: Do we need Arduino pin numbers here or can we use the GPIO numbers directly?
  // These pin numbers go into Arduino functions like digitalWrite, so they need to be Arduino pin numbers.
  // But translation is not necessary for RP2350.
  driver = new BLDCDriver3PWM(D2, D4, D6, D3, D5, D7);
  // Our motor has Kv about 50 rpm/V and L about 150 uH. Resistance wire to wire is 0.6 Ohms.
  motor = new BLDCMotor(6, 0.6, 50, 0.000150);
  sensor = new MagneticSensorSPI(AS5048_SPI, D9);
  // We have ACS712 hall sensors which we sample with 8 bit resolution.
  // We run the sensors at 3.3 V and have the 712 20 T type which has nominally 100 mV/A at 5 V.
  currentSense = new MyCurrentSense(adc_engine->phases, 10.0*5.0/3.3, 3.3/2);

  // Make sure the ADC engine runs AND collects input before going into FOC intiailization.
  pwm_clear_irq(converter->slice_num);
  pwm_set_irq_enabled(converter->slice_num, true);

  irq_add_shared_handler(PWM_DEFAULT_IRQ_NUM(), kickOffAdcConversion, 128);
  irq_set_enabled(PWM_DEFAULT_IRQ_NUM(), true);

  // Wait for current sensing to work
  for(int i = 0; i < 1000; i++) {
    Serial.printf("Engine health: cycles=%d cycle_overlaps=%d sync_losses=%d", adc_engine->cycles, adc_engine->cycle_overlaps, adc_engine->sync_losses);
    Serial.println();
    Serial.printf("Engine health: last_control_count=%d last_transfer_count=%d", adc_engine->last_control_count, adc_engine->last_transfer_count);
    Serial.println();
    Serial.printf("DMA transfer count: control=%d, transfer=%d", 
      dma_channel_hw_addr(adc_engine->dma_control_channel)->transfer_count,
      dma_channel_hw_addr(adc_engine->dma_transfer_channel)->transfer_count);
    Serial.println();
    supply_sensor_update(supply_sensor);
    Serial.printf("Battery current: %d or %f", adc_engine->i_bat, supply_sensor->i_battery);
    Serial.println();
    Serial.printf("Phase current A: %d or %f", adc_engine->phases[0], currentSense->getPhaseCurrents().a);
    Serial.println();
    busy_wait_ms(1);
  }

  do {
    initOk = initFOC();
  } while(Serial && !initOk);

  commander = new Commander(Serial, '\r');
  commander->add('M', doMotor, "motor");
  commander->add('V', command_velocity_pid, "FOC velocity PID");
  commander->add('A', command_angle_pid, "FOC angle PID");
  commander->add('N', getN, "loop");
  commander->add('C', converterInfo, "converter");
  commander->add('R', command_resistance_level, "training resistance");
  commander->add('I', command_current_scale, "Current scale");

  // Quick round of FOC without any action to get the angle calibrated
  motor->enable();
  motor->controller = MotionControlType::torque;
  motor->target = 0;
  for(int n = 0; n < 1000; n++) {
    motor->loopFOC();
    motor->move();
  }
  angle_0 = motor->shaft_angle;
  motor->disable();

  if(false) {
  // Shut down converter to find battery current 0 point
  converter_set_state(converter, off);
  for(int n = 0; n < 100; n++) {
    supply_sensor_update(supply_sensor);
    sleep_ms(1);
  }
  supply_sensor->i_battery_offset = supply_sensor->i_battery_raw;
  Serial.printf("Set battery current measurement offset to %f V\n", supply_sensor->i_battery_offset);
  converter_set_state
(converter, consuming);

  // Second round of FOC to find battery current polarity
  // Create some power consumption by 
  motor->controller = MotionControlType::angle;
  motor->target = angle_0;
  for(int n = 0; n < 1000; n++) {
    motor->loopFOC();
    motor->move();
    supply_sensor_update(supply_sensor);
    if(supply_sensor->i_battery < 0) {
      supply_sensor->i_battery_scale = -1.0 * supply_sensor->i_battery_scale;
      Serial.println("Flipped sign of battery current\n");
    }
  }
  }
  motor->disable();

  motor->controller = MotionControlType::angle;

  switch(motor->controller) {
    case MotionControlType::angle:
      motor->target = angle_0;
      motor->P_angle.limit = 150;
      motor->velocity_limit = 150;
      // Torque limit
      motor->PID_velocity.limit = 0.5;
      break;
    default:
      motor->target = 0;
  }

  motor->enable();

  Serial.printf("--- Setup finished ---");
  Serial.println();
}

int n = 0;

void my_monitor() {
  static int monitor_cnt = 0;

  if( !motor->monitor_downsample || monitor_cnt++ < (motor->monitor_downsample-1) ) return;
  monitor_cnt = 0;

  Serial.printf("%7.2f %7.2f %7.2f ", motor->target, motor->shaft_velocity, motor->shaft_angle);
  Serial.printf("%7.2f %7.2f ", motor->voltage.d, motor->voltage.q);
  Serial.printf("%7.0f %7.0f ", motor->current.q * 1000, motor->current.d * 1000);
  Serial.printf("%9.4f %9.4f %7.2f ", supply_sensor->i_battery, supply_sensor->i_battery_variance, supply_sensor->v_motor);
  Serial.printf("%9.4f %9.4f %7.2f %d ", converter->last_i, converter->last_i2, converter->last_voltage_reached, converter->level);
  Serial.println();
}

/**
 * PID controller for making sure that the motor
 * voltage stays on target.
 *
 * What should the parameters be?
 * Our error signal is the target voltage of 27 Volts minus the measured voltage.
 * What should we use as control value? Let U2 be the higher voltage.
 * Then U1 = duty_ration * U2. U1 is mostly constant but unknown in detail.
 * The PID will have an easier job if the control signal is in a mostly linear relationship with the
 * error. As U2 = 1/duty_ratio * U1, we pick 1/duty_ratio as control value.
 * And we take into account that a PID controller wants to control something that is related
 * to the rate of change of the error, not directly to the error.
 * So we let the PID controller change the rate of change of 1/duty_ratio.
 * When our voltage U2 is too low (error positive),
 * we need to decrease the duty ratio hence increase the control value. So our P needs to be positive.
 * Assuming a U1 of 14V, a 1V increase in U2 roughly corresponds to going from 2 to 2.05.
 * We probably want to make smaller adjustment steps than that, so a plausible pick for P would be 0.02.
 * Our limit for the step size would be of the same order of magnitude.
 * We don't care much about the range of change of the rate of change of the control variable.
 */ 
PIDController pid_v_motor = PIDController(0.02, 0.0, 0.0, 1.0, 0.05);

/**
 * PID controller for the target torque (we want to avoid abrupt changes)
 */
PIDController pid_torque_sp = PIDController(1.0, 0.0, 0.0, 20.0, 10.0);

float resistance_level = 1.0;

void command_resistance_level(char *cmd) {
  commander->scalar(&resistance_level, cmd);
}

LowPassFilter converter_triggered(1.0/5000);

void loop() {

  gpio_put(SIGNAL_PIN, true);
  supply_sensor_update(supply_sensor);

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

  #if true
  // Simple hysterisis loop to prevent idling of the converter. After we decide to
  // switch on ("trigger") the converter, we want to give the current some time to
  // develop to prevent immediate shutdown. So we use a low pass filtered activation
  // signal that decays over about 4 PWM cycles.
  //
  // TODO: How would we detect that our duty cycle needs adjustment? Wouldn't it have
  // to depend on the input (battery) voltage?
  float triggered = 0.0;
  if(converter->state == off) {
    if (supply_sensor->v_motor < 26.0) {
      converter_set_state(converter, consuming);
      triggered = 1.0;
    }
    if(supply_sensor->v_motor > 29.0) {
      converter_set_state(converter, charging);
      triggered = 1.0;
    }
  } else {
    if(converter_triggered(triggered) < 0.2) {
      // Converter is on and had enough time to develop a current pattern
      if(abs(supply_sensor->i_battery) < 2.0) {
        converter_set_state(converter, off);
        // Take note of the voltage we reached and adjust accordingly
        // In case there is no current flow through the coil, assume a power failure
        // and choose a safe starting point for power return instead of adjusting
        if(supply_sensor->i_battery_variance > 2.0) {
          converter->last_voltage_reached = supply_sensor->v_motor;
          converter->last_i = supply_sensor->i_battery;
          converter->last_i2 = supply_sensor->i_battery_variance;
          converter->control += pid_v_motor(27.0 - converter->last_voltage_reached);
        } else {
          converter->control = 2;
        }
        // Next call is probably redundant, will happen once we switch the converter on
        converter_apply(converter);
      }
    }
  }    
  #endif

  gpio_put(SIGNAL_PIN, false);

  if(initOk) {

    motor->loopFOC();

    if(resistance_level > 0) {
      switch(motor->controller) {
        case MotionControlType::torque:
          if(motor->shaft_angle < angle_0) {
            float torque_sp = -0.5;
            if(supply_sensor->i_battery < -0.2) torque_sp = -0.5 * resistance_level;
            motor->target = pid_torque_sp(torque_sp);
          }
          else {
            motor->target = 0;
          }
          break;
        case MotionControlType::angle:
          if(motor->shaft_angle < angle_0 - 0.1 && motor->shaft_velocity < -0.2) 
            motor->PID_velocity.limit = max(0.6, 2 * resistance_level);
          else {
            motor->PID_velocity.limit = 0.6;
          }
        default:
          break;
      }
    }
   
    motor->move();

  }
  my_monitor();
  commander->run();
  n++;
  n_value = n*1.0;
}