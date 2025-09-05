#include <Arduino.h>
#include <SimpleFOC.h>

#include "supply_sensor.h"

// For convverter

#include <hardware/pwm.h>

#include <current_sense/hardware_specific/rp2040/rp2040_mcu.h>

#define SIGNAL_PIN D22

// We construct these later so that we can debug output

BLDCDriver3PWM *driver;
BLDCMotor *motor;
MagneticSensorSPI *sensor;
InlineCurrentSense *currentSense;
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

struct converter {
  pin_size_t pwm_pin;
  pin_size_t enable_pin;
  int slice_num;
  bool enabled;
};
typedef struct converter converter_t;

void converter_enable(converter_t *c, boolean enabled) {
  c->enabled = enabled;
  gpio_put(c->enable_pin, c->enabled);
}

extern RP2040ADCEngine engine; 


converter_t *converter;
supply_sensor_t *supply_sensor;

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

  converter = (converter_t*) malloc(sizeof(converter_t));
  converter->pwm_pin = D0; // PWM 0A
  converter->slice_num = pwm_gpio_to_slice_num(converter->pwm_pin);
  converter->enable_pin = D1;

  // It is crucial that we set up the PWM to a sane pulse width
  // (too much connection to ground will short our power source)
  // before enabling the half bridge

  // We use PICO SDK functions for now
  gpio_init(converter->enable_pin);
  gpio_set_dir(converter->enable_pin, GPIO_OUT);
  // Make sure the half bridge is not enabled
  gpio_put(converter->enable_pin, false);
  
  gpio_set_function(converter->pwm_pin, GPIO_FUNC_PWM);
  // Hold back while configuring
  pwm_set_enabled(converter->slice_num, false);
  // We want the output high by default (which is the safe way)
  // so that as long as the counters don't run, we don't short our power supply
  pwm_set_output_polarity(converter->slice_num, false, false);
  
  pwm_set_clkdiv_mode(converter->slice_num, PWM_DIV_FREE_RUNNING);
  // This will result in roughly 20kHz frequency
  pwm_set_wrap(converter->slice_num, 4095);
  pwm_set_clkdiv_int_frac4(converter->slice_num, 2, 0);
  pwm_set_phase_correct(converter->slice_num, false);

  // Start out with 50% duty cycle so that we get a voltage that is about twice the battery voltage
  pwm_set_chan_level(converter->slice_num, PWM_CHAN_A, 2048);
  // The SimpleFOC PWM driver will later (re) enable all slices.
  pwm_set_enabled(converter->slice_num, true);
  converter_enable(converter, true);

  // We also want to use some of the ADC channels.
  // As the currentsense module hogs the complete ADC infra structure, our only chance
  // is to register our channels of interest before the currentsense instance
  // gets initialized. The second initialization will then be a noop.
  // We need to also add the pins that the currentsense module is supposed to use.

  engine.addPin(A0);
  engine.addPin(A1);
  engine.addPin(A2);
  engine.addPin(A3);
  engine.addPin(A4);

  engine.init();
  // The actual engine start will be done by the inline currentsense instance

  supply_sensor = supply_sensor_create();

  SimpleFOCDebug::enable(&Serial);

  // TODO: Do we need Arduino pin numbers here or can we use the GPIO numbers directly?
  // These pin numbers go into Arduino functions like digitalWrite, so they need to be Arduino pin numbers.
  // But translation is not necessary for RP2350.
  driver = new BLDCDriver3PWM(D2, D4, D6, D3, D5, D7);
  // Our motor has Kv about 50 rpm/V and L about 150 uH. Resistance wire to wire is 0.6 Ohms.
  motor = new BLDCMotor(6, 0.6, 50, 0.000150);
  sensor = new MagneticSensorSPI(AS5048_SPI, D9);
  // We have ACS712 hall sensors, so we just specify the mV per Amp ratio.
  // We run the sensors at 3.3 V and have the 712 20 T type which has nominally 100 mV/A at 5 V.
  currentSense = new InlineCurrentSense(100, A0, A1, A2);

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
  converter_enable(converter, false);
  for(int n = 0; n < 100; n++) {
    supply_sensor_update(supply_sensor);
    sleep_ms(1);
  }
  supply_sensor->i_battery_offset = supply_sensor->i_battery_raw;
  Serial.printf("Set battery current measurement offset to %f V\n", supply_sensor->i_battery_offset);
  converter_enable(converter, true);

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
  Serial.printf("%9.4f %9.4f %7.2f", supply_sensor->i_battery, sqrt(supply_sensor->i_battery_variance), supply_sensor->v_motor);
  Serial.println();
}

/**
 * PID controller for making sure that the motor
 * voltage stays on target.
 *
 * What should the parameters be?
 * Our error signal is measured voltage minus target voltage of 27 Volts.
 * When our voltage is too high, we need to increase the control value. So our P needs to be positive.
 * We want to be able to drop or raise the voltage in 100 msecs. So our rate limit needs to be about 15000.
 * In theory, motor voltage is battery voltage over duty cycle. So the inverse duty cycle would be a
 * good control variable. Our limits would be 1.2 and about 2.8 with a center around 2 (for a 3S battery at 15V).
 * So lets take a limit of 0.8 to either side. We want to be able to swing in about 100 msecs.
 * So the ramp would be about 10. By design of the control variable, our P should be 1.0.
 * Not sure about the I and D parts. Let's just try.
 */ 
PIDController pid_v_motor = PIDController(1.0, 0.0, 0.0, 10.0, 0.8);

/**
 * PID controller for the target torque (we want to avoid abrupt changes)
 */
PIDController pid_torque_sp = PIDController(1.0, 0.0, 0.0, 20.0, 10.0);

float resistance_level = 1.0;

void command_resistance_level(char *cmd) {
  commander->scalar(&resistance_level, cmd);
}

void loop() {
  // For now, just try to check the sensors are working ok.
  if(initOk) {
    gpio_put(SIGNAL_PIN, true);

    motor->loopFOC();
    supply_sensor_update(supply_sensor);

    // Catch some power issues
    if(false && supply_sensor->v_motor > 33.0 || supply_sensor->v_motor < 10.0 && supply_sensor->i_battery > 1) {
      // Our regulation seems to fail, shut down motor
      motor->disable();
      Serial.println("Over or under voltage, motor disabled, shutdown");
      initOk = false;
    }
    if(abs(supply_sensor->i_battery) > 12.0) {
      motor->disable();
      converter_enable(converter, false);
      Serial.println("Over current, motor and converter disabled");
      initOk = false;
    }

    float inv_converter_duty = 2 + pid_v_motor(27.0 - supply_sensor->v_motor);
    int converter_level = 4096/inv_converter_duty;
    // Limit again for safety -- the critical part is the low end so that we
    // don't start putting too much current through the coil.
    if(converter_level < 500) converter_level = 500;
    if(converter_level > 4000) converter_level = 4000;
    pwm_set_chan_level(converter->slice_num, PWM_CHAN_A, converter_level);

    // Avoid idling the converter, i.e. having it shuffle current back and forth without significant
    // energy transfer (this leads to energy loss due to the resistance losses).
    bool low_efficiency = converter->enabled && supply_sensor->i_battery_variance > supply_sensor->i_battery * supply_sensor->i_battery;
    if(supply_sensor->v_motor < 18 || supply_sensor->v_motor > 29) {
      converter_enable(converter, true);
    } else {
      converter_enable(converter, !low_efficiency);
    };

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
   
    gpio_put(SIGNAL_PIN, false);

    motor->move();

    my_monitor();

    commander->run();
    n++;
    n_value = n*1.0;
  }
}