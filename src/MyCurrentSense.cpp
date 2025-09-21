#include "MyCurrentSense.h"

MyCurrentSense::MyCurrentSense(int* input, float gain, float offset) {
    raw_values = input;
    // Need to set up variables of the base class for converting raw values to final values.
    // Also need to set up "pins". The expectation of the base class is that swapping these
    // values results in changed value assignments to the components of the return value
    // of getPhaseCurrents. The expectation is also that flipping the sign of a gain
    // variable inverts the signal.
    gain_a = gain; gain_b = gain; gain_c = gain;
    offset_ia = offset; offset_ib = offset; offset_ic = offset;
    // We don't use physical pins here but offsets into raw_values.
    pinA = 0; pinB = 1; pinC = 2;
}

int MyCurrentSense::init() {

    if(driver) {
        // Be sure the driver is off so we don't see any currents
        driver->disable();
        // Wait a bit to let the system settle
        busy_wait_ms(5);
        // We have a new measurements every PWM cycle so about every 50 micro seconds.
        // Let's average over 1000 measurements
        offset_ia = 0;
        offset_ib = 0;
        offset_ic = 0;
        for(int i = 0; i < 1000; i++) {
            offset_ia += raw_values[pinA];
            offset_ib += raw_values[pinB];
            offset_ic += raw_values[pinC];
            busy_wait_us(50);
        }
        offset_ia = offset_ia*3.3/4096/1000;
        offset_ib = offset_ib*3.3/4096/1000;
        offset_ic = offset_ic*3.3/4096/1000;
        // Leave the driver off, it will be enabled later anyways
    }

    initialized = true;
    return 1;
}

static PhaseCurrent_s phase_currents;

PhaseCurrent_s MyCurrentSense::getPhaseCurrents() {
    phase_currents.a = (raw_values[pinA]*3.3/4096 - offset_ia) * gain_a;
    phase_currents.b = (raw_values[pinB]*3.3/4096 - offset_ib) * gain_b;
    phase_currents.c = (raw_values[pinC]*3.3/4096 - offset_ic) * gain_c;
    i2 = phase_currents.a*phase_currents.a + phase_currents.b*phase_currents.b + phase_currents.c*phase_currents.c;
    return phase_currents;
}