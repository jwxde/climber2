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
    // Nothing to do here. Or should we run some calibration?
    initialized = true;
    return 1;
}

static PhaseCurrent_s phase_currents;

PhaseCurrent_s MyCurrentSense::getPhaseCurrents() {
    phase_currents.a = (raw_values[pinA]*3.3/4096 - offset_ia) * gain_a;
    phase_currents.b = (raw_values[pinB]*3.3/4096 - offset_ib) * gain_b;
    phase_currents.c = (raw_values[pinC]*3.3/4096 - offset_ic) * gain_c;
    return phase_currents;
}