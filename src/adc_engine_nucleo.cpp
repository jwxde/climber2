#if ARDUINO_NUCLEO_G474RE

#include "adc_engine.h"
#include <Arduino.h>
#include <cstdlib>

// General comments: 
// The InlineCurrentSense implementation for STM32 uses plain analogRead.
// So be using these directly here and using MyCurrentSense on top if this,
// we are off no worse than the default implementation.

#define PIN_BASE PIN_A0
#define PA 0
#define PB 1
#define PC 2
#define IB 3
#define UM 4

struct engine_data {
    uint16_t r[8];
};
typedef struct engine_data engine_data_t;

adc_engine_t* adc_engine_create() {
    adc_engine_t* e = (adc_engine_t*) malloc(sizeof(adc_engine));
    engine_data_t* ed = (engine_data_t*) malloc(sizeof(engine_data_t));

    e->implementation_data = ed;

    return e;
}

int adc_engine_init(adc_engine_t* e) {

    engine_data_t *ed = (engine_data_t*) e->implementation_data;

    pinMode(PIN_BASE + PA, INPUT);
    pinMode(PIN_BASE + PB, INPUT);
    pinMode(PIN_BASE + PC, INPUT);
    pinMode(PIN_BASE + IB, INPUT);
    pinMode(PIN_BASE + UM, INPUT);

    e->cycles = 0;
    e->cycle_overlaps = 0;
    e->sync_losses = 0;

    return 0;
}

bool adc_engine_run(adc_engine_t* e) {
    engine_data_t *ed = (engine_data_t*) e->implementation_data;

    ed->r[PA] = analogRead(PIN_BASE + PA);
    ed->r[PB] = analogRead(PIN_BASE + PB);
    ed->r[PC] = analogRead(PIN_BASE + PC);
    ed->r[IB] = analogRead(PIN_BASE + IB);
    ed->r[UM] = analogRead(PIN_BASE + UM);

    e->cycles++;

    return true;
}

void adc_engine_collect(adc_engine_t* e) {
    engine_data_t *ed = (engine_data_t*) e->implementation_data;
    uint16_t* v = ed->r;
    e->phases[0] = v[PA];
    e->phases[1] = v[PB];
    e->phases[2] = v[PC];
    e->i_bat = v[IB];
    e->i_bat2 = v[IB]*v[IB];
    e->v_mot = v[UM];
}

#endif