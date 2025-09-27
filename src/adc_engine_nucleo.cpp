#if ARDUINO_NUCLEO_G474RE

#include "adc_engine.h"
#include <cstdlib>

// This assumes we have an RP2350B (QFN-80 package)
#define PIN_BASE 40
#define PA 1
#define PB 2
#define PC 3
#define IB 4
#define UM 5

struct engine_data {
    // TODO
};
typedef struct engine_data engine_data_t;

adc_engine_t* adc_engine_create() {
    adc_engine_t* e = (adc_engine_t*) malloc(sizeof(adc_engine));
    engine_data_t* ed = (engine_data_t*) malloc(sizeof(engine_data_t));

    e->implementation_data = ed;

    e->sequence_length = 16;
    e->sequence = (int*) malloc(sizeof(int)*e->sequence_length);
    e->adc_commands = (uint32_t*) malloc(sizeof(uint32_t)*e->sequence_length);
    int sequence[] = { IB, PA, IB, PB, IB, PC, IB, UM};
    for(int i = 0; i < e->sequence_length; i ++) {
        e->sequence[i] = sequence[i % 8];
    }
    e->buf = (uint16_t*) malloc(sizeof(uint16_t)*e->sequence_length*2);

    return e;
}

int adc_engine_init(adc_engine_t* e) {

    engine_data_t *ed = (engine_data_t*) e->implementation_data;

    // TODO

    e->buf_write = NULL;

    e->cycles = 0;
    e->cycle_overlaps = 0;
    e->sync_losses = 0;

    return 0;
}

bool adc_engine_run(adc_engine_t* e) {
    engine_data_t *ed = (engine_data_t*) e->implementation_data;


    // Did previous runs finish?
    // TODO
    uint32_t control_counts = 0;
    uint32_t transfer_counts = 0;
    e->last_control_count = control_counts;
    e->last_transfer_count = transfer_counts;

    e->cycles++;

    if(control_counts != 0 || transfer_counts != 0) {
        e->cycle_overlaps++;
        if(control_counts + 1 != transfer_counts) e->sync_losses++;
        // Do not attempt to restart, this will only result in more mess.
        // Instead let the current transfer run out.
        return false;
    } 

    // Make sure that data will be transferred to the next buffer
    if(e->buf_write != NULL) {
        e->buf_read = e->buf_write;
        e->buf_write += e->sequence_length;
        if(e->buf_write >= e->buf + 2 * e->sequence_length) e->buf_write = e->buf;
    } else {
        e->buf_write = e->buf;
        e->buf_read = e->buf;
    }
    // TODO: Get DMA configured
    // Kick off the ADC by writing the first command
    // TODO: adc_hw->cs = e->adc_commands[0];

    return true;
}

void adc_engine_collect(adc_engine_t* e) {
    int v[] = {0, 0, 0, 0, 0, 0, 0, 0};
    int n[] = {0, 0, 0, 0, 0, 0, 0, 0};
    long i2 = 0;
    for(int i = 0; i < e->sequence_length; i++) {
        int c = e->sequence[i];
        int val = e->buf_read[i];
        // TODO: Check whether value is usable
        v[c] += val;
        n[c] += 1;
        if(c == IB) i2 += val*val; 
    }
    e->phases[0] = v[PA]/n[PA];
    e->phases[1] = v[PB]/n[PB];
    e->phases[2] = v[PC]/n[PC];
    e->i_bat = v[IB]/n[IB];
    e->i_bat2 = i2/n[IB];
    e->v_mot = v[UM]/n[UM];
}

#endif