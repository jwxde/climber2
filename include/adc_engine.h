#include <inttypes.h>

struct adc_engine {
    int sequence_length;
    int* sequence;
    uint32_t* adc_commands;
    uint16_t* buf;
    uint16_t* buf_read;
    uint16_t* buf_write;
    // Engine health indicators
    uint32_t cycles;
    uint32_t cycle_overlaps;
    uint32_t sync_losses;
    int last_control_count;
    int last_transfer_count;
    // Output variables
    // TODO: Might be somewhere else
    int phases[3];
    int i_bat;
    long i_bat2;
    int v_mot;
    void* implementation_data;
};

typedef struct adc_engine adc_engine_t;

adc_engine_t* adc_engine_create();
int adc_engine_init(adc_engine_t* engine);
bool adc_engine_run(adc_engine_t* engine);
// TODO: Should this really be here?
void adc_engine_collect(adc_engine_t* engine);