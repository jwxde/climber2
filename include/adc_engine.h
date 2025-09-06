#include <hardware/dma.h>

struct adc_engine {
    int sequence_length;
    int* sequence;
    uint32_t* adc_commands;
    uint16_t* buf;
    uint16_t* buf_read;
    uint16_t* buf_write;
    // DMA channels
    int dma_transfer_channel;
    dma_channel_config_t dma_transfer_config;
    int dma_control_channel;
    dma_channel_config_t dma_control_config;
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
};

typedef struct adc_engine adc_engine_t;

adc_engine_t* adc_engine_create();
int adc_engine_init(adc_engine_t* engine);
void adc_engine_run(adc_engine_t* engine);
// TODO: Should this really be here?
void adc_engine_collect(adc_engine_t* engine);
