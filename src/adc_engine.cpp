#include "adc_engine.h"
#include <cstdlib>
#include <hardware/adc.h>
#include <hardware/dma.h>

// This assumes we have an RP2350B (QFN-80 package)
#define PIN_BASE 40
#define PA 1
#define PB 2
#define PC 3
#define IB 4
#define UM 5

adc_engine_t* adc_engine_create() {
    adc_engine_t* e = (adc_engine_t*) malloc(sizeof(adc_engine));

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
    // Set up pins to be used for ADC
    adc_gpio_init(PIN_BASE + PA);
    adc_gpio_init(PIN_BASE + PB);
    adc_gpio_init(PIN_BASE + PC);
    adc_gpio_init(PIN_BASE + IB);
    adc_gpio_init(PIN_BASE + UM);
    // init the adc hardware
    adc_init();

    adc_fifo_setup(
        true,              // Write each completed conversion to the sample FIFO
        true,              // Enable DMA data request (DREQ)
        1,                 // DREQ (and IRQ) asserted when all samples present
        true,              // Set the ERR bit
        false              // Don't hift each sample to 8 bits when pushing to FIFO
    );

    // Set up a DMA channel to transfer samples to the buffer

    e->dma_transfer_channel = dma_claim_unused_channel(true);
    e->dma_transfer_config = dma_channel_get_default_config(e->dma_transfer_channel);
    // we transfer 16 bit at a time
    channel_config_set_transfer_data_size(&e->dma_transfer_config, DMA_SIZE_16);
    // as we are reading from a FIFO, the source address never changes
    channel_config_set_read_increment(&e->dma_transfer_config, false);
    // but the target address does
    channel_config_set_write_increment(&e->dma_transfer_config, true);
    // and transfers are triggered by the ADC having written one sample to the FIFO
    channel_config_set_dreq(&e->dma_transfer_config, DREQ_ADC);

    // Set up a second DMA channel to configure the ADC for sampling
    // The ADC is triggered to act by writing a suitable value to its CS register.

    for(int i = 0; i < e->sequence_length; i++) {
        e->adc_commands[i] = (e->sequence[i] << ADC_CS_AINSEL_LSB) & ADC_CS_AINSEL_BITS | ADC_CS_START_ONCE_BITS | ADC_CS_ERR_STICKY_BITS | ADC_CS_EN_BITS;
    }

    e->dma_control_channel = dma_claim_unused_channel(true);
    e->dma_control_config = dma_channel_get_default_config(e->dma_control_channel);
    // The control register entries we transfer a 32 bit wide
    channel_config_set_transfer_data_size(&e->dma_control_config, DMA_SIZE_32);
    // For each transfer, we get the next value
    channel_config_set_read_increment(&e->dma_control_config, true);
    // All values get written to the same address
    channel_config_set_write_increment(&e->dma_control_config, false);
    // We start a new conversion as soon as the previous one is finished
    channel_config_set_dreq(&e->dma_control_config, DREQ_ADC);

    e->buf_write = NULL;

    e->cycles = 0;
    e->cycle_overlaps = 0;
    e->sync_losses = 0;

    return 0;
}

bool adc_engine_run(adc_engine_t* e) {

    // Did previous runs finish?
    uint32_t control_counts = dma_channel_hw_addr(e->dma_control_channel)->transfer_count;
    uint32_t transfer_counts = dma_channel_hw_addr(e->dma_transfer_channel)->transfer_count;
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
    dma_channel_configure(e->dma_transfer_channel,
        &e->dma_transfer_config, 
        e->buf_write,         // dest
        &adc_hw->fifo,  // source
        e->sequence_length,   // count
        true // start once the first DREQ is seen           
    );
    dma_channel_configure(e->dma_control_channel,
        &e->dma_control_config,
        &adc_hw->cs,
        e->adc_commands + 1, // The first command will be written manually below to kick off everything
        e->sequence_length -1,
        true // start once the first DREQ is seen
    );
    // Kick off the ADC by writing the first command
    adc_hw->cs = e->adc_commands[0];

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