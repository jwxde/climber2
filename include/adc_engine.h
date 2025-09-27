#include <inttypes.h>

struct adc_engine {
    // Engine health indicators
    uint32_t cycles;
    uint32_t cycle_overlaps;
    uint32_t sync_losses;
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
/**
 * Trigger a sample run of the ADC engine. The run is expected to
 * take about one PWM period.
 * The expected outcome is that successfully triggering a new run,
 * fresh aggregate results can be collected by calling adc_engine_collect.
 * The function will return true if a new run has really been triggered.
 * This will happen only if the old run has finished.
 * If a new run has been triggered, cycles will be increased by one.
 * If not, cycle_overlaps will be increased.
 * The field sync_losses will be increased if there is reason to believe
 * that the computation of aggregate results will lead to incorrect values
 * because internal bookkeeping of which reading belongs to which channel
 * is out of sync.
 */
bool adc_engine_run(adc_engine_t* engine);
/**
 * Compute (raw) aggregate results from the the results of the previous
 * rund and store them in phases[3], i_bat, i_bat2 and v_mot.
 */
void adc_engine_collect(adc_engine_t* engine);