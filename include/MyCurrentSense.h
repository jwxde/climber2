#include <common/base_classes/CurrentSense.h>

class MyCurrentSense: public CurrentSense {
public:
    MyCurrentSense(int* input, float gain, float offset);
    // CurrentSense interface implementing functions 
    int init() override;
    PhaseCurrent_s getPhaseCurrents() override;
private:
    int* raw_values;
};