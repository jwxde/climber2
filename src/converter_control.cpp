#include "converter_control.h"
#include <cstdlib>

converter_control_t* converter_control_create() {
    converter_control_t* c = (converter_control_t*) malloc(sizeof(converter_control_t));
    return c;
}
