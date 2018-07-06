#include "init.h"
#include "global_variables.h"
#include "stm32f1xx_hal.h"
/*
    Init global variables
*/
    struct vector placement = {0.0,0.0,0.0,0.0};
    struct vector location = {0.0,0.0,0.0,0.0};
    unsigned char current_temperature = 0;
    unsigned char current_temperature_bed = 0;
    uint8_t state0 = 0;

