#include "CRC.h"
#include "stm32f1xx_hal.h"
#include "config.h"


uint8_t checkCRC(uint8_t *input_buffer)
{
    uint16_t ourCRC = 0;
    int i = 0;
    while (input_buffer[i] != '*'){
        ourCRC = (ourCRC << 3) + (uint16_t) input_buffer[i];
        ourCRC = (ourCRC << 3) + (uint16_t) input_buffer[i];
        ourCRC = (ourCRC ^ (ourCRC >> 8));
        i++;
        if (i == BUFFER_LENGTH) return 0;
    }
    i++;
    if (input_buffer[i] == (uint8_t)ourCRC){
        return 1;
    }else{
        return 0;
    }
}
