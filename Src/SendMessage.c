#include "SendMessage.h"
#include "config.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include "string.h"

extern UART_HandleTypeDef huart2;

void sendStaicMessage(uint8_t * text)
{
    if (text[0]!='\0')
    {
        HAL_UART_Transmit(&huart2, (uint8_t *) text, strlen(text), HAL_MAX_DELAY);
    }
}

void sendCharter(uint8_t character)
{
    uint8_t text[2];
    text[0] = character;
    text[1] = '\n';
    HAL_UART_Transmit(&huart2, (uint8_t *) text, strlen(text), HAL_MAX_DELAY);
}
