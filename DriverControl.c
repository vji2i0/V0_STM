#include "DriverControl.h"
#include "config.h"
#include "stm32f1xx_hal.h"

void DriverPhaseX(char phase)
{
    switch (phase)
    {
        case 0b10000000:
        {
            HAL_GPIO_WritePin(DRIVER_X1_PORT, DRIVER_X1_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_X2_PORT, DRIVER_X2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_X3_PORT, DRIVER_X3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_X4_PORT, DRIVER_X4_PIN, RESET);
            break;
        }
        case 0b11000000:
        {
            HAL_GPIO_WritePin(DRIVER_X1_PORT, DRIVER_X1_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_X2_PORT, DRIVER_X2_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_X3_PORT, DRIVER_X3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_X4_PORT, DRIVER_X4_PIN, RESET);
            break;
        }
        case 0b01000000:
        {
            HAL_GPIO_WritePin(DRIVER_X1_PORT, DRIVER_X1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_X2_PORT, DRIVER_X2_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_X3_PORT, DRIVER_X3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_X4_PORT, DRIVER_X4_PIN, RESET);
            break;
        }
        case 0b01100000:
        {
            HAL_GPIO_WritePin(DRIVER_X1_PORT, DRIVER_X1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_X2_PORT, DRIVER_X2_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_X3_PORT, DRIVER_X3_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_X4_PORT, DRIVER_X4_PIN, RESET);
            break;
        }
        case 0b00100000:
        {
            HAL_GPIO_WritePin(DRIVER_X1_PORT, DRIVER_X1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_X2_PORT, DRIVER_X2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_X3_PORT, DRIVER_X3_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_X4_PORT, DRIVER_X4_PIN, RESET);
            break;
        }
        case 0b00110000:
        {
            HAL_GPIO_WritePin(DRIVER_X1_PORT, DRIVER_X1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_X2_PORT, DRIVER_X2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_X3_PORT, DRIVER_X3_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_X4_PORT, DRIVER_X4_PIN, SET);
            break;
        }
        case 0b00010000:
        {
            HAL_GPIO_WritePin(DRIVER_X1_PORT, DRIVER_X1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_X2_PORT, DRIVER_X2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_X3_PORT, DRIVER_X3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_X4_PORT, DRIVER_X4_PIN, SET);
            break;
        }
        case 0b10010000:
        {
            HAL_GPIO_WritePin(DRIVER_X1_PORT, DRIVER_X1_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_X2_PORT, DRIVER_X2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_X3_PORT, DRIVER_X3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_X4_PORT, DRIVER_X4_PIN, SET);
            break;
        }
    }
}

void DriverPhaseY(char phase)
{
    switch (phase)
    {
        case 0b00001000:
        {
            HAL_GPIO_WritePin(DRIVER_Y1_PORT, DRIVER_Y1_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_Y2_PORT, DRIVER_Y2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Y3_PORT, DRIVER_Y3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Y4_PORT, DRIVER_Y4_PIN, RESET);
            break;
        }
        case 0b00001100:
        {
            HAL_GPIO_WritePin(DRIVER_Y1_PORT, DRIVER_Y1_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_Y2_PORT, DRIVER_Y2_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_Y3_PORT, DRIVER_Y3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Y4_PORT, DRIVER_Y4_PIN, RESET);
            break;
        }
        case 0b00000100:
        {
            HAL_GPIO_WritePin(DRIVER_Y1_PORT, DRIVER_Y1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Y2_PORT, DRIVER_Y2_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_Y3_PORT, DRIVER_Y3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Y4_PORT, DRIVER_Y4_PIN, RESET);
            break;
        }
        case 0b00000110:
        {
            HAL_GPIO_WritePin(DRIVER_Y1_PORT, DRIVER_Y1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Y2_PORT, DRIVER_Y2_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_Y3_PORT, DRIVER_Y3_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_Y4_PORT, DRIVER_Y4_PIN, RESET);
            break;
        }
        case 0b00000010:
        {
            HAL_GPIO_WritePin(DRIVER_Y1_PORT, DRIVER_Y1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Y2_PORT, DRIVER_Y2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Y3_PORT, DRIVER_Y3_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_Y4_PORT, DRIVER_Y4_PIN, RESET);
            break;
        }
        case 0b00000011:
        {
            HAL_GPIO_WritePin(DRIVER_Y1_PORT, DRIVER_Y1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Y2_PORT, DRIVER_Y2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Y3_PORT, DRIVER_Y3_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_Y4_PORT, DRIVER_Y4_PIN, SET);
            break;
        }
        case 0b00000001:
        {
            HAL_GPIO_WritePin(DRIVER_Y1_PORT, DRIVER_Y1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Y2_PORT, DRIVER_Y2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Y3_PORT, DRIVER_Y3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Y4_PORT, DRIVER_Y4_PIN, SET);
            break;
        }
        case 0b00001001:
        {
            HAL_GPIO_WritePin(DRIVER_Y1_PORT, DRIVER_Y1_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_Y2_PORT, DRIVER_Y2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Y3_PORT, DRIVER_Y3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Y4_PORT, DRIVER_Y4_PIN, SET);
            break;
        }
    }
}

void DriverPhaseZ(char phase)
{
    switch (phase)
    {
        case 0b10000000:
        {
            HAL_GPIO_WritePin(DRIVER_Z1_PORT, DRIVER_Z1_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_Z2_PORT, DRIVER_Z2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Z3_PORT, DRIVER_Z3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Z4_PORT, DRIVER_Z4_PIN, RESET);
            break;
        }
        case 0b11000000:
        {
            HAL_GPIO_WritePin(DRIVER_Z1_PORT, DRIVER_Z1_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_Z2_PORT, DRIVER_Z2_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_Z3_PORT, DRIVER_Z3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Z4_PORT, DRIVER_Z4_PIN, RESET);
            break;
        }
        case 0b01000000:
        {
            HAL_GPIO_WritePin(DRIVER_Z1_PORT, DRIVER_Z1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Z2_PORT, DRIVER_Z2_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_Z3_PORT, DRIVER_Z3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Z4_PORT, DRIVER_Z4_PIN, RESET);
            break;
        }
        case 0b01100000:
        {
            HAL_GPIO_WritePin(DRIVER_Z1_PORT, DRIVER_Z1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Z2_PORT, DRIVER_Z2_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_Z3_PORT, DRIVER_Z3_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_Z4_PORT, DRIVER_Z4_PIN, RESET);
            break;
        }
        case 0b00100000:
        {
            HAL_GPIO_WritePin(DRIVER_Z1_PORT, DRIVER_Z1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Z2_PORT, DRIVER_Z2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Z3_PORT, DRIVER_Z3_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_Z4_PORT, DRIVER_Z4_PIN, RESET);
            break;
        }
        case 0b00110000:
        {
            HAL_GPIO_WritePin(DRIVER_Z1_PORT, DRIVER_Z1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Z2_PORT, DRIVER_Z2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Z3_PORT, DRIVER_Z3_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_Z4_PORT, DRIVER_Z4_PIN, SET);
            break;
        }
        case 0b00010000:
        {
            HAL_GPIO_WritePin(DRIVER_Z1_PORT, DRIVER_Z1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Z2_PORT, DRIVER_Z2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Z3_PORT, DRIVER_Z3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Z4_PORT, DRIVER_Z4_PIN, SET);
            break;
        }
        case 0b10010000:
        {
            HAL_GPIO_WritePin(DRIVER_Z1_PORT, DRIVER_Z1_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_Z2_PORT, DRIVER_Z2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Z3_PORT, DRIVER_Z3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_Z4_PORT, DRIVER_Z4_PIN, SET);
            break;
        }
    }
}

void DriverPhaseE(char phase)
{
    switch (phase)
    {
        case 0b00001000:
        {
            HAL_GPIO_WritePin(DRIVER_E1_PORT, DRIVER_E1_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_E2_PORT, DRIVER_E2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_E3_PORT, DRIVER_E3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_E4_PORT, DRIVER_E4_PIN, RESET);
            break;
        }
        case 0b00001100:
        {
            HAL_GPIO_WritePin(DRIVER_E1_PORT, DRIVER_E1_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_E2_PORT, DRIVER_E2_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_E3_PORT, DRIVER_E3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_E4_PORT, DRIVER_E4_PIN, RESET);
            break;
        }
        case 0b00000100:
        {
            HAL_GPIO_WritePin(DRIVER_E1_PORT, DRIVER_E1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_E2_PORT, DRIVER_E2_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_E3_PORT, DRIVER_E3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_E4_PORT, DRIVER_E4_PIN, RESET);
            break;
        }
        case 0b00000110:
        {
            HAL_GPIO_WritePin(DRIVER_E1_PORT, DRIVER_E1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_E2_PORT, DRIVER_E2_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_E3_PORT, DRIVER_E3_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_E4_PORT, DRIVER_E4_PIN, RESET);
            break;
        }
        case 0b00000010:
        {
            HAL_GPIO_WritePin(DRIVER_E1_PORT, DRIVER_E1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_E2_PORT, DRIVER_E2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_E3_PORT, DRIVER_E3_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_E4_PORT, DRIVER_E4_PIN, RESET);
            break;
        }
        case 0b00000011:
        {
            HAL_GPIO_WritePin(DRIVER_E1_PORT, DRIVER_E1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_E2_PORT, DRIVER_E2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_E3_PORT, DRIVER_E3_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_E4_PORT, DRIVER_E4_PIN, SET);
            break;
        }
        case 0b00000001:
        {
            HAL_GPIO_WritePin(DRIVER_E1_PORT, DRIVER_E1_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_E2_PORT, DRIVER_E2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_E3_PORT, DRIVER_E3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_E4_PORT, DRIVER_E4_PIN, SET);
            break;
        }
        case 0b00001001:
        {
            HAL_GPIO_WritePin(DRIVER_E1_PORT, DRIVER_E1_PIN, SET);
            HAL_GPIO_WritePin(DRIVER_E2_PORT, DRIVER_E2_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_E3_PORT, DRIVER_E3_PIN, RESET);
            HAL_GPIO_WritePin(DRIVER_E4_PORT, DRIVER_E4_PIN, SET);
            break;
        }
    }
}


void DriverEnableX()
{
    HAL_GPIO_WritePin(DRIVER_ENABLE_X_PORT, DRIVER_ENABLE_X_PIN, SET);
}

void DriverEnableY()
{
    HAL_GPIO_WritePin(DRIVER_ENABLE_Y_PORT, DRIVER_ENABLE_Y_PIN, SET);
}

void DriverEnableZ()
{
    HAL_GPIO_WritePin(DRIVER_ENABLE_Z_PORT, DRIVER_ENABLE_Z_PIN, SET);
}

void DriverEnableE()
{
    HAL_GPIO_WritePin(DRIVER_ENABLE_E_PORT, DRIVER_ENABLE_E_PIN, SET);
}


void DriverDisableX()
{
    HAL_GPIO_WritePin(DRIVER_ENABLE_X_PORT, DRIVER_ENABLE_X_PIN, RESET);
}

void DriverDisableY()
{
    HAL_GPIO_WritePin(DRIVER_ENABLE_Y_PORT, DRIVER_ENABLE_Y_PIN, RESET);
}

void DriverDisableZ()
{
    HAL_GPIO_WritePin(DRIVER_ENABLE_Z_PORT, DRIVER_ENABLE_Z_PIN, RESET);
}

void DriverDisableE()
{
    HAL_GPIO_WritePin(DRIVER_ENABLE_E_PORT, DRIVER_ENABLE_E_PIN, RESET);
}
