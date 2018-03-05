#ifndef config
#define config

    #define TEMPERATURE_OF_EXTRUDER 220
    #define TEMPERATURE_OF_BED 110
    #define TEMPERATURE_WINDOW 1

    #define HEAT_EXTRUDER_PIN GPIO_PIN_10
    #define HEAT_EXTRUDER_PORT GPIOC
    #define HEAT_BED_PIN GPIO_PIN_11
    #define HEAT_BED_PORT GPIOC

    #define C0 20000
    #define TIMER1_FREQURENCY 1000000

    #define PHASE_LENGTH 8
    #define BUFFER_LENGTH 64
    #define WAS_OVERFOLLOW 0b00000001
    #define WAS_COMMENT 0b00000010
    #define CRC_CHECK_ENABLED 0b00100000


    #define ERROR_BUFFER_OVERFOLLOW "Error: the command buffer full!\n"
    #define WARNING_NOTHING_TO_DO "Warning: nothing to do!\n"
    #define WARNING_UNSUPPORTED_COMMAND "Warning: unsupported command!\n"
    #define ERROR_CHECKSUM_FAILED "rs\n"
    #define SUCCESS_DONE "ok\n"
    #define WARNING_WAITING "_\n"
    #define START_MESSAGE "start\n"




    #define DRIVER_X1_PIN GPIO_PIN_0
    #define DRIVER_X1_PORT GPIOC
    #define DRIVER_X2_PIN GPIO_PIN_1
    #define DRIVER_X2_PORT GPIOC
    #define DRIVER_X3_PIN GPIO_PIN_2
    #define DRIVER_X3_PORT GPIOC
    #define DRIVER_X4_PIN GPIO_PIN_3
    #define DRIVER_X4_PORT GPIOC

    #define DRIVER_Y1_PIN GPIO_PIN_4
    #define DRIVER_Y1_PORT GPIOA
    #define DRIVER_Y2_PIN GPIO_PIN_5
    #define DRIVER_Y2_PORT GPIOA
    #define DRIVER_Y3_PIN GPIO_PIN_6
    #define DRIVER_Y3_PORT GPIOA
    #define DRIVER_Y4_PIN GPIO_PIN_7
    #define DRIVER_Y4_PORT GPIOA

    #define DRIVER_Z1_PIN GPIO_PIN_6
    #define DRIVER_Z1_PORT GPIOC
    #define DRIVER_Z2_PIN GPIO_PIN_7
    #define DRIVER_Z2_PORT GPIOC
    #define DRIVER_Z3_PIN GPIO_PIN_8
    #define DRIVER_Z3_PORT GPIOC
    #define DRIVER_Z4_PIN GPIO_PIN_9
    #define DRIVER_Z4_PORT GPIOC

    #define DRIVER_E1_PIN GPIO_PIN_8
    #define DRIVER_E1_PORT GPIOA
    #define DRIVER_E2_PIN GPIO_PIN_9
    #define DRIVER_E2_PORT GPIOA
    #define DRIVER_E3_PIN GPIO_PIN_10
    #define DRIVER_E3_PORT GPIOA
    #define DRIVER_E4_PIN GPIO_PIN_11
    #define DRIVER_E4_PORT GPIOA

    #define DRIVER_ENABLE_X_PIN GPIO_PIN_6
    #define DRIVER_ENABLE_X_PORT GPIOB
    #define DRIVER_ENABLE_Y_PIN GPIO_PIN_7
    #define DRIVER_ENABLE_Y_PORT GPIOB
    #define DRIVER_ENABLE_Z_PIN GPIO_PIN_8
    #define DRIVER_ENABLE_Z_PORT GPIOB
    #define DRIVER_ENABLE_E_PIN GPIO_PIN_9
    #define DRIVER_ENABLE_E_PORT GPIOB


    #define ABSOLUTE_POSITIONING 0b01000000
    #define NEW_TASK 0b10000000
    #define STEPS_PER_X 10 //5
    #define STEPS_PER_Y 10 //5
    #define STEPS_PER_Z 304
    #define STEPS_PER_E 20 //10
    #define MAX_ALLOWED_SPEED_X 15000
    #define MAX_ALLOWED_SPEED_Y 15000
    #define MAX_ALLOWED_SPEED_Z 3000
    #define MAX_ALLOWED_SPEED_E 5000
    #define INITIAL_F 1000
    #define INC_X  1
    #define DEC_X -1
    #define INC_Y  2
    #define DEC_Y -2
    #define INC_Z  3
    #define DEC_Z -3
    #define INC_E  4
    #define DEC_E -4

    struct vector
    {
        float x;
        float y;
        float z;
        float e;
    };

    struct discret_vector
    {
        long x;
        long y;
        long z;
        long e;
    };

#endif
