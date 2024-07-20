#pragma once

#include <stdint.h>

#define OP_READ_ARRAY 0x0B
#define OP_WRITE_ARRAY 0x02
#define OP_READ_STATUS_REGISTER 0x05
#define OP_READ_MANUFACTURER_DEVICE_ID 0x9F
#define OP_WRITE_ENABLE 0x06

#define SETTINGS_START_ADDRESS 0x0000

#define READ_COMMAND_SIZE 5
#define TRANSMIT_COMMAND_SIZE 4

extern uint8_t readCommand[READ_COMMAND_SIZE];
extern uint8_t transmitCommand[TRANSMIT_COMMAND_SIZE];
extern uint8_t readIDCommand;
extern uint8_t readStatusCommand;
extern uint8_t writeEnableCommand;


#define flash_select() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_RESET)
#define flash_deselect() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_SET)

#define flash_writeEnable() flash_select(); HAL_SPI_Transmit(&hspi1, &writeEnableCommand, 1, -1); flash_deselect()
