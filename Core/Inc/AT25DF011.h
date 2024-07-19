#pragma once

#include <stdint.h>

#define OP_READ_ARRAY 0x0B
#define OP_WRITE_ARRAY 0x02

#define SETTINGS_START_ADDRESS 0x0000

#define READ_COMMAND_SIZE 5
#define TRANSMIT_COMMAND_SIZE 4

const uint8_t readCommand[READ_COMMAND_SIZE] = {OP_READ_ARRAY, 0x00, 0x00, 0x00, 0x00};
const uint8_t transmitCommand[TRANSMIT_COMMAND_SIZE] = {OP_WRITE_ARRAY, 0x00, 0x00, 0x00};

#define spi_flash_select() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_RESET)

