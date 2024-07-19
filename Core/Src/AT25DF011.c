#include "AT25DF011.h"


uint8_t readCommand[READ_COMMAND_SIZE] = {OP_READ_ARRAY, 0x00, 0x00, 0x00, 0x00};
uint8_t transmitCommand[TRANSMIT_COMMAND_SIZE] = {OP_WRITE_ARRAY, 0x00, 0x00, 0x00};