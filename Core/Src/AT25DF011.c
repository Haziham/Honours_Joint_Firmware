#include "AT25DF011.h"


uint8_t readCommand[READ_COMMAND_SIZE] = {OP_READ_ARRAY, 0x00, 0x00, 0x00, 0x00};
uint8_t transmitCommand[TRANSMIT_COMMAND_SIZE] = {OP_WRITE_ARRAY, 0x00, 0x00, 0x00};


uint8_t readIDCommand = OP_READ_MANUFACTURER_DEVICE_ID;
uint8_t readStatusCommand = OP_READ_STATUS_REGISTER;
uint8_t writeEnableCommand = OP_WRITE_ENABLE;
