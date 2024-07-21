#pragma once

#include <stdint.h>
#include "spi.h"

#define OP_READ_ARRAY 0x0B
#define OP_WRITE_ARRAY 0x02
#define OP_READ_STATUS_REGISTER 0x05
#define OP_READ_MANUFACTURER_DEVICE_ID 0x9F
#define OP_WRITE_ENABLE 0x06
#define OP_ERASE_CHIP 0x60
#define OP_PAGE_ERASE 0x81


#define SETTINGS_START_ADDRESS 0x0000

#define READ_COMMAND_SIZE 5
#define TRANSMIT_COMMAND_SIZE 4
#define PAGE_ERASE_COMMAND_SIZE 4

extern uint8_t readCommand[READ_COMMAND_SIZE];
extern uint8_t transmitCommand[TRANSMIT_COMMAND_SIZE];
extern uint8_t pageEraseCommand[TRANSMIT_COMMAND_SIZE];
extern uint8_t readIDCommand;
extern uint8_t readStatusCommand;
extern uint8_t writeEnableCommand;
extern uint8_t eraseChipCommand;


typedef struct {
    uint8_t BSY : 1; // Busy
    uint8_t WEL : 1; // Write Enable Latch
    uint8_t BP0 : 1; // Block Protect 0
    uint8_t RES : 1; // Reserved
    uint8_t WPP : 1; // Write Protect
    uint8_t EPE : 1; // Erase/Program Error
    uint8_t RES2 : 1; // Reserved
    uint8_t BPL : 1; // Block Protect Locked 

} FlashStatusRegister_t;

extern FlashStatusRegister_t flashStatusRegister;

#define flash_select() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_RESET)
#define flash_deselect() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_SET)


void flash_enableWrite(void);
void flash_updateStatusRegister(void);
void flash_waitForReady(void);
void flash_erasePage(void);
void flash_writeBytes(uint8_t* data, uint32_t size);
void flash_readBytes(uint8_t* data, uint32_t size);
