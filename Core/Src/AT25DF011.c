#include "AT25DF011.h"

FlashStatusRegister_t flashStatusRegister;

uint8_t readCommand[READ_COMMAND_SIZE] = {OP_READ_ARRAY, 0x00, 0x00, 0x00, 0x00};
uint8_t transmitCommand[TRANSMIT_COMMAND_SIZE] = {OP_WRITE_ARRAY, 0x00, 0x00, 0x00};
uint8_t pageEraseCommand[TRANSMIT_COMMAND_SIZE] = {OP_PAGE_ERASE, 0x00, 0x00, 0x00};  

uint8_t readIDCommand = OP_READ_MANUFACTURER_DEVICE_ID;
uint8_t readStatusCommand = OP_READ_STATUS_REGISTER;
uint8_t writeEnableCommand = OP_WRITE_ENABLE;
uint8_t eraseChipCommand = OP_ERASE_CHIP;

void flash_enableWrite(void)
{
    flash_select();
    HAL_SPI_Transmit(&hspi1, &writeEnableCommand, 1, -1); 
    flash_deselect();
}

void flash_updateStatusRegister(void)
{
    flash_select();
    HAL_SPI_Transmit(&hspi1, &readStatusCommand, 1, -1);
    HAL_SPI_Receive(&hspi1, (uint8_t *) &flashStatusRegister, 1, -1);
    flash_deselect();
}

void flash_waitForReady(void)
{
    flash_updateStatusRegister();
    while (flashStatusRegister.BSY)
    {
        flash_updateStatusRegister();
    }
}

// Only erases page 0
void flash_erasePage(void)
{
    flash_waitForReady();
    flash_enableWrite();
    flash_select();
    HAL_SPI_Transmit(&hspi1, pageEraseCommand, PAGE_ERASE_COMMAND_SIZE, -1);
    flash_deselect();
}

// Blocking
void flash_writeBytes(uint8_t *data, uint32_t size)
{
    flash_erasePage();
    flash_waitForReady();
    flash_enableWrite();
    flash_select();
    HAL_SPI_Transmit(&hspi1, transmitCommand, TRANSMIT_COMMAND_SIZE, -1);
    HAL_SPI_Transmit(&hspi1, data, size, -1);
    flash_deselect();
}

void flash_readBytes(uint8_t *data, uint32_t size)
{
    flash_waitForReady();
    flash_select();
    HAL_SPI_Transmit(&hspi1, readCommand, READ_COMMAND_SIZE, -1);
    HAL_SPI_Receive(&hspi1, data, size, -1);
    flash_deselect();
}
