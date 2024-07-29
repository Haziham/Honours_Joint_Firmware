#!/bin/bash
# sleep 3
STM32_Programmer_CLI -c  port=SWD freq=4000 reset=HWrst -w build/joint_firmware.elf 0x08000000 -g