#!/bin/bash

STM32_Programmer_CLI -c  port=SWD freq=4000 reset=HWrst -w build/joint_firmware.bin 0x08000000 