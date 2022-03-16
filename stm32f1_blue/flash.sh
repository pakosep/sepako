#!/bin/sh -e
openocd -f ocd.cfg -c init -c "reset halt" -c "flash write_image erase out/flash.elf" -c "verify_image out/flash.elf" -c reset -c shutdown
#openocd -f ocd.cfg -c init -c "reset halt" -c "flash write_image erase out/flash.bin 0x08000000" -c "verify_image out/flash.bin 0x08000000" -c reset -c shutdown
#openocd -f ocd.cfg -c init -c "reset halt" -c "verify_image out/flash.elf" -c reset -c shutdown


