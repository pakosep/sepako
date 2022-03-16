#!/bin/sh -e
#openocd -f lockpick.cfg -c init -c "reset halt" -c "flash write_image erase out/flash.elf" -c "verify_image out/flash.elf" -c reset -c shutdown
#openocd -f lockpick.cfg -c init -c "reset halt" -c "verify_image out/flash.elf" -c reset -c shutdown
#openocd -f lockpick.cfg -c init -c "reset halt" -c "load_image out/ram.bin 0x20000000" -c reset -c shutdown
#openocd -f lockpick.cfg -c init -c "reset halt" -c "load_image out/ram.bin 0x20000000" -c "resume 0x20000000" -c shutdown
openocd -f ocd.cfg -c init -c "reset halt" -c reset -c shutdown

