#!/bin/sh -e
#openocd -f ocd.cfg -c init -c "reset halt" -c "flash write_image erase out/ram.elf" -c "verify_image out/ram.elf" -c reset -c shutdown
#openocd -f ocd.cfg -c init -c "reset halt" -c "verify_image out/flash.elf" -c reset -c shutdown
#openocd -f ocd.cfg -c init -c "reset halt" -c "load_image out/ram.bin 0x20000000" -c reset -c shutdown
openocd -f ocd.cfg -c init -c "reset halt" -c "load_image out/ram.bin 0x20000000" -c "resume 0x20000000" -c shutdown
#openocd -f ocd.cfg -c init -c "reset halt" -c "load_image out/ram.bin 0x20000000" -c reset -c shutdown
