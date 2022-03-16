target remote localhost:3333
monitor reset init
monitor halt
load out/flash.elf
continue