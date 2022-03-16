all: add.s stm32.ld clean
  
  arm-none-eabi-as -mcpu=cortex-m3 -o add.o add.s
  arm-none-eabi-ld -mcpu=cortex-m3 -Tstm32f103vc_rom.ld -o add.elf add.o
  arm-none-eabi-objcopy -O binary add.elf add.bin
  
clean:
  @echo "Running target clean"
  rm -f *.o
  rm -f *.elf
  rm -f *.bin