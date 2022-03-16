#define Sram_Start 0x20000000

    .word Sram_Start
    .word Reset_Handler
Reset_Handler:

    /* SP */
    ldr r0, =Sram_Start
    ldr r1, [r0]
    msr MSP, r1
    
    /* VTOR */
    ldr r1, =0xe000ed08
    str r0, [r1]
    
    /* Hop */
    ldr r0, [r0, #4]
    bx r0
    