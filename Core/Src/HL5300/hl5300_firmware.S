    .section .rodata.hl5300_firmware,"a",%progbits
    .balign 4
    .global hl5300_firmware
    .type hl5300_firmware, %object
hl5300_firmware:
    /* Firmware_Ranging payload from XL5300_Firmware_8.lib in the STM32F030 7 m example. */
    .incbin "Core/Src/HL5300/hl5300_firmware.blob"
    .size hl5300_firmware, . - hl5300_firmware
    .if (. - hl5300_firmware) != 7679
    .error "Unexpected HL5300 firmware length"
    .endif
    .section .note.GNU-stack,"",%progbits
