TARGET := STM32G030F6P6TR
BUILD_DIR := Build

PREFIX := arm-none-eabi-
CC := $(PREFIX)gcc
AS := $(PREFIX)gcc -x assembler-with-cpp
CP := $(PREFIX)objcopy
SZ := $(PREFIX)size

MCU := -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft
DEFS := -DDEBUG -DUSE_HAL_DRIVER -DSTM32G030xx
EXTRA_DEFS ?=
DEFS += $(EXTRA_DEFS)

C_SOURCES := $(shell find Core Drivers -name '*.c')
ASM_SOURCES := $(shell find Core Drivers -name '*.s' -o -name '*.S')

INC_DIRS := \
Core/Inc \
Core/Inc/VL53L1X \
Core/Inc/VL53L1X/core \
Core/Inc/VL53L1X/platform \
Drivers/STM32G0xx_HAL_Driver/Inc \
Drivers/STM32G0xx_HAL_Driver/Inc/Legacy \
Drivers/CMSIS/Device/ST/STM32G0xx/Include \
Drivers/CMSIS/Include

INCLUDES := $(addprefix -I,$(INC_DIRS))

CFLAGS := $(MCU) $(DEFS) $(INCLUDES) -Og -g3 -Wall -fdata-sections -ffunction-sections
CFLAGS += -MMD -MP
ASFLAGS := $(MCU) $(DEFS) $(INCLUDES) -g3
LDSCRIPT := STM32G030F6PX_FLASH.ld
LDFLAGS := $(MCU) -T$(LDSCRIPT) --specs=nano.specs -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections
LIBS := -lc -lm

OBJECTS := $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))
OBJECTS := $(OBJECTS:.S=.o)

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	$(SZ) $(BUILD_DIR)/$(TARGET).elf

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) $(LIBS) -o $@

$(BUILD_DIR)/%.o: %.c Makefile
	mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile
	mkdir -p $(dir $@)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.S Makefile
	mkdir -p $(dir $@)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(CP) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(CP) -O binary -S $< $@

clean:
	rm -rf $(BUILD_DIR)

-include $(OBJECTS:.o=.d)

.PHONY: all clean
