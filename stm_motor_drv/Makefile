# ============================================================================
# 直流电机驱动 Makefile — STM32F1 / STM32F4
# ============================================================================

# ---- 目标平台选择 ----
# make PLATFORM=F1   → STM32F103C8 (default)
# make PLATFORM=F4   → STM32F407VG
PLATFORM ?= F1

# ---- 工具链 ----
CROSS   ?= arm-none-eabi-
CC      := $(CROSS)gcc
CXX     := $(CROSS)g++
AS      := $(CROSS)gcc
LD      := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
OBJDUMP := $(CROSS)objdump
SIZE    := $(CROSS)size
GDB     := $(CROSS)gdb

# ---- 目录 ----
PROJ_DIR = .
INC_DIR  = $(PROJ_DIR)/inc
SRC_DIR  = $(PROJ_DIR)/src
CAN_DIR  = $(SRC_DIR)/canopen
BLD_DIR  = $(PROJ_DIR)/build

# ---- STM32 HAL/CMSIS 路径 (需根据实际安装位置修改) ----
STM32_HAL ?= ./STM32_HAL

# ---- 芯片/平台定义 ----
ifeq ($(PLATFORM),F1)
  MCU           := cortex-m3
  MCU_SPEC      := -mcpu=cortex-m3 -mthumb
  DEFINES       := -DSTM32_PLATFORM=1 -DSTM32F103xB
  CHIP          := STM32F103C8Tx
  LDSCRIPT      := $(PROJ_DIR)/STM32F103C8Tx_FLASH.ld
  HAL_DIR       := $(STM32_HAL)/STM32F1xx_HAL_Driver
  CMSIS_DIR     := $(STM32_HAL)/CMSIS
  STARTUP       := $(STM32_HAL)/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s
  SYSTEM        := $(STM32_HAL)/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c
  FLASH_DEF     :=
else ifeq ($(PLATFORM),F4)
  MCU           := cortex-m4
  MCU_SPEC      := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
  DEFINES       := -DSTM32_PLATFORM=2 -DSTM32F407xx
  CHIP          := STM32F407VGTx
  LDSCRIPT      := $(PROJ_DIR)/STM32F407VGTx_FLASH.ld
  HAL_DIR       := $(STM32_HAL)/STM32F4xx_HAL_Driver
  CMSIS_DIR     := $(STM32_HAL)/CMSIS
  STARTUP       := $(STM32_HAL)/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s
  SYSTEM        := $(STM32_HAL)/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
endif

# ---- FreeRTOS 路径 ----
FREERTOS_DIR ?= ./FreeRTOSv202212.01/FreeRTOS/Source
FREERTOS_INC  = $(FREERTOS_DIR)/include
ifeq ($(PLATFORM),F1)
  FREERTOS_PORT_DIR = ARM_CM3
else ifeq ($(PLATFORM),F4)
  FREERTOS_PORT_DIR = ARM_CM4F
endif
FREERTOS_PORT = $(FREERTOS_DIR)/portable/GCC/$(FREERTOS_PORT_DIR)
FREERTOS_MEM  = $(FREERTOS_DIR)/portable/MemMang/heap_4.c
FREERTOS_C    = $(FREERTOS_DIR)/tasks.c $(FREERTOS_DIR)/queue.c \
                $(FREERTOS_DIR)/list.c $(FREERTOS_DIR)/timers.c \
                $(FREERTOS_DIR)/event_groups.c $(FREERTOS_DIR)/stream_buffer.c

# ---- 源文件 ----
C_SOURCES  = $(wildcard $(SRC_DIR)/*.c)
C_SOURCES += $(wildcard $(CAN_DIR)/*.c)
C_SOURCES += $(SYSTEM)
C_SOURCES += $(HAL_DIR)/Src/stm32f1xx_hal.c
C_SOURCES += $(HAL_DIR)/Src/stm32f1xx_hal_adc.c
C_SOURCES += $(HAL_DIR)/Src/stm32f1xx_hal_adc_ex.c
C_SOURCES += $(HAL_DIR)/Src/stm32f1xx_hal_can.c
C_SOURCES += $(HAL_DIR)/Src/stm32f1xx_hal_cortex.c
C_SOURCES += $(HAL_DIR)/Src/stm32f1xx_hal_dma.c
C_SOURCES += $(HAL_DIR)/Src/stm32f1xx_hal_flash.c
C_SOURCES += $(HAL_DIR)/Src/stm32f1xx_hal_flash_ex.c
C_SOURCES += $(HAL_DIR)/Src/stm32f1xx_hal_gpio.c
C_SOURCES += $(HAL_DIR)/Src/stm32f1xx_hal_gpio_ex.c
C_SOURCES += $(HAL_DIR)/Src/stm32f1xx_hal_iwdg.c
C_SOURCES += $(HAL_DIR)/Src/stm32f1xx_hal_rcc.c
C_SOURCES += $(HAL_DIR)/Src/stm32f1xx_hal_rcc_ex.c
C_SOURCES += $(HAL_DIR)/Src/stm32f1xx_hal_tim.c
C_SOURCES += $(HAL_DIR)/Src/stm32f1xx_hal_tim_ex.c
C_SOURCES += $(HAL_DIR)/Src/stm32f1xx_hal_uart.c
C_SOURCES += $(FREERTOS_C) $(FREERTOS_MEM) $(FREERTOS_PORT)/port.c
AS_SOURCES = $(STARTUP)

# ---- 包含路径 ----
INC  = -I$(INC_DIR)
INC += -I$(INC_DIR)/canopen
INC += -I$(HAL_DIR)/Inc
INC += -I$(CMSIS_DIR)/Device/ST/STM32F1xx/Include
INC += -I$(CMSIS_DIR)/Include
INC += -I$(FREERTOS_INC)
INC += -I$(FREERTOS_PORT)

# ---- 编译标志 ----
DEFINES += -DUSE_HAL_DRIVER -DHSE_VALUE=8000000UL

CFLAGS  = $(MCU_SPEC) $(DEFINES) $(INC)
CFLAGS += -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-variable -Wno-unused-but-set-variable
CFLAGS += -Os -g3 -ffunction-sections -fdata-sections
CFLAGS += -fno-common -fno-strict-aliasing
CFLAGS += -std=gnu11

ASFLAGS = $(MCU_SPEC) $(DEFINES)

LDFLAGS = $(MCU_SPEC) -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections -Wl,-Map=$(BLD_DIR)/motor_drv.map
LDFLAGS += -specs=nosys.specs -specs=nano.specs

# ---- 目标 ----
TARGET = $(BLD_DIR)/motor_drv

C_OBJS  = $(patsubst %.c, $(BLD_DIR)/%.o, $(notdir $(C_SOURCES)))
AS_OBJS = $(patsubst %.s, $(BLD_DIR)/%.o, $(notdir $(AS_SOURCES)))
OBJS    = $(C_OBJS) $(AS_OBJS)

VPATH = $(SRC_DIR):$(CAN_DIR):$(dir $(SYSTEM)):$(HAL_DIR)/Src:$(FREERTOS_DIR):$(FREERTOS_PORT):$(dir $(FREERTOS_MEM)):$(dir $(STARTUP))

.PHONY: all clean flash debug size

all: $(TARGET).elf $(TARGET).bin $(TARGET).hex size

$(BLD_DIR)/%.o: %.c
	@mkdir -p $(BLD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BLD_DIR)/%.o: %.s
	@mkdir -p $(BLD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(TARGET).elf: $(OBJS)
	$(LD) $(LDFLAGS) $(OBJS) -o $@ -lm

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

size: $(TARGET).elf
	$(SIZE) $<

flash: $(TARGET).bin
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
	        -c "program $(TARGET).bin 0x08000000 verify reset exit"

debug: $(TARGET).elf
	$(GDB) -ex "target extended-remote :3333" \
	       -ex "monitor reset halt" \
	       -ex "load" \
	       $(TARGET).elf

clean:
	rm -rf $(BLD_DIR)
