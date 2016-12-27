## Cross-compilation commands 
CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-gcc
AR      = arm-none-eabi-ar
AS      = arm-none-eabi-as
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE    = arm-none-eabi-size

# our code
SRC  += $(wildcard ./*.c)
SRC  += $(wildcard $(CMSIS_SRC)/*.c)
OBJS  = $(SRC:.c=.o)

## Platform and optimization options
CFLAGS += -c -fno-common -Os -g -mcpu=cortex-m0plus -mthumb
CFLAGS += -Wall -ffunction-sections -fdata-sections -fno-builtin
CFLAGS += -Wno-unused-function -ffreestanding

## Locate the main libraries
CMSIS     = /usr/local/resources/CMSIS
CMSIS_SRC = $(CMSIS)/Device/NXP/MKL02Z4

LFILE = $(CMSIS_SRC)/MKL02Z4.ld
LFLAGS  = -$(LFILE) -nostartfiles -Wl,--gc-sections

## Drivers/library includes
INC     = -I$(CMSIS)/Include/ -I$(CMSIS_SRC)
CFLAGS += $(INC)

## Rules
all: size flash

main.elf: $(OBJS) $(LFILE)
	$(LD) $(LFLAGS) -o main.elf $(OBJS) 

%.bin: %.elf
	$(OBJCOPY) --strip-unneeded -O binary $< $@

## Convenience targets

flash: main.bin
	st-flash --reset write $< 0x8000000 

size: main.elf
	$(SIZE) $< 

clean:
	-rm -f $(OBJS) main.lst main.elf main.hex main.map main.bin main.list

.PHONY: all flash size clean distclean

