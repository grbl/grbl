#  Grbl is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  Grbl is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

#This is configured for the MBED

MCU	   = cortex-m3
CLOCK      = 99000000
## Application

RTOSSRCDIR = freertos/Source
RTOSINCDIR = $(RTOSSRCDIR)/include

DEV_SRC += eeprom.c
DEV_SRC += dev_misc.c 
DEV_SRC += serial.c 
DEV_SRC += stepper.c

#stuff
DEV_SRC += printf.c
DEV_SRC += sbrk.c

#Hardware
DEV_SRC += hardware/led.c
DEV_SRC += hardware/uart.c
DEV_SRC += hardware/gpio.c
DEV_SRC += hardware/cr_startup_lpc17.c

## RTOS
DEV_SRC += $(RTOSSRCDIR)/list.c
DEV_SRC += $(RTOSSRCDIR)/queue.c
DEV_SRC += $(RTOSSRCDIR)/tasks.c
DEV_SRC += $(RTOSSRCDIR)/timers.c

## RTOS Portable
DEV_SRC += $(RTOSSRCDIR)/portable/GCC/ARM_CM3/port.c
DEV_SRC += $(RTOSSRCDIR)/portable/MemMang/heap_1.c

TCHAIN_PREFIX = arm-none-eabi-

DEV_INC  += hardware
DEV_INC  += freertos
DEV_INC  += $(RTOSINCDIR)
DEV_INC  += $(RTOSSRCDIR)/portable/GCC/ARM_CM3

CFLAGS += -mcpu=$(MCU) -mthumb -msoft-float -mfix-cortex-m3-ldrd -msoft-float
CFLAGS += -DF_CPU=$(CLOCK)
CFLAGS += -mapcs-frame
CFLAGS += -O2 
CFLAGS += -D PACK_STRUCT_END=__attribute\(\(packed\)\) -D ALIGN_STRUCT_END=__attribute\(\(aligned\(4\)\)\)
CFLAGS += -Wa,-adhlns=$(addprefix $(OUTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))

LDFLAGS = -nostartfiles -Wl,-Map=$(OUTDIR)/$(TARGET).map,--cref,--gc-sections
LDFLAGS += -lc
LDFLAGS += -lm
LDFLAGS += -lgcc
LDFLAGS +=-T arch/mbed_lpc17/hardware/rdb1768_Debug.ld 

FLASH_COMMAND  = $(OBJCOPY) -O binary $(OUTDIR)/$(TARGET).elf $(OUTDIR)/$(TARGET).bin &&\
		cp $(OUTDIR)/$(TARGET).bin -f /media/MBED

LINK_COMMAND  = $(CC) $(CFLAGS) $(ALLOBJ) --output $@ $(LDFLAGS)

