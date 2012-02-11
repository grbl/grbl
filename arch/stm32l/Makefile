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

#for the stm32l discovery board!
#

MCU	   = cortex-m3
CLOCK      = 24000000
CHIP	   = stm32f100rb
MODEL	   = md_vl
BOARD	   = STM32Discovery
F_XTAL     = 8000000
SYSCLOCK_CL = SYSCLK_FREQ_24MHz=24000000
STACK_SIZE ?= 1024

CFLAGS += -DSTM32F10X_$(shell echo $(MODEL) | tr a-z A-Z)
CFLAGS += -DUSE_STDPERIPH_DRIVER
CFLAGS += -DUSE_$(BOARD)
CFLAGS += -DHSE_VALUE=$(F_XTAL)UL
CFLAGS += -D$(SYSCLOCK_CL)
ifneq ($(STACK_SIZE),)
CFLAGS += -DSTACK_SIZE=$(STACK_SIZE)
endif

STMLIBDIR = stm32/Libraries
STMSPDSRCDIR = $(STMLIBDIR)/STM32F10x_StdPeriph_Driver/src
STMSPDINCDIR = $(STMLIBDIR)/STM32F10x_StdPeriph_Driver/inc
CMSISDIR  = $(STMLIBDIR)/CMSIS/CM3

DEV_INC  += $(CMSISDIR)/CoreSupport
DEV_INC  += $(CMSISDIR)/CoreSupport
DEV_INC  += $(CMSISDIR)/DeviceSupport/ST/STM32F10x
DEV_INC  += stm32
DEV_INC  += $(STMSPDINCDIR)

DEV_SRC    += eeprom.c 
DEV_SRC	   += dev_misc.c 
DEV_SRC	   += serial.c 
DEV_SRC    += sbrk.c 
DEV_SRC    += stepper.c 

## CMSIS for STM32
DEV_SRC += $(CMSISDIR)/CoreSupport/core_cm3.c

## STM32:
DEV_SRC += stm32/vector.c
DEV_SRC += stm32/system_stm32f10x.c

## Used parts of the STM-Library
#DEV_SRC += $(STMSPDSRCDIR)/stm32f10x_adc.c
#DEV_SRC += $(STMSPDSRCDIR)/stm32f10x_bkp.c
#DEV_SRC += $(STMSPDSRCDIR)/stm32f10x_crc.c
#DEV_SRC += $(STMSPDSRCDIR)/stm32f10x_dac.c
#DEV_SRC += $(STMSPDSRCDIR)/stm32f10x_dma.c
#DEV_SRC += $(STMSPDSRCDIR)/stm32f10x_exti.c
#DEV_SRC += $(STMSPDSRCDIR)/stm32f10x_flash.c
DEV_SRC += $(STMSPDSRCDIR)/stm32f10x_gpio.c
#DEV_SRC += $(STMSPDSRCDIR)/stm32f10x_i2c.c
DEV_SRC += $(STMSPDSRCDIR)/stm32f10x_pwr.c
DEV_SRC += $(STMSPDSRCDIR)/stm32f10x_rcc.c
DEV_SRC += $(STMSPDSRCDIR)/stm32f10x_rtc.c
#DEV_SRC += $(STMSPDSRCDIR)/stm32f10x_spi.c
DEV_SRC += $(STMSPDSRCDIR)/stm32f10x_tim.c
DEV_SRC += $(STMSPDSRCDIR)/stm32f10x_usart.c
DEV_SRC += $(STMSPDSRCDIR)/misc.c


# Linker flags.
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS += -nostartfiles -Wl,-Map=$(OUTDIR)/$(TARGET).map,--cref,--gc-sections
LDFLAGS += -lc -lm -lgcc

# Set linker-script name depending on selected submodel name
LDFLAGS +=-Tarch/stm32l/stm32/$(CHIP).ld
LDFLAGS +=-Tarch/stm32l/stm32/stm32f1xx.ld

TCHAIN_PREFIX = arm-none-eabi-

CFLAGS += -mcpu=$(MCU) -mthumb -msoft-float -mfix-cortex-m3-ldrd -msoft-float 
CFLAGS += -DF_CPU=$(CLOCK)
CFLAGS += -mapcs-frame
CFLAGS += -O2 

LINK_COMMAND  = $(CC) $(LDFLAGS) -o $(OUTDIR)/$(TARGET).elf $(ALLOBJ)

#stlink gdbserver must be running:
FLASH_COMMAND  = arm-none-eabi-gdb -x stm32_flash.gdb 

