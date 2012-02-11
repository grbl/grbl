 #####
 # Stephen Caudle, Copyright (C) 2010.
 #
 #
 # This program is free software; you can redistribute it and/or modify
 # it under the terms of the GNU General Public License as published by
 # the Free Software Foundation; either version 3 of the License, or
 # (at your option) any later version.
 #
 # This program is distributed in the hope that it will be useful, but
 # WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 # or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 # for more details.
 #
 # You should have received a copy of the GNU General Public License along
 # with this program; if not, write to the Free Software Foundation, Inc.,
 # 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 #####

#	adjusted by chrisu_de for various projects.
#

#change your device: 
# 	avr
# 	freertos_lpc17
#	stm32l
#	simulator

ifeq ($(ARCH),)
#ARCH = stm32l
ARCH = mbed_lpc17
#ARCH = avr
#ARCH = simulator
endif

DEVICE_DIR  = arch/$(ARCH)
-include $(DEVICE_DIR)/Makefile

# Directory for output files (lst, obj, dep, elf, map, hex, bin etc.)
OUTDIR = build

# Target file name (without extension).
TARGET = grbl-$(ARCH)

# Source files:
SRC += main.c 
SRC += nuts_bolts.c
SRC += motion_control.c 
SRC += gcode.c
SRC += spindle_control.c
SRC += protocol.c
SRC += settings.c
SRC += planner.c
SRC += limits.c
SRC += print.c

#include device specific files
SRC += $(patsubst %,$(DEVICE_DIR)/%,$(DEV_SRC))

# List any extra directories to look for include files here.
INCDIRS  += $(DEVICE_DIR) 
INCDIRS  += $(patsubst %,$(DEVICE_DIR)/%,$(DEV_INC))

CFLAGS += $(patsubst %,-I%,$(INCDIRS)) -I.
CFLAGS += -std=gnu99
CFLAGS += -Wall

# Define programs and commands.
CC      = $(TCHAIN_PREFIX)gcc
AR      = $(TCHAIN_PREFIX)ar
OBJCOPY = $(TCHAIN_PREFIX)objcopy
OBJDUMP = $(TCHAIN_PREFIX)objdump
SIZE    = $(TCHAIN_PREFIX)size
NM      = $(TCHAIN_PREFIX)nm
GDB     = $(TCHAIN_PREFIX)gdb
REMOVE  = rm -rf

# List of all source files without directory and file-extension.
ALLSRCBASE = $(notdir $(basename $(SRC)))

# Define all object files.
ALLOBJ     = $(addprefix $(OUTDIR)/, $(addsuffix .o, $(ALLSRCBASE)))

default: all

# Default target.
all: $(OUTDIR)/$(TARGET).elf

# Program the device.
flash: $(OUTDIR)/$(TARGET).elf
	@echo "Flashing to device"
	$(FLASH_COMMAND)

# Link: create ELF output file from object files.
.SECONDARY : $(OUTDIR)/$(TARGET).elf
.PRECIOUS : $(ALLOBJ)
$(OUTDIR)/$(TARGET).elf: $(ALLOBJ)
	@echo
	@echo "**** Linking :" $@
	@$(LINK_COMMAND)

# Compile: create object files from C source files.
define COMPILE_C_SOURCE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
##	@echo
	@echo "**** Compiling :" $$< "->" $$@
	@$(CC) -c $$(CFLAGS) $$< -o $$@
endef
$(foreach src, $(SRC), $(eval $(call COMPILE_C_SOURCE, $(src))))

# Target: clean project.
clean:
	@echo "cleaned build"
	@$(REMOVE) $(OUTDIR)

test: 	
	@echo "STM32:"
	@make clean	
	@make ARCH=stm32l
	
	@echo "Compiling freertos_lpc17"
	@make clean	
	@make ARCH=freertos_lpc17
	
	@echo "Compiling avr:"
	@make clean	
	@make ARCH=avr
	
	@echo "Compiling simulator"
	@make clean	
	@make ARCH=simulator


# Create output files directory
$(shell mkdir -p $(OUTDIR))

# Listing of phony targets.
.PHONY: all build lss clean program flash
