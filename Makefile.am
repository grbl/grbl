#
#        Copyright (C) 2011 Shawn Wilson
#        shawn@ch2a.ca
#        
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

#
# This config generates a makefile for building the grbl firmware.
# It converts it into an avrdude-compatible hex file which then gets
# uploaded to the arduino when you run 'make install'.
#

# Set the CPU type when compiling.
AM_CFLAGS = -Wall -Os -mmcu=$(MMCU) -DF_CPU=$(MHZ)000000 -I $(includedir) -ffunction-sections

# Instructions for building the executable file.
noinst_PROGRAMS = grbl.elf 
noinst_HEADERS = config.h limits.h planner.h serial.h stepper.h \
   eeprom.h motion_control.h print.h settings.h gcode.h nuts_bolts.h \
   protocol.h spindle_control.h
grbl_elf_SOURCES = main.c eeprom.c planner.c serial.c stepper.c \
   gcode.c motion_control.c print.c settings.c limits.c nuts_bolts.c \
   protocol.c  spindle_control.c
grbl_elf_LDADD = -lm 
grbl_elf_LDFLAGS = --gc-sections
noinst_DATA = grbl.hex


#
# In order to upload the grbl.elf executable to the arduino CPU,
# we have to convert it to hex format.  This file in turn gets uploaded
# to the arduino using avrdude when you run 'make install'.
#
# After we're done converting the executable file to hex, we print
# out the file size info using the same format as the original Makefile.
#
.elf.hex :
	$(STRIP) -s $<
	$(OBJCOPY) -O ihex -j .text -j .data $< $@
	@echo -e "\n\n--- Requires `$(OBJDUMP) -h grbl.elf | grep .bss | awk '{print "ibase=16 ; " toupper($$3) }' | bc` bytes of SRAM"
	@avr-size *.hex *.elf *.o


#
# Cleanup
#
clean-local:
	rm -f *.elf 
	rm -f *.hex 

# On installation, run avrdude to upload the compiled code to the arduino.
install-data-hook:
	avrdude -p $(AVRDUDE_PARTNO) -c $(AVRDUDE_PROGRAMMER) -P$(AVRDUDE_SERIAL_PORT) $(AVRDUDEFLAGS) -Uflash:w:grbl.hex:i
