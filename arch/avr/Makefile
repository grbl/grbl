#  Part of Grbl
#
#  Copyright (c) 2009-2011 Simen Svale Skogsrud
#
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

DEVICE     = atmega328p
CLOCK      = 16000000
DEV_SRC    = eeprom.c dev_misc.c serial.c stepper.c

FUSES      = -U hfuse:w:0xd2:m -U lfuse:w:0xff:m
# update that line with this when programmer is back up: 
# FUSES      = -U hfuse:w:0xd7:m -U lfuse:w:0xff:m 

TCHAIN_PREFIX = avr-
CFLAGS += -Os
CFLAGS += -ffunction-sections 
CFLAGS += -DF_CPU=$(CLOCK)
CFLAGS += -mmcu=$(DEVICE) 

FLASH_COMMAND = avrdude -c avrisp2 -P usb -p $(DEVICE) -B 10 -F 

LINK_COMMAND = $(CC) $(CFLAGS) -o $(OUTDIR)/$(TARGET).elf $(ALLOBJ) -lm -Wl,--gc-sections && \
	       $(CREATE_HEX)

CREATE_HEX = rm -f $(OUTDIR)/$(TARGET).hex && \
	$(OBJCOPY) -j .text -j .data -O ihex $(OUTDIR)/$(TARGET).elf $(OUTDIR)/$(TARGET).hex &&\
	$(OBJDUMP) -h $(OUTDIR)/$(TARGET).elf | grep .bss | \
	ruby -e 'puts "\n\n--- Requires %s bytes of SRAM" % STDIN.read.match(/0[0-9a-f]+\s/)[0].to_i(16)' && \
	$(SIZE) $(OUTDIR)/*.hex $(OUTDIR)/*.elf $(OUTDIR)/*.o

