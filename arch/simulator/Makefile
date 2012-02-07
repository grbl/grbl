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

CLOCK      = 99000000
DEV_SRC    = eeprom.c dev_misc.c serial.c stepper.c simul_graphic.c

TCHAIN_PREFIX = 
CFLAGS += -Os
CFLAGS += -ffunction-sections 
CFLAGS += -DF_CPU=$(CLOCK) -lSDL -lSDLmain -lSDL_ttf -lSDL_image -lpthread -lrt

LINK_COMMAND = $(CC) $(CFLAGS) -o $(OUTDIR)/$(TARGET).elf $(ALLOBJ) -lm -Wl,--gc-sections 	       
