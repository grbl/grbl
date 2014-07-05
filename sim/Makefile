#  Part of Grbl Simulator
#
#  Copyright (c) 2012 Jens Geisler
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

PLATFORM   = WINDOWS

OBJECTS    = main.o simulator.o serial.o ../main.o ../protocol.o ../planner.o ../settings.o ../print.o ../nuts_bolts.o eeprom.o ../serial.o avr/pgmspace.o avr/interrupt.o avr/io.o util/delay.o util/floatunsisf.o ../stepper.o ../gcode.o ../spindle_control.o ../motion_control.o ../limits.o ../report.o ../coolant_control.o ../probe.o ../system.o platform_$(PLATFORM).o
CLOCK      = 16000000
EXE_NAME   = grbl_sim.exe
COMPILE    = $(CC) -Wall -g -DF_CPU=$(CLOCK) -include config.h -I. -DPLAT_$(PLATFORM)
LINUX_LIBRARIES = -lrt -pthread
WINDOWS_LIBRARIES = 
# symbolic targets:
all:	main

new: clean main

clean:
	rm -f $(EXE_NAME) $(OBJECTS)

# file targets:
main: $(OBJECTS)
	$(COMPILE) -o $(EXE_NAME) $(OBJECTS) -lm  $($(PLATFORM)_LIBRARIES)

%.o: %.c
	$(COMPILE)  -c $< -o $@

../planner.o: ../planner.c
	$(COMPILE) -include planner_inject_accessors.c -c $< -o $@

../serial.o: ../serial.c
	$(COMPILE) -include serial_hooks.h -c $< -o $@

../main.o: ../main.c
	$(COMPILE) -include rename_main.h -c $< -o $@


