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

OBJECTS    = main.o simulator.o runtime.o ../protocol.o ../planner.o ../settings.o ../print.o ../nuts_bolts.o eeprom.o serial.o avr/pgmspace.o avr/interrupt.o util/delay.o util/floatunsisf.o ../stepper.o ../gcode.o ../spindle_control.o ../motion_control.o ../limits.o ../report.o ../coolant_control.o
CLOCK      = 16000000
EXE_NAME   = grbl_sim.exe
COMPILE    = $(CC) -Wall -Os -DF_CPU=$(CLOCK) -include config.h -I.

# symbolic targets:
all:	main

new: clean main

clean:
	rm -f $(EXE_NAME) $(OBJECTS)

# file targets:
main: $(OBJECTS)
	$(COMPILE) -o $(EXE_NAME) $(OBJECTS) -lm

%.o: %.c
	$(COMPILE) -c $< -o $@

../protocol.o: ../protocol.c
	$(COMPILE) -include rename_execute_runtime.h -c $< -o $@

../planner.o: ../planner.c
	$(COMPILE) -include planner_inject_accessors.c -c $< -o $@


