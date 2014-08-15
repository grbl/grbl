 sudo /home/markus/arduino-1.0.1/hardware/tools/avrdude64 -C/home/markus/arduino-1.0.1/hardware/tools/avrdude.conf -p m328p -cjtag2isp -P usb -U flash:w:grbl.hex
 less flash.sh
 sudo /home/markus/arduino-1.0.1/hardware/tools/avrdude64 -C/home/markus/arduino-1.0.1/hardware/tools/avrdude.conf -p m328p -cjtag2isp -P usb -D -U flash:w:grbl.hex
