import random
import serial
import time
ser = serial.Serial('/dev/tty.usbmodem24111', 115200, timeout=0.001)
time.sleep(1)
outstanding = 0
data = ''
while True:
    time.sleep(0.1)
    data += ser.read()
    pos = data.find('\n')
    if pos == -1:
        line = ''
    else:
        line = data[0:pos + 1]
        data = data[pos + 1:]
    if line == '' and outstanding < 3:
        while outstanding < 3:
            ser.write("G0 Z%0.3f\n" % (0.01 * (random.random() - 0.5)))
            #ser.write("M3\n")
            outstanding += 1
        continue
    if line == 'ok\r\n':
        outstanding -= 1
    print outstanding, repr(line.rstrip())