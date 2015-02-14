#!/usr/bin/env python
"""\
Simple g-code streaming script for grbl

Provided as an illustration of the basic communication interface
for grbl. When grbl has finished parsing the g-code block, it will
return an 'ok' or 'error' response. When the planner buffer is full,
grbl will not send a response until the planner buffer clears space.

G02/03 arcs are special exceptions, where they inject short line 
segments directly into the planner. So there may not be a response 
from grbl for the duration of the arc.

---------------------
The MIT License (MIT)

Copyright (c) 2012 Sungeun K. Jeon

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
---------------------
"""

import serial
import time

# Open grbl serial port
s = serial.Serial('/dev/tty.usbmodem1811',115200)

# Open g-code file
f = open('grbl.gcode','r');

# Wake up grbl
s.write("\r\n\r\n")
time.sleep(2)   # Wait for grbl to initialize 
s.flushInput()  # Flush startup text in serial input

# Stream g-code to grbl
for line in f:
    l = line.strip() # Strip all EOL characters for consistency
    print 'Sending: ' + l,
    s.write(l + '\n') # Send g-code block to grbl
    grbl_out = s.readline() # Wait for grbl response with carriage return
    print ' : ' + grbl_out.strip()

# Wait here until grbl is finished to close serial port and file.
raw_input("  Press <Enter> to exit and disable grbl.") 

# Close file and serial port
f.close()
s.close()    