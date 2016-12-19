#!/usr/bin/env python
"""\

Stream g-code to grbl controller

This script differs from the simple_stream.py script by 
tracking the number of characters in grbl's serial read
buffer. This allows grbl to fetch the next line directly
from the serial buffer and does not have to wait for a 
response from the computer. This effectively adds another
buffer layer to prevent buffer starvation.

CHANGELOG:
- 20161212: Added push message feedback for simple streaming
- 20140714: Updated baud rate to 115200. Added a settings
  write mode via simple streaming method. MIT-licensed.

TODO: 
- Add runtime command capabilities

---------------------
The MIT License (MIT)

Copyright (c) 2012-2016 Sungeun K. Jeon

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
import re
import time
import sys
import argparse
# import threading

RX_BUFFER_SIZE = 128

# Define command line argument interface
parser = argparse.ArgumentParser(description='Stream g-code file to grbl. (pySerial and argparse libraries required)')
parser.add_argument('gcode_file', type=argparse.FileType('r'),
        help='g-code filename to be streamed')
parser.add_argument('device_file',
        help='serial device path')
parser.add_argument('-q','--quiet',action='store_true', default=False, 
        help='suppress output text')
parser.add_argument('-s','--settings',action='store_true', default=False, 
        help='settings write mode')        
args = parser.parse_args()

# Periodic timer to query for status reports
# TODO: Need to track down why this doesn't restart consistently before a release.
# def periodic():
#     s.write('?')
#     t = threading.Timer(0.1, periodic) # In seconds
#     t.start()

# Initialize
s = serial.Serial(args.device_file,115200)
f = args.gcode_file
verbose = True
if args.quiet : verbose = False
settings_mode = False
if args.settings : settings_mode = True

# Wake up grbl
print "Initializing grbl..."
s.write("\r\n\r\n")

# Wait for grbl to initialize and flush startup text in serial input
time.sleep(2)
s.flushInput()

# Stream g-code to grbl
l_count = 0
if settings_mode:
    # Send settings file via simple call-response streaming method. Settings must be streamed
    # in this manner since the EEPROM accessing cycles shut-off the serial interrupt.
    print "SETTINGS MODE: Streaming", args.gcode_file.name, " to ", args.device_file
    for line in f:
        l_count += 1 # Iterate line counter    
        # l_block = re.sub('\s|\(.*?\)','',line).upper() # Strip comments/spaces/new line and capitalize
        l_block = line.strip() # Strip all EOL characters for consistency
        if verbose: print 'SND: ' + str(l_count) + ':' + l_block,
        s.write(l_block + '\n') # Send g-code block to grbl
        while 1:
            grbl_out = s.readline().strip() # Wait for grbl response with carriage return
            if grbl_out.find('ok') < 0 and grbl_out.find('error') < 0 :
                print "\n  Debug: ",grbl_out,
            else : 
                if verbose: print 'REC:',grbl_out
                break
else:    
    # Send g-code program via a more agressive streaming protocol that forces characters into
    # Grbl's serial read buffer to ensure Grbl has immediate access to the next g-code command
    # rather than wait for the call-response serial protocol to finish. This is done by careful
    # counting of the number of characters sent by the streamer to Grbl and tracking Grbl's 
    # responses, such that we never overflow Grbl's serial read buffer. 
    g_count = 0
    c_line = []
    # periodic() # Start status report periodic timer
    for line in f:
        l_count += 1 # Iterate line counter
        # l_block = re.sub('\s|\(.*?\)','',line).upper() # Strip comments/spaces/new line and capitalize
        l_block = line.strip()
        c_line.append(len(l_block)+1) # Track number of characters in grbl serial read buffer
        grbl_out = '' 
        while sum(c_line) >= RX_BUFFER_SIZE-1 | s.inWaiting() :
            out_temp = s.readline().strip() # Wait for grbl response
            if out_temp.find('ok') < 0 and out_temp.find('error') < 0 :
                print "  Debug: ",out_temp # Debug response
            else :
                grbl_out += out_temp;
                g_count += 1 # Iterate g-code counter
                grbl_out += str(g_count); # Add line finished indicator
                del c_line[0] # Delete the block character count corresponding to the last 'ok'
        if verbose: print "SND: " + str(l_count) + " : " + l_block,
        s.write(l_block + '\n') # Send g-code block to grbl
        if verbose : print "BUF:",str(sum(c_line)),"REC:",grbl_out

# Wait for user input after streaming is completed
print "G-code streaming finished!\n"
print "WARNING: Wait until grbl completes buffered g-code blocks before exiting."
raw_input("  Press <Enter> to exit and disable grbl.") 

# Close file and serial port
f.close()
s.close()
