#!/usr/bin/env python
"""\
The MIT License (MIT)

Copyright (c) 2014 Sungeun K. Jeon

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
----------------------------------------------------------------------------------------
"""
"""\
G-code preprocessor for the grbl_sim.m MATLAB script. Parses the g-code program to a
specific file format for the MATLAB script to use. Based on PreGrbl by @chamnit.

How to use: When running this python script, it will process the g-code program under
the filename "test.gcode" (may be changed below) and produces a file called "matlab.gcode"
that the grbl_sim.m MATLAB script will search for and execute.
"""

import re
from math import *
from copy import *

# -= SETTINGS =-
filein = 'test.gcode'   # Input file name
fileout = 'matlab.gcode'  # Output file name
ndigits_in = 4 # inch significant digits after '.'
ndigits_mm = 2 # mm significant digits after '.'
# mm_per_arc_segment = 0.38 # mm per arc segment      
arc_tolerance = 0.00005*25.4
n_arc_correction = 20
inch2mm = 25.4 # inch to mm conversion scalar
verbose = False  # Verbose flag to show all progress
remove_unsupported = True   # Removal flag for all unsupported statements

# Initialize parser state
gc = { 'current_xyz' : [0,0,0], 
       'feed_rate' : 0,         # F0
       'motion_mode' : 'SEEK',  # G00
       'plane_axis' : [0,1,2],  # G17
       'inches_mode' : False,   # G21
       'inverse_feedrate_mode' : False, # G94
       'absolute_mode' : True}  # G90

def unit_conv(val) : # Converts value to mm
    if gc['inches_mode'] : val *= inch2mm
    return(val)

def fout_conv(val) : # Returns converted value as rounded string for output file.
    if gc['inches_mode'] : return( str(round(val/inch2mm,ndigits_in)) )
    else : return( str(round(val,ndigits_mm)) )

# Open g-code file
fin = open(filein,'r');
fout = open(fileout,'w');

# Iterate through g-code file
l_count = 0
for line in fin:
    l_count += 1 # Iterate line counter
    
    # Strip comments/spaces/tabs/new line and capitalize. Comment MSG not supported.
    block = re.sub('\s|\(.*?\)','',line).upper() 
    block = re.sub('\\\\','',block) # Strip \ block delete character
    block = re.sub('%','',block) # Strip % program start/stop character
    
    if len(block) == 0 :  # Ignore empty blocks
        
        print "Skipping: " + line.strip()
        
    else :  # Process valid g-code clean block. Assumes no block delete characters or comments
        
        g_cmd = re.findall(r'[^0-9\.\-]+',block) # Extract block command characters
        g_num = re.findall(r'[0-9\.\-]+',block) # Extract block numbers
        
        # G-code block error checks
        # if len(g_cmd) != len(g_num) : print block; raise Exception('Invalid block. Unbalanced word and values.')
        if 'N' in g_cmd :
            if g_cmd[0]!='N': raise Exception('Line number must be first command in line.')
            if g_cmd.count('N') > 1: raise Exception('More than one line number in block.')
            g_cmd = g_cmd[1:]  # Remove line number word
            g_num = g_num[1:]
        # Block item repeat checks? (0<=n'M'<5, G/M modal groups)
        
        # Initialize block state
        blk = { 'next_action' : 'DEFAULT',
                'absolute_override' : False,
                'target_xyz' : deepcopy(gc['current_xyz']),
                'offset_ijk' : [0,0,0],
                'radius_mode' : False, 
                'unsupported': [] }

        # Pass 1
        for cmd,num in zip(g_cmd,g_num) :
            fnum = float(num)
            inum = int(fnum)
            if cmd is 'G' :
                if   inum is 0 : gc['motion_mode'] = 'SEEK'
                elif inum is 1 : gc['motion_mode'] = 'LINEAR'
                elif inum is 2 : gc['motion_mode'] = 'CW_ARC'
                elif inum is 3 : gc['motion_mode'] = 'CCW_ARC'
                elif inum is 4 : blk['next_action'] = 'DWELL'
                elif inum is 17 : gc['plane_axis'] = [0,1,2]    # Select XY Plane
                elif inum is 18 : gc['plane_axis'] = [0,2,1]    # Select XZ Plane
                elif inum is 19 : gc['plane_axis'] = [1,2,0]    # Select YZ Plane
                elif inum is 20 : gc['inches_mode'] = True      
                elif inum is 21 : gc['inches_mode'] = False
                elif inum in [28,30] : blk['next_action'] = 'GO_HOME'
                elif inum is 53 : blk['absolute_override'] = True
                elif inum is 54 : pass
                elif inum is 80 : gc['motion_mode'] = 'MOTION_CANCEL'
                elif inum is 90 : gc['absolute_mode'] = True
                elif inum is 91 : gc['absolute_mode'] = False
                elif inum is 92 : blk['next_action'] = 'SET_OFFSET'
                elif inum is 93 : gc['inverse_feedrate_mode'] = True
                elif inum is 94 : gc['inverse_feedrate_mode'] = False
                else : 
                    print 'Unsupported command ' + cmd + num + ' on line ' + str(l_count) 
                    if remove_unsupported : blk['unsupported'].append(zip(g_cmd,g_num).index((cmd,num)))
            elif cmd is 'M' :
                if   inum in [0,1] : pass   # Program Pause
                elif inum in [2,30,60] : pass   # Program Completed
                elif inum is 3 : pass   # Spindle Direction 1
                elif inum is 4 : pass   # Spindle Direction -1
                elif inum is 5 : pass   # Spindle Direction 0
                else : 
                    print 'Unsupported command ' + cmd + num + ' on line ' + str(l_count) 
                    if remove_unsupported : blk['unsupported'].append(zip(g_cmd,g_num).index((cmd,num)))
            elif cmd is 'T' : pass      # Tool Number
            
        # Pass 2
        for cmd,num in zip(g_cmd,g_num) :
            fnum = float(num)         
            if   cmd is 'F' : gc['feed_rate'] = unit_conv(fnum)   # Feed Rate
            elif cmd in ['I','J','K'] : blk['offset_ijk'][ord(cmd)-ord('I')] = unit_conv(fnum) # Arc Center Offset
            elif cmd is 'N' : pass
            elif cmd is 'P' : p = fnum  # Misc value parameter
            elif cmd is 'R' : r = unit_conv(fnum); blk['radius_mode'] = True    # Arc Radius Mode
            elif cmd is 'S' : pass      # Spindle Speed
            elif cmd in ['X','Y','Z'] : # Target Coordinates
                if (gc['absolute_mode'] | blk['absolute_override']) :
                    blk['target_xyz'][ord(cmd)-ord('X')] = unit_conv(fnum)
                else :
                    blk['target_xyz'][ord(cmd)-ord('X')] += unit_conv(fnum)

        # Execute actions
        if   blk['next_action'] is 'GO_HOME' : 
            gc['current_xyz'] = deepcopy(blk['target_xyz']) # Update position      
        elif blk['next_action'] is 'SET_OFFSET' : 
            pass 
        elif blk['next_action'] is 'DWELL' :
            if p < 0 : raise Exception('Dwell time negative.')
        else : # 'DEFAULT'
            if gc['motion_mode'] is 'SEEK' : 
                fout.write('0 '+fout_conv(gc['feed_rate']))
                fout.write(' '+fout_conv(blk['target_xyz'][0]))
                fout.write(' '+fout_conv(blk['target_xyz'][1]))
                fout.write(' '+fout_conv(blk['target_xyz'][2]))
                fout.write('\n')  
                gc['current_xyz'] = deepcopy(blk['target_xyz']) # Update position
            elif gc['motion_mode'] is 'LINEAR' :
                fout.write('1 '+fout_conv(gc['feed_rate']))
                fout.write(' '+fout_conv(blk['target_xyz'][0]))
                fout.write(' '+fout_conv(blk['target_xyz'][1]))
                fout.write(' '+fout_conv(blk['target_xyz'][2]))
                fout.write('\n')  
                gc['current_xyz'] = deepcopy(blk['target_xyz']) # Update position
            elif gc['motion_mode'] in ['CW_ARC','CCW_ARC'] :
                axis = gc['plane_axis']
                
                # Convert radius mode to ijk mode
                if blk['radius_mode'] :
                    x = blk['target_xyz'][axis[0]]-gc['current_xyz'][axis[0]]
                    y = blk['target_xyz'][axis[1]]-gc['current_xyz'][axis[1]]
                    if not (x==0 and y==0) : raise Exception('Same target and current XYZ not allowed in arc radius mode.') 
                    h_x2_div_d = -sqrt(4 * r*r - x*x - y*y)/hypot(x,y)
                    if isnan(h_x2_div_d) : raise Exception('Floating point error in arc conversion')
                    if gc['motion_mode'] is 'CCW_ARC' : h_x2_div_d = -h_x2_div_d
                    if r < 0 : h_x2_div_d = -h_x2_div_d
                    blk['offset_ijk'][axis[0]] = (x-(y*h_x2_div_d))/2;
                    blk['offset_ijk'][axis[1]] = (y+(x*h_x2_div_d))/2;
                else :
                    radius = sqrt(blk['offset_ijk'][axis[0]]**2+blk['offset_ijk'][axis[1]]**2)

                center_axis0 = gc['current_xyz'][axis[0]]+blk['offset_ijk'][axis[0]]
                center_axis1 = gc['current_xyz'][axis[1]]+blk['offset_ijk'][axis[1]]
                linear_travel = blk['target_xyz'][axis[2]]-gc['current_xyz'][axis[2]]
                r_axis0 = -blk['offset_ijk'][axis[0]]
                r_axis1 = -blk['offset_ijk'][axis[1]]
                rt_axis0 = blk['target_xyz'][axis[0]] - center_axis0;
                rt_axis1 = blk['target_xyz'][axis[1]] - center_axis1;
                clockwise_sign = 1
                if gc['motion_mode'] is 'CW_ARC' : clockwise_sign = -1

                angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1)
                if gc['motion_mode'] is 'CW_ARC' :
                    if angular_travel >= 0 :
                        angular_travel -= 2*pi
                else :
                    if angular_travel <= 0 :
                        angular_travel += 2*pi
               
                millimeters_of_travel = sqrt((angular_travel*radius)**2 + abs(linear_travel)**2)

                mm_per_arc_segment = sqrt(4*(2*radius*arc_tolerance-arc_tolerance**2))
                segments = int(millimeters_of_travel/mm_per_arc_segment)
                print segments
                print l_count
                theta_per_segment = angular_travel/segments
                linear_per_segment = linear_travel/segments
                cos_T = 1-0.5*theta_per_segment*theta_per_segment
                sin_T = theta_per_segment-theta_per_segment**3/6
                print(fout_conv(mm_per_arc_segment))  
                print theta_per_segment*180/pi
                                
                arc_target = [0,0,0]
                arc_target[axis[2]] = gc['current_xyz'][axis[2]]

                count = 0
                for i in range(1,segments+1) :
                    if i < segments :
                        if count < n_arc_correction :
                            r_axisi = r_axis0*sin_T + r_axis1*cos_T       
                            r_axis0 = r_axis0*cos_T - r_axis1*sin_T
                            r_axis1 = deepcopy(r_axisi)
                            count += 1
                        else :
                            cos_Ti = cos((i-1)*theta_per_segment)
                            sin_Ti = sin((i-1)*theta_per_segment)
                            print n_arc_correction*(r_axis0 -( -blk['offset_ijk'][axis[0]]*cos_Ti + blk['offset_ijk'][axis[1]]*sin_Ti))
                            print n_arc_correction*(r_axis1 -( -blk['offset_ijk'][axis[0]]*sin_Ti - blk['offset_ijk'][axis[1]]*cos_Ti))
                            cos_Ti = cos(i*theta_per_segment)
                            sin_Ti = sin(i*theta_per_segment)
                            r_axis0 = -blk['offset_ijk'][axis[0]]*cos_Ti + blk['offset_ijk'][axis[1]]*sin_Ti
                            r_axis1 = -blk['offset_ijk'][axis[0]]*sin_Ti - blk['offset_ijk'][axis[1]]*cos_Ti
                            count = 0
                        arc_target[axis[0]] = center_axis0 + r_axis0
                        arc_target[axis[1]] = center_axis1 + r_axis1
                        arc_target[axis[2]] += linear_per_segment
                    else : 
                        arc_target = deepcopy(blk['target_xyz']) # Last segment at target_xyz
                    # Write only changed variables. 
                    fout.write('1 '+fout_conv(gc['feed_rate']))
                    fout.write(' '+fout_conv(arc_target[0]))
                    fout.write(' '+fout_conv(arc_target[1]))
                    fout.write(' '+fout_conv(arc_target[2]))
                    fout.write('\n')            
                    gc['current_xyz'] = deepcopy(arc_target) # Update position


print 'Done!'

# Close files
fin.close()
fout.close()