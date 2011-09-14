#!/usr/bin/env python
"""\
G-code preprocessor for grbl (BETA!)
- Converts G02/03 arcs to G01 linear interpolations
- Removes comments, block delete characters, and line numbers
- Removes spaces and capitalizes commands
- Minor input error checking
- OPTIONAL: Remove unsupported grbl G and M commands

TODO: 
- Number precision truncation
- Arc conversion option
- More robust error checking
- Improve interface to command line options
- Improve g-code parsing to NIST standards
- Fix problem with inverse feed rates
- Positioning updates may not be correct on grbl. Need to check.

Based on GRBL 0.7b source code by Simen Svale Skogsrud

By: Sungeun (Sonny) Jeon
Version: 20100825
"""
import re
from math import *
from copy import *

# -= SETTINGS =-
filein = 'test.gcode'   # Input file name
fileout = 'grbl.gcode'  # Output file name
ndigits_in = 4 # inch significant digits after '.'
ndigits_mm = 2 # mm significant digits after '.'
mm_per_arc_segment = 0.1 # mm per arc segment      
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
                gc['current_xyz'] = deepcopy(blk['target_xyz']) # Update position
            elif gc['motion_mode'] is 'LINEAR' :
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
                               
                # Compute arc center, radius, theta, and depth parameters
                theta_start = atan2(-blk['offset_ijk'][axis[0]], -blk['offset_ijk'][axis[1]])
                theta_end = atan2(blk['target_xyz'][axis[0]] - blk['offset_ijk'][axis[0]] - gc['current_xyz'][axis[0]], \
                                  blk['target_xyz'][axis[1]] - blk['offset_ijk'][axis[1]] - gc['current_xyz'][axis[1]])
                if theta_end < theta_start : theta_end += 2*pi
                radius = hypot(blk['offset_ijk'][axis[0]], blk['offset_ijk'][axis[1]])
                depth = blk['target_xyz'][axis[2]]-gc['current_xyz'][axis[2]]
                center_x = gc['current_xyz'][axis[0]]-sin(theta_start)*radius
                center_y = gc['current_xyz'][axis[1]]-cos(theta_start)*radius
                
                # Compute arc incremental linear segment parameters
                angular_travel = theta_end-theta_start
                if gc['motion_mode'] is 'CCW_ARC' : angular_travel = angular_travel-2*pi
                millimeters_of_travel = hypot(angular_travel*radius, fabs(depth))
                if millimeters_of_travel is 0 : raise Exception('G02/03 arc travel is zero')
                segments = int(round(millimeters_of_travel/mm_per_arc_segment))
                if segments is 0 : raise Exception('G02/03 zero length arc segment')
#        ???    # if gc['inverse_feedrate_mode'] : gc['feed_rate'] *= segments
                theta_per_segment = angular_travel/segments
                depth_per_segment = depth/segments
                
                # Generate arc linear segments
                if verbose: print 'Converting: '+ block + ' : ' + str(l_count)
                fout.write('G01F'+fout_conv(gc['feed_rate']))
                if not gc['absolute_mode'] : fout.write('G90')    
                arc_target = [0,0,0]
                for i in range(1,segments+1) :
                    if i < segments : 
                        arc_target[axis[0]] = center_x + radius * sin(theta_start + i*theta_per_segment)
                        arc_target[axis[1]] = center_y + radius * cos(theta_start + i*theta_per_segment)
                        arc_target[axis[2]] = gc['current_xyz'][axis[2]] + i*depth_per_segment
                    else : 
                        arc_target = deepcopy(blk['target_xyz']) # Last segment at target_xyz
                    # Write only changed variables. 
                    if arc_target[0] != gc['current_xyz'][0] : fout.write('X'+fout_conv(arc_target[0]))
                    if arc_target[1] != gc['current_xyz'][1] : fout.write('Y'+fout_conv(arc_target[1]))
                    if arc_target[2] != gc['current_xyz'][2] : fout.write('Z'+fout_conv(arc_target[2]))
                    fout.write('\n')            
                    gc['current_xyz'] = deepcopy(arc_target) # Update position
                if not gc['absolute_mode'] : fout.write('G91\n')    
                
        # Rebuild original gcode block sans line numbers, extra characters, and unsupported commands
        if gc['motion_mode'] not in ['CW_ARC','CCW_ARC'] :
            if remove_unsupported and len(blk['unsupported']) : 
                for i in blk['unsupported'][::-1] : del g_cmd[i]; del g_num[i]
            out_block = "".join([i+j for (i,j) in zip(g_cmd,g_num)]) 
            if len(out_block) : 
                if verbose : print "Writing: " + out_block + ' : ' + str(l_count)
                fout.write(out_block + '\n')

print 'Done!'

# Close files
fin.close()
fout.close()