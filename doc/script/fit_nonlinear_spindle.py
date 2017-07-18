"""
---------------------
The MIT License (MIT)

Copyright (c) 2017 Sungeun K. Jeon for Gnea Research LLC

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


"""
This Python script produces a continuous piece-wise line fit of actual spindle speed over
programmed speed/PWM, which must be measured and provided by the user. A plot of the data
and line fit will be auto-generated and saved in the working directory as 'line_fit.png'.

REQUIREMENTS:
  - Python 2.7 or 3.x with SciPy, NumPy, and Matplotlib Python Libraries
  
  - For the most people, the easiest way to run this script is on the free cloud service
    https://repl.it/site/languages/python3. No account necessary. Unlimited runs. To use,
    go to the website and start the Python REPL. Copy and paste this script into the 
    browser editor. Click the 'Add New File' icon on the upper left side. This is very
    important. It places the REPL in multiple file mode and will enable viewing the plot.
    Click the 'Run' icon. The solution will be presented in the console on the right side,
    and the data plot will appear as a tab called 'line_fit.png'. You can edit the script
    directly in the browser and re-run the script as many times as you need. A free 
    account is only necessary if you want to save files on their servers.
  
  - For offline Python installs, most Mac and Linux computers have Python pre-installed 
    with the required libraries. If not, a quick google search will show you how to 
    install them. For Windows, Python installations are bit more difficult. Anaconda and 
    Pyzo seem to work well.

USAGE: 
  - First, make sure you are using the stock build of Grbl for the 328p processor. Most
    importantly, the SPINDLE_PWM_MAX_VALUE and SPINDLE_PWM_MIN_VALUE should be unaltered
    from defaults, otherwise change them back to 255.0 and 1.0 respectively for this test.
    
  - Next, program the max and min rpm Grbl settings to '$30=255' and '$31=1'. This sets
    the internal PWM values equal to 'S' spindle speed for the standard Grbl build.
         
  - Check if your spindle does not turn on at very low voltages by setting 'S' spindle
    speed to 'S1'. If it does not turn on or turns at a non-useful rpm, increase 'S' by 
    one until it does. Write down this 'S' value for later. You'll start the rpm data
    collection from this point onward and will need to update the SPINDLE_PWM_MIN_VALUE 
    in cpu_map.h afterwards.
         
  - Collect actual spindle speed with a tachometer or similar means over a range of 'S' 
    and PWM values. Start by setting the spindle 'S' speed to the minimum useful 'S' from 
    the last step and measure and record actual spindle rpm. Next, increase 'S' spindle 
    speed over equally sized intervals and repeat the measurement. Increments of 20 rpm 
    should be more than enough, but decrease increment size, if highly nonlinear. Complete
    the data collection the 'S' spindle speed equal to '$30' max rpm, or at the max useful 
    rpm, and record the actual rpm output. Make sure to collect rpm data all the way 
    throughout useful output rpm. The actual operating range within this model may be set 
    later within Grbl with the '$30' and '$31' settings.
    
  - In some cases, spindle PWM output can have discontinuities or not have a useful rpm
    in certain ranges. For example, a known controller board has the spindle rpm drop
    completely at voltages above ~4.5V. If you have discontinuities like this at the low
    or high range of rpm, simply trim them from the data set. Don't include them. For 
    Grbl to compensate, you'll need to alter the SPINDLE_PWM_MIN_VALUE and/or 
    SPINDLE_PWM_MAX_VALUE in cpu_map.h to where your data set ends. This script will 
    indicate if you need to do that in the solution output.
           
  - Keep in mind that spindles without control electronics can slow down drastically when 
    cutting and under load. How much it slows down is dependent on a lot of factors, such 
    as feed rate, chip load, cutter diameter, flutes, cutter type, lubricant/coolant, 
    material being cut, etc. Even spindles with controllers can still slow down if the 
    load is higher than the max current the controller can provide. It's recommended to 
    frequently re-check and measure actual spindle speed during a job. You can always use 
    spindle speed overrides to tweak it temporarily to the desired speed.

  - Edit this script and enter the measured rpm values and their corresponding 'S' spindle 
    speed values in the data arrays below. Set the number of piecewise lines you would 
    like to use, from one to four lines. For most cases, four lines is perfectly fine.
    In certain scenarios (laser engraving), this may significantly degrade performance and
    should be reduced if possible. 
  
  - Run the Python script. Visually assess the line fit from the plot. It will not likely 
    to what you want on the first go. Dial things in by altering the line fit junction 
    points 'PWM_pointX' in this script to move where the piecewise line junctions are 
    located along the plot x-axis. It may be desired to tweak the junction points so the 
    model solution is more accurate in the region that the spindle typically running. 
    Re-run the script and tweak the junction points until you are satified with the model.
    
  - Record the solution and enter the RPM_POINT and RPM_LINE values into config.h. Set the 
    number of piecewise lines used in this model in config.h. Also set the '$30' and '$31'
    max and min rpm values to the solution values or in a range between them in Grbl '$' 
    settings. And finally, alter the SPINDLE_PWM_MIN_VALUE in cpu_map.h, if your spindle 
    needs to be above a certain voltage to produce a useful low rpm.
    
  - Once the solution is entered. Recompile and flash Grbl. This solution model is only 
    valid for this particular set of data. If the machine is altered, you will need to 
    perform this experiment again and regenerate a new model here. 
  
OUTPUT: 
  The solver produces a set of values that define the piecewise fit and can be used by 
  Grbl to quickly and efficiently compute spindle PWM output voltage for a desired RPM.
  
  The first two are the RPM_MAX ($30) and RPM_MIN ($31) Grbl settings. These must be 
  programmed into Grbl manually or setup in defaults.h for new systems. Altering these 
  values within Grbl after a piece-wise linear model is installed will not change alter 
  model. It will only alter the range of spindle speed rpm values Grbl output. 
  
  For example, if the solver produces an RPM_MAX of 9000 and Grbl is programmed with 
  $30=8000, S9000 may be programmed, but Grbl will only produce the output voltage to run 
  at 8000 rpm. In other words, Grbl will only output voltages the range between 
  max(RPM_MIN,$31) and min(RPM_MAX,$30).
        
  The remaining values define the slopes and offsets of the line segments and the junction 
  points between line segments, like so for n_pieces=3:
        
    PWM_output = RPM_LINE_A1 * rpm - RPM_LINE_B1     [ RPM_MIN < rpm < RPM_POINT12 ]
    PWM_output = RPM_LINE_A2 * rpm - RPM_LINE_B2     [ RPM_POINT12 < rpm < RPM_POINT23 ]
    PWM_output = RPM_LINE_A3 * rpm - RPM_LINE_B3     [ RPM_POINT23 < rpm < RPM_MAX ]
    
  NOTE: The script solves in terms of PWM but the final equations and values are expressed 
  in terms of rpm in the form 'PWM = a*rpm - b'. 

"""

from scipy import optimize
import numpy as np

# ----------------------------------------------------------------------------------------
# Configure spindle PWM line fit solver

n_pieces = 4 # Number of line segments used for data fit. Only 1 to 4 line segments supported.

# Programmed 'S' spindle speed values. Must start with minimum useful PWM or 'S' programmed
# value and end with the maximum useful PWM or 'S' programmed value. Order of the array must
# be synced with the RPM_measured array below. 
# NOTE: ** DO NOT USE DATA FROM AN EXISTING PIECEWISE LINE FIT. USE DEFAULT GRBL MODEL ONLY. **
PWM_set = np.array([2,18,36,55,73,91,109,127,146,164,182,200,218,237,254], dtype=float)

# Actual RPM measured at the spindle. Must be in the ascending value and equal in length 
# as the PWM_set array. Must include the min and max measured rpm output in the first and 
# last array entries, respectively.
RPM_measured = np.array([213.,5420,7145,8282,9165,9765,10100,10500,10700,10900,11100,11250,11400,11550,11650], dtype=float)

# Configure line fit points by 'S' programmed rpm or PWM value. Values must be between 
# PWM_max and PWM_min. Typically, alter these values to space the points evenly between 
# max and min PWM range. However, they may be tweaked to maximize accuracy in the places 
# you normally operate for highly nonlinear curves. Plot to visually assess how well the 
# solution fits the data.
PWM_point1 = 20.0 # (S) Point between segments 0 and 1. Used when n_pieces >= 2.
PWM_point2 = 80.0  # (S) Point between segments 1 and 2. Used when n_pieces >= 3.
PWM_point3 = 150.0  # (S) Point between segments 2 and 3. Used when n_pieces = 4.

# ----------------------------------------------------------------------------------------

# Advanced settings

# The optimizer requires an initial guess of the solution. Change value if solution fails.
slope_i = 100.0;  # > 0.0 

PWM_max = max(PWM_set) # Maximum PWM set in measured range
PWM_min = min(PWM_set) # Minimum PWM set in measured range
plot_figure = True # Set to False, if matplotlib is not available.

# ----------------------------------------------------------------------------------------
# DO NOT ALTER ANYTHING BELOW.

def piecewise_linear_1(x,b,k1):
    return np.piecewise(x, [(x>=PWM_min)&(x<=PWM_max)], [lambda x:k1*(x-PWM_min)+b])
    
def piecewise_linear_2(x,b,k1,k2):
    c = [b, 
         b+k1*(PWM_point1-PWM_min)]
    funcs = [lambda x:k1*(x-PWM_min)+c[0], 
             lambda x:k2*(x-PWM_point1)+c[1]]
    conds = [(x<PWM_point1)&(x>=PWM_min),
             (x<=PWM_max)&(x>=PWM_point1)]
    return np.piecewise(x, conds, funcs)
    
def piecewise_linear_3(x,b,k1,k2,k3):
    c = [b, 
         b+k1*(PWM_point1-PWM_min),
         b+k1*(PWM_point1-PWM_min)+k2*(PWM_point2-PWM_point1)]
    funcs = [lambda x:k1*(x-PWM_min)+c[0], 
             lambda x:k2*(x-PWM_point1)+c[1],
             lambda x:k3*(x-PWM_point2)+c[2]]
    conds = [(x<PWM_point1)&(x>=PWM_min),
             (x<PWM_point2)&(x>=PWM_point1),
             (x<=PWM_max)&(x>=PWM_point2)]
    return np.piecewise(x, conds, funcs)
    
def piecewise_linear_4(x,b,k1,k2,k3,k4):
    c = [b, 
         b+k1*(PWM_point1-PWM_min),
         b+k1*(PWM_point1-PWM_min)+k2*(PWM_point2-PWM_point1),
         b+k1*(PWM_point1-PWM_min)+k2*(PWM_point2-PWM_point1)+k3*(PWM_point3-PWM_point2)]
    funcs = [lambda x:k1*(x-PWM_min)+c[0], 
             lambda x:k2*(x-PWM_point1)+c[1],
             lambda x:k3*(x-PWM_point2)+c[2],
             lambda x:k4*(x-PWM_point3)+c[3]]
    conds = [(x<PWM_point1)&(x>=PWM_min),
             (x<PWM_point2)&(x>=PWM_point1),
             (x<PWM_point3)&(x>=PWM_point2),
             (x<=PWM_max)&(x>=PWM_point3)]
    return np.piecewise(x, conds, funcs)

# ----------------------------------------------------------------------------------------

print("\nCONFIG:")
print("  N_pieces: %i" % n_pieces)
print("  PWM_min: %.1f" % PWM_min)
print("  PWM_max: %.1f" % PWM_max)
if n_pieces > 1:
  print("  PWM_point1: %.1f" % PWM_point1)
if n_pieces > 2:
  print("  PWM_point2: %.1f" % PWM_point2)
if n_pieces > 3:
  print("  PWM_point3: %.1f" % PWM_point3)
print("  N_data: %i" % len(RPM_measured))
print("  PWM_set: ", PWM_set)
print("  RPM_measured: ", RPM_measured)
  
if n_pieces == 1:
  piece_func = piecewise_linear_1
  p_initial = [RPM_measured[0],slope_i]

  p , e = optimize.curve_fit(piece_func, PWM_set, RPM_measured, p0=p_initial)
  a = [p[1]]
  b = [ p[0]-p[1]*PWM_min]
  rpm = [ p[0],
          p[0]+p[1]*(PWM_point1-PWM_min)]

elif n_pieces == 2:
  piece_func = piecewise_linear_2
  p_initial = [RPM_measured[0],slope_i,slope_i]

  p , e = optimize.curve_fit(piece_func, PWM_set, RPM_measured, p0=p_initial)
  a = [p[1],p[2]]
  b = [ p[0]-p[1]*PWM_min,
        p[0]+p[1]*(PWM_point1-PWM_min)-p[2]*PWM_point1]
  rpm = [ p[0],
          p[0]+p[1]*(PWM_point1-PWM_min),
          p[0]+p[1]*(PWM_point1-PWM_min)+p[2]*(PWM_max-PWM_point1)]

elif n_pieces == 3:
  piece_func = piecewise_linear_3
  p_initial = [RPM_measured[0],slope_i,slope_i,slope_i]
  
  p , e = optimize.curve_fit(piece_func, PWM_set, RPM_measured, p0=p_initial)
  a = [p[1],p[2],p[3]]
  b = [ p[0]-p[1]*PWM_min,
        p[0]+p[1]*(PWM_point1-PWM_min)-p[2]*PWM_point1,
        p[0]+p[1]*(PWM_point1-PWM_min)+p[2]*(PWM_point2-PWM_point1)-p[3]*PWM_point2]
  rpm = [ p[0],
          p[0]+p[1]*(PWM_point1-PWM_min),
          p[0]+p[1]*(PWM_point1-PWM_min)+p[2]*(PWM_point2-PWM_point1),
          p[0]+p[1]*(PWM_point1-PWM_min)+p[2]*(PWM_point2-PWM_point1)+p[3]*(PWM_max-PWM_point2) ]

elif n_pieces == 4:
  piece_func = piecewise_linear_4
  p_initial = [RPM_measured[0],slope_i,slope_i,slope_i,slope_i]
  
  p , e = optimize.curve_fit(piece_func, PWM_set, RPM_measured, p0=p_initial)
  a = [p[1],p[2],p[3],p[4]]
  b = [ p[0]-p[1]*PWM_min,
        p[0]+p[1]*(PWM_point1-PWM_min)-p[2]*PWM_point1,
        p[0]+p[1]*(PWM_point1-PWM_min)+p[2]*(PWM_point2-PWM_point1)-p[3]*PWM_point2,
        p[0]+p[1]*(PWM_point1-PWM_min)+p[2]*(PWM_point2-PWM_point1)+p[3]*(PWM_point3-PWM_point2)-p[4]*PWM_point3 ]
  rpm = [ p[0],
          p[0]+p[1]*(PWM_point1-PWM_min),
          p[0]+p[1]*(PWM_point1-PWM_min)+p[2]*(PWM_point2-PWM_point1),
          p[0]+p[1]*(PWM_point1-PWM_min)+p[2]*(PWM_point2-PWM_point1)+p[3]*(PWM_point3-PWM_point2),
          p[0]+p[1]*(PWM_point1-PWM_min)+p[2]*(PWM_point2-PWM_point1)+p[3]*(PWM_point3-PWM_point2)+p[4]*(PWM_max-PWM_point3) ]
      
else :
  print("ERROR: Unsupported number of pieces. Check and alter n_pieces")
  quit()

print("\nSOLUTION:\n\n[Update these #define values and uncomment]\n[ENABLE_PIECEWISE_LINEAR_SPINDLE in config.h.]")
print("#define N_PIECES %.0f" % n_pieces)
print("#define RPM_MAX %.1f" % rpm[-1])
print("#define RPM_MIN %.1f" % rpm[0])

if n_pieces > 1:
  print("#define RPM_POINT12 %.1f" % rpm[1])
if n_pieces > 2:
  print("#define RPM_POINT23 %.1f" %rpm[2])
if n_pieces > 3:
  print("#define RPM_POINT34 %.1f" %rpm[3])

print("#define RPM_LINE_A1 %.6e" % (1./a[0]))
print("#define RPM_LINE_B1 %.6e" % (b[0]/a[0]))
if n_pieces > 1:
  print("#define RPM_LINE_A2 %.6e" % (1./a[1]))
  print("#define RPM_LINE_B2 %.6e" % (b[1]/a[1]))
if n_pieces > 2:
  print("#define RPM_LINE_A3 %.6e" % (1./a[2]))
  print("#define RPM_LINE_B3 %.6e" % (b[2]/a[2]))
if n_pieces > 3:
  print("#define RPM_LINE_A4 %.6e" % (1./a[3]))
  print("#define RPM_LINE_B4 %.6e" % (b[3]/a[3]))

print("\n[To operate over full model range, manually write these]")
print("['$' settings or alter values in defaults.h. Grbl will]")
print("[operate between min($30,RPM_MAX) and max($31,RPM_MIN)]")
print("$30=%.1f (rpm max)" % rpm[-1])
print("$31=%.1f (rpm min)" % rpm[0])

if (PWM_min > 1)|(PWM_max<255):
  print("\n[Update the following #define values in cpu_map.h]")
  if (PWM_min >1) :
    print("#define SPINDLE_PWM_MIN_VALUE %.0f" % PWM_min)
  if PWM_max <255:
    print("#define SPINDLE_PWM_MAX_VALUE %.0f" % PWM_max)
else:
  print("\n[No cpu_map.h changes required.]")
print("\n")

test_val = (1./a[0])*rpm[0] - (b[0]/a[0])
if test_val < 0.0 :
  print("ERROR: Solution is negative at RPM_MIN. Adjust junction points or increase n_pieces.\n")

if plot_figure:
  import matplotlib
  matplotlib.use("Agg")
  import matplotlib.pyplot as plt
  
  fig = plt.figure()
  ax = fig.add_subplot(111)
  xd = np.linspace(PWM_min, PWM_max, 10000)
  ax.plot(PWM_set, RPM_measured, "o")
  ax.plot(xd, piece_func(xd, *p),'g')
  plt.xlabel("Programmed PWM")
  plt.ylabel("Measured RPM")
  
  # Check solution by plotting in terms of rpm.
#   x = np.linspace(rpm[0], rpm[1], 10000)
#   ax.plot((1./a[0])*x-(b[0]/a[0]),x,'r:')
#   if n_pieces > 1:
#     x = np.linspace(rpm[1], rpm[2], 10000)
#     ax.plot((1./a[1])*x-(b[1]/a[1]),x,'r:')
#   if n_pieces > 2:
#     x = np.linspace(rpm[2], rpm[3], 10000)
#     ax.plot((1./a[2])*x-(b[2]/a[2]),x,'r:')
#   if n_pieces > 3:
#     x = np.linspace(rpm[3], rpm[-1], 10000)
#     ax.plot((1./a[3])*x-(b[3]/a[3]),x,'r:')

  fig.savefig("line_fit.png")
