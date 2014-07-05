GRBL SIM : by Jens Geisler

This directory contains an experimental Grbl simulator that compiles the main Grbl source code into a wrapped executable for use on a computer. No Arduino required. When the executable is run, the user should be able to interact with the Grbl simulator as if connected to an Arduino with Grbl.

WARNING: Grbl Sim is under heavy development. So many things may not work, or respond in ways unexpected. At the moment, this code is a proof-of-concept.

What can you do with Grbl Sim? 
- Simply checking out how Grbl works without needing an Arduino.
- Visualize a g-code program by having the simulator parse and execute to a GUI. Fluctuations in feed rates by the acceleration planner can be viewed as well.
- A powerful debugging tool for development.
- Each of the AVR functions are replaced with dummy functions, like the stepper ISR. These could be written to whatever you need. For example, output simulated step pulses over time and examine its performance.

Realtime modifications by Adam Shelly: 

  Simulates Atmel hardware in separate thread.  Runs in aproximate realtime.  
  
  On Linux, use `socat PTY,raw,link=/dev/ttyFAKE,echo=0 "EXEC:'./grbl_sim.exe -n -s step.out -b block.out',pty,raw,echo=0"` to create a fake serial port connected to the simulator.  This is useful for testing grbl interface software.
  
