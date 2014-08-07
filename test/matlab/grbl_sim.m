% ----------------------------------------------------------------------------------------
% The MIT License (MIT)
% 
% Copyright (c) 2014 Sungeun K. Jeon
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
% THE SOFTWARE.
% ----------------------------------------------------------------------------------------

% This MATLAB script was written for the purpose of being a GRBL planner simulator. This
% simulator is a rough representation of the actual workings of Grbl on the Arduino, but
% was used to hone and proof the actual planner by providing quick visual feedback on its 
% functionality when experimented on. This script should be considered for educational 
% purposes only. This script requires and executes a pre-parsed g-code file from the 
% matlab_convert.py script that is in a specific non-g-code format. 

% There will be two figures plotted. The first is the line motion paths of the complete
% g-code program. The second is a representation of Grbl's planner buffer as new line
% motions are fed to it, plotting the velocity profiles the stepper motors will execute.
% Every time the user inputs an <Enter>, this feeds the simulator planner one line motion
% block. The left side is the first block in the buffer and the one that will be executed
% by the stepper module first. The right side is the end of the planner buffer, where the
% most recent streamed block is appended onto the planner buffer. Grbl's planner 
% optimizes the velocity profiles between the beginning and end of the buffer based on 
% the acceleration limits, intended velocity/feedrate, and line motion junction angles
% with their corresponding velocity limits (i.e. junctions with acute angles needs to come
% to a complete stop vs straight junctions can continue through at full speed.)

% ----------------------------------------------------------------------------------------


% Main function 
% NOTE: This is just a way to keep all functions in one place, but all non-global variables 
%   are cleared as soon as this script completes.
function main()

% Load pre-parsed gcode moves.
close all;
warning off;
clearvars -global
fid = fopen('matlab.gcode','r');
gcode = textscan(fid,'%d8%f32%f32%f32%f32');
nblock = length(gcode{1});

% Plot all g-code moves.
figure
line(gcode{3},gcode{4},gcode{5});
axis equal;
% axis([min(gcode{3}) max(gcode{3}) min(gcode{4}) max(gcode{4}) min(gcode{5}) max(gcode{5})]);
title('G-code programming line motions');
view(3);

% Set up figure for planner queue
figure

% Print help.
disp('<NOTE: Press Enter to Advance One G-Code Line Motion>');
disp('   BLUE line indicates completed planner blocks that require no recalculation.');
disp('   RED line indicates planner blocks that have been recalculated.');
disp('   GREEN line indicates the location of the BPLANNED pointer. Always a recalculated block.');
disp('   BLACK dotted-line and ''x'' indicates block nominal speed and max junction velocity, respectively.');
disp('   CYAN ''.'' indicates block initial entry speed.');

% Define Grbl settings.
BUFFER_SIZE = 18; % Number of planner blocks in its ring buffer.
steps_per_mm = 200;
seekrate = 2500; % mm/min
acceleration = [100 100 100]; % mm/sec^2 [ X Y Z ] axes
junction_deviation = 0.1; % mm. See Grbl documentation on this parameter.
inch_2_mm = 25.4;
ACCELERATION_TICKS_PER_SECOND = 100;

gcode{2} = gcode{2};
gcode{2} = inch_2_mm*gcode{2};
gcode{3} = inch_2_mm*gcode{3};
gcode{4} = inch_2_mm*gcode{4};
gcode{5} = inch_2_mm*gcode{5};

% Initialize blocks
block.steps = [];
block.step_event_count = [];
block.delta_mm = [];
block.millimeters = [];
block.acceleration = [];
block.speed = [];
block.nominal_speed = [];
block.max_entry_speed = [];
block.entry_speed = [];
block.recalculate_flag = false;
for i = 2:BUFFER_SIZE
  block(i) = block(1);
end

% Initialize planner
position = [0 0 0];
prev_unit_vec = [0 0 0];
previous_nominal_speed = 0;
pos = 0;

% BHEAD and BTAIL act as pointers to the block head and tail. 
% BPLANNED acts as a pointer of the location of the end of a completed/optimized plan.
bhead = 1;
btail = 1;
bplanned = 1;

global block bhead btail bplanned nind acceleration BUFFER_SIZE pos ACCELERATION_TICKS_PER_SECOND

% Main loop. Simulates plan_buffer_line(). All of the precalculations for the newest incoming
% block occurs here. Anything independent of the planner changes.
for i = 1:nblock
    
  target = round([gcode{3}(i) gcode{4}(i) gcode{5}(i)].*steps_per_mm);
  if gcode{1}(i) == 1
    feedrate = gcode{2}(i);
  else
    feedrate = seekrate;
  end
  
  nind = next_block_index(bhead);
  if nind == btail
    % Simulate a constantly full buffer. Move buffer tail. 
    bind = next_block_index(btail);
    % Push planned pointer if encountered. Prevents it from looping back around the ring buffer.    
    if btail == bplanned; bplanned = bind; end
    btail = bind;
  end

  block(bhead).steps = abs(target-position);
  block(bhead).step_event_count = max(block(bhead).steps);

  % Bail if this is a zero-length block
  if block(bhead).step_event_count == 0
    disp(['Zero-length block in line ',int2str(i)]);
  else

    % Compute path vector in terms of absolute step target and current positions  
    delta_mm = single((target-position)./steps_per_mm);
    block(bhead).millimeters = single(norm(delta_mm));
    inverse_millimeters = single(1/block(bhead).millimeters);
 
    % Compute path unit vector                            
    unit_vec = delta_mm/block(bhead).millimeters;
 
    % Calculate speed in mm/minute for each axis
    inverse_minute = single(feedrate * inverse_millimeters);
    block(bhead).speed = delta_mm*inverse_minute;
    block(bhead).nominal_speed = block(bhead).millimeters*inverse_minute;
    
    % Calculate block acceleration. Operates on absolute value of unit vector.
    [max_acc,ind] = max(abs(unit_vec)./acceleration); % Determine limiting acceleration
    block(bhead).acceleration = acceleration(ind)/abs(unit_vec(ind));   
    
    % Compute maximum junction speed
    block(bhead).max_entry_speed = 0.0;
    if previous_nominal_speed > 0.0 
      cos_theta = dot(-previous_unit_vec,unit_vec);                           
      if (cos_theta < 0.95) 
          block(bhead).max_entry_speed = min([block(bhead).nominal_speed,previous_nominal_speed]);
        if (cos_theta > -0.95) 
          sin_theta_d2 = sqrt(0.5*(1.0-cos_theta));
          block(bhead).max_entry_speed = min([block(bhead).max_entry_speed,sqrt(block(bhead).acceleration*3600*junction_deviation*sin_theta_d2/(1.0-sin_theta_d2))]);
        end
      end
    end
    
    block(bhead).entry_speed = 0; % Just initialize. Set accurately in the replanning function.  
    block(bhead).recalculate_flag = true; % Plotting flag to indicate this block has been updated.
    
    previous_unit_vec = unit_vec;
    previous_nominal_speed = block(bhead).nominal_speed;
    position = target;
    
    bhead = nind; % Block complete. Push buffer pointer.
    planner_recalculate();   
     
    plot_buffer_velocities();
  end
end
return

% Computes the next block index in the planner ring buffer
function block_index = next_block_index(block_index) 
global BUFFER_SIZE
  block_index = block_index + 1;
  if block_index > BUFFER_SIZE
    block_index = 1;
  end
return

% Computes the previous block index in the planner ring buffer
function block_index = prev_block_index(block_index) 
global BUFFER_SIZE
  block_index = block_index-1;
  if block_index < 1
    block_index = BUFFER_SIZE;
  end
return


% Planner recalculate function. The magic happens here.
function planner_recalculate(block) 

  global block bhead btail bplanned acceleration

  bind = prev_block_index(bhead);
  if bind == bplanned; return; end  % Bail, if only one block in buffer. Can't be operated on.
  
  % Reverse Pass: Coarsely maximize all possible deceleration curves back-planning from the last
  % block in buffer. Cease planning when the last optimal planned or tail pointer is reached.
  % NOTE: Forward pass will later refine and correct the reverse pass to create an optimal plan.
  next = [];
  curr = bind;  % Last block in buffer.

  % Calculate maximum entry speed for last block in buffer, where the exit speed is always zero.
  block(curr).entry_speed = min([block(curr).max_entry_speed,sqrt(2*block(curr).acceleration*60*60*block(curr).millimeters)]);

  bind = prev_block_index(bind); % Btail or second to last block
  if (bind == bplanned)
     % Only two plannable blocks in buffer. Reverse pass complete.
     % Check if the first block is the tail. If so, notify stepper module to update its current parameters.
     % if bind == btail; update_tail_block; end
  else    
    % Three or more plannable blocks in buffer. Loop it.
    while bind ~= bplanned % Loop until bplanned point hits. Replans to last plan point.
      next = curr; 
      curr = bind;
      bind = prev_block_index( bind ); % Previous block pointer.  
            
      % Check if the first block is the tail. If so, notify stepper module to update its current parameters.
      % if bind == btail; update_tail_block; end
                   
      % Compute maximum entry speed decelerating over the current block from its exit speed.
      if block(curr).entry_speed ~= block(curr).max_entry_speed  
        block(curr).recalculate_flag = true; % Plotting flag to indicate this block has been updated.
        block(curr).entry_speed = min([ block(curr).max_entry_speed,...
                                        sqrt(block(next).entry_speed^2 + 2*block(curr).acceleration*60*60*block(curr).millimeters)]);                                        
      end    

    end
  end

  % For two blocks, reverse pass is skipped, but forward pass plans second block entry speed
  % onward. This prevents the first, or the potentially executing block, from being over-written.
  % NOTE: Can never be bhead, since bsafe is always in active buffer.
  next = bplanned;  
  bind = next_block_index(bplanned); % Start at bplanned
  while bind ~= bhead
    curr = next;
    next = bind;
    
    % An acceleration block is always an optimally planned block since it starts from the first 
    % block's current speed or a maximum junction speed. Compute accelerations from this block 
    % and update the next block's entry speed.
    if (block(curr).entry_speed < block(next).entry_speed)
      % Once speed is set by forward planner, the plan for this block is finished and optimal.
      % Increment the planner pointer forward one block.      
        
      entry_speed = sqrt(block(curr).entry_speed^2 + 2*block(curr).acceleration*60*60*block(curr).millimeters);
      if (block(next).entry_speed > entry_speed)
        block(next).entry_speed = entry_speed;
        bplanned = bind;
      end

    end
    
    % Check if the next block entry speed is at max_entry_speed. If so, move the planned pointer, since
    % this entry speed cannot be improved anymore and all prior blocks have been completed and optimally planned.
    if block(next).entry_speed == block(next).max_entry_speed
      bplanned = bind;
    end
    
    % Recalculate trapezoid can be installed here, since it scans through all of the plannable blocks.
    % NOTE: Eventually this will only be computed when being executed.
    
    bind = next_block_index( bind );

  end
  
return

% ----------------------------------------------------------------------------------------
% PLOTTING FUNCTIONS

% Plots the entire buffer plan into a MATLAB figure to visual the plan.
%   BLUE line indicates completed planner blocks that require no recalculation.
%   RED line indicates planner blocks that have been recalculated.
%   GREEN line indicates the location of the BPLANNED pointer. Always a recalculated block.
%   BLACK dotted-line and 'x' indicates block nominal speed and max junction velocity, respectively.
%   CYAN '.' indicates block initial entry speed.
function plot_buffer_velocities() 
  global block bhead btail bplanned acceleration pos ACCELERATION_TICKS_PER_SECOND
  bind = btail;
  curr = [];
  next = [];
  
  pos_initial = 0;
  pos = 0;
  while bind ~= bhead
    curr = next;
    next = bind;
    hold on;
    if ~isempty(curr) 
      accel_d = estimate_acceleration_distance(block(curr).entry_speed, block(curr).nominal_speed, block(curr).acceleration*60*60);
      decel_d = estimate_acceleration_distance(block(curr).nominal_speed, block(next).entry_speed,-block(curr).acceleration*60*60);
      plateau_d = block(curr).millimeters-accel_d-decel_d;
      if plateau_d < 0
        accel_d = intersection_distance(block(curr).entry_speed, block(next).entry_speed, block(curr).acceleration*60*60, block(curr).millimeters);
        if accel_d < 0
          accel_d = 0;
        elseif accel_d > block(curr).millimeters
          accel_d = block(curr).millimeters;
        end
        plateau_d = 0;
      end 
      color = 'b'; 
      if (block(curr).recalculate_flag || block(next).recalculate_flag) 
        block(curr).recalculate_flag = false;
        color = 'r'; 
      end
      if bplanned == curr
        color = 'g'; 
      end
      
      plot_trap(pos,block(curr).entry_speed,block(next).entry_speed,block(curr).nominal_speed,block(curr).acceleration,accel_d,plateau_d,block(curr).millimeters,color)
      plot([pos pos+block(curr).millimeters],block(curr).nominal_speed*[1 1],'k:') % BLACK dotted indicates
      plot(pos,block(curr).max_entry_speed,'kx')

      pos = pos + block(curr).millimeters;
      plot(pos,block(next).entry_speed,'c.');
    end
    bind = next_block_index( bind );
  end

  accel_d = estimate_acceleration_distance(block(next).entry_speed, block(next).nominal_speed, block(next).acceleration*60*60);
  decel_d = estimate_acceleration_distance(block(next).nominal_speed, 0, -block(next).acceleration*60*60);
  plateau_d = block(next).millimeters-accel_d-decel_d;
  if plateau_d < 0
     accel_d = intersection_distance(block(next).entry_speed, 0, block(next).acceleration*60*60, block(next).millimeters);
     if accel_d < 0
       accel_d = 0;
     elseif accel_d > block(next).millimeters
       accel_d = block(next).millimeters;
     end
     plateau_d = 0;
   end 
   block(next).recalculate_flag = false;
   color = 'r';
   if bplanned == next
     color= 'g';
   end
   
  plot_trap(pos,block(next).entry_speed,0,block(next).nominal_speed,block(next).acceleration,accel_d,plateau_d,block(next).millimeters,color)
  plot([pos pos+block(next).millimeters],block(next).nominal_speed*[1 1],'k:')
  plot(pos,block(next).max_entry_speed,'kx')

  plot(pos,block(next).entry_speed,'.');
  pos = pos + block(next).millimeters;
  plot(pos,0,'rx');
  xlabel('mm');
  ylabel('mm/sec');
  xlim([pos_initial pos])
  title('Planner buffer optimized velocity profile');
  pause();
  hold off;
  
  plot(pos,0)
return


function d_a = estimate_acceleration_distance(initial_rate, target_rate, acceleration,rate_delta)
  d_a = (target_rate*target_rate-initial_rate*initial_rate)/(2*acceleration);
return

function d_i = intersection_distance(initial_rate, final_rate, acceleration, distance, rate_delta)
  d_i = (2*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/(4*acceleration);
return


% Simply plots the ac/de-celeration curves and plateaus of a trapezoid.
function plot_trap(pos,initial_rate,final_rate,rate,accel,accel_d,plateau_d,millimeters,color)

  dx = 1.0; % Line segment length
  linex = [pos]; liney = [initial_rate];

  % Acceleration
  np = floor(accel_d/dx);
  if np
    v = initial_rate;
    for i = 1:np
      v = sqrt(v^2+2*accel*60*60*dx);
      linex = [linex pos+i*dx];
      liney = [liney v];
    end
  end
  
  % Plateau
  v = sqrt(initial_rate^2 + 2*accel*60*60*accel_d);
  if v < rate
    rate = v;
  end
  linex = [linex pos+[accel_d accel_d+plateau_d]];
  liney = [liney [rate rate]];
  
  % Deceleration
  np = floor((millimeters-accel_d-plateau_d)/dx);
  if np
    v = rate;
    for i = 1:np
      v = sqrt(v^2-2*accel*60*60*dx);
      linex = [linex pos+i*dx+accel_d+plateau_d];
      liney = [liney v];
    end
  end        
        
  linex = [linex pos+millimeters];
  liney = [ liney final_rate];
  plot(linex,liney,color);

return



