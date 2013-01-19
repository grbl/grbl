require 'pp'

def estimate_acceleration_distance(initial_rate, target_rate, acceleration) 
    (target_rate*target_rate-initial_rate*initial_rate)/(2*acceleration)
end

def intersection_distance(initial_rate, final_rate, acceleration, distance) 
    (2*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/(4*acceleration)
end

ACCELERATION_TICKS_PER_SECOND = 20

def trapezoid_params(step_event_count, nominal_rate, rate_delta, entry_factor, exit_factor)
  initial_rate = (nominal_rate * entry_factor).round
  final_rate = (nominal_rate * exit_factor).round
  acceleration_per_minute = rate_delta*ACCELERATION_TICKS_PER_SECOND*60

  accelerate_steps = 
    estimate_acceleration_distance(initial_rate, nominal_rate, acceleration_per_minute).round;
  decelerate_steps = 
    estimate_acceleration_distance(nominal_rate, final_rate, -acceleration_per_minute).round;
  
  # Calculate the size of Plateau of Nominal Rate. 
  plateau_steps = step_event_count-accelerate_steps-decelerate_steps;
  
  # Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  # have to use intersection_distance() to calculate when to abort acceleration and start braking 
  # in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0)
    accelerate_steps = 
      intersection_distance(initial_rate, final_rate, acceleration_per_minute, step_event_count).round
    plateau_steps = 0;
  end
  
  accelerate_until = accelerate_steps;
  decelerate_after = accelerate_steps+plateau_steps;
  {:step_event_count => step_event_count,
   :initial_rate => initial_rate,
   :final_rate => final_rate,
   :nominal_rate => nominal_rate,
   :rate_delta => rate_delta,
   :accelerate_until => accelerate_until,
   :decelerate_after => decelerate_after}
end

def simulate_trapezoid(params)
  result = {}
  rate = params[:initial_rate]
  step_event = 0.0
  max_rate = 0
  while(step_event < params[:step_event_count]) do
    step_events_in_frame = rate/60.0/ACCELERATION_TICKS_PER_SECOND
    step_event += step_events_in_frame
    max_rate = rate if rate > max_rate
    if (step_event < params[:accelerate_until]) 
      rate += params[:rate_delta]
    elsif (step_event > params[:decelerate_after])
      if rate > params[:final_rate]
        rate -= params[:rate_delta]
      else
        return :underflow_at => step_event, :final_rate => rate, :max_rate => max_rate
      end
    end
#    puts "#{step_event} #{rate}"
  end 
  {:final_rate => rate, :max_rate => max_rate}
end

(10..100).each do |rate|
  (1..5).each do |steps|
    params = trapezoid_params(steps*1000, rate*100, 10, 0.1, 0.1)
    result = simulate_trapezoid(params)
  #  puts params.inspect
    line = "#{steps*10} final: #{result[:final_rate]} == #{params[:final_rate]} peak: #{result[:max_rate]} == #{params[:nominal_rate]} d#{params[:nominal_rate]-result[:max_rate]} "
    line << " (underflow at #{result[:underflow_at]})" if result[:underflow_at]
    puts line
  end
end