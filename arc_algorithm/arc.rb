require "pp"
# A tiny ruby script to design and test the implementation of the arc-support

class Numeric
  # -1 if the number < 0, 1 if number >= 0
  def sign
    return 0 if self == 0
    (self < 0) ? -1 : 1
  end
end

class CircleTest
  include Math
  
  def init
    @pixels = []
    @tool_position = [14,14]
    30.times { @pixels << '.'*30 }
  end

  def plot_pixel(x,y, c) 
    return if x<0 || y<0 || x>29 || y > 29
    @pixels[y] = @pixels[y][0..x][0..-2]+c+@pixels[y][(x+1)..-1]
  end
  
  def show
    @pixels.each do |line|
      puts line.gsub('.','. ').gsub('0','0 ').gsub('1','1 ').gsub('2','2 ').gsub('X','X ').gsub('o','o ')
    end
  end
  
  # dP[x+1,y]: 1 + 2 x
  # dP[x-1,y]: 1 - 2 x
  # dP[x, y+1]: 1 + 2 y
  # dP[x, y-1]: 1 - 2 y

  # dP[x+1, y+1]: 2 (1 + x + y)
  # dP[x+1, y-1]: 2 (1 + x - y)
  # dP[x-1, y-1]: 2 (1 - x - y)
  # dP[x-1, y+1]: 2 (1 - x + y)
  
  # dP[x+a, y+b]: |dx| - 2*dx*x + |dy| + 2*dy*y
  
  # Algorithm from the wikipedia aricle on the Midpoint circle algorithm.
  def raster_circle(radius)
    f = 1-radius
    ddF_x = 1
    ddF_y = -2*radius
    x = 0
    y = radius
    while x<=y
      if f>0
        y -= 1
        ddF_y += 2
        f += ddF_y
      end
      x += 1
      ddF_x += 2
      f += ddF_x
      plot_pixel(x+14,-y+14,'X')
    end    
    x += 1
    ddF_x += 2
    f += ddF_x
    while y>0
      if f<0
        x += 1
        ddF_x += 2
        f += ddF_x
      end
      y -= 1
      ddF_y += 2
      f += ddF_y
      plot_pixel(x+14,-y+14,'o')
    end
    
  end
  
  # An "ideal" arc. Computationally expensive, but always pure
  def pure_arc(theta, segment, angular_direction, radius) 
    var = [(sin(theta)*(radius)), (cos(theta)*(radius))]
    error_sum = 0.0
    error_count = 0.0
    max_error = 0.0
    (PI*2*radius).ceil.times do
      if var[0].abs<var[1].abs 
        major_axis = 0
        minor_axis = 1
      else
        major_axis = 1
        minor_axis = 0
      end
      delta = [var[1].sign*angular_direction, -var[0].sign*angular_direction]
      delta[0] = angular_direction if delta[0] == 0
      delta[1] = angular_direction if delta[1] == 0

      var[major_axis] += delta[major_axis]
      var[minor_axis] = (sqrt((radius**2-var[major_axis]**2))*var[minor_axis].sign).round

      error_count += 1
      error = (var[minor_axis].abs-(sqrt((radius**2)-var[major_axis]**2)).abs).abs
      max_error = [max_error,error].max
      error_sum += error

      plot_pixel(var[0]+14, -var[1]+14, 'X')
    end
    puts "Average error: #{error_sum/error_count} Maximum eror: #{max_error}"
    
  end
  

  # A DDA-direct search circle interpolator. Optimal and impure
  def arc_clean(theta, angular_travel, radius) 
    x = (sin(theta)*radius).round
    y = (cos(theta)*radius).round
    angular_direction = angular_travel.sign
    tx = (sin(theta+angular_travel)*radius).round
    ty = (cos(theta+angular_travel)*radius).round
    f = (x**2 + y**2 - radius**2).round
    min_x = 0
    max_x = 0
    i = 0
    
    pp [x,y]
    
    while true
      if i > 0
        plot_pixel(x+14, -y+14, "012"[i%3].chr)
      else
        plot_pixel(x+14, -y+14, "X")
      end
        
      dx = (y==0) ? angular_direction : y.sign*angular_direction
      dy = (x==0) ? angular_direction : -x.sign*angular_direction

      if x.abs<y.abs
        f_straight = f + 1+2*x*dx
        f_diagonal = f_straight + 1+2*y*dy
        x += dx
        if  (f_straight.abs < f_diagonal.abs)
          f = f_straight
        else
          y += dy
          f = f_diagonal
        end
      else
        f_straight = f + 1+2*y*dy
        f_diagonal = f_straight + 1+2*x*dx
        y += dy
        if  (f_straight.abs < f_diagonal.abs)
          f = f_straight
        else
          x += dx
          f = f_diagonal
        end
      end
      
      f_should_be = (x**2+y**2-radius**2)
      if f.round != f_should_be.round
        raise "f out of range. Is #{f}, should be #{f_should_be}"
      end
      
      min_x = [x,min_x].min
      max_x = [x,max_x].max      
      break if (x.sign == tx.sign && y.sign == ty.sign) && (x.abs>=tx.abs) && (y.abs>=ty.abs)
      i += 1
    end
    puts "Target #{[tx,ty].inspect}"
    plot_pixel(tx+14, -ty+14, "o")
    
    pp [x,y]
    
    puts "Diameter: #{max_x-min_x}"
  end

  
end

test = CircleTest.new
test.init

test.arc_clean(0, -Math::PI, 5)

test.show
