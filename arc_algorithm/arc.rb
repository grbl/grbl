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
    40.times { @pixels << '.'*40 }
  end

  def plot_pixel(x,y, c) 
    return if x<0 || y<0 || x>39 || y > 39
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

  # dP[x+1, y+1]: 2 (1 + x + y) 1+2x+1+2y
  # dP[x+1, y-1]: 2 (1 + x - y) 1+2x+1-2y
  # dP[x-1, y-1]: 2 (1 - x - y) 2-2x-2y 
  # dP[x-1, y+1]: 2 (1 - x + y) 2-2x+2x
   
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
    radius = radius
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
        
      dx = (y==0) ? -x.sign : y.sign*angular_direction
      dy = (x==0) ? -y.sign : -x.sign*angular_direction
      
      pp [[x,y],[dx,dy]]

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

  # A DDA-direct search circle interpolator. Optimal and impure
  def arc_supaoptimal(theta, angular_travel, radius) 
    radius = radius
    x = (sin(theta)*radius).round
    y = (cos(theta)*radius).round
    angular_direction = angular_travel.sign
    tx = (sin(theta+angular_travel)*(radius-0.5)).floor
    ty = (cos(theta+angular_travel)*(radius-0.5)).floor
    f = (x**2 + y**2 - radius**2).round
    x2 = 2*x
    y2 = 2*y
    dx = (y==0) ? -x.sign : y.sign*angular_direction
    dy = (x==0) ? -y.sign : -x.sign*angular_direction
    
    max_steps = (angular_travel.abs*radius*2).floor
    
    # dP[x+1,y]: 1 + 2 x
    # dP[x, y+1]: 1 + 2 y
    
    max_steps.times do |i|
      if i > 0
        plot_pixel(x+20, -y+20, "012"[i%3].chr)
      else
        plot_pixel(x+20, -y+20, "X")
      end
        
      
      raise "x2 out of range" unless x2 == 2*x
      raise "y2 out of range" unless y2 == 2*y
      f_should_be = (x**2+y**2-radius**2)
      if f.round != f_should_be.round
        show
        raise "f out of range. Is #{f}, should be #{f_should_be}"
        
      end

      if x.abs<y.abs
        x += dx
        
        f += 1+x2*dx
        x2 += 2*dx
        f_diagonal = f + 1 + y2*dy
        if  (f.abs >= f_diagonal.abs)
          y += dy
          dx = y.sign*angular_direction unless y == 0
          y2 += 2*dy
          f = f_diagonal
        end
        dy = -x.sign*angular_direction unless x == 0
      else
        y += dy
        f += 1+y2*dy
        y2 += 2*dy
        f_diagonal = f + 1 + x2*dx
        if  (f.abs >= f_diagonal.abs)
          x += dx
          dy = -x.sign*angular_direction unless x == 0
          x2 += 2*dx
          f = f_diagonal
        end
        dx = y.sign*angular_direction unless y == 0
      end
      break if x*ty.sign*angular_direction>=tx*ty.sign*angular_direction && y*tx.sign*angular_direction<=ty*tx.sign*angular_direction
    end
    plot_pixel(tx+20, -ty+20, "o")
    return {:tx => tx, :ty => ty, :x => x, :y => y}
  end

  # A DDA-direct search circle interpolator unrolled for each octant. Optimal and impure
  def arc_unrolled(theta, angular_travel, radius) 
    radius = radius
    x = (sin(theta)*radius).round
    y = (cos(theta)*radius).round
    angular_direction = angular_travel.sign
    tx = (sin(theta+angular_travel)*(radius-0.5)).floor
    ty = (cos(theta+angular_travel)*(radius-0.5)).floor
    f = (x**2 + y**2 - radius**2).round
    x2 = 2*x
    y2 = 2*y
    dx = (y==0) ? -x.sign : y.sign*angular_direction
    dy = (x==0) ? -y.sign : -x.sign*angular_direction
    
    max_steps = (angular_travel.abs*radius*2).floor
    
    # Quandrants of the circls
    #       \ 1|2 /
    #      8\ | / 3
    #         \|/
    # ---------|-----------
    #     7   /|\  4
    #       /  | \  
    #     / 6 | 5 \ 
    #            
    #
    #
    
    if angular_direction>0 # clockwise
      if x.abs<y.abs # quad 1,2,6,5
        if y>0 # quad 1,2
          while x<0 # quad 1  x+,y+
            x += 1        
            f += 1+x2
            x2 += 2
            f_diagonal = f + 1 + y2
            if  (f.abs >= f_diagonal.abs)
              y += 1
              y2 += 2
              f = f_diagonal
            end
          end
          while x>=0 # quad 2, x+, y-
            x += 1        
            f += 1+x2
            x2 += 2
            f_diagonal = f + 1 - y2
            if  (f.abs >= f_diagonal.abs)
              y -= 1
              y2 -= 2
              f = f_diagonal
            end
          end
        end
        if y<=0 # quad 6, 5
          while x<0 # quad 6 x-, y+
            x -= 1        
            f += 1-x2
            x2 -= 2
            f_diagonal = f + 1 + y2
            if  (f.abs >= f_diagonal.abs)
              y += 1
              y2 += 2
              f = f_diagonal
            end
          end
          while x>=0 # quad 5 x-, y-
            x -= 1        
            f += 1-x2
            x2 -= 2
            f_diagonal = f + 1 - y2
            if  (f.abs >= f_diagonal.abs)
              y -= 1
              y2 -= 2
              f = f_diagonal
            end
          end
        end
        # Quandrants of the circls
        #       \ 1|2 /
        #      8\ | / 3
        #         \|/
        # ---------|-----------
        #     7   /|\  4
        #       /  | \  
        #     / 6 | 5 \ 
      else 3 # quad 3,4,7,8
        if x>0 # quad 3,4
          while y>0 # quad 3  x+,y+
            x += 1        
            f += 1+x2
            x2 += 2
            f_diagonal = f + 1 + y2
            if  (f.abs >= f_diagonal.abs)
              y += 1
              y2 += 2
              f = f_diagonal
            end
          end
          while x>=0 # quad 2, x+, y-
            x += 1        
            f += 1+x2
            x2 += 2
            f_diagonal = f + 1 - y2
            if  (f.abs >= f_diagonal.abs)
              y -= 1
              y2 -= 2
              f = f_diagonal
            end
          end
        end
        if y<=0 # quad 6, 5
          while x<0 # quad 6 x-, y+
            x -= 1        
            f += 1-x2
            x2 -= 2
            f_diagonal = f + 1 + y2
            if  (f.abs >= f_diagonal.abs)
              y += 1
              y2 += 2
              f = f_diagonal
            end
          end
          while x>=0 # quad 5 x-, y-
            x -= 1        
            f += 1-x2
            x2 -= 2
            f_diagonal = f + 1 - y2
            if  (f.abs >= f_diagonal.abs)
              y -= 1
              y2 -= 2
              f = f_diagonal
            end
          end
      end  
        
      else
        y += dy
        f += 1+y2*dy
        y2 += 2*dy
        f_diagonal = f + 1 + x2*dx
        if  (f.abs >= f_diagonal.abs)
          x += dx
          dy = -x.sign*angular_direction unless x == 0
          x2 += 2*dx
          f = f_diagonal
        end
        dx = y.sign*angular_direction unless y == 0
      end
      break if x*ty.sign*angular_direction>=tx*ty.sign*angular_direction && y*tx.sign*angular_direction<=ty*tx.sign*angular_direction
    end
    plot_pixel(tx+20, -ty+20, "o")
    return {:tx => tx, :ty => ty, :x => x, :y => y}
  end

  
end

test = CircleTest.new
test.init

#test.arc_clean(0, Math::PI*2, 5)
(1..10000).each do |r|
  test.init
  data = test.arc_supaoptimal(2.9, Math::PI*1, r)
  if (data[:tx]-data[:x]).abs > 1 || (data[:ty]-data[:y]).abs > 1
    puts "r=#{r} fails target control" 
    pp data
    puts
  end
end

# test.init
# data = test.arc_supaoptimal(1.1, -Math::PI, 19)
# pp data

#test.pure_arc(0,1,1,4)

test.show
