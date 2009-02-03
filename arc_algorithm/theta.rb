require 'pp'
include Math

def calc_theta(x,y)
  theta = atan(1.0*x/y.abs)
  return(theta) if(y>0)
  if (theta>0)
    return(PI-theta)
  else
    return(-PI-theta)
  end
end

pp calc_theta(5,0)/PI*180;

# (-180..180).each do |deg|
#   pp [deg, calc_theta(sin(1.0*deg/180*PI), cos(1.0*deg/180*PI))/PI*180]
# end