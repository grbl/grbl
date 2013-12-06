t_= NaN
x_= NaN
y_= NaN
z_= NaN

splot 'HelloWorldSteps.dat' u 2:3:4:(dx= $2-x_,x_=$2,dy=$3-y_,y_=$3,dz=$4-z_,z_=$4,dt=$1-t_,t_=$1,sqrt(dx*dx+dy*dy+dz*dz)/(dt<0.01?0.01:dt)) title "Simulated steps with speed dependent coloring" with lines palette