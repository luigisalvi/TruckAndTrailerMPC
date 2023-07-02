function xdot = stateFcn(x,u)
%Params
L=1; %Hitch length [m]
L1=8; %Truck length [m]
L2=14; %Trailer length [m]

%INPUT: u=[alpha v]'
% |u1| = alpha - Truck steering angle [rad]
% |u2| = v - Truck longitudinal velocity [m/s]

%STATES: x=[xp yp theta beta]'
% |x1| = xp - Center of the trailer rear axle on the x axis [m]
% |x2| = yp - Center of the trailer rear axle on the y axis [m] 
% |x3| = theta - Trailer orientation, global angle [rad]
% |x4| = beta - Truck orientation with respect to trailer [rad]

%OUTPUT: y=x=[x y theta beta] 

x1=x(1);
x2=x(2);
theta=x(3);
beta=x(4);

alpha=u(1);
v=u(2);

xdot=zeros(4,1);

xdot(1)= v*cos(beta)*(1+(L/L1)*tan(beta)*tan(alpha))*cos(theta);
xdot(2)= v*cos(beta)*(1+(L/L1)*tan(beta)*tan(alpha))*sin(theta);
xdot(3)= v*(sin(beta)/L2-(L/(L1*L2))*cos(beta)*tan(alpha));
xdot(4)= v*(tan(alpha)/L1-sin(beta)/L2+(L/(L1*L2))*cos(beta)*tan(alpha));

end