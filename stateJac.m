function [A,Bmv] = stateJac(x,u)
%Params
L=1; %Hitch length [m]
L1=8; %Truck length [m]
L2=14; %Trailer length [m]

x1=x(1);
x2=x(2);
theta=x(3);
beta=x(4);

alpha=u(1);
v=u(2);

A=zeros(4,4);
Bmv=zeros(4,2);

A(1,3)=-v*cos(beta)*sin(theta)*((L*tan(alpha)*tan(beta))/L1 + 1);
A(1,4)=(L*v*cos(beta)*tan(alpha)*cos(theta)*(tan(beta)^2 + 1))/L1 - v*sin(beta)*cos(theta)*((L*tan(alpha)*tan(beta))/L1 + 1);
A(2,3)=v*cos(beta)*cos(theta)*((L*tan(alpha)*tan(beta))/L1 + 1);
A(2,4)=(L*v*cos(beta)*tan(alpha)*sin(theta)*(tan(beta)^2 + 1))/L1 - v*sin(beta)*sin(theta)*((L*tan(alpha)*tan(beta))/L1 + 1);
A(3,4)=v*(cos(beta)/L2 + (L*tan(alpha)*sin(beta))/(L1*L2));
A(4,4)=-v*(cos(beta)/L2 + (L*tan(alpha)*sin(beta))/(L1*L2));

Bmv(1,1)=(L*v*cos(beta)*tan(beta)*cos(theta)*(tan(alpha)^2 + 1))/L1;
Bmv(1,2)=cos(beta)*cos(theta)*((L*tan(alpha)*tan(beta))/L1 + 1);
Bmv(2,1)=(L*v*cos(beta)*tan(beta)*sin(theta)*(tan(alpha)^2 + 1))/L1;
Bmv(2,2)=cos(beta)*sin(theta)*((L*tan(alpha)*tan(beta))/L1 + 1);
Bmv(3,1)=-(L*v*cos(beta)*(tan(alpha)^2 + 1))/(L1*L2);
Bmv(3,2)=sin(beta)/L2 - (L*cos(beta)*tan(alpha))/(L1*L2);
Bmv(4,1)=v*((tan(alpha)^2 + 1)/L1 + (L*cos(beta)*(tan(alpha)^2 + 1))/(L1*L2));
Bmv(4,2)=tan(alpha)/L1 - sin(beta)/L2 + (L*cos(beta)*tan(alpha))/(L1*L2);
end

