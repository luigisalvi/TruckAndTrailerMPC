Ts=0.5; %[s]
Hp=12;
Hc=5;
x0=[0;0;0;0];
u0=[0;0];
yref=[1 -25 pi/2 0];

%Time vector
Tf=10; %[s]
t=0:Ts:Tf;
N=length(t);

%% NL-MPC init
nx=4; ny=4; nu=2;
nlobj= nlmpc(nx,ny,nu);

nlobj.Ts=Ts;
nlobj.PredictionHorizon=Hp;
nlobj.ControlHorizon=Hc;

nlobj.Model.StateFcn='stateFcn';
nlobj.Model.OutputFcn='outFcn';
nlobj.Jacobian.StateFcn='stateJac';
nlobj.Jacobian.OutputFcn='outJac';


%Names and Units
nlobj.ManipulatedVariables(1).Name='Steering angle';
nlobj.ManipulatedVariables(2).Name='Longitudinl velocity';
nlobj.ManipulatedVariables(1).Units='rad';
nlobj.ManipulatedVariables(2).Units='m/s';
nlobj.OutputVariables(1).Name='Center of the trailer axle on x-axis';
nlobj.OutputVariables(2).Name='Center of the trailer axle on y-axis';
nlobj.OutputVariables(3).Name='Trailer orientation';
nlobj.OutputVariables(4).Name='Truck orientation';
nlobj.OutputVariables(1).Units='m';
nlobj.OutputVariables(2).Units='m';
nlobj.OutputVariables(3).Units='rad';
nlobj.OutputVariables(4).Units='rad';

%% Constraints 
nlobj.ManipulatedVariables(1).Min= deg2rad(-45);
nlobj.ManipulatedVariables(1).Max= deg2rad(45);
nlobj.ManipulatedVariables(2).Min= -15;
nlobj.ManipulatedVariables(2).Max= 15;

nlobj.States(4).Min= deg2rad(-90);
nlobj.States(4).Max= deg2rad(90);
nlobj.OutputVariables(4).Min= deg2rad(-90);
nlobj.OutputVariables(4).Max= deg2rad(90);

%% Weights
nlobj.Weights.OutputVariables(1)=2;
nlobj.Weights.OutputVariables(4)=5;
%% Simulation loop 
xk=x0;
mv=u0;
validateFcns(nlobj,xk,mv)

%History Vectors Init
x_history=[];
mv_history=[];
y_history=[];
c_history=[];
j_history=[];
tic;

for i=0:N-1

    [mv,~,info]=nlmpcmove(nlobj,xk,mv,yref);
    xk=info.Xopt(2,:);
    y=info.Yopt(1,:);
    x_history=[x_history;xk];
    mv_history=[mv_history;mv'];
    y_history=[y_history;y];
    c_history=[c_history;toc];
    j_history=[j_history;info.Cost];
    tic; 

end


%% Plotting
figure('Name','Input signals')
subplot(211)
plot(t,rad2deg(mv_history(:,1)))
xlabel('Time [s]'), ylabel('\alpha [°]'), title('Steering angle')
hold on
plot(t, 45*ones(length(t)),'--r', t,-45*ones(length(t)),'--r')
grid on
subplot(212)
plot(t,mv_history(:,2))
hold on
plot(t, 15*ones(length(t)),'--r', t,-15*ones(length(t)),'--r')
xlabel('Time [s]'), ylabel('v [m/s]'), title('Longitudinl velocity')
grid on

figure('Name','Output')
subplot(411)
plot(t,y_history(:,1))
hold on
plot(t, yref(1)*ones(length(t)),'--r')
xlabel('Time [s]'), ylabel('x_p [m]'), title('Center of the trailer axle on x-axis')
xlim([0 Tf]);
grid minor
subplot(412)
plot(t,y_history(:,2))
hold on
plot(t, yref(2)*ones(length(t)),'--r')
xlabel('Time [s]'), ylabel('y_p [m]'), title('Center of the trailer axle on y-axis')
xlim([0 Tf]);
grid minor
subplot(413)
plot(t,rad2deg(y_history(:,3)))
hold on
plot(t, rad2deg(yref(3))*ones(length(t)),'--r')
xlabel('Time [s]'), ylabel('\theta [°]'), title('Trailer orientation')
xlim([0 Tf]);
grid minor
subplot(414)
plot(t,rad2deg(y_history(:,4)))
hold on
plot(t, rad2deg(yref(4))*ones(length(t)),'--r')
xlabel('Time [s]'), ylabel('\beta [°]'), title('Truck orientation')
xlim([0 Tf]);
grid minor

figure('Name','Cost Function')
plot(t, j_history)

figure('Name','Computational time')
plot(t,c_history)

figure('Name','XY-Plot')
plot(y_history(:,1), y_history(:,2))


