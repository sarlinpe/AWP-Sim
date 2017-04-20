% Title:        ME5402 Project 1-2: Trajectory Planning
% File:         dynModel.m
% Date:         2017-04-20
% Author:       Nicolai Domingo Nielsen (A0164015R)
%               Paul-Edouard Sarlin (A0153124U)
% Description:  Compute a step of the simulation, including dynamic model
%               of the manipulator and controller reponse.

function [ xdot ] = dynModel( t,x,robot,PID,lspb_cfg)

[thd, vd, ad] = lspb_get(t,lspb_cfg);

th1d = thd(1);
th2d = thd(2);
v1d = vd(1);
v2d = vd(2);
a1d = ad(1);
a2d = ad(2);
xdot=zeros(8,1);

%% Robot specification
M1 = robot.m1;
M2 = robot.m2;
L1 = robot.l1;
L2 = robot.l2;
Lc1 = robot.lc1;
Lc2 = robot.lc2;
I1 = robot.i1;
I2 = robot.i2;
g = 9.82;

%% Inertia Matrix
d11=M1*Lc1^2 + M2*(L1^2+Lc2^2+2*L1*Lc2*cos(x(4)))+I1+I2;
d12=M2*(Lc2^2 +L1*Lc2*cos(x(4)))+I2; 
d21=M2*(Lc2^2 +L1*Lc2*cos(x(4)))+I2;
d22=M2*Lc2^2+I2;
Dq=[d11 d12;d21 d22];

%% C Matrix
c11=-M2*L1*Lc2*sin(x(4)) * x(6); 
c12=-M2*L1*Lc2*sin(x(4)) * x(6) - M2*L1*Lc2*sin(x(4)) * x(5); 
c21=M2*L1*Lc2*sin(x(4)) * x(5); 
c22=0; 
Cq=[c11 c12;c21 c22];

%% G Matrix
g1=(M1*Lc1+M2*L1)*g*cos(x(3)) + M2*Lc2*g*cos(x(3)+x(4));
g2=M2*Lc2*g*cos(x(3)+x(4));
Gq=[g1;g2];
Gq_comp = Dq\Gq;

%% PID Control
% decoupled control input 
u1 = a1d + PID.Kp1*(th1d-x(3)) + PID.Kd1*(v1d-x(5)) + PID.Ki1*(x(1)) + Gq_comp(1); 
u2 = a2d + PID.Kp2*(th2d-x(4)) + PID.Kd2*(v2d-x(6)) + PID.Ki2*(x(2)) + Gq_comp(2); 
% actual input to the system
F=Dq*[u1;u2];

%% System states
xdot(1)=(th1d-x(3)); % theta1 integration 
xdot(2)=(th2d-x(4)); % theta2 integration

xdot(3)=x(5); %theta1-dot
xdot(4)=x(6); %theta2-dot

q2dot = Dq\(-Cq-Gq+F);
  
xdot(5)=q2dot(1); %theta1-2dot
xdot(6)=q2dot(2); %theta1-2dot

%control input 
xdot(7)=F(1);
xdot(8)=F(2);

end

