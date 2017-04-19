function xdot = dynfun(t,x,thd_end,specs,Kpid)
addpath('./trajectory_generator');


lspb_obstacle_avoidance %computing desired trajectory
[thdtest, vdtest, adtest] = lspb_get(t,lspb);  %thdtest becomes desired qs = thetas.
                             % which are computed for each ode45 t
                             % timestep.

th1d = thdtest(1);
th2d = thdtest(2);

a1d = adtest(1);
a2d = adtest(2);

xdot=zeros(8,1);

%% Robot Specs
M1=specs(3);
M2=specs(4);
ME=specs(5);
L1=specs(1);
L2=specs(2);
Lc1=specs(6);
Lc2=specs(7);
I1=specs(8);
I2=specs(9);
g=9.82;

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
g1=(M1*Lc1+M2*L1)*g*cos(x(3)) + M2*Lc2*g*cos(x(3)+x(4));          %-(M1+M2)*g*L1*sin(x(3))-M2*g*L2*sin(x(3)+x(4)); 
g2=M2*Lc2*g*cos(x(3)+x(4));
Gq=[g1;g2];
%% PID Control
% PID parameters for theta 1 
Kp1=Kpid(1);
Kd1=Kpid(2);
Ki1=Kpid(3);
% PID parameters for theta 2 
Kp2=Kpid(4);
Kd2=Kpid(5);
Ki2=Kpid(6);
%decoupled control input 
u1=a1d+Kp1*(th1d-x(3))-Kd1*x(5)+Ki1*(x(1)); 
u2=a2d+Kp2*(th2d-x(4))-Kd2*x(6)+Ki2*(x(2)); 
Fhat=[u1;u2];

F=Dq*Fhat; % actual input to the system
%% System states
xdot(1)=(th1d-x(3)); % theta1 integration 
xdot(2)=(th2d-x(4)); % theta2 integration

xdot(3)=x(5); %theta1-dot
xdot(4)=x(6); %theta2-dot

   q2dot=inv(Dq)*(-Cq-Gq+F);
  
xdot(5)=q2dot(1); %theta1-2dot
xdot(6)=q2dot(2); %theta1-2dot

%control input 
xdot(7)=F(1);
xdot(8)=F(2);