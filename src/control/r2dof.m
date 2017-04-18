function xdot = r2dof(t,x,thd_end,spec,Kpid)
%addpath('./trajectory_generator');


lspb_test %computing desired trajectory
[thdtest, vdtest, adtest] = lspb_get(t,lspb);  %thdtest becomes desired qs = thetas.
                             % which are computed for each ode45 t
                             % timestep.

th1d = thdtest(1);
th2d = thdtest(2);

a1d = adtest(1);
a2d = adtest(2);


xdot=zeros(8,1);
%% via-points
%th1s=ths(1); 
%th2s=ths(2);

%% Robot Specifications
M1=spec(3);
M2=spec(4);
ME=spec(5);
L1=spec(1);
L2=spec(2);
Lc1=spec(6);
Lc2=spec(7);
I1=spec(8);
I2=spec(9);
g=9.82;

%% Inertia Matrix
b11=M1*Lc1^2 + M2*(L1^2+Lc2^2+2*L1*Lc2*cos(x(4)))+I1+I2;
b12=M2*(Lc2^2 +L1*Lc2*cos(x(4)))+I2; 
b21=M2*(Lc2^2 +L1*Lc2*cos(x(4)))+I2;
b22=M2*Lc2^2+I2;
Bq=[b11 b12;b21 b22];

%% C Matrix
c11=-M2*L1*Lc2*sin(x(4)) * x(6); 
c12=-M2*L1*Lc2*sin(x(4)) * x(6) - M2*L1*Lc2*sin(x(4)) * x(5); 
c21=M2*L1*Lc2*sin(x(4)) * x(5); 
c22=0; 
Cq=[c11 c12;c21 c22];

%% Gravity Matrix
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
f1=a1d+Kp1*(th1d-x(3))-Kd1*x(5)+Ki1*(x(1)); 
f2=a2d+Kp2*(th2d-x(4))-Kd2*x(6)+Ki2*(x(2)); 
Fhat=[f1;f2];

F=Bq*Fhat; % actual input to the system
%% System states
xdot(1)=(th1d-x(3)); %dummy state of theta1 integration 
xdot(2)=(th2d-x(4)); %dummy state of theta2 integration

xdot(3)=x(5); %theta1-dot
xdot(4)=x(6); %theta2-dot

   q2dot=inv(Bq)*(-Cq-Gq+F);
  
xdot(5)=q2dot(1); %theta1-2dot
xdot(6)=q2dot(2); %theta1-2dot

%control input function output to outside computer program
xdot(7)=F(1);
xdot(8)=F(2);