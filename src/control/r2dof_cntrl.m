close all
clear all
clc
%addpath('./trajectory_generator');
%% Initialization
th_int=[pi/4 -pi/4]; %initial positions
ths=[0 0]; % dummy set-points

lspb_test
thd_init = q(1,:);  %initial value for desired theta1 and 2
thdotd_init = v(1,:); %initial value for desired thetadot1 and 2
thdotdotd_init = a(1,:); %initial value for desired thetadotdot1 and 2



x0=[0 0 thd_init thdotd_init thdotdotd_init]; %states initial values
Ts=[0 20]; %time span

%% Robot Specifications
L1=6.5; %link 1
L2=7.5; %link 2
Lc1=L1/2; %link 1 mass center dist
rho_steel = 7850; %density of steel
Acr1 = 0.0196; % cross section area of stair 1
Acr2 = 0.0196; % cross section area of stair 2
M1=rho_steel*L1*Acr1; %mass 1
Mc2=rho_steel*L2*Acr2; %mass 2
ME=150; %platform mass (End effector mass)
M2=ME+Mc2; %mass of end effector and stair 2
Lc2 = ((L2/2)*Mc2 + L2*ME) / (Mc2+ME); %length from joint 2 to center of mass
I1 = 3520.83;
I2 = 3520.83;
spec=[L1 L2 M1 M2 ME Lc1 Lc2 I1 I2];
%% PID Parameters
% PID parameters for theta 1
Kp1=15;
Kd1=7;
Ki1=10;
% PID parameters for theta 2
Kp2=15;
Kd2=10;
Ki2=10;
Kpid=[Kp1 Kd1 Ki1 Kp2 Kd2 Ki2];
%% ODE solving
% opt1=odeset('RelTol',1e-10,'AbsTol',1e-20,'NormControl','off');

[T,X] = ode45(@(t,x) r2dof(t,x,lspb_get(t,lspb),spec,Kpid),Ts,x0);

% function qqqq = myfun(t)
% thd_end = q(t,:);
% end
%% Output
th1=X(:,3); %theta1 wavwform
th2=X(:,4); %theta2 wavwform

%torque inputs computation from the 7th,8th states inside ODE
F1=diff(X(:,7))./diff(T); 
F2=diff(X(:,8))./diff(T); 
tt=0:(T(end)/(length(F1)-1)):T(end); %time allocating for same dimension as F1

%xy kinematics
x1=Lc1.*cos(th1); % X1
y1=Lc1.*sin(th1); % Y1 
x2=L1.*cos(th1)+Lc2.*cos(th1+th2); % X2 
y2=L1.*sin(th1)+Lc2.*sin(th1+th2); % Y2


%regeneration of desired trajectory outside r2dof function
[thdtestNew, vNew, aNew] = lspb_get(T,lspb);
thdtest1 = thdtestNew(:,1);
thdtest2 = thdtestNew(:,2);

%F1d=diff(aNew(:,1))./diff(T); 


%theta1 error plot
plot(T,thdtest1-th1) % Difference between final position angle and computed angles with controller
grid
title('Theta-1 error')
ylabel('theta1 error (rad)')
xlabel('time (sec)')
%theta2 error plot
figure
plot(T,thdtest2-th2)
grid
title('Theta-2 error')
ylabel('theta2 error (rad)')
xlabel('time (sec)')
%torque1 plot
figure
plot(tt,F1)
grid
title('Torque of theta 1')
ylabel('theta1 torque')
xlabel('time (sec)')
%torque2 plot
figure
plot(tt,F2)
grid
title('Torque of theta 2')
ylabel('theta2 torque')
xlabel('time (sec)')
%theta1 plot
figure
plot(T,thdtest1)
hold on 
plot(T,th1)
grid
title('Theta 1 desired')
legend('Reference','Controlled')
ylabel('Theta')
xlabel('time (sec)')
%theta2 plot
figure
plot(T,thdtest2)
hold on 
plot(T,th2)
grid
title('Theta 2 desired')
legend('Reference','Controlled')
ylabel('Theta')
xlabel('time (sec)')
%theta1 vs theta2 plot
figure
plot(thdtest1,thdtest2)
hold on 
plot(th1,th2)
legend('Reference Theta','Controlled Theta')
grid
title('Theta 1 vs 2')
ylabel('theta2 torque')
xlabel('time (sec)')


 