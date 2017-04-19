close all
clear all
clc
%addpath('./trajectory_generator');
addpath('./path_finder');
%% Initialize
th_init=[pi/3, -pi/4]; %initial positions

%Type of test, load of desired trajectory path
    %lspb_test
    lspb_obstacle_avoidance
    testcase = 'obs1';

thd_init = q(1,:);  %initial value for desired theta1 and 2
thdotd_init = v(1,:); %initial value for desired thetadot1 and 2
thdotdotd_init = a(1,:); %initial value for desired thetadotdot1 and 2

%x0=[0 0 thd_init thdotd_init thdotdotd_init]; %initial values
x0=[0 0 th_init 0 0 0 0]; %initial values
Ts=[0 240]; %time span for ode45

%% Robot Specs
L1=6.5; %link 1
L2=7.5; %link 2
Lc1=L1/2; %link 1 mass center dist
rho_steel = 7850; %density of steel
Acr1 = 0.0625; % cross section area of stair 1
Acr2 = 0.0400; % cross section area of stair 2
M1=rho_steel*L1*Acr1; %mass 1
Mc2=rho_steel*L2*Acr2; %mass 2
ME=150; %platform mass (End effector mass)
M2=ME+Mc2; %mass of end effector and stair 2
Lc2 = ((L2/2)*Mc2 + L2*ME) / (Mc2+ME); %length from joint 2 to center of mass
I1 = sqrt(Acr1)^4 / 12;
I2 = sqrt(Acr2)^4 / 12;
specs=[L1 L2 M1 M2 ME Lc1 Lc2 I1 I2];
%% PID Parameters
% PID parameters for theta 1
Kp1=17; %17
Kd1=9; %9
Ki1=11; %11
% PID parameters for theta 2
Kp2=18; %18
Kd2=13; %13
Ki2=12; %12
Kpid=[Kp1 Kd1 Ki1 Kp2 Kd2 Ki2];
%% Solving the ODE with ODE45

[T,X] = ode45(@(t,x) dynfun(t,x,lspb_get(t,lspb),specs,Kpid),Ts,x0);

%% Output results from solving
th1=X(:,3); % theta1 
th2=X(:,4); % theta2

%Torque inputs
F1=diff(X(:,7))./diff(T); 
F2=diff(X(:,8))./diff(T); 
tt=0:(T(end)/(length(F1)-1)):T(end); %time allocating for same dimension as F1

%xy kinematics for link 1 and 2
x1=L1.*cos(th1); %link1
y1=L1.*sin(th1); %link1
x2=L1.*cos(th1)+L2.*cos(th1+th2); %end effector
y2=L1.*sin(th1)+L2.*sin(th1+th2); %end effector

%regeneration of desired trajectory outside dynfun function
[thdtestNew, vNew, aNew] = lspb_get(T,lspb);
thdtest1 = thdtestNew(:,1);
thdtest2 = thdtestNew(:,2);

%xy kinematics desired
x1d=L1.*cos(thdtest1); % X1d
y1d=L1.*sin(thdtest1); % Y1d 
x2d=L1.*cos(thdtest1)+L2.*cos(thdtest1+thdtest2); % X2d
y2d=L1.*sin(thdtest1)+L2.*sin(thdtest1+thdtest2); % Y2d

% PID for joint 1
Kp1=14;
Kd1=8;
Ki1=13;
% PID for joint 2
Kp2=16;
Kd2=11;
Ki2=9;

%theta1 error plot
figure 
plot(T,thdtest1-th1) % Difference between final position angle and computed angles with controller
grid
ylabel('$\theta_1$ error [rad]','Interpreter','Latex')
xlabel('time [sec]','Interpreter','Latex')
set(get(gca,'Children'), 'LineWidth', 1.5)
plotopt
E1 = ['E1_' testcase '_Kp1_' num2str(Kp1) '_Kd1_' num2str(Kd1) '_Ki1_' num2str(Ki2) '_Kp2_' num2str(Kp2) '_Kd2_' num2str(Kd2) '_Ki2_' num2str(Ki2) ];
print(gcf,'-dpng',E1,'-r400')

%theta2 error plot
figure
plot(T,thdtest2-th2)
grid
ylabel('$\theta_2$ error [rad]','Interpreter','Latex')
xlabel('time [sec]','Interpreter','Latex')
set(get(gca,'Children'), 'LineWidth', 1.5)
plotopt
E2 = ['E2_' testcase '_Kp1_' num2str(Kp1) '_Kd1_' num2str(Kd1) '_Ki1_' num2str(Ki2) '_Kp2_' num2str(Kp2) '_Kd2_' num2str(Kd2) '_Ki2_' num2str(Ki2) ];
print(gcf,'-dpng',E2,'-r400')


%theta 1&2 error plot
figure
plot(T,thdtest1-th1) % Difference between final position angle and computed angles with controller
hold on
plot(T,thdtest2-th2)
grid
ylabel('$\theta$ error [rad]','Interpreter','Latex')
xlabel('time [sec]','Interpreter','Latex')
set(get(gca,'Children'), 'LineWidth', 1.5)
hl = legend('$\theta_1$ error','$\theta_2$ error');
set(hl,'Interpreter','Latex','Orientation','Vertical','FontSize',16);
plotopt
E12 = ['E12_' testcase '_Kp1_' num2str(Kp1) '_Kd1_' num2str(Kd1) '_Ki1_' num2str(Ki2) '_Kp2_' num2str(Kp2) '_Kd2_' num2str(Kd2) '_Ki2_' num2str(Ki2) ];
print(gcf,'-dpng',E12,'-r400')


%torque1 plot
figure
plot(tt,F1)
grid
ylabel('$\tau_1$ [N$\cdot$m]','Interpreter','Latex')
xlabel('time [sec]','Interpreter','Latex')
set(get(gca,'Children'), 'LineWidth', 1.5)
plotopt
t1 = ['t1_' testcase '_Kp1_' num2str(Kp1) '_Kd1_' num2str(Kd1) '_Ki1_' num2str(Ki2) '_Kp2_' num2str(Kp2) '_Kd2_' num2str(Kd2) '_Ki2_' num2str(Ki2) ];
print(gcf,'-dpng',t1,'-r400')

%torque2 plot
figure
plot(tt,F2)
grid
ylabel('$\tau_2$ [N$\cdot$m]','Interpreter','Latex')
xlabel('time [sec]','Interpreter','Latex')
set(get(gca,'Children'), 'LineWidth', 1.5)
plotopt
t2 = ['t2_' testcase '_Kp1_' num2str(Kp1) '_Kd1_' num2str(Kd1) '_Ki1_' num2str(Ki2) '_Kp2_' num2str(Kp2) '_Kd2_' num2str(Kd2) '_Ki2_' num2str(Ki2) ];
print(gcf,'-dpng',t2,'-r400')

%theta1 plot
 figure
 plot(T,thdtest1)
 hold on 
 plot(T,th1)
 grid
 ylabel('$\theta_1$ [rad]','Interpreter','Latex')
 xlabel('time [sec]','Interpreter','Latex')
 set(get(gca,'Children'), 'LineWidth', 1.5)
 hl = legend('Reference $\theta$','Controlled $\theta$');
 set(hl,'Interpreter','Latex','Orientation','Vertical','FontSize',16);
 plotopt
 theta1 = ['th1_' testcase '_Kp1_' num2str(Kp1) '_Kd1_' num2str(Kd1) '_Ki1_' num2str(Ki2) '_Kp2_' num2str(Kp2) '_Kd2_' num2str(Kd2) '_Ki2_' num2str(Ki2) ];
 print(gcf,'-dpng',theta1,'-r400')
 
 
%theta2 plot
 figure
 plot(T,thdtest2)
 hold on 
 plot(T,th2)
 grid
 ylabel('$\theta_2$ [rad]','Interpreter','Latex')
 xlabel('time [sec]','Interpreter','Latex')
 set(get(gca,'Children'), 'LineWidth', 1.5)
 hl = legend('Reference $\theta$','Controlled $\theta$');
 set(hl,'Interpreter','Latex','Orientation','Vertical','FontSize',16);
 plotopt
 theta2 = ['th2_' testcase '_Kp1_' num2str(Kp1) '_Kd1_' num2str(Kd1) '_Ki1_' num2str(Ki2) '_Kp2_' num2str(Kp2) '_Kd2_' num2str(Kd2) '_Ki2_' num2str(Ki2) ];
 print(gcf,'-dpng',theta2,'-r400')

 
%theta1 vs theta2 plot
figure
plot(thdtest1,thdtest2,'--')
hold on 
plot(th1,th2)
legend('Reference Theta','Controlled Theta')
grid
ylabel('$\theta_1$ [rad]','Interpreter','Latex')
 xlabel('$\theta_2$ [rad]','Interpreter','Latex')
 set(get(gca,'Children'), 'LineWidth', 1)
 hl = legend('Reference $\theta$','Controlled $\theta$');
 set(hl,'Interpreter','Latex','Orientation','Vertical','FontSize',16);
 plotopt
 theta12tot = ['th12tot_' testcase '_Kp1_' num2str(Kp1) '_Kd1_' num2str(Kd1) '_Ki1_' num2str(Ki2) '_Kp2_' num2str(Kp2) '_Kd2_' num2str(Kd2) '_Ki2_' num2str(Ki2) ];
 print(gcf,'-dpng',theta12tot,'-r400')
 

 
 
 
 









 
 
 
 
 


 