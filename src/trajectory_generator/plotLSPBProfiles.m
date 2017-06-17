% Title:        AWP Trajectory Planning and Control
% File:         plotLSPBProfiles.m
% Date:         2017-04-20
% Authors:      Nicolai Domingo Nielsen
%               Paul-Edouard Sarlin
% Description:  Plot the resulting profiles of the LSPB generation.

function [ f ] = plotLSPBProfiles(robot, lspb, t, q, v, a)

f = figure('Name', 'LSPB trajectory profiles');

subplot(2,2,1);
title('Positions of joints');
hold on; grid on; box on;
xlim([0, t(end)]);
h1 = plot([0 lspb.T_stamps],lspb.p(:,1),'k-');
h2 = plot(t,q(:,1),'b','LineWidth',1);
h3 = plot([0 lspb.T_stamps],lspb.p(:,2),'k-');
h4 = plot(t,q(:,2),'r','LineWidth',1);
l = legend([h1,h2,h4],'Linear', '\theta_1', '\theta_2', ...
    'Orientation','Horizontal','Location','South');
set(l,'FontSize',6);

xlabel('t [s]');
ylabel('\theta_1, \theta_2 [rad]');

subplot(2,2,2);
title('Velocities of joints');
hold on; grid on; box on;
xlim([0, t(end)]);
plot(t,v(:,1),'b','LineWidth',1);
plot(t,v(:,2),'r','LineWidth',1);
xlabel('t [s]');
ylabel('v_1, v_2 [rad/s]');

subplot(2,2,3);
title('Accelerations of joints');
hold on; grid on; box on;
xlim([0, t(end)]);
plot(t,a(:,1),'b','LineWidth',1);
plot(t,a(:,2),'r','LineWidth',1);
xlabel('t [s]');
ylabel('a_1, a_2 [rad/s^2]');

l1 = robot.l1;
l2 = robot.l2;
th1 = q(:,1);
th2 = q(:,2);
J = [-l1*sin(th1)-l2*sin(th1+th2), -l2*sin(th1+th2); ...
     l1*cos(th1)+l2*cos(th1+th2), l2*cos(th1+th2)];
C = J*[v',v'];
d = diag(C);
vxy = reshape(d,length(t),2);

subplot(2,2,4);
title('Velocities of end effector');
hold on; grid on; box on;
xlim([0, t(end)]);
plot(t,vxy(:,1),'b','LineWidth',1);
plot(t,vxy(:,2),'r','LineWidth',1);
plot(t,sqrt(sum(vxy.^2,2)),'k','LineWidth',1);
xlabel('t [s]');
ylabel('v_x, v_y, v [m/s]');
l = legend('v_x', 'v_y', 'total', ...
    'Orientation','Horizontal','Location','South');
set(l,'FontSize',6);

end

