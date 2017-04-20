% Title:        ME5402 Project 1-2: Trajectory Planning
% File:         plotCtrlProfiles.m
% Date:         2017-04-20
% Author:       Nicolai Domingo Nielsen (A0164015R)
%               Paul-Edouard Sarlin (A0153124U)
% Description:  Plot the performances of the controller.

function [f] = plotCtrlProfiles(t, th, F, th_ref)

f = figure('Name', 'Controller trajectory profiles');

subplot(2,2,1:2);
title('Position of the joints');
hold on; grid on; box on;
xlim([0, t(end)]);
h1 = plot(t,th_ref(:,1),'k-');
h2 = plot(t,th(:,1),'b','LineWidth',1);
h3 = plot(t,th_ref(:,2),'k-');
h4 = plot(t,th(:,2),'r','LineWidth',1);
xlabel('t [s]');
ylabel('\theta_1, \theta_2 [rad]');
l = legend([h1,h2,h4],'Ref.', '\theta_1', '\theta_2','Orientation','Horizontal','Location','South');
set(l,'FontSize',9);

e = th_ref - th;
subplot(2,2,3);
title('Controller errors');
hold on; grid on; box on;
xlim([0, t(end)]);
plot(t,e(:,1),'b','LineWidth',1);
plot(t,e(:,2),'r','LineWidth',1);
xlabel('t [s]');
ylabel('e_1, e_2 [rad]');

subplot(2,2,4);
title('Torques');
hold on; grid on; box on;
xlim([0, t(end)]);
plot(t,F(:,1),'b','LineWidth',1);
plot(t,F(:,2),'r','LineWidth',1);
xlabel('t [s]');
ylabel('\tau_1, \tau_2 [N.m]');



end

