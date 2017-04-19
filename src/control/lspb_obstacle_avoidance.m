%clear; 
close all;
addpath('./trajectory_generator');

% Initial constraints
%pasdasdasd = [0,0;1,2;3,3;5,2.5;5,4;4,5];
 p=[1.0472   -0.7854
    0.1860    1.3265
    0.2929    1.4546
    0.8031    1.7297
    1.1965    1.8769
    1.6204    1.5266
    2.0487    0.8050
    3.0027   -1.3772];

v_max = [0.1,0.1];
a_max = [0.005,0.005];

% Compute trajectory parameters
[lspb, T_tot] = lspb_init(p,v_max,a_max);

% Get 100 time points along the trajectory
ttt = linspace(0,T_tot);
[q,v,a] = lspb_get(ttt, lspb);

% Find first ODE45 timesteps

% Plot
% figure('Name', 'Position of joint 1')
% plot(ttt,q(:,1),'LineWidth',2)
% line([0 lspb.T_stamps],lspb.p(:,1),'Color','r','LineStyle','--')
% 
% figure('Name', 'Velocity of joint 1')
% plot(ttt,v(:,1),'LineWidth',2)
% 
% figure('Name', 'Acceleration of joint 1')
% plot(ttt,a(:,1),'LineWidth',2)
% 
% figure('Name', 'Total velocity')
% plot(ttt,sqrt(sum(v.^2,2)),'LineWidth',2)
% 
% figure('Name', 'Trajectory in the config space')
% plot(q(:,1),q(:,2),'LineWidth',2)
% line(lspb.p(:,1),lspb.p(:,2),'Color','r','LineStyle','--', 'LineWidth',1)
% xlim([0 6]); ylim([0 6]);
% 
% figure('Name', 'Animation')
% r = robotics.Rate(10);
% for i = 1:length(ttt)
%     clf('reset')
%     plot(q(1:i,1),q(1:i,2),'k--','LineWidth',1)
%     hold on
%     plot(q(i,1),q(i,2),'r*','LineWidth',2)
%     xlim([0 6]); ylim([0 6]);
%     drawnow
%     waitfor(r);
% end
