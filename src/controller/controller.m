% Title:        AWP Trajectory Planning and Control
% File:         controller.m
% Date:         2017-04-20
% Authors:      Nicolai Domingo Nielsen
%               Paul-Edouard Sarlin
% Description:  A wrapper around the ODE solver for the simulation.

function [T, th, F] = controller(robot, PID, lspb_cfg, th_init, T_tot)

x_init = [0 0 th_init 0 0 0 0]; % initial states
T_span = [0, 1.05*T_tot];

[T,X] = ode45(@(t,x) dynModel(t,x,robot,PID,lspb_cfg),T_span,x_init);

% Joint positions
th1 = X(:,3); % theta1
th2 = X(:,4); % theta2
th = [th1, th2];

% Torque inputs
F1 = diff(X(:,7))./diff(T);
F2 = diff(X(:,8))./diff(T);
F = [F1, F2];

end

