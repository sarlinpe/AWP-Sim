% Title:        ME5402 Project 1-2: Trajectory Planning
% File:         groundCollision.m
% Date:         2017-04-20
% Author:       Nicolai Domingo Nielsen (A0164015R)
%               Paul-Edouard Sarlin (A0153124U)
% Description:  Check for a collision between the links and the ground.

function [ collision ] = groundCollision( robot, th1th2 )

th1 = th1th2(:,1);
th2 = th1th2(:,2);

collision = (robot.l1*sin(th1) + robot.l2*sin(th1+th2)) < 0;

end

