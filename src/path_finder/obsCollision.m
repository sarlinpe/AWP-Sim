% Title:        ME5402 Project 1-2: Trajectory Planning
% File:         obsCollision.m
% Date:         2017-04-20
% Author:       Nicolai Domingo Nielsen (A0164015R)
%               Paul-Edouard Sarlin (A0153124U)
% Description:  Check for a collision between the links and all the
%               obstacles, for multiple configurations at the same time.

function [ collision ] = obsCollision( obs, xy1, xy2 )

n = 20;
p = size(xy1, 1);
xy0 = zeros(p,2);

seg1x = linspacen(xy0(:,1),xy1(:,1),n);
seg1y = linspacen(xy0(:,2),xy1(:,2),n);
seg2x = linspacen(xy1(:,1),xy2(:,1),n);
seg2y = linspacen(xy1(:,2),xy2(:,2),n);

segx = [seg1x, seg2x];
segy = [seg1y, seg2y];

dist2 = (segx - obs.center(1)).^2 + (segy - obs.center(2)).^2;
collision = any(dist2 < obs.radius^2, 2);

end