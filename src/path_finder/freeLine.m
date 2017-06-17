% Title:        AWP Trajectory Planning and Control
% File:         freeLine.m
% Date:         2017-04-20
% Authors:      Nicolai Domingo Nielsen
%               Paul-Edouard Sarlin
% Description:  Check if the given line(s) lie(s) in the free space.

function [ free ] = freeLine( map, xy1, xy2 )

n = 25;
p = size(xy1,1);
free = zeros(p,1);

for i = 1:p
    segx = linspace(xy1(i,1), xy2(i,1),n);
    segy = linspace(xy1(i,2), xy2(i,2),n);
    free(i) = ~any(getOccupancy(map,[segx', segy']));
end

end

