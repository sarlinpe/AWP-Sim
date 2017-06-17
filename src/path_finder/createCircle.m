% Title:        AWP Trajectory Planning and Control
% File:         createCircle.m
% Date:         2017-04-20
% Authors:      Nicolai Domingo Nielsen
%               Paul-Edouard Sarlin
% Description:  Initialise a simple circle structure.

function c = createCircle( center, radius )

c.center = center;
c.radius = radius;
c.type = 'circle';

end
