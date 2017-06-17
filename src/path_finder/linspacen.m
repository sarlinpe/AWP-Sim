% Title:        AWP Trajectory Planning and Control
% File:         linspacen.m
% Date:         2017-04-20
% Authors:      Nicolai Domingo Nielsen
%               Paul-Edouard Sarlin
% Description:  Discretise the line between the sets of points (x1,x2).

function [lin] = linspacen(x1, x2, n)

if size(x1,2) > 1
    x1 = x1';
end
if size(x2,2) > 1
    x2 = x2';
end

dx = (x2-x1) / (n-1);
pts = [x1, repmat(dx,1,n-1)];
lin = cumsum(pts, 2);

end

