% Title:        AWP Trajectory Planning and Control
% File:         cspaceCollision.m
% Date:         2017-04-20
% Authors:      Nicolai Domingo Nielsen
%               Paul-Edouard Sarlin
% Description:  Create the binary grid of the collision C-space.

function [collision] = cspaceCollision(robot, map, obstacles, inflate_rad)

s = map.GridSize; nrows = s(1); ncols = s(2);

[ROWS, COLS] = meshgrid(1:nrows,1:ncols);
th1th2 = grid2world(map, [ROWS(:) COLS(:)]);
[xy2, xy1] = forwardKin(robot,th1th2);

% Ground and obstacles
collision = groundCollision(robot, th1th2);
for obs = obstacles
    if obs.type == 'circle'
        collision = collision | obsCollision(obs, xy1, xy2);
    end
end
% Add boundaries
collision = collision | (COLS(:) == ncols) | (COLS(:) == 1);
collision = collision | (ROWS(:) == nrows) | (ROWS(:) == 1);

setOccupancy(map,[ROWS(:),COLS(:)],collision,'grid');
inflate(map,inflate_rad);
collision = getOccupancy(map,[ROWS(:),COLS(:)],'grid');

collision = reshape(collision,ncols,nrows)';

end

