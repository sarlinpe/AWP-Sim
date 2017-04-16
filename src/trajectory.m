clear; close all;
addpath('./path_finder');

%% Parameters

% Robot
robot.l1 = 2;
robot.l2 = 1;
th_init = [pi/3, -pi/4];
v_max = [1,1];
a_max = [0.3,0.3];

% Environment
th1_lim = [0,pi];
th2_lim = [-pi, pi];
x_lim = (robot.l1+robot.l2)*[-1.1,1.1];
y_lim = (robot.l1+robot.l2)*[-0.1,1.1];

obstacles = [createCircle([0.7,2.1],0.2), ...
             createCircle([-2.2,1.2],0.2)];

% Path finding
resolution = 100;
inflate_rad = 0.05;
prm_nodes = 1500;
prm_dist = 0.5;

%% Create UI

f = figure();
colormap(f, gray);
cspace = subplot(1,2,1);
wspace = subplot(1,2,2);

axes(wspace);
xlim(x_lim);
xlabel('x')
ylim(y_lim);
ylabel('y')
box on; grid on; hold on;
daspect([1,1,1]);

axes(cspace);
xlim(th1_lim);
xlabel('\theta_1')
xticks(0:pi/4:pi)
xticklabels({'0','\pi/4','\pi/2','3\pi/4','\pi'})
ylim(th2_lim);
ylabel('\theta_2')
yticks([-pi -pi/2 0 pi/2 pi])
yticklabels({'-\pi', '-\pi/2', 0','\pi/2','\pi'})
box on; grid on; hold on;

drawRobot(wspace, robot, th_init);
plot(cspace, th_init(1), th_init(2), 'ro');

for obs = obstacles
    drawCircle(wspace, obs, 'k', true);
end

%% Create occupancy grid with obstacles

map = robotics.BinaryOccupancyGrid(diff(th1_lim),diff(th2_lim),resolution);
map.GridLocationInWorld = [th1_lim(1),th2_lim(1)];
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

%% Display collision-free positions

% In c-space
bg = reshape(collision,ncols,nrows)';
im = imagesc(cspace, th1_lim,[th2_lim(2),th2_lim(1)],~bg);
uistack(im,'bottom');

% In workspace
ws = robotics.BinaryOccupancyGrid(diff(x_lim),diff(y_lim),int16(resolution));
ws.GridLocationInWorld = [x_lim(1),y_lim(1)];
grid_ws = zeros(ws.GridSize(1),ws.GridSize(2));

for i = 1:ws.GridSize(1) % rows
    for j = 1:ws.GridSize(2) % columns
        xy = grid2world(ws, [i,j]);
        th = inverseKin(robot, xy);
        if all(isnan(th(:)))
            setOccupancy(ws, [i,j], 1, 'grid');
            grid_ws(i,j) = 1;
            continue;
        end
        occ = 1;
        for k = 1:size(th,1)
            if th(k,1) < th1_lim(1) || th(k,1) > th1_lim(2) || ... 
                th(k,2) < th2_lim(1) || th(k,2) > th2_lim(2) || ...
                getOccupancy(map, th(k,:))
                ;
            else
                occ = 0;
            end  
        end
        setOccupancy(ws, [i,j], occ, 'grid');
        grid_ws(i,j) = occ;
    end
end

colormap(wspace, [.7,.7,.7;1,1,1]);
im = imagesc(wspace, x_lim,[y_lim(2),y_lim(1)],~grid_ws);
im.AlphaData = 0.5;
uistack(im,'bottom');

% % Display collision-free effector positions
% xy2_free = xy2(~collision,:);
% h = plot(wspace,xy2_free(:,1),xy2_free(:,2),'*','Color',[.8,.8,.8],'Marker','o','Markersize',1);
% %k = boundary(xy2_free,.8);
% %plot(wspace,xy2_free(k,1),xy2_free(k,2),'*','Color','k','Marker','o','Markersize',1);
% %h = fill(wspace, xy2_free(k,1),xy2_free(k,2),'g');
% %set(h,'facealpha',.2)
% uistack(h,'bottom');


%% Find shortest path

prm = robotics.PRM(map,prm_nodes);
prm.ConnectionDistance = prm_dist;

% Get target point
axes(wspace);
[x,y] = ginput(1);
xy_end = [x,y];

%th_end = xy_end;
th_candidates = inverseKin(robot,xy_end);
th_start = th_init;
if all(isnan(th_candidates(:)))
    error('Target point is out of workspace.');
end
if all(th_candidates(1,:) == th_candidates(2,:))
    th_candidates = th_candidates(1,:);
end

plot(cspace, th_candidates(:,1), th_candidates(:,2), 'go');
drawnow

th_accepted = [];
paths = {};
lengths = [];

for i = 1:length(th_candidates)
    th = th_candidates(i,:);
    if th(1) < th1_lim(1) || th(1) > th1_lim(2) || ... 
            th(2) < th2_lim(1) || th(2) > th2_lim(2)
        continue;
    end
        
    try
        path = findpath(prm,th_start,th);
    catch ME
        switch ME.identifier
            case 'robotics:robotalgs:prm:OccupiedLocation'
                continue
            otherwise
                rethrow(ME);
        end
    end
    if size(path,1) ~= 0 % a path is found!
        th_accepted = [th_accepted; th];
        paths = [paths, {path}];
        lengths = [lengths, sum(sqrt(sum(diff(path,1).^2,2)),1)];
    end
end

if size(th_accepted,1) == 0
    error('No valid configuration found.');
end
[~, idx] = min(lengths);
th_end = th_accepted(idx,:);
path = paths{idx};

drawRobot(wspace, robot, th_end, 'b-');
plot(cspace, th_end(1), th_end(2), 'bo');
plot(cspace, path(:,1),path(:,2),'r*-');
drawnow

% figure();
% show(prm);

%% Perform pruning
pruned = [path(1,:)];
i = 1;
while i < size(path,1)

    j = i+1;
    while true
        if (j <= size(path,1)) && freeLine(map,path(i,:),path(j,:))
            j = j + 1;
        else
            i = j - 1;
            pruned = [pruned; path(i,:)];
            break;
        end
    end
end
path = pruned;
plot(cspace, pruned(:,1),pruned(:,2),'b*-');

%% Generate reference inputs

[lspb, T_tot] = lspb_init(path,v_max,a_max);
t = linspace(0,T_tot);
[q,v,a] = lspb_get(t, lspb);

plot(cspace, q(:,1),q(:,2),'b');

%% Plot trajectory

t = linspace(0, T_tot);
[q,v,a] = lspb_get(t, lspb);

r = robotics.Rate(10);
for i = 2:length(t)-1
    hold on;
    h = drawRobot(wspace, robot, q(i,:), 'k');
    drawnow
    waitfor(r);
    delete(h);
end